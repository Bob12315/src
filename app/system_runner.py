from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque

from app.app_config import AppConfig
from app.debug_runtime import DebugRuntime
from app.health_monitor import HealthMonitor, HealthMonitorConfig
from app.mission_manager import MissionManager, MissionManagerConfig, MissionMode
from app.mode_registry import ModeRegistry
from app.service_manager import ServiceManager
from flight_modes.common import (
    CommandShaper,
    FlightCommand,
    FlightCommandExecutor,
    FlightModeInputAdapter,
)
from uav_ui.control_switches import ControlRuntimeSwitches
from uav_ui.terminal_ui import run_terminal_ui
from uav_ui.ui_commands import CommandResult, build_ui_command_handler, format_controller_snapshot
from uav_ui.yolo_command_client import YoloCommandClient


class SystemRunner:
    def __init__(self, config: AppConfig, stop_event: threading.Event | None = None) -> None:
        self.config = config
        self.stop_event = stop_event or threading.Event()
        self.logger = logging.getLogger(self.__class__.__name__)
        self.services = ServiceManager(config, self.stop_event)
        self.input_adapter = FlightModeInputAdapter(config=config.input_adapter)
        self.health_monitor = HealthMonitor(
            HealthMonitorConfig(
                max_vision_age_s=config.control.mode.max_vision_age_s,
                max_drone_age_s=config.control.mode.max_drone_age_s,
                max_gimbal_age_s=config.control.mode.max_gimbal_age_s,
            )
        )
        self.mission_manager = MissionManager(
            MissionManagerConfig(
                overhead_entry_target_size_thresh=(
                    config.control.mode.overhead_entry_target_size_thresh
                ),
                overhead_entry_pitch_rad=config.control.mode.overhead_entry_pitch_rad,
                overhead_entry_pitch_tol_rad=config.control.mode.overhead_entry_pitch_tol_rad,
                overhead_entry_yaw_tol_rad=config.control.mode.overhead_entry_yaw_tol_rad,
                overhead_entry_hold_s=config.control.mode.overhead_entry_hold_s,
                overhead_exit_target_size_drop=config.control.mode.overhead_exit_target_size_drop,
            )
        )
        self.mode_registry = ModeRegistry(
            approach_config=config.approach_track,
            overhead_config=config.overhead_hold,
        )
        self.command_shaper = CommandShaper(config=config.shaper)
        self.executor = FlightCommandExecutor(config=config.executor)
        self.debug_runtime = DebugRuntime(config.debug)
        self.controller_switches = ControlRuntimeSwitches(
            gimbal=config.start_gimbal,
            body=config.start_body,
            approach=config.start_approach,
            send_commands=config.start_send_commands,
        )
        self.control_command_log: deque[str] = deque(maxlen=120)
        self.control_command_log_lock = threading.Lock()
        self.latest_task_mode = "UNKNOWN"
        self.last_send_commands: bool | None = None
        self.target_lost_since: float | None = None
        self.lost_target_recenter_sent = False

    def run(self) -> None:
        self.services.start()
        self.executor.set_telemetry_link(self.services.link_manager)
        try:
            if self.config.runtime.ui_enabled and self.services.link_manager is not None:
                self._run_with_ui()
            else:
                if self.config.runtime.ui_enabled and self.services.link_manager is None:
                    self.logger.warning("UI disabled because telemetry is not connected")
                self._control_loop()
        finally:
            self.stop()

    def stop(self) -> None:
        self.stop_event.set()
        self.executor.reset()
        self.services.stop()
        self.logger.info("app runtime stopped")

    def _run_with_ui(self) -> None:
        assert self.services.link_manager is not None
        ui_command_handler = build_ui_command_handler(
            self.services.link_manager,
            controller_switches=self.controller_switches,
            yolo_client=YoloCommandClient(self.config.yolo_command),
            task_mode_handler=self._set_task_mode_override,
        )
        control_thread = threading.Thread(
            name="AppControlLoop",
            target=self._control_loop,
            daemon=True,
        )
        control_thread.start()
        try:
            run_terminal_ui(
                self.services.link_manager,
                self.stop_event,
                self._get_control_command_lines,
                ui_command_handler,
            )
        finally:
            self.stop_event.set()
            control_thread.join(timeout=1.0)

    def _control_loop(self) -> None:
        loop_sleep_sec = 1.0 / max(self.config.runtime.loop_hz, 0.1)
        print_sleep_sec = 1.0 / max(self.config.runtime.print_rate_hz, 0.1)
        started_at = time.time()
        last_print_time = 0.0

        try:
            while not self.stop_event.is_set():
                now = time.time()
                run_seconds = self.config.runtime.run_seconds
                if run_seconds is not None and (now - started_at) >= run_seconds:
                    self.stop_event.set()
                    break

                perception = self.services.get_perception(now)
                drone = self.services.get_drone_state()
                gimbal = self.services.get_gimbal_state()
                fused = self.services.fusion_manager.update(perception, drone, gimbal)
                inputs = self.input_adapter.adapt(fused)
                health = self.health_monitor.update(inputs)
                mission = self.mission_manager.update(inputs, health)
                mission = self.debug_runtime.apply_mission_override(mission)
                raw_command, mode_status = self._update_active_mode(mission.active_mode, inputs)
                raw_command = self.debug_runtime.apply_command_override(raw_command)
                raw_command = self._apply_controller_switches(raw_command)
                raw_for_log = FlightCommand(
                    vx_cmd=raw_command.vx_cmd,
                    vy_cmd=raw_command.vy_cmd,
                    vz_cmd=raw_command.vz_cmd,
                    yaw_rate_cmd=raw_command.yaw_rate_cmd,
                    gimbal_yaw_rate_cmd=raw_command.gimbal_yaw_rate_cmd,
                    gimbal_pitch_rate_cmd=raw_command.gimbal_pitch_rate_cmd,
                    enable_body=raw_command.enable_body,
                    enable_gimbal=raw_command.enable_gimbal,
                    enable_approach=raw_command.enable_approach,
                    active=raw_command.active,
                    valid=raw_command.valid,
                )
                shaped = self.command_shaper.update(raw_command, inputs.dt)

                controller_enabled = self.controller_switches.snapshot()
                self.executor.config.send_commands = bool(controller_enabled.send_commands)
                if controller_enabled.send_commands:
                    self.executor.execute(shaped)
                    self._record_control_command(now, shaped, send_commands=True)
                    self._maybe_recenter_gimbal_after_target_loss(now, bool(inputs.target_valid), True)
                else:
                    if self.last_send_commands is not False:
                        with self.control_command_log_lock:
                            self.control_command_log.clear()
                            self.control_command_log.appendleft(
                                f"{time.strftime('%H:%M:%S', time.localtime(now))} "
                                "DRY continuous command sending disabled"
                            )
                    self._maybe_recenter_gimbal_after_target_loss(now, bool(inputs.target_valid), False)
                self.last_send_commands = bool(controller_enabled.send_commands)

                with self.control_command_log_lock:
                    self.latest_task_mode = mission.active_mode

                if (now - last_print_time) >= print_sleep_sec:
                    self.logger.info(
                        "mode=%s mission=%s health=%s hold=%s enabled=(gimbal:%s body:%s approach:%s send:%s) "
                        "control_allowed=%s target_valid=%s track_id=%s target_size=%.3f "
                        "raw=(vx=%.3f vy=%.3f vz=%.3f yaw=%.3f gimbal=%.3f,%.3f) "
                        "shaped=(vx=%.3f vy=%.3f vz=%.3f yaw=%.3f gimbal=%.3f,%.3f)",
                        mode_status.mode_name,
                        mission.active_mode,
                        health.hold_reason,
                        mode_status.hold_reason or mission.hold_reason,
                        shaped.enable_gimbal,
                        shaped.enable_body,
                        shaped.enable_approach,
                        controller_enabled.send_commands,
                        inputs.control_allowed,
                        inputs.target_valid,
                        inputs.track_id,
                        inputs.target_size,
                        raw_for_log.vx_cmd,
                        raw_for_log.vy_cmd,
                        raw_for_log.vz_cmd,
                        raw_for_log.yaw_rate_cmd,
                        raw_for_log.gimbal_yaw_rate_cmd,
                        raw_for_log.gimbal_pitch_rate_cmd,
                        shaped.vx_cmd,
                        shaped.vy_cmd,
                        shaped.vz_cmd,
                        shaped.yaw_rate_cmd,
                        shaped.gimbal_yaw_rate_cmd,
                        shaped.gimbal_pitch_rate_cmd,
                    )
                    last_print_time = now

                time.sleep(loop_sleep_sec)
        except Exception:
            self.logger.exception("app control loop failed")
            self.stop_event.set()

    def _update_active_mode(self, mode_name: str, inputs) -> tuple[FlightCommand, object]:
        if mode_name == MissionMode.IDLE.value:
            return FlightCommand(valid=True), _Status("IDLE", False, True, "idle")
        try:
            mode = self.mode_registry.get(mode_name)
        except KeyError:
            self.logger.warning("unknown mission mode %s; commanding zero", mode_name)
            return FlightCommand(valid=True), _Status(mode_name, False, False, "unknown_mode")
        return mode.update(inputs)

    def _apply_controller_switches(self, command: FlightCommand) -> FlightCommand:
        snapshot = self.controller_switches.snapshot()
        if not snapshot.gimbal:
            command.enable_gimbal = False
            command.gimbal_yaw_rate_cmd = 0.0
            command.gimbal_pitch_rate_cmd = 0.0
        if not snapshot.body:
            command.enable_body = False
            command.vy_cmd = 0.0
            command.vz_cmd = 0.0
            command.yaw_rate_cmd = 0.0
        if not snapshot.approach:
            command.enable_approach = False
            command.vx_cmd = 0.0
        command.active = bool(command.enable_gimbal or command.enable_body or command.enable_approach)
        return command

    def _format_control_command(self, now: float, shaped: FlightCommand, send_commands: bool) -> str:
        return (
            f"{time.strftime('%H:%M:%S', time.localtime(now))} "
            f"{'TX' if send_commands else 'DRY'} "
            f"vx={shaped.vx_cmd:.3f} vy={shaped.vy_cmd:.3f} vz={shaped.vz_cmd:.3f} "
            f"yaw={shaped.yaw_rate_cmd:.3f} "
            f"gimbal=({shaped.gimbal_yaw_rate_cmd:.3f},{shaped.gimbal_pitch_rate_cmd:.3f}) "
            f"en=G{int(shaped.enable_gimbal)} B{int(shaped.enable_body)} A{int(shaped.enable_approach)} "
            f"active={shaped.active} valid={shaped.valid}"
        )

    def _record_control_command(self, now: float, shaped: FlightCommand, send_commands: bool) -> None:
        line = self._format_control_command(now, shaped, send_commands)
        with self.control_command_log_lock:
            self.control_command_log.appendleft(line)

    def _get_control_command_lines(self) -> list[str]:
        with self.control_command_log_lock:
            return [
                f"Controllers {format_controller_snapshot(self.controller_switches.snapshot())}",
                f"Task mode {self.latest_task_mode}",
                *list(self.control_command_log),
            ]

    def _set_task_mode_override(self, mode_name: str | None) -> CommandResult:
        if mode_name is None:
            self.debug_runtime.config.force_mode = None
            self.mode_registry.reset_all()
            return CommandResult(True, "task mode auto")
        normalized = mode_name.strip().upper()
        if normalized == MissionMode.IDLE.value:
            self.debug_runtime.config.force_mode = normalized
            return CommandResult(True, "task mode forced IDLE")
        try:
            self.mode_registry.get(normalized)
        except KeyError:
            return CommandResult(
                False,
                "task mode must be APPROACH_TRACK, OVERHEAD_HOLD, CORRIDOR_FOLLOW, IDLE, or auto",
            )
        self.debug_runtime.config.force_mode = normalized
        return CommandResult(True, f"task mode forced {normalized}")

    def _maybe_recenter_gimbal_after_target_loss(
        self,
        now: float,
        target_valid: bool,
        send_commands: bool,
    ) -> None:
        if target_valid:
            self.target_lost_since = None
            self.lost_target_recenter_sent = False
            return
        if self.target_lost_since is None:
            self.target_lost_since = now
            return
        if self.lost_target_recenter_sent or not send_commands:
            return
        if self.services.link_manager is None:
            return
        if not self.config.runtime.lost_target_recenter_enabled:
            return
        if (now - self.target_lost_since) < self.config.runtime.lost_target_recenter_timeout_sec:
            return
        clear_sender = getattr(self.services.link_manager, "clear_continuous_commands", None)
        if callable(clear_sender):
            clear_sender()
        self.services.link_manager.send_gimbal_angle(
            pitch=self.config.runtime.lost_target_recenter_pitch_deg,
            yaw=self.config.runtime.lost_target_recenter_yaw_deg,
            roll=self.config.executor.gimbal_roll_deg,
        )
        self.lost_target_recenter_sent = True
        with self.control_command_log_lock:
            self.control_command_log.appendleft(
                f"{time.strftime('%H:%M:%S', time.localtime(now))} "
                "TX lost target recenter gimbal "
                f"pitch={self.config.runtime.lost_target_recenter_pitch_deg:.1f} "
                f"yaw={self.config.runtime.lost_target_recenter_yaw_deg:.1f}"
            )


class _Status:
    def __init__(self, mode_name: str, active: bool, valid: bool, hold_reason: str) -> None:
        self.mode_name = mode_name
        self.active = active
        self.valid = valid
        self.hold_reason = hold_reason
        self.detail = {}
