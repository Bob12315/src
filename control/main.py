from __future__ import annotations

import argparse
import json
import logging
import signal
import socket
import sys
import threading
import time
from collections import deque
from dataclasses import replace
from pathlib import Path
from typing import Any

import yaml

ROOT_DIR = Path(__file__).resolve().parent.parent
TELEMETRY_DIR = ROOT_DIR / "telemetry_link"
for path in (str(ROOT_DIR), str(TELEMETRY_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

from control.approach_controller import ApproachController
from control.body_controller import BodyController
from control.command_shaper import CommandShaper
from control.config import load_config as load_control_config
from control.control_executor import ControlExecutor
from control.control_input import ControlInputAdapter
from control.control_mode import ControlModeManager
from control.gimbal_controller import GimbalController
from fusion.fusion_manager import FusionManager
from fusion.models import FusionConfig, PerceptionTarget
from link_manager import LinkManager
from telemetry_link.config import EndpointConfig, TelemetryConfig
from uav_ui.control_switches import ControlRuntimeSwitches
from uav_ui.terminal_ui import run_terminal_ui
from uav_ui.ui_commands import build_ui_command_handler, format_controller_snapshot
from uav_ui.yolo_command_client import YoloCommandClient, YoloCommandConfig


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Live control service")
    parser.add_argument(
        "--control-config",
        default=str(ROOT_DIR / "control" / "config.yaml"),
        help="Path to control config.yaml",
    )
    parser.add_argument(
        "--telemetry-config",
        default=str(ROOT_DIR / "telemetry_link" / "config.yaml"),
        help="Path to telemetry_link config.yaml",
    )
    parser.add_argument(
        "--yolo-config",
        default=str(ROOT_DIR / "yolo_app" / "config.yaml"),
        help="Path to yolo_app config.yaml for UI target commands",
    )
    parser.add_argument("--yolo-udp-ip")
    parser.add_argument("--yolo-udp-port", type=int)
    parser.add_argument("--loop-hz", type=float)
    parser.add_argument("--perception-timeout-sec", type=float)
    parser.add_argument("--print-rate-hz", type=float)
    parser.add_argument("--require-gimbal-feedback", type=lambda value: str(value).lower() in {"1", "true", "yes", "y", "on"})
    parser.add_argument("--log-level")
    return parser


def setup_logging(level: str, log_file: str | None = None) -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        filename=log_file,
    )


def load_telemetry_config(path: str) -> TelemetryConfig:
    with open(path, "r", encoding="utf-8") as handle:
        merged = yaml.safe_load(handle) or {}
    if not isinstance(merged, dict):
        raise ValueError("telemetry config yaml must be a mapping")

    def build_endpoint(name: str, data: dict[str, Any]) -> EndpointConfig:
        return EndpointConfig(
            name=name,
            connection_type=str(data["connection_type"]),
            serial_port=str(data.get("serial_port", "/dev/ttyUSB0")),
            baudrate=int(data.get("baudrate", 115200)),
            udp_mode=str(data.get("udp_mode", "udpin")),
            udp_host=str(data.get("udp_host", "0.0.0.0")),
            udp_port=int(data.get("udp_port", 14550)),
            tcp_host=str(data.get("tcp_host", "127.0.0.1")),
            tcp_port=int(data.get("tcp_port", 5760)),
        )

    return TelemetryConfig(
        data_source=str(merged["data_source"]),
        active_source=str(merged["active_source"]),
        sitl=build_endpoint("sitl", dict(merged["sitl"])),
        real=build_endpoint("real", dict(merged["real"])),
        control_send_rate_hz=float(merged["control_send_rate_hz"]),
        action_cmd_retries=int(merged["action_cmd_retries"]),
        action_retry_interval_sec=float(merged["action_retry_interval_sec"]),
        heartbeat_timeout_sec=float(merged["heartbeat_timeout_sec"]),
        rx_timeout_sec=float(merged["rx_timeout_sec"]),
        reconnect_interval_sec=float(merged["reconnect_interval_sec"]),
        receiver_idle_sleep_sec=float(merged["receiver_idle_sleep_sec"]),
        sender_idle_sleep_sec=float(merged["sender_idle_sleep_sec"]),
        request_message_intervals=bool(merged["request_message_intervals"]),
        message_interval_hz={str(k): float(v) for k, v in dict(merged.get("message_interval_hz", {})).items()},
        gimbal_mount_mode=int(merged.get("gimbal_mount_mode", 2)),
        gimbal_yaw_min_deg=float(merged.get("gimbal_yaw_min_deg", -180.0)),
        gimbal_yaw_max_deg=float(merged.get("gimbal_yaw_max_deg", 180.0)),
        gimbal_pitch_min_deg=float(merged.get("gimbal_pitch_min_deg", -180.0)),
        gimbal_pitch_max_deg=float(merged.get("gimbal_pitch_max_deg", 180.0)),
        state_udp_enabled=bool(merged.get("state_udp_enabled", True)),
        state_udp_ip=str(merged.get("state_udp_ip", "127.0.0.1")),
        state_udp_port=int(merged.get("state_udp_port", 5010)),
        ui_enabled=bool(merged.get("ui_enabled", False)),
        log_level=str(merged.get("log_level", "INFO")),
    )


def load_yolo_command_config(path: str) -> YoloCommandConfig:
    with open(path, "r", encoding="utf-8") as handle:
        merged = yaml.safe_load(handle) or {}
    if not isinstance(merged, dict):
        raise ValueError("yolo config yaml must be a mapping")
    command_ip = str(merged.get("command_ip", "127.0.0.1"))
    if command_ip in {"0.0.0.0", "::"}:
        command_ip = "127.0.0.1"
    return YoloCommandConfig(
        ip=command_ip,
        port=int(merged.get("command_port", 5006)),
        enabled=bool(merged.get("command_enabled", True)),
    )


class YoloUdpReceiver(threading.Thread):
    def __init__(self, ip: str, port: int, stop_event: threading.Event) -> None:
        super().__init__(name="ControlYoloUdpReceiver", daemon=True)
        self.stop_event = stop_event
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, port))
        self.sock.settimeout(0.2)
        self._lock = threading.Lock()
        self._latest_target = PerceptionTarget()
        self._last_packet_time = 0.0
        self.logger = logging.getLogger(self.__class__.__name__)

    def run(self) -> None:
        while not self.stop_event.is_set():
            try:
                payload, _addr = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                data = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                self.logger.warning("drop invalid YOLO UDP payload: %s", exc)
                continue
            if not isinstance(data, dict):
                self.logger.warning("drop YOLO UDP payload because it is not a JSON object")
                continue
            target = self._decode_target(data)
            with self._lock:
                self._latest_target = target
                self._last_packet_time = time.time()

    def get_latest_target(self, now: float, timeout_sec: float) -> PerceptionTarget:
        with self._lock:
            target = replace(self._latest_target)
            last_packet_time = self._last_packet_time
        if last_packet_time <= 0 or (now - last_packet_time) > timeout_sec:
            target.target_valid = False
            target.tracking_state = "lost"
            target.ex = 0.0
            target.ey = 0.0
        return target

    def close(self) -> None:
        self.sock.close()

    @staticmethod
    def _decode_target(data: dict[str, Any]) -> PerceptionTarget:
        return PerceptionTarget(
            timestamp=float(data.get("timestamp", 0.0)),
            frame_id=int(data.get("frame_id", 0)),
            target_valid=bool(data.get("target_valid", False)),
            tracking_state=str(data.get("tracking_state", "lost")),
            track_id=int(data.get("track_id", -1)),
            class_name=str(data.get("class_name", "")),
            confidence=float(data.get("confidence", 0.0)),
            cx=float(data.get("cx", 0.0)),
            cy=float(data.get("cy", 0.0)),
            w=float(data.get("w", 0.0)),
            h=float(data.get("h", 0.0)),
            image_width=float(data.get("image_width", 0.0)),
            image_height=float(data.get("image_height", 0.0)),
            target_size=float(data.get("target_size", 0.0)),
            ex=float(data.get("ex", 0.0)),
            ey=float(data.get("ey", 0.0)),
            lost_count=int(data.get("lost_count", 0)),
        )


def main() -> int:
    args = build_arg_parser().parse_args()
    control_cfg = load_control_config(["--config", args.control_config])
    telemetry_cfg = load_telemetry_config(args.telemetry_config)
    yolo_command_cfg = load_yolo_command_config(args.yolo_config)
    controller_switches = ControlRuntimeSwitches(
        gimbal=bool(control_cfg.runtime.enable_gimbal_controller),
        body=bool(control_cfg.runtime.enable_body_controller),
        approach=bool(control_cfg.runtime.enable_approach_controller),
        send_commands=bool(control_cfg.executor.send_commands),
    )

    yolo_udp_ip = args.yolo_udp_ip or control_cfg.runtime.yolo_udp_ip
    yolo_udp_port = (
        args.yolo_udp_port if args.yolo_udp_port is not None else control_cfg.runtime.yolo_udp_port
    )
    loop_hz = args.loop_hz if args.loop_hz is not None else control_cfg.runtime.loop_hz
    perception_timeout_sec = (
        args.perception_timeout_sec
        if args.perception_timeout_sec is not None
        else control_cfg.runtime.perception_timeout_sec
    )
    print_rate_hz = (
        args.print_rate_hz if args.print_rate_hz is not None else control_cfg.runtime.print_rate_hz
    )
    require_gimbal_feedback = (
        args.require_gimbal_feedback
        if args.require_gimbal_feedback is not None
        else control_cfg.runtime.require_gimbal_feedback
    )
    log_level = args.log_level or control_cfg.runtime.log_level

    ui_log_file = str(ROOT_DIR / "control" / "control_ui.log") if telemetry_cfg.ui_enabled else None
    setup_logging(log_level, ui_log_file)
    logger = logging.getLogger("control.main")
    stop_event = threading.Event()

    def _handle_signal(signum, _frame) -> None:
        logger.info("received signal %s, shutting down", signum)
        stop_event.set()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    yolo_receiver = YoloUdpReceiver(yolo_udp_ip, yolo_udp_port, stop_event)
    link_manager = LinkManager(telemetry_cfg)
    fusion_manager = FusionManager(
        FusionConfig(require_gimbal_feedback=bool(require_gimbal_feedback))
    )
    input_adapter = ControlInputAdapter(config=control_cfg.input_adapter)
    mode_manager = ControlModeManager(config=control_cfg.mode)
    gimbal_controller = GimbalController(config=control_cfg.gimbal)
    body_controller = BodyController(config=control_cfg.body)
    approach_controller = ApproachController(config=control_cfg.approach)
    command_shaper = CommandShaper(config=control_cfg.shaper)
    control_executor = ControlExecutor(
        telemetry_link=link_manager,
        config=control_cfg.executor,
    )

    logger.info(
        "starting control service yolo_udp=%s:%s loop_hz=%.2f telemetry_source=%s controllers=gimbal:%s body:%s approach:%s send_commands=%s",
        yolo_udp_ip,
        yolo_udp_port,
        loop_hz,
        telemetry_cfg.active_source,
        controller_switches.snapshot().gimbal,
        controller_switches.snapshot().body,
        controller_switches.snapshot().approach,
        control_cfg.executor.send_commands,
    )

    yolo_receiver.start()
    link_manager.start()
    loop_sleep_sec = 1.0 / max(loop_hz, 0.1)
    print_sleep_sec = 1.0 / max(print_rate_hz, 0.1)
    last_print_time = 0.0
    control_command_log: deque[str] = deque(maxlen=120)
    control_command_log_lock = threading.Lock()
    last_send_commands: bool | None = None

    def _format_control_command(now: float, shaped, send_commands: bool) -> str:
        return (
            f"{time.strftime('%H:%M:%S', time.localtime(now))} "
            f"{'TX' if send_commands else 'DRY'} "
            f"vx={shaped.vx_cmd:.3f} vy={shaped.vy_cmd:.3f} yaw={shaped.yaw_rate_cmd:.3f} "
            f"gimbal=({shaped.gimbal_yaw_rate_cmd:.3f},{shaped.gimbal_pitch_rate_cmd:.3f}) "
            f"en=G{int(shaped.enable_gimbal)} B{int(shaped.enable_body)} A{int(shaped.enable_approach)} "
            f"active={shaped.active} valid={shaped.valid}"
        )

    def _record_control_command(now: float, shaped, send_commands: bool) -> None:
        line = _format_control_command(now, shaped, send_commands)
        with control_command_log_lock:
            control_command_log.appendleft(line)

    def _get_control_command_lines() -> list[str]:
        with control_command_log_lock:
            return [
                f"Controllers {format_controller_snapshot(controller_switches.snapshot())}",
                *list(control_command_log),
            ]

    def _control_loop() -> None:
        nonlocal last_print_time, last_send_commands
        try:
            while not stop_event.is_set():
                now = time.time()
                perception = yolo_receiver.get_latest_target(now, perception_timeout_sec)
                drone = link_manager.get_latest_drone_state()
                gimbal = link_manager.get_latest_gimbal_state()
                fused = fusion_manager.update(perception, drone, gimbal)
                control_input = input_adapter.adapt(fused)
                raw_mode = mode_manager.update(control_input)
                controller_enabled = controller_switches.snapshot()
                control_executor.config.send_commands = bool(controller_enabled.send_commands)
                mode = replace(
                    raw_mode,
                    enable_gimbal=bool(raw_mode.enable_gimbal and controller_enabled.gimbal),
                    enable_body=bool(raw_mode.enable_body and controller_enabled.body),
                    enable_approach=bool(raw_mode.enable_approach and controller_enabled.approach),
                )
                gimbal_cmd = gimbal_controller.update(
                    control_input,
                    task_mode=mode.task_mode,
                    enabled=mode.enable_gimbal,
                )
                body_cmd = body_controller.update(
                    control_input,
                    task_mode=mode.task_mode,
                    enabled=mode.enable_body,
                )
                approach_cmd = approach_controller.update(
                    control_input,
                    task_mode=mode.task_mode,
                    enabled=mode.enable_approach,
                )
                shaped = command_shaper.update(
                    mode=mode,
                    gimbal_cmd=gimbal_cmd,
                    body_cmd=body_cmd,
                    approach_cmd=approach_cmd,
                    dt=control_input.dt,
                )
                if controller_enabled.send_commands:
                    control_executor.execute(shaped)
                    _record_control_command(now, shaped, send_commands=True)
                else:
                    if last_send_commands is not False:
                        clear_sender = getattr(link_manager, "clear_continuous_commands", None)
                        if callable(clear_sender):
                            clear_sender()
                        with control_command_log_lock:
                            control_command_log.clear()
                            control_command_log.appendleft(
                                f"{time.strftime('%H:%M:%S', time.localtime(now))} "
                                "DRY continuous command sending disabled"
                            )
                last_send_commands = bool(controller_enabled.send_commands)

                if (now - last_print_time) >= print_sleep_sec:
                    logger.info(
                        "mode=%s task_mode=%s hold=%s enabled=(gimbal:%s body:%s approach:%s) control_allowed=%s target_valid=%s track_id=%s "
                        "target_size=%.3f size_valid=%s approach_vx=%.3f shaped_vx=%.3f "
                        "ex=%.3f ey=%.3f gimbal_yaw=%.3f gimbal_pitch=%.3f "
                        "gimbal_rate=(%.3f,%.3f) shaped_rate=(%.3f,%.3f)",
                        mode.mode_name,
                        mode.task_mode,
                        mode.hold_reason,
                        mode.enable_gimbal,
                        mode.enable_body,
                        mode.enable_approach,
                        control_input.control_allowed,
                        control_input.target_valid,
                        control_input.track_id,
                        control_input.target_size,
                        control_input.target_size_valid,
                        approach_cmd.vx_cmd,
                        shaped.vx_cmd,
                        control_input.ex_cam,
                        control_input.ey_cam,
                        control_input.gimbal_yaw,
                        control_input.gimbal_pitch,
                        gimbal_cmd.yaw_rate_cmd,
                        gimbal_cmd.pitch_rate_cmd,
                        shaped.gimbal_yaw_rate_cmd,
                        shaped.gimbal_pitch_rate_cmd,
                    )
                    last_print_time = now

                time.sleep(loop_sleep_sec)
        except Exception:
            logger.exception("control loop failed")
            stop_event.set()

    control_thread: threading.Thread | None = None
    try:
        if telemetry_cfg.ui_enabled:
            ui_command_handler = build_ui_command_handler(
                link_manager,
                controller_switches=controller_switches,
                yolo_client=YoloCommandClient(yolo_command_cfg),
            )
            control_thread = threading.Thread(
                name="ControlLoop",
                target=_control_loop,
                daemon=True,
            )
            control_thread.start()
            run_terminal_ui(
                link_manager,
                stop_event,
                _get_control_command_lines,
                ui_command_handler,
            )
        else:
            _control_loop()
    finally:
        stop_event.set()
        if control_thread is not None:
            control_thread.join(timeout=1.0)
        control_executor.reset()
        yolo_receiver.close()
        link_manager.stop()
        logger.info("control service stopped")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
