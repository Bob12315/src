from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass

from command_queue import CommandQueue
from command_sender import CommandSender
from config import EndpointConfig, TelemetryConfig
from mavlink_client import MavlinkClient
from models import ActionCommand, ActionType, ControlCommand, ControlType, DroneState, GimbalState, LinkStatus
from state_cache import StateCache
from telemetry_receiver import TelemetryReceiver


@dataclass(slots=True)
class SourceRuntime:
    name: str
    endpoint: EndpointConfig
    cfg: TelemetryConfig
    state_cache: StateCache
    command_queue: CommandQueue
    client: MavlinkClient
    stop_event: threading.Event
    worker_stop_event: threading.Event
    receiver: TelemetryReceiver | None = None
    sender: CommandSender | None = None
    monitor_thread: threading.Thread | None = None
    worker_lock: threading.Lock | None = None

    def __post_init__(self) -> None:
        if self.worker_lock is None:
            self.worker_lock = threading.Lock()

    def start(self, logger: logging.Logger) -> None:
        self._connect_and_start_workers(logger)
        self.monitor_thread = threading.Thread(
            name=f"LinkMonitor-{self.name}",
            target=self._monitor_loop,
            args=(logger,),
            daemon=True,
        )
        self.monitor_thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        self._stop_workers(close_client=True)
        if self.monitor_thread is not None:
            self.monitor_thread.join(timeout=1.0)

    def _connect_and_start_workers(self, logger: logging.Logger) -> None:
        while not self.stop_event.is_set():
            try:
                self.state_cache.mark_reconnecting()
                logger.info("source=%s Attempting to reconnect...", self.name)
                self.client.connect()
                self.client.wait_heartbeat(timeout=max(5.0, self.cfg.heartbeat_timeout_sec + 2.0))
                logger.info(
                    "source=%s link ready connection_type=%s endpoint=%s sitl_mode=%s target_system=%s target_component=%s",
                    self.name,
                    self.endpoint.connection_type,
                    self.client.connection_string,
                    self.client.is_sitl,
                    self.client.target_system,
                    self.client.target_component,
                )
                now = time.time()
                self.state_cache.mark_connected(
                    target_system=self.client.target_system,
                    target_component=self.client.target_component,
                    transport=self.endpoint.connection_type,
                    now=now,
                )
                self.worker_stop_event = threading.Event()
                self.receiver = TelemetryReceiver(
                    self.client,
                    self.state_cache,
                    self.cfg,
                    self.worker_stop_event,
                )
                self.sender = CommandSender(
                    self.client,
                    self.command_queue,
                    self.state_cache,
                    self.cfg,
                    self.worker_stop_event,
                )
                self.receiver.start()
                self.sender.start()
                if self.cfg.request_message_intervals:
                    self._request_default_message_intervals()
                logger.info("source=%s Reconnected successfully", self.name)
                return
            except Exception as exc:
                logger.warning("source=%s reconnect failed: %s", self.name, exc)
                self.client.close()
                if self.stop_event.wait(self.cfg.reconnect_interval_sec):
                    return

    def _stop_workers(self, close_client: bool) -> None:
        with self.worker_lock:
            self.worker_stop_event.set()
            if self.receiver is not None:
                self.receiver.join(timeout=1.0)
                self.receiver = None
            if self.sender is not None:
                self.sender.join(timeout=1.0)
                self.sender = None
            self.command_queue.clear_control()
            if close_client:
                self.client.close()

    def _monitor_loop(self, logger: logging.Logger) -> None:
        while not self.stop_event.is_set():
            state = self.state_cache.get_latest_drone_state_validated(time.time())
            link = self.state_cache.get_link_status()
            if (not state.connected or state.stale) and not link.reconnecting:
                self.state_cache.mark_reconnecting()
                self._stop_workers(close_client=True)
                if self.stop_event.wait(self.cfg.reconnect_interval_sec):
                    break
                self._connect_and_start_workers(logger)
                continue
            if self.stop_event.wait(0.2):
                break

    def _request_default_message_intervals(self) -> None:
        for message_name, rate_hz in self.cfg.message_interval_hz.items():
            self.command_queue.put_action(
                ActionCommand(
                    action_type=ActionType.REQUEST_MESSAGE_INTERVAL,
                    params={"message_name": message_name, "rate_hz": rate_hz},
                    priority=20,
                    retries_left=1,
                    retry_interval_sec=self.cfg.action_retry_interval_sec,
                    created_at=time.time(),
                )
            )


class LinkManager:
    """
    Multi-source link manager.

    - Maintains separate runtimes for `real` and `sitl`.
    - Exposes only one active source outward.
    - Sends control commands only to the active source.
    """

    def __init__(self, cfg: TelemetryConfig) -> None:
        self.cfg = cfg
        self.logger = logging.getLogger(self.__class__.__name__)
        self.active_source = cfg.active_source
        self.active_lock = threading.Lock()
        self.runtimes: dict[str, SourceRuntime] = {}

        enabled_sources = (
            ["real", "sitl"] if cfg.data_source == "dual"
            else [cfg.data_source]
        )
        for source_name in enabled_sources:
            endpoint = cfg.real if source_name == "real" else cfg.sitl
            self.runtimes[source_name] = SourceRuntime(
                name=source_name,
                endpoint=endpoint,
                cfg=cfg,
                state_cache=StateCache(cfg.heartbeat_timeout_sec, cfg.rx_timeout_sec),
                command_queue=CommandQueue(),
                client=MavlinkClient(endpoint),
                stop_event=threading.Event(),
                worker_stop_event=threading.Event(),
            )

    def start(self) -> None:
        for runtime in self.runtimes.values():
            runtime.start(self.logger)

    def stop(self) -> None:
        for runtime in self.runtimes.values():
            runtime.stop()

    def get_active_source(self) -> str:
        with self.active_lock:
            return self.active_source

    def switch_active_source(self, source_name: str) -> bool:
        if source_name not in self.runtimes:
            self.logger.warning("switch_source failed: source=%s is not enabled by data_source=%s", source_name, self.cfg.data_source)
            return False
        with self.active_lock:
            self.active_source = source_name
        self.logger.info("switched active_source=%s", source_name)
        return True

    def _active_runtime(self) -> SourceRuntime:
        source_name = self.get_active_source()
        return self.runtimes[source_name]

    def get_latest_drone_state(self) -> DroneState:
        runtime = self._active_runtime()
        return runtime.state_cache.get_latest_drone_state_validated(time.time())

    def get_latest_gimbal_state(self) -> GimbalState:
        runtime = self._active_runtime()
        return runtime.state_cache.get_latest_gimbal_state_validated(time.time())

    def get_latest_state(self) -> DroneState:
        return self.get_latest_drone_state()

    def get_latest_state_raw(self) -> DroneState:
        return self._active_runtime().state_cache.get_latest_drone_state_raw()

    def get_link_status(self) -> LinkStatus:
        return self._active_runtime().state_cache.get_link_status()

    def get_source_state(self, source_name: str) -> DroneState:
        runtime = self.runtimes[source_name]
        return runtime.state_cache.get_latest_drone_state_validated(time.time())

    def get_source_gimbal_state(self, source_name: str) -> GimbalState:
        runtime = self.runtimes[source_name]
        return runtime.state_cache.get_latest_gimbal_state_validated(time.time())

    def get_source_link_status(self, source_name: str) -> LinkStatus:
        return self.runtimes[source_name].state_cache.get_link_status()

    def is_connected(self) -> bool:
        link = self.get_link_status()
        now = time.time()
        heartbeat_expired = link.last_heartbeat_time > 0 and (now - link.last_heartbeat_time) > link.heartbeat_timeout_sec
        rx_expired = link.last_rx_time > 0 and (now - link.last_rx_time) > link.rx_timeout_sec
        return link.connected and not link.reconnecting and not heartbeat_expired and not rx_expired

    def submit_control_command(self, command: ControlCommand) -> None:
        self._active_runtime().command_queue.put_control(command)

    def submit_action_command(self, command: ActionCommand) -> None:
        self._active_runtime().command_queue.put_action(command)

    def set_mode(self, mode: str, priority: int = 5) -> None:
        self.submit_action_command(
            ActionCommand(
                action_type=ActionType.SET_MODE,
                params={"mode": mode},
                priority=priority,
                retries_left=self.cfg.action_cmd_retries,
                retry_interval_sec=self.cfg.action_retry_interval_sec,
                created_at=time.time(),
            )
        )

    def arm(self, priority: int = 1) -> None:
        self.submit_action_command(
            ActionCommand(
                action_type=ActionType.ARM,
                priority=priority,
                retries_left=self.cfg.action_cmd_retries,
                retry_interval_sec=self.cfg.action_retry_interval_sec,
                created_at=time.time(),
            )
        )

    def disarm(self, priority: int = 1) -> None:
        self.submit_action_command(
            ActionCommand(
                action_type=ActionType.DISARM,
                priority=priority,
                retries_left=self.cfg.action_cmd_retries,
                retry_interval_sec=self.cfg.action_retry_interval_sec,
                created_at=time.time(),
            )
        )

    def send_gimbal_angle(
        self,
        pitch: float,
        yaw: float,
        roll: float = 0.0,
        mount_mode: int | None = None,
        priority: int = 5,
    ) -> None:
        self.submit_action_command(
            ActionCommand(
                action_type=ActionType.GIMBAL_ANGLE,
                params={
                    "pitch": float(pitch),
                    "yaw": float(yaw),
                    "roll": float(roll),
                    "mount_mode": int(self.cfg.gimbal_mount_mode if mount_mode is None else mount_mode),
                },
                priority=priority,
                retries_left=self.cfg.action_cmd_retries,
                retry_interval_sec=self.cfg.action_retry_interval_sec,
                created_at=time.time(),
            )
        )

    def send_velocity_command(self, vx: float, vy: float, vz: float, frame: int = 1) -> None:
        self.submit_control_command(
            ControlCommand(
                command_type=ControlType.VELOCITY,
                vx=vx,
                vy=vy,
                vz=vz,
                timestamp=time.time(),
                frame=frame,
            )
        )

    def send_yaw_rate_command(self, yaw_rate: float, frame: int = 1) -> None:
        self.submit_control_command(
            ControlCommand(
                command_type=ControlType.YAW_RATE,
                yaw_rate=yaw_rate,
                timestamp=time.time(),
                frame=frame,
            )
        )

    def stop_control(self, frame: int = 1) -> None:
        self.submit_control_command(
            ControlCommand(
                command_type=ControlType.STOP,
                vx=0.0,
                vy=0.0,
                vz=0.0,
                yaw_rate=0.0,
                timestamp=time.time(),
                frame=frame,
            )
        )
