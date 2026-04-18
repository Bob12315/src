from __future__ import annotations

import logging
import threading
import time

from command_queue import CommandQueue
from command_sender import CommandSender
from config import TelemetryConfig
from mavlink_client import MavlinkClient
from models import ActionCommand, ActionType, ControlCommand, ControlType, DroneState, LinkStatus
from state_cache import StateCache
from telemetry_receiver import TelemetryReceiver


class LinkManager:
    """
    Single owner of the MAVLink connection object.

    External callers should only interact with this class rather than touching
    the pymavlink master object directly.
    """

    def __init__(self, cfg: TelemetryConfig) -> None:
        self.cfg = cfg
        self.logger = logging.getLogger(self.__class__.__name__)
        self.state_cache = StateCache(cfg.heartbeat_timeout_sec, cfg.rx_timeout_sec)
        self.command_queue = CommandQueue()
        self.client = MavlinkClient(cfg)
        self.stop_event = threading.Event()
        self.worker_stop_event = threading.Event()
        self.receiver: TelemetryReceiver | None = None
        self.sender: CommandSender | None = None
        self.monitor_thread: threading.Thread | None = None
        self._worker_lock = threading.Lock()

    def start(self) -> None:
        self._connect_and_start_workers()
        self.monitor_thread = threading.Thread(name="LinkMonitor", target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        self._stop_workers(close_client=True)
        if self.monitor_thread is not None:
            self.monitor_thread.join(timeout=1.0)

    def get_latest_state(self) -> DroneState:
        return self.state_cache.get_latest_state_validated(time.time())

    def get_latest_state_raw(self) -> DroneState:
        return self.state_cache.get_latest_state_raw()

    def get_link_status(self) -> LinkStatus:
        return self.state_cache.get_link_status()

    def is_connected(self) -> bool:
        link = self.get_link_status()
        now = time.time()
        heartbeat_expired = link.last_heartbeat_time > 0 and (now - link.last_heartbeat_time) > link.heartbeat_timeout_sec
        rx_expired = link.last_rx_time > 0 and (now - link.last_rx_time) > link.rx_timeout_sec
        return link.connected and not link.reconnecting and not heartbeat_expired and not rx_expired

    def submit_control_command(self, command: ControlCommand) -> None:
        self.command_queue.put_control(command)

    def submit_action_command(self, command: ActionCommand) -> None:
        self.command_queue.put_action(command)

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

    def request_message_interval(self, message_name: str, rate_hz: float, priority: int = 20) -> None:
        self.submit_action_command(
            ActionCommand(
                action_type=ActionType.REQUEST_MESSAGE_INTERVAL,
                params={"message_name": message_name, "rate_hz": rate_hz},
                priority=priority,
                retries_left=1,
                retry_interval_sec=self.cfg.action_retry_interval_sec,
                created_at=time.time(),
            )
        )

    def _request_default_message_intervals(self) -> None:
        # ArduPilot usually streams telemetry by default, but the ground side can
        # still request specific rates with MAV_CMD_SET_MESSAGE_INTERVAL when needed.
        for message_name, rate_hz in self.cfg.message_interval_hz.items():
            self.request_message_interval(message_name, rate_hz)

    def _connect_and_start_workers(self) -> None:
        while not self.stop_event.is_set():
            try:
                self.state_cache.mark_reconnecting()
                self.logger.info("Attempting to reconnect...")
                self.client.connect()
                self.client.wait_heartbeat(timeout=max(5.0, self.cfg.heartbeat_timeout_sec + 2.0))
                now = time.time()
                self.state_cache.mark_connected(
                    target_system=self.client.target_system,
                    target_component=self.client.target_component,
                    transport=self.cfg.connection_type,
                    now=now,
                )
                self.worker_stop_event = threading.Event()
                self.receiver = TelemetryReceiver(self.client, self.state_cache, self.cfg, self.worker_stop_event)
                self.sender = CommandSender(self.client, self.command_queue, self.state_cache, self.cfg, self.worker_stop_event)
                self.receiver.start()
                self.sender.start()
                if self.cfg.request_message_intervals:
                    self._request_default_message_intervals()
                self.logger.info("Reconnected successfully")
                return
            except Exception as exc:
                self.logger.warning("reconnect failed: %s", exc)
                self.client.close()
                if self.stop_event.wait(self.cfg.reconnect_interval_sec):
                    return

    def _stop_workers(self, close_client: bool) -> None:
        with self._worker_lock:
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

    def _monitor_loop(self) -> None:
        while not self.stop_event.is_set():
            state = self.state_cache.get_latest_state_validated(time.time())
            link = self.state_cache.get_link_status()
            if (not state.connected or state.stale) and not link.reconnecting:
                self.state_cache.mark_reconnecting()
                self._stop_workers(close_client=True)
                if self.stop_event.wait(self.cfg.reconnect_interval_sec):
                    break
                self._connect_and_start_workers()
                continue
            if self.stop_event.wait(0.2):
                break
