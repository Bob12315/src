from __future__ import annotations

import logging
import threading
import time

from pymavlink import mavutil

from command_queue import CommandQueue
from config import TelemetryConfig
from mavlink_client import MavlinkClient
from models import ActionCommand, ActionType, ControlCommand, ControlType
from rate_controller import RateController
from state_cache import StateCache


class CommandSender(threading.Thread):
    def __init__(
        self,
        client: MavlinkClient,
        command_queue: CommandQueue,
        state_cache: StateCache,
        cfg: TelemetryConfig,
        stop_event: threading.Event,
    ) -> None:
        super().__init__(name="CommandSender", daemon=True)
        self.client = client
        self.command_queue = command_queue
        self.state_cache = state_cache
        self.cfg = cfg
        self.stop_event = stop_event
        self.logger = logging.getLogger(self.__class__.__name__)
        self.control_rate = RateController(cfg.control_send_rate_hz)

    def run(self) -> None:
        warned_disconnected = False
        while not self.stop_event.is_set():
            did_work = False
            link = self.state_cache.get_link_status()

            if not link.connected:
                if self.command_queue.peek_control() is not None:
                    self.command_queue.clear_control()
                if not warned_disconnected:
                    self.logger.warning("Stop sending control commands because link is disconnected")
                    warned_disconnected = True
                action = self.command_queue.get_next_action()
                if action is not None:
                    self.logger.warning("Drop action command %s because link is disconnected", action.action_type)
                time.sleep(self.cfg.sender_idle_sleep_sec)
                continue

            warned_disconnected = False

            action = self.command_queue.get_next_action()
            if action is not None:
                did_work = True
                self._send_action(action)

            control = self.command_queue.peek_control()
            if control is not None and self.control_rate.ready():
                did_work = True
                self._send_control(control)

            if not did_work:
                time.sleep(self.cfg.sender_idle_sleep_sec)

    def _send_control(self, command: ControlCommand) -> None:
        try:
            if command.command_type == ControlType.VELOCITY:
                self.client.send_raw_message(lambda master: self._send_velocity(master, command))
            elif command.command_type == ControlType.YAW_RATE:
                self.client.send_raw_message(lambda master: self._send_yaw_rate(master, command))
            elif command.command_type == ControlType.STOP:
                self.client.send_raw_message(lambda master: self._send_velocity(master, command))
            else:
                return
            self.state_cache.update_link(last_tx_time=time.time())
        except Exception as exc:
            self.logger.warning("failed to send control command: %s", exc)

    def _send_action(self, command: ActionCommand) -> None:
        try:
            if command.action_type == ActionType.ARM:
                self.client.send_raw_message(
                    lambda master: self._command_long(
                        master,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    )
                )
            elif command.action_type == ActionType.DISARM:
                self.client.send_raw_message(
                    lambda master: self._command_long(
                        master,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    )
                )
            elif command.action_type == ActionType.SET_MODE:
                self.client.send_raw_message(lambda master: self._send_set_mode(master, str(command.params["mode"])))
            elif command.action_type == ActionType.REQUEST_MESSAGE_INTERVAL:
                self.client.send_raw_message(
                    lambda master: self._send_message_interval_request(
                        master,
                        message_name=str(command.params["message_name"]),
                        rate_hz=float(command.params["rate_hz"]),
                    )
                )
            else:
                return
            self.state_cache.update_link(last_tx_time=time.time())
        except Exception as exc:
            self.logger.warning("failed to send action command %s: %s", command.action_type, exc)
            if command.retries_left > 0:
                command.retries_left -= 1
                time.sleep(command.retry_interval_sec)
                self.command_queue.requeue_action(command)

    def _send_velocity(self, master, command: ControlCommand) -> None:
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        master.mav.set_position_target_local_ned_send(
            0,
            master.target_system,
            master.target_component,
            command.frame,
            type_mask,
            0.0,
            0.0,
            0.0,
            command.vx,
            command.vy,
            command.vz,
            0.0,
            0.0,
            0.0,
            0.0,
            command.yaw_rate,
        )

    def _send_yaw_rate(self, master, command: ControlCommand) -> None:
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        )
        master.mav.set_position_target_local_ned_send(
            0,
            master.target_system,
            master.target_component,
            command.frame,
            type_mask,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            command.yaw_rate,
        )

    def _send_set_mode(self, master, mode_name: str) -> None:
        mapping = {str(key).upper(): int(value) for key, value in (master.mode_mapping() or {}).items()}
        mode = str(mode_name).strip().upper()
        if mode not in mapping:
            raise ValueError(f"unknown mode: {mode_name}")
        master.set_mode(mapping[mode])

    def _command_long(self, master, command: int, params: list[float]) -> None:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            int(command),
            0,
            float(params[0]),
            float(params[1]),
            float(params[2]),
            float(params[3]),
            float(params[4]),
            float(params[5]),
            float(params[6]),
        )

    def _send_message_interval_request(self, master, message_name: str, rate_hz: float) -> None:
        message_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{message_name}")
        interval_us = int(1e6 / rate_hz) if rate_hz > 0 else -1
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            interval_us,
            0,
            0,
            0,
            0,
            0,
        )
