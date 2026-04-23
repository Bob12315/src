from __future__ import annotations

import logging
import math
import threading
import time

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega

from command_queue import CommandQueue
from config import TelemetryConfig
from mavlink_client import MavlinkClient
from models import ActionCommand, ActionType, ControlCommand, ControlType, GimbalRateCommand
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

    def _clamp_gimbal_yaw_deg(self, yaw_deg: float) -> float:
        lower = min(float(self.cfg.gimbal_yaw_min_deg), float(self.cfg.gimbal_yaw_max_deg))
        upper = max(float(self.cfg.gimbal_yaw_min_deg), float(self.cfg.gimbal_yaw_max_deg))
        return max(lower, min(upper, float(yaw_deg)))

    def _clamp_gimbal_pitch_deg(self, pitch_deg: float) -> float:
        lower = min(float(self.cfg.gimbal_pitch_min_deg), float(self.cfg.gimbal_pitch_max_deg))
        upper = max(float(self.cfg.gimbal_pitch_min_deg), float(self.cfg.gimbal_pitch_max_deg))
        return max(lower, min(upper, float(pitch_deg)))

    def run(self) -> None:
        warned_disconnected = False
        while not self.stop_event.is_set():
            did_work = False
            link = self.state_cache.get_link_status()

            if not link.connected:
                if self.command_queue.peek_control() is not None:
                    self.command_queue.clear_control()
                if self.command_queue.peek_gimbal_rate() is not None:
                    self.command_queue.clear_gimbal_rate()
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

            ready_for_control = self.control_rate.ready()
            control = self.command_queue.peek_control()
            if control is not None and ready_for_control:
                did_work = True
                self._send_control(control)
            gimbal_rate = self.command_queue.peek_gimbal_rate()
            if gimbal_rate is not None and ready_for_control:
                did_work = True
                self._send_gimbal_rate(gimbal_rate)

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
                self.logger.info("sending action command=arm")
                self.client.send_raw_message(
                    lambda master: self._command_long(
                        master,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    )
                )
            elif command.action_type == ActionType.DISARM:
                self.logger.info("sending action command=disarm")
                self.client.send_raw_message(
                    lambda master: self._command_long(
                        master,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    )
                )
            elif command.action_type == ActionType.TAKEOFF:
                altitude_m = float(command.params["altitude_m"])
                self.logger.info("sending action command=takeoff altitude_m=%.2f", altitude_m)
                self.client.send_raw_message(
                    lambda master: self._command_long(
                        master,
                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, altitude_m],
                    )
                )
            elif command.action_type == ActionType.LAND:
                self.logger.info("sending action command=land")
                self.client.send_raw_message(
                    lambda master: self._command_long(
                        master,
                        mavutil.mavlink.MAV_CMD_NAV_LAND,
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    )
                )
            elif command.action_type == ActionType.SET_MODE:
                self.logger.info("sending action command=set_mode mode=%s", str(command.params["mode"]))
                self.client.send_raw_message(lambda master: self._send_set_mode(master, str(command.params["mode"])))
            elif command.action_type == ActionType.REQUEST_MESSAGE_INTERVAL:
                self.logger.info(
                    "sending action command=request_message_interval message=%s rate_hz=%.2f",
                    str(command.params["message_name"]),
                    float(command.params["rate_hz"]),
                )
                self.client.send_raw_message(
                    lambda master: self._send_message_interval_request(
                        master,
                        message_name=str(command.params["message_name"]),
                        rate_hz=float(command.params["rate_hz"]),
                    )
                )
            elif command.action_type == ActionType.GIMBAL_ANGLE:
                self.logger.info(
                    "sending action command=gimbal_angle pitch=%.2f yaw=%.2f roll=%.2f mount_mode=%s",
                    float(command.params["pitch"]),
                    float(command.params["yaw"]),
                    float(command.params.get("roll", 0.0)),
                    int(command.params.get("mount_mode", self.cfg.gimbal_mount_mode)),
                )
                self.client.send_raw_message(
                    lambda master: self._send_gimbal_angle(
                        master,
                        pitch=float(command.params["pitch"]),
                        yaw=float(command.params["yaw"]),
                        roll=float(command.params.get("roll", 0.0)),
                        mount_mode=int(
                            command.params.get(
                                "mount_mode",
                                self.cfg.gimbal_mount_mode,
                            )
                        ),
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

    def _send_gimbal_rate(self, command: GimbalRateCommand) -> None:
        try:
            yaw_rate_deg_s = math.degrees(float(command.yaw_rate))
            pitch_rate_deg_s = math.degrees(float(command.pitch_rate))
            self.logger.debug(
                "sending gimbal rate command pitch_rate=%.2f deg/s yaw_rate=%.2f deg/s yaw_lock=%s gimbal_device_id=%s",
                pitch_rate_deg_s,
                yaw_rate_deg_s,
                bool(command.yaw_lock),
                int(command.gimbal_device_id),
            )
            self.client.send_raw_message(
                lambda master: self._send_gimbal_manager_pitchyaw_rate(
                    master,
                    pitch_rate_deg_s=pitch_rate_deg_s,
                    yaw_rate_deg_s=yaw_rate_deg_s,
                    yaw_lock=bool(command.yaw_lock),
                    gimbal_device_id=int(command.gimbal_device_id),
                )
            )
            self.state_cache.update_link(last_tx_time=time.time())
        except Exception as exc:
            self.logger.warning("failed to send gimbal rate command: %s", exc)

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

    def _send_gimbal_manager_pitchyaw_rate(
        self,
        master,
        *,
        pitch_rate_deg_s: float,
        yaw_rate_deg_s: float,
        yaw_lock: bool,
        gimbal_device_id: int,
    ) -> None:
        flags = mavutil.mavlink.GIMBAL_MANAGER_FLAGS_YAW_LOCK if yaw_lock else 0
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            0,
            float("nan"),
            float("nan"),
            float(pitch_rate_deg_s),
            float(yaw_rate_deg_s),
            float(flags),
            0.0,
            float(gimbal_device_id),
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
        message_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{message_name}", None)
        if message_id is None:
            message_id = getattr(ardupilotmega, f"MAVLINK_MSG_ID_{message_name}", None)
        if message_id is None:
            raise ValueError(f"unknown MAVLink message name: {message_name}")
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

    def _send_gimbal_angle(
        self,
        master,
        *,
        pitch: float,
        yaw: float,
        roll: float,
        mount_mode: int,
    ) -> None:
        yaw = self._clamp_gimbal_yaw_deg(yaw)
        pitch = self._clamp_gimbal_pitch_deg(pitch)
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            0,
            pitch,
            roll,
            yaw,
            0.0,
            0.0,
            0.0,
            float(mount_mode),
        )
