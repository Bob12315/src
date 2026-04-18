from __future__ import annotations

import logging
import threading
import time

from config import TelemetryConfig
from mavlink_client import MavlinkClient
from state_cache import StateCache
from telemetry_parser import (
    control_allowed_for_mode,
    decode_copter_mode,
    global_position_is_valid,
    heartbeat_is_armed,
    parse_sys_status_values,
)


class TelemetryReceiver(threading.Thread):
    def __init__(self, client: MavlinkClient, state_cache: StateCache, cfg: TelemetryConfig, stop_event: threading.Event) -> None:
        super().__init__(name="TelemetryReceiver", daemon=True)
        self.client = client
        self.state_cache = state_cache
        self.cfg = cfg
        self.stop_event = stop_event
        self.logger = logging.getLogger(self.__class__.__name__)

    def run(self) -> None:
        while not self.stop_event.is_set():
            now = time.time()
            self._check_timeouts(now)
            try:
                message = self.client.recv_message(timeout=0.2)
            except Exception as exc:
                self.logger.warning("recv_message failed: %s", exc)
                time.sleep(self.cfg.receiver_idle_sleep_sec)
                continue

            if message is None:
                self._check_timeouts(time.time())
                time.sleep(self.cfg.receiver_idle_sleep_sec)
                continue

            now = time.time()
            msg_type = message.get_type()
            self.state_cache.update_link(last_rx_time=now)
            self.state_cache.update_state(last_message_type=msg_type)
            self._handle_message(msg_type, message, now)

    def _check_timeouts(self, now: float) -> None:
        link = self.state_cache.get_link_status()
        if link.reconnecting or not link.connected:
            return

        if link.last_heartbeat_time > 0 and (now - link.last_heartbeat_time) > self.cfg.heartbeat_timeout_sec:
            self.logger.warning("Heartbeat timeout, mark link disconnected")
            self.state_cache.mark_disconnected("heartbeat_timeout")
            return

        if link.last_rx_time > 0 and (now - link.last_rx_time) > self.cfg.rx_timeout_sec:
            self.logger.warning("RX timeout, mark link disconnected")
            self.state_cache.mark_disconnected("rx_timeout")
            return

    def _handle_message(self, msg_type: str, message, now: float) -> None:
        if msg_type == "HEARTBEAT":
            armed = heartbeat_is_armed(message.base_mode)
            mode = decode_copter_mode(message.custom_mode)
            self.state_cache.update_state(
                connected=True,
                stale=False,
                armed=armed,
                mode=mode,
                control_allowed=control_allowed_for_mode(mode),
                last_heartbeat_time=now,
            )
            self.state_cache.update_link(
                connected=True,
                reconnecting=False,
                last_rx_time=now,
                last_heartbeat_time=now,
                status_text="connected",
            )
            return

        if msg_type == "ATTITUDE":
            self.state_cache.update_state(
                attitude_valid=True,
                roll=float(message.roll),
                pitch=float(message.pitch),
                yaw=float(message.yaw),
                roll_rate=float(message.rollspeed),
                pitch_rate=float(message.pitchspeed),
                yaw_rate=float(message.yawspeed),
            )
            return

        if msg_type == "GLOBAL_POSITION_INT":
            lat = float(message.lat) / 1e7
            lon = float(message.lon) / 1e7
            self.state_cache.update_state(
                altitude_valid=True,
                lat=lat,
                lon=lon,
                altitude=float(message.alt) / 1000.0,
                relative_altitude=float(message.relative_alt) / 1000.0,
                relative_alt_valid=True,
                global_position_valid=global_position_is_valid(
                    lat,
                    lon,
                    self.state_cache.get_latest_state_raw().gps_fix_type,
                ),
            )
            return

        if msg_type == "LOCAL_POSITION_NED":
            raw_state = self.state_cache.get_latest_state_raw()
            self.state_cache.update_state(
                velocity_valid=True,
                vx=float(message.vx),
                vy=float(message.vy),
                vz=float(message.vz),
                last_velocity_time=now,
                velocity_source="ekf",
                velocity_quality="good" if int(raw_state.gps_fix_type) >= 3 else "poor",
                relative_altitude=float(-message.z),
                local_position_valid=True,
                relative_alt_valid=True,
            )
            return

        if msg_type == "VFR_HUD":
            self.state_cache.update_state(
                altitude_valid=True,
                altitude=float(message.alt),
                relative_altitude=float(message.alt),
                relative_alt_valid=True,
            )
            return

        if msg_type == "SYS_STATUS":
            voltage_v, _current_a, percentage = parse_sys_status_values(
                message.voltage_battery,
                message.current_battery,
                message.battery_remaining,
            )
            self.state_cache.update_state(
                battery_valid=True,
                battery_voltage=voltage_v,
                battery_remaining=int(round(percentage * 100.0)) if percentage == percentage else -1,
            )
            return

        if msg_type == "GPS_RAW_INT":
            gps_fix_type = int(message.fix_type)
            satellites_visible = int(message.satellites_visible)
            raw_state = self.state_cache.get_latest_state_raw()
            self.state_cache.update_state(
                gps_fix_type=gps_fix_type,
                satellites_visible=satellites_visible,
                gps_eph=float(getattr(message, "eph", -1)) / 100.0 if int(getattr(message, "eph", -1)) >= 0 else -1.0,
                gps_epv=float(getattr(message, "epv", -1)) / 100.0 if int(getattr(message, "epv", -1)) >= 0 else -1.0,
                global_position_valid=global_position_is_valid(raw_state.lat, raw_state.lon, gps_fix_type),
            )
            return

        if msg_type == "RC_CHANNELS":
            return
