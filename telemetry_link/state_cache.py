from __future__ import annotations

import threading
import time
from dataclasses import replace

from models import DroneState, LinkStatus
from telemetry_parser import control_allowed_for_mode, global_position_is_valid


class StateCache:
    def __init__(self, heartbeat_timeout_sec: float, rx_timeout_sec: float) -> None:
        self._lock = threading.Lock()
        self._drone_state = DroneState()
        self._link_status = LinkStatus(
            heartbeat_timeout_sec=heartbeat_timeout_sec,
            rx_timeout_sec=rx_timeout_sec,
        )

    def update_state(self, **fields) -> None:
        with self._lock:
            for key, value in fields.items():
                setattr(self._drone_state, key, value)
            self._drone_state.timestamp = time.time()

    def get_latest_state_raw(self) -> DroneState:
        with self._lock:
            return replace(self._drone_state)

    def update_link(self, **fields) -> None:
        with self._lock:
            for key, value in fields.items():
                setattr(self._link_status, key, value)

    def get_link_status(self) -> LinkStatus:
        with self._lock:
            return replace(self._link_status)

    def get_latest_state_validated(self, now: float) -> DroneState:
        with self._lock:
            state = replace(self._drone_state)
            link = replace(self._link_status)

        heartbeat_expired = link.last_heartbeat_time > 0 and (now - link.last_heartbeat_time) > link.heartbeat_timeout_sec
        rx_expired = link.last_rx_time > 0 and (now - link.last_rx_time) > link.rx_timeout_sec
        has_no_rx = link.last_rx_time <= 0
        disconnected = link.reconnecting or not link.connected or heartbeat_expired or rx_expired or has_no_rx

        state.last_rx_time = link.last_rx_time
        state.last_heartbeat_time = link.last_heartbeat_time
        state.hb_age_sec = float("inf") if link.last_heartbeat_time <= 0 else max(0.0, now - link.last_heartbeat_time)
        state.rx_age_sec = float("inf") if link.last_rx_time <= 0 else max(0.0, now - link.last_rx_time)
        state.control_allowed = control_allowed_for_mode(state.mode)
        state.global_position_valid = global_position_is_valid(state.lat, state.lon, state.gps_fix_type)
        state.velocity_source = "ekf"
        state.velocity_quality = "good" if int(state.gps_fix_type) >= 3 else "poor"

        velocity_recent = state.last_velocity_time > 0 and (now - state.last_velocity_time) <= link.rx_timeout_sec

        if disconnected:
            state.connected = False
            state.stale = True
            state.mode = "DISCONNECTED"
            state.control_allowed = False
            state.global_position_valid = False
            state.attitude_valid = False
            state.velocity_valid = False
            state.altitude_valid = False
            state.battery_valid = False
            state.local_position_valid = False
            state.relative_alt_valid = False
        else:
            state.connected = True
            state.stale = False
            state.velocity_valid = bool(state.velocity_valid and velocity_recent)
        return state

    def mark_disconnected(self, reason: str) -> None:
        with self._lock:
            self._link_status.connected = False
            self._link_status.status_text = reason
            self._drone_state.connected = False
            self._drone_state.stale = True
            self._drone_state.mode = "DISCONNECTED"
            self._drone_state.control_allowed = False
            self._drone_state.attitude_valid = False
            self._drone_state.velocity_valid = False
            self._drone_state.altitude_valid = False
            self._drone_state.battery_valid = False
            self._drone_state.global_position_valid = False
            self._drone_state.local_position_valid = False
            self._drone_state.relative_alt_valid = False
            self._drone_state.timestamp = time.time()

    def mark_reconnecting(self) -> None:
        with self._lock:
            self._link_status.connected = False
            self._link_status.reconnecting = True
            self._link_status.status_text = "reconnecting"
            self._drone_state.connected = False
            self._drone_state.stale = True
            self._drone_state.mode = "DISCONNECTED"
            self._drone_state.control_allowed = False
            self._drone_state.attitude_valid = False
            self._drone_state.velocity_valid = False
            self._drone_state.altitude_valid = False
            self._drone_state.battery_valid = False
            self._drone_state.global_position_valid = False
            self._drone_state.local_position_valid = False
            self._drone_state.relative_alt_valid = False
            self._drone_state.timestamp = time.time()

    def mark_connected(self, *, target_system: int, target_component: int, transport: str, now: float) -> None:
        with self._lock:
            self._link_status.connected = True
            self._link_status.reconnecting = False
            self._link_status.last_rx_time = now
            self._link_status.last_heartbeat_time = now
            self._link_status.target_system = target_system
            self._link_status.target_component = target_component
            self._link_status.transport = transport
            self._link_status.status_text = "connected"
            self._drone_state.connected = True
            self._drone_state.stale = False
            self._drone_state.last_rx_time = now
            self._drone_state.last_heartbeat_time = now
            self._drone_state.hb_age_sec = 0.0
            self._drone_state.rx_age_sec = 0.0
            self._drone_state.velocity_source = "ekf"
            self._drone_state.velocity_quality = "good" if int(self._drone_state.gps_fix_type) >= 3 else "poor"
            self._drone_state.timestamp = now
