from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class ControlType(str, Enum):
    VELOCITY = "velocity"
    YAW_RATE = "yaw_rate"
    STOP = "stop"


class ActionType(str, Enum):
    ARM = "arm"
    DISARM = "disarm"
    SET_MODE = "set_mode"
    REQUEST_MESSAGE_INTERVAL = "request_message_interval"
    GIMBAL_ANGLE = "gimbal_angle"


@dataclass(slots=True)
class DroneState:
    timestamp: float = 0.0
    connected: bool = False
    stale: bool = True
    last_rx_time: float = 0.0
    last_heartbeat_time: float = 0.0
    hb_age_sec: float = float("inf")
    rx_age_sec: float = float("inf")
    armed: bool = False
    mode: str = "UNKNOWN"
    control_allowed: bool = False
    attitude_valid: bool = False
    velocity_valid: bool = False
    altitude_valid: bool = False
    battery_valid: bool = False
    global_position_valid: bool = False
    relative_alt_valid: bool = False
    local_position_valid: bool = False
    last_attitude_time: float = 0.0
    last_velocity_time: float = 0.0
    last_altitude_time: float = 0.0
    last_battery_time: float = 0.0
    last_global_position_time: float = 0.0
    last_relative_alt_time: float = 0.0
    last_local_position_time: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    roll_rate: float = 0.0
    pitch_rate: float = 0.0
    yaw_rate: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    velocity_source: str = "ekf"
    velocity_quality: str = "poor"
    altitude: float = 0.0
    relative_altitude: float = 0.0
    lat: float = 0.0
    lon: float = 0.0
    battery_voltage: float = 0.0
    battery_remaining: int = -1
    gps_fix_type: int = 0
    satellites_visible: int = 0
    gps_eph: float = -1.0
    gps_epv: float = -1.0
    last_message_type: str = ""


@dataclass(slots=True)
class GimbalState:
    timestamp: float = 0.0
    gimbal_valid: bool = False
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    source_msg_type: str = ""
    last_update_time: float = 0.0
    raw_quaternion: tuple[float, float, float, float] | None = None


@dataclass(slots=True)
class LinkStatus:
    connected: bool = False
    reconnecting: bool = False
    last_rx_time: float = 0.0
    last_tx_time: float = 0.0
    last_heartbeat_time: float = 0.0
    heartbeat_timeout_sec: float = 3.0
    rx_timeout_sec: float = 2.0
    target_system: int = 0
    target_component: int = 0
    transport: str = ""
    status_text: str = "disconnected"


@dataclass(slots=True)
class ControlCommand:
    command_type: ControlType
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0
    timestamp: float = 0.0
    frame: int = 1


@dataclass(slots=True)
class ActionCommand:
    action_type: ActionType
    params: dict[str, Any] = field(default_factory=dict)
    priority: int = 10
    retries_left: int = 0
    retry_interval_sec: float = 0.5
    created_at: float = 0.0


@dataclass(order=True, slots=True)
class QueuedAction:
    priority: int
    sequence: int
    command: ActionCommand
