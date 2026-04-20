from __future__ import annotations

import time
from dataclasses import asdict, is_dataclass
from typing import Any

from telemetry_link.models import DroneState, GimbalState

from .models import FusedState, PerceptionTarget

PERCEPTION_FIELDS = (
    "timestamp",
    "frame_id",
    "target_valid",
    "tracking_state",
    "track_id",
    "class_name",
    "confidence",
    "cx",
    "cy",
    "w",
    "h",
    "ex",
    "ey",
    "lost_count",
)


def normalize_perception_target(target: Any | None) -> PerceptionTarget:
    if target is None:
        return PerceptionTarget()
    if isinstance(target, PerceptionTarget):
        return target
    if is_dataclass(target):
        data = asdict(target)
    elif isinstance(target, dict):
        data = dict(target)
    else:
        data = {field: getattr(target, field) for field in PERCEPTION_FIELDS if hasattr(target, field)}
    filtered = {field: data[field] for field in PERCEPTION_FIELDS if field in data}
    return PerceptionTarget(**filtered)


def compute_body_error(
    ex_cam: float,
    ey_cam: float,
    gimbal_state: GimbalState,
    require_gimbal_feedback: bool,
) -> tuple[float, float, bool]:
    if gimbal_state.gimbal_valid:
        return ex_cam + gimbal_state.yaw, ey_cam + gimbal_state.pitch, True
    if require_gimbal_feedback:
        return ex_cam, ey_cam, False
    return ex_cam, ey_cam, True


def compute_state_valid(drone_state: DroneState) -> bool:
    if not drone_state.connected or drone_state.stale:
        return False
    return any(
        (
            drone_state.attitude_valid,
            drone_state.velocity_valid,
            drone_state.altitude_valid,
            drone_state.global_position_valid,
            drone_state.relative_alt_valid,
            drone_state.local_position_valid,
            drone_state.battery_valid,
        )
    )


def build_fused_state(
    perception_target: PerceptionTarget,
    drone_state: DroneState,
    gimbal_state: GimbalState,
    require_gimbal_feedback: bool,
) -> FusedState:
    ex_body, ey_body, gimbal_usable = compute_body_error(
        perception_target.ex,
        perception_target.ey,
        gimbal_state,
        require_gimbal_feedback,
    )

    target_locked = bool(perception_target.target_valid and perception_target.tracking_state == "locked")
    state_valid = compute_state_valid(drone_state)
    control_enabled = bool(
        perception_target.target_valid
        and perception_target.tracking_state == "locked"
        and drone_state.control_allowed
        and not drone_state.stale
        and drone_state.connected
    )
    fusion_valid = bool(state_valid and target_locked and gimbal_usable)
    if drone_state.stale:
        control_enabled = False
        fusion_valid = False

    return FusedState(
        timestamp=time.time(),
        perception_timestamp=perception_target.timestamp,
        drone_timestamp=drone_state.timestamp,
        gimbal_timestamp=gimbal_state.timestamp,
        frame_id=perception_target.frame_id,
        target_valid=perception_target.target_valid,
        target_locked=target_locked,
        track_id=perception_target.track_id,
        tracking_state=perception_target.tracking_state,
        ex_cam=perception_target.ex,
        ey_cam=perception_target.ey,
        gimbal_valid=gimbal_state.gimbal_valid,
        gimbal_yaw=gimbal_state.yaw,
        gimbal_pitch=gimbal_state.pitch,
        ex_body=ex_body,
        ey_body=ey_body,
        yaw=drone_state.yaw,
        roll=drone_state.roll,
        pitch=drone_state.pitch,
        yaw_rate=drone_state.yaw_rate,
        vx=drone_state.vx,
        vy=drone_state.vy,
        vz=drone_state.vz,
        altitude=drone_state.altitude,
        control_allowed=drone_state.control_allowed,
        control_enabled=control_enabled,
        state_valid=state_valid,
        fusion_valid=fusion_valid,
    )
