from __future__ import annotations

import pytest

from flight_modes.approach_track import ApproachTrackMode
from flight_modes.common.types import FlightModeInput


def _inputs(**overrides) -> FlightModeInput:
    data = dict(
        timestamp=1.0,
        dt=0.02,
        fused_valid=True,
        target_valid=True,
        target_locked=True,
        vision_valid=True,
        drone_valid=True,
        gimbal_valid=True,
        control_allowed=True,
        track_id=1,
        track_switched=False,
        target_stable=True,
        ex_cam=0.06,
        ey_cam=-0.05,
        ex_body=0.04,
        gimbal_yaw=0.1,
        gimbal_pitch=-1.0,
        target_size=0.2,
        target_size_valid=True,
        vision_age_s=0.01,
        drone_age_s=0.01,
        gimbal_age_s=0.01,
    )
    data.update(overrides)
    return FlightModeInput(**data)


def test_approach_track_maps_errors_to_raw_command() -> None:
    command, status = ApproachTrackMode().update(_inputs())

    assert status.mode_name == "APPROACHING"
    assert command.gimbal_yaw_rate_cmd == pytest.approx(0.19206)
    assert command.gimbal_pitch_rate_cmd == pytest.approx(0.09003)
    assert command.vy_cmd == pytest.approx(0.04)
    assert command.yaw_rate_cmd == pytest.approx(0.12)
    assert command.vx_cmd == pytest.approx(0.15)


def test_approach_track_zeroes_when_target_invalid() -> None:
    command, status = ApproachTrackMode().update(_inputs(target_valid=False))

    assert command.valid is True
    assert command.active is False
    assert command.vx_cmd == pytest.approx(0.0)
    assert command.gimbal_yaw_rate_cmd == pytest.approx(0.0)
    assert status.hold_reason == "no_target"

