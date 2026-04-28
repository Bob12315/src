from __future__ import annotations

import pytest

from flight_modes.common.types import FlightModeInput
from flight_modes.overhead_hold import OverheadHoldMode


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
        ex_cam=0.06,
        ey_cam=0.05,
        gimbal_yaw=0.1,
        gimbal_pitch=1.0,
        target_size=0.4,
        target_size_valid=True,
        vision_age_s=0.01,
        drone_age_s=0.01,
        gimbal_age_s=0.01,
    )
    data.update(overrides)
    return FlightModeInput(**data)


def test_overhead_hold_maps_overhead_errors_to_raw_command() -> None:
    command, status = OverheadHoldMode().update(_inputs())

    assert status.mode_name == "OVERHEAD_HOLD"
    assert command.gimbal_pitch_rate_cmd == pytest.approx(-0.525)
    assert command.gimbal_yaw_rate_cmd == pytest.approx(0.0)
    assert command.vy_cmd == pytest.approx(0.06)
    assert command.vx_cmd == pytest.approx(0.05)


def test_overhead_hold_zeroes_when_target_invalid() -> None:
    command, status = OverheadHoldMode().update(_inputs(target_valid=False))

    assert command.valid is True
    assert command.active is False
    assert command.vx_cmd == pytest.approx(0.0)
    assert command.vy_cmd == pytest.approx(0.0)
    assert status.hold_reason == "no_target"
