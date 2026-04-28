from __future__ import annotations

import pytest

from app.app_config import build_arg_parser, load_app_config


def test_loads_new_flight_mode_config_layout() -> None:
    args = build_arg_parser().parse_args(
        ["--no-yolo-udp", "--no-ui", "--run-seconds", "1", "--send-commands", "false"]
    )

    config = load_app_config(args)

    assert config.runtime.ui_enabled is False
    assert config.runtime.connect_telemetry is False
    assert config.approach_track.approach.kp_vx == pytest.approx(20.0)
    assert config.approach_track.require_yaw_aligned_for_approach is True
    assert config.overhead_hold.gimbal.downward_pitch_rad == pytest.approx(
        -1.5707963267948966
    )
    assert config.overhead_hold.body.kp_vy == pytest.approx(1.0)
    assert config.overhead_hold.approach.kp_vx == pytest.approx(1.0)
    assert config.shaper.max_vx == pytest.approx(1.2)
