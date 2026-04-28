from __future__ import annotations

import pytest

from flight_modes.common.input_adapter import FlightModeInputAdapter, InputAdapterConfig
from fusion.models import FusedState


def test_invalid_timestamp_uses_default_dt() -> None:
    adapter = FlightModeInputAdapter(config=InputAdapterConfig(dt_default=0.05))

    result = adapter.adapt(FusedState(timestamp=0.0))

    assert result.dt == pytest.approx(0.05)


def test_track_id_change_sets_track_switched() -> None:
    adapter = FlightModeInputAdapter()

    adapter.adapt(FusedState(timestamp=1.0, track_id=1))
    result = adapter.adapt(FusedState(timestamp=1.02, track_id=2))

    assert result.track_switched is True


def test_target_stable_after_hold_time() -> None:
    adapter = FlightModeInputAdapter(config=InputAdapterConfig(stable_hold_s=0.3))

    first = adapter.adapt(
        FusedState(timestamp=1.0, target_valid=True, target_locked=True, track_id=4)
    )
    second = adapter.adapt(
        FusedState(timestamp=1.31, target_valid=True, target_locked=True, track_id=4)
    )

    assert first.target_stable is False
    assert second.target_stable is True


def test_source_ages_are_computed_from_time_fn() -> None:
    adapter = FlightModeInputAdapter(_time_fn=lambda: 10.0)

    result = adapter.adapt(
        FusedState(
            timestamp=9.0,
            perception_timestamp=8.5,
            drone_timestamp=8.0,
            gimbal_timestamp=7.5,
        )
    )

    assert result.fusion_age_s == pytest.approx(1.0)
    assert result.vision_age_s == pytest.approx(1.5)
    assert result.drone_age_s == pytest.approx(2.0)
    assert result.gimbal_age_s == pytest.approx(2.5)

