from __future__ import annotations

from dataclasses import dataclass, field

from flight_modes.approach_track import ApproachTrackConfig, ApproachTrackMode
from flight_modes.base_mode import FlightMode
from flight_modes.corridor_follow import CorridorFollowMode
from flight_modes.overhead_hold import OverheadHoldConfig, OverheadHoldMode


@dataclass(slots=True)
class ModeRegistry:
    approach_config: ApproachTrackConfig
    overhead_config: OverheadHoldConfig
    _modes: dict[str, FlightMode] = field(init=False)

    def __post_init__(self) -> None:
        self._modes = {
            ApproachTrackMode.name: ApproachTrackMode(config=self.approach_config),
            OverheadHoldMode.name: OverheadHoldMode(config=self.overhead_config),
            CorridorFollowMode.name: CorridorFollowMode(),
        }

    def get(self, name: str) -> FlightMode:
        try:
            return self._modes[name]
        except KeyError as exc:
            raise KeyError(f"unknown flight mode: {name}") from exc

    def reset_all(self) -> None:
        for mode in self._modes.values():
            mode.reset()

