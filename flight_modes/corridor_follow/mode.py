from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

from flight_modes.common.types import FlightCommand, FlightModeInput, FlightModeStatus
from flight_modes.corridor_follow.config import CorridorFollowConfig


@dataclass(slots=True)
class CorridorFollowMode:
    name: ClassVar[str] = "CORRIDOR_FOLLOW"
    config: CorridorFollowConfig = field(default_factory=CorridorFollowConfig)

    def reset(self) -> None:
        pass

    def update(self, inputs: FlightModeInput) -> tuple[FlightCommand, FlightModeStatus]:
        command = FlightCommand(valid=True)
        status = FlightModeStatus(
            mode_name=self.name,
            active=False,
            valid=True,
            hold_reason="not_implemented",
            detail={"enabled": self.config.enabled},
        )
        return command, status
