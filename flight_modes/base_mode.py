from __future__ import annotations

from typing import Protocol

from flight_modes.common.types import FlightCommand, FlightModeInput, FlightModeStatus


class FlightMode(Protocol):
    name: str

    def reset(self) -> None:
        ...

    def update(self, inputs: FlightModeInput) -> tuple[FlightCommand, FlightModeStatus]:
        ...

