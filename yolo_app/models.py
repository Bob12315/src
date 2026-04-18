from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any


@dataclass(slots=True)
class Track:
    track_id: int
    class_id: int
    class_name: str
    confidence: float
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def cx(self) -> float:
        return (self.x1 + self.x2) / 2.0

    @property
    def cy(self) -> float:
        return (self.y1 + self.y2) / 2.0

    @property
    def w(self) -> float:
        return max(0.0, self.x2 - self.x1)

    @property
    def h(self) -> float:
        return max(0.0, self.y2 - self.y1)

    @property
    def area(self) -> float:
        return self.w * self.h


@dataclass(slots=True)
class CurrentTarget:
    timestamp: float
    frame_id: int
    target_valid: bool
    tracking_state: str
    track_id: int
    class_id: int
    class_name: str
    confidence: float
    cx: float
    cy: float
    w: float
    h: float
    ex: float
    ey: float
    image_width: int
    image_height: int
    lost_count: int

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class CommandMessage:
    action: str
    track_id: int | None = None


@dataclass(slots=True)
class FramePacket:
    frame: Any
    frame_id: int
    timestamp: float
