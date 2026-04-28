from __future__ import annotations

import math
from dataclasses import dataclass, field

from flight_modes.common.types import FlightModeInput
from flight_modes.overhead_hold.config import OverheadGimbalConfig


@dataclass(slots=True)
class OverheadGimbalCommand:
    yaw_rate_cmd: float = 0.0
    pitch_rate_cmd: float = 0.0
    active: bool = False
    valid: bool = False


@dataclass(slots=True)
class OverheadGimbalController:
    config: OverheadGimbalConfig = field(default_factory=OverheadGimbalConfig)

    def reset(self) -> None:
        pass

    def update(self, inputs: FlightModeInput, enabled: bool = True) -> OverheadGimbalCommand:
        if not enabled:
            return self._make_inactive_command(valid=False)
        if not self._validate_input(inputs):
            return self._make_inactive_command(valid=False)

        yaw_error = self._apply_deadband(
            value=-float(inputs.gimbal_yaw),
            threshold=self.config.deadband_yaw,
        )
        pitch_error = self._apply_deadband(
            value=self.config.downward_pitch_rad - float(inputs.gimbal_pitch),
            threshold=self.config.deadband_pitch,
        )

        yaw_rate_cmd = self.config.yaw_sign * self.config.kp_yaw * yaw_error
        pitch_rate_cmd = self.config.pitch_sign * self.config.kp_pitch * pitch_error

        yaw_rate_cmd = self._clamp(
            yaw_rate_cmd,
            -self.config.max_yaw_rate,
            self.config.max_yaw_rate,
        )
        pitch_rate_cmd = self._clamp(
            pitch_rate_cmd,
            -self.config.max_pitch_rate,
            self.config.max_pitch_rate,
        )

        return OverheadGimbalCommand(
            yaw_rate_cmd=yaw_rate_cmd,
            pitch_rate_cmd=pitch_rate_cmd,
            active=not (
                math.isclose(yaw_rate_cmd, 0.0, abs_tol=1e-9)
                and math.isclose(pitch_rate_cmd, 0.0, abs_tol=1e-9)
            ),
            valid=True,
        )

    def _validate_input(self, inputs: FlightModeInput) -> bool:
        if not (inputs.gimbal_valid and inputs.vision_valid):
            return False
        return math.isfinite(float(inputs.gimbal_yaw)) and math.isfinite(
            float(inputs.gimbal_pitch)
        )

    def _apply_deadband(self, value: float, threshold: float) -> float:
        if not math.isfinite(value):
            return 0.0
        if threshold <= 0.0:
            return value
        if abs(value) < threshold:
            return 0.0
        return value

    def _clamp(self, value: float, lower: float, upper: float) -> float:
        if not math.isfinite(value):
            return 0.0
        return min(upper, max(lower, value))

    def _make_inactive_command(self, valid: bool) -> OverheadGimbalCommand:
        return OverheadGimbalCommand(active=False, valid=valid)

