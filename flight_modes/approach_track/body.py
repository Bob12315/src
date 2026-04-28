from __future__ import annotations

import math
from dataclasses import dataclass, field

from flight_modes.approach_track.config import ApproachBodyConfig
from flight_modes.common.types import FlightModeInput


@dataclass(slots=True)
class ApproachBodyCommand:
    vy_cmd: float = 0.0
    yaw_rate_cmd: float = 0.0
    active: bool = False
    valid: bool = False


@dataclass(slots=True)
class ApproachBodyController:
    config: ApproachBodyConfig = field(default_factory=ApproachBodyConfig)
    _last_ex_body: float | None = field(init=False, default=None)
    _last_gimbal_yaw: float | None = field(init=False, default=None)

    def reset(self) -> None:
        self._last_ex_body = None
        self._last_gimbal_yaw = None

    def update(self, inputs: FlightModeInput, enabled: bool = True) -> ApproachBodyCommand:
        if not enabled:
            self.reset()
            return self._make_inactive_command(valid=False)
        if not self._validate_input(inputs):
            self.reset()
            return self._make_inactive_command(valid=False)

        ex_body = self._apply_deadband(
            value=float(inputs.ex_body),
            threshold=self.config.deadband_ex_body,
        )
        gimbal_yaw = self._apply_deadband(
            value=float(inputs.gimbal_yaw),
            threshold=self.config.deadband_gimbal_yaw,
        )

        vy_cmd = self.config.vy_sign * self.config.kp_vy * ex_body
        yaw_rate_cmd = self.config.yaw_sign * self.config.kp_yaw * gimbal_yaw

        if self.config.use_derivative_vy:
            d_ex_body = self._compute_derivative(
                value=float(inputs.ex_body),
                last_value=self._last_ex_body,
                dt=float(inputs.dt),
            )
            vy_cmd += self.config.vy_sign * self.config.kd_vy * d_ex_body

        if self.config.use_derivative_yaw:
            d_gimbal_yaw = self._compute_derivative(
                value=float(inputs.gimbal_yaw),
                last_value=self._last_gimbal_yaw,
                dt=float(inputs.dt),
            )
            yaw_rate_cmd += self.config.yaw_sign * self.config.kd_yaw * d_gimbal_yaw

        vy_cmd = self._clamp(vy_cmd, -self.config.max_vy, self.config.max_vy)
        yaw_rate_cmd = self._clamp(
            yaw_rate_cmd,
            -self.config.max_yaw_rate,
            self.config.max_yaw_rate,
        )

        self._last_ex_body = float(inputs.ex_body)
        self._last_gimbal_yaw = float(inputs.gimbal_yaw)

        return ApproachBodyCommand(
            vy_cmd=vy_cmd,
            yaw_rate_cmd=yaw_rate_cmd,
            active=not (
                math.isclose(vy_cmd, 0.0, abs_tol=1e-9)
                and math.isclose(yaw_rate_cmd, 0.0, abs_tol=1e-9)
            ),
            valid=True,
        )

    def _validate_input(self, inputs: FlightModeInput) -> bool:
        if not (inputs.target_valid and inputs.vision_valid and inputs.drone_valid):
            return False
        if not math.isfinite(float(inputs.ex_body)):
            return False
        return math.isfinite(float(inputs.gimbal_yaw))

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

    def _compute_derivative(
        self,
        value: float,
        last_value: float | None,
        dt: float,
    ) -> float:
        if last_value is None:
            return 0.0
        if not math.isfinite(value) or not math.isfinite(last_value):
            return 0.0
        if not math.isfinite(dt) or dt < self.config.dt_min:
            return 0.0
        return (value - last_value) / dt

    def _make_inactive_command(self, valid: bool) -> ApproachBodyCommand:
        return ApproachBodyCommand(active=False, valid=valid)

