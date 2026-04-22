from __future__ import annotations

import math
from dataclasses import dataclass, field

from control.control_input import ControlInput


@dataclass(slots=True)
class BodyControllerConfig:
    approach_task_mode: str = "APPROACH_TRACK"
    overhead_task_mode: str = "OVERHEAD_HOLD"
    kp_vy: float = 1.0
    kd_vy: float = 0.0
    use_derivative_vy: bool = False
    kp_yaw: float = 1.2
    kd_yaw: float = 0.0
    use_derivative_yaw: bool = False
    deadband_ex_body: float = 0.02
    deadband_gimbal_yaw: float = 0.02
    max_vy: float = 1.0
    max_yaw_rate: float = 1.0
    vy_sign: float = 1.0
    yaw_sign: float = 1.0
    overhead_kp_vy: float = 1.0
    overhead_kd_vy: float = 0.0
    overhead_use_derivative_vy: bool = False
    overhead_deadband_ex_cam: float = 0.02
    overhead_kp_yaw: float = 0.0
    overhead_deadband_yaw: float = 0.05
    dt_min: float = 1e-3


@dataclass(slots=True)
class BodyCommand:
    vy_cmd: float = 0.0
    yaw_rate_cmd: float = 0.0
    active: bool = False
    valid: bool = False


@dataclass(slots=True)
class BodyController:
    config: BodyControllerConfig = field(default_factory=BodyControllerConfig)
    _last_ex_body: float | None = field(init=False, default=None)
    _last_gimbal_yaw: float | None = field(init=False, default=None)

    def reset(self) -> None:
        self._last_ex_body = None
        self._last_gimbal_yaw = None

    def update(
        self,
        ci: ControlInput,
        task_mode: str,
        enabled: bool = True,
    ) -> BodyCommand:
        if not enabled:
            self.reset()
            return self._make_inactive_command(valid=False)

        if not self._validate_input(ci, task_mode):
            self.reset()
            return self._make_inactive_command(valid=False)

        if task_mode == self.config.overhead_task_mode:
            return self._update_overhead_hold(ci)
        return self._update_approach_track(ci)

    def _update_approach_track(self, ci: ControlInput) -> BodyCommand:
        ex_body = self._apply_deadband(
            value=float(ci.ex_body),
            threshold=self.config.deadband_ex_body,
        )
        gimbal_yaw = self._apply_deadband(
            value=float(ci.gimbal_yaw),
            threshold=self.config.deadband_gimbal_yaw,
        )

        vy_cmd = self.config.vy_sign * self.config.kp_vy * ex_body
        yaw_rate_cmd = self.config.yaw_sign * self.config.kp_yaw * gimbal_yaw

        if self.config.use_derivative_vy:
            d_ex_body = self._compute_derivative(
                value=float(ci.ex_body),
                last_value=self._last_ex_body,
                dt=float(ci.dt),
            )
            vy_cmd += self.config.vy_sign * self.config.kd_vy * d_ex_body

        if self.config.use_derivative_yaw:
            d_gimbal_yaw = self._compute_derivative(
                value=float(ci.gimbal_yaw),
                last_value=self._last_gimbal_yaw,
                dt=float(ci.dt),
            )
            yaw_rate_cmd += self.config.yaw_sign * self.config.kd_yaw * d_gimbal_yaw

        vy_cmd = self._clamp(vy_cmd, -self.config.max_vy, self.config.max_vy)
        yaw_rate_cmd = self._clamp(
            yaw_rate_cmd,
            -self.config.max_yaw_rate,
            self.config.max_yaw_rate,
        )

        self._last_ex_body = float(ci.ex_body)
        self._last_gimbal_yaw = float(ci.gimbal_yaw)

        return BodyCommand(
            vy_cmd=vy_cmd,
            yaw_rate_cmd=yaw_rate_cmd,
            active=not (
                math.isclose(vy_cmd, 0.0, abs_tol=1e-9)
                and math.isclose(yaw_rate_cmd, 0.0, abs_tol=1e-9)
            ),
            valid=True,
        )

    def _update_overhead_hold(self, ci: ControlInput) -> BodyCommand:
        ex_cam = self._apply_deadband(
            value=float(ci.ex_cam),
            threshold=self.config.overhead_deadband_ex_cam,
        )
        yaw_error = self._apply_deadband(
            value=-float(ci.gimbal_yaw),
            threshold=self.config.overhead_deadband_yaw,
        )

        vy_cmd = self.config.vy_sign * self.config.overhead_kp_vy * ex_cam
        if self.config.overhead_use_derivative_vy:
            d_ex_cam = self._compute_derivative(
                value=float(ci.ex_cam),
                last_value=self._last_ex_body,
                dt=float(ci.dt),
            )
            vy_cmd += self.config.vy_sign * self.config.overhead_kd_vy * d_ex_cam

        yaw_rate_cmd = self.config.yaw_sign * self.config.overhead_kp_yaw * yaw_error

        vy_cmd = self._clamp(vy_cmd, -self.config.max_vy, self.config.max_vy)
        yaw_rate_cmd = self._clamp(
            yaw_rate_cmd,
            -self.config.max_yaw_rate,
            self.config.max_yaw_rate,
        )

        self._last_ex_body = float(ci.ex_cam)
        self._last_gimbal_yaw = float(ci.gimbal_yaw)

        return BodyCommand(
            vy_cmd=vy_cmd,
            yaw_rate_cmd=yaw_rate_cmd,
            active=not (
                math.isclose(vy_cmd, 0.0, abs_tol=1e-9)
                and math.isclose(yaw_rate_cmd, 0.0, abs_tol=1e-9)
            ),
            valid=True,
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

    def _make_inactive_command(self, valid: bool) -> BodyCommand:
        return BodyCommand(
            vy_cmd=0.0,
            yaw_rate_cmd=0.0,
            active=False,
            valid=valid,
        )

    def _validate_input(self, ci: ControlInput, task_mode: str) -> bool:
        if not (ci.target_valid and ci.vision_valid and ci.drone_valid):
            return False
        if task_mode == self.config.overhead_task_mode:
            if not math.isfinite(float(ci.ex_cam)):
                return False
            if not math.isfinite(float(ci.gimbal_yaw)):
                return False
            return True
        if not math.isfinite(float(ci.ex_body)):
            return False
        if not math.isfinite(float(ci.gimbal_yaw)):
            return False
        return True
