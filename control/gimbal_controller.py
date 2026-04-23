from __future__ import annotations

import math
from dataclasses import dataclass, field

from control.control_input import ControlInput


@dataclass(slots=True)
class GimbalControllerConfig:
    approach_task_mode: str = "APPROACH_TRACK"
    overhead_task_mode: str = "OVERHEAD_HOLD"
    kp_yaw: float = 3.2
    kp_pitch: float = 1.8
    ki_yaw: float = 0.05
    ki_pitch: float = 0.03
    kd_yaw: float = 0.18
    kd_pitch: float = 0.12
    use_derivative: bool = False
    deadband_x: float = 0.01
    deadband_y: float = 0.01
    center_hold_yaw_threshold: float = 0.018
    center_hold_pitch_threshold: float = 0.024
    integral_limit: float = 0.25
    derivative_limit_yaw: float | None = 1.5
    derivative_limit_pitch: float | None = 1.0
    max_yaw_rate: float = 1.1
    max_pitch_rate: float = 0.85
    yaw_sign: float = 1.0
    pitch_sign: float = -1.0
    overhead_downward_pitch_rad: float = -1.35
    overhead_deadband_yaw: float = 0.02
    overhead_deadband_pitch: float = 0.03
    overhead_kp_yaw: float = 0.0
    overhead_kp_pitch: float = 1.5
    overhead_max_yaw_rate: float = 0.3
    overhead_max_pitch_rate: float = 0.8
    dt_min: float = 1e-3


@dataclass(slots=True)
class GimbalCommand:
    yaw_rate_cmd: float = 0.0
    pitch_rate_cmd: float = 0.0
    active: bool = False
    valid: bool = False


@dataclass(slots=True)
class GimbalController:
    config: GimbalControllerConfig = field(default_factory=GimbalControllerConfig)
    _yaw_integral: float = field(init=False, default=0.0)
    _pitch_integral: float = field(init=False, default=0.0)
    _last_ex_cam: float | None = field(init=False, default=None)
    _last_ey_cam: float | None = field(init=False, default=None)

    def reset(self) -> None:
        self._yaw_integral = 0.0
        self._pitch_integral = 0.0
        self._last_ex_cam = None
        self._last_ey_cam = None

    def update(
        self,
        ci: ControlInput,
        task_mode: str,
        enabled: bool = True,
    ) -> GimbalCommand:
        if not enabled:
            self.reset()
            return self._make_inactive_command(valid=False)

        if not self._validate_input(ci, task_mode):
            self.reset()
            return self._make_inactive_command(valid=False)

        if task_mode == self.config.overhead_task_mode:
            return self._update_overhead_hold(ci)
        return self._update_approach_track(ci)

    def _update_approach_track(self, ci: ControlInput) -> GimbalCommand:
        ex_cam = self._apply_deadband(float(ci.ex_cam), self.config.deadband_x)
        ey_cam = self._apply_deadband(float(ci.ey_cam), self.config.deadband_y)
        dt = self._sanitize_dt(float(ci.dt))

        if abs(ex_cam) <= self.config.center_hold_yaw_threshold:
            yaw_rate_cmd = 0.0
            self._yaw_integral = 0.0
            self._last_ex_cam = 0.0
        else:
            d_ex = self._compute_derivative(
                value=float(ci.ex_cam),
                last_value=self._last_ex_cam,
                dt=dt,
                derivative_limit=self.config.derivative_limit_yaw,
            )
            self._yaw_integral = self._integrate(self._yaw_integral, ex_cam, dt)
            yaw_rate_cmd = self.config.yaw_sign * (
                self.config.kp_yaw * ex_cam
                + self.config.ki_yaw * self._yaw_integral
                + self.config.kd_yaw * d_ex
            )
            self._last_ex_cam = float(ci.ex_cam)

        if abs(ey_cam) <= self.config.center_hold_pitch_threshold:
            pitch_rate_cmd = 0.0
            self._pitch_integral = 0.0
            self._last_ey_cam = 0.0
        else:
            d_ey = self._compute_derivative(
                value=float(ci.ey_cam),
                last_value=self._last_ey_cam,
                dt=dt,
                derivative_limit=self.config.derivative_limit_pitch,
            )
            self._pitch_integral = self._integrate(self._pitch_integral, ey_cam, dt)
            pitch_rate_cmd = self.config.pitch_sign * (
                self.config.kp_pitch * ey_cam
                + self.config.ki_pitch * self._pitch_integral
                + self.config.kd_pitch * d_ey
            )
            self._last_ey_cam = float(ci.ey_cam)

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

        return GimbalCommand(
            yaw_rate_cmd=yaw_rate_cmd,
            pitch_rate_cmd=pitch_rate_cmd,
            active=not (
                math.isclose(yaw_rate_cmd, 0.0, abs_tol=1e-9)
                and math.isclose(pitch_rate_cmd, 0.0, abs_tol=1e-9)
            ),
            valid=True,
        )

    def _update_overhead_hold(self, ci: ControlInput) -> GimbalCommand:
        yaw_error = self._apply_deadband(
            value=-float(ci.gimbal_yaw),
            threshold=self.config.overhead_deadband_yaw,
        )
        pitch_error = self._apply_deadband(
            value=self.config.overhead_downward_pitch_rad - float(ci.gimbal_pitch),
            threshold=self.config.overhead_deadband_pitch,
        )

        yaw_rate_cmd = self.config.yaw_sign * self.config.overhead_kp_yaw * yaw_error
        pitch_rate_cmd = self.config.pitch_sign * self.config.overhead_kp_pitch * pitch_error

        yaw_rate_cmd = self._clamp(
            yaw_rate_cmd,
            -self.config.overhead_max_yaw_rate,
            self.config.overhead_max_yaw_rate,
        )
        pitch_rate_cmd = self._clamp(
            pitch_rate_cmd,
            -self.config.overhead_max_pitch_rate,
            self.config.overhead_max_pitch_rate,
        )

        self._last_ex_cam = None
        self._last_ey_cam = None
        return GimbalCommand(
            yaw_rate_cmd=yaw_rate_cmd,
            pitch_rate_cmd=pitch_rate_cmd,
            active=not (
                math.isclose(yaw_rate_cmd, 0.0, abs_tol=1e-9)
                and math.isclose(pitch_rate_cmd, 0.0, abs_tol=1e-9)
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
        derivative_limit: float | None,
    ) -> float:
        if last_value is None:
            return 0.0
        if not math.isfinite(value) or not math.isfinite(last_value):
            return 0.0
        if not math.isfinite(dt) or dt < self.config.dt_min:
            return 0.0
        derivative = (value - last_value) / dt
        if derivative_limit is not None:
            derivative = max(-derivative_limit, min(derivative_limit, derivative))
        return derivative

    def _sanitize_dt(self, dt: float) -> float:
        if not math.isfinite(dt) or dt < self.config.dt_min:
            return 0.02
        return dt

    def _integrate(self, integral: float, error: float, dt: float) -> float:
        integral += error * dt
        return max(-self.config.integral_limit, min(self.config.integral_limit, integral))

    def _make_inactive_command(self, valid: bool) -> GimbalCommand:
        return GimbalCommand(
            yaw_rate_cmd=0.0,
            pitch_rate_cmd=0.0,
            active=False,
            valid=valid,
        )

    def _validate_input(self, ci: ControlInput, task_mode: str) -> bool:
        if task_mode == self.config.overhead_task_mode:
            if not (ci.gimbal_valid and ci.vision_valid):
                return False
            if not math.isfinite(float(ci.gimbal_yaw)) or not math.isfinite(float(ci.gimbal_pitch)):
                return False
            return True
        if not (ci.target_valid and ci.vision_valid):
            return False
        if not math.isfinite(float(ci.ex_cam)) or not math.isfinite(float(ci.ey_cam)):
            return False
        return True
