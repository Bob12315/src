from __future__ import annotations

import math
from dataclasses import dataclass, field

from control.control_input import ControlInput


@dataclass(slots=True)
class ApproachControllerConfig:
    approach_task_mode: str = "APPROACH_TRACK"
    overhead_task_mode: str = "OVERHEAD_HOLD"
    target_size_ref: float = 0.35
    kp_vx: float = 1.0
    kd_vx: float = 0.0
    use_derivative: bool = False
    deadband_size: float = 0.02
    max_forward_vx: float = 0.8
    max_backward_vx: float = 0.2
    vx_sign: float = 1.0
    allow_backward: bool = False
    min_valid_target_size: float = 0.01
    overhead_kp_vx: float = 1.0
    overhead_kd_vx: float = 0.0
    overhead_use_derivative: bool = False
    overhead_deadband_ey_cam: float = 0.02
    overhead_vx_sign: float = 1.0
    overhead_allow_backward: bool = False
    dt_min: float = 1e-3


@dataclass(slots=True)
class ApproachCommand:
    vx_cmd: float = 0.0
    active: bool = False
    valid: bool = False


@dataclass(slots=True)
class ApproachController:
    config: ApproachControllerConfig = field(default_factory=ApproachControllerConfig)
    _last_longitudinal_error: float | None = field(init=False, default=None)

    def reset(self) -> None:
        self._last_longitudinal_error = None

    def update(
        self,
        ci: ControlInput,
        task_mode: str,
        enabled: bool = True,
    ) -> ApproachCommand:
        if not enabled:
            self.reset()
            return self._make_inactive_command(valid=False)

        if not self._validate_input(ci, task_mode):
            self.reset()
            return self._make_inactive_command(valid=False)

        if task_mode == self.config.overhead_task_mode:
            return self._update_overhead_hold(ci)
        return self._update_approach_track(ci)

    def _update_approach_track(self, ci: ControlInput) -> ApproachCommand:
        size_error = self._compute_size_error(
            target_size=float(ci.target_size),
            target_size_ref=self.config.target_size_ref,
        )
        size_error_for_control = self._apply_deadband(
            value=size_error,
            threshold=self.config.deadband_size,
        )

        # size_error > 0 means the target looks smaller than desired, so
        # the vehicle should move forward. size_error < 0 means the target
        # looks larger than desired, so the vehicle should slow down or back off.
        vx_cmd = self.config.vx_sign * self.config.kp_vx * size_error_for_control

        if self.config.use_derivative and not math.isclose(
            size_error_for_control,
            0.0,
            abs_tol=1e-12,
        ):
            d_size_error = self._compute_derivative(
                value=size_error,
                last_value=self._last_longitudinal_error,
                dt=float(ci.dt),
            )
            vx_cmd += self.config.vx_sign * self.config.kd_vx * d_size_error

        vx_cmd = self._clamp_vx(
            vx_cmd=vx_cmd,
            allow_backward=self.config.allow_backward,
        )
        self._last_longitudinal_error = size_error

        return ApproachCommand(
            vx_cmd=vx_cmd,
            active=not math.isclose(vx_cmd, 0.0, abs_tol=1e-9),
            valid=True,
        )

    def _update_overhead_hold(self, ci: ControlInput) -> ApproachCommand:
        longitudinal_error = self._apply_deadband(
            value=float(ci.ey_cam),
            threshold=self.config.overhead_deadband_ey_cam,
        )

        vx_cmd = (
            self.config.overhead_vx_sign
            * self.config.overhead_kp_vx
            * longitudinal_error
        )

        if self.config.overhead_use_derivative and not math.isclose(
            longitudinal_error,
            0.0,
            abs_tol=1e-12,
        ):
            d_error = self._compute_derivative(
                value=float(ci.ey_cam),
                last_value=self._last_longitudinal_error,
                dt=float(ci.dt),
            )
            vx_cmd += self.config.overhead_vx_sign * self.config.overhead_kd_vx * d_error

        vx_cmd = self._clamp_vx(
            vx_cmd=vx_cmd,
            allow_backward=self.config.overhead_allow_backward,
        )
        self._last_longitudinal_error = float(ci.ey_cam)

        return ApproachCommand(
            vx_cmd=vx_cmd,
            active=not math.isclose(vx_cmd, 0.0, abs_tol=1e-9),
            valid=True,
        )

    def _compute_size_error(self, target_size: float, target_size_ref: float) -> float:
        if not math.isfinite(target_size) or not math.isfinite(target_size_ref):
            return 0.0
        return target_size_ref - target_size

    def _apply_deadband(self, value: float, threshold: float) -> float:
        if not math.isfinite(value):
            return 0.0
        if threshold <= 0.0:
            return value
        if abs(value) < threshold:
            return 0.0
        return value

    def _clamp_vx(self, vx_cmd: float, allow_backward: bool) -> float:
        if not math.isfinite(vx_cmd):
            return 0.0
        vx_cmd = min(self.config.max_forward_vx, vx_cmd)
        if allow_backward:
            return max(-self.config.max_backward_vx, vx_cmd)
        return max(0.0, vx_cmd)

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

    def _make_inactive_command(self, valid: bool) -> ApproachCommand:
        return ApproachCommand(
            vx_cmd=0.0,
            active=False,
            valid=valid,
        )

    def _validate_input(self, ci: ControlInput, task_mode: str) -> bool:
        if not (ci.target_valid and ci.vision_valid and ci.drone_valid):
            return False
        if task_mode == self.config.overhead_task_mode:
            return math.isfinite(float(ci.ey_cam))
        if not ci.target_size_valid:
            return False
        target_size = float(ci.target_size)
        if not math.isfinite(target_size):
            return False
        if target_size <= self.config.min_valid_target_size:
            return False
        return True
