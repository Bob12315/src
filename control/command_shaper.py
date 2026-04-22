from __future__ import annotations

import math
from dataclasses import dataclass, field

from control.approach_controller import ApproachCommand
from control.body_controller import BodyCommand
from control.control_mode import ControlModeOutput
from control.gimbal_controller import GimbalCommand


@dataclass(slots=True)
class CommandShaperConfig:
    max_vx: float = 0.8
    max_vy: float = 1.0
    max_yaw_rate: float = 1.0
    max_gimbal_yaw_rate: float = 1.0
    max_gimbal_pitch_rate: float = 1.0
    max_vx_rate: float = 1.0
    max_vy_rate: float = 1.5
    max_yaw_rate_rate: float = 2.0
    max_gimbal_yaw_rate_rate: float = 3.0
    max_gimbal_pitch_rate_rate: float = 3.0
    smooth_to_zero_when_disabled: bool = True
    dt_min: float = 1e-3


@dataclass(slots=True)
class ShapedCommand:
    vx_cmd: float = 0.0
    vy_cmd: float = 0.0
    yaw_rate_cmd: float = 0.0
    gimbal_yaw_rate_cmd: float = 0.0
    gimbal_pitch_rate_cmd: float = 0.0
    enable_gimbal: bool = False
    enable_body: bool = False
    enable_approach: bool = False
    active: bool = False
    valid: bool = False


@dataclass(slots=True)
class CommandShaper:
    config: CommandShaperConfig = field(default_factory=CommandShaperConfig)
    _last_command: ShapedCommand = field(init=False, default_factory=ShapedCommand)

    def reset(self) -> None:
        self._last_command = self._make_zero_command(valid=False)

    def update(
        self,
        mode: ControlModeOutput,
        gimbal_cmd: GimbalCommand | None,
        body_cmd: BodyCommand | None,
        approach_cmd: ApproachCommand | None,
        dt: float,
    ) -> ShapedCommand:
        enable_gimbal = bool(mode.enable_gimbal)
        enable_body = bool(mode.enable_body)
        enable_approach = bool(mode.enable_approach)

        target_vx = self._select_vx(
            approach_cmd=approach_cmd,
            enabled=enable_approach,
        )
        target_vy = self._select_vy(
            body_cmd=body_cmd,
            enabled=enable_body,
        )
        target_yaw_rate = self._select_yaw_rate(
            body_cmd=body_cmd,
            enabled=enable_body,
        )
        target_gimbal_yaw_rate, target_gimbal_pitch_rate = self._select_gimbal(
            gimbal_cmd=gimbal_cmd,
            enabled=enable_gimbal,
        )

        target_vx = self._clamp_vx(target_vx)
        target_vy = self._clamp(target_vy, -self.config.max_vy, self.config.max_vy)
        target_yaw_rate = self._clamp(
            target_yaw_rate,
            -self.config.max_yaw_rate,
            self.config.max_yaw_rate,
        )
        target_gimbal_yaw_rate = self._clamp(
            target_gimbal_yaw_rate,
            -self.config.max_gimbal_yaw_rate,
            self.config.max_gimbal_yaw_rate,
        )
        target_gimbal_pitch_rate = self._clamp(
            target_gimbal_pitch_rate,
            -self.config.max_gimbal_pitch_rate,
            self.config.max_gimbal_pitch_rate,
        )

        shaped_vx = self._shape_channel(
            target=target_vx,
            previous=self._last_command.vx_cmd,
            enabled=enable_approach,
            max_delta_rate=self.config.max_vx_rate,
            dt=dt,
        )
        shaped_vy = self._shape_channel(
            target=target_vy,
            previous=self._last_command.vy_cmd,
            enabled=enable_body,
            max_delta_rate=self.config.max_vy_rate,
            dt=dt,
        )
        shaped_yaw_rate = self._shape_channel(
            target=target_yaw_rate,
            previous=self._last_command.yaw_rate_cmd,
            enabled=enable_body,
            max_delta_rate=self.config.max_yaw_rate_rate,
            dt=dt,
        )
        shaped_gimbal_yaw_rate = self._shape_channel(
            target=target_gimbal_yaw_rate,
            previous=self._last_command.gimbal_yaw_rate_cmd,
            enabled=enable_gimbal,
            max_delta_rate=self.config.max_gimbal_yaw_rate_rate,
            dt=dt,
        )
        shaped_gimbal_pitch_rate = self._shape_channel(
            target=target_gimbal_pitch_rate,
            previous=self._last_command.gimbal_pitch_rate_cmd,
            enabled=enable_gimbal,
            max_delta_rate=self.config.max_gimbal_pitch_rate_rate,
            dt=dt,
        )

        command = ShapedCommand(
            vx_cmd=self._clamp_vx(shaped_vx),
            vy_cmd=self._clamp(shaped_vy, -self.config.max_vy, self.config.max_vy),
            yaw_rate_cmd=self._clamp(
                shaped_yaw_rate,
                -self.config.max_yaw_rate,
                self.config.max_yaw_rate,
            ),
            gimbal_yaw_rate_cmd=self._clamp(
                shaped_gimbal_yaw_rate,
                -self.config.max_gimbal_yaw_rate,
                self.config.max_gimbal_yaw_rate,
            ),
            gimbal_pitch_rate_cmd=self._clamp(
                shaped_gimbal_pitch_rate,
                -self.config.max_gimbal_pitch_rate,
                self.config.max_gimbal_pitch_rate,
            ),
            enable_gimbal=enable_gimbal,
            enable_body=enable_body,
            enable_approach=enable_approach,
            active=self._compute_active(
                enable_gimbal=enable_gimbal,
                enable_body=enable_body,
                enable_approach=enable_approach,
                vx_cmd=shaped_vx,
                vy_cmd=shaped_vy,
                yaw_rate_cmd=shaped_yaw_rate,
                gimbal_yaw_rate_cmd=shaped_gimbal_yaw_rate,
                gimbal_pitch_rate_cmd=shaped_gimbal_pitch_rate,
            ),
            valid=True,
        )

        self._last_command = command
        return command

    def _select_vx(
        self,
        approach_cmd: ApproachCommand | None,
        enabled: bool,
    ) -> float:
        if not enabled:
            return 0.0
        if not self._command_available(approach_cmd):
            return 0.0
        return float(approach_cmd.vx_cmd)

    def _select_vy(
        self,
        body_cmd: BodyCommand | None,
        enabled: bool,
    ) -> float:
        if not enabled:
            return 0.0
        if not self._command_available(body_cmd):
            return 0.0
        return float(body_cmd.vy_cmd)

    def _select_yaw_rate(
        self,
        body_cmd: BodyCommand | None,
        enabled: bool,
    ) -> float:
        if not enabled:
            return 0.0
        if not self._command_available(body_cmd):
            return 0.0
        return float(body_cmd.yaw_rate_cmd)

    def _select_gimbal(
        self,
        gimbal_cmd: GimbalCommand | None,
        enabled: bool,
    ) -> tuple[float, float]:
        if not enabled:
            return 0.0, 0.0
        if not self._command_available(gimbal_cmd):
            return 0.0, 0.0
        return float(gimbal_cmd.yaw_rate_cmd), float(gimbal_cmd.pitch_rate_cmd)

    def _shape_channel(
        self,
        target: float,
        previous: float,
        enabled: bool,
        max_delta_rate: float,
        dt: float,
    ) -> float:
        target = self._sanitize(target)
        previous = self._sanitize(previous)
        if not enabled and not self.config.smooth_to_zero_when_disabled:
            return 0.0
        return self._slew_limit(
            target=target,
            previous=previous,
            max_delta_rate=max_delta_rate,
            dt=dt,
        )

    def _clamp_vx(self, value: float) -> float:
        value = self._sanitize(value)
        return self._clamp(value, -self.config.max_vx, self.config.max_vx)

    def _clamp(self, value: float, lower: float, upper: float) -> float:
        value = self._sanitize(value)
        if lower > upper:
            lower, upper = upper, lower
        return min(upper, max(lower, value))

    def _slew_limit(
        self,
        target: float,
        previous: float,
        max_delta_rate: float,
        dt: float,
    ) -> float:
        if max_delta_rate <= 0.0:
            return target
        if not math.isfinite(float(dt)) or float(dt) < self.config.dt_min:
            return target
        max_delta = max_delta_rate * float(dt)
        delta = target - previous
        delta = self._clamp(delta, -max_delta, max_delta)
        return previous + delta

    def _make_zero_command(self, valid: bool) -> ShapedCommand:
        return ShapedCommand(valid=valid)

    def _compute_active(
        self,
        enable_gimbal: bool,
        enable_body: bool,
        enable_approach: bool,
        vx_cmd: float,
        vy_cmd: float,
        yaw_rate_cmd: float,
        gimbal_yaw_rate_cmd: float,
        gimbal_pitch_rate_cmd: float,
    ) -> bool:
        if enable_gimbal or enable_body or enable_approach:
            return True
        return not all(
            math.isclose(value, 0.0, abs_tol=1e-9)
            for value in (
                vx_cmd,
                vy_cmd,
                yaw_rate_cmd,
                gimbal_yaw_rate_cmd,
                gimbal_pitch_rate_cmd,
            )
        )

    def _command_available(self, command: object | None) -> bool:
        if command is None:
            return False
        valid = getattr(command, "valid", False)
        active = getattr(command, "active", False)
        return bool(valid and active)

    @staticmethod
    def _sanitize(value: float) -> float:
        value = float(value)
        if not math.isfinite(value):
            return 0.0
        return value
