from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import ClassVar

from flight_modes.approach_track.approach import ApproachForwardController
from flight_modes.approach_track.body import ApproachBodyController
from flight_modes.approach_track.config import ApproachTrackConfig
from flight_modes.approach_track.gimbal import ApproachGimbalController
from flight_modes.common.types import FlightCommand, FlightModeInput, FlightModeStatus


@dataclass(slots=True)
class ApproachTrackMode:
    name: ClassVar[str] = "APPROACH_TRACK"
    config: ApproachTrackConfig = field(default_factory=ApproachTrackConfig)
    gimbal: ApproachGimbalController = field(init=False)
    body: ApproachBodyController = field(init=False)
    approach: ApproachForwardController = field(init=False)

    def __post_init__(self) -> None:
        self.gimbal = ApproachGimbalController(config=self.config.gimbal)
        self.body = ApproachBodyController(config=self.config.body)
        self.approach = ApproachForwardController(config=self.config.approach)

    def reset(self) -> None:
        self.gimbal.reset()
        self.body.reset()
        self.approach.reset()

    def update(self, inputs: FlightModeInput) -> tuple[FlightCommand, FlightModeStatus]:
        vision_fresh = self._vision_fresh(inputs)
        drone_fresh = self._drone_fresh(inputs)
        gimbal_fresh = self._gimbal_fresh(inputs)

        enable_gimbal = self._compute_enable_gimbal(inputs, vision_fresh, gimbal_fresh)
        enable_body = self._compute_enable_body(
            inputs,
            vision_fresh,
            drone_fresh,
            gimbal_fresh,
        )
        enable_approach = self._compute_enable_approach(
            inputs,
            vision_fresh,
            drone_fresh,
            gimbal_fresh,
        )

        gimbal_cmd = self.gimbal.update(inputs, enabled=enable_gimbal)
        body_cmd = self.body.update(inputs, enabled=enable_body)
        approach_cmd = self.approach.update(inputs, enabled=enable_approach)

        command = FlightCommand(
            vx_cmd=approach_cmd.vx_cmd if approach_cmd.valid and approach_cmd.active else 0.0,
            vy_cmd=body_cmd.vy_cmd if body_cmd.valid and body_cmd.active else 0.0,
            yaw_rate_cmd=body_cmd.yaw_rate_cmd if body_cmd.valid and body_cmd.active else 0.0,
            gimbal_yaw_rate_cmd=(
                gimbal_cmd.yaw_rate_cmd if gimbal_cmd.valid and gimbal_cmd.active else 0.0
            ),
            gimbal_pitch_rate_cmd=(
                gimbal_cmd.pitch_rate_cmd if gimbal_cmd.valid and gimbal_cmd.active else 0.0
            ),
            enable_body=enable_body,
            enable_gimbal=enable_gimbal,
            enable_approach=enable_approach,
            active=enable_gimbal or enable_body or enable_approach,
            valid=True,
        )
        mode_name = self._compute_mode_name(enable_gimbal, enable_body, enable_approach)
        hold_reason = self._compute_hold_reason(
            inputs,
            vision_fresh,
            drone_fresh,
            gimbal_fresh,
            enable_body,
            enable_approach,
        )
        status = FlightModeStatus(
            mode_name=mode_name,
            active=command.active,
            valid=True,
            hold_reason=hold_reason,
            detail={
                "enable_gimbal": enable_gimbal,
                "enable_body": enable_body,
                "enable_approach": enable_approach,
                "vision_fresh": vision_fresh,
                "drone_fresh": drone_fresh,
                "gimbal_fresh": gimbal_fresh,
                "target_stable": bool(inputs.target_stable),
            },
        )
        return command, status

    def _compute_enable_gimbal(
        self,
        inputs: FlightModeInput,
        vision_fresh: bool,
        gimbal_fresh: bool,
    ) -> bool:
        if not (
            inputs.control_allowed
            and inputs.fused_valid
            and vision_fresh
            and inputs.target_valid
        ):
            return False
        if self.config.require_gimbal_fresh_for_gimbal and not gimbal_fresh:
            return False
        return True

    def _compute_enable_body(
        self,
        inputs: FlightModeInput,
        vision_fresh: bool,
        drone_fresh: bool,
        gimbal_fresh: bool,
    ) -> bool:
        if not inputs.control_allowed:
            return False
        if not inputs.fused_valid or not vision_fresh or not drone_fresh:
            return False
        if self.config.require_gimbal_fresh_for_body and not gimbal_fresh:
            return False
        if not inputs.target_valid:
            return False
        if self.config.require_target_locked_for_body and not inputs.target_locked:
            return False
        return True

    def _compute_enable_approach(
        self,
        inputs: FlightModeInput,
        vision_fresh: bool,
        drone_fresh: bool,
        gimbal_fresh: bool,
    ) -> bool:
        if not inputs.control_allowed:
            return False
        if not inputs.fused_valid or not vision_fresh or not drone_fresh:
            return False
        if self.config.require_gimbal_fresh_for_approach and not gimbal_fresh:
            return False
        if not inputs.target_valid or not inputs.target_locked:
            return False
        if self.config.require_target_stable_for_approach and not inputs.target_stable:
            return False
        if inputs.track_switched:
            return False
        if not inputs.target_size_valid:
            return False
        if self.config.require_yaw_aligned_for_approach and not self._yaw_aligned(inputs):
            return False
        return True

    def _compute_mode_name(
        self,
        enable_gimbal: bool,
        enable_body: bool,
        enable_approach: bool,
    ) -> str:
        if enable_gimbal and enable_body and enable_approach:
            return "APPROACHING"
        if enable_gimbal and enable_body:
            return "ALIGNING"
        if enable_gimbal:
            return "TRACKING"
        return "IDLE"

    def _compute_hold_reason(
        self,
        inputs: FlightModeInput,
        vision_fresh: bool,
        drone_fresh: bool,
        gimbal_fresh: bool,
        enable_body: bool,
        enable_approach: bool,
    ) -> str:
        if enable_body and enable_approach:
            return ""
        if not inputs.control_allowed:
            return "control_not_allowed"
        if not inputs.fused_valid:
            return "fusion_invalid"
        if not inputs.vision_valid:
            return "vision_invalid"
        if not vision_fresh:
            return "vision_stale"
        if not inputs.target_valid:
            return "no_target"
        if not inputs.drone_valid:
            return "drone_invalid"
        if not drone_fresh:
            return "drone_stale"
        if self.config.require_gimbal_fresh_for_gimbal and not gimbal_fresh:
            return "gimbal_stale"
        if (
            self.config.require_gimbal_fresh_for_body
            or self.config.require_gimbal_fresh_for_approach
        ) and not inputs.gimbal_valid:
            return "gimbal_invalid"
        if (
            self.config.require_gimbal_fresh_for_body
            or self.config.require_gimbal_fresh_for_approach
        ) and not gimbal_fresh:
            return "gimbal_stale"
        if self.config.require_target_locked_for_body and not inputs.target_locked:
            return "target_not_locked"
        if not inputs.target_locked:
            return "target_not_locked"
        if inputs.track_switched:
            return "track_switched"
        if self.config.require_target_stable_for_approach and not inputs.target_stable:
            return "target_not_stable"
        if not inputs.target_size_valid:
            return "target_size_invalid"
        if self.config.require_yaw_aligned_for_approach and not self._yaw_aligned(inputs):
            return "yaw_not_aligned"
        return ""

    def _vision_fresh(self, inputs: FlightModeInput) -> bool:
        return inputs.vision_valid and self._age_fresh(
            inputs.vision_age_s,
            self.config.max_vision_age_s,
        )

    def _drone_fresh(self, inputs: FlightModeInput) -> bool:
        return inputs.drone_valid and self._age_fresh(
            inputs.drone_age_s,
            self.config.max_drone_age_s,
        )

    def _gimbal_fresh(self, inputs: FlightModeInput) -> bool:
        return inputs.gimbal_valid and self._age_fresh(
            inputs.gimbal_age_s,
            self.config.max_gimbal_age_s,
        )

    def _yaw_aligned(self, inputs: FlightModeInput) -> bool:
        yaw = float(inputs.gimbal_yaw)
        if not math.isfinite(yaw):
            return False
        return abs(yaw) <= self.config.yaw_align_thresh_rad

    @staticmethod
    def _age_fresh(age_s: float, max_age_s: float) -> bool:
        age_s = float(age_s)
        max_age_s = float(max_age_s)
        if not math.isfinite(age_s) or not math.isfinite(max_age_s):
            return False
        if max_age_s < 0.0:
            return False
        return age_s <= max_age_s
