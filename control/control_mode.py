from __future__ import annotations

import math
from dataclasses import dataclass, field

from control.control_input import ControlInput


@dataclass(slots=True)
class ControlModeConfig:
    approach_task_mode: str = "APPROACH_TRACK"
    overhead_task_mode: str = "OVERHEAD_HOLD"
    max_vision_age_s: float = 0.3
    max_drone_age_s: float = 0.3
    max_gimbal_age_s: float = 0.3
    yaw_align_thresh_rad: float = 0.35
    overhead_entry_target_size_thresh: float = 0.30
    overhead_entry_pitch_rad: float = -1.35
    overhead_entry_pitch_tol_rad: float = 0.20
    overhead_entry_yaw_tol_rad: float = 0.15
    overhead_entry_hold_s: float = 0.5
    overhead_exit_target_size_drop: float = 0.06
    require_target_locked_for_body: bool = True
    require_target_stable_for_approach: bool = True
    require_yaw_aligned_for_approach: bool = True
    require_gimbal_fresh_for_gimbal: bool = False
    require_gimbal_fresh_for_body: bool = True
    require_gimbal_fresh_for_approach: bool = True


@dataclass(slots=True)
class ControlModeOutput:
    mode_name: str = "IDLE"
    task_mode: str = "APPROACH_TRACK"

    enable_gimbal: bool = False
    enable_body: bool = False
    enable_approach: bool = False

    has_target: bool = False
    target_stable: bool = False
    control_ready: bool = False

    hold_reason: str = "control_not_allowed"


@dataclass(slots=True)
class ControlModeManager:
    config: ControlModeConfig = field(default_factory=ControlModeConfig)
    _task_mode: str = field(init=False, default="APPROACH_TRACK")
    _overhead_entry_since: float | None = field(init=False, default=None)
    _overhead_entry_size: float | None = field(init=False, default=None)

    def __post_init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._task_mode = self.config.approach_task_mode
        self._overhead_entry_since = None
        self._overhead_entry_size = None

    def update(self, ci: ControlInput) -> ControlModeOutput:
        self._update_task_mode(ci)
        vision_fresh = self._vision_fresh(ci)
        drone_fresh = self._drone_fresh(ci)
        gimbal_fresh = self._gimbal_fresh(ci)

        enable_gimbal = self._compute_enable_gimbal(
            ci=ci,
            vision_fresh=vision_fresh,
            gimbal_fresh=gimbal_fresh,
        )
        enable_body = self._compute_enable_body(
            ci=ci,
            vision_fresh=vision_fresh,
            drone_fresh=drone_fresh,
            gimbal_fresh=gimbal_fresh,
        )
        enable_approach = self._compute_enable_approach(
            ci=ci,
            vision_fresh=vision_fresh,
            drone_fresh=drone_fresh,
            gimbal_fresh=gimbal_fresh,
        )

        control_ready = (
            ci.control_allowed
            and ci.fused_valid
            and vision_fresh
            and drone_fresh
            and ci.target_valid
        )
        mode_name = self._compute_mode_name(
            task_mode=self._task_mode,
            enable_gimbal=enable_gimbal,
            enable_body=enable_body,
            enable_approach=enable_approach,
        )
        hold_reason = self._compute_hold_reason(
            ci=ci,
            vision_fresh=vision_fresh,
            drone_fresh=drone_fresh,
            gimbal_fresh=gimbal_fresh,
            enable_body=enable_body,
            enable_approach=enable_approach,
        )

        return ControlModeOutput(
            mode_name=mode_name,
            task_mode=self._task_mode,
            enable_gimbal=enable_gimbal,
            enable_body=enable_body,
            enable_approach=enable_approach,
            has_target=bool(ci.target_valid),
            target_stable=bool(ci.target_stable),
            control_ready=control_ready,
            hold_reason=hold_reason,
        )

    def _vision_fresh(self, ci: ControlInput) -> bool:
        return ci.vision_valid and self._age_fresh(
            ci.vision_age_s,
            self.config.max_vision_age_s,
        )

    def _drone_fresh(self, ci: ControlInput) -> bool:
        return ci.drone_valid and self._age_fresh(
            ci.drone_age_s,
            self.config.max_drone_age_s,
        )

    def _gimbal_fresh(self, ci: ControlInput) -> bool:
        return ci.gimbal_valid and self._age_fresh(
            ci.gimbal_age_s,
            self.config.max_gimbal_age_s,
        )

    def _compute_enable_gimbal(
        self,
        ci: ControlInput,
        vision_fresh: bool,
        gimbal_fresh: bool,
    ) -> bool:
        if not (ci.control_allowed and ci.fused_valid and vision_fresh and ci.target_valid):
            return False
        if self.config.require_gimbal_fresh_for_gimbal and not gimbal_fresh:
            return False
        return True

    def _compute_enable_body(
        self,
        ci: ControlInput,
        vision_fresh: bool,
        drone_fresh: bool,
        gimbal_fresh: bool,
    ) -> bool:
        if not ci.control_allowed:
            return False
        if not ci.fused_valid or not vision_fresh or not drone_fresh:
            return False
        if self.config.require_gimbal_fresh_for_body and not gimbal_fresh:
            return False
        if not ci.target_valid:
            return False
        if self.config.require_target_locked_for_body and not ci.target_locked:
            return False
        return True

    def _compute_enable_approach(
        self,
        ci: ControlInput,
        vision_fresh: bool,
        drone_fresh: bool,
        gimbal_fresh: bool,
    ) -> bool:
        if not ci.control_allowed:
            return False
        if not ci.fused_valid or not vision_fresh or not drone_fresh:
            return False
        if self.config.require_gimbal_fresh_for_approach and not gimbal_fresh:
            return False
        if not ci.target_valid or not ci.target_locked:
            return False
        if self._task_mode == self.config.overhead_task_mode:
            return True
        if self.config.require_target_stable_for_approach and not ci.target_stable:
            return False
        if ci.track_switched:
            return False
        if not ci.target_size_valid:
            return False
        if self.config.require_yaw_aligned_for_approach and not self._yaw_aligned(ci):
            return False
        return True

    def _compute_mode_name(
        self,
        task_mode: str,
        enable_gimbal: bool,
        enable_body: bool,
        enable_approach: bool,
    ) -> str:
        if task_mode == self.config.overhead_task_mode:
            return "OVERHEAD_HOLD" if (enable_gimbal and enable_body and enable_approach) else "OVERHEAD_RECOVERY"
        if enable_gimbal and enable_body and enable_approach:
            return "APPROACHING"
        if enable_gimbal and enable_body:
            return "ALIGNING"
        if enable_gimbal:
            return "TRACKING"
        return "IDLE"

    def _update_task_mode(self, ci: ControlInput) -> None:
        if self._task_mode == self.config.overhead_task_mode:
            if self._should_exit_overhead(ci):
                self._task_mode = self.config.approach_task_mode
                self._overhead_entry_since = None
                self._overhead_entry_size = None
            return

        if self._entry_conditions_met(ci):
            if self._overhead_entry_since is None:
                self._overhead_entry_since = float(ci.timestamp)
            entry_elapsed = float(ci.timestamp) - self._overhead_entry_since
            if entry_elapsed >= self.config.overhead_entry_hold_s:
                self._task_mode = self.config.overhead_task_mode
                self._overhead_entry_size = float(ci.target_size)
                self._overhead_entry_since = None
            return

        self._overhead_entry_since = None

    def _entry_conditions_met(self, ci: ControlInput) -> bool:
        if not ci.target_stable or not ci.target_locked:
            return False
        if ci.track_switched:
            return False
        if not self._vision_fresh(ci) or not self._drone_fresh(ci) or not self._gimbal_fresh(ci):
            return False
        if not ci.target_size_valid or float(ci.target_size) <= self.config.overhead_entry_target_size_thresh:
            return False
        if not self._pitch_near_downward(ci):
            return False
        if not self._yaw_aligned_for_overhead(ci):
            return False
        return True

    def _should_exit_overhead(self, ci: ControlInput) -> bool:
        if not ci.target_valid or not ci.target_locked:
            return True
        if ci.track_switched:
            return True
        if not self._vision_fresh(ci):
            return True
        if not self._gimbal_fresh(ci):
            return True
        if not ci.target_size_valid:
            return True
        entry_size = self._overhead_entry_size
        if entry_size is not None:
            if float(ci.target_size) < (entry_size - self.config.overhead_exit_target_size_drop):
                return True
        return False

    def _pitch_near_downward(self, ci: ControlInput) -> bool:
        pitch = float(ci.gimbal_pitch)
        if not math.isfinite(pitch):
            return False
        return abs(pitch - self.config.overhead_entry_pitch_rad) <= self.config.overhead_entry_pitch_tol_rad

    def _yaw_aligned_for_overhead(self, ci: ControlInput) -> bool:
        yaw = float(ci.gimbal_yaw)
        if not math.isfinite(yaw):
            return False
        return abs(yaw) <= self.config.overhead_entry_yaw_tol_rad

    def _compute_hold_reason(
        self,
        ci: ControlInput,
        vision_fresh: bool,
        drone_fresh: bool,
        gimbal_fresh: bool,
        enable_body: bool,
        enable_approach: bool,
    ) -> str:
        if enable_body and enable_approach:
            return ""
        if not ci.control_allowed:
            return "control_not_allowed"
        if not ci.fused_valid:
            return "fusion_invalid"
        if not ci.vision_valid:
            return "vision_invalid"
        if not vision_fresh:
            return "vision_stale"
        if not ci.target_valid:
            return "no_target"
        if not ci.drone_valid:
            return "drone_invalid"
        if not drone_fresh:
            return "drone_stale"
        if self.config.require_gimbal_fresh_for_gimbal and not gimbal_fresh:
            return "gimbal_stale"
        if (self.config.require_gimbal_fresh_for_body or self.config.require_gimbal_fresh_for_approach) and not ci.gimbal_valid:
            return "gimbal_invalid"
        if (self.config.require_gimbal_fresh_for_body or self.config.require_gimbal_fresh_for_approach) and not gimbal_fresh:
            return "gimbal_stale"
        if self.config.require_target_locked_for_body and not ci.target_locked:
            return "target_not_locked"
        if not ci.target_locked:
            return "target_not_locked"
        if self._task_mode == self.config.overhead_task_mode:
            return ""
        if ci.track_switched:
            return "track_switched"
        if self.config.require_target_stable_for_approach and not ci.target_stable:
            return "target_not_stable"
        if not ci.target_size_valid:
            return "target_size_invalid"
        if self.config.require_yaw_aligned_for_approach and not self._yaw_aligned(ci):
            return "yaw_not_aligned"
        return ""

    def _yaw_aligned(self, ci: ControlInput) -> bool:
        yaw = float(ci.gimbal_yaw)
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
