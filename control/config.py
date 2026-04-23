from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from control.approach_controller import ApproachControllerConfig
from control.body_controller import BodyControllerConfig
from control.command_shaper import CommandShaperConfig
from control.control_executor import ControlExecutorConfig
from control.control_input import ControlInputAdapterConfig
from control.control_mode import ControlModeConfig
from control.gimbal_controller import GimbalControllerConfig


@dataclass(slots=True)
class ControlRuntimeConfig:
    yolo_udp_ip: str
    yolo_udp_port: int
    loop_hz: float
    perception_timeout_sec: float
    print_rate_hz: float
    require_gimbal_feedback: bool
    log_level: str


@dataclass(slots=True)
class ControlConfig:
    runtime: ControlRuntimeConfig
    input_adapter: ControlInputAdapterConfig
    mode: ControlModeConfig
    gimbal: GimbalControllerConfig
    body: BodyControllerConfig
    approach: ApproachControllerConfig
    shaper: CommandShaperConfig
    executor: ControlExecutorConfig


def _to_bool(value: str | bool) -> bool:
    if isinstance(value, bool):
        return value
    lowered = value.lower()
    if lowered in {"1", "true", "yes", "y", "on"}:
        return True
    if lowered in {"0", "false", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"invalid bool value: {value}")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Control layer config loader")
    parser.add_argument("--config", default=str(Path(__file__).with_name("config.yaml")))
    parser.add_argument("--loop-hz", type=float)
    parser.add_argument("--print-rate-hz", type=float)
    parser.add_argument("--perception-timeout-sec", type=float)
    parser.add_argument("--yolo-udp-ip")
    parser.add_argument("--yolo-udp-port", type=int)
    parser.add_argument("--require-gimbal-feedback", type=_to_bool)
    parser.add_argument("--log-level")
    return parser


def _load_yaml(path: str) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError("config yaml must be a mapping")
    return data


def _merge_runtime_overrides(merged: dict[str, Any], args: argparse.Namespace) -> None:
    runtime = dict(merged.get("runtime", {}))
    for key, value in vars(args).items():
        if key == "config" or value is None:
            continue
        runtime[key.replace("-", "_")] = value
    merged["runtime"] = runtime


def _section(data: dict[str, Any], key: str) -> dict[str, Any]:
    section = data.get(key, {})
    if section is None:
        return {}
    if not isinstance(section, dict):
        raise ValueError(f"config section '{key}' must be a mapping")
    return dict(section)


def load_config(argv: list[str] | None = None) -> ControlConfig:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    merged = _load_yaml(args.config)
    _merge_runtime_overrides(merged, args)

    runtime = _section(merged, "runtime")
    input_adapter = _section(merged, "input_adapter")
    mode = _section(merged, "mode")
    gimbal = _section(merged, "gimbal")
    body = _section(merged, "body")
    approach = _section(merged, "approach")
    shaper = _section(merged, "shaper")
    executor = _section(merged, "executor")

    return ControlConfig(
        runtime=ControlRuntimeConfig(
            yolo_udp_ip=str(runtime.get("yolo_udp_ip", "0.0.0.0")),
            yolo_udp_port=int(runtime.get("yolo_udp_port", 5005)),
            loop_hz=float(runtime.get("loop_hz", 20.0)),
            perception_timeout_sec=float(runtime.get("perception_timeout_sec", 1.0)),
            print_rate_hz=float(runtime.get("print_rate_hz", 2.0)),
            require_gimbal_feedback=bool(runtime.get("require_gimbal_feedback", True)),
            log_level=str(runtime.get("log_level", "INFO")),
        ),
        input_adapter=ControlInputAdapterConfig(
            dt_default=float(input_adapter.get("dt_default", 0.02)),
            dt_min=float(input_adapter.get("dt_min", 0.001)),
            dt_max=float(input_adapter.get("dt_max", 0.5)),
            stable_hold_s=float(input_adapter.get("stable_hold_s", 0.3)),
            age_invalid_value=float(input_adapter.get("age_invalid_value", float("inf"))),
            ex_cam_tau_s=float(input_adapter.get("ex_cam_tau_s", 0.08)),
            ey_cam_tau_s=float(input_adapter.get("ey_cam_tau_s", 0.08)),
            ex_body_tau_s=float(input_adapter.get("ex_body_tau_s", 0.08)),
            ey_body_tau_s=float(input_adapter.get("ey_body_tau_s", 0.08)),
            gimbal_yaw_tau_s=float(input_adapter.get("gimbal_yaw_tau_s", 0.10)),
            gimbal_pitch_tau_s=float(input_adapter.get("gimbal_pitch_tau_s", 0.10)),
            target_size_tau_s=float(input_adapter.get("target_size_tau_s", 0.12)),
        ),
        mode=ControlModeConfig(
            approach_task_mode=str(mode.get("approach_task_mode", "APPROACH_TRACK")),
            overhead_task_mode=str(mode.get("overhead_task_mode", "OVERHEAD_HOLD")),
            max_vision_age_s=float(mode.get("max_vision_age_s", 0.3)),
            max_drone_age_s=float(mode.get("max_drone_age_s", 0.3)),
            max_gimbal_age_s=float(mode.get("max_gimbal_age_s", 0.3)),
            yaw_align_thresh_rad=float(mode.get("yaw_align_thresh_rad", 0.35)),
            overhead_entry_target_size_thresh=float(mode.get("overhead_entry_target_size_thresh", 0.30)),
            overhead_entry_pitch_rad=float(mode.get("overhead_entry_pitch_rad", -1.35)),
            overhead_entry_pitch_tol_rad=float(mode.get("overhead_entry_pitch_tol_rad", 0.20)),
            overhead_entry_yaw_tol_rad=float(mode.get("overhead_entry_yaw_tol_rad", 0.15)),
            overhead_entry_hold_s=float(mode.get("overhead_entry_hold_s", 0.5)),
            overhead_exit_target_size_drop=float(mode.get("overhead_exit_target_size_drop", 0.06)),
            require_target_locked_for_body=bool(mode.get("require_target_locked_for_body", True)),
            require_target_stable_for_approach=bool(mode.get("require_target_stable_for_approach", True)),
            require_yaw_aligned_for_approach=bool(mode.get("require_yaw_aligned_for_approach", True)),
            require_gimbal_fresh_for_gimbal=bool(mode.get("require_gimbal_fresh_for_gimbal", False)),
            require_gimbal_fresh_for_body=bool(mode.get("require_gimbal_fresh_for_body", True)),
            require_gimbal_fresh_for_approach=bool(mode.get("require_gimbal_fresh_for_approach", True)),
        ),
        gimbal=GimbalControllerConfig(
            approach_task_mode=str(gimbal.get("approach_task_mode", "APPROACH_TRACK")),
            overhead_task_mode=str(gimbal.get("overhead_task_mode", "OVERHEAD_HOLD")),
            kp_yaw=float(gimbal.get("kp_yaw", 3.2)),
            kp_pitch=float(gimbal.get("kp_pitch", 1.8)),
            ki_yaw=float(gimbal.get("ki_yaw", 0.05)),
            ki_pitch=float(gimbal.get("ki_pitch", 0.03)),
            kd_yaw=float(gimbal.get("kd_yaw", 0.18)),
            kd_pitch=float(gimbal.get("kd_pitch", 0.12)),
            use_derivative=bool(gimbal.get("use_derivative", False)),
            deadband_x=float(gimbal.get("deadband_x", 0.01)),
            deadband_y=float(gimbal.get("deadband_y", 0.01)),
            center_hold_yaw_threshold=float(gimbal.get("center_hold_yaw_threshold", 0.018)),
            center_hold_pitch_threshold=float(gimbal.get("center_hold_pitch_threshold", 0.024)),
            integral_limit=float(gimbal.get("integral_limit", 0.25)),
            derivative_limit_yaw=(
                None
                if gimbal.get("derivative_limit_yaw", 1.5) is None
                else float(gimbal.get("derivative_limit_yaw", 1.5))
            ),
            derivative_limit_pitch=(
                None
                if gimbal.get("derivative_limit_pitch", 1.0) is None
                else float(gimbal.get("derivative_limit_pitch", 1.0))
            ),
            max_yaw_rate=float(gimbal.get("max_yaw_rate", 1.1)),
            max_pitch_rate=float(gimbal.get("max_pitch_rate", 0.85)),
            yaw_sign=float(gimbal.get("yaw_sign", 1.0)),
            pitch_sign=float(gimbal.get("pitch_sign", -1.0)),
            overhead_downward_pitch_rad=float(gimbal.get("overhead_downward_pitch_rad", -1.35)),
            overhead_deadband_yaw=float(gimbal.get("overhead_deadband_yaw", 0.02)),
            overhead_deadband_pitch=float(gimbal.get("overhead_deadband_pitch", 0.03)),
            overhead_kp_yaw=float(gimbal.get("overhead_kp_yaw", 0.0)),
            overhead_kp_pitch=float(gimbal.get("overhead_kp_pitch", 1.5)),
            overhead_max_yaw_rate=float(gimbal.get("overhead_max_yaw_rate", 0.3)),
            overhead_max_pitch_rate=float(gimbal.get("overhead_max_pitch_rate", 0.8)),
            dt_min=float(gimbal.get("dt_min", 1e-3)),
        ),
        body=BodyControllerConfig(
            approach_task_mode=str(body.get("approach_task_mode", "APPROACH_TRACK")),
            overhead_task_mode=str(body.get("overhead_task_mode", "OVERHEAD_HOLD")),
            kp_vy=float(body.get("kp_vy", 1.0)),
            kd_vy=float(body.get("kd_vy", 0.0)),
            use_derivative_vy=bool(body.get("use_derivative_vy", False)),
            kp_yaw=float(body.get("kp_yaw", 1.2)),
            kd_yaw=float(body.get("kd_yaw", 0.0)),
            use_derivative_yaw=bool(body.get("use_derivative_yaw", False)),
            deadband_ex_body=float(body.get("deadband_ex_body", 0.02)),
            deadband_gimbal_yaw=float(body.get("deadband_gimbal_yaw", 0.02)),
            max_vy=float(body.get("max_vy", 1.0)),
            max_yaw_rate=float(body.get("max_yaw_rate", 1.0)),
            vy_sign=float(body.get("vy_sign", 1.0)),
            yaw_sign=float(body.get("yaw_sign", 1.0)),
            overhead_kp_vy=float(body.get("overhead_kp_vy", 1.0)),
            overhead_kd_vy=float(body.get("overhead_kd_vy", 0.0)),
            overhead_use_derivative_vy=bool(body.get("overhead_use_derivative_vy", False)),
            overhead_deadband_ex_cam=float(body.get("overhead_deadband_ex_cam", 0.02)),
            overhead_kp_yaw=float(body.get("overhead_kp_yaw", 0.0)),
            overhead_deadband_yaw=float(body.get("overhead_deadband_yaw", 0.05)),
            dt_min=float(body.get("dt_min", 1e-3)),
        ),
        approach=ApproachControllerConfig(
            approach_task_mode=str(approach.get("approach_task_mode", "APPROACH_TRACK")),
            overhead_task_mode=str(approach.get("overhead_task_mode", "OVERHEAD_HOLD")),
            target_size_ref=float(approach.get("target_size_ref", 0.35)),
            kp_vx=float(approach.get("kp_vx", 1.0)),
            kd_vx=float(approach.get("kd_vx", 0.0)),
            use_derivative=bool(approach.get("use_derivative", False)),
            deadband_size=float(approach.get("deadband_size", 0.02)),
            max_forward_vx=float(approach.get("max_forward_vx", 0.8)),
            max_backward_vx=float(approach.get("max_backward_vx", 0.2)),
            vx_sign=float(approach.get("vx_sign", 1.0)),
            allow_backward=bool(approach.get("allow_backward", False)),
            min_valid_target_size=float(approach.get("min_valid_target_size", 0.01)),
            overhead_kp_vx=float(approach.get("overhead_kp_vx", 1.0)),
            overhead_kd_vx=float(approach.get("overhead_kd_vx", 0.0)),
            overhead_use_derivative=bool(approach.get("overhead_use_derivative", False)),
            overhead_deadband_ey_cam=float(approach.get("overhead_deadband_ey_cam", 0.02)),
            overhead_vx_sign=float(approach.get("overhead_vx_sign", 1.0)),
            overhead_allow_backward=bool(approach.get("overhead_allow_backward", False)),
            dt_min=float(approach.get("dt_min", 1e-3)),
        ),
        shaper=CommandShaperConfig(
            max_vx=float(shaper.get("max_vx", 0.8)),
            max_vy=float(shaper.get("max_vy", 1.0)),
            max_yaw_rate=float(shaper.get("max_yaw_rate", 1.0)),
            max_gimbal_yaw_rate=float(shaper.get("max_gimbal_yaw_rate", 1.0)),
            max_gimbal_pitch_rate=float(shaper.get("max_gimbal_pitch_rate", 1.0)),
            max_vx_rate=float(shaper.get("max_vx_rate", 1.0)),
            max_vy_rate=float(shaper.get("max_vy_rate", 1.5)),
            max_yaw_rate_rate=float(shaper.get("max_yaw_rate_rate", 2.0)),
            max_gimbal_yaw_rate_rate=float(shaper.get("max_gimbal_yaw_rate_rate", 3.0)),
            max_gimbal_pitch_rate_rate=float(shaper.get("max_gimbal_pitch_rate_rate", 3.0)),
            smooth_to_zero_when_disabled=bool(shaper.get("smooth_to_zero_when_disabled", True)),
            dt_min=float(shaper.get("dt_min", 1e-3)),
        ),
        executor=ControlExecutorConfig(
            body_frame=int(executor.get("body_frame", 1)),
            gimbal_roll_deg=float(executor.get("gimbal_roll_deg", 0.0)),
            log_commands=bool(executor.get("log_commands", True)),
        ),
    )
