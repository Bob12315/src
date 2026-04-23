from __future__ import annotations

import logging
import math
import sys
import threading
import time
from dataclasses import replace
from pathlib import Path

if __package__ in {None, ""}:
    ROOT_DIR = Path(__file__).resolve().parents[2]
    if str(ROOT_DIR) not in sys.path:
        sys.path.insert(0, str(ROOT_DIR))
    from control.control_input import ControlInputAdapter
    from control.control_mode import ControlModeManager
    from control.gimbal_controller import GimbalController
    from control.debug.common import (
        build_debug_arg_parser,
        create_live_inputs,
        install_signal_handlers,
        loop_sleep,
        print_sleep,
        read_control_input,
        resolve_debug_runtime,
    )
else:
    from control.control_input import ControlInputAdapter
    from control.control_mode import ControlModeManager
    from control.gimbal_controller import GimbalController
    from control.debug.common import (
        build_debug_arg_parser,
        create_live_inputs,
        install_signal_handlers,
        loop_sleep,
        print_sleep,
        read_control_input,
        resolve_debug_runtime,
    )


def _enabled_for_debug(control_input, mode, respect_control_gating: bool) -> tuple[bool, str, str]:
    if respect_control_gating:
        return bool(mode.enable_gimbal), str(mode.hold_reason), str(mode.mode_name)

    if not control_input.vision_valid:
        return False, "vision_invalid", "IDLE"
    if not control_input.target_valid:
        return False, "no_target", "IDLE"
    if not control_input.gimbal_valid:
        return False, "gimbal_invalid", "IDLE"
    return True, "", "TRACKING"


def _mask_command_axis(command, axis: str):
    if axis == "both":
        return command
    yaw_rate_cmd = float(command.yaw_rate_cmd) if axis == "yaw" else 0.0
    pitch_rate_cmd = float(command.pitch_rate_cmd) if axis == "pitch" else 0.0
    return replace(
        command,
        yaw_rate_cmd=yaw_rate_cmd,
        pitch_rate_cmd=pitch_rate_cmd,
        active=not (
            math.isclose(yaw_rate_cmd, 0.0, abs_tol=1e-9)
            and math.isclose(pitch_rate_cmd, 0.0, abs_tol=1e-9)
        ),
    )


def main() -> int:
    parser = build_debug_arg_parser("Debug gimbal controller with direct rate tracking")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Log the rate command only and do not send commands to telemetry_link",
    )
    parser.add_argument(
        "--respect-control-gating",
        action="store_true",
        help="Use the full control mode gating from the main pipeline; default debug mode only requires valid vision, target, and gimbal feedback",
    )
    parser.add_argument(
        "--axis",
        choices=("both", "yaw", "pitch"),
        default="both",
        help="Limit emitted gimbal rate commands to one axis while tuning; default sends both axes",
    )
    parser.add_argument("--deadband", type=float, default=0.01)
    parser.add_argument("--center-hold-yaw-threshold", type=float, default=0.018)
    parser.add_argument("--center-hold-pitch-threshold", type=float, default=0.024)
    parser.add_argument("--pid-yaw-kp", type=float, default=3.2)
    parser.add_argument("--pid-yaw-ki", type=float, default=0.05)
    parser.add_argument("--pid-yaw-kd", type=float, default=0.18)
    parser.add_argument("--pid-pitch-kp", type=float, default=1.8)
    parser.add_argument("--pid-pitch-ki", type=float, default=0.03)
    parser.add_argument("--pid-pitch-kd", type=float, default=0.12)
    parser.add_argument("--pid-integral-limit", type=float, default=0.25)
    parser.add_argument("--pid-yaw-max-rate", type=float, default=3.3)
    parser.add_argument("--pid-pitch-max-rate", type=float, default=2.55)
    parser.add_argument("--pid-yaw-derivative-limit", type=float, default=1.5)
    parser.add_argument("--pid-pitch-derivative-limit", type=float, default=1.0)
    args = parser.parse_args()
    runtime = resolve_debug_runtime(args)
    logger = logging.getLogger("control.debug.gimbal")
    stop_event = threading.Event()
    install_signal_handlers(stop_event, logger)

    input_adapter = ControlInputAdapter(config=runtime.control_cfg.input_adapter)
    mode_manager = ControlModeManager(config=runtime.control_cfg.mode)
    controller = GimbalController(
        config=replace(
            runtime.control_cfg.gimbal,
            deadband_x=args.deadband,
            deadband_y=args.deadband,
            center_hold_yaw_threshold=args.center_hold_yaw_threshold,
            center_hold_pitch_threshold=args.center_hold_pitch_threshold,
            kp_yaw=args.pid_yaw_kp,
            ki_yaw=args.pid_yaw_ki,
            kd_yaw=args.pid_yaw_kd,
            kp_pitch=args.pid_pitch_kp,
            ki_pitch=args.pid_pitch_ki,
            kd_pitch=args.pid_pitch_kd,
            integral_limit=args.pid_integral_limit,
            derivative_limit_yaw=args.pid_yaw_derivative_limit,
            derivative_limit_pitch=args.pid_pitch_derivative_limit,
            max_yaw_rate=args.pid_yaw_max_rate,
            max_pitch_rate=args.pid_pitch_max_rate,
        )
    )
    yolo_receiver, link_manager, fusion_manager = create_live_inputs(runtime, stop_event)

    logger.info(
        "starting gimbal debug yolo_udp=%s:%s loop_hz=%.2f telemetry_source=%s dry_run=%s gating=%s axis=%s",
        runtime.yolo_udp_ip,
        runtime.yolo_udp_port,
        runtime.loop_hz,
        runtime.telemetry_cfg.active_source,
        args.dry_run,
        "full" if args.respect_control_gating else "debug",
        args.axis,
    )
    logger.info(
        "gimbal tune params yaw kp=%.3f ki=%.3f kd=%.3f max_rate=%.3f derivative_limit=%.3f "
        "pitch kp=%.3f ki=%.3f kd=%.3f max_rate=%.3f derivative_limit=%.3f "
        "deadband=%.3f center_hold_yaw=%.3f center_hold_pitch=%.3f integral_limit=%.3f",
        args.pid_yaw_kp,
        args.pid_yaw_ki,
        args.pid_yaw_kd,
        args.pid_yaw_max_rate,
        args.pid_yaw_derivative_limit,
        args.pid_pitch_kp,
        args.pid_pitch_ki,
        args.pid_pitch_kd,
        args.pid_pitch_max_rate,
        args.pid_pitch_derivative_limit,
        args.deadband,
        args.center_hold_yaw_threshold,
        args.center_hold_pitch_threshold,
        args.pid_integral_limit,
    )
    if args.dry_run:
        logger.warning("debug_gimbal is running in dry-run mode; commands are only logged")

    yolo_receiver.start()
    link_manager.start()
    loop_sleep_sec = loop_sleep(runtime.loop_hz)
    print_sleep_sec = print_sleep(runtime.print_rate_hz)
    last_print_time = 0.0
    last_sent_nonzero = False

    try:
        while not stop_event.is_set():
            now = time.time()
            control_input = read_control_input(
                now=now,
                yolo_receiver=yolo_receiver,
                link_manager=link_manager,
                fusion_manager=fusion_manager,
                input_adapter=input_adapter,
                perception_timeout_sec=runtime.perception_timeout_sec,
            )
            mode = mode_manager.update(control_input)
            enabled, hold_reason, mode_name = _enabled_for_debug(
                control_input,
                mode,
                args.respect_control_gating,
            )
            command = controller.update(
                control_input,
                task_mode=mode.task_mode,
                enabled=enabled,
            )
            command = _mask_command_axis(command, args.axis)

            if not args.dry_run:
                rate_sender = getattr(link_manager, "send_gimbal_rate", None)
                if not callable(rate_sender):
                    raise AttributeError(
                        "telemetry_link has no send_gimbal_rate(...) interface; cannot execute gimbal debug commands"
                    )
                if enabled:
                    rate_sender(
                        yaw_rate=float(command.yaw_rate_cmd),
                        pitch_rate=float(command.pitch_rate_cmd),
                    )
                    last_sent_nonzero = not (
                        math.isclose(float(command.yaw_rate_cmd), 0.0, abs_tol=1e-9)
                        and math.isclose(float(command.pitch_rate_cmd), 0.0, abs_tol=1e-9)
                    )
                elif last_sent_nonzero:
                    rate_sender(yaw_rate=0.0, pitch_rate=0.0)
                    last_sent_nonzero = False

            if (now - last_print_time) >= print_sleep_sec:
                logger.info(
                    "mode=%s hold=%s enabled=%s target_valid=%s vision_valid=%s gimbal_valid=%s track_id=%s "
                    "ex_cam=%.3f ey_cam=%.3f dt=%.3f gimbal_yaw=%.3f gimbal_pitch=%.3f "
                    "yaw_rate_cmd=%.3f pitch_rate_cmd=%.3f active=%s valid=%s",
                    mode_name,
                    hold_reason,
                    enabled,
                    control_input.target_valid,
                    control_input.vision_valid,
                    control_input.gimbal_valid,
                    control_input.track_id,
                    control_input.ex_cam,
                    control_input.ey_cam,
                    control_input.dt,
                    control_input.gimbal_yaw,
                    control_input.gimbal_pitch,
                    command.yaw_rate_cmd,
                    command.pitch_rate_cmd,
                    command.active,
                    command.valid,
                )
                last_print_time = now

            time.sleep(loop_sleep_sec)
    finally:
        stop_event.set()
        if not args.dry_run:
            rate_sender = getattr(link_manager, "send_gimbal_rate", None)
            if callable(rate_sender):
                rate_sender(yaw_rate=0.0, pitch_rate=0.0)
        yolo_receiver.close()
        link_manager.stop()
        logger.info("gimbal controller debug stopped")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
