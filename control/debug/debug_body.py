from __future__ import annotations

import logging
import sys
import threading
import time
from pathlib import Path

if __package__ in {None, ""}:
    ROOT_DIR = Path(__file__).resolve().parents[2]
    if str(ROOT_DIR) not in sys.path:
        sys.path.insert(0, str(ROOT_DIR))
    from control.body_controller import BodyController
    from control.control_input import ControlInputAdapter
    from control.control_mode import ControlModeManager
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
    from control.body_controller import BodyController
    from control.control_input import ControlInputAdapter
    from control.control_mode import ControlModeManager
    from control.debug.common import (
        build_debug_arg_parser,
        create_live_inputs,
        install_signal_handlers,
        loop_sleep,
        print_sleep,
        read_control_input,
        resolve_debug_runtime,
    )


def main() -> int:
    args = build_debug_arg_parser("Debug body controller only").parse_args()
    runtime = resolve_debug_runtime(args)
    logger = logging.getLogger("control.debug.body")
    stop_event = threading.Event()
    install_signal_handlers(stop_event, logger)

    input_adapter = ControlInputAdapter(config=runtime.control_cfg.input_adapter)
    mode_manager = ControlModeManager(config=runtime.control_cfg.mode)
    controller = BodyController(config=runtime.control_cfg.body)
    yolo_receiver, link_manager, fusion_manager = create_live_inputs(runtime, stop_event)

    logger.info(
        "starting body controller debug yolo_udp=%s:%s loop_hz=%.2f telemetry_source=%s",
        runtime.yolo_udp_ip,
        runtime.yolo_udp_port,
        runtime.loop_hz,
        runtime.telemetry_cfg.active_source,
    )

    yolo_receiver.start()
    link_manager.start()
    loop_sleep_sec = loop_sleep(runtime.loop_hz)
    print_sleep_sec = print_sleep(runtime.print_rate_hz)
    last_print_time = 0.0

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
            command = controller.update(
                control_input,
                task_mode=mode.task_mode,
                enabled=mode.enable_body,
            )

            if (now - last_print_time) >= print_sleep_sec:
                logger.info(
                    "mode=%s task_mode=%s hold=%s enabled=%s target_valid=%s track_id=%s "
                    "ex_body=%.3f ex_cam=%.3f gimbal_yaw=%.3f "
                    "vy_cmd=%.3f yaw_rate_cmd=%.3f active=%s valid=%s",
                    mode.mode_name,
                    mode.task_mode,
                    mode.hold_reason,
                    mode.enable_body,
                    control_input.target_valid,
                    control_input.track_id,
                    control_input.ex_body,
                    control_input.ex_cam,
                    control_input.gimbal_yaw,
                    command.vy_cmd,
                    command.yaw_rate_cmd,
                    command.active,
                    command.valid,
                )
                last_print_time = now

            time.sleep(loop_sleep_sec)
    finally:
        stop_event.set()
        yolo_receiver.close()
        link_manager.stop()
        logger.info("body controller debug stopped")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
