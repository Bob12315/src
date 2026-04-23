from __future__ import annotations

import argparse
import logging
import signal
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[2]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from control.config import load_config as load_control_config
from control.main import YoloUdpReceiver, load_telemetry_config, setup_logging
from fusion.fusion_manager import FusionManager
from fusion.models import FusionConfig
from link_manager import LinkManager


@dataclass(slots=True)
class DebugRuntime:
    control_cfg: object
    telemetry_cfg: object
    yolo_udp_ip: str
    yolo_udp_port: int
    loop_hz: float
    perception_timeout_sec: float
    print_rate_hz: float
    require_gimbal_feedback: bool
    log_level: str


def build_debug_arg_parser(description: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument(
        "--control-config",
        default=str(ROOT_DIR / "control" / "config.yaml"),
        help="Path to control config.yaml",
    )
    parser.add_argument(
        "--telemetry-config",
        default=str(ROOT_DIR / "telemetry_link" / "config.yaml"),
        help="Path to telemetry_link config.yaml",
    )
    parser.add_argument("--yolo-udp-ip")
    parser.add_argument("--yolo-udp-port", type=int)
    parser.add_argument("--loop-hz", type=float)
    parser.add_argument("--perception-timeout-sec", type=float)
    parser.add_argument("--print-rate-hz", type=float)
    parser.add_argument(
        "--require-gimbal-feedback",
        type=lambda value: str(value).lower() in {"1", "true", "yes", "y", "on"},
    )
    parser.add_argument("--log-level")
    return parser


def resolve_debug_runtime(args: argparse.Namespace) -> DebugRuntime:
    control_cfg = load_control_config(["--config", args.control_config])
    telemetry_cfg = load_telemetry_config(args.telemetry_config)
    return DebugRuntime(
        control_cfg=control_cfg,
        telemetry_cfg=telemetry_cfg,
        yolo_udp_ip=args.yolo_udp_ip or control_cfg.runtime.yolo_udp_ip,
        yolo_udp_port=(
            args.yolo_udp_port if args.yolo_udp_port is not None else control_cfg.runtime.yolo_udp_port
        ),
        loop_hz=args.loop_hz if args.loop_hz is not None else control_cfg.runtime.loop_hz,
        perception_timeout_sec=(
            args.perception_timeout_sec
            if args.perception_timeout_sec is not None
            else control_cfg.runtime.perception_timeout_sec
        ),
        print_rate_hz=(
            args.print_rate_hz if args.print_rate_hz is not None else control_cfg.runtime.print_rate_hz
        ),
        require_gimbal_feedback=(
            args.require_gimbal_feedback
            if args.require_gimbal_feedback is not None
            else control_cfg.runtime.require_gimbal_feedback
        ),
        log_level=args.log_level or control_cfg.runtime.log_level,
    )


def install_signal_handlers(stop_event: threading.Event, logger: logging.Logger) -> None:
    def _handle_signal(signum, _frame) -> None:
        logger.info("received signal %s, shutting down", signum)
        stop_event.set()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)


def create_live_inputs(runtime: DebugRuntime, stop_event: threading.Event):
    setup_logging(runtime.log_level)
    yolo_receiver = YoloUdpReceiver(runtime.yolo_udp_ip, runtime.yolo_udp_port, stop_event)
    link_manager = LinkManager(runtime.telemetry_cfg)
    fusion_manager = FusionManager(
        FusionConfig(require_gimbal_feedback=bool(runtime.require_gimbal_feedback))
    )
    return yolo_receiver, link_manager, fusion_manager


def read_control_input(
    *,
    now: float,
    yolo_receiver: YoloUdpReceiver,
    link_manager: LinkManager,
    fusion_manager: FusionManager,
    input_adapter,
    perception_timeout_sec: float,
):
    perception = yolo_receiver.get_latest_target(now, perception_timeout_sec)
    drone = link_manager.get_latest_drone_state()
    gimbal = link_manager.get_latest_gimbal_state()
    fused = fusion_manager.update(perception, drone, gimbal)
    return input_adapter.adapt(fused)


def loop_sleep(loop_hz: float) -> float:
    return 1.0 / max(loop_hz, 0.1)


def print_sleep(print_rate_hz: float) -> float:
    return 1.0 / max(print_rate_hz, 0.1)
