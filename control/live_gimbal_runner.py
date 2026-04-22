from __future__ import annotations

import argparse
import json
import logging
import signal
import socket
import sys
import threading
import time
from dataclasses import replace
from pathlib import Path
from typing import Any

import yaml

ROOT_DIR = Path(__file__).resolve().parent.parent
TELEMETRY_DIR = ROOT_DIR / "telemetry_link"
for path in (str(ROOT_DIR), str(TELEMETRY_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

from control.approach_controller import ApproachController
from control.body_controller import BodyController
from control.command_shaper import CommandShaper
from control.control_executor import ControlExecutor
from control.control_input import ControlInputAdapter
from control.control_mode import ControlModeManager
from control.gimbal_controller import GimbalController
from fusion.fusion_manager import FusionManager
from fusion.models import FusionConfig, PerceptionTarget
from link_manager import LinkManager
from models import DroneState, GimbalState
from telemetry_link.config import EndpointConfig, TelemetryConfig


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Live gimbal-only control runner")
    parser.add_argument(
        "--telemetry-config",
        default=str(ROOT_DIR / "telemetry_link" / "config.yaml"),
        help="Path to telemetry_link config.yaml",
    )
    parser.add_argument("--yolo-udp-ip", default="0.0.0.0")
    parser.add_argument("--yolo-udp-port", type=int, default=5005)
    parser.add_argument("--loop-hz", type=float, default=20.0)
    parser.add_argument("--perception-timeout-sec", type=float, default=1.0)
    parser.add_argument("--print-rate-hz", type=float, default=2.0)
    parser.add_argument("--require-gimbal-feedback", action="store_true", default=True)
    parser.add_argument("--log-level", default="INFO")
    return parser


def setup_logging(level: str) -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )


def load_telemetry_config(path: str) -> TelemetryConfig:
    with open(path, "r", encoding="utf-8") as handle:
        merged = yaml.safe_load(handle) or {}
    if not isinstance(merged, dict):
        raise ValueError("telemetry config yaml must be a mapping")

    def build_endpoint(name: str, data: dict[str, Any]) -> EndpointConfig:
        return EndpointConfig(
            name=name,
            connection_type=str(data["connection_type"]),
            serial_port=str(data.get("serial_port", "/dev/ttyUSB0")),
            baudrate=int(data.get("baudrate", 115200)),
            udp_mode=str(data.get("udp_mode", "udpin")),
            udp_host=str(data.get("udp_host", "0.0.0.0")),
            udp_port=int(data.get("udp_port", 14550)),
            tcp_host=str(data.get("tcp_host", "127.0.0.1")),
            tcp_port=int(data.get("tcp_port", 5760)),
        )

    return TelemetryConfig(
        data_source=str(merged["data_source"]),
        active_source=str(merged["active_source"]),
        sitl=build_endpoint("sitl", dict(merged["sitl"])),
        real=build_endpoint("real", dict(merged["real"])),
        control_send_rate_hz=float(merged["control_send_rate_hz"]),
        action_cmd_retries=int(merged["action_cmd_retries"]),
        action_retry_interval_sec=float(merged["action_retry_interval_sec"]),
        heartbeat_timeout_sec=float(merged["heartbeat_timeout_sec"]),
        rx_timeout_sec=float(merged["rx_timeout_sec"]),
        reconnect_interval_sec=float(merged["reconnect_interval_sec"]),
        receiver_idle_sleep_sec=float(merged["receiver_idle_sleep_sec"]),
        sender_idle_sleep_sec=float(merged["sender_idle_sleep_sec"]),
        request_message_intervals=bool(merged["request_message_intervals"]),
        message_interval_hz={str(k): float(v) for k, v in dict(merged.get("message_interval_hz", {})).items()},
        gimbal_mount_mode=int(merged.get("gimbal_mount_mode", 2)),
        state_udp_enabled=bool(merged.get("state_udp_enabled", True)),
        state_udp_ip=str(merged.get("state_udp_ip", "127.0.0.1")),
        state_udp_port=int(merged.get("state_udp_port", 5010)),
        log_level=str(merged.get("log_level", "INFO")),
    )


class YoloUdpReceiver(threading.Thread):
    def __init__(self, ip: str, port: int, stop_event: threading.Event) -> None:
        super().__init__(name="ControlYoloUdpReceiver", daemon=True)
        self.stop_event = stop_event
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, port))
        self.sock.settimeout(0.2)
        self._lock = threading.Lock()
        self._latest_target = PerceptionTarget()
        self._last_packet_time = 0.0
        self.logger = logging.getLogger(self.__class__.__name__)

    def run(self) -> None:
        while not self.stop_event.is_set():
            try:
                payload, _addr = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                data = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                self.logger.warning("drop invalid YOLO UDP payload: %s", exc)
                continue
            if not isinstance(data, dict):
                self.logger.warning("drop YOLO UDP payload because it is not a JSON object")
                continue
            target = self._decode_target(data)
            with self._lock:
                self._latest_target = target
                self._last_packet_time = time.time()

    def get_latest_target(self, now: float, timeout_sec: float) -> PerceptionTarget:
        with self._lock:
            target = replace(self._latest_target)
            last_packet_time = self._last_packet_time
        if last_packet_time <= 0 or (now - last_packet_time) > timeout_sec:
            target.target_valid = False
            target.tracking_state = "lost"
            target.ex = 0.0
            target.ey = 0.0
        return target

    def close(self) -> None:
        self.sock.close()

    @staticmethod
    def _decode_target(data: dict[str, Any]) -> PerceptionTarget:
        return PerceptionTarget(
            timestamp=float(data.get("timestamp", 0.0)),
            frame_id=int(data.get("frame_id", 0)),
            target_valid=bool(data.get("target_valid", False)),
            tracking_state=str(data.get("tracking_state", "lost")),
            track_id=int(data.get("track_id", -1)),
            class_name=str(data.get("class_name", "")),
            confidence=float(data.get("confidence", 0.0)),
            cx=float(data.get("cx", 0.0)),
            cy=float(data.get("cy", 0.0)),
            w=float(data.get("w", 0.0)),
            h=float(data.get("h", 0.0)),
            ex=float(data.get("ex", 0.0)),
            ey=float(data.get("ey", 0.0)),
            lost_count=int(data.get("lost_count", 0)),
        )


def main() -> int:
    args = build_arg_parser().parse_args()
    setup_logging(args.log_level)
    logger = logging.getLogger("control_live_gimbal")
    stop_event = threading.Event()

    def _handle_signal(signum, _frame) -> None:
        logger.info("received signal %s, shutting down", signum)
        stop_event.set()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    telemetry_cfg = load_telemetry_config(args.telemetry_config)
    yolo_receiver = YoloUdpReceiver(args.yolo_udp_ip, args.yolo_udp_port, stop_event)
    link_manager = LinkManager(telemetry_cfg)
    fusion_manager = FusionManager(
        FusionConfig(require_gimbal_feedback=bool(args.require_gimbal_feedback))
    )
    input_adapter = ControlInputAdapter()
    mode_manager = ControlModeManager()
    gimbal_controller = GimbalController()
    body_controller = BodyController()
    approach_controller = ApproachController()
    command_shaper = CommandShaper()
    control_executor = ControlExecutor(telemetry_link=link_manager)

    logger.info(
        "starting live gimbal runner yolo_udp=%s:%s loop_hz=%.2f telemetry_source=%s",
        args.yolo_udp_ip,
        args.yolo_udp_port,
        args.loop_hz,
        telemetry_cfg.active_source,
    )

    yolo_receiver.start()
    link_manager.start()
    loop_sleep_sec = 1.0 / max(args.loop_hz, 0.1)
    print_sleep_sec = 1.0 / max(args.print_rate_hz, 0.1)
    last_print_time = 0.0

    try:
        while not stop_event.is_set():
            now = time.time()
            perception = yolo_receiver.get_latest_target(now, args.perception_timeout_sec)
            drone = link_manager.get_latest_drone_state()
            gimbal = link_manager.get_latest_gimbal_state()
            fused = fusion_manager.update(perception, drone, gimbal)
            control_input = input_adapter.adapt(fused)
            mode = mode_manager.update(control_input)
            gimbal_cmd = gimbal_controller.update(
                control_input,
                task_mode=mode.task_mode,
                enabled=mode.enable_gimbal,
            )
            body_cmd = body_controller.update(
                control_input,
                task_mode=mode.task_mode,
                enabled=mode.enable_body,
            )
            approach_cmd = approach_controller.update(
                control_input,
                task_mode=mode.task_mode,
                enabled=mode.enable_approach,
            )
            shaped = command_shaper.update(
                mode=mode,
                gimbal_cmd=gimbal_cmd,
                body_cmd=body_cmd,
                approach_cmd=approach_cmd,
                dt=control_input.dt,
            )
            control_executor.execute(shaped)

            if (now - last_print_time) >= print_sleep_sec:
                logger.info(
                    "mode=%s task_mode=%s hold=%s control_allowed=%s target_valid=%s track_id=%s "
                    "ex=%.3f ey=%.3f gimbal_yaw=%.3f gimbal_pitch=%.3f "
                    "gimbal_rate=(%.3f,%.3f) shaped_rate=(%.3f,%.3f)",
                    mode.mode_name,
                    mode.task_mode,
                    mode.hold_reason,
                    control_input.control_allowed,
                    control_input.target_valid,
                    control_input.track_id,
                    control_input.ex_cam,
                    control_input.ey_cam,
                    control_input.gimbal_yaw,
                    control_input.gimbal_pitch,
                    gimbal_cmd.yaw_rate_cmd,
                    gimbal_cmd.pitch_rate_cmd,
                    shaped.gimbal_yaw_rate_cmd,
                    shaped.gimbal_pitch_rate_cmd,
                )
                last_print_time = now

            time.sleep(loop_sleep_sec)
    finally:
        stop_event.set()
        control_executor.reset()
        yolo_receiver.close()
        link_manager.stop()
        logger.info("live gimbal runner stopped")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
