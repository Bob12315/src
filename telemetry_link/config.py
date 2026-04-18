from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(slots=True)
class TelemetryConfig:
    connection_type: str
    serial_port: str
    baudrate: int
    udp_mode: str
    udp_host: str
    udp_port: int
    control_send_rate_hz: float
    action_cmd_retries: int
    action_retry_interval_sec: float
    heartbeat_timeout_sec: float
    rx_timeout_sec: float
    reconnect_interval_sec: float
    receiver_idle_sleep_sec: float
    sender_idle_sleep_sec: float
    request_message_intervals: bool
    message_interval_hz: dict[str, float]
    log_level: str


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
    parser = argparse.ArgumentParser(description="Standalone MAVLink telemetry link service")
    parser.add_argument("--config", default=str(Path(__file__).with_name("config.yaml")))
    parser.add_argument("--connection-type", choices=["serial", "udp"])
    parser.add_argument("--serial-port")
    parser.add_argument("--baudrate", type=int)
    parser.add_argument("--udp-mode", choices=["udpin", "udpout"])
    parser.add_argument("--udp-host")
    parser.add_argument("--udp-port", type=int)
    parser.add_argument("--control-send-rate-hz", type=float)
    parser.add_argument("--action-cmd-retries", type=int)
    parser.add_argument("--action-retry-interval-sec", type=float)
    parser.add_argument("--heartbeat-timeout-sec", type=float)
    parser.add_argument("--rx-timeout-sec", type=float)
    parser.add_argument("--reconnect-interval-sec", type=float)
    parser.add_argument("--receiver-idle-sleep-sec", type=float)
    parser.add_argument("--sender-idle-sleep-sec", type=float)
    parser.add_argument("--request-message-intervals", type=_to_bool)
    parser.add_argument("--log-level")
    return parser


def _load_yaml(path: str) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError("config yaml must be a mapping")
    return data


def load_config() -> TelemetryConfig:
    parser = build_arg_parser()
    args = parser.parse_args()
    merged = _load_yaml(args.config)

    for key, value in vars(args).items():
        if key == "config":
            continue
        if value is not None:
            merged[key.replace("-", "_")] = value

    return TelemetryConfig(
        connection_type=str(merged["connection_type"]),
        serial_port=str(merged["serial_port"]),
        baudrate=int(merged["baudrate"]),
        udp_mode=str(merged["udp_mode"]),
        udp_host=str(merged["udp_host"]),
        udp_port=int(merged["udp_port"]),
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
        log_level=str(merged["log_level"]),
    )
