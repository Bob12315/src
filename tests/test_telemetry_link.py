from __future__ import annotations

import logging
import threading
import time

import pytest

from telemetry_link.command_queue import CommandQueue
from telemetry_link.config import EndpointConfig, TelemetryConfig, load_config
from telemetry_link.link_manager import LinkManager, SourceRuntime
from telemetry_link.models import ControlCommand, ControlType, GimbalRateCommand
from telemetry_link.state_cache import StateCache


def _endpoint(name: str) -> EndpointConfig:
    return EndpointConfig(
        name=name,
        connection_type="tcp",
        serial_port="/dev/null",
        baudrate=115200,
        udp_mode="udpin",
        udp_host="127.0.0.1",
        udp_port=14550,
        tcp_host="127.0.0.1",
        tcp_port=5762,
    )


def _config(**overrides) -> TelemetryConfig:
    data = dict(
        data_source="dual",
        active_source="sitl",
        sitl=_endpoint("sitl"),
        real=_endpoint("real"),
        control_send_rate_hz=10.0,
        action_cmd_retries=0,
        action_retry_interval_sec=0.01,
        heartbeat_timeout_sec=0.05,
        rx_timeout_sec=0.05,
        reconnect_interval_sec=0.01,
        receiver_idle_sleep_sec=0.01,
        sender_idle_sleep_sec=0.01,
        request_message_intervals=False,
        message_interval_hz={},
        gimbal_mount_mode=2,
        gimbal_yaw_min_deg=-180.0,
        gimbal_yaw_max_deg=180.0,
        gimbal_pitch_min_deg=-180.0,
        gimbal_pitch_max_deg=180.0,
        state_udp_enabled=False,
        state_udp_ip="127.0.0.1",
        state_udp_port=5010,
        ui_enabled=False,
        log_level="INFO",
    )
    data.update(overrides)
    return TelemetryConfig(**data)


def test_switch_active_source_clears_inactive_continuous_queues() -> None:
    manager = LinkManager(_config())
    manager.runtimes["sitl"].command_queue.put_control(
        ControlCommand(command_type=ControlType.VELOCITY, vx=1.0)
    )
    manager.runtimes["sitl"].command_queue.put_gimbal_rate(GimbalRateCommand(yaw_rate=0.2))

    assert manager.switch_active_source("real") is True

    assert manager.get_active_source() == "real"
    assert manager.runtimes["sitl"].command_queue.peek_control() is None
    assert manager.runtimes["sitl"].command_queue.peek_gimbal_rate() is None


class _FailingClient:
    connection_string = "fake"
    is_sitl = True
    target_system = 0
    target_component = 0

    def __init__(self) -> None:
        self.connect_calls = 0
        self.closed = False

    def connect(self) -> None:
        self.connect_calls += 1

    def wait_heartbeat(self, timeout: float) -> None:
        raise TimeoutError("no heartbeat")

    def close(self) -> None:
        self.closed = True


def test_source_runtime_start_returns_while_connect_retries_in_background() -> None:
    client = _FailingClient()
    runtime = SourceRuntime(
        name="sitl",
        endpoint=_endpoint("sitl"),
        cfg=_config(data_source="sitl", active_source="sitl"),
        state_cache=StateCache(heartbeat_timeout_sec=0.05, rx_timeout_sec=0.05),
        command_queue=CommandQueue(),
        client=client,
        stop_event=threading.Event(),
        worker_stop_event=threading.Event(),
    )

    started_at = time.monotonic()
    runtime.start(logging.getLogger("test"))
    elapsed = time.monotonic() - started_at

    try:
        assert elapsed < 0.05
        assert runtime.monitor_thread is not None
        assert runtime.monitor_thread.is_alive()
    finally:
        runtime.stop()


def test_load_config_parses_quoted_false_as_false(tmp_path, monkeypatch) -> None:
    path = tmp_path / "telemetry.yaml"
    path.write_text(
        """
data_source: sitl
active_source: sitl
sitl:
  connection_type: tcp
real:
  connection_type: tcp
control_send_rate_hz: 10
action_cmd_retries: 0
action_retry_interval_sec: 0.01
heartbeat_timeout_sec: 0.05
rx_timeout_sec: 0.05
reconnect_interval_sec: 0.01
receiver_idle_sleep_sec: 0.01
sender_idle_sleep_sec: 0.01
request_message_intervals: "false"
message_interval_hz: {}
state_udp_enabled: "false"
ui_enabled: "false"
log_level: INFO
""".lstrip(),
        encoding="utf-8",
    )
    monkeypatch.setattr("sys.argv", ["telemetry_link.main", "--config", str(path)])

    cfg = load_config()

    assert cfg.request_message_intervals is False
    assert cfg.state_udp_enabled is False
    assert cfg.ui_enabled is False


def test_load_config_rejects_invalid_bool_string(tmp_path, monkeypatch) -> None:
    path = tmp_path / "telemetry.yaml"
    path.write_text(
        """
data_source: sitl
active_source: sitl
sitl:
  connection_type: tcp
real:
  connection_type: tcp
control_send_rate_hz: 10
action_cmd_retries: 0
action_retry_interval_sec: 0.01
heartbeat_timeout_sec: 0.05
rx_timeout_sec: 0.05
reconnect_interval_sec: 0.01
receiver_idle_sleep_sec: 0.01
sender_idle_sleep_sec: 0.01
request_message_intervals: maybe
message_interval_hz: {}
state_udp_enabled: false
ui_enabled: false
log_level: INFO
""".lstrip(),
        encoding="utf-8",
    )
    monkeypatch.setattr("sys.argv", ["telemetry_link.main", "--config", str(path)])

    with pytest.raises(Exception, match="invalid bool value"):
        load_config()
