from __future__ import annotations

import logging
from typing import Any, Callable

from pymavlink import mavutil

from config import EndpointConfig


def open_mavlink_connection(url: str, baud: int | None = None):
    """Open a MAVLink connection, adding baud only for local serial devices."""
    if url.startswith("/dev/"):
        if baud is None:
            return mavutil.mavlink_connection(url)
        return mavutil.mavlink_connection(url, baud=int(baud))
    return mavutil.mavlink_connection(url)


class MavlinkClient:
    def __init__(self, endpoint: EndpointConfig) -> None:
        self.endpoint = endpoint
        self.logger = logging.getLogger(self.__class__.__name__)
        self.master: Any | None = None
        self.target_system = 0
        self.target_component = 0
        self.connection_string = ""
        self.is_sitl = endpoint.name == "sitl"

    def connect(self) -> None:
        if self.endpoint.connection_type == "serial":
            connection_string = self.endpoint.serial_port
            baud = self.endpoint.baudrate
        elif self.endpoint.connection_type == "udp":
            connection_string = f"{self.endpoint.udp_mode}:{self.endpoint.udp_host}:{self.endpoint.udp_port}"
            baud = None
        elif self.endpoint.connection_type == "tcp":
            connection_string = f"tcp:{self.endpoint.tcp_host}:{self.endpoint.tcp_port}"
            baud = None
        else:
            raise ValueError(f"unsupported connection_type: {self.endpoint.connection_type}")

        self.connection_string = connection_string
        self.logger.info(
            "source=%s connection_type=%s endpoint=%s sitl_mode=%s",
            self.endpoint.name,
            self.endpoint.connection_type,
            connection_string,
            self.is_sitl,
        )
        self.master = open_mavlink_connection(connection_string, baud=baud)

    def wait_heartbeat(self, timeout: float = 10.0) -> None:
        if self.master is None:
            raise RuntimeError("MAVLink client is not connected")
        self.logger.info("waiting heartbeat...")
        self.master.wait_heartbeat(timeout=timeout)
        self.target_system = int(self.master.target_system)
        self.target_component = int(self.master.target_component)
        self.logger.info(
            "heartbeat received target_system=%s target_component=%s",
            self.target_system,
            self.target_component,
        )

    def recv_message(self, timeout: float = 0.1):
        if self.master is None:
            raise RuntimeError("MAVLink client is not connected")
        return self.master.recv_match(blocking=True, timeout=timeout)

    def send_raw_message(self, sender: Callable[[Any], None]) -> None:
        if self.master is None:
            raise RuntimeError("MAVLink client is not connected")
        sender(self.master)

    def close(self) -> None:
        if self.master is not None and hasattr(self.master, "close"):
            self.master.close()
        self.master = None
