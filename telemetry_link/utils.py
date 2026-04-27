from __future__ import annotations

import logging


def build_connection_string(connection_type: str, serial_port: str, udp_mode: str, udp_host: str, udp_port: int) -> str:
    if connection_type == "serial":
        return serial_port
    return f"{udp_mode}:{udp_host}:{udp_port}"


def setup_logging(level: str, log_file: str | None = None) -> None:
    kwargs = {
        "level": getattr(logging, level.upper(), logging.INFO),
        "format": "%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    }
    if log_file is not None:
        kwargs["filename"] = log_file
    logging.basicConfig(**kwargs)
