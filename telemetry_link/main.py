from __future__ import annotations

import logging
import signal
import threading
import time

from config import load_config
from link_manager import LinkManager
from utils import setup_logging


def main() -> int:
    cfg = load_config()
    setup_logging(cfg.log_level)
    logger = logging.getLogger("telemetry_link")
    manager = LinkManager(cfg)
    stop_event = threading.Event()

    def _handle_signal(signum, frame) -> None:
        logger.info("received signal %s, shutting down", signum)
        stop_event.set()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    while not stop_event.is_set():
        try:
            logger.info("starting telemetry link service")
            manager.start()
            break
        except Exception as exc:
            logger.warning("start failed: %s; retry in %.1fs", exc, cfg.reconnect_interval_sec)
            time.sleep(cfg.reconnect_interval_sec)

    try:
        while not stop_event.is_set():
            state = manager.get_latest_state()
            link = manager.get_link_status()
            logger.info(
                "link=%s reconnecting=%s connected=%s stale=%s mode=%s control_allowed=%s armed=%s\n"
                "att_valid=%s rpy=(%.2f,%.2f,%.2f) rates=(%.2f,%.2f,%.2f)\n"
                "alt_valid=%s alt=%.2f rel_alt=%.2f\n"
                "vel_valid=%s vel=(%.2f,%.2f,%.2f) vel_src=%s vel_q=%s\n"
                "global_valid=%s pos=(%.7f,%.7f)\n"
                "batt_valid=%s batt=%.2fV batt_rem=%s gps_fix=%s sats=%s\n"
                "hb_age=%.2fs rx_age=%.2fs",
                link.status_text,
                link.reconnecting,
                state.connected,
                state.stale,
                state.mode,
                state.control_allowed,
                state.armed,
                state.attitude_valid,
                state.roll,
                state.pitch,
                state.yaw,
                state.roll_rate,
                state.pitch_rate,
                state.yaw_rate,
                state.altitude_valid,
                state.altitude,
                state.relative_altitude,
                state.velocity_valid,
                state.vx,
                state.vy,
                state.vz,
                state.velocity_source,
                state.velocity_quality,
                state.global_position_valid,
                state.lat,
                state.lon,
                state.battery_valid,
                state.battery_voltage,
                state.battery_remaining,
                state.gps_fix_type,
                state.satellites_visible,
                state.hb_age_sec,
                state.rx_age_sec,
            )
            time.sleep(1.0)
    finally:
        manager.stop()
        logger.info("telemetry link service stopped")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
