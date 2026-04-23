from __future__ import annotations

import logging
import math
import signal
import sys
import threading
import time

from pymavlink import mavutil

from config import load_config
from link_manager import LinkManager
from state_publisher import StatePublisher
from utils import setup_logging


def main() -> int:
    cfg = load_config()
    setup_logging(cfg.log_level)
    logger = logging.getLogger("telemetry_link")
    manager = LinkManager(cfg)
    stop_event = threading.Event()
    state_publisher = StatePublisher(cfg.state_udp_ip, cfg.state_udp_port) if cfg.state_udp_enabled else None

    def _stdin_command_loop() -> None:
        while not stop_event.is_set():
            line = sys.stdin.readline()
            if not line:
                time.sleep(0.1)
                continue
            command = line.strip()
            if command.startswith("switch_source "):
                source_name = command.split(maxsplit=1)[1].strip()
                ok = manager.switch_active_source(source_name)
                if ok:
                    logger.info("switch_source command applied active_source=%s", source_name)
                else:
                    logger.warning("switch_source command rejected source=%s", source_name)
            elif command.startswith("mode "):
                mode_name = command.split(maxsplit=1)[1].strip()
                if not mode_name:
                    logger.warning("mode command format: mode <MODE_NAME>")
                    continue
                manager.set_mode(mode_name)
                logger.info("mode command queued mode=%s", mode_name)
            elif command == "arm":
                manager.arm()
                logger.info("arm command queued")
            elif command == "disarm":
                manager.disarm()
                logger.info("disarm command queued")
            elif command.startswith("takeoff "):
                parts = command.split()
                if len(parts) != 2:
                    logger.warning("takeoff command format: takeoff <altitude_m>")
                    continue
                try:
                    altitude_m = float(parts[1])
                except ValueError:
                    logger.warning("takeoff command parse failed: %s", command)
                    continue
                manager.takeoff(altitude_m)
                logger.info("takeoff command queued altitude_m=%.2f", altitude_m)
            elif command == "land":
                manager.land()
                logger.info("land command queued")
            elif command.startswith("body_vel "):
                parts = command.split()
                if len(parts) != 4:
                    logger.warning("body_vel command format: body_vel <forward_mps> <right_mps> <down_mps>")
                    continue
                try:
                    forward = float(parts[1])
                    right = float(parts[2])
                    down = float(parts[3])
                except ValueError:
                    logger.warning("body_vel command parse failed: %s", command)
                    continue
                manager.send_velocity_command(
                    vx=forward,
                    vy=right,
                    vz=down,
                    frame=mavutil.mavlink.MAV_FRAME_BODY_NED,
                )
                logger.info(
                    "body_vel command queued forward=%.2f right=%.2f down=%.2f frame=BODY_NED",
                    forward,
                    right,
                    down,
                )
            elif command.startswith("yaw_rate "):
                parts = command.split()
                if len(parts) != 2:
                    logger.warning("yaw_rate command format: yaw_rate <rad_per_sec>")
                    continue
                try:
                    yaw_rate = float(parts[1])
                except ValueError:
                    logger.warning("yaw_rate command parse failed: %s", command)
                    continue
                manager.send_yaw_rate_command(
                    yaw_rate=yaw_rate,
                    frame=mavutil.mavlink.MAV_FRAME_BODY_NED,
                )
                logger.info(
                    "yaw_rate command queued yaw_rate=%.2f frame=BODY_NED",
                    yaw_rate,
                )
            elif command == "stop":
                manager.stop_control(frame=mavutil.mavlink.MAV_FRAME_BODY_NED)
                logger.info("stop command queued frame=BODY_NED")
            elif command.startswith("gimbal "):
                parts = command.split()
                if len(parts) not in {3, 4}:
                    logger.warning("gimbal command format: gimbal <pitch_deg> <yaw_deg> [roll_deg]")
                    continue
                try:
                    pitch = float(parts[1])
                    yaw = float(parts[2])
                    roll = float(parts[3]) if len(parts) == 4 else 0.0
                except ValueError:
                    logger.warning("gimbal command parse failed: %s", command)
                    continue
                manager.send_gimbal_angle(pitch=pitch, yaw=yaw, roll=roll)
                logger.info(
                    "gimbal command queued pitch=%.2f yaw=%.2f roll=%.2f",
                    pitch,
                    yaw,
                    roll,
                )
            elif command.startswith("gimbal_rate "):
                parts = command.split()
                if len(parts) not in {3, 4}:
                    logger.warning(
                        "gimbal_rate command format: gimbal_rate <pitch_rate_deg_s> <yaw_rate_deg_s> [follow|lock]"
                    )
                    continue
                try:
                    pitch_rate_deg_s = float(parts[1])
                    yaw_rate_deg_s = float(parts[2])
                except ValueError:
                    logger.warning("gimbal_rate command parse failed: %s", command)
                    continue
                yaw_mode = parts[3].strip().lower() if len(parts) == 4 else "follow"
                if yaw_mode not in {"follow", "lock"}:
                    logger.warning("gimbal_rate yaw mode must be follow or lock: %s", yaw_mode)
                    continue
                manager.send_gimbal_rate(
                    yaw_rate=math.radians(yaw_rate_deg_s),
                    pitch_rate=math.radians(pitch_rate_deg_s),
                    yaw_lock=(yaw_mode == "lock"),
                )
                logger.info(
                    "gimbal_rate command queued pitch_rate=%.2f deg/s yaw_rate=%.2f deg/s yaw_mode=%s",
                    pitch_rate_deg_s,
                    yaw_rate_deg_s,
                    yaw_mode,
                )
            elif command:
                logger.warning("unknown command: %s", command)

    def _handle_signal(signum, frame) -> None:
        logger.info("received signal %s, shutting down", signum)
        stop_event.set()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    while not stop_event.is_set():
        try:
            logger.info(
                "starting telemetry link service data_source=%s active_source=%s",
                cfg.data_source,
                cfg.active_source,
            )
            manager.start()
            break
        except Exception as exc:
            logger.warning("start failed: %s; retry in %.1fs", exc, cfg.reconnect_interval_sec)
            time.sleep(cfg.reconnect_interval_sec)

    command_thread = threading.Thread(name="StdinCommandLoop", target=_stdin_command_loop, daemon=True)
    command_thread.start()

    try:
        while not stop_event.is_set():
            state = manager.get_latest_drone_state()
            gimbal = manager.get_latest_gimbal_state()
            link = manager.get_link_status()
            active_source = manager.get_active_source()
            if state_publisher is not None:
                state_publisher.publish(state, gimbal, link, active_source)
            logger.info(
                "source=%s active_source=%s link=%s reconnecting=%s connected=%s stale=%s mode=%s control_allowed=%s armed=%s\n"
                "att_valid=%s rpy=(%.2f,%.2f,%.2f) rates=(%.2f,%.2f,%.2f)\n"
                "alt_valid=%s alt=%.2f rel_alt=%.2f\n"
                "vel_valid=%s vel=(%.2f,%.2f,%.2f) vel_src=%s vel_q=%s\n"
                "global_valid=%s pos=(%.7f,%.7f)\n"
                "batt_valid=%s batt=%.2fV batt_rem=%s gps_fix=%s sats=%s\n"
                "gimbal_valid=%s gimbal_rpy=(%.2f,%.2f,%.2f)\n"
                "hb_age=%.2fs rx_age=%.2fs",
                active_source,
                active_source,
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
                gimbal.gimbal_valid,
                gimbal.roll,
                gimbal.pitch,
                gimbal.yaw,
                state.hb_age_sec,
                state.rx_age_sec,
            )
            time.sleep(1.0)
    finally:
        if state_publisher is not None:
            state_publisher.close()
        manager.stop()
        logger.info("telemetry link service stopped")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
