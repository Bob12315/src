from __future__ import annotations

from pymavlink import mavutil


def heartbeat_is_armed(base_mode: int) -> bool:
    return bool(int(base_mode) & 128)


def decode_copter_mode(custom_mode: int) -> str:
    mode_map = {
        0: "STABILIZE",
        1: "ACRO",
        2: "ALT_HOLD",
        3: "AUTO",
        4: "GUIDED",
        5: "LOITER",
        6: "RTL",
        7: "CIRCLE",
        9: "LAND",
        17: "BRAKE",
        20: "GUIDED_NOGPS",
        21: "SMART_RTL",
    }
    return mode_map.get(int(custom_mode), f"MODE_{int(custom_mode)}")


def parse_sys_status_values(
    voltage_mv: int,
    current_centiamp: int,
    remaining_percent: int,
) -> tuple[float, float, float]:
    voltage_v = 0.0 if int(voltage_mv) == 0xFFFF else float(voltage_mv) / 1000.0
    current_a = float("nan") if int(current_centiamp) == -1 else float(current_centiamp) / 100.0
    percentage = float("nan") if int(remaining_percent) == -1 else float(remaining_percent) / 100.0
    return voltage_v, current_a, percentage


def control_allowed_for_mode(mode_name: str) -> bool:
    allowed_modes = {
        "GUIDED",
        "GUIDED_NOGPS",
        "OFFBOARD",
    }
    return str(mode_name).strip().upper() in allowed_modes


def global_position_is_valid(lat: float, lon: float, gps_fix_type: int) -> bool:
    if abs(float(lat)) < 1e-9 and abs(float(lon)) < 1e-9:
        return False
    return int(gps_fix_type) >= 3
