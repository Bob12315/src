from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(slots=True)
class AppConfig:
    model_path: str
    source: str
    img_size: int
    conf_thres: float
    iou_thres: float
    tracker: str
    device: str
    classes: list[int]
    udp_ip: str
    udp_port: int
    selection_mode: str
    target_class: str
    max_lost_frames: int
    show: bool
    save_video: bool
    save_path: str
    line_width: int
    show_all_tracks: bool
    command_enabled: bool
    command_ip: str
    command_port: int
    window_name: str


def _str_to_bool(value: str | bool) -> bool:
    if isinstance(value, bool):
        return value
    lowered = value.lower()
    if lowered in {"1", "true", "yes", "y", "on"}:
        return True
    if lowered in {"0", "false", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"invalid boolean value: {value}")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Ground-side YOLO tracking app")
    parser.add_argument("--config", default=str(Path(__file__).with_name("config.yaml")))
    parser.add_argument("--model-path")
    parser.add_argument("--source")
    parser.add_argument("--img-size", type=int)
    parser.add_argument("--conf-thres", type=float)
    parser.add_argument("--iou-thres", type=float)
    parser.add_argument("--tracker")
    parser.add_argument("--device")
    parser.add_argument("--classes", nargs="*", type=int)
    parser.add_argument("--udp-ip")
    parser.add_argument("--udp-port", type=int)
    parser.add_argument("--selection-mode", choices=["center", "biggest", "class"])
    parser.add_argument("--target-class")
    parser.add_argument("--max-lost-frames", type=int)
    parser.add_argument("--show", type=_str_to_bool)
    parser.add_argument("--save-video", type=_str_to_bool)
    parser.add_argument("--save-path")
    parser.add_argument("--line-width", type=int)
    parser.add_argument("--show-all-tracks", type=_str_to_bool)
    parser.add_argument("--command-enabled", type=_str_to_bool)
    parser.add_argument("--command-ip")
    parser.add_argument("--command-port", type=int)
    parser.add_argument("--window-name")
    return parser


def _load_yaml_config(path: str) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError("config yaml must be a mapping")
    return data


def load_config() -> AppConfig:
    parser = build_arg_parser()
    args = parser.parse_args()
    yaml_config = _load_yaml_config(args.config)

    merged = dict(yaml_config)
    for key, value in vars(args).items():
        if key == "config":
            continue
        if value is not None:
            merged[key.replace("-", "_")] = value

    return AppConfig(
        model_path=str(merged["model_path"]),
        source=str(merged["source"]),
        img_size=int(merged["img_size"]),
        conf_thres=float(merged["conf_thres"]),
        iou_thres=float(merged["iou_thres"]),
        tracker=str(merged["tracker"]),
        device=str(merged.get("device", "")),
        classes=list(merged.get("classes", [])),
        udp_ip=str(merged["udp_ip"]),
        udp_port=int(merged["udp_port"]),
        selection_mode=str(merged["selection_mode"]),
        target_class=str(merged.get("target_class", "")),
        max_lost_frames=int(merged["max_lost_frames"]),
        show=bool(merged["show"]),
        save_video=bool(merged["save_video"]),
        save_path=str(merged["save_path"]),
        line_width=int(merged.get("line_width", 2)),
        show_all_tracks=bool(merged.get("show_all_tracks", True)),
        command_enabled=bool(merged.get("command_enabled", True)),
        command_ip=str(merged.get("command_ip", "0.0.0.0")),
        command_port=int(merged.get("command_port", 5006)),
        window_name=str(merged.get("window_name", "YOLO Tracking")),
    )
