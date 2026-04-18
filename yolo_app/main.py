from __future__ import annotations

import os
import sys

# Some conda-packaged OpenCV builds use the Qt backend but do not bundle fonts.
# Point Qt to a common system font directory before importing cv2 to suppress warnings.
os.environ.setdefault("QT_QPA_FONTDIR", "/usr/share/fonts/truetype/dejavu")

import cv2

from annotator import Annotator
from command_receiver import CommandReceiver
from config import load_config
from target_manager import TargetManager
from tracker_runner import TrackerRunner
from udp_publisher import UdpPublisher
from utils import ensure_parent_dir
from video_source import VideoSource


def build_video_writer(save_path: str, fps: float, width: int, height: int) -> cv2.VideoWriter:
    ensure_parent_dir(save_path)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    return cv2.VideoWriter(save_path, fourcc, fps if fps > 0 else 30.0, (width, height))


def main() -> int:
    cfg = load_config()
    video_source = VideoSource(cfg.source)
    tracker = TrackerRunner(cfg)
    target_manager = TargetManager(cfg)
    udp_publisher = UdpPublisher(cfg.udp_ip, cfg.udp_port)
    command_receiver = CommandReceiver(cfg.command_ip, cfg.command_port, enabled=cfg.command_enabled)
    annotator = Annotator(cfg)
    writer = None

    try:
        while True:
            packet = video_source.read()
            if packet is None:
                break

            frame = packet.frame
            image_height, image_width = frame.shape[:2]

            tracks = tracker.run(frame)
            commands = command_receiver.poll()
            for command in commands:
                target_manager.apply_command(command, tracks)

            current_target = target_manager.update(
                tracks=tracks,
                image_width=image_width,
                image_height=image_height,
                frame_id=packet.frame_id,
                timestamp=packet.timestamp,
            )
            udp_publisher.publish(current_target)

            if cfg.show or cfg.save_video:
                annotated = annotator.annotate(
                    frame=frame,
                    tracks=tracks,
                    current_target=current_target,
                    locked_track_id=target_manager.locked_track_id,
                )
                if cfg.show:
                    cv2.imshow(cfg.window_name, annotated)
                    key = cv2.waitKey(1) & 0xFF
                    if key in {27, ord("q")}:
                        break
                if cfg.save_video:
                    if writer is None:
                        fps = video_source.cap.get(cv2.CAP_PROP_FPS)
                        writer = build_video_writer(cfg.save_path, fps, image_width, image_height)
                    writer.write(annotated)
    finally:
        video_source.release()
        udp_publisher.close()
        command_receiver.close()
        if writer is not None:
            writer.release()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
