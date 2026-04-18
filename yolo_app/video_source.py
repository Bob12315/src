from __future__ import annotations

import time

import cv2

from models import FramePacket


class VideoSource:
    def __init__(self, source: str) -> None:
        self.source = source
        self.frame_id = 0
        capture_source: int | str = int(source) if source.isdigit() else source
        self.cap = cv2.VideoCapture(capture_source)
        if not self.cap.isOpened():
            raise RuntimeError(f"failed to open video source: {source}")

    def read(self) -> FramePacket | None:
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return None
        packet = FramePacket(frame=frame, frame_id=self.frame_id, timestamp=time.time())
        self.frame_id += 1
        return packet

    def release(self) -> None:
        self.cap.release()
