from __future__ import annotations

import json
import os
import subprocess
import time

import cv2
import numpy as np

from models import FramePacket


class VideoSource:
    def __init__(self, source: str) -> None:
        self.source = source
        self.frame_id = 0
        self.cap: cv2.VideoCapture | None = None
        self.udp_process: subprocess.Popen | None = None
        self.udp_frame_shape: tuple[int, int, int] | None = None

        if source.isdigit():
            self._open_udp_port_source(int(source))
        else:
            self.cap = self._open_capture(source)
            if not self.cap.isOpened():
                raise RuntimeError(f"failed to open video source: {source}")

    def read(self) -> FramePacket | None:
        if self.udp_process is not None:
            frame = self._read_udp_frame()
            if frame is None:
                return None
        else:
            if self.cap is None:
                return None
            ok, frame = self.cap.read()
            if not ok or frame is None:
                return None

        packet = FramePacket(frame=frame, frame_id=self.frame_id, timestamp=time.time())
        self.frame_id += 1
        return packet

    def release(self) -> None:
        if self.cap is not None:
            self.cap.release()
        if self.udp_process is not None:
            self.udp_process.terminate()
            try:
                self.udp_process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.udp_process.kill()

    def _open_capture(self, capture_source: str) -> cv2.VideoCapture:
        if self._looks_like_gstreamer_pipeline(capture_source):
            if not self._opencv_has_gstreamer():
                raise RuntimeError(
                    "source looks like a GStreamer pipeline, but this OpenCV build has no GStreamer support. "
                    "Please use /dev/videoX, RTSP, file input, or plain UDP port mode instead."
                )
            cap = cv2.VideoCapture(capture_source, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                return cap
            cap.release()
            raise RuntimeError(
                "failed to open GStreamer pipeline. Please check the pipeline string and required plugins."
            )
        return cv2.VideoCapture(capture_source)

    def _open_udp_port_source(self, udp_port: int) -> None:
        helper_path = "/home/level6/uav_project/src/yolo_app/udp_gst_bridge_helper.py"
        cmd = ["/usr/bin/python3", "-u", helper_path, "--port", str(udp_port)]
        env = os.environ.copy()
        env["PYTHONNOUSERSITE"] = "1"
        self.udp_process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=env,
        )

        if self.udp_process.stdout is None:
            raise RuntimeError("failed to create UDP bridge stdout pipe")

        header_line = self.udp_process.stdout.readline()
        if not header_line:
            error_text = ""
            if self.udp_process.stderr is not None:
                error_text = self.udp_process.stderr.read(1024).decode("utf-8", errors="ignore")
            raise RuntimeError(f"failed to start UDP bridge on port {udp_port}: {error_text.strip()}")

        header = json.loads(header_line.decode("utf-8"))
        if "error" in header:
            raise RuntimeError(header["error"])

        self.udp_frame_shape = (
            int(header["height"]),
            int(header["width"]),
            int(header.get("channels", 3)),
        )

    def _read_udp_frame(self):
        if self.udp_process is None or self.udp_process.stdout is None or self.udp_frame_shape is None:
            return None
        height, width, channels = self.udp_frame_shape
        frame_size = height * width * channels
        raw = self.udp_process.stdout.read(frame_size)
        if raw is None or len(raw) != frame_size:
            return None
        return np.frombuffer(raw, dtype=np.uint8).reshape((height, width, channels))

    def _looks_like_gstreamer_pipeline(self, source: str) -> bool:
        pipeline_markers = (
            " ! ",
            "appsink",
            "udpsrc",
            "rtspsrc",
            "rtph264depay",
            "avdec_h264",
            "videoconvert",
        )
        lowered = source.lower()
        return any(marker in lowered for marker in pipeline_markers)

    def _opencv_has_gstreamer(self) -> bool:
        info = cv2.getBuildInformation()
        for line in info.splitlines():
            if "GStreamer" in line:
                return "YES" in line.upper()
        return False
