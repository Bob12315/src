from __future__ import annotations

from ultralytics import YOLO

from config import AppConfig
from models import Track


class TrackerRunner:
    """
    Thin wrapper around the official Ultralytics tracking API.

    This class intentionally keeps the official `model.track(...)` flow and only
    normalizes the result into project-level Track objects.
    """

    def __init__(self, cfg: AppConfig) -> None:
        self.cfg = cfg
        self.model = YOLO(cfg.model_path)

    def run(self, frame) -> list[Track]:
        results = self.model.track(
            source=frame,
            persist=True,
            tracker=self.cfg.tracker,
            conf=self.cfg.conf_thres,
            iou=self.cfg.iou_thres,
            imgsz=self.cfg.img_size,
            device=self.cfg.device if self.cfg.device else None,
            classes=self.cfg.classes or None,
            verbose=False,
        )
        if not results:
            return []

        result = results[0]
        boxes = result.boxes
        if boxes is None or boxes.id is None or boxes.xyxy is None:
            return []

        names = result.names or {}
        xyxy_list = boxes.xyxy.cpu().tolist()
        id_list = boxes.id.int().cpu().tolist()
        cls_list = boxes.cls.int().cpu().tolist() if boxes.cls is not None else [0] * len(id_list)
        conf_list = boxes.conf.cpu().tolist() if boxes.conf is not None else [0.0] * len(id_list)

        tracks: list[Track] = []
        for xyxy, track_id, class_id, confidence in zip(xyxy_list, id_list, cls_list, conf_list):
            x1, y1, x2, y2 = [float(v) for v in xyxy]
            tracks.append(
                Track(
                    track_id=int(track_id),
                    class_id=int(class_id),
                    class_name=str(names.get(int(class_id), int(class_id))),
                    confidence=float(confidence),
                    x1=x1,
                    y1=y1,
                    x2=x2,
                    y2=y2,
                )
            )
        return tracks
