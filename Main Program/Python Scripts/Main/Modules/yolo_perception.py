import os
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class Detection:
    x1: int
    y1: int
    x2: int
    y2: int
    label: str
    confidence: float
    timestamp: float
    class_id: Optional[int] = None


class YoloPerception:
    """
    Threaded YOLO detector for CARLA camera frames.

    Usage:
      1) Construct once.
      2) Call update_frame(frame_bgr) from camera callback.
      3) Poll get_latest_detections() from main thread.
      4) Call stop() on shutdown.
    """

    def __init__(
        self,
        model_path: str,
        conf_threshold: float = 0.4,
        infer_hz: float = 10.0,
        device: int = 0,
        fuse_model: bool = True,
    ):
        self.model_path = model_path
        self.conf_threshold = float(conf_threshold)
        self.infer_hz = max(0.1, float(infer_hz))
        self.device = int(device)

        if not os.path.isfile(self.model_path):
            raise FileNotFoundError(f"YOLO model not found: {self.model_path}")

        os.environ.setdefault("ULTRALYTICS_HUB", "False")

        try:
            from ultralytics import YOLO
        except Exception as exc:
            raise RuntimeError("ultralytics is required for YoloPerception") from exc

        self._model = YOLO(self.model_path, task="detect")
        if fuse_model:
            self._model.fuse()

        self._lock = threading.Lock()
        self._running = True
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_detections: List[Detection] = []
        self._latest_batch_id = 0
        self._infer_count = 0

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    @staticmethod
    def resolve_model_path(
        explicit_model: Optional[str],
        script_dir: str,
        candidates: Sequence[str],
    ) -> str:
        if explicit_model:
            p = explicit_model if os.path.isabs(explicit_model) else os.path.join(script_dir, explicit_model)
            if os.path.isfile(p):
                return p
            raise FileNotFoundError(f"YOLO model from explicit path not found: {p}")

        for candidate in candidates:
            p = candidate if os.path.isabs(candidate) else os.path.join(script_dir, candidate)
            if os.path.isfile(p):
                return p

        searched = [c if os.path.isabs(c) else os.path.join(script_dir, c) for c in candidates]
        raise FileNotFoundError("No YOLO model found. Checked:\n" + "\n".join(searched))

    def update_frame(self, frame_bgr: np.ndarray) -> None:
        with self._lock:
            self._latest_frame = frame_bgr

    def get_latest_detections(self) -> List[Detection]:
        with self._lock:
            return list(self._latest_detections)

    def get_latest_detections_with_batch_id(self) -> Tuple[List[Detection], int]:
        with self._lock:
            return list(self._latest_detections), int(self._latest_batch_id)

    def consume_infer_count(self) -> int:
        with self._lock:
            n = self._infer_count
            self._infer_count = 0
            return n

    def _run(self) -> None:
        next_infer_time = 0.0

        while self._running:
            with self._lock:
                frame = self._latest_frame

            if frame is None:
                time.sleep(0.01)
                continue




            now = time.monotonic()
            if now < next_infer_time:
                time.sleep(0.002)
                continue

            results = self._model.predict(
                frame,
                conf=self.conf_threshold,
                verbose=False,
                device=self.device,
            )

            ts = time.monotonic()
            parsed: List[Detection] = []
            result = results[0]

            if result.boxes is not None:
                xyxy = result.boxes.xyxy.cpu().numpy()
                confs = result.boxes.conf.cpu().numpy()
                classes = result.boxes.cls.cpu().numpy().astype(int)
                names = result.names

                for box, conf, cls_id in zip(xyxy, confs, classes):
                    x1, y1, x2, y2 = [int(v) for v in box]
                    if isinstance(names, dict):
                        label = str(names.get(int(cls_id), cls_id))
                    else:
                        label = str(names[int(cls_id)])

                    parsed.append(
                        Detection(
                            x1=x1,
                            y1=y1,
                            x2=x2,
                            y2=y2,
                            label=label,
                            confidence=float(conf),
                            timestamp=ts,
                            class_id=int(cls_id),
                        )
                    )

            with self._lock:
                self._latest_detections = parsed
                self._latest_batch_id += 1
                self._infer_count += 1

            next_infer_time = now + (1.0 / self.infer_hz)

    def stop(self) -> None:
        self._running = False
        if self._thread.is_alive():
            self._thread.join(timeout=0.25)
