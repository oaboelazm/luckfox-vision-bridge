from __future__ import annotations

import time
from typing import Any

import cv2
import numpy as np


class PlantDetectorRKNN:
    def __init__(self, model_path: str) -> None:
        from rknnlite.api import RKNNLite  # type: ignore

        self.rknn = RKNNLite()
        rc = self.rknn.load_rknn(model_path)
        if rc != 0:
            raise RuntimeError(f"load_rknn failed: {rc}")
        rc = self.rknn.init_runtime()
        if rc != 0:
            raise RuntimeError(f"init_runtime failed: {rc}")

    def infer(self, image_bgr: np.ndarray) -> np.ndarray:
        outputs = self.rknn.inference(inputs=[image_bgr])
        return outputs[0]


def _decode_stub(output: np.ndarray, width: int, height: int, min_conf: float):
    if output is None or len(output) == 0:
        return None
    row = output[0]
    if len(row) < 5:
        return None
    score = float(row[4])
    if score < min_conf:
        return None

    x1, y1, x2, y2 = [float(v) for v in row[:4]]
    cx = (x1 + x2) / 2.0
    cy = (y1 + y2) / 2.0
    dx_norm = ((cx - (width / 2.0)) / (width / 2.0))
    dy_norm = ((cy - (height / 2.0)) / (height / 2.0))

    area = max(1.0, (x2 - x1) * (y2 - y1))
    z_est_m = max(0.3, min(8.0, 20000.0 / area))

    return dx_norm, dy_norm, z_est_m, score


def plant_worker(config: dict[str, Any], shared: dict[str, Any], stop_flag) -> None:
    plant_cfg = config["plant"]
    if not plant_cfg.get("enabled", False):
        return

    cam_cfg = config["camera"]
    cap = cv2.VideoCapture(cam_cfg["device"])
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_cfg["width"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_cfg["height"])
    cap.set(cv2.CAP_PROP_FPS, cam_cfg["fps"])

    detector = PlantDetectorRKNN(plant_cfg["model_path"])
    input_size = int(plant_cfg["input_size"])
    min_conf = float(plant_cfg["min_confidence"])

    while not stop_flag.is_set():
        t0 = int(time.time() * 1000)
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.02)
            continue

        resized = cv2.resize(frame, (input_size, input_size))
        output = detector.infer(resized)
        decoded = _decode_stub(output, cam_cfg["width"], cam_cfg["height"], min_conf)

        if decoded is None:
            shared["plant"] = None
            continue

        dx_norm, dy_norm, z_est_m, confidence = decoded
        shared["plant"] = {
            "source": "plant",
            "timestamp_ms": t0,
            "dx_norm": float(dx_norm),
            "dy_norm": float(dy_norm),
            "z_est_m": float(z_est_m),
            "yaw_err_deg": 0.0,
            "confidence": float(confidence),
            "latency_ms": int(time.time() * 1000) - t0,
        }

    cap.release()
