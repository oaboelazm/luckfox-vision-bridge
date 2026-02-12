from __future__ import annotations

import math
import time
from typing import Any

import cv2
import numpy as np
from pupil_apriltags import Detector


def _confidence_from_margin(decision_margin: float) -> float:
    return max(0.0, min(1.0, decision_margin / 100.0))


def apriltag_worker(config: dict[str, Any], shared: dict[str, Any], stop_flag) -> None:
    cam_cfg = config["camera"]
    tag_cfg = config["apriltag"]
    intr = cam_cfg["intrinsics"]
    detector = Detector(families=tag_cfg["family"])

    cap = cv2.VideoCapture(cam_cfg["device"])
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_cfg["width"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_cfg["height"])
    cap.set(cv2.CAP_PROP_FPS, cam_cfg["fps"])

    last_seen_ms = 0

    while not stop_flag.is_set():
        start_ms = int(time.time() * 1000)
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.01)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(intr["fx"], intr["fy"], intr["cx"], intr["cy"]),
            tag_size=cam_cfg["tag_size_m"],
        )

        sample = None
        if detections:
            best = max(detections, key=lambda d: float(d.decision_margin))
            c_x, c_y = best.center
            w = float(cam_cfg["width"])
            h = float(cam_cfg["height"])
            dx_norm = float(((c_x - (w / 2.0)) / (w / 2.0)))
            dy_norm = float(((c_y - (h / 2.0)) / (h / 2.0)))
            z_est_m = float(best.pose_t[2][0]) if best.pose_t is not None else 9.99

            yaw_err_deg = 0.0
            if best.pose_R is not None:
                r = np.array(best.pose_R)
                yaw_err_deg = math.degrees(math.atan2(float(r[1][0]), float(r[0][0])))

            confidence = _confidence_from_margin(float(best.decision_margin))
            sample = {
                "source": "apriltag",
                "timestamp_ms": start_ms,
                "dx_norm": dx_norm,
                "dy_norm": dy_norm,
                "z_est_m": z_est_m,
                "yaw_err_deg": yaw_err_deg,
                "confidence": confidence,
                "latency_ms": int(time.time() * 1000) - start_ms,
            }
            last_seen_ms = start_ms

        lost_timeout_ms = int(tag_cfg["lost_timeout_ms"])
        now_ms = int(time.time() * 1000)
        if sample is None and (now_ms - last_seen_ms) > lost_timeout_ms:
            shared["apriltag"] = None
        elif sample is not None:
            shared["apriltag"] = sample

    cap.release()
