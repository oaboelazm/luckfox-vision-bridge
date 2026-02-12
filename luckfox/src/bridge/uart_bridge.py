from __future__ import annotations

import csv
import time
from pathlib import Path
from typing import Any, Optional

import serial

from common.protocol import STATUS_HEADER, STATUS_SIZE, pack_guidance, parse_fixed_packets, unpack_status
from common.types import GuidanceFrame, TargetSample
from mission.mission_manager import MissionManager


def _as_target(sample: Optional[dict[str, Any]]) -> Optional[TargetSample]:
    if sample is None:
        return None
    return TargetSample(
        source=sample["source"],
        timestamp_ms=int(sample["timestamp_ms"]),
        dx_norm=float(sample["dx_norm"]),
        dy_norm=float(sample["dy_norm"]),
        z_est_m=float(sample["z_est_m"]),
        yaw_err_deg=float(sample["yaw_err_deg"]),
        confidence=float(sample["confidence"]),
    )


def _choose_target(shared: dict[str, Any], min_tag_conf: float) -> Optional[TargetSample]:
    apr = _as_target(shared.get("apriltag"))
    pln = _as_target(shared.get("plant"))

    if apr and apr.confidence >= min_tag_conf:
        return apr
    if pln:
        return pln
    return apr


def bridge_worker(config: dict[str, Any], shared: dict[str, Any], stop_flag) -> None:
    uart_cfg = config["uart"]
    apr_cfg = config["apriltag"]
    mission_cfg = config["mission"]

    manager = MissionManager(
        target_ttl_ms=int(mission_cfg["target_ttl_ms"]),
        approach_z_m=float(mission_cfg["approach_z_m"]),
        hold_z_m=float(mission_cfg["hold_z_m"]),
        max_abs_offset_for_hold=float(mission_cfg["max_abs_offset_for_hold"]),
    )

    csv_path = Path(config["logging"]["csv_path"])
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    ser = serial.Serial(uart_cfg["port"], int(uart_cfg["baud"]), timeout=0.001)
    rx_buf = bytearray()

    last_status_ms = int(time.time() * 1000)
    seq = 0
    send_period = 1.0 / float(uart_cfg["send_rate_hz"])

    with csv_path.open("a", newline="") as fp:
        writer = csv.writer(fp)
        writer.writerow(["timestamp_ms", "seq", "state", "target", "confidence", "dx", "dy", "z", "status_mode", "status_ack"])

        while not stop_flag.is_set():
            t0 = time.time()
            now_ms = int(t0 * 1000)

            target = _choose_target(shared, float(apr_cfg["min_confidence"]))
            decision = manager.update(now_ms, target)

            if decision.active_target is None:
                frame = GuidanceFrame(
                    timestamp_ms=now_ms,
                    target_type=0,
                    dx_norm=0.0,
                    dy_norm=0.0,
                    z_est_m=99.0,
                    yaw_err_deg=0.0,
                    confidence=0.0,
                    target_valid=False,
                )
            else:
                t = decision.active_target
                frame = GuidanceFrame(
                    timestamp_ms=now_ms,
                    target_type=1 if t.source == "apriltag" else 2,
                    dx_norm=t.dx_norm,
                    dy_norm=t.dy_norm,
                    z_est_m=t.z_est_m,
                    yaw_err_deg=t.yaw_err_deg,
                    confidence=t.confidence,
                    target_valid=True,
                )

            packet = pack_guidance(seq, frame, emergency_hover=decision.emergency_hover)
            ser.write(packet)

            status_mode = -1
            status_ack = -1
            chunk = ser.read(128)
            if chunk:
                rx_buf.extend(chunk)
                for raw in parse_fixed_packets(rx_buf, STATUS_HEADER, STATUS_SIZE):
                    try:
                        status = unpack_status(raw)
                    except Exception:
                        continue
                    last_status_ms = now_ms
                    status_mode = status.mode
                    status_ack = status.ack_code
                    shared["status"] = {
                        "seq_echo": status.seq_echo,
                        "mode": status.mode,
                        "control_state": status.control_state,
                        "battery_v": status.battery_v,
                        "ack_code": status.ack_code,
                    }

            if (now_ms - last_status_ms) > int(uart_cfg["status_timeout_ms"]):
                ser.close()
                time.sleep(0.05)
                ser = serial.Serial(uart_cfg["port"], int(uart_cfg["fallback_baud"]), timeout=0.001)
                time.sleep(0.05)
                ser.close()
                ser = serial.Serial(uart_cfg["port"], int(uart_cfg["baud"]), timeout=0.001)
                last_status_ms = now_ms

            writer.writerow(
                [
                    now_ms,
                    seq,
                    decision.state,
                    frame.target_type,
                    frame.confidence,
                    frame.dx_norm,
                    frame.dy_norm,
                    frame.z_est_m,
                    status_mode,
                    status_ack,
                ]
            )

            seq = (seq + 1) & 0xFFFF
            elapsed = time.time() - t0
            sleep_s = max(0.0, send_period - elapsed)
            time.sleep(sleep_s)

    ser.close()
