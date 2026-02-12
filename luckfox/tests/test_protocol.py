from __future__ import annotations

import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
sys.path.insert(0, str(SRC))

from common.protocol import GUIDANCE_SIZE, STATUS_SIZE, pack_guidance, pack_status, unpack_guidance, unpack_status
from common.types import GuidanceFrame, StatusFrame


def test_guidance_roundtrip() -> None:
    frame = GuidanceFrame(
        timestamp_ms=123456,
        target_type=1,
        dx_norm=0.1,
        dy_norm=-0.2,
        z_est_m=1.7,
        yaw_err_deg=3.0,
        confidence=0.88,
        target_valid=True,
    )
    data = pack_guidance(77, frame, emergency_hover=False)
    assert len(data) == GUIDANCE_SIZE
    parsed = unpack_guidance(data)
    assert parsed.target_valid is True
    assert abs(parsed.dx_norm - frame.dx_norm) < 1e-6


def test_status_roundtrip() -> None:
    frame = StatusFrame(seq_echo=77, mode=2, control_state=1, battery_v=11.7, ack_code=0)
    data = pack_status(frame)
    assert len(data) == STATUS_SIZE
    parsed = unpack_status(data)
    assert parsed.seq_echo == 77
    assert parsed.mode == 2
