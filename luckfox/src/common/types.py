from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class GuidanceFrame:
    timestamp_ms: int
    target_type: int
    dx_norm: float
    dy_norm: float
    z_est_m: float
    yaw_err_deg: float
    confidence: float
    target_valid: bool


@dataclass
class StatusFrame:
    seq_echo: int
    mode: int
    control_state: int
    battery_v: float
    ack_code: int


@dataclass
class TargetSample:
    source: str
    timestamp_ms: int
    dx_norm: float
    dy_norm: float
    z_est_m: float
    yaw_err_deg: float
    confidence: float


@dataclass
class MissionDecision:
    state: str
    emergency_hover: bool
    active_target: Optional[TargetSample]
