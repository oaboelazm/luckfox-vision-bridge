from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from common.types import MissionDecision, TargetSample

SEARCH = "SEARCH"
TRACK = "TRACK"
APPROACH = "APPROACH"
HOLD = "HOLD"


@dataclass
class MissionManager:
    target_ttl_ms: int
    approach_z_m: float
    hold_z_m: float
    max_abs_offset_for_hold: float
    current_state: str = SEARCH

    def update(self, now_ms: int, target: Optional[TargetSample]) -> MissionDecision:
        active_target = None
        emergency_hover = False

        if target is not None:
            age_ms = now_ms - target.timestamp_ms
            if age_ms <= self.target_ttl_ms:
                active_target = target

        if active_target is None:
            self.current_state = SEARCH
            emergency_hover = True
            return MissionDecision(self.current_state, emergency_hover, None)

        if self.current_state == SEARCH:
            self.current_state = TRACK

        abs_dx = abs(active_target.dx_norm)
        abs_dy = abs(active_target.dy_norm)

        if self.current_state in (TRACK, APPROACH):
            if (
                active_target.z_est_m <= self.hold_z_m
                and abs_dx <= self.max_abs_offset_for_hold
                and abs_dy <= self.max_abs_offset_for_hold
            ):
                self.current_state = HOLD
            elif active_target.z_est_m <= self.approach_z_m:
                self.current_state = APPROACH
            else:
                self.current_state = TRACK
        elif self.current_state == HOLD:
            if active_target.z_est_m > self.approach_z_m * 1.2:
                self.current_state = TRACK

        return MissionDecision(self.current_state, emergency_hover, active_target)
