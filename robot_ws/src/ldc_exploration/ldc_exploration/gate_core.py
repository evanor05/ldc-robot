"""Pure velocity-gate decisions for cmd_vel_gate.

No ROS imports: the node supplies ages and values, tests exercise this module.
"""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class GateDecision:
    ok: bool
    linear: float
    angular: float
    reason: str


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def decide_velocity(linear: float, angular: float, scan_age: float | None,
                    estop: bool, command_age: float | None,
                    min_linear: float = 0.0, max_linear: float = 0.15,
                    max_angular: float = 0.8, scan_timeout: float = 1.0,
                    command_timeout: float = 0.5) -> GateDecision:
    """Return a safe command or zero; never lets a stale/unsafe command through."""
    if estop:
        return GateDecision(False, 0.0, 0.0, 'estop')
    if not math.isfinite(linear) or not math.isfinite(angular):
        return GateDecision(False, 0.0, 0.0, 'nonfinite')
    if scan_age is None or not math.isfinite(scan_age) or scan_age > scan_timeout:
        return GateDecision(False, 0.0, 0.0, 'scan_timeout')
    if command_age is None or not math.isfinite(command_age) or command_age > command_timeout:
        return GateDecision(False, 0.0, 0.0, 'command_timeout')
    low = min(float(min_linear), float(max_linear))
    high = max(float(min_linear), float(max_linear))
    safe_linear = clamp(float(linear), low, high)
    safe_angular = clamp(float(angular), -abs(float(max_angular)), abs(float(max_angular)))
    # A configured zero minimum means this vehicle must not reverse autonomously.
    if safe_linear < 0 and low >= 0:
        safe_linear = 0.0
    return GateDecision(True, safe_linear, safe_angular, 'ok')
