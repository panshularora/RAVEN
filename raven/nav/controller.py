from __future__ import annotations

import math

from raven import config
from raven.types import Pose2D


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def wheel_speeds(
    pose: Pose2D,
    waypoint_xy: tuple[float, float],
    v_max: float = 0.4,
    w_max: float = 1.2,
) -> tuple[float, float]:
    dx = waypoint_xy[0] - pose.x
    dy = waypoint_xy[1] - pose.y
    desired = math.atan2(dy, dx)
    err = wrap(desired - pose.yaw)
    if abs(err) > 0.45:
        v = 0.0
        w = max(-w_max, min(w_max, 2.2 * err))
    else:
        dist = math.hypot(dx, dy)
        v = min(v_max, 1.5 * dist)
        w = max(-w_max, min(w_max, 2.2 * err))
    half = config.TRACK / 2.0
    wr = (v + w * half) / config.WHEEL_R
    wl = (v - w * half) / config.WHEEL_R
    return wl, wr
