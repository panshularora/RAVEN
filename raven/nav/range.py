from __future__ import annotations

import math

from raven import config, plant
from raven.types import Pose2D

BEARINGS = (-0.70, -0.35, 0.0, 0.35, 0.70)
MAX_RANGE = 2.0
STEP = 0.08


def range_rays(pose: Pose2D, grid=None) -> list[float]:
    """Five forward sonars in the occupancy grid, metres."""
    if grid is None:
        grid = plant.occupancy_grid()
    ny, nx = grid.shape
    out: list[float] = []
    for b in BEARINGS:
        ang = pose.yaw + b
        c, s = math.cos(ang), math.sin(ang)
        hit = MAX_RANGE
        r = STEP
        while r <= MAX_RANGE:
            x = pose.x + r * c
            y = pose.y + r * s
            ix = int(x / config.GRID)
            iy = int(y / config.GRID)
            if ix < 0 or iy < 0 or ix >= nx or iy >= ny or bool(grid[iy, ix]):
                hit = r
                break
            r += STEP
        out.append(round(float(hit), 3))
    return out
