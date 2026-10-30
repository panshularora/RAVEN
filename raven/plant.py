from __future__ import annotations

import numpy as np

from raven import config
from raven.types import Pose2D


def occupancy_grid() -> np.ndarray:
    g = np.zeros((config.GRID_NY, config.GRID_NX), dtype=bool)
    g[0, :] = True
    g[-1, :] = True
    g[:, 0] = True
    g[:, -1] = True

    cx, cy = config.OBSTACLE_CENTER
    sx, sy = config.OBSTACLE_SIZE
    pad = config.INFLATION
    x0, x1 = cx - sx / 2 - pad, cx + sx / 2 + pad
    y0, y1 = cy - sy / 2 - pad, cy + sy / 2 + pad
    for iy in range(config.GRID_NY):
        for ix in range(config.GRID_NX):
            x, y = cell_center(ix, iy)
            if x0 <= x <= x1 and y0 <= y <= y1:
                g[iy, ix] = True
    return g


def pose_to_cell(pose: Pose2D) -> tuple[int, int]:
    ix = int(np.floor(pose.x / config.GRID))
    iy = int(np.floor(pose.y / config.GRID))
    return ix, iy


def cell_center(ix: int, iy: int) -> tuple[float, float]:
    return ((ix + 0.5) * config.GRID, (iy + 0.5) * config.GRID)
