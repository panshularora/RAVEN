"""Plant (x east, y north, z up) → Three.js y-up, right-handed.

Robot Object3D local forward is -z. Plant yaw=0 faces +x (east).
yaw_to_three(0) = -π/2 so local -z lands on world +x.
"""

from __future__ import annotations

import math


def plant_to_three(x: float, y: float, z: float = 0.0) -> tuple[float, float, float]:
    return (float(x), float(z), float(y))


def yaw_to_three(yaw: float) -> float:
    return -math.pi / 2.0 - float(yaw)


def forward_xz(yaw: float) -> tuple[float, float]:
    """World XZ of Object3D local -z after yaw_to_three."""
    ry = yaw_to_three(yaw)
    return (-math.sin(ry), -math.cos(ry))
