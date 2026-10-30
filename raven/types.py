from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import numpy.typing as npt

Joints = npt.NDArray[np.float64]


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class InspectResult:
    leak_seen: bool
    leak_centroid: tuple[float, float] | None
    handle_seen: bool
    handle_centroid: tuple[float, float] | None


@dataclass(frozen=True)
class MissionResult:
    status: str
    reason: str
    attempts: int
    valve_angle: float
