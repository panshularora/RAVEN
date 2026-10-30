from __future__ import annotations

import math

import numpy as np

from raven import config, plant
from raven.arm.dh import fk_prefix, gripper_fk
from raven.nav.range import range_rays
from raven.types import MissionResult, Pose2D
from raven.vision import inspect as vision


def _base_T(pose: Pose2D) -> np.ndarray:
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    T = np.eye(4, dtype=np.float64)
    T[0, 0], T[0, 1], T[0, 3] = c, -s, pose.x
    T[1, 0], T[1, 1], T[1, 3] = s, c, pose.y
    return T


def _r(v: float, n: int = 4) -> float:
    return round(float(v), n)


def arm_world_points(pose: Pose2D, q: np.ndarray) -> list[list[float]]:
    bt = _base_T(pose)
    pts: list[list[float]] = []
    for n in range(0, 6):
        T = bt @ fk_prefix(q, n) if n else bt
        pts.append([_r(T[0, 3]), _r(T[1, 3]), _r(T[2, 3])])
    g = bt @ gripper_fk(q)
    pts.append([_r(g[0, 3]), _r(g[1, 3]), _r(g[2, 3])])
    return pts


class MissionRecorder:
    """Wrap a backend and keep every step so the control room can replay it."""

    def __init__(self, inner) -> None:
        self.inner = inner
        self.frames: list[dict] = []
        self.phase = "IDLE"
        self.story = "Waiting on the leak alarm."
        self.path: list[list[float]] = []
        self.path_history: list[dict] = []
        self.t = 0.0
        self._grip = False
        self._wl = 0.0
        self._wr = 0.0
        self._ik = None
        self.inspect: dict | None = None
        self._grid = plant.occupancy_grid()
        self._capture()

    def __getattr__(self, name: str):
        return getattr(self.inner, name)

    def set_phase(self, phase: str, story: str | None = None) -> None:
        self.phase = str(phase)
        if story:
            self.story = str(story)
        self._capture()

    def set_nav_path(self, waypoints) -> None:
        self.path = [[_r(x), _r(y)] for x, y in waypoints]
        self.path_history.append({"t": _r(self.t, 3), "pts": self.path})

    def set_wheel_speeds(self, left: float, right: float) -> None:
        self._wl = float(left)
        self._wr = float(right)
        self.inner.set_wheel_speeds(left, right)

    def set_gripper(self, closed: bool) -> None:
        self._grip = bool(closed)
        self.inner.set_gripper(closed)

    def set_inspect(self, payload: dict) -> None:
        self.inspect = payload
        self._capture()

    def set_ik(self, residual: float) -> None:
        self._ik = _r(residual, 4)
        self._capture()

    def step(self, dt: float) -> None:
        self.inner.step(dt)
        self.t += float(dt)
        self._capture()

    def close(self) -> None:
        close = getattr(self.inner, "close", None)
        if callable(close):
            close()

    def _capture(self) -> None:
        pose = self.inner.get_pose()
        q = np.asarray(self.inner.get_joints(), dtype=np.float64)
        rgb = self.inner.get_camera_rgb()
        seen = vision.inspect(rgb)
        lc = [_r(seen.leak_centroid[0], 1), _r(seen.leak_centroid[1], 1)] if seen.leak_centroid else None
        hc = [_r(seen.handle_centroid[0], 1), _r(seen.handle_centroid[1], 1)] if seen.handle_centroid else None
        self.frames.append(
            {
                "t": _r(self.t, 3),
                "phase": self.phase,
                "story": self.story,
                "x": _r(pose.x),
                "y": _r(pose.y),
                "yaw": _r(pose.yaw, 5),
                "q": [_r(v, 5) for v in q.tolist()],
                "grip": 1 if self._grip else 0,
                "leak": 1 if self.inner.get_leak() else 0,
                "valve": _r(self.inner.get_valve_angle(), 2),
                "arm": arm_world_points(pose, q),
                "wl": _r(self._wl, 2),
                "wr": _r(self._wr, 2),
                "rays": range_rays(pose, self._grid),
                "lc": lc,
                "hc": hc,
                "ik": self._ik,
            }
        )


def pack_mission(recorder: MissionRecorder, result: MissionResult) -> dict:
    return {
        "title": "RAVEN leak isolation",
        "blurb": (
            "A tank is leaking. The robot drives around the aisle, looks with its "
            "camera, then turns the green isolation valve to stop the leak."
        ),
        "dt": config.DT,
        "world": {
            "w": config.WORLD_W,
            "h": config.WORLD_H,
            "home": [config.HOME.x, config.HOME.y],
            "valve": [config.VALVE_XY[0], config.VALVE_XY[1], config.VALVE_Z],
            "tank": [config.TANK_XY[0], config.TANK_XY[1]],
            "obstacle": {
                "c": [config.OBSTACLE_CENTER[0], config.OBSTACLE_CENTER[1]],
                "s": [config.OBSTACLE_SIZE[0], config.OBSTACLE_SIZE[1]],
            },
            "standoff": config.WORKSPACE_STANDOFF,
        },
        "result": {
            "status": result.status,
            "reason": result.reason,
            "attempts": result.attempts,
            "valve_angle": _r(result.valve_angle, 2),
        },
        "path": recorder.path,
        "paths": recorder.path_history,
        "inspect": recorder.inspect,
        "frames": recorder.frames,
    }


def compact_grid() -> dict:
    g = plant.occupancy_grid()
    ny, nx = g.shape
    rows = ["".join("1" if cell else "0" for cell in g[iy]) for iy in range(ny)]
    return {"nx": int(nx), "ny": int(ny), "res": config.GRID, "rows": rows}


def gate_time(frames: list[dict]) -> float:
    """Last INSPECT sample, pulled a hair earlier so same-t APPROACH is not shown."""
    t = 0.0
    for f in frames:
        if f["phase"] == "INSPECT":
            t = float(f["t"])
        elif f["phase"] in {"APPROACH", "PLAN", "INTERVENE"} and t > 0:
            break
    return max(0.0, t - 0.02)
