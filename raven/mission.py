from __future__ import annotations

import math
import time
from collections.abc import Callable

import numpy as np

from raven import config, plant
from raven.arm import ik, motion
from raven.nav import astar
from raven.nav.controller import wheel_speeds, wrap
from raven.types import MissionResult, Pose2D
from raven.vision import inspect as vision


def _announce(backend, phase: str, story: str) -> None:
    fn = getattr(backend, "set_phase", None)
    if callable(fn):
        fn(phase, story)


def _set_nav_path(backend, waypoints: list[tuple[float, float]]) -> None:
    fn = getattr(backend, "set_nav_path", None)
    if callable(fn):
        fn(waypoints)


def _handle_T(world_x: float | None = None) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[0, 3] = config.VALVE_XY[0] if world_x is None else world_x
    T[1, 3] = config.VALVE_XY[1]
    T[2, 3] = config.VALVE_Z
    T[:3, :3] = np.array([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]])
    return T


def _base_T(pose: Pose2D) -> np.ndarray:
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    T = np.eye(4, dtype=np.float64)
    T[0, 0], T[0, 1], T[0, 3] = c, -s, pose.x
    T[1, 0], T[1, 1], T[1, 3] = s, c, pose.y
    return T


def _in_base(T_world: np.ndarray, pose: Pose2D) -> np.ndarray:
    return np.linalg.inv(_base_T(pose)) @ T_world


def _move_arm(backend, q_goal: np.ndarray, dt: float) -> None:
    q0 = backend.get_joints()
    n = max(int(config.CMD_HZ * 0.4), 2)
    for q in motion.interpolate(q0, q_goal, n):
        backend.set_joint_targets(q)
        backend.step(dt)
    backend.set_joint_targets(q_goal)
    backend.step(dt)


def _goto_pose(backend, goal: Pose2D, dt: float, now: Callable[[], float], t0: float) -> str | None:
    grid = plant.occupancy_grid()
    start = plant.pose_to_cell(backend.get_pose())
    path = astar.plan(grid, start, plant.pose_to_cell(goal))
    if not path:
        return "no_path"
    waypoints = [plant.cell_center(ix, iy) for ix, iy in path]
    waypoints.append((goal.x, goal.y))
    _set_nav_path(backend, waypoints)
    wi = 0
    while True:
        if now() - t0 >= config.NAV_TIMEOUT_S:
            backend.set_wheel_speeds(0.0, 0.0)
            backend.step(dt)
            return "nav_timeout"
        pose = backend.get_pose()
        gx, gy = waypoints[min(wi, len(waypoints) - 1)]
        last = wi >= len(waypoints) - 1
        dist = math.hypot(goal.x - pose.x, goal.y - pose.y) if last else math.hypot(gx - pose.x, gy - pose.y)
        yaw_err = abs(wrap(goal.yaw - pose.yaw)) if last else 0.0
        if last and dist < config.NAV_POS_TOL and yaw_err < config.NAV_YAW_TOL:
            backend.set_wheel_speeds(0.0, 0.0)
            backend.step(dt)
            return None
        if not last and dist < 0.18:
            wi += 1
            continue
        if last and dist < config.NAV_POS_TOL:
            w = max(-1.2, min(1.2, 2.5 * wrap(goal.yaw - pose.yaw)))
            half = config.TRACK / 2.0
            wr = (w * half) / config.WHEEL_R
            wl = (-w * half) / config.WHEEL_R
            backend.set_wheel_speeds(wl, wr)
        else:
            target = (goal.x, goal.y) if last else (gx, gy)
            wl, wr = wheel_speeds(pose, target)
            backend.set_wheel_speeds(wl, wr)
        backend.step(dt)


def _solve_ik(ik_solver, T_target: np.ndarray, q_seed: np.ndarray) -> tuple[np.ndarray, float]:
    q, res = ik_solver(T_target, q_seed)
    if res <= config.IK_RESIDUAL_MAX:
        return q, res
    q2, res2 = ik_solver(T_target, np.zeros(5, dtype=np.float64))
    if res2 < res:
        return q2, res2
    return q, res


def _push_inspect(backend, seen) -> None:
    fn = getattr(backend, "set_inspect", None)
    if callable(fn):
        fn(
            {
                "leak_seen": bool(seen.leak_seen),
                "leak_centroid": seen.leak_centroid,
                "handle_seen": bool(seen.handle_seen),
                "handle_centroid": seen.handle_centroid,
            }
        )


def _push_ik(backend, residual: float) -> None:
    fn = getattr(backend, "set_ik", None)
    if callable(fn):
        fn(float(residual))


def run_mission(
    backend,
    *,
    dt: float = config.DT,
    now: Callable[[], float] | None = None,
    ik_solver=None,
    after_inspect: str = "close",
) -> MissionResult:
    clock = now or time.monotonic
    solver = ik_solver or ik.solve
    attempts = 0

    _announce(
        backend,
        "NAVIGATE",
        "Tank leak alarm. Driving around the aisle to the inspect pose.",
    )
    nav_err = _goto_pose(backend, config.APPROACH, dt, clock, clock())
    if nav_err:
        _announce(backend, "HUMAN_ASSIST", "Could not reach the tank. Stopping for a human.")
        return MissionResult("HUMAN_ASSIST", nav_err, attempts, backend.get_valve_angle())

    _announce(
        backend,
        "INSPECT",
        "Raising the wrist camera. Looking for red leak spray and the green valve handle.",
    )
    _move_arm(backend, config.INSPECT_Q, dt)
    seen = vision.inspect(backend.get_camera_rgb())
    _push_inspect(backend, seen)
    if not seen.leak_seen:
        if backend.get_leak():
            _announce(backend, "HUMAN_ASSIST", "Camera never saw the leak. Stopping for a human.")
            return MissionResult("HUMAN_ASSIST", "no_leak_in_view", attempts, backend.get_valve_angle())
        _announce(backend, "DONE", "No leak in the image. False alarm.")
        return MissionResult("DONE", "false_alarm", attempts, backend.get_valve_angle())

    if after_inspect == "inspect_only":
        _announce(
            backend,
            "DONE",
            "Leak confirmed on the wrist camera. Holding. Valve left untouched.",
        )
        return MissionResult("DONE", "leak_confirmed", attempts, backend.get_valve_angle())
    if after_inspect == "human":
        _announce(
            backend,
            "HUMAN_ASSIST",
            "Operator requested human entry. Cell stays locked. Valve left open.",
        )
        return MissionResult("HUMAN_ASSIST", "operator_request", attempts, backend.get_valve_angle())
    if after_inspect == "rescan":
        _announce(backend, "INSPECT", "Backing off half a metre and scanning the leak again.")
        pose = backend.get_pose()
        retreat = Pose2D(max(pose.x - 0.5, 1.2), pose.y, pose.yaw)
        nav_err = _goto_pose(backend, retreat, dt, clock, clock())
        if nav_err:
            _announce(backend, "HUMAN_ASSIST", "Could not reverse for a rescan.")
            return MissionResult("HUMAN_ASSIST", nav_err, attempts, backend.get_valve_angle())
        _move_arm(backend, config.INSPECT_Q, dt)
        seen = vision.inspect(backend.get_camera_rgb())
        _push_inspect(backend, seen)
        if not seen.leak_seen:
            _announce(backend, "DONE", "Second scan saw no leak. Holding.")
            return MissionResult("DONE", "false_alarm", attempts, backend.get_valve_angle())

    grasp_pose = Pose2D(
        config.VALVE_XY[0] - config.WORKSPACE_STANDOFF,
        config.VALVE_XY[1],
        0.0,
    )
    _announce(
        backend,
        "APPROACH",
        "Leak confirmed. Rolling in so the arm can reach the valve.",
    )
    nav_err = _goto_pose(backend, grasp_pose, dt, clock, clock())
    if nav_err:
        _announce(backend, "HUMAN_ASSIST", "Could not reach the valve. Stopping for a human.")
        return MissionResult("HUMAN_ASSIST", nav_err, attempts, backend.get_valve_angle())

    while True:
        attempts += 1
        _announce(
            backend,
            "PLAN",
            "Solving arm inverse kinematics for a grasp on the handle.",
        )
        pose = backend.get_pose()
        T_grasp = _in_base(_handle_T(), pose)
        T_pre = _in_base(_handle_T(config.VALVE_XY[0] - config.PREGRASP_X), pose)
        q_pre, r_pre = _solve_ik(solver, T_pre, backend.get_joints())
        q_g, r_g = _solve_ik(solver, T_grasp, q_pre)
        _push_ik(backend, max(r_pre, r_g))
        if r_pre > config.IK_RESIDUAL_MAX or r_g > config.IK_RESIDUAL_MAX:
            _announce(backend, "HUMAN_ASSIST", "Arm cannot reach the handle. Stopping for a human.")
            return MissionResult("HUMAN_ASSIST", "ik_failed", attempts, backend.get_valve_angle())

        _announce(
            backend,
            "INTERVENE",
            "Gripping the handle and rolling the wrist a quarter turn.",
        )
        _move_arm(backend, q_pre, dt)
        backend.set_gripper(True)
        backend.step(dt)
        _move_arm(backend, q_g, dt)
        q_turn = backend.get_joints().copy()
        q_turn[4] = q_turn[4] + math.pi / 2
        _move_arm(backend, q_turn, dt)
        backend.set_gripper(False)
        backend.step(dt)

        _announce(backend, "VERIFY", "Checking valve angle and whether the leak is gone.")
        _move_arm(backend, config.INSPECT_Q, dt)

        angle = backend.get_valve_angle()
        leak = backend.get_leak()
        closed = config.VALVE_CLOSED[0] <= angle <= config.VALVE_CLOSED[1]
        if (not leak) and closed:
            _announce(backend, "DONE", "Valve at 90 degrees. Leak isolated.")
            return MissionResult("DONE", "valve_closed", attempts, angle)
        if attempts >= 2:
            _announce(backend, "HUMAN_ASSIST", "Two close attempts failed. Requesting a human.")
            return MissionResult("HUMAN_ASSIST", "valve_not_closed", attempts, angle)
        _announce(backend, "RETRY", "Valve still open. Trying one more close.")
