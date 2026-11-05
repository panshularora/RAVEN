from __future__ import annotations

from raven import config
from raven.backends.mock import MockBackend
from raven.mission import run_mission
from raven.sim.fire import pack_fire
from raven.sim.recorder import MissionRecorder, compact_grid, gate_time, pack_mission

ACTIONS = [
    {
        "id": "close",
        "title": "Close isolation valve V-17",
        "detail": "Solve DLS IK, grasp the green handle, wrist-roll +90°, verify leak clear.",
    },
    {
        "id": "inspect_only",
        "title": "Confirm leak and hold",
        "detail": "Keep the camera log. Do not touch the valve. Cell stays locked.",
    },
    {
        "id": "human",
        "title": "Request human entry",
        "detail": "Abort autonomy. Flag HUMAN_ASSIST. Valve left open.",
    },
    {
        "id": "stuck",
        "title": "Force a seized-valve recovery",
        "detail": "Two close attempts fail on purpose, then the robot stops for a human.",
    },
]

SENSORS = [
    {
        "name": "Wheel odometry",
        "use": "Dead-reckon (x, y, yaw) from left/right wheel speeds on a 0.30 m track.",
        "output": "Pose2D at 20 Hz",
    },
    {
        "name": "Five forward sonars",
        "use": "0.08 m steps into the inflated occupancy grid, ±40° fan, 2 m max.",
        "output": "Five ranges in metres",
    },
    {
        "name": "Wrist RGB camera",
        "use": "OpenCV HSV: red spray = leak, green blob = valve handle, with centroids.",
        "output": "leak_seen, handle_seen, pixel centroids",
    },
    {
        "name": "Valve encoder",
        "use": "Ground-truth handle angle after the wrist roll. Closed band 80–100°.",
        "output": "Degrees",
    },
    {
        "name": "Leak flag",
        "use": "Plant signal used only in VERIFY, never as a cheat during INSPECT.",
        "output": "true / false",
    },
]

METHODS = [
    "A* 8-connected on a 0.10 m grid, 0.35 m inflation around the aisle",
    "Unicycle follow: turn in place past 0.45 rad, then drive at 0.4 m/s",
    "5-DOF DH forward kinematics, damped-least-squares IK, 2 cm residual cap",
    "Linear joint interpolation at 20 Hz, parallel gripper, wrist +90° to seat the valve",
]

BRIEFING = {
    "incident_id": "INC-1042",
    "when": "14:22 IST · Bay 3",
    "site": "Tank T-04  ·  isolation valve V-17",
    "problem": (
        "A hydrocarbon tank is spraying. Humans are locked out of the cell. "
        "RAVEN already has the plant map and the alarm pose. It must confirm the leak "
        "with its own camera, then you choose the intervention."
    ),
    "solving": (
        "Isolate Tank T-04 by closing V-17 without sending a person in. "
        "Success = valve 80–100° and leak flag false. Two failed closes → HUMAN_ASSIST."
    ),
}


def pack_story() -> dict:
    """Record every on-scene order so the control room can branch like a live sim."""
    specs = (
        ("close", False, "close"),
        ("inspect_only", False, "inspect_only"),
        ("human", False, "human"),
        ("stuck", True, "close"),
    )
    branches: dict = {}
    for key, fail, policy in specs:
        rec = MissionRecorder(MockBackend(fail_valve=fail))
        result = run_mission(rec, after_inspect=policy)
        packed = pack_mission(rec, result)
        branches[key] = {
            "result": packed["result"],
            "frames": packed["frames"],
            "paths": packed["paths"],
            "path": packed["path"],
            "inspect": packed["inspect"],
        }

    close = branches["close"]
    return {
        "mode": "story",
        "title": "RAVEN · INC-1042 leak isolation",
        "blurb": BRIEFING["problem"],
        "briefing": BRIEFING,
        "sensors": SENSORS,
        "methods": METHODS,
        "actions": ACTIONS,
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
        "grid": compact_grid(),
        "gate_t": gate_time(close["frames"]),
        "result": close["result"],
        "path": close["path"],
        "paths": close["paths"],
        "inspect": close["inspect"],
        "frames": close["frames"],
        "branches": branches,
        "fire": pack_fire(),
    }
