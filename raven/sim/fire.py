"""Hydrocarbon fire field and the sensors that hunt it.

Operator ignites a free cell. RAVEN does not get that pose. IR, smoke,
VOC, UV flame, heat-flux and wrist RGB all have to agree before locate.
Constants are shared with raven/web/fire.js — keep the numbers identical.
"""

from __future__ import annotations

import math

import numpy as np

from raven import config, plant
from raven.nav import astar
from raven.types import Pose2D

AMBIENT_C = 28.0
CORE_C = 820.0
THERMAL_LEN = 1.85
SMOKE_LEN = 2.4
VOC_K = 420.0
FLUX_K = 48.0
UV_RANGE = 2.3
RGB_RANGE = 1.55
STANDOFF = 0.95

THRESH = {
    "thermal": 55.0,
    "smoke": 0.12,
    "voc": 80.0,
    "uv": 0.4,
    "flux": 2.0,
}

SITES = [
    {"id": "tank", "xy": [6.2, 4.2], "label": "T-04 skirt", "fuel": "hydrocarbon pool"},
    {"id": "valve", "xy": [6.2, 3.35], "label": "V-17 packing", "fuel": "valve gland"},
    {"id": "south", "xy": [2.0, 1.5], "label": "south aisle", "fuel": "spill trail"},
    {"id": "east", "xy": [7.0, 2.0], "label": "east bay", "fuel": "drip tray"},
    {"id": "north", "xy": [1.5, 4.8], "label": "north wall", "fuel": "cable tray"},
]

ACTIONS = [
    {
        "id": "co2",
        "title": "CO₂ flood from the wrist",
        "detail": "Hold standoff. Dump inert gas onto the seat. Fire starves of oxygen.",
    },
    {
        "id": "foam",
        "title": "AFFF foam blanket",
        "detail": "Cover the fuel surface so vapour cannot reignite.",
    },
    {
        "id": "isolate",
        "title": "Isolate fuel at V-17",
        "detail": "Close the tank valve. Works on T-04 / V-17 fires. Remote spill keeps burning.",
    },
    {
        "id": "human",
        "title": "Request human with extinguisher",
        "detail": "Abort autonomy. Cell stays locked until a person enters.",
    },
]

SENSORS = [
    {
        "id": "IR-TPA",
        "name": "IR thermopile",
        "use": "8-pixel array ±40°. Temperature from inverse-range heat, line-of-sight through occupancy.",
        "unit": "°C",
        "output": "ambient 28 °C, trips above 55 °C",
    },
    {
        "id": "SMOKE",
        "name": "Optical smoke",
        "use": "Scattered IR in the 0.9 µm band. Optical density along the look ray.",
        "unit": "OD",
        "output": "0–1, trips above 0.12",
    },
    {
        "id": "VOC",
        "name": "PID VOC",
        "use": "Photo-ionisation of hydrocarbon vapour. Inverse-square from the seat.",
        "unit": "ppm",
        "output": "trips above 80 ppm",
    },
    {
        "id": "UV-FLAME",
        "name": "UV flame",
        "use": "185–260 nm. Needs a clear look at the flame front inside 2.3 m.",
        "unit": "bool",
        "output": "1 when the robot faces the fire",
    },
    {
        "id": "HF",
        "name": "Heat flux",
        "use": "Radiometer on the mast. kW/m² from the pool.",
        "unit": "kW/m²",
        "output": "trips above 2.0",
    },
    {
        "id": "RGB",
        "name": "Wrist RGB",
        "use": "OpenCV HSV orange/red blob once the camera is inside 1.55 m and facing.",
        "unit": "blob",
        "output": "flame_seen + centroid",
    },
]


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def line_of_sight(x0: float, y0: float, x1: float, y1: float, grid: np.ndarray | None = None) -> float:
    """1.0 clear, 0.12 if the ray hits an occupied cell (heat still leaks)."""
    if grid is None:
        grid = plant.occupancy_grid()
    dist = math.hypot(x1 - x0, y1 - y0)
    if dist < 1e-6:
        return 1.0
    steps = max(2, int(dist / 0.08))
    ny, nx = grid.shape
    for i in range(1, steps):
        u = i / steps
        x = x0 + (x1 - x0) * u
        y = y0 + (y1 - y0) * u
        ix = int(x / config.GRID)
        iy = int(y / config.GRID)
        if 0 <= ix < nx and 0 <= iy < ny and grid[iy, ix]:
            return 0.12
    return 1.0


def sample_sensors(
    rx: float,
    ry: float,
    yaw: float,
    fx: float,
    fy: float,
    intensity: float,
    t: float = 0.0,
    grid: np.ndarray | None = None,
) -> dict:
    """Sensor suite at the robot looking toward a fire of given intensity 0..1."""
    dx = fx - rx
    dy = fy - ry
    dist = math.hypot(dx, dy)
    bearing = math.atan2(dy, dx)
    facing = math.cos(wrap(bearing - yaw))
    los = line_of_sight(rx, ry, fx, fy, grid)
    flicker = 0.92 + 0.08 * math.sin(t * 11.0)
    thermal = AMBIENT_C + (
        intensity
        * (CORE_C - AMBIENT_C)
        * math.exp(-dist / THERMAL_LEN)
        * (0.55 + 0.45 * max(0.0, facing))
        * los
        * flicker
    )
    smoke = intensity * math.exp(-dist / SMOKE_LEN) * los * (0.85 + 0.15 * math.sin(t * 3.1))
    voc = intensity * VOC_K / (dist * dist + 0.35)
    uv = 1.0 if (intensity > 0.35 and dist < UV_RANGE and facing > 0.15 and los > 0.5) else 0.0
    flux = intensity * FLUX_K / (dist + 0.35) ** 2 * los
    rgb = 1 if (intensity > 0.4 and dist < RGB_RANGE and facing > 0.25 and los > 0.4) else 0
    pile = []
    for i in range(8):
        a = yaw + (-0.7 + i * (1.4 / 7))
        f = math.cos(wrap(bearing - a))
        cell = AMBIENT_C + intensity * (CORE_C - AMBIENT_C) * math.exp(-dist / THERMAL_LEN) * max(0.0, f) * los
        pile.append(round(cell, 1))
    activated = {
        "IR-TPA": thermal > THRESH["thermal"],
        "SMOKE": smoke > THRESH["smoke"],
        "VOC": voc > THRESH["voc"],
        "UV-FLAME": uv >= THRESH["uv"],
        "HF": flux > THRESH["flux"],
        "RGB": bool(rgb),
    }
    return {
        "d": round(dist, 3),
        "bearing": round(bearing, 4),
        "facing": round(facing, 3),
        "los": round(los, 2),
        "thermal": round(thermal, 1),
        "smoke": round(max(0.0, smoke), 3),
        "voc": round(voc, 1),
        "uv": uv,
        "flux": round(flux, 2),
        "rgb": rgb,
        "pile": pile,
        "activated": activated,
        "alarm": any(activated.values()),
        "located": activated["IR-TPA"] and activated["UV-FLAME"] and (activated["VOC"] or activated["RGB"]),
    }


def standoff_pose(fx: float, fy: float) -> Pose2D:
    """Face the fire from the west/south free side."""
    home = config.HOME
    dx = fx - home.x
    dy = fy - home.y
    n = math.hypot(dx, dy) or 1.0
    x = fx - STANDOFF * dx / n
    y = fy - STANDOFF * dy / n
    x = min(config.WORLD_W - 0.4, max(0.4, x))
    y = min(config.WORLD_H - 0.4, max(0.4, y))
    yaw = math.atan2(fy - y, fx - x)
    return Pose2D(x, y, yaw)


def patrol_waypoints() -> list[list[float]]:
    grid = plant.occupancy_grid()
    corners = [
        (1.2, 1.2),
        (1.2, 4.7),
        (5.4, 4.7),
        (5.4, 1.2),
        (7.2, 1.2),
        (7.2, 4.7),
        (5.4, 4.7),
    ]
    out: list[list[float]] = []
    prev = plant.pose_to_cell(config.HOME)
    for xy in corners:
        goal = plant.pose_to_cell(Pose2D(xy[0], xy[1], 0.0))
        path = astar.plan(grid, prev, goal)
        for ix, iy in path:
            out.append(list(plant.cell_center(ix, iy)))
        prev = goal
    return out


def nearest_site(x: float, y: float) -> dict:
    best = SITES[0]
    best_d = 1e9
    for s in SITES:
        d = math.hypot(s["xy"][0] - x, s["xy"][1] - y)
        if d < best_d:
            best, best_d = s, d
    return best


def snap_free(x: float, y: float, grid: np.ndarray | None = None) -> tuple[float, float] | None:
    """Clamp a click onto a free cell. None if the click is inside a wall/obstacle."""
    if grid is None:
        grid = plant.occupancy_grid()
    ny, nx = grid.shape
    ix = int(x / config.GRID)
    iy = int(y / config.GRID)
    if not (0 <= ix < nx and 0 <= iy < ny):
        return None
    if not grid[iy, ix]:
        return plant.cell_center(ix, iy)
    for r in range(1, 14):
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                jx, jy = ix + dx, iy + dy
                if 0 <= jx < nx and 0 <= jy < ny and not grid[jy, jx]:
                    return plant.cell_center(jx, jy)
    return None


def pack_fire() -> dict:
    return {
        "enabled": True,
        "ambient_c": AMBIENT_C,
        "core_c": CORE_C,
        "thermal_len": THERMAL_LEN,
        "smoke_len": SMOKE_LEN,
        "voc_k": VOC_K,
        "flux_k": FLUX_K,
        "uv_range": UV_RANGE,
        "rgb_range": RGB_RANGE,
        "standoff": STANDOFF,
        "thresh": THRESH,
        "sites": SITES,
        "actions": ACTIONS,
        "sensors": SENSORS,
        "patrol": patrol_waypoints(),
        "briefing": {
            "incident_id": "INC-1042",
            "when": "14:22 IST · Bay 3",
            "site": "Hydrocarbon cell · you ignite",
            "problem": (
                "You start the fire. RAVEN does not get the seat. It hunts with IR, "
                "optical smoke, PID VOC, UV flame, heat flux and wrist RGB, then you "
                "choose how to put it out."
            ),
            "solving": (
                "Locate the seat from sensors only. Mitigate with CO₂, foam, fuel "
                "isolation at V-17, or a human. Success = intensity 0 and sensors below trip."
            ),
        },
        "methods": [
            "Operator click on a free cell (or a marked pad) injects the fire",
            "Coverage patrol until IR > 55 °C or VOC > 80 ppm",
            "A* to a 0.95 m standoff facing the estimated seat",
            "Locate when IR + UV and (VOC or RGB) agree",
            "CO₂ / foam kill the pool. V-17 isolation only starves tank-side fires.",
        ],
    }
