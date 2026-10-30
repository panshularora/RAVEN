import numpy as np

from raven.types import Pose2D

WORLD_W = 8.0
WORLD_H = 6.0
GRID = 0.10
INFLATION = 0.35

HOME = Pose2D(1.0, 1.0, 0.0)
APPROACH = Pose2D(5.2, 3.35, 0.0)
VALVE_XY = (6.2, 3.35)
TANK_XY = (6.2, 4.2)
VALVE_Z = 0.25
OBSTACLE_CENTER = (3.5, 3.0)
OBSTACLE_SIZE = (0.6, 2.0)

TRACK = 0.30
WHEEL_R = 0.05
FOOTPRINT_R = 0.25

# (d, a, alpha) per joint; theta comes from q
DH = [
    (0.12, 0.00, np.pi / 2),
    (0.00, 0.22, 0.0),
    (0.00, 0.18, 0.0),
    (0.00, 0.00, np.pi / 2),
    (0.12, 0.00, 0.0),
]
GRIPPER_Z = 0.06
HOME_Q = np.array([0.0, -0.4, 0.8, 0.0, 0.0], dtype=np.float64)

VALVE_CLOSED = (80.0, 100.0)
NAV_POS_TOL = 0.12
NAV_YAW_TOL = np.deg2rad(15)
NAV_TIMEOUT_S = 60.0
IK_RESIDUAL_MAX = 0.02
PREGRASP_X = 0.08
DT = 0.05
CMD_HZ = 20
CAMERA_MOCK = (64, 64)
GRID_NY = int(WORLD_H / GRID)
GRID_NX = int(WORLD_W / GRID)
