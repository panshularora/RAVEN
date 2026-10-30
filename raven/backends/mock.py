from __future__ import annotations

import numpy as np

from raven.sim.mock_world import MockWorld
from raven.types import Pose2D


class MockBackend:
    def __init__(self, fail_valve: bool = False) -> None:
        self.world = MockWorld(fail_valve=fail_valve)

    def get_pose(self) -> Pose2D:
        return self.world.pose

    def set_wheel_speeds(self, left: float, right: float) -> None:
        self.world.set_wheel_speeds(left, right)

    def get_joints(self) -> np.ndarray:
        return np.array(self.world.joints, dtype=np.float64, copy=True)

    def set_joint_targets(self, q: np.ndarray) -> None:
        self.world.set_joint_targets(q)

    def set_gripper(self, closed: bool) -> None:
        self.world.set_gripper(closed)

    def get_camera_rgb(self) -> np.ndarray:
        return self.world.camera_rgb()

    def get_leak(self) -> bool:
        return self.world.leak

    def get_valve_angle(self) -> float:
        return self.world.valve_angle

    def set_leak(self, leak: bool) -> None:
        self.world.set_leak(leak)

    def step(self, dt: float) -> None:
        self.world.step(dt)

    def close(self) -> None:
        return
