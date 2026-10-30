from __future__ import annotations

import math

import numpy as np

from raven import config
from raven.types import Pose2D

_WRIST_TURN = math.pi / 2 - 0.2


class MockWorld:
    def __init__(self, fail_valve: bool = False) -> None:
        self.pose = config.HOME
        self.joints = np.array(config.HOME_Q, dtype=np.float64, copy=True)
        self.joint_targets = self.joints.copy()
        self.wl = 0.0
        self.wr = 0.0
        self.gripper_closed = False
        self.leak = True
        self.valve_angle = 0.0
        self.fail_valve = fail_valve
        self.close_count = 0
        self._q5_at_grasp: float | None = None
        self._blocked_closes = 0

    def set_wheel_speeds(self, left: float, right: float) -> None:
        self.wl = float(left)
        self.wr = float(right)

    def set_joint_targets(self, q: np.ndarray) -> None:
        self.joint_targets = np.array(q, dtype=np.float64, copy=True)

    def set_gripper(self, closed: bool) -> None:
        was = self.gripper_closed
        self.gripper_closed = bool(closed)
        if closed and not was:
            self.close_count += 1
            self._q5_at_grasp = float(self.joints[4])
            if self.fail_valve and self._blocked_closes < 2:
                self._blocked_closes += 1

    def set_leak(self, leak: bool) -> None:
        self.leak = bool(leak)

    def apply_valve_from_arm(self) -> None:
        if not self.gripper_closed or self._q5_at_grasp is None:
            return
        if self.fail_valve and self._blocked_closes <= 2 and self.close_count <= 2:
            return
        if float(self.joints[4]) - self._q5_at_grasp >= _WRIST_TURN:
            self.valve_angle = 90.0
            self.leak = False

    def step(self, dt: float) -> None:
        v = config.WHEEL_R * (self.wr + self.wl) / 2.0
        w = config.WHEEL_R * (self.wr - self.wl) / config.TRACK
        yaw = self.pose.yaw + w * dt
        x = self.pose.x + v * math.cos(self.pose.yaw) * dt
        y = self.pose.y + v * math.sin(self.pose.yaw) * dt
        self.pose = Pose2D(x, y, yaw)
        self.joints = np.array(self.joint_targets, dtype=np.float64, copy=True)
        self.apply_valve_from_arm()

    def camera_rgb(self) -> np.ndarray:
        h, w = config.CAMERA_MOCK
        img = np.zeros((h, w, 3), dtype=np.uint8)
        img[:] = (30, 30, 36)
        if self.leak:
            yy, xx = np.ogrid[:h, :w]
            disk = (xx - 32) ** 2 + (yy - 16) ** 2 <= 8**2
            img[disk] = (220, 20, 20)
        img[48:58, 48:58] = (20, 200, 20)
        return img
