from __future__ import annotations

import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle

from raven import config, plant


class LiveView:
    """Matplotlib top-down plant + camera thumbnail wrapping a backend."""

    def __init__(self, inner) -> None:
        self.inner = inner
        self._closed = False
        self.fig, (self.ax, self.ax_cam) = plt.subplots(1, 2, figsize=(10, 5), width_ratios=[2.2, 1])
        self.fig.canvas.mpl_connect("close_event", self._on_close)
        self.ax.set_aspect("equal")
        self.ax.set_xlim(0, config.WORLD_W)
        self.ax.set_ylim(0, config.WORLD_H)
        self.ax.set_title("RAVEN plant")
        grid = plant.occupancy_grid()
        self.ax.imshow(
            grid,
            origin="lower",
            extent=(0, config.WORLD_W, 0, config.WORLD_H),
            cmap="Greys",
            alpha=0.35,
            interpolation="nearest",
        )
        tx, ty = config.TANK_XY
        self.ax.add_patch(Circle((tx, ty), 0.28, color="steelblue", zorder=3))
        vx, vy = config.VALVE_XY
        self.ax.add_patch(Circle((vx, vy), 0.10, color="limegreen", zorder=4))
        self.robot = Circle((config.HOME.x, config.HOME.y), config.FOOTPRINT_R, color="tab:orange", zorder=5)
        self.ax.add_patch(self.robot)
        self.heading, = self.ax.plot([], [], color="k", lw=2, zorder=6)
        self.badge = self.ax.text(0.15, config.WORLD_H - 0.3, "", fontsize=11, color="crimson")
        self.status = self.ax.text(0.15, 0.25, "", fontsize=10)
        self.cam_im = self.ax_cam.imshow(np.zeros((*config.CAMERA_MOCK, 3), dtype=np.uint8))
        self.ax_cam.set_title("wrist camera")
        self.ax_cam.axis("off")
        plt.tight_layout()
        plt.ion()
        self.fig.show()

    def _on_close(self, _event) -> None:
        self._closed = True

    def _draw(self) -> None:
        if self._closed:
            sys.exit(0)
        pose = self.inner.get_pose()
        self.robot.center = (pose.x, pose.y)
        hx = pose.x + config.FOOTPRINT_R * np.cos(pose.yaw)
        hy = pose.y + config.FOOTPRINT_R * np.sin(pose.yaw)
        self.heading.set_data([pose.x, hx], [pose.y, hy])
        leak = self.inner.get_leak()
        self.badge.set_text("LEAK" if leak else "SEALED")
        self.badge.set_color("crimson" if leak else "forestgreen")
        self.status.set_text(f"valve {self.inner.get_valve_angle():.0f}°")
        self.cam_im.set_data(self.inner.get_camera_rgb())
        self.fig.canvas.draw_idle()
        plt.pause(0.001)

    def step(self, dt: float) -> None:
        self.inner.step(dt)
        self._draw()

    def close(self) -> None:
        self.inner.close()
        plt.close(self.fig)

    def get_pose(self):
        return self.inner.get_pose()

    def set_wheel_speeds(self, left: float, right: float) -> None:
        self.inner.set_wheel_speeds(left, right)

    def get_joints(self):
        return self.inner.get_joints()

    def set_joint_targets(self, q) -> None:
        self.inner.set_joint_targets(q)

    def set_gripper(self, closed: bool) -> None:
        self.inner.set_gripper(closed)

    def get_camera_rgb(self):
        return self.inner.get_camera_rgb()

    def get_leak(self) -> bool:
        return self.inner.get_leak()

    def get_valve_angle(self) -> float:
        return self.inner.get_valve_angle()
