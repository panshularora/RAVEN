from __future__ import annotations

import math

import numpy as np

from raven import config
from raven.arm.dh import fk_prefix, gripper_fk
from raven.sim.mock_world import MockWorld
from raven.sim.scene_builder import ensure_scene
from raven.types import Pose2D


class CoppeliaUnavailable(Exception):
    pass


def _quat_from_R(R: np.ndarray) -> list[float]:
    tr = float(np.trace(R))
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return [qx, qy, qz, qw]


def _pose7(T: np.ndarray) -> list[float]:
    x, y, z = (float(v) for v in T[:3, 3])
    return [x, y, z, *_quat_from_R(T[:3, :3])]


def _base_T(pose: Pose2D) -> np.ndarray:
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    T = np.eye(4, dtype=np.float64)
    T[0, 0], T[0, 1], T[0, 3] = c, -s, pose.x
    T[1, 0], T[1, 1], T[1, 3] = s, c, pose.y
    T[2, 3] = 0.07
    return T


class CoppeliaBackend:
    def __init__(self, sim, client, world: MockWorld) -> None:
        self.sim = sim
        self.client = client
        self.world = world
        self._handles()
        self._push()

    def _handles(self) -> None:
        sim = self.sim
        self.h_base = sim.getObject("/RAVEN/base")
        self.h_links = [sim.getObject(f"/RAVEN/arm/l{i}") for i in range(1, 6)]
        self.h_grip = sim.getObject("/RAVEN/gripper")
        self.h_cam = sim.getObject("/RAVEN/camera")
        self.h_valve = sim.getObject("/Plant/valve")
        self.h_leak = sim.getObject("/Plant/leak")

    @classmethod
    def connect(cls, fail_valve: bool = False) -> CoppeliaBackend:
        try:
            import zmq
            from coppeliasim_zmqremoteapi_client import RemoteAPIClient

            client = RemoteAPIClient()
            client.socket.setsockopt(zmq.RCVTIMEO, 2500)
            client.socket.setsockopt(zmq.SNDTIMEO, 2500)
            client.socket.setsockopt(zmq.LINGER, 0)
            sim = client.require("sim")
            sim.getSimulationTime()
            ensure_scene(sim)
            if sim.getSimulationState() == sim.simulation_stopped:
                sim.setStepping(True)
                sim.startSimulation()
            return cls(sim, client, MockWorld(fail_valve=fail_valve))
        except CoppeliaUnavailable:
            raise
        except Exception as exc:
            raise CoppeliaUnavailable(str(exc)) from exc

    def _push(self) -> None:
        sim = self.sim
        pose = self.world.pose
        Tb = _base_T(pose)
        sim.setObjectPose(self.h_base, _pose7(Tb))
        T_grip = gripper_fk(self.world.joints)
        Tg = np.eye(4)
        Tg[:3, :3] = Tb[:3, :3] @ T_grip[:3, :3]
        Tg[:3, 3] = Tb[:3, 3] + Tb[:3, :3] @ T_grip[:3, 3]
        sim.setObjectPose(self.h_grip, _pose7(Tg))
        try:
            sim.setObjectPose(self.h_cam, _pose7(Tg))
        except Exception:
            pass
        for i, h in enumerate(self.h_links):
            Ti = fk_prefix(self.world.joints, i + 1)
            Tw = np.eye(4)
            Tw[:3, :3] = Tb[:3, :3] @ Ti[:3, :3]
            Tw[:3, 3] = Tb[:3, 3] + Tb[:3, :3] @ Ti[:3, 3]
            sim.setObjectPose(h, _pose7(Tw))
        try:
            sim.setJointPosition(self.h_valve, math.radians(self.world.valve_angle))
        except Exception:
            pass
        leak_z = config.VALVE_Z if self.world.leak else -1.0
        tx, ty = config.TANK_XY
        sim.setObjectPose(self.h_leak, [tx, ty - 0.28, leak_z, 0, 0, 0, 1])
        try:
            sim.setInt32Signal("leakActive", 1 if self.world.leak else 0)
        except Exception:
            pass
        try:
            sim.step()
        except Exception:
            pass

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

    def step(self, dt: float) -> None:
        self.world.step(dt)
        self._push()

    def close(self) -> None:
        try:
            self.sim.stopSimulation()
        except Exception:
            pass
