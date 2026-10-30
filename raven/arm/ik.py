from __future__ import annotations

import numpy as np

from raven import config
from raven.arm import dh
from raven.arm.dh import _dh

_LAMBDA = 1e-3
_MAX_ITERS = 80
_W_ORI = 0.25


def _frames(q: np.ndarray) -> list[np.ndarray]:
    frames = [np.eye(4, dtype=np.float64)]
    T = frames[0]
    for qi, (d, a, alpha) in zip(q, config.DH, strict=True):
        T = T @ _dh(float(qi), d, a, alpha)
        frames.append(T)
    return frames


def _jacobian(q: np.ndarray, p_ee: np.ndarray) -> np.ndarray:
    frames = _frames(q)
    J = np.zeros((6, q.size), dtype=np.float64)
    for i in range(q.size):
        z = frames[i][:3, 2]
        o = frames[i][:3, 3]
        J[:3, i] = np.cross(z, p_ee - o)
        J[3:, i] = z
    return J


def solve(T_target: np.ndarray, q_seed: np.ndarray) -> tuple[np.ndarray, float]:
    q = np.array(q_seed, dtype=np.float64, copy=True)
    best_q = q.copy()
    best_res = float("inf")
    I6 = np.eye(6)
    for _ in range(_MAX_ITERS):
        T = dh.gripper_fk(q)
        p = T[:3, 3]
        res = float(np.linalg.norm(T_target[:3, 3] - p))
        if res < best_res:
            best_res = res
            best_q = q.copy()
        if res < 1e-4:
            break
        z_err = np.cross(T[:3, 2], T_target[:3, 2])
        e = np.concatenate([T_target[:3, 3] - p, _W_ORI * z_err])
        J = _jacobian(q, p)
        JJ = J @ J.T + (_LAMBDA ** 2) * I6
        dq = J.T @ np.linalg.solve(JJ, e)
        q = q + dq
    T = dh.gripper_fk(best_q)
    best_res = float(np.linalg.norm(T_target[:3, 3] - T[:3, 3]))
    return best_q, best_res

