from __future__ import annotations

import numpy as np

from raven import config


def _dh(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def fk(q: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    for qi, (d, a, alpha) in zip(q, config.DH, strict=True):
        T = T @ _dh(float(qi), d, a, alpha)
    return T


def gripper_fk(q: np.ndarray) -> np.ndarray:
    T = fk(q)
    grip = np.eye(4, dtype=np.float64)
    grip[2, 3] = config.GRIPPER_Z
    return T @ grip
