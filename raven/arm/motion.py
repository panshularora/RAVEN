from __future__ import annotations

import numpy as np


def interpolate(q0: np.ndarray, q1: np.ndarray, n_steps: int) -> list[np.ndarray]:
    n = max(int(n_steps), 1)
    if n == 1:
        return [np.array(q1, dtype=np.float64, copy=True)]
    ts = np.linspace(0.0, 1.0, n)
    q0 = np.asarray(q0, dtype=np.float64)
    q1 = np.asarray(q1, dtype=np.float64)
    return [(1.0 - t) * q0 + t * q1 for t in ts]
