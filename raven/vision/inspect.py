from __future__ import annotations

import cv2
import numpy as np

from raven.types import InspectResult

_S_MIN = 80
_V_MIN = 80


def _centroid(mask: np.ndarray) -> tuple[float, float] | None:
    ys, xs = np.nonzero(mask)
    if xs.size == 0:
        return None
    return (float(xs.mean()), float(ys.mean()))


def inspect(frame: np.ndarray) -> InspectResult:
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    red_lo = cv2.inRange(hsv, (0, _S_MIN, _V_MIN), (10, 255, 255))
    red_hi = cv2.inRange(hsv, (170, _S_MIN, _V_MIN), (179, 255, 255))
    red = cv2.bitwise_or(red_lo, red_hi)
    green = cv2.inRange(hsv, (40, _S_MIN, _V_MIN), (80, 255, 255))
    leak_c = _centroid(red)
    handle_c = _centroid(green)
    return InspectResult(
        leak_seen=leak_c is not None,
        leak_centroid=leak_c,
        handle_seen=handle_c is not None,
        handle_centroid=handle_c,
    )
