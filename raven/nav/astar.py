from __future__ import annotations

import heapq
import math

import numpy as np

_NEIGHBORS = (
    (-1, -1), (0, -1), (1, -1),
    (-1, 0),           (1, 0),
    (-1, 1),  (0, 1),  (1, 1),
)


def _octile(a: tuple[int, int], b: tuple[int, int]) -> float:
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)


def plan(
    grid: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
) -> list[tuple[int, int]]:
    ny, nx = grid.shape
    sx, sy = start
    gx, gy = goal
    if not (0 <= sx < nx and 0 <= sy < ny and 0 <= gx < nx and 0 <= gy < ny):
        return []
    if grid[sy, sx] or grid[gy, gx]:
        return []
    if start == goal:
        return [start]

    inf = float("inf")
    gscore = {start: 0.0}
    came: dict[tuple[int, int], tuple[int, int]] = {}
    heap: list[tuple[float, int, tuple[int, int]]] = []
    tie = 0
    heapq.heappush(heap, (_octile(start, goal), tie, start))
    closed: set[tuple[int, int]] = set()

    while heap:
        _, _, current = heapq.heappop(heap)
        if current in closed:
            continue
        if current == goal:
            path = [current]
            while current in came:
                current = came[current]
                path.append(current)
            path.reverse()
            return path
        closed.add(current)
        cx, cy = current
        for dx, dy in _NEIGHBORS:
            nx_, ny_ = cx + dx, cy + dy
            if not (0 <= nx_ < nx and 0 <= ny_ < ny):
                continue
            if grid[ny_, nx_]:
                continue
            step = math.sqrt(2) if dx and dy else 1.0
            tentative = gscore[current] + step
            neigh = (nx_, ny_)
            if tentative >= gscore.get(neigh, inf):
                continue
            came[neigh] = current
            gscore[neigh] = tentative
            tie += 1
            heapq.heappush(heap, (tentative + _octile(neigh, goal), tie, neigh))
    return []
