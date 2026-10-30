import numpy as np

from raven import config, plant
from raven.nav import astar


def test_home_to_approach_exists_and_free():
    g = plant.occupancy_grid()
    path = astar.plan(g, plant.pose_to_cell(config.HOME), plant.pose_to_cell(config.APPROACH))
    assert path[0] == plant.pose_to_cell(config.HOME)
    assert path[-1] == plant.pose_to_cell(config.APPROACH)
    for ix, iy in path:
        assert g[iy, ix] == False


def test_blocked_world_returns_empty():
    g = np.ones((10, 10), dtype=bool)
    g[1, 1] = False
    assert astar.plan(g, (1, 1), (8, 8)) == []
