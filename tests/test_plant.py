from raven import config, plant
from raven.types import Pose2D


def test_grid_shape_and_home_free():
    g = plant.occupancy_grid()
    assert g.shape == (60, 80)
    assert g.dtype == bool
    ix, iy = plant.pose_to_cell(config.HOME)
    assert g[iy, ix] == False


def test_approach_free_obstacle_occupied():
    g = plant.occupancy_grid()
    ax, ay = plant.pose_to_cell(config.APPROACH)
    assert g[ay, ax] == False
    ox, oy = plant.pose_to_cell(Pose2D(3.5, 3.0, 0.0))
    assert g[oy, ox] == True


def test_cell_center_home():
    ix, iy = plant.pose_to_cell(config.HOME)
    assert (ix, iy) == (10, 10)
    x, y = plant.cell_center(ix, iy)
    assert (x, y) == (1.05, 1.05)
