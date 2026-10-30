import math

from raven.nav.range import MAX_RANGE, range_rays
from raven.types import Pose2D


def test_forward_ray_hits_aisle_from_south():
    rays = range_rays(Pose2D(3.5, 1.2, math.pi / 2))
    assert len(rays) == 5
    assert rays[2] < MAX_RANGE
    assert rays[2] > 0.2


def test_ray_into_wall_is_short():
    rays = range_rays(Pose2D(0.3, 3.0, 3.1416))
    assert rays[2] < 0.5
