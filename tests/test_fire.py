import math

from raven.sim.fire import (
    nearest_site,
    pack_fire,
    sample_sensors,
    snap_free,
    standoff_pose,
)
from raven.sim.story import pack_story


def test_ambient_is_quiet_far_from_a_dead_fire():
    s = sample_sensors(1.0, 1.0, 0.0, 6.2, 4.2, intensity=0.0)
    assert s["thermal"] < 35
    assert s["voc"] < 5
    assert s["rgb"] == 0
    assert s["alarm"] is False
    assert s["located"] is False


def test_close_facing_fire_trips_ir_uv_and_locate():
    s = sample_sensors(5.4, 4.2, 0.0, 6.2, 4.2, intensity=1.0, t=0.2)
    assert s["thermal"] > 55
    assert s["uv"] == 1.0
    assert s["voc"] > 80
    assert s["located"] is True
    assert s["activated"]["IR-TPA"] is True


def test_looking_away_drops_uv_and_rgb():
    s = sample_sensors(5.4, 4.2, math.pi, 6.2, 4.2, intensity=1.0)
    assert s["uv"] == 0.0
    assert s["rgb"] == 0
    assert s["thermal"] > 40


def test_obstacle_blocks_line_of_sight():
    clear = sample_sensors(7.2, 4.2, math.pi, 6.2, 4.2, 1.0)
    blocked = sample_sensors(1.2, 3.0, 0.0, 6.2, 3.0, 1.0)
    assert clear["los"] == 1.0
    assert blocked["los"] == 0.12
    assert blocked["thermal"] < clear["thermal"]


def test_snap_rejects_the_aisle_obstacle_and_keeps_free_cells():
    assert snap_free(1.2, 1.2) is not None
    hit = snap_free(3.5, 3.0)
    assert hit is not None
    assert abs(hit[0] - 3.5) > 0.3 or abs(hit[1] - 3.0) > 0.3


def test_standoff_faces_the_seat():
    p = standoff_pose(6.2, 4.2)
    assert math.hypot(p.x - 6.2, p.y - 4.2) > 0.7
    bearing = math.atan2(4.2 - p.y, 6.2 - p.x)
    err = abs((bearing - p.yaw + math.pi) % (2 * math.pi) - math.pi)
    assert err < 0.05


def test_nearest_site_picks_tank():
    s = nearest_site(6.1, 4.1)
    assert s["id"] == "tank"


def test_pack_story_carries_the_fire_bundle():
    packed = pack_story()
    fire = packed["fire"]
    assert fire["enabled"] is True
    assert len(fire["sites"]) >= 4
    assert len(fire["actions"]) == 4
    assert {a["id"] for a in fire["actions"]} == {"co2", "foam", "isolate", "human"}
    assert fire["patrol"]
    assert fire["thresh"]["thermal"] == 55.0
    bundle = pack_fire()
    assert bundle["sensors"][0]["id"] == "IR-TPA"
