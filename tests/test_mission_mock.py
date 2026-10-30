import sys

from raven.backends.mock import MockBackend
from raven.mission import run_mission
from raven.vision import inspect


def test_mock_starts_leaking_at_home():
    b = MockBackend()
    p = b.get_pose()
    assert abs(p.x - 1.0) < 1e-9 and abs(p.y - 1.0) < 1e-9
    assert b.get_leak() is True
    assert b.get_valve_angle() == 0.0


def test_mock_drive_moves_forward():
    b = MockBackend()
    b.set_wheel_speeds(4.0, 4.0)
    for _ in range(20):
        b.step(0.05)
    assert b.get_pose().x > 1.1


def test_camera_shows_leak():
    b = MockBackend()
    frame = b.get_camera_rgb()
    assert frame.shape == (64, 64, 3)
    assert inspect.inspect(frame).leak_seen is True


def test_run_mission_mock_done_and_valve_closed():
    b = MockBackend()
    r = run_mission(b)
    assert r.status == "DONE"
    assert 80.0 <= r.valve_angle <= 100.0


def test_fail_valve_human_assist():
    b = MockBackend(fail_valve=True)
    r = run_mission(b)
    assert r.status == "HUMAN_ASSIST"
    assert r.reason == "valve_not_closed"


def test_cli_no_view_done(monkeypatch):
    monkeypatch.setattr(sys, "argv", ["raven", "--no-view"])
    from raven.__main__ import main

    result = main()
    assert result.status == "DONE"
