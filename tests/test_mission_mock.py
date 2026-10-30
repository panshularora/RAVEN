from raven.backends.mock import MockBackend
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
