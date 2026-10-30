from raven.backends.mock import MockBackend
from raven.mission import run_mission


def test_success_sequence_done():
    b = MockBackend()
    result = run_mission(b)
    assert result.status == "DONE"
    assert 80.0 <= result.valve_angle <= 100.0
    assert b.get_leak() is False


def test_two_failed_verifies_human_assist():
    b = MockBackend(fail_valve=True)
    result = run_mission(b)
    assert result.status == "HUMAN_ASSIST"
    assert result.reason == "valve_not_closed"
    assert result.attempts == 2


def test_nav_timeout_human_assist():
    b = MockBackend()
    calls = {"n": 0}

    def now2():
        calls["n"] += 1
        return 0.0 if calls["n"] < 3 else 61.0

    result = run_mission(b, now=now2)
    assert result.status == "HUMAN_ASSIST"
    assert result.reason == "nav_timeout"


def test_false_alarm_done():
    b = MockBackend()
    b.set_leak(False)
    result = run_mission(b)
    assert result.status == "DONE"
    assert result.reason == "false_alarm"


def test_ik_failed_human_assist():
    b = MockBackend()

    def bad_ik(T, q):
        return q, 0.05

    result = run_mission(b, ik_solver=bad_ik)
    assert result.status == "HUMAN_ASSIST"
    assert result.reason == "ik_failed"
