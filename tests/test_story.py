from raven.backends.mock import MockBackend
from raven.mission import run_mission
from raven.sim.story import pack_story


def test_inspect_only_holds_with_valve_open():
    r = run_mission(MockBackend(), after_inspect="inspect_only")
    assert r.status == "DONE"
    assert r.reason == "leak_confirmed"
    assert r.valve_angle == 0.0


def test_operator_human_does_not_close_valve():
    b = MockBackend()
    r = run_mission(b, after_inspect="human")
    assert r.status == "HUMAN_ASSIST"
    assert r.reason == "operator_request"
    assert b.get_leak() is True
    assert b.get_valve_angle() == 0.0


def test_pack_story_has_map_gate_and_four_orders():
    packed = pack_story()
    assert packed["mode"] == "story"
    assert packed["gate_t"] > 1.0
    assert set(packed["branches"]) == {"close", "inspect_only", "human", "stuck"}
    assert packed["branches"]["close"]["result"]["status"] == "DONE"
    assert packed["branches"]["inspect_only"]["result"]["reason"] == "leak_confirmed"
    assert packed["branches"]["human"]["result"]["reason"] == "operator_request"
    assert packed["branches"]["stuck"]["result"]["status"] == "HUMAN_ASSIST"
    assert packed["grid"]["nx"] > 10
    assert packed["frames"][0]["rays"]
    assert packed["briefing"]["incident_id"] == "INC-1042"
    assert len(packed["actions"]) == 4
    assert packed["inspect"]["leak_seen"] is True
    last = None
    for f in packed["frames"]:
        if f["t"] <= packed["gate_t"]:
            last = f
    assert last is not None
    assert last["phase"] == "INSPECT"
