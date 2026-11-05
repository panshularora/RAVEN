import json
import math
import urllib.request

from raven.backends.mock import MockBackend
from raven.mission import run_mission
from raven.sim.frames import forward_xz, plant_to_three, yaw_to_three
from raven.sim.recorder import MissionRecorder, pack_mission
from raven.sim.studio import start_studio, web_root


def test_yaw0_faces_plus_x():
    y = yaw_to_three(0.0)
    assert y == math.pi / -2
    fx, fz = forward_xz(0.0)
    assert abs(fx - 1.0) < 1e-9
    assert abs(fz) < 1e-9


def test_yaw_north_faces_plus_z():
    fx, fz = forward_xz(math.pi / 2)
    assert abs(fx) < 1e-9
    assert abs(fz - 1.0) < 1e-9


def test_plant_to_three_swaps_height_onto_y():
    assert plant_to_three(6.2, 3.35, 0.25) == (6.2, 0.25, 3.35)


def test_recorder_tells_the_mission_story():
    rec = MissionRecorder(MockBackend())
    result = run_mission(rec)
    assert result.status == "DONE"
    packed = pack_mission(rec, result)
    phases = [f["phase"] for f in packed["frames"]]
    assert "NAVIGATE" in phases
    assert "INSPECT" in phases
    assert "INTERVENE" in phases
    assert "VERIFY" in phases
    assert packed["frames"][-1]["phase"] == "DONE"
    assert packed["frames"][0]["leak"] == 1
    assert packed["frames"][-1]["leak"] == 0
    assert packed["frames"][-1]["valve"] >= 80.0
    assert packed["frames"][-1]["x"] > 4.5
    assert len(packed["frames"]) > 80
    assert packed["path"]
    assert len(packed["paths"]) >= 2
    assert packed["paths"][0]["pts"][0][0] < 2.0
    assert packed["world"]["w"] == 8.0
    assert "leaking" in packed["blurb"].lower()
    json.dumps(packed)


def test_fail_valve_recording_asks_for_a_human():
    rec = MissionRecorder(MockBackend(fail_valve=True))
    result = run_mission(rec)
    packed = pack_mission(rec, result)
    assert packed["result"]["status"] == "HUMAN_ASSIST"
    assert packed["frames"][-1]["phase"] == "HUMAN_ASSIST"
    assert packed["result"]["attempts"] == 2


def test_studio_serves_page_and_mission():
    rec = MissionRecorder(MockBackend())
    result = run_mission(rec)
    packed = pack_mission(rec, result)
    httpd, url = start_studio(packed, port=0, open_browser=False, block=False)
    try:
        html = urllib.request.urlopen(url, timeout=3).read().decode("utf-8")
        assert "RAVEN" in html
        assert "three" in html.lower()
        assert "INC-1042" in html
        assert "minimap" in html
        assert "order-list" in html
        assert "Deploy RAVEN" in html
        raw = urllib.request.urlopen(url.rstrip("/") + "/mission.json", timeout=3).read()
        data = json.loads(raw)
        assert data["result"]["status"] == "DONE"
        assert data["frames"][-1]["leak"] == 0
        js = urllib.request.urlopen(url.rstrip("/") + "/studio.js", timeout=3).read().decode("utf-8")
        assert "yawToThree" in js
        assert "chooseOrder" in js
        assert "drawMinimap" in js
        css = urllib.request.urlopen(url.rstrip("/") + "/studio.css", timeout=3).read().decode("utf-8")
        assert "--amber" in css
    finally:
        httpd.shutdown()
        httpd.server_close()


def test_web_root_has_the_control_room():
    root = web_root()
    assert (root / "index.html").is_file()
    assert (root / "studio.js").is_file()
    assert (root / "studio.css").is_file()
    sensors = (root / "sensors.html").read_text(encoding="utf-8")
    analytics = (root / "analytics.html").read_text(encoding="utf-8")
    assert "Sensor lab" in sensors
    assert "IR thermopile" in sensors
    assert "Graph-based sensor fusion" in analytics
    assert "located = IR-TPA" in analytics
    html = (root / "index.html").read_text(encoding="utf-8")
    assert "sensors.html" in html
    assert "analytics.html" in html
    assert "cam-bar" in html
    js = (root / "studio.js").read_text(encoding="utf-8")
    assert "minDistance = 1.05" in js
    assert "KeyW" in js
