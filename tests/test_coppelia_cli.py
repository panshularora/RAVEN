import sys

import pytest

from raven.backends.coppelia import CoppeliaUnavailable
from raven.__main__ import main


def test_connect_failure_prints_hint(monkeypatch, capsys):
    monkeypatch.setattr(sys, "argv", ["raven", "--backend", "coppelia", "--no-view"])

    def boom(*_a, **_k):
        raise CoppeliaUnavailable("down")

    monkeypatch.setattr("raven.__main__.make_backend", boom)
    with pytest.raises(SystemExit) as e:
        main()
    assert e.value.code == 1
    out = capsys.readouterr().out
    assert "Start CoppeliaSim Edu, then retry --backend coppelia" in out
    assert "down" in out
    assert "ZMQ remote API" in out


def test_make_backend_passes_fail_valve(monkeypatch):
    seen: dict = {}

    def fake_connect(*, fail_valve=False):
        seen["fail_valve"] = fail_valve
        raise CoppeliaUnavailable("skip")

    monkeypatch.setattr("raven.backends.coppelia.CoppeliaBackend.connect", fake_connect)
    from raven.__main__ import make_backend

    with pytest.raises(CoppeliaUnavailable):
        make_backend("coppelia", True)
    assert seen.get("fail_valve") is True
