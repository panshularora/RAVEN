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
    assert "Start CoppeliaSim Edu, then retry --backend coppelia" in capsys.readouterr().out
