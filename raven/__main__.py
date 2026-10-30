from __future__ import annotations

import argparse
import sys

from raven.backends.mock import MockBackend
from raven.mission import run_mission
from raven.types import MissionResult


def make_backend(name: str, fail_valve: bool):
    if name == "mock":
        return MockBackend(fail_valve=fail_valve)
    if name == "coppelia":
        from raven.backends.coppelia import CoppeliaBackend

        return CoppeliaBackend.connect()
    raise ValueError(f"unknown backend {name}")


def main() -> MissionResult:
    parser = argparse.ArgumentParser(prog="raven")
    parser.add_argument("--backend", choices=("mock", "coppelia"), default="mock")
    parser.add_argument("--fail-valve", action="store_true")
    parser.add_argument("--no-view", action="store_true")
    args = parser.parse_args()
    backend = make_backend(args.backend, args.fail_valve)
    if not args.no_view:
        from raven.sim.view import LiveView

        backend = LiveView(backend)
    try:
        result = run_mission(backend)
    finally:
        backend.close()
    print(
        f"{result.status} reason={result.reason} "
        f"attempts={result.attempts} valve={result.valve_angle:.1f}"
    )
    return result


if __name__ == "__main__":
    main()
