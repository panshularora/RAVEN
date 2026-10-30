from __future__ import annotations

import argparse

from raven.backends.coppelia import CoppeliaUnavailable
from raven.backends.mock import MockBackend
from raven.mission import run_mission
from raven.sim.story import pack_story
from raven.types import MissionResult


def make_backend(name: str, fail_valve: bool):
    if name == "mock":
        return MockBackend(fail_valve=fail_valve)
    if name == "coppelia":
        from raven.backends.coppelia import CoppeliaBackend

        return CoppeliaBackend.connect(fail_valve=fail_valve)
    raise ValueError(f"unknown backend {name}")


def main() -> MissionResult:
    parser = argparse.ArgumentParser(
        prog="raven",
        description=(
            "RAVEN — story-mode leak isolation. Default run opens a live control room "
            "with a plant map, sensors, and on-scene orders."
        ),
    )
    parser.add_argument("--backend", choices=("mock", "coppelia"), default="mock")
    parser.add_argument("--fail-valve", action="store_true")
    parser.add_argument(
        "--no-view",
        action="store_true",
        help="headless: print the result and exit (used by tests)",
    )
    parser.add_argument("--port", type=int, default=8765, help="control-room HTTP port")
    args = parser.parse_args()

    if args.no_view:
        try:
            backend = make_backend(args.backend, args.fail_valve)
        except CoppeliaUnavailable as exc:
            _coppelia_fail(exc)
        try:
            result = run_mission(backend)
        finally:
            backend.close()
        print(
            f"{result.status} reason={result.reason} "
            f"attempts={result.attempts} valve={result.valve_angle:.1f}",
            flush=True,
        )
        return result

    if args.backend == "coppelia":
        try:
            backend = make_backend("coppelia", args.fail_valve)
        except CoppeliaUnavailable as exc:
            _coppelia_fail(exc)
        rec = None
        try:
            from raven.sim.recorder import MissionRecorder, pack_mission

            rec = MissionRecorder(backend)
            result = run_mission(rec)
        finally:
            backend.close()
        packed = pack_mission(rec, result)
    else:
        packed = pack_story()
        r = packed["result"]
        result = MissionResult(r["status"], r["reason"], r["attempts"], r["valve_angle"])

    print(
        f"{result.status} reason={result.reason} "
        f"attempts={result.attempts} valve={result.valve_angle:.1f}",
        flush=True,
    )
    from raven.sim.studio import start_studio

    print()
    print("Incident INC-1042 is loaded. Open the control room, read the ticket, deploy.")
    httpd, url = start_studio(packed, port=args.port, open_browser=True, block=False)
    print(f"Control room: {url}", flush=True)
    print("Ctrl+C to stop.", flush=True)
    try:
        import time

        while True:
            time.sleep(0.25)
    except KeyboardInterrupt:
        print("\nstopped")
        httpd.shutdown()
        httpd.server_close()
    return result


def _coppelia_fail(exc: CoppeliaUnavailable) -> None:
    print("Start CoppeliaSim Edu, then retry --backend coppelia")
    if str(exc):
        print(str(exc))
    print(
        "If CoppeliaSim is already open: dismiss first-run windows, then "
        "Add-ons → ZMQ remote API server (port 23000)."
    )
    raise SystemExit(1) from None


if __name__ == "__main__":
    main()
