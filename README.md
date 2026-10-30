# RAVEN

Robotic Autonomous Verification & Emergency Navigation — simulated industrial leak inspection and isolation-valve intervention.

Python 3.13, NumPy, OpenCV, matplotlib. CoppeliaSim Edu is optional for the 3D plant.

## Setup

```
pip install -r requirements.txt
```

## Mock demo (no CoppeliaSim)

```
python -m raven
python -m raven --fail-valve
python -m raven --no-view
```

`--no-view` skips the matplotlib window (used by tests). `--fail-valve` forces two failed closes, then `HUMAN_ASSIST`.

## CoppeliaSim 3D demo

1. Install CoppeliaSim Edu (already at `C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu` on this PC).
2. `pip install coppeliasim-zmqremoteapi-client`
3. Open CoppeliaSim Edu (ZeroMQ remote API, port 23000).
4. `python -m raven --backend coppelia`

The Python client builds the plant and robot if `/RAVEN` is missing, then runs the same mission FSM.

If CoppeliaSim is open but the command still prints the start hint, close any first-run or license window, then start **Add-ons → ZMQ remote API server** (port 23000).

## Tests

```
python -m pytest tests -q
```
