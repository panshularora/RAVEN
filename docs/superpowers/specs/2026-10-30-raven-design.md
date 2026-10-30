# RAVEN — Design Spec

**Date:** 2026-10-30  
**Project:** Robotic Autonomous Verification & Emergency Navigation  
**Status:** Draft for review  
**Stack:** Python 3.13 + CoppeliaSim Edu (optional 3D) + matplotlib mock backend

## 1. Intent

RAVEN is a simulation-only industrial incident robot. When a tank leak is injected, the robot navigates to the equipment, inspects with a camera, plans an arm motion, turns an isolation valve, and verifies the leak is gone. If the intervention fails twice, it stops and requests human assistance.

This is a working CoppeliaSim + Python demo for 21MEO106T. No physical hardware. CoppeliaSim Edu is optional for the 3D plant view; the same mission runs on a Python mock so development and tests do not wait on the GUI.

## 2. Non-goals

- Multi-robot, multi-incident, or multi-valve missions
- ROS
- Real hardware, real gas sensors, or plant historians
- Returning to home after success
- Learned / RL policies
- A web UI
- Full 6-DOF Cartesian impedance control

## 3. Architecture

One mission brain in Python. Two backends that implement the same plant/robot interface.

```
Alarm (leak flag)
    → Mission FSM
        → navigator (grid + A* + diff-drive follow)
        → vision (OpenCV leak + valve handle)
        → arm (DH FK + damped-least-squares IK + joint interpolation)
        → gripper (open / close)
    → Backend
        → mock (matplotlib top-down + synthetic camera)
        → coppelia (ZeroMQ remote API, default port 23000)
```

CLI:

```
python -m raven                  # mock backend (default)
python -m raven --backend coppelia
python -m raven --fail-valve     # force verify failure path (human assist)
```

## 4. Plant and robot

World is 8.0 m × 6.0 m, occupancy grid 0.10 m cells, origin at southwest corner, x east, y north.

| Object | Pose / size | Notes |
|---|---|---|
| Home | (1.0, 1.0, yaw=0) | robot spawn |
| Aisle obstacle | cuboid centered (3.5, 3.0), 0.6 × 2.0 m | inflates 0.35 m in the grid |
| Tank | (6.2, 4.2) | leak source; red visual cue when leaking |
| Isolation valve | (6.2, 3.35) | handle is a green cylinder; closed = +90° about vertical |
| Approach pose | (5.2, 3.35, yaw=0) | A* goal; robot faces +x toward valve |
| Walls | world border | occupied cells |

Robot:

- Differential-drive base, track 0.30 m, wheel radius 0.05 m, footprint radius 0.25 m
- 5-DOF arm on the base: yaw, shoulder, elbow, wrist pitch, wrist roll
- Parallel gripper on the wrist
- RGB camera on the wrist (64×64 mock; Coppelia vision sensor ≥128×128)
- Five forward range rays for bumper/obstacle sanity (mock: ray-grid; Coppelia: proximity)

DH table (metres, radians), applied in `raven/config.py` as the single source of truth:

| Joint | θ | d | a | α |
|---|---|---|---|---|
| 1 yaw | q1 | 0.12 | 0.00 | +π/2 |
| 2 shoulder | q2 | 0.00 | 0.22 | 0 |
| 3 elbow | q3 | 0.00 | 0.18 | 0 |
| 4 wrist pitch | q4 | 0.00 | 0.00 | +π/2 |
| 5 wrist roll | q5 | 0.12 | 0.00 | 0 |

Gripper offset along joint-5 z: 0.06 m. Home arm: all qi = 0 except q2 = −0.4, q3 = 0.8 (folded, clear of the base).

Valve closed when handle angle ∈ [80°, 100°] from the open (0°) position. Leak clears only when the valve is closed.

## 5. Components and files

Keep the existing course handout. New tree:

```
raven/
  __init__.py
  __main__.py              CLI, backend factory, matplotlib loop
  config.py                numbers above; no magic numbers elsewhere
  types.py                 Pose2D, Joints, Alarm, InspectResult, MissionResult
  plant.py                 layout → occupancy grid, named poses
  mission.py               FSM + recovery policy
  nav/astar.py             8-connected A* on inflated grid
  nav/controller.py        unicycle follow: heading then drive
  arm/dh.py                homogeneous FK
  arm/ik.py                damped least squares to handle pose
  arm/motion.py            linear joint interpolation at 20 Hz command rate
  vision/inspect.py        HSV leak (red) + handle (green); centroids
  backends/base.py         RobotBackend protocol
  backends/mock.py
  backends/coppelia.py
  sim/mock_world.py        kinematics, leak/valve state, synthetic camera
  sim/scene_builder.py     Coppelia primitives if scene empty
tests/
  test_astar.py
  test_dh.py
  test_ik.py
  test_vision.py
  test_fsm.py
  test_mission_mock.py
requirements.txt
README.md
docs/superpowers/specs/2026-10-30-raven-design.md
```

`RobotBackend` methods (all backends implement exactly these):

- `get_pose() -> Pose2D`
- `set_wheel_speeds(left, right)`
- `get_joints() / set_joint_targets(q)`
- `set_gripper(closed: bool)`
- `get_camera_rgb() -> np.ndarray HxWx3 uint8`
- `get_leak() -> bool` (plant ground truth, used only by VERIFY and by the mock renderer; vision uses the image)
- `get_valve_angle() -> float` degrees
- `step(dt)` advance sim
- `close()`

CoppeliaSim backend never reads `get_leak()` from a Lua cheat except as a named scene signal `leakActive` that the scene builder sets from valve angle. Vision still has to see the leak in the image during INSPECT.

## 6. Data flow

Mission starts with leak injected (`leakActive = true`, valve angle = 0).

1. **IDLE → NAVIGATE**  
   `plant.approach_pose` is the goal. `astar.plan(robot_cell, goal_cell)` returns a cell path. Controller converts to waypoints in metres and commands wheel speeds until pose error < 0.12 m and heading error < 15°.

2. **NAVIGATE → INSPECT**  
   Arm goes to inspect joints (config). Camera frame in. `vision.inspect(frame)` returns `{leak_seen: bool, leak_centroid, handle_seen: bool, handle_centroid}`.  
   `handle_seen` is logged only; the valve pose for IK comes from `plant` (known equipment layout).  
   If `leak_seen` is false and ground-truth leak is still true → **HUMAN_ASSIST** `no_leak_in_view`.  
   If `leak_seen` is false and ground-truth leak is false → **DONE** `false_alarm`.

3. **INSPECT → PLAN**  
   Target Cartesian pose: valve handle from `plant`, gripper z along +z (valve stem), approach from −x. `ik.solve(T_target, q_seed)` → joint vector. If IK residual > 2 cm, try one backup seed; then HUMAN_ASSIST `ik_failed`.

4. **PLAN → INTERVENE**  
   Interpolate to pre-grasp (handle − 0.08 m in x), close gripper, interpolate to grasp, command q5 (wrist roll) +90°, open gripper, retract to inspect pose.

5. **INTERVENE → VERIFY**  
   `get_leak() == False` and `get_valve_angle()` in [80, 100] → **DONE**.  
   Otherwise increment `attempts`. If `attempts < 2`, go **INTERVENE** again (recovery). If `attempts >= 2`, **HUMAN_ASSIST** `valve_not_closed`.

`--fail-valve` makes the mock ignore the first two close commands so the human-assist path is demonstrable.

Mock render each `step`: top-down plant, robot, planned path, FSM label, leak badge, plus a camera thumbnail.

Coppelia path: on connect, `scene_builder.ensure_scene(sim)` creates floor, walls, tank, pipe, valve, robot if `/RAVEN` does not exist; starts simulation; then the same FSM runs.

## 7. Failure and recovery

| Condition | Action |
|---|---|
| A* returns empty | HUMAN_ASSIST `no_path` |
| Nav timeout 60 s | HUMAN_ASSIST `nav_timeout` |
| Vision miss while leak true | HUMAN_ASSIST `no_leak_in_view` |
| IK residual > 2 cm after backup seed | HUMAN_ASSIST `ik_failed` |
| Valve still open after 1st intervene | retry INTERVENE once |
| Valve still open after 2nd intervene | HUMAN_ASSIST `valve_not_closed` |
| Coppelia not running / ZMQ fail | CLI exits with “Start CoppeliaSim Edu, then retry `--backend coppelia`” (no traceback dump) |
| Mock window closed | abort mission, exit 0 |

No silent retries beyond the single recovery intervene. Reasons are printed and stored on `MissionResult.reason`.

## 8. Testing

Run with `python -m pytest tests -q`. All default tests use the mock backend. Coppelia is never required in CI.

| Test | Proves |
|---|---|
| `test_astar.py` | path from home to approach exists; path cells are free; blocked world returns empty |
| `test_dh.py` | FK at zero joints matches a hand-computed T; round-trip homogeneous multiply |
| `test_ik.py` | reachable handle pose: IK then FK translation error < 1 cm |
| `test_vision.py` | synthetic red blob → leak_seen; no red → not seen; green handle centroid within 5 px |
| `test_fsm.py` | success sequence of mocked backend → DONE; two failed verifies → HUMAN_ASSIST |
| `test_mission_mock.py` | `run_mission(mock)` with injected leak → DONE and valve closed; `--fail-valve` equivalent → HUMAN_ASSIST |

Success criteria for the demo (manual):

1. `python -m raven` shows the robot reach the valve, turn it, leak badge off, status DONE.
2. `python -m raven` with fail-valve flag ends HUMAN_ASSIST after two tries.
3. With CoppeliaSim Edu open: `python -m raven --backend coppelia` builds the scene and repeats the success mission.

## 9. CoppeliaSim

Not required to develop, test, or demo the mock.

For the 3D demo: CoppeliaSim Edu (free), `pip install coppeliasim-zmqremoteapi-client`, default ZMQ port 23000, simulation started by the Python client after scene build.

Scene objects created by `scene_builder` use fixed names (`/RAVEN`, `/RAVEN/base`, `/RAVEN/arm/j1`…`j5`, `/RAVEN/gripper`, `/RAVEN/camera`, `/Plant/tank`, `/Plant/valve`). Coppelia backend looks up those names; it does not depend on a checked-in `.ttt` binary.

## 10. Dependencies

`numpy`, `opencv-python`, `matplotlib`, `pytest`.  
Optional extra: `coppeliasim-zmqremoteapi-client` (imported only by the coppelia backend).
