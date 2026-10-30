from __future__ import annotations

from raven import config


def _exists(sim, path: str) -> bool:
    try:
        h = sim.getObject(path)
        return h != -1
    except Exception:
        return False


def _alias(sim, handle: int, name: str) -> int:
    sim.setObjectAlias(handle, name)
    return handle


def _color(sim, handle: int, rgb: list[float]) -> None:
    sim.setShapeColor(handle, None, sim.colorcomponent_ambient_diffuse, rgb)


def _cuboid(sim, size, pos, rgb, parent=None):
    h = sim.createPrimitiveShape(sim.primitiveshape_cuboid, list(size), 0)
    sim.setObjectPose(h, [pos[0], pos[1], pos[2], 0, 0, 0, 1])
    _color(sim, h, rgb)
    if parent is not None:
        sim.setObjectParent(h, parent, True)
    return h


def _cylinder(sim, size, pos, rgb, parent=None):
    h = sim.createPrimitiveShape(sim.primitiveshape_cylinder, list(size), 0)
    sim.setObjectPose(h, [pos[0], pos[1], pos[2], 0, 0, 0, 1])
    _color(sim, h, rgb)
    if parent is not None:
        sim.setObjectParent(h, parent, True)
    return h


def ensure_scene(sim) -> None:
    if _exists(sim, "/RAVEN"):
        return

    floor = _cuboid(sim, [config.WORLD_W, config.WORLD_H, 0.04], [config.WORLD_W / 2, config.WORLD_H / 2, -0.02], [0.45, 0.45, 0.48])
    _alias(sim, floor, "Floor")

    plant = sim.createDummy(0.01)
    sim.setObjectPose(plant, [0, 0, 0, 0, 0, 0, 1])
    _alias(sim, plant, "Plant")

    ox, oy = config.OBSTACLE_CENTER
    sx, sy = config.OBSTACLE_SIZE
    _alias(sim, _cuboid(sim, [sx, sy, 0.8], [ox, oy, 0.4], [0.35, 0.35, 0.38], plant), "obstacle")

    tx, ty = config.TANK_XY
    _alias(sim, _cylinder(sim, [0.5, 0.5, 0.9], [tx, ty, 0.45], [0.25, 0.45, 0.7], plant), "tank")
    leak = _alias(sim, sim.createPrimitiveShape(sim.primitiveshape_sphere, [0.18, 0.18, 0.18], 0), "leak")
    sim.setObjectPose(leak, [tx, ty - 0.28, 0.25, 0, 0, 0, 1])
    _color(sim, leak, [0.9, 0.08, 0.08])
    sim.setObjectParent(leak, plant, True)

    vx, vy = config.VALVE_XY
    joint = sim.createJoint(sim.joint_revolute_subtype)
    sim.setObjectPose(joint, [vx, vy, config.VALVE_Z, 0, 0, 0, 1])
    sim.setObjectParent(joint, plant, True)
    _alias(sim, joint, "valve")
    handle = _cylinder(sim, [0.04, 0.04, 0.22], [vx, vy, config.VALVE_Z + 0.12], [0.1, 0.8, 0.15], joint)
    _alias(sim, handle, "handle")

    raven = sim.createDummy(0.01)
    sim.setObjectPose(raven, [config.HOME.x, config.HOME.y, 0.0, 0, 0, 0, 1])
    _alias(sim, raven, "RAVEN")

    base = _cuboid(sim, [0.40, 0.30, 0.14], [config.HOME.x, config.HOME.y, 0.07], [0.95, 0.55, 0.15], raven)
    _alias(sim, base, "base")

    arm = sim.createDummy(0.01)
    sim.setObjectParent(arm, raven, True)
    _alias(sim, arm, "arm")
    for i in range(1, 6):
        j = sim.createJoint(sim.joint_revolute_subtype)
        sim.setObjectParent(j, arm, True)
        _alias(sim, j, f"j{i}")
        link = _cuboid(sim, [0.06, 0.06, 0.16], [config.HOME.x, config.HOME.y, 0.20], [0.7, 0.7, 0.75], arm)
        _alias(sim, link, f"l{i}")

    grip = _cuboid(sim, [0.08, 0.05, 0.04], [config.HOME.x, config.HOME.y, 0.28], [0.2, 0.2, 0.2], raven)
    _alias(sim, grip, "gripper")

    try:
        cam = sim.createVisionSensor(
            0, [128, 128, 0, 0], [0.01, 2.0, 1.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )
        sim.setObjectParent(cam, raven, True)
        _alias(sim, cam, "camera")
    except Exception:
        cam = sim.createDummy(0.02)
        sim.setObjectParent(cam, raven, True)
        _alias(sim, cam, "camera")
