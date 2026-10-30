import numpy as np

from raven import config
from raven.arm import dh, ik


def test_ik_then_fk_translation_under_1cm():
    q_seed = config.HOME_Q.copy()
    T_target = dh.gripper_fk(np.array([0.2, -0.3, 0.7, 0.1, 0.0]))
    q, residual = ik.solve(T_target, q_seed)
    T_got = dh.gripper_fk(q)
    assert np.linalg.norm(T_got[:3, 3] - T_target[:3, 3]) < 0.01
    assert residual < 0.01
