import numpy as np

from raven.arm import dh


def test_fk_zero_joints_translation():
    T = dh.fk(np.zeros(5))
    np.testing.assert_allclose(T[:3, 3], [0.40, 0.00, 0.00], atol=1e-9)


def test_homogeneous_roundtrip_multiply():
    T = dh.fk(np.zeros(5))
    I = T @ np.linalg.inv(T)
    np.testing.assert_allclose(I, np.eye(4), atol=1e-9)
