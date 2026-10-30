import numpy as np

from raven.arm import dh


def test_fk_zero_joints_translation():
    T = dh.fk(np.zeros(5))
    np.testing.assert_allclose(T[:3, 3], [0.40, 0.00, 0.00], atol=1e-9)


def test_homogeneous_roundtrip_multiply():
    T = dh.fk(np.zeros(5))
    I = T @ np.linalg.inv(T)
    np.testing.assert_allclose(I, np.eye(4), atol=1e-9)


def test_fk_prefix_stops_after_n_joints():
    q = np.array([0.2, -0.3, 0.7, 0.1, 0.0])
    T2 = dh.fk_prefix(q, 2)
    T_zero_tail = dh.fk(np.array([0.2, -0.3, 0.0, 0.0, 0.0]))
    assert not np.allclose(T2, T_zero_tail, atol=1e-6)
    np.testing.assert_allclose(dh.fk_prefix(q, 5), dh.fk(q), atol=1e-9)
