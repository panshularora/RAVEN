import numpy as np

from raven.vision import inspect


def test_red_blob_leak_seen():
    img = np.zeros((64, 64, 3), dtype=np.uint8)
    img[20:30, 40:50] = (220, 20, 20)
    r = inspect.inspect(img)
    assert r.leak_seen is True
    assert r.leak_centroid is not None
    assert abs(r.leak_centroid[0] - 44.5) < 5
    assert abs(r.leak_centroid[1] - 24.5) < 5


def test_no_red_leak_not_seen():
    img = np.zeros((64, 64, 3), dtype=np.uint8)
    assert inspect.inspect(img).leak_seen is False


def test_green_handle_centroid():
    img = np.zeros((64, 64, 3), dtype=np.uint8)
    img[10:16, 8:14] = (20, 200, 20)
    r = inspect.inspect(img)
    assert r.handle_seen is True
    assert abs(r.handle_centroid[0] - 10.5) < 5
    assert abs(r.handle_centroid[1] - 12.5) < 5
