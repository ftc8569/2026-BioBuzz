# Off-robot tests for autodog_balls.py on synthetic frames.
#   pip install opencv-python-headless numpy pytest
#   python -m pytest limelight/
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import autodog_balls as ab  # noqa: E402

YELLOW = (0, 220, 240)   # BGR, inside the POLLEN placeholder range
BLUE = (230, 80, 20)     # BGR, inside the NECTAR placeholder range


def frame():
    return np.full((480, 640, 3), (60, 90, 70), np.uint8)  # dull green-gray floor


def test_finds_one_ball_at_its_center():
    img = frame()
    cv2.circle(img, (400, 300), 30, YELLOW, -1)
    out = ab.runPipeline(img, [0.0] * 8)[2]
    assert out[:4] == [1.0, 640.0, 480.0, 1.0]
    code, x, y, r = out[4:8]
    assert code == ab.POLLEN
    assert abs(x - 400) < 1.5 and abs(y - 300) < 1.5 and abs(r - 30) < 2


def test_two_colors_closest_first():
    img = frame()
    cv2.circle(img, (200, 150), 15, YELLOW, -1)
    cv2.circle(img, (450, 400), 40, BLUE, -1)
    out = ab.runPipeline(img, [0.0] * 8)[2]
    assert out[3] == 2.0
    assert out[4] == ab.NECTAR and out[8] == ab.POLLEN


def test_rejects_noise_and_stripes():
    img = frame()
    cv2.circle(img, (100, 100), 3, YELLOW, -1)                 # too small
    cv2.rectangle(img, (0, 400), (639, 410), YELLOW, -1)       # tape line, low fill
    out = ab.runPipeline(img, [0.0] * 8)[2]
    assert out[3] == 0.0 and len(out) == 4


def test_output_fits_llpython_limit():
    img = frame()
    for i in range(12):
        cv2.circle(img, (40 + 50 * i, 240), 18, YELLOW, -1)
    out = ab.runPipeline(img, [0.0] * 8)[2]
    assert out[3] == ab.MAX_BALLS and len(out) <= 32


def test_empty_frame_returns_valid_contour():
    largest, _, out = ab.runPipeline(frame(), [0.0] * 8)
    assert out == [1.0, 640.0, 480.0, 0.0]
    assert isinstance(largest, np.ndarray)
