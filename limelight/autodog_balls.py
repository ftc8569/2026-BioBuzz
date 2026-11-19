# Auto Dog ball detector: a Limelight 3A Python SnapScript.
#
# Upload in the Limelight UI (http://192.168.43.1:5801): create a "Python"
# pipeline in the slot set by AutoDogVisionTuning.pipeline (default 1), paste
# this file, and save. The robot reads the llpython array through
# LLResult.getPythonOutput() (see AutoDogOnboardTeleOp.kt).
#
# llpython layout (at most 32 doubles; must match LimelightBallFrame.parse):
#   [0] format version (1)
#   [1] image width px   [2] image height px   [3] ball count n (<= 7)
#   then n x (color code, center x px, center y px, radius px)
# Color codes: 1 = POLLEN, 2 = NECTAR. Balls are ordered lowest in the image
# (usually closest) first.
#
# TUNE THE HSV RANGES BELOW under the real lights. The defaults are
# placeholders, not measured BIOBUZZ colors. OpenCV hue runs 0-179.

import cv2
import numpy as np

FORMAT_VERSION = 1
POLLEN = 1
NECTAR = 2

# (code, [(low HSV, high HSV), ...]). Several ranges per color are OR'ed, which
# handles red hues that wrap around 0/179.
COLOR_RANGES = [
    (POLLEN, [((18, 110, 90), (35, 255, 255))]),    # placeholder: yellow
    (NECTAR, [((95, 120, 60), (130, 255, 255))]),   # placeholder: blue
]

MIN_RADIUS_PX = 6.0   # smaller blobs are noise or balls too far to chase
MIN_FILL = 0.55       # contour area / enclosing-circle area; rejects stripes and edges
MAX_BALLS = 7         # 4 + 4 * 7 = 32, the llpython limit
HEADER = 4

_KERNEL = np.ones((3, 3), np.uint8)


def color_mask(hsv, ranges):
    mask = None
    for low, high in ranges:
        m = cv2.inRange(hsv, np.array(low, np.uint8), np.array(high, np.uint8))
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _KERNEL)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _KERNEL)


def detect(image):
    """Returns [(code, cx, cy, r, contour)], lowest in the image first."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    balls = []
    for code, ranges in COLOR_RANGES:
        contours, _ = cv2.findContours(color_mask(hsv, ranges), cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            (x, y), r = cv2.minEnclosingCircle(c)
            if r < MIN_RADIUS_PX:
                continue
            if cv2.contourArea(c) / (np.pi * r * r) < MIN_FILL:
                continue
            balls.append((code, float(x), float(y), float(r), c))
    balls.sort(key=lambda b: -b[2])
    return balls[:MAX_BALLS]


def encode(balls, width, height):
    out = [float(FORMAT_VERSION), float(width), float(height), float(len(balls))]
    for code, x, y, r, _ in balls:
        out += [float(code), x, y, r]
    return out


def runPipeline(image, llrobot):
    balls = detect(image)
    height, width = image.shape[:2]
    for code, x, y, r, _ in balls:
        cv2.circle(image, (int(x), int(y)), int(r),
                   (0, 255, 255) if code == POLLEN else (255, 0, 0), 2)
    largest = max((b[4] for b in balls), key=cv2.contourArea, default=np.array([[]]))
    return largest, image, encode(balls, width, height)
