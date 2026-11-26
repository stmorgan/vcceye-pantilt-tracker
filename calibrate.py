#!/usr/bin/env python3
"""Calibration helpers for the pan/tilt mount.

  python calibrate.py move PAN TILT   move servos to absolute angles (deg)
  python calibrate.py release         stop driving the servos (go limp)
  python calibrate.py directions      measure PAN_SIGN / TILT_SIGN with the camera

'directions' moves each servo a few degrees and uses phase correlation between
camera frames to see which way the image moved. Point the camera at a scene
with some texture (not a blank wall) and keep it still while it runs.
"""

import sys
import time

import cv2
import numpy as np

import config
from servos import PanTilt

STEP_DEG = 8.0


def grab(cap, flush=6):
    for _ in range(flush):          # drop buffered frames taken mid-move
        cap.read()
    ok, frame = cap.read()
    if not ok:
        raise RuntimeError("camera read failed")
    return np.float32(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))


def measure(cap, rig, axis):
    rig.home()
    time.sleep(1.0)
    a = grab(cap)
    if axis == "pan":
        rig.move_by(d_pan=STEP_DEG)
    else:
        rig.move_by(d_tilt=STEP_DEG)
    time.sleep(1.0)
    b = grab(cap)
    rig.home()
    win = cv2.createHanningWindow(a.shape[::-1], cv2.CV_32F)
    (dx, dy), response = cv2.phaseCorrelate(a, b, win)
    return dx, dy, response


def directions():
    cap = cv2.VideoCapture(config.CAMERA_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.FRAME_HEIGHT)
    rig = PanTilt()
    try:
        w, h = config.FRAME_WIDTH, config.FRAME_HEIGHT
        for axis in ("pan", "tilt"):
            dx, dy, resp = measure(cap, rig, axis)
            main_shift = dx if axis == "pan" else dy
            other = dy if axis == "pan" else dx
            print(f"\n{axis.upper()}: +{STEP_DEG:.0f} deg -> image shift dx={dx:+.1f}px "
                  f"dy={dy:+.1f}px (confidence {resp:.2f})")
            if resp < 0.05 or abs(main_shift) < 2:
                print("  Shift too small / unreliable. Aim at a more textured scene and retry.")
                continue
            if abs(other) > abs(main_shift):
                print("  WARNING: shift is mostly on the other axis - are channels swapped?")
            sign = +1 if main_shift < 0 else -1
            name = "PAN_SIGN" if axis == "pan" else "TILT_SIGN"
            size, fov_name = (w, "FOV_H_DEG") if axis == "pan" else (h, "FOV_V_DEG")
            fov = size * STEP_DEG / abs(main_shift)
            if axis == "pan":
                where = "RIGHT" if sign > 0 else "LEFT"
                print(f"  Increasing pan angle turns the camera {where}.")
            else:
                where = "DOWN" if sign > 0 else "UP"
                print(f"  Increasing tilt angle tilts the camera {where}.")
            print(f"  -> set {name} = {sign:+d}   (estimated {fov_name} ~ {fov:.0f})")
    finally:
        cap.release()
        rig.home()
        time.sleep(0.5)
        rig.release()


def main(argv):
    if len(argv) >= 1 and argv[0] == "move" and len(argv) == 3:
        rig = PanTilt()
        rig.move_to(float(argv[1]), float(argv[2]))
        print(f"pan={rig.pan:.1f} tilt={rig.tilt:.1f} (clamped to config limits)")
    elif argv == ["release"]:
        PanTilt().release()
    elif argv == ["directions"]:
        directions()
    else:
        print(__doc__)


if __name__ == "__main__":
    main(sys.argv[1:])
