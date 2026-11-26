#!/usr/bin/env python3
"""Sweep the camera over the scene and record every Haar detection with its
confidence, to help tune the false-positive settings in config.py.

Run with nobody in view:  python survey.py [outdir]
Writes annotated frames and prints one line per detection.
"""

import os
import sys
import time

import cv2

import config
from face_tracker import find_cascade, open_camera
from servos import PanTilt

PAN_STEPS = range(config.PAN_MIN, config.PAN_MAX + 1, 15)
TILT_STEPS = (config.TILT_HOME - 20, config.TILT_HOME, config.TILT_HOME + 20)


def main():
    outdir = sys.argv[1] if len(sys.argv) > 1 else "survey"
    os.makedirs(outdir, exist_ok=True)
    detector = cv2.CascadeClassifier(find_cascade())
    cap = open_camera()
    rig = PanTilt()
    s = config.DETECT_SCALE
    try:
        for tilt in TILT_STEPS:
            for pan in PAN_STEPS:
                rig.move_to(pan, tilt)
                time.sleep(0.6)
                for _ in range(6):
                    cap.read()
                ok, frame = cap.read()
                small = cv2.resize(frame, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
                gray = cv2.equalizeHist(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY))
                # minNeighbors=3 so we also see the weak detections; the
                # neighbour count and level weight tell us how strong each one is.
                boxes, neigh, weights = detector.detectMultiScale3(
                    gray, scaleFactor=config.SCALE_FACTOR, minNeighbors=3,
                    minSize=(config.MIN_FACE_SIZE, config.MIN_FACE_SIZE),
                    outputRejectLevels=True)
                if len(boxes) == 0:
                    continue
                for (x, y, w, h), n, wt in zip(boxes, neigh, weights):
                    X, Y, W, H = (int(v / s) for v in (x, y, w, h))
                    print(f"pan={pan:3d} tilt={tilt:3d} box=({X},{Y},{W}x{H}) "
                          f"weight={float(wt):5.2f}")
                    cv2.rectangle(frame, (X, Y), (X + W, Y + H), (0, 0, 255), 2)
                    cv2.putText(frame, f"{float(wt):.2f}", (X, Y - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.imwrite(os.path.join(outdir, f"p{pan:03d}_t{tilt:03d}.jpg"), frame)
    finally:
        cap.release()
        rig.home()
        time.sleep(0.5)
        rig.release()


if __name__ == "__main__":
    main()
