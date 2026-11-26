#!/usr/bin/env python3
"""Track the largest face with a pan/tilt webcam.

Detects faces with OpenCV's Haar cascade, picks the largest bounding box and
uses proportional control (with a dead zone) to move it to the frame centre.
When no face is seen for a while, the camera slowly scans back and forth.

Usage:  python face_tracker.py [--headless]
Keys in the preview window:  q / Esc = quit
"""

import argparse
import os
import time

import cv2

import config
from servos import PanTilt

CASCADE_NAME = "haarcascade_frontalface_default.xml"


def find_cascade(name=CASCADE_NAME):
    candidates = []
    if hasattr(cv2, "data"):
        candidates.append(os.path.join(cv2.data.haarcascades, name))
    candidates += [
        f"/usr/share/opencv4/haarcascades/{name}",
        f"/usr/share/opencv/haarcascades/{name}",
        os.path.join(os.path.dirname(os.path.abspath(__file__)), name),
    ]
    for path in candidates:
        if os.path.isfile(path):
            return path
    raise FileNotFoundError(
        f"{name} not found. Install it with: sudo apt install opencv-data")


def has_eyes(eye_detectors, frame, face):
    """True if at least one eye is found in the upper part of the face box."""
    x, y, w, h = face
    roi = frame[y:y + int(h * 0.6), x:x + w]
    if roi.size == 0:
        return False
    gray = cv2.equalizeHist(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY))
    m = max(6, w // 10)
    for det in eye_detectors:
        if len(det.detectMultiScale(gray, scaleFactor=1.1,
                                    minNeighbors=config.EYE_MIN_NEIGHBORS,
                                    minSize=(m, m))) > 0:
            return True
    return False


def open_camera():
    cap = cv2.VideoCapture(config.CAMERA_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # keep latency low
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera {config.CAMERA_INDEX}")
    return cap


def detect_faces(detector, frame):
    """Return all face boxes (x, y, w, h) in full-frame pixels."""
    s = config.DETECT_SCALE
    small = cv2.resize(frame, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
    gray = cv2.equalizeHist(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY))
    faces = detector.detectMultiScale(
        gray, scaleFactor=config.SCALE_FACTOR, minNeighbors=config.MIN_NEIGHBORS,
        minSize=(config.MIN_FACE_SIZE, config.MIN_FACE_SIZE))
    return [tuple(int(v / s) for v in f) for f in faces]


def _area(b):
    return b[2] * b[3]


class FaceLock:
    """Decides which detection (if any) the tracker should follow.

    A face locks once it has been seen in CONFIRM_FRAMES frames at about the
    same position and eyes were found in it at least once - or, as a fallback,
    after CONFIRM_FRAMES_NO_EYES frames without eyes. Up to MISS_TOLERANCE
    dropped frames are forgiven. Detections with eyes are preferred over ones
    without, so face-like wall patterns don't push a real face aside.
    Once locked, the face is followed without eye checks; the lock moves to
    another face only if it is clearly larger and has eyes.
    """

    def __init__(self, eye_check=None):
        self.eye_check = eye_check   # callable(box) -> bool, or None
        self._reset()

    def _reset(self):
        self.track = None
        self.hits = 0
        self.misses = 0
        self.eyes_seen = False
        self.locked = False

    def _start(self, box, eyes):
        self.track, self.hits, self.misses = box, 1, 0
        self.eyes_seen, self.locked = eyes, False

    def _miss(self):
        self.misses += 1
        if self.misses > config.MISS_TOLERANCE:
            self._reset()
        return None

    def update(self, boxes):
        """boxes: all detections this frame. Returns (followed_box, candidate)."""
        if not boxes:
            return self._miss(), None

        match = None
        if self.track is not None:
            near = [b for b in boxes if self._same(b, self.track)]
            if near:
                match = min(near, key=lambda b: self._dist(b, self.track))

        if self.locked and match is not None:
            # Switch only to a clearly larger face that has eyes.
            bigger = [b for b in boxes if b is not match and _area(b) > 1.3 * _area(match)]
            bigger = [b for b in sorted(bigger, key=_area, reverse=True) if self._eyes(b)]
            if bigger:
                self._start(bigger[0], True)
                return None, bigger[0]
            self.track, self.hits, self.misses = match, self.hits + 1, 0
            return match, match

        if self.locked and match is None:
            return self._miss(), None

        # Not locked yet: prefer the largest detection with eyes.
        eyed = {id(b): self._eyes(b) for b in boxes}
        with_eyes = [b for b in boxes if eyed[id(b)]]
        if match is not None and (eyed[id(match)] or not with_eyes):
            self.track, self.hits, self.misses = match, self.hits + 1, 0
            self.eyes_seen |= eyed[id(match)]
        elif with_eyes:
            self._start(max(with_eyes, key=_area), True)
        elif self.track is not None and self.misses < config.MISS_TOLERANCE:
            self._miss()
            return None, max(boxes, key=_area)
        else:
            self._start(max(boxes, key=_area), False)

        need_eyes = self.eye_check is not None
        if self.hits >= config.CONFIRM_FRAMES and (self.eyes_seen or not need_eyes):
            self.locked = True
        elif config.CONFIRM_FRAMES_NO_EYES and self.hits >= config.CONFIRM_FRAMES_NO_EYES:
            self.locked = True
        return (self.track if self.locked else None), self.track

    def _eyes(self, box):
        return self.eye_check is None or self.eye_check(box)

    @staticmethod
    def _dist(a, b):
        return (abs(a[0] + a[2] / 2 - b[0] - b[2] / 2)
                + abs(a[1] + a[3] / 2 - b[1] - b[3] / 2))

    @staticmethod
    def _same(a, b):
        ax, ay = a[0] + a[2] / 2, a[1] + a[3] / 2
        bx, by = b[0] + b[2] / 2, b[1] + b[3] / 2
        limit = config.MATCH_DIST * max(a[2], b[2])
        return abs(ax - bx) <= limit and abs(ay - by) <= limit


def axis_correction(offset_frac, fov_deg, kp, sign):
    """Servo delta (deg) for a normalised offset (-0.5..0.5) from frame centre."""
    if abs(offset_frac) < config.DEAD_ZONE:
        return 0.0
    step = kp * offset_frac * fov_deg
    step = max(-config.MAX_STEP_DEG, min(config.MAX_STEP_DEG, step))
    return sign * step


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--headless", action="store_true", help="no preview window")
    args = parser.parse_args()

    if not args.headless and not os.environ.get("DISPLAY") \
            and not os.environ.get("WAYLAND_DISPLAY"):
        os.environ["DISPLAY"] = ":0"  # show on the Pi's monitor when run over SSH

    detector = cv2.CascadeClassifier(find_cascade())
    cap = open_camera()
    rig = PanTilt()
    time.sleep(0.5)

    scan_dir = 1.0
    # Tilt levels visited during scanning, up then back down: e.g. 0, 25, 50, 25.
    levels = list(config.SCAN_TILT_UP_DEG) or [0]
    scan_seq = levels + levels[-2:0:-1]
    scan_i = 0
    up_dir = -config.TILT_SIGN   # TILT_SIGN -1 means increasing angle tilts up
    last_seen = time.monotonic()
    last_t = time.monotonic()
    fps = 0.0
    last_print = 0.0
    eye_check = None
    if config.REQUIRE_EYES:
        eye_detectors = [cv2.CascadeClassifier(find_cascade(n)) for n in config.EYE_CASCADES]
        eye_check = lambda f: has_eyes(eye_detectors, frame, f)  # uses current frame
    lock = FaceLock(eye_check)
    raw_frames = confirmed_frames = 0  # per-second counts for the status line
    window = "Face tracker"

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Camera read failed; retrying")
                time.sleep(0.1)
                continue
            now = time.monotonic()
            dt = max(1e-3, now - last_t)
            last_t = now
            fps = 0.9 * fps + 0.1 * (1.0 / dt)

            h, w = frame.shape[:2]
            faces = detect_faces(detector, frame)
            face, candidate = lock.update(faces)

            if face is not None:
                last_seen = now
                x, y, fw, fh = face
                off_x = (x + fw / 2 - w / 2) / w   # + = face right of centre
                off_y = (y + fh / 2 - h / 2) / h   # + = face below centre
                rig.move_by(
                    axis_correction(off_x, config.FOV_H_DEG, config.KP_PAN, config.PAN_SIGN),
                    axis_correction(off_y, config.FOV_V_DEG, config.KP_TILT, config.TILT_SIGN))
                state = "TRACKING"
            elif now - last_seen < config.LOST_FACE_HOLD_S:
                state = "HOLD"
            else:
                # Scan: sweep pan between limits; at each end, step tilt to the
                # next level so the scan also covers faces above (or below) home.
                new_pan = rig.pan + scan_dir * config.SCAN_SPEED_DEG_S * dt
                if new_pan >= config.PAN_MAX or new_pan <= config.PAN_MIN:
                    scan_dir = -scan_dir
                    scan_i = (scan_i + 1) % len(scan_seq)
                target_tilt = config.TILT_HOME + up_dir * scan_seq[scan_i]
                d_tilt = target_tilt - rig.tilt
                max_t = config.SCAN_TILT_SPEED_DEG_S * dt
                rig.move_to(new_pan, rig.tilt + max(-max_t, min(max_t, d_tilt)))
                state = "SCANNING"

            raw_frames += len(faces) > 0
            confirmed_frames += face is not None
            if now - last_print >= 1.0:
                last_print = now
                print(f"{state:8s} raw={raw_frames:2d} confirmed={confirmed_frames:2d} "
                      f"pan={rig.pan:5.1f} tilt={rig.tilt:5.1f} fps={fps:4.1f}", flush=True)
                raw_frames = confirmed_frames = 0

            if not args.headless:
                cx, cy = w // 2, h // 2
                dzx, dzy = int(config.DEAD_ZONE * w), int(config.DEAD_ZONE * h)
                cv2.rectangle(frame, (cx - dzx, cy - dzy), (cx + dzx, cy + dzy),
                              (255, 255, 0), 1)
                # green = locked face being tracked, orange = candidate not yet
                # locked, red = other detections
                for (x, y, fw, fh) in faces:
                    box = (x, y, fw, fh)
                    colour = ((0, 255, 0) if box == face else
                              (0, 165, 255) if box == candidate else (0, 0, 255))
                    cv2.rectangle(frame, (x, y), (x + fw, y + fh), colour, 2)
                cv2.putText(frame,
                            f"{state}  pan {rig.pan:5.1f}  tilt {rig.tilt:5.1f}  {fps:4.1f} fps",
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.imshow(window, frame)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if not args.headless:
            cv2.destroyAllWindows()
        rig.home()
        time.sleep(0.5)
        rig.release()


if __name__ == "__main__":
    main()
