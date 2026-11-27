#!/usr/bin/env python3
"""Track the largest face with a pan/tilt webcam.

Detects faces with OpenCV's Haar cascade, picks the largest bounding box and
uses proportional control (with a dead zone) to move it to the frame centre.
When no face is seen for a while, the camera slowly scans back and forth.

Usage:  python face_tracker.py [--headless]
Keys in the preview window:  q / Esc = quit
(Full screen is handled by the desktop: see autostart/labwc-rc.xml; F11 toggles.)
"""

import argparse
import collections
import glob
import math
import os
import time

import cv2
import numpy as np

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


def _centre(b):
    return b[0] + b[2] / 2, b[1] + b[3] / 2


class FaceLock:
    """Decides which detection (if any) the tracker should follow.

    A face locks once it has been seen in CONFIRM_FRAMES frames at about the
    same position and eyes were found in it at least once - or, as a fallback,
    after CONFIRM_FRAMES_NO_EYES frames without eyes. Up to MISS_TOLERANCE
    dropped frames are forgiven. Detections with eyes are preferred over ones
    without, so face-like wall patterns don't push a real face aside.
    Once locked, the face is followed without eye checks; the lock moves to
    another face only if it is clearly larger and has eyes.

    Fast movement: while locked, the face is looked for around its predicted
    position, in an area that widens with each missed frame, and the lock is
    kept for up to COAST_FRAMES missed frames. Just after a lock is lost, a
    face near the predicted position is locked at once (re-lock).
    """

    def __init__(self, eye_check=None):
        self.eye_check = eye_check   # callable(box) -> bool, or None
        self._reset()

    def _reset(self):
        self.track = None
        self.hits = 0
        self.misses = 0
        self.eyes_seen = False
        self.eye_hits = 0        # frames of this face in which eyes were found
        self.locked = False
        self.relocked = False

    def _start(self, box, eyes):
        self.track, self.hits, self.misses = box, 1, 0
        self.eyes_seen, self.eye_hits, self.locked = eyes, int(eyes), False

    def _miss(self):
        self.misses += 1
        limit = config.COAST_FRAMES if self.trusted else config.MISS_TOLERANCE
        if self.misses > limit:
            self._reset()
        return None

    @property
    def coasting(self):
        """Locked, but the face is missing from the latest frame(s)."""
        return self.locked and self.misses > 0

    @property
    def trusted(self):
        """Locked onto a face that has been tracked long enough, with eyes found
        often enough, to trust. (Blurred false detections rarely pass the eye
        check more than once.)"""
        eyes_ok = self.eye_check is None or self.eye_hits >= config.TRUST_EYE_FRAMES
        return self.locked and self.hits >= config.TRUST_FRAMES and eyes_ok

    def update(self, boxes, predicted=None, relock=None):
        """boxes: all detections this frame.
        predicted: (x, y) image position where the locked face is expected, or None.
        relock: callable(box) -> distance or None; a box it accepts is locked at
        once (used just after a lock was lost).
        Returns (followed_box, candidate)."""
        self.relocked = False
        if not boxes:
            return self._miss(), None

        match = None
        if self.track is not None:
            centre = (predicted if self.locked and predicted is not None
                      else _centre(self.track))
            near = [b for b in boxes if self._near(b, centre, self.track[2])]
            if near:
                match = min(near, key=lambda b: abs(_centre(b)[0] - centre[0])
                            + abs(_centre(b)[1] - centre[1]))

        if self.locked and match is not None:
            # Switch only to a clearly larger face that has eyes.
            bigger = [b for b in boxes if b is not match and _area(b) > 1.3 * _area(match)]
            bigger = [b for b in sorted(bigger, key=_area, reverse=True) if self._eyes(b)]
            if bigger:
                self._start(bigger[0], True)
                return None, bigger[0]
            self.track, self.hits, self.misses = match, self.hits + 1, 0
            if self.eye_check is not None and self.eye_hits < config.TRUST_EYE_FRAMES:
                self.eye_hits += self.eye_check(match)
            return match, match

        if self.locked and match is None:
            return self._miss(), None

        if relock is not None:
            scored = [(d, b) for b in boxes if (d := relock(b)) is not None]
            if scored:
                box = min(scored)[1]
                self._start(box, True)
                self.hits, self.locked, self.relocked = config.CONFIRM_FRAMES, True, True
                return box, box

        # Not locked yet: prefer the largest detection with eyes.
        eyed = {id(b): self._eyes(b) for b in boxes}
        with_eyes = [b for b in boxes if eyed[id(b)]]
        if match is not None and (eyed[id(match)] or not with_eyes):
            self.track, self.hits, self.misses = match, self.hits + 1, 0
            self.eyes_seen |= eyed[id(match)]
            self.eye_hits += eyed[id(match)]
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

    def _near(self, box, centre, ref_w):
        """Is box within the matching area around centre? The area widens
        with each missed frame, since a fast face may have moved further."""
        bx, by = _centre(box)
        limit = config.MATCH_DIST * max(box[2], ref_w) * (1 + 0.5 * self.misses)
        return abs(bx - centre[0]) <= limit and abs(by - centre[1]) <= limit

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


class FaceMotion:
    """Where the tracked face is in the room and how fast it is moving.

    Positions are room angles in degrees: x = PAN_SIGN*pan + offset (positive =
    to the camera's right), y = TILT_SIGN*tilt + offset (positive = down).
    Working in room angles means the camera's own movement is not mistaken for
    the face moving.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.pos = None        # (x, y) degrees
        self.vel = (0.0, 0.0)  # degrees per second
        self.t = None
        self.width = None      # face width in pixels when last seen

    @staticmethod
    def to_room(box, cam, size):
        """Room angles of a face box, given the camera angles (pan, tilt) when
        the frame was captured and the frame size (w, h)."""
        (cx, cy), (w, h) = _centre(box), size
        return (config.PAN_SIGN * cam[0] + (cx - w / 2) / w * config.FOV_H_DEG,
                config.TILT_SIGN * cam[1] + (cy - h / 2) / h * config.FOV_V_DEG)

    @staticmethod
    def to_offset(pos, cam):
        """Normalised offset (-0.5..0.5 = frame edges) of a room position from
        the centre of a camera pointing at cam = (pan, tilt)."""
        return ((pos[0] - config.PAN_SIGN * cam[0]) / config.FOV_H_DEG,
                (pos[1] - config.TILT_SIGN * cam[1]) / config.FOV_V_DEG)

    def observe(self, box, cam, size, now):
        p = self.to_room(box, cam, size)
        if self.pos is not None and now > self.t:
            dt = now - self.t
            a = config.VELOCITY_SMOOTHING
            self.vel = tuple(a * (p[i] - self.pos[i]) / dt + (1 - a) * self.vel[i]
                             for i in (0, 1))
        self.pos, self.t, self.width = p, now, box[2]

    def predict(self, now):
        dt = min(now - self.t, config.PREDICT_MAX_S)
        return self.pos[0] + self.vel[0] * dt, self.pos[1] + self.vel[1] * dt


class OtherFaces:
    """Keeps track of the faces other than the tracked one, for decorations.

    A face counts once it has been seen for CONFIRM_FRAMES frames with eyes
    found at least once. Unless REINDEER_REQUIRE_EYES is False, there is no
    no-eyes fallback here: face-like wall pictures stay in view for a long time
    while someone is being tracked. Faces are kept through up to MISS_TOLERANCE
    missed frames so decorations don't flicker.
    """

    def __init__(self, eye_check=None):
        self.eye_check = eye_check
        self.tracks = []   # dicts: box, hits, misses, eyes

    def update(self, boxes, exclude):
        unmatched = list(self.tracks)
        for box in boxes:
            if any(box is e for e in exclude if e is not None):
                continue
            near = [t for t in unmatched if FaceLock._same(box, t["box"])]
            if near:
                t = min(near, key=lambda t: FaceLock._dist(box, t["box"]))
                unmatched.remove(t)
                t["box"], t["hits"], t["misses"] = box, t["hits"] + 1, 0
            else:
                t = {"box": box, "hits": 1, "misses": 0, "eyes": False}
                self.tracks.append(t)
            if not t["eyes"]:
                t["eyes"] = self.eye_check is None or self.eye_check(box)
        for t in unmatched:
            t["misses"] += 1
        self.tracks = [t for t in self.tracks if t["misses"] <= config.MISS_TOLERANCE]
        # Never decorate the tracked face (or its candidate) a second time.
        return [t["box"] for t in self.tracks if self._confirmed(t)
                and not any(e is not None and FaceLock._same(t["box"], e) for e in exclude)]

    @staticmethod
    def _confirmed(t):
        if t["hits"] >= config.CONFIRM_FRAMES and t["eyes"]:
            return True
        return (not config.REINDEER_REQUIRE_EYES and bool(config.CONFIRM_FRAMES_NO_EYES)
                and t["hits"] >= config.CONFIRM_FRAMES_NO_EYES)


def _bezier(p0, p1, p2, n=24):
    """Points along a quadratic Bezier curve from p0 to p2 with control p1."""
    pts = []
    for i in range(n + 1):
        t = i / n
        a, b, c = (1 - t) ** 2, 2 * (1 - t) * t, t * t
        pts.append((a * p0[0] + b * p1[0] + c * p2[0],
                    a * p0[1] + b * p1[1] + c * p2[1]))
    return pts


def draw_santa_hat(img, face):
    """Draw a Santa hat sitting on top of the face box (x, y, w, h)."""
    x, y, w, h = face
    cx = x + w / 2
    band_w, band_h = 1.25 * w, 0.26 * w
    band_cy = y + 0.12 * h - band_h / 2          # fur band sits on the forehead
    band_top = band_cy - band_h / 3
    left, right = cx - 0.52 * w, cx + 0.52 * w
    tip = (cx + 0.62 * w, band_top - 0.45 * w)   # the point flops over to the right

    # Red cone: up the left side, over the top to the tip, back down the right side.
    outline = (_bezier((left, band_top), (cx - 0.3 * w, band_top - 1.25 * w), tip)
               + _bezier(tip, (cx + 0.2 * w, band_top - 0.55 * w), (right, band_top)))
    cone = np.array(outline, np.int32)
    cv2.fillPoly(img, [cone], (40, 30, 200), cv2.LINE_AA)
    # Shading along the right-hand fold.
    fold = np.array(_bezier(tip, (cx + 0.2 * w, band_top - 0.55 * w), (right, band_top))
                    + _bezier((right - 0.2 * w, band_top), (cx + 0.05 * w, band_top - 0.6 * w),
                              tip), np.int32)
    cv2.fillPoly(img, [fold], (30, 20, 150), cv2.LINE_AA)
    cv2.polylines(img, [cone], True, (20, 10, 110), max(1, w // 60), cv2.LINE_AA)

    # White fur band and pompom.
    white, shadow = (245, 245, 245), (200, 200, 210)
    axes = (int(band_w / 2), int(band_h / 2))
    centre = (int(cx), int(band_cy))
    cv2.ellipse(img, centre, axes, 0, 0, 360, white, -1, cv2.LINE_AA)
    cv2.ellipse(img, centre, axes, 0, 0, 360, shadow, max(1, w // 50), cv2.LINE_AA)
    r = int(0.15 * w)
    cv2.circle(img, (int(tip[0]), int(tip[1])), r, white, -1, cv2.LINE_AA)
    cv2.circle(img, (int(tip[0]), int(tip[1])), r, shadow, max(1, w // 50), cv2.LINE_AA)


def draw_reindeer(img, face):
    """Draw antlers above the face box (x, y, w, h) and a red Rudolph nose on it."""
    x, y, w, h = face
    cx = x + w / 2
    brown, dark = (35, 75, 125), (20, 45, 80)
    beam_t, tine_t = max(2, w // 12), max(2, w // 16)

    def P(u, v):                       # face-relative units (fractions of w)
        return (cx + u * w, y + v * w)

    for side in (-1, 1):               # left antler, then mirrored right one
        base, ctrl, tip = P(side * 0.22, 0.06), P(side * 0.62, -0.2), P(side * 0.48, -0.82)
        beam = _bezier(base, ctrl, tip)
        lines = [beam]
        for t, end in ((0.35, P(side * 0.85, -0.42)),     # outer tine
                       (0.55, P(side * 0.82, -0.8)),      # upper outer tine
                       (0.7, P(side * 0.22, -0.7))):      # inner tine
            start = beam[int(t * (len(beam) - 1))]
            mid = ((start[0] + end[0]) / 2, (start[1] + end[1]) / 2 - 0.08 * w)
            lines.append(_bezier(start, mid, end, 10))
        for pts, thick in zip(lines, [beam_t] + [tine_t] * 3):
            arr = np.array(pts, np.int32)
            cv2.polylines(img, [arr], False, dark, thick + 2, cv2.LINE_AA)
            cv2.polylines(img, [arr], False, brown, thick, cv2.LINE_AA)

    # Red nose with a small highlight.
    nose = (int(cx), int(y + 0.6 * h))
    r = max(4, int(0.11 * w))
    cv2.circle(img, nose, r, (20, 20, 220), -1, cv2.LINE_AA)
    cv2.circle(img, nose, r, (10, 10, 140), max(1, w // 60), cv2.LINE_AA)
    cv2.circle(img, (nose[0] - r // 3, nose[1] - r // 3), max(1, r // 4),
               (230, 230, 255), -1, cv2.LINE_AA)


def draw_label(img, text):
    """Draw text centred at the bottom of the frame on a translucent background."""
    font, scale, thick = cv2.FONT_HERSHEY_SIMPLEX, config.LABEL_SCALE, 2
    (tw, th), base = cv2.getTextSize(text, font, scale, thick)
    pad_x, pad_y, margin = 12, 8, 10
    h, w = img.shape[:2]
    x0 = max(0, (w - tw) // 2 - pad_x)
    x1 = min(w, (w + tw) // 2 + pad_x)
    y1 = h - margin
    y0 = max(0, y1 - th - base - 2 * pad_y)
    roi = img[y0:y1, x0:x1]
    bg = np.full_like(roi, config.LABEL_BG_COLOUR)
    a = config.LABEL_BG_OPACITY
    cv2.addWeighted(bg, a, roi, 1 - a, 0, dst=roi)
    cv2.putText(img, text, ((w - tw) // 2, y1 - pad_y - base), font, scale,
                config.LABEL_COLOUR, thick, cv2.LINE_AA)


def detect_screen_size():
    """(width, height) of the first connected monitor, or None."""
    for status in sorted(glob.glob("/sys/class/drm/card*-*/status")):
        try:
            with open(status) as f:
                if f.read().strip() != "connected":
                    continue
            with open(os.path.join(os.path.dirname(status), "modes")) as f:
                w, h = f.readline().strip().split("x")[:2]
            return int(w), int(h)
        except (OSError, ValueError):
            continue
    return None


def preview_size(frame_w, frame_h):
    """Size to show the preview at: as large as fits the screen below the
    window's toolbar and status bar, keeping the aspect ratio."""
    if config.PREVIEW_SIZE:
        return tuple(config.PREVIEW_SIZE)
    screen = detect_screen_size()
    if screen is None:
        return None
    sw, sh = screen[0], screen[1] - config.PREVIEW_CHROME_PX
    scale = min(sw / frame_w, sh / frame_h)
    return int(frame_w * scale), int(frame_h * scale)


def camera_angles_at(angle_log, t):
    """(pan, tilt) the camera was commanded to at time t (from the servo log)."""
    for when, pan, tilt in reversed(angle_log):
        if when <= t:
            return pan, tilt
    return angle_log[0][1], angle_log[0][2]


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
    parser.add_argument("--debug-log", metavar="CSV",
                        help="write one line per frame (state, face/predicted position, "
                             "speed, servo angles) to this CSV file")
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
    motion = FaceMotion()
    lost_at = -1e9           # when the lock was last lost
    search_targets = []      # room pan angles to visit after losing the lock
    # Recent servo commands, to know where the camera pointed when a frame
    # was captured (frames arrive a little after they are taken).
    angle_log = collections.deque([(time.monotonic(), rig.pan, rig.tilt)], maxlen=30)
    debug = None
    if args.debug_log:
        debug = open(args.debug_log, "w", buffering=1)
        debug.write("t,state,faces,face_x,face_w,pred_x,pan,tilt,vel_x,vel_y\n")
    t0 = time.monotonic()
    others = OtherFaces(eye_check)
    raw_frames = confirmed_frames = 0  # per-second counts for the status line
    window = "Face tracker"
    if not args.headless:
        # The desktop makes this window full screen (autostart/labwc-rc.xml).
        # OpenCV's Qt window doesn't scale the picture itself, so frames are
        # scaled to fit the screen below the toolbar and shown at actual size
        # (AUTOSIZE). GUI_EXPANDED keeps the toolbar. OpenCV's own full-screen
        # mode is not used because it hides the toolbar.
        cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE | cv2.WINDOW_GUI_EXPANDED)
    display_size = None

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
            cam = camera_angles_at(angle_log, now - config.CAMERA_LATENCY_S)

            # Where the locked face should be in this frame, and (just after
            # losing it) which faces may be re-locked at once.
            predicted = relock = pred_x = None
            searching = bool(search_targets)
            if motion.pos is not None and (lock.locked or searching
                                           or now - lost_at < config.RELOCK_S):
                pred = motion.predict(now)
                ox, oy = FaceMotion.to_offset(pred, cam)
                pred_x = w / 2 + ox * w
                if lock.trusted:
                    predicted = (pred_x, h / 2 + oy * h)
                elif not lock.locked:
                    # While searching, any face of similar size is taken to be
                    # the lost one; otherwise it must be near the prediction.
                    radius = math.inf if searching else config.RELOCK_RADIUS_DEG

                    def relock(b, pred=pred, radius=radius):
                        p = FaceMotion.to_room(b, cam, (w, h))
                        d = math.hypot(p[0] - pred[0], p[1] - pred[1])
                        size_ok = 0.5 <= b[2] / motion.width <= 2.0
                        return d if d <= radius and size_ok else None

            was_locked, was_trusted = lock.locked, lock.trusted
            face, candidate = lock.update(faces, predicted, relock)
            other_faces = others.update(faces, (face, candidate)) if config.REINDEER else []

            if face is not None:
                if not was_locked and not lock.relocked:
                    motion.reset()          # a new face: forget the old one's speed
                motion.observe(face, cam, (w, h), now)
                last_seen = now
                search_targets = []
                x, y, fw, fh = face
                off_x = (x + fw / 2 - w / 2) / w   # + = face right of centre
                off_y = (y + fh / 2 - h / 2) / h   # + = face below centre
                d_pan = axis_correction(off_x, config.FOV_H_DEG, config.KP_PAN, config.PAN_SIGN)
                d_tilt = axis_correction(off_y, config.FOV_V_DEG, config.KP_TILT, config.TILT_SIGN)
                # Feed-forward: also turn with the face's measured speed, so a
                # moving face isn't always left behind.
                vx, vy = motion.vel if lock.trusted else (0.0, 0.0)
                if abs(vx) >= config.FEEDFORWARD_MIN_SPEED:
                    d_pan += config.PAN_SIGN * config.FEEDFORWARD * vx * dt
                if abs(vy) >= config.FEEDFORWARD_MIN_SPEED:
                    d_tilt += config.TILT_SIGN * config.FEEDFORWARD * vy * dt
                m = config.MAX_STEP_DEG
                rig.move_by(max(-m, min(m, d_pan)), max(-m, min(m, d_tilt)))
                state = "RELOCKED" if lock.relocked else "TRACKING"
            elif lock.coasting and lock.trusted:
                # Face briefly missing: keep turning towards where it should be.
                ox, oy = FaceMotion.to_offset(motion.predict(now), (rig.pan, rig.tilt))
                rig.move_by(
                    axis_correction(ox, config.FOV_H_DEG, config.KP_PAN, config.PAN_SIGN),
                    axis_correction(oy, config.FOV_V_DEG, config.KP_TILT, config.TILT_SIGN))
                state = "COASTING"
            else:
                # (lock.track is set when the lock just moved to a larger face:
                # that isn't a loss, so don't search.)
                if was_trusted and lock.track is None and motion.pos is not None:
                    # Lock just lost: search the way the face was going (or
                    # towards the edge it was near). If that's unknown, look
                    # to both sides of where it was last seen and come back.
                    lost_at = now
                    base, vx = motion.pos[0], motion.vel[0]
                    edge = FaceMotion.to_offset(motion.pos, (rig.pan, rig.tilt))[0]
                    span = config.SEARCH_DEG
                    if abs(vx) >= config.SEARCH_MIN_SPEED_DEG_S:
                        search_targets = [base + math.copysign(span, vx)]
                    elif abs(edge) > 0.3:
                        search_targets = [base + math.copysign(span, edge)]
                    else:
                        search_targets = [base + span / 2, base - span / 2, base]
                state = None
            if state is None and search_targets:
                goal = config.PAN_SIGN * search_targets[0]      # room angle -> servo angle
                goal = max(config.PAN_MIN, min(config.PAN_MAX, goal))
                diff, step = goal - rig.pan, config.SEARCH_SPEED_DEG_S * dt
                if diff:
                    scan_dir = math.copysign(1.0, diff)       # carry on this way when scanning
                if abs(diff) <= step:
                    rig.move_to(goal, rig.tilt)
                    search_targets.pop(0)
                else:
                    rig.move_to(rig.pan + math.copysign(step, diff), rig.tilt)
                state = "SEARCHING"
            elif state is None and now - last_seen < config.LOST_FACE_HOLD_S:
                state = "HOLD"
            elif state is None:
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
            angle_log.append((now, rig.pan, rig.tilt))

            if debug:
                debug.write(f"{now - t0:.3f},{state},{len(faces)},"
                            f"{'' if face is None else round(_centre(face)[0])},"
                            f"{'' if face is None else face[2]},"
                            f"{'' if pred_x is None else round(pred_x)},"
                            f"{rig.pan:.1f},{rig.tilt:.1f},"
                            f"{motion.vel[0]:.1f},{motion.vel[1]:.1f}\n")

            raw_frames += len(faces) > 0
            confirmed_frames += face is not None
            if now - last_print >= 1.0:
                last_print = now
                print(f"{state:8s} raw={raw_frames:2d} confirmed={confirmed_frames:2d} "
                      f"pan={rig.pan:5.1f} tilt={rig.tilt:5.1f} fps={fps:4.1f}", flush=True)
                raw_frames = confirmed_frames = 0

            if not args.headless:
                if config.SHOW_OVERLAY:
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
                for other in other_faces:
                    draw_reindeer(frame, other)
                if config.SANTA_HAT and face is not None:
                    draw_santa_hat(frame, face)
                if config.SHOW_OVERLAY:
                    cv2.putText(frame,
                                f"{state}  pan {rig.pan:5.1f}  tilt {rig.tilt:5.1f}  {fps:4.1f} fps",
                                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                if config.LABEL_TEXT:
                    draw_label(frame, config.LABEL_TEXT)
                if display_size is None:
                    display_size = preview_size(w, h) or (w, h)
                if display_size != (w, h):
                    frame = cv2.resize(frame, display_size, interpolation=cv2.INTER_LINEAR)
                cv2.imshow(window, frame)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        if debug:
            debug.close()
        cap.release()
        if not args.headless:
            cv2.destroyAllWindows()
        rig.home()
        time.sleep(0.5)
        rig.release()


if __name__ == "__main__":
    main()
