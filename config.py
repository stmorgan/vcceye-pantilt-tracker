"""Settings for the pan/tilt face tracker. Edit these to tune behaviour."""

# --- Servo hardware (Adafruit 16-channel PWM/Servo HAT, PCA9685) ---
PAN_CHANNEL = 0
TILT_CHANNEL = 1

# HS-65MG pulse range. Hitec servos accept roughly 550-2450 us end to end;
# 600-2400 us maps to ServoKit's 0-180 "degrees" while staying off the hard stops.
PULSE_MIN_US = 600
PULSE_MAX_US = 2400

# Safe angle limits (ServoKit degrees). The bracket allows about 150 degrees, so
# 15-165 is the full range. Narrow these if calibrate.py shows the bracket binding.
PAN_MIN, PAN_MAX = 15, 165
TILT_MIN, TILT_MAX = 15, 165

# Home position: where the camera points on startup and the tilt used while scanning.
PAN_HOME = 90
TILT_HOME = 90

# Direction signs. These depend on how the servos are mounted; calibrate.py
# measures them. +1: increasing the angle moves the image content left/up.
# Measured: increasing pan turns the camera right, increasing tilt tilts it up.
PAN_SIGN = +1
TILT_SIGN = -1

# --- Camera ---
CAMERA_INDEX = 0          # /dev/video0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
# Horizontal/vertical field of view in degrees (measured by calibrate.py).
FOV_H_DEG = 51.0
FOV_V_DEG = 35.0

# --- Face detection (Haar cascade) ---
DETECT_SCALE = 0.5        # detect on a half-size image for speed
SCALE_FACTOR = 1.15
MIN_NEIGHBORS = 6
MIN_FACE_SIZE = 30        # pixels, measured on the downscaled image
# False-positive filter: a face must be detected in this many consecutive frames,
# at roughly the same place, before the tracker follows it (30 fps -> 4 = ~0.13 s).
CONFIRM_FRAMES = 4
# Detection may drop out for this many frames without losing count of a face.
MISS_TOLERANCE = 5
# Eye check: detections with eyes inside are preferred over ones without, and
# lock after CONFIRM_FRAMES. This rejects face-shaped patterns such as picture
# frames on a wall. Once locked, eyes are no longer required.
REQUIRE_EYES = True
EYE_CASCADES = ("haarcascade_eye_tree_eyeglasses.xml", "haarcascade_eye.xml")
EYE_MIN_NEIGHBORS = 2
# Fallback: a face seen steadily for this many frames locks even if no eyes
# were found (e.g. far away or turned slightly). 0 disables the fallback.
# Lower = faster lock but more risk of false locks on face-like patterns.
CONFIRM_FRAMES_NO_EYES = 30
# How far (as a fraction of the face width) the face centre may move between
# frames and still count as the same face.
MATCH_DIST = 0.5

# --- Preview ---
# Draw a Santa hat on the tracked face in the preview window.
SANTA_HAT = True

# --- Proportional control ---
# Fraction of the measured angular error corrected per frame (0..1).
# Lower = smoother/slower, higher = faster but may overshoot.
KP_PAN = 0.25
KP_TILT = 0.25
# Dead zone: no correction while the face centre is within this fraction of the
# frame size from the centre (0.06 = 6% of width/height).
DEAD_ZONE = 0.06
# Maximum servo movement per frame, in degrees.
MAX_STEP_DEG = 4.0

# --- Scanning when no face is visible ---
LOST_FACE_HOLD_S = 1.5    # hold position this long before starting to scan
SCAN_SPEED_DEG_S = 20.0   # pan sweep speed
# Tilt levels for scanning, in degrees above TILT_HOME (negative = below).
# After each pan sweep the tilt steps to the next level and back down again,
# e.g. (0, 25, 50) gives 0, 25, 50, 25, 0, ... The camera sees about +/-17 deg
# vertically, so levels about 25 deg apart overlap slightly.
SCAN_TILT_UP_DEG = (0, 25, 50)
SCAN_TILT_SPEED_DEG_S = 15.0  # how fast tilt moves between scan levels
