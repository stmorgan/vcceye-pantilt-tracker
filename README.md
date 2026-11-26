# vcceye: pan/tilt face tracker

A Raspberry Pi 5 points a USB webcam at the largest face it can see. OpenCV's
Haar cascade finds the faces. Two servos on a pan/tilt mount then move the
camera until the largest face is in the centre of the picture. When nobody is
in view, the camera slowly scans back and forth.

## Hardware

| Part | Details |
|---|---|
| Computer | Raspberry Pi 5, Raspberry Pi OS Trixie (desktop) |
| Servo driver | Adafruit 16-channel PWM/Servo HAT (PCA9685, I2C address `0x40`) |
| Servos | 2 × Hitec HS-65MG: **pan on channel 0**, **tilt on channel 1** |
| Camera | USB webcam (`/dev/video0`) on the pan/tilt bracket |
| Servo power | 5–6 V supply (2 A or more) on the HAT's V+/GND screw terminal |

> The HAT's chip runs on power from the Pi, but **the servos do not.** If
> nothing is connected to the HAT's screw terminal, the servos get a signal
> but stay still.

On each servo plug, the brown/black wire goes to the pin marked GND, on the
edge of the board.

## Setup

These steps are done on the Pi. The project lives in `~/vcceye`.

### 1. Enable I2C and check the HAT

```sh
sudo raspi-config nonint do_i2c 0
sudo apt install -y i2c-tools
/usr/sbin/i2cdetect -y 1        # should show 40 (and 70)
```

### 2. Install OpenCV and the Haar cascades

```sh
sudo apt update
sudo apt install -y python3-opencv opencv-data
```

`python3-opencv` is OpenCV itself. `opencv-data` provides the face and eye
cascade files, which it puts in `/usr/share/opencv4/haarcascades/`.

### 3. Create the Python environment and install the servo library

Trixie doesn't allow system-wide `pip install`. Instead, make a virtual
environment that can still see the OpenCV installed by apt:

```sh
cd ~/vcceye
python3 -m venv --system-site-packages .venv
.venv/bin/pip install adafruit-circuitpython-servokit
```

### 4. Calibrate the servo directions

Point the camera at a scene with plenty of detail (a bookshelf, not a blank
wall). Then run:

```sh
.venv/bin/python calibrate.py directions
```

This moves each servo 8°, measures which way the picture shifts, and suggests
values for `PAN_SIGN`, `TILT_SIGN`, `FOV_H_DEG` and `FOV_V_DEG`. Copy those
values into `config.py`. The values already in `config.py` were measured on
this rig: increasing the pan angle turns the camera **right**, and increasing
the tilt angle tilts it **up**.

Other calibration commands:

```sh
.venv/bin/python calibrate.py move 90 90   # move pan/tilt to given angles
.venv/bin/python calibrate.py release      # stop holding the servos (they go limp)
```

### 5. Start the tracker when the Pi boots (optional)

```sh
sh autostart/install.sh
```

This installs a systemd user service (`vcceye.service`). The service is
started by an entry in `~/.config/autostart/` once the desktop has loaded, so
the preview window can open on the monitor. To remove it:

```sh
sh autostart/install.sh --uninstall
```

## Usage

### Running by hand

```sh
cd ~/vcceye
.venv/bin/python face_tracker.py              # with the preview window
.venv/bin/python face_tracker.py --headless   # no window
```

When started over SSH, the window opens on the Pi's own monitor (`DISPLAY=:0`).
Press **q** or **Esc** in the window, or Ctrl-C in the terminal, to quit. On
exit the camera returns to centre and the servos are released.

Only one copy can use the camera at a time. If the autostart service is
running, stop it first (see below).

### Controlling the autostart service

```sh
systemctl --user stop vcceye        # stop
systemctl --user start vcceye       # start
systemctl --user restart vcceye     # restart (e.g. after editing config.py)
journalctl --user-unit vcceye -f      # watch the status output
```

If the tracker crashes, the service restarts it after 5 seconds. If you quit
with q/Esc, it stays stopped until the next boot or a manual start.

### The preview window

| Shown | Meaning |
|---|---|
| Green box | The face being tracked (locked) |
| Orange box | A possible face, not locked yet |
| Red box | Other detections |
| Bottom label | "VCC Electronic Repair" on a translucent beige strip (always shown) |
| Santa hat | Drawn on the tracked face (turn off with `SANTA_HAT = False`) |
| Cyan rectangle | Dead zone: the camera doesn't move while the face is inside it |
| Top line | State (`TRACKING` / `HOLD` / `SCANNING`), servo angles, frame rate |

Set `SHOW_OVERLAY = False` in `config.py` to hide the boxes, the dead-zone
rectangle and the top line.

Once a second, the tracker also prints a status line to the terminal or the
journal:

```
TRACKING raw=31 confirmed=31 pan= 51.9 tilt= 94.3 fps=30.0
```

`raw` counts the frames in the last second that had any detection.
`confirmed` counts the frames with a locked face.

## How it works

1. **Detect:** each frame is shrunk to half size, converted to greyscale and
   searched with `haarcascade_frontalface_default.xml`.
2. **Lock:** a detection is followed only after it has been seen in 4 frames at
   about the same place *and* eyes have been found in it at least once. A face
   that stays steady for 1 s also locks without eyes, for faces that are far
   away or slightly turned. Up to 5 missed frames are allowed. The eye check
   stops the camera locking onto face-like patterns, such as picture frames on
   a wall.
3. **Centre:** the face's offset from the middle of the picture is converted to
   degrees using the camera's field of view. Each frame, the camera moves a
   fraction (`KP_PAN`/`KP_TILT`) of that offset, at most 4°. Inside the dead
   zone it doesn't move.
4. **Scan:** if no face has been seen for 1.5 s, pan sweeps between its limits
   at 20°/s. At the end of each sweep, tilt moves to the next scan level
   (0°, 25°, 50° above home, then back down). This finds people standing close
   to the camera, whose heads are above its normal view.

## Tuning (`config.py`)

| Setting | Default | Effect |
|---|---|---|
| `PAN_MIN/MAX`, `TILT_MIN/MAX` | 15–165 | Angle limits. Narrow them if the bracket binds or a servo strains at the ends. |
| `PAN_HOME`, `TILT_HOME` | 90, 90 | Start position, and the lowest scan level. |
| `KP_PAN`, `KP_TILT` | 0.25 | Higher = faster response; too high overshoots and wobbles. |
| `DEAD_ZONE` | 0.06 | Raise it if the camera twitches when the face is roughly centred. |
| `MAX_STEP_DEG` | 4.0 | Largest move in one frame. |
| `MIN_NEIGHBORS` | 6 | Higher = stricter face detection (fewer false boxes, but more missed faces). |
| `MIN_FACE_SIZE` | 30 | Smallest face, in pixels on the half-size image (60 px on the full frame). |
| `CONFIRM_FRAMES` | 4 | Frames needed to lock when eyes are found. |
| `CONFIRM_FRAMES_NO_EYES` | 30 | Frames needed to lock without eyes. Lower locks faster but makes false locks more likely; 0 turns this off. |
| `REQUIRE_EYES` | True | Set to False to turn off the eye check altogether. |
| `MISS_TOLERANCE` | 5 | Missed frames allowed before a face is dropped. |
| `SANTA_HAT` | True | Draw a Santa hat on the tracked face in the preview window. |
| `SHOW_OVERLAY` | True | Show the face boxes, dead-zone rectangle and status text. False gives a clean picture (the hat is still drawn). |
| `LABEL_TEXT` | "VCC Electronic Repair" | Label centred at the bottom of the preview, in forest green on translucent beige (`""` hides it). Size, colours and opacity: `LABEL_SCALE`, `LABEL_COLOUR`, `LABEL_BG_COLOUR`, `LABEL_BG_OPACITY`. |
| `LOST_FACE_HOLD_S` | 1.5 | How long to wait after losing a face before scanning. |
| `SCAN_SPEED_DEG_S` | 20 | Pan speed while scanning. |
| `SCAN_TILT_UP_DEG` | (0, 25, 50) | Tilt levels for scanning, in degrees above home (negative = below). Use `(0,)` for a level-only scan. |
| `SCAN_TILT_SPEED_DEG_S` | 15 | How fast tilt moves between scan levels. |
| `PULSE_MIN_US`, `PULSE_MAX_US` | 600, 2400 | Servo pulse range for the HS-65MG. |

After editing `config.py`, run `systemctl --user restart vcceye` (or restart
the program by hand).

## Troubleshooting

**Servos don't move.** Check the servo power supply on the HAT's screw
terminal, and that each plug is the right way round and on channels 0/1.
`i2cdetect -y 1` should show `40`.

**The camera moves away from the face.** A direction setting is wrong. Run
`calibrate.py directions` again and update `PAN_SIGN`/`TILT_SIGN`.

**It locks onto things that aren't faces.** With nobody in view, run:

```sh
systemctl --user stop vcceye
.venv/bin/python survey.py survey
```

This sweeps the room and saves a picture to `survey/` for every spot where the
detector fires. Then:
- raise `CONFIRM_FRAMES_NO_EYES`,
- or raise `MIN_NEIGHBORS`,
- or move or cover whatever the detector is reacting to.

**It's slow to lock onto real faces.** Lower `CONFIRM_FRAMES_NO_EYES` (e.g.
to 18), or make sure the face is well lit.

**"Could not open camera".** Another program (or a second copy of the tracker)
is using the webcam. Run `systemctl --user stop vcceye` first.

## Files

| File | Purpose |
|---|---|
| `face_tracker.py` | Main program: detection, face locking, control loop, preview |
| `servos.py` | Wrapper around Adafruit ServoKit that keeps the servos within their limits |
| `config.py` | All settings |
| `calibrate.py` | Measures servo directions and field of view; moves or releases the servos by hand |
| `survey.py` | Sweeps the scene and records detections, to track down false positives |
| `autostart/` | systemd user service, desktop autostart entry, and the `install.sh` that installs or removes them |
| `PROJECT_PLAN.md` | The original project brief |
