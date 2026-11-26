"""Thin wrapper around Adafruit ServoKit for the pan/tilt mount."""

from adafruit_servokit import ServoKit

import config


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


class PanTilt:
    def __init__(self):
        self.kit = ServoKit(channels=16)
        self.pan_servo = self.kit.servo[config.PAN_CHANNEL]
        self.tilt_servo = self.kit.servo[config.TILT_CHANNEL]
        for s in (self.pan_servo, self.tilt_servo):
            s.set_pulse_width_range(config.PULSE_MIN_US, config.PULSE_MAX_US)
            s.actuation_range = 180
        self.pan = float(config.PAN_HOME)
        self.tilt = float(config.TILT_HOME)
        self.move_to(self.pan, self.tilt)

    def move_to(self, pan=None, tilt=None):
        if pan is not None:
            self.pan = _clamp(float(pan), config.PAN_MIN, config.PAN_MAX)
            self.pan_servo.angle = self.pan
        if tilt is not None:
            self.tilt = _clamp(float(tilt), config.TILT_MIN, config.TILT_MAX)
            self.tilt_servo.angle = self.tilt

    def move_by(self, d_pan=0.0, d_tilt=0.0):
        self.move_to(self.pan + d_pan, self.tilt + d_tilt)

    def home(self):
        self.move_to(config.PAN_HOME, config.TILT_HOME)

    def release(self):
        """Stop sending pulses so the servos go limp (no holding torque or buzz)."""
        self.pan_servo.angle = None
        self.tilt_servo.angle = None
