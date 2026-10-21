"""Driver for the 2-DOF neck (pan + tilt), two Feetech bus servos.

Built on lerobot's FeetechMotorsBus. Angles are in degrees, measured from the
middle of each joint's calibrated range (0 = centre of travel), and every goal
is clipped to that range.

Calibration and the home pose live in robot/neck/neck_calibration.json, written
by robot/neck/neck_setup.py (`calibrate`, then `set-home`).

Port: NECK_PORT env var, else the first USB serial adapter that is not one of
the robot's other serial devices (Pico lift, Teensy).
"""

from __future__ import annotations

import glob
import json
import os
import threading
import time
from dataclasses import asdict
from pathlib import Path

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

CALIB_PATH = Path(__file__).with_name("neck_calibration.json")

# Servo IDs and model, from `neck_setup.py scan` (2026-10-19: IDs 7 and 8, STS3215 @ 1 Mbps).
# Which ID is pan vs tilt: confirm with `neck_setup.py identify`.
MOTOR_IDS = {"pan": 7, "tilt": 8}
MOTOR_MODEL = os.environ.get("NECK_MODEL", "sts3215")

# Other USB serial devices on the robot that must never be treated as the neck.
_NOT_NECK = ("MicroPython", "Teensyduino")

# Servo-side speed limits (STS units: Acceleration 0-254 in 100 steps/s^2, Goal_Velocity
# steps/s, 4096 steps/rev; 0 = no limit). 2000 steps/s ~ 175 deg/s. Was 30 / 800 (~70 deg/s).
DEFAULT_ACCEL = 80
DEFAULT_VELOCITY = 2000


def find_port() -> str:
    port = os.environ.get("NECK_PORT")
    if port:
        return port
    candidates = [p for p in sorted(glob.glob("/dev/serial/by-id/*"))
                  if not any(tag in p for tag in _NOT_NECK)]
    if not candidates:
        candidates = sorted(glob.glob("/dev/ttyUSB*"))
    if not candidates:
        raise RuntimeError("No Feetech serial adapter found. Plug it in or set NECK_PORT.")
    return candidates[0]


def make_motors() -> dict[str, Motor]:
    return {name: Motor(mid, MOTOR_MODEL, MotorNormMode.DEGREES) for name, mid in MOTOR_IDS.items()}


def load_calibration(path: Path = CALIB_PATH) -> tuple[dict[str, MotorCalibration] | None, dict | None]:
    if not path.exists():
        return None, None
    data = json.loads(path.read_text())
    calib = {name: MotorCalibration(**c) for name, c in data.get("motors", {}).items()}
    return calib or None, data.get("home_deg")


def save_calibration(calib: dict[str, MotorCalibration], home_deg: dict | None,
                     path: Path = CALIB_PATH) -> None:
    data = {"model": MOTOR_MODEL,
            "motors": {name: asdict(c) for name, c in calib.items()},
            "home_deg": home_deg}
    path.write_text(json.dumps(data, indent=2) + "\n")


class Neck:
    def __init__(self, port: str | None = None, calib_path: Path = CALIB_PATH):
        self.calib_path = Path(calib_path)
        calib, self.home_deg = load_calibration(self.calib_path)
        self.bus = FeetechMotorsBus(port or find_port(), make_motors(), calibration=calib)
        # yor.py's RPC server is threaded; one serial transaction at a time.
        self._lock = threading.RLock()

    # --- connection ---
    def connect(self, configure: bool = True, velocity: int | None = None, accel: int | None = None) -> None:
        """velocity / accel: servo speed limits in STS units (default DEFAULT_VELOCITY / DEFAULT_ACCEL)."""
        self.bus.connect()
        if configure:
            velocity = DEFAULT_VELOCITY if velocity is None else int(velocity)
            accel = DEFAULT_ACCEL if accel is None else int(accel)
            with self.bus.torque_disabled():
                self.bus.configure_motors(acceleration=accel)
                for m in self.bus.motors:
                    self.bus.write("Goal_Velocity", m, velocity)
            if self.bus.calibration and not self.bus.is_calibrated:
                # The servos lost or never got the saved calibration; write it back.
                self.bus.write_calibration(self.bus.calibration)

    def close(self, disable_torque: bool = True) -> None:
        with self._lock:
            if self.bus.is_connected:
                self.bus.disconnect(disable_torque=disable_torque)

    @property
    def is_calibrated(self) -> bool:
        return bool(self.bus.calibration)

    def _require_calibration(self):
        if not self.is_calibrated:
            raise RuntimeError("Neck is not calibrated. Run: python robot/neck/neck_setup.py calibrate")

    # --- state ---
    def get_position_deg(self) -> dict[str, float]:
        self._require_calibration()
        with self._lock:
            return {k: float(v) for k, v in self.bus.sync_read("Present_Position").items()}

    def get_limits_deg(self) -> dict[str, tuple[float, float]]:
        """(min, max) reachable angle per joint, in the same degrees as get_position_deg()."""
        self._require_calibration()
        out = {}
        for name, c in self.bus.calibration.items():
            res = 4096 if MOTOR_MODEL.startswith(("sts", "sm")) else 1024
            half = (c.range_max - c.range_min) / 2 * 360 / (res - 1)
            out[name] = (-half, half)
        return out

    # --- commands ---
    def torque(self, on: bool) -> None:
        with self._lock:
            (self.bus.enable_torque if on else self.bus.disable_torque)()

    def set_position_deg(self, pan: float | None = None, tilt: float | None = None) -> dict[str, float]:
        """Move to (pan, tilt) degrees; None leaves that joint where it is. Returns the clipped goal."""
        self._require_calibration()
        limits = self.get_limits_deg()
        goal = {}
        for name, val in (("pan", pan), ("tilt", tilt)):
            if val is None:
                continue
            lo, hi = limits[name]
            goal[name] = max(lo, min(hi, float(val)))
        if goal:
            with self._lock:
                self.bus.sync_write("Goal_Position", goal)
        return goal

    def go_home(self, wait: bool = True, tol_deg: float = 2.0, timeout_s: float = 5.0) -> bool:
        if not self.home_deg:
            raise RuntimeError("No home pose saved. Run: python robot/neck/neck_setup.py set-home")
        self.torque(True)
        goal = self.set_position_deg(**self.home_deg)
        if not wait:
            return True
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            pos = self.get_position_deg()
            if all(abs(pos[k] - goal[k]) <= tol_deg for k in goal):
                return True
            time.sleep(0.05)
        return False
