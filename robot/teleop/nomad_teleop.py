"""
nomad_teleop.py — joystick teleoperation with optional NoMaD autonomous navigation.

Controls:
  Start      — enable control loop
  Back       — disable control loop
  A          — toggle NoMaD autonomous mode on/off (safety interlock)
  L1         — cycle speed preset (manual mode only)
  R1         — print joystick debug info
  Right stick — translation (vx, vy)   [manual mode]
  Left stick X — yaw rate              [manual mode]
  D-pad up/down — lift                 [always]

When NoMaD mode is ON the base follows waypoints streamed over TCP from the
Jetson (run_nomad_live.py).  Pressing A again immediately hands control back
to the joystick.

Usage:
    python robot/teleop/nomad_teleop.py
    python robot/teleop/nomad_teleop.py --host 192.168.123.117 --port 5005
"""

import argparse
import json
import math
import socket
import threading
import time

import numpy as np
import pygame
from loop_rate_limiters import RateLimiter
from pygame.joystick import Joystick

from commlink import RPCClient
from robot.config import ORIN_IP

# ---------------------------------------------------------------------------
# Controller maps
# ---------------------------------------------------------------------------

XBOX_CONTROLLER_MAP = {
    "start": 7,
    "back": 6,
    "a": 0,
    "l1": 4,
    "r1": 5,
    "left_horizontal_axis": 0,
    "left_vertical_axis": 1,
    "right_horizontal_axis": 3,
    "right_vertical_axis": 4,
}

PS4_CONTROLLER_MAP = {
    "start": 3,
    "back": 0,  # square
    "a": 1,     # cross
    "l1": 4,
    "r1": 5,
    "left_horizontal_axis": 0,
    "right_horizontal_axis": 3,
    "right_vertical_axis": 4,
    "pad_y": 7,
}

controller_map = XBOX_CONTROLLER_MAP

# ---------------------------------------------------------------------------
# NoMaD velocity limits (deployment/config/robot.yaml)
# ---------------------------------------------------------------------------
MAX_V = 0.2   # m/s
MAX_W = 0.4   # rad/s
GOAL_DIST_THRESHOLD = 0.3


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def apply_deadzone(arr, dz=0.05):
    return np.where(np.abs(arr) <= dz, 0.0, np.sign(arr) * (np.abs(arr) - dz) / (1 - dz))


# ---------------------------------------------------------------------------
# Background NoMaD TCP reader
# ---------------------------------------------------------------------------

NOMAD_COMMAND_TIMEOUT = 1.0  # seconds before a stale command stops the robot


class NoMaDReader:
    """Connects to the NoMaD TCP stream and keeps the latest message available."""

    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.latest: dict | None = None
        self._latest_time: float | None = None
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def get_latest(self) -> tuple[dict | None, float | None]:
        """Return (msg, age_seconds) where age is None if no message ever received."""
        with self._lock:
            age = None if self._latest_time is None else time.time() - self._latest_time
            return self.latest, age

    def _run(self):
        buf = ""
        sock = None
        while True:
            try:
                if sock is None:
                    print(f"[NoMaD] Connecting to {self.host}:{self.port} …")
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    sock.settimeout(5.0)
                    sock.connect((self.host, self.port))
                    sock.settimeout(None)
                    buf = ""
                    print("[NoMaD] Connected.")

                chunk = sock.recv(4096)
                if not chunk:
                    raise ConnectionResetError("server closed connection")
                buf += chunk.decode("utf-8")
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    msg = json.loads(line)
                    with self._lock:
                        self.latest = msg
                        self._latest_time = time.time()

            except (OSError, ConnectionResetError, json.JSONDecodeError) as e:
                print(f"[NoMaD] Stream error: {e} — reconnecting in 2 s …")
                try:
                    sock.close()
                except Exception:
                    pass
                sock = None
                time.sleep(2.0)


# ---------------------------------------------------------------------------
# Combined node
# ---------------------------------------------------------------------------

class NoMaDJoystickNode:
    def __init__(self, nomad_host: str, nomad_port: int, rpc_host: str, rpc_port: int,
                 max_v: float, max_w: float, vel_alpha: float, lookahead_idx: int):
        pygame.init()
        if pygame.joystick.get_count() < 1:
            raise RuntimeError("No joystick detected")
        self.joystick = Joystick(0)

        self.max_vels = [
            np.array([0.50, 0.50, 1.57]),
            np.array([0.25, 0.25, 1.57]),
            np.array([1.5, 1.5, 1.57]),
        ]
        self.max_vel_setting = 0
        self.vel_alpha = vel_alpha
        self.nomad_max_v = max_v
        self.nomad_max_w = max_w
        self.lookahead_idx = lookahead_idx

        self.control_loop_running = False
        self.nomad_enabled = False  # toggled by A button
        self._last_a_state = False  # for edge detection

        self.yor = RPCClient(host=rpc_host, port=rpc_port)
        self.yor.init()

        self.nomad = NoMaDReader(nomad_host, nomad_port)
        self.nomad.start()

        self.last_pad_y = 0
        self.warned_no_hat = False

    # ------------------------------------------------------------------
    # Velocity helpers
    # ------------------------------------------------------------------

    def _nomad_velocity(self, msg: dict, dist: float) -> np.ndarray:
        # Final approach / end state based on dist:
        #   dist >= 8  : full speed, normal turning
        #   4 <= dist < 8 : half speed, 3x turning
        #   dist < 4   : stop (handled by caller)
        traj = msg["trajectory"]
        wp = traj[0]
        la = traj[min(self.lookahead_idx, len(traj) - 1)]

        mag = math.hypot(wp[0], wp[1])
        if mag < 1e-4:
            return np.zeros(3)

        if dist < 8.0:
            speed_scale = 0.5
            turn_scale = 3.0
        else:
            speed_scale = 1.0
            turn_scale = 1.0

        vx = self.nomad_max_v * speed_scale * (wp[0] / mag)
        vy = self.nomad_max_v * speed_scale * (wp[1] / mag)

        angle_rad = math.atan2(-la[1], la[0])
        angular_w = clamp(
            -angle_rad * self.nomad_max_w * turn_scale,
            -self.nomad_max_w * turn_scale,
            self.nomad_max_w * turn_scale,
        )

        return np.array([vx, vy, angular_w], dtype=float)

    def _joystick_velocity(self) -> np.ndarray:
        vy = -self.joystick.get_axis(controller_map["right_horizontal_axis"])
        vx = -self.joystick.get_axis(controller_map["right_vertical_axis"])
        w  = -self.joystick.get_axis(controller_map["left_horizontal_axis"])
        vel = apply_deadzone(np.array([vx, vy, w], dtype=float))
        return self.max_vels[self.max_vel_setting] * vel

    # ------------------------------------------------------------------
    # Debug display
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # Main control loop
    # ------------------------------------------------------------------

    def control_loop(self):
        rate = RateLimiter(60, name="nomad_joystick")
        last_vel = np.zeros(3, dtype=float)
        last_nomad_seq = -1

        while True:
            pygame.event.pump()

            # --- start / stop ---
            if not self.control_loop_running and self.joystick.get_button(controller_map["start"]):
                self.control_loop_running = True
                print("Control started")

            if self.control_loop_running and self.joystick.get_button(controller_map["back"]):
                self.control_loop_running = False
                self.nomad_enabled = False
                print("Control stopped")

            if self.control_loop_running:
                # --- A button: toggle NoMaD (edge-triggered) ---
                a_state = bool(self.joystick.get_button(controller_map["a"]))
                if a_state and not self._last_a_state:
                    self.nomad_enabled = not self.nomad_enabled
                    state_str = "ENABLED" if self.nomad_enabled else "DISABLED"
                    print(f"NoMaD autonomous mode: {state_str}")
                    last_vel = np.zeros(3)  # clear carry-over from the other mode
                self._last_a_state = a_state

                # --- speed preset / debug (manual mode only) ---
                if not self.nomad_enabled:
                    if self.joystick.get_button(controller_map["l1"]):
                        self.max_vel_setting = (self.max_vel_setting + 1) % len(self.max_vels)
                        print("Max velocity setting:", self.max_vel_setting)
                        time.sleep(0.1)

                # --- velocity source ---
                if self.nomad_enabled:
                    msg, age = self.nomad.get_latest()
                    if msg is None:
                        # No message yet — hold still
                        rate.sleep()
                        continue

                    if age > NOMAD_COMMAND_TIMEOUT:
                        self.yor.set_base_velocity(np.zeros(3))
                        last_vel = np.zeros(3)
                        rate.sleep()
                        continue

                    seq = msg.get("seq", -1)
                    dist = msg.get("dist", float("inf"))
                    label = msg.get("label", "")

                    if dist < -1: #Disabled
                        print(f"[NoMaD seq {seq}] End state (dist={dist:.3f}) — holding.")
                        self.yor.set_base_velocity(np.zeros(3))
                        last_vel = np.zeros(3)
                        rate.sleep()
                        continue

                    target_vel = self._nomad_velocity(msg, dist)

                    if seq != last_nomad_seq:
                        print(
                            f"[NoMaD seq {seq:4d}] {label:<20s} "
                            f"vx={target_vel[0]:+.3f}  w={target_vel[2]:+.3f}  dist={dist:.3f}"
                        )
                        last_nomad_seq = seq
                else:
                    target_vel = self._joystick_velocity()

                # --- smooth & send ---
                target_vel = (1.0 - self.vel_alpha) * target_vel + self.vel_alpha * last_vel
                last_vel = target_vel

                if np.linalg.norm(target_vel, ord=1) > 1e-2:
                    self.yor.set_base_velocity(target_vel)

                # --- D-pad lift ---
                num_hats = self.joystick.get_numhats()
                if num_hats < 1:
                    if not self.warned_no_hat:
                        print("WARN: Controller reports 0 hats; D-pad unsupported")
                        self.warned_no_hat = True
                    pad_y = 0
                else:
                    pad_y = self.joystick.get_hat(0)[1]

                if pad_y != self.last_pad_y:
                    print(f"D-pad Y: {self.last_pad_y} -> {pad_y}")
                    if pad_y > 0:
                        self.yor.lift_up()
                    elif pad_y < 0:
                        self.yor.lift_down()
                    else:
                        self.yor.lift_stop()
                    self.last_pad_y = pad_y

            rate.sleep()


def main():
    parser = argparse.ArgumentParser(
        description="Joystick teleop with toggleable NoMaD autonomous navigation"
    )
    parser.add_argument("--host", default=ORIN_IP,
                        help=f"NoMaD server IP (Orin, default {ORIN_IP})")
    parser.add_argument("--port", type=int, default=5005,
                        help="NoMaD TCP port (default 5005)")
    parser.add_argument("--rpc-host", default="localhost",
                        help="YOR RPC host (default localhost)")
    parser.add_argument("--rpc-port", type=int, default=5557,
                        help="YOR RPC port (default 5557)")
    parser.add_argument("--max-v", type=float, default=MAX_V,
                        help=f"NoMaD max linear speed m/s (default {MAX_V})")
    parser.add_argument("--max-w", type=float, default=MAX_W,
                        help=f"NoMaD max yaw rate rad/s (default {MAX_W})")
    parser.add_argument("--vel-alpha", type=float, default=0.7,
                        help="Low-pass smoothing 0–1 (default 0.7; 0 = no smoothing)")
    parser.add_argument("--lookahead-idx", type=int, default=5,
                        help="Trajectory index used for turning direction (default 5)")
    args = parser.parse_args()

    node = NoMaDJoystickNode(
        nomad_host=args.host,
        nomad_port=args.port,
        rpc_host=args.rpc_host,
        rpc_port=args.rpc_port,
        max_v=args.max_v,
        max_w=args.max_w,
        vel_alpha=args.vel_alpha,
        lookahead_idx=args.lookahead_idx,
    )
    node.control_loop()


if __name__ == "__main__":
    main()
