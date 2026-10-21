"""
collect_nav_data.py — minimal joystick teleop for base-navigation data collection.

Drives the swerve base with a gamepad and publishes telemetry on the
"teleop/episode" topic (JSON over ZMQ PUB, same wire format as vr_teleop.py;
port from robot/config.py, default 5558) for collect_data.py (NoMaD) or
collect_data_act.py (ACT) to record. No arms, no VR — just base velocity in,
base_vel/cmd_vel + measured enc_vel out.

Controls (Xbox layout):
  Start        — enable control loop
  Back         — disable control loop (stops the base)
  L1           — cycle speed preset
  A            — start / end episode (success)
  B            — end episode (fail)
  Right stick  — translation (vx, vy)
  Left stick X — yaw rate

Usage:
    python robot/teleop/collect_nav_data.py
"""

import argparse
import json
import time
from datetime import datetime

import numpy as np
import pygame
import zmq
from pygame.joystick import Joystick
from loop_rate_limiters import RateLimiter

from commlink import RPCClient

from robot.config import EPISODE_PUB_HOST, EPISODE_PUB_PORT

EPISODE_TOPIC = "teleop/episode"
EPISODE_SIGNAL_TOPIC = "episode/signal"
EPISODE_RESULT_TOPIC = "teleop/episode_result"
LOOP_RATE = 30  # Hz — publish at/above the collector's expected rate (10 Hz NoMaD, 30 Hz ACT)

XBOX_CONTROLLER_MAP = {
    "a": 0,
    "b": 1,
    "start": 7,
    "back": 6,
    "l1": 4,
    "left_horizontal_axis": 0,
    "right_horizontal_axis": 3,
    "right_vertical_axis": 4,
}


def apply_deadzone(arr, dz=0.05):
    return np.where(np.abs(arr) <= dz, 0.0, np.sign(arr) * (np.abs(arr) - dz) / (1 - dz))


class NavDataCollector:
    def __init__(self, pub_host: str, pub_port: int):
        pygame.init()
        if pygame.joystick.get_count() < 1:
            raise RuntimeError("No joystick detected")
        self.joystick = Joystick(0)
        print(f"[CollectNavData] Controller: {self.joystick.get_name()}")

        self.max_vels = [
            np.array([0.50, 0.50, 1.57]),
            np.array([0.25, 0.25, 1.57]),
            np.array([0.75, 0.75, 1.57]),
        ]
        self.max_vel_setting = 0
        self.vel_alpha = 0.9  # low-pass on commanded vel
        self.last_target_velocity = np.zeros(3, dtype=float)
        self.control_loop_running = False
        self.prev_buttons = {}

        self.collect_episode = False
        self.episode_id = None
        self.episode_seq = 0

        print("[CollectNavData] Connecting to YOR RPC at localhost:5557...")
        self.yor = RPCClient("localhost", 5557)
        self.yor.init()

        self._zmq_ctx = zmq.Context()
        self._pub_sock = self._zmq_ctx.socket(zmq.PUB)
        self._pub_sock.bind(f"tcp://{pub_host}:{pub_port}")
        print(f"[CollectNavData] Publishing '{EPISODE_TOPIC}' on tcp://{pub_host}:{pub_port}")

    def _publish(self, topic: str, data: dict):
        self._pub_sock.send_multipart([topic.encode(), json.dumps(data).encode()])

    def _pressed(self, name: str) -> bool:
        """Rising edge of a gamepad button."""
        down = bool(self.joystick.get_button(XBOX_CONTROLLER_MAP[name]))
        was_down = self.prev_buttons.get(name, False)
        self.prev_buttons[name] = down
        return down and not was_down

    def _stop_base(self):
        self.yor.set_base_velocity(np.zeros(3))
        self.last_target_velocity = np.zeros(3, dtype=float)

    def _start_episode(self):
        self.collect_episode = True
        self.episode_id = datetime.now().strftime("%Y-%m-%d--%H-%M-%S")
        self.episode_seq = 0
        self._publish(EPISODE_SIGNAL_TOPIC, {"timestamp": time.time(), "active": True, "episode_id": self.episode_id})
        print(f"[Episode] STARTED {self.episode_id}")

    def _end_episode(self, success: bool):
        self.collect_episode = False
        now = time.time()
        self._publish(EPISODE_SIGNAL_TOPIC, {"timestamp": now, "active": False, "success": success,
                                             "episode_id": self.episode_id, "num_frames": self.episode_seq})
        self._publish(EPISODE_RESULT_TOPIC, {"timestamp": now, "result": "success" if success else "fail",
                                             "episode_id": self.episode_id})
        print(f"[Episode] ENDED {self.episode_id} — {'success' if success else 'fail'} ({self.episode_seq} frames)")

    def control_loop(self):
        rate = RateLimiter(LOOP_RATE, name="collect_nav_data")
        print("[CollectNavData] Ready.")
        print(__doc__.split("Controls (Xbox layout):")[1].split("Usage:")[0])

        while True:
            pygame.event.pump()

            if self._pressed("start") and not self.control_loop_running:
                self.control_loop_running = True
                print("[CollectNavData] Control started")

            if self._pressed("back") and self.control_loop_running:
                self.control_loop_running = False
                self._stop_base()
                print("[CollectNavData] Control stopped")

            if self._pressed("a"):
                if self.collect_episode:
                    self._end_episode(success=True)
                else:
                    self._start_episode()
            if self._pressed("b") and self.collect_episode:
                self._end_episode(success=False)

            l1 = self._pressed("l1")
            target_velocity = np.zeros(3, dtype=float)
            if self.control_loop_running:
                if l1:
                    self.max_vel_setting = (self.max_vel_setting + 1) % len(self.max_vels)
                    print(f"[CollectNavData] Max velocity setting: {self.max_vel_setting}")

                vy = -self.joystick.get_axis(XBOX_CONTROLLER_MAP["right_horizontal_axis"])
                vx = -self.joystick.get_axis(XBOX_CONTROLLER_MAP["right_vertical_axis"])
                w = -self.joystick.get_axis(XBOX_CONTROLLER_MAP["left_horizontal_axis"])
                target_velocity = apply_deadzone(np.array([vx, vy, w], dtype=float))
                target_velocity = self.max_vels[self.max_vel_setting] * target_velocity
                target_velocity = (1 - self.vel_alpha) * target_velocity + self.vel_alpha * self.last_target_velocity
                target_velocity[np.abs(target_velocity) < 1e-3] = 0.0
                self.last_target_velocity = target_velocity
                # Sent every tick (zeros included): the robot stops the base if commands stop arriving.
                self.yor.set_base_velocity(target_velocity)

            # Publish every tick (even when idle) so the collector's staleness
            # check never trips between drive inputs.
            state = self.yor.get_collection_state() or {}
            self._publish(EPISODE_TOPIC, {
                "timestamp": state.get("timestamp", time.time()),
                "base_vel": target_velocity.tolist(),
                "cmd_vel": target_velocity.tolist(),
                "enc_vel": state.get("enc_vel"),
                "active": self.collect_episode,
                "episode_id": self.episode_id if self.collect_episode else None,
                "seq": self.episode_seq if self.collect_episode else None,
            })
            if self.collect_episode:
                self.episode_seq += 1

            rate.sleep()

    def stop(self):
        try:
            self._stop_base()
        except Exception:
            pass
        self._pub_sock.close(linger=0)
        self._zmq_ctx.term()


def main():
    parser = argparse.ArgumentParser(description="Minimal joystick teleop for nav data collection")
    parser.add_argument("--act", action="store_true",
                        help="No-op, kept for compatibility: base_vel, cmd_vel and enc_vel are always published")
    parser.add_argument("--pub-host", default=EPISODE_PUB_HOST, help="Interface to bind the publisher to")
    parser.add_argument("--pub-port", type=int, default=EPISODE_PUB_PORT, help="Port to bind the publisher to")
    args = parser.parse_args()

    node = NavDataCollector(pub_host=args.pub_host, pub_port=args.pub_port)
    try:
        node.control_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()


if __name__ == "__main__":
    main()
