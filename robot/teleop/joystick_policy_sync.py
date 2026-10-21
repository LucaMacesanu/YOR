"""
joystick_policy_sync.py — Joystick policy teleop tuned for synchronous inference.

Changes from joystick_policy_teleop.py:
  - Holds last commanded position when action is stale (no snap-back between chunks)
  - ACTION_TIMEOUT increased to 30s — allows time for Enter prompt + inference
  - Holds last position when policy is disabled via A button
  - Default policy port changed to 5572 (matches inference servers)
  - Clearer logging showing action age and hold state

Controls (Xbox layout):
  Start      — enable control loop
  Back       — disable control loop / stop policy
  A          — toggle policy rollout ON / OFF (edge-triggered)
  X          — home left arm
  B          — home right arm
"""

import argparse
import atexit
import threading
import time
import json

import numpy as np
import pygame
import zmq
from loop_rate_limiters import RateLimiter
from pygame.joystick import Joystick

from commlink import RPCClient

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

LOOP_RATE        = 30     # Hz
ACTION_TIMEOUT   = 30.0   # seconds — generous gap for Enter prompt + inference time
POLICY_TCP_PORT  = 5572   # matches inference_server_kamil.py / inference_server_sync.py

EPISODE_PUB_PORT = 5568
EPISODE_TOPIC    = "teleop/episode"

XBOX_CONTROLLER_MAP = {
    "start": 7,
    "back":  6,
    "a":     0,
    "b":     1,
    "x":     2,
    "y":     3,
}


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class JoystickPolicySync:
    def __init__(self, policy_port: int):
        pygame.init()
        if pygame.joystick.get_count() < 1:
            raise RuntimeError("No joystick detected")
        self.joystick = Joystick(0)
        print(f"[JoystickSync] Controller: {self.joystick.get_name()}")

        self.control_loop_running = False
        self.policy_enabled       = False
        self._last_a_state        = False
        self._holding             = False

        print("[JoystickSync] Connecting to YOR RPC at localhost:5557...")
        self.yor = RPCClient("localhost", 5557)
        self.yor.init()
        self.has_arms = True
        try:
            print("[JoystickSync] Homing arms...")
            self.yor.home_left_arm()
            self.yor.home_right_arm()
        except Exception as e:
            print(f"[JoystickSync] Warning: Could not home arms: {e}")
            self.has_arms = False

        self._action_lock        = threading.Lock()
        self._latest_action      = None
        self._latest_action_time = None
        self._action_thread = threading.Thread(
            target=self._action_worker, args=(policy_port,), daemon=True
        )
        self._action_thread.start()

        _ctx = zmq.Context.instance()
        self._episode_socket = _ctx.socket(zmq.PUB)
        self._episode_socket.bind(f"tcp://*:{EPISODE_PUB_PORT}")
        print(f"[JoystickSync] Subscribing to inference/left_joints_action on port {policy_port}")
        print(f"[JoystickSync] Publishing joint states on port {EPISODE_PUB_PORT}")

    def _action_worker(self, port: int):
        ctx  = zmq.Context.instance()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.SUBSCRIBE, b"inference/left_joints_action")
        sock.connect(f"tcp://127.0.0.1:{port}")
        print(f"[JoystickSync] Connected to inference server on 127.0.0.1:{port}")
        while True:
            try:
                topic, msg = sock.recv_multipart()
                pkt = json.loads(msg)
                with self._action_lock:
                    self._latest_action      = pkt
                    self._latest_action_time = time.time()
            except Exception as e:
                print(f"[JoystickSync] Subscriber error: {e}")
                time.sleep(0.1)

    def _get_latest_action(self):
        with self._action_lock:
            age = None if self._latest_action_time is None else time.time() - self._latest_action_time
            return self._latest_action, age

    def _btn(self, name: str) -> bool:
        return bool(self.joystick.get_button(XBOX_CONTROLLER_MAP[name]))

    def _send_joint_target(self, action: dict):
        left_joints  = np.array(action["left_joints"], dtype=float)
        left_gripper = action.get("left_gripper")
        if left_gripper is not None:
            left_gripper = float(left_gripper)
        self.yor.set_left_joint_target(
            joint_target=left_joints,
            gripper_target=left_gripper,
            preview_time=1 / LOOP_RATE,
        )
        return left_joints, left_gripper

    def control_loop(self):
        rate = RateLimiter(LOOP_RATE, name="joystick_policy_sync")
        print("[JoystickSync] Ready.")
        print("  Start      — enable control loop")
        print("  Back       — disable control loop")
        print("  A          — toggle policy rollout ON/OFF")
        print("  X          — home left arm")
        print("  B          — home right arm")

        while True:
            pygame.event.pump()

            if not self.control_loop_running and self._btn("start"):
                self.control_loop_running = True
                print("[JoystickSync] Control started")

            if self.control_loop_running and self._btn("back"):
                self.control_loop_running = False
                self.policy_enabled       = False
                self._last_a_state        = False
                self._holding             = False
                print("[JoystickSync] Control stopped")

            if self.control_loop_running:
                a_state = self._btn("a")
                if a_state and not self._last_a_state:
                    self.policy_enabled = not self.policy_enabled
                    print(f"[JoystickSync] Policy rollout: {'ENABLED' if self.policy_enabled else 'DISABLED'}")
                    if not self.policy_enabled:
                        self._holding = False
                self._last_a_state = a_state

                if self._btn("x") and self.has_arms:
                    print("[JoystickSync] Homing left arm")
                    self.yor.home_left_arm()
                    self.policy_enabled = False
                    self._holding       = False
                    time.sleep(0.3)

                if self._btn("b") and self.has_arms:
                    print("[JoystickSync] Homing right arm")
                    self.yor.home_right_arm()
                    self.policy_enabled = False
                    self._holding       = False
                    time.sleep(0.3)

                curr_pose = self.yor.get_bimanual_state()

                left_joints = left_gripper = right_joints = right_gripper = None

                if self.policy_enabled and self.has_arms:
                    action, age = self._get_latest_action()

                    if action is None:
                        pass  # waiting for first action

                    elif age <= ACTION_TIMEOUT:
                        # Fresh action — execute
                        left_joints, left_gripper = self._send_joint_target(action)
                        if self._holding:
                            print(f"[JoystickSync] Resuming  age={age:.3f}s")
                            self._holding = False
                        else:
                            print(f"[JoystickSync] Executing  age={age:.3f}s  joints={np.round(left_joints, 3)}")

                    else:
                        # Stale — hold last position instead of reverting
                        left_joints, left_gripper = self._send_joint_target(action)
                        if not self._holding:
                            print(f"[JoystickSync] Chunk done — holding position (age={age:.1f}s)")
                            self._holding = True

                frame = {
                    "timestamp":            curr_pose[0],
                    "left_ee":              curr_pose[1:8],
                    "left_joints":          curr_pose[8:15],
                    "left_gripper":         curr_pose[15],
                    "right_ee":             curr_pose[16:23],
                    "right_joints":         curr_pose[23:30],
                    "right_gripper":        curr_pose[30],
                    "lift":                 curr_pose[31],
                    "left_joints_target":   left_joints,
                    "right_joints_target":  right_joints,
                    "left_gripper_target":  left_gripper,
                    "right_gripper_target": right_gripper,
                    "is_recording":         self.policy_enabled,
                }
                self._episode_socket.send_multipart([
                    EPISODE_TOPIC.encode(),
                    json.dumps(
                        frame,
                        default=lambda x: x.tolist() if hasattr(x, "tolist") else float(x),
                    ).encode(),
                ])

            rate.sleep()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--policy-port", type=int, default=POLICY_TCP_PORT,
        help=f"Port to listen on for inference actions (default {POLICY_TCP_PORT})",
    )
    args = parser.parse_args()

    node = JoystickPolicySync(policy_port=args.policy_port)
    atexit.register(pygame.quit)
    node.control_loop()


if __name__ == "__main__":
    main()
