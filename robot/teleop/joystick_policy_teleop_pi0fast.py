"""
joystick_policy_teleop_pi0fast.py — Execute joint-space policy actions via joystick interlock (pi0 fast).

Controls (Xbox layout):
  Start      — enable control loop
  Back       — disable control loop / stop policy
  A          — toggle policy rollout ON / OFF (edge-triggered)
  X          — home left arm
  B          — home right arm

Action source: ZMQ SUB on the configured port, topic "inference/left_delta_joints".
Each message is a JSON object:
  {
    "left_delta_joints": [8 floats],   # 7 joint deltas (rad) + 1 gripper delta (0-1)
    "timestamp":         float         # optional
  }

Usage:
    python robot/teleop/joystick_policy_teleop_pi0fast.py
    python robot/teleop/joystick_policy_teleop_pi0fast.py --policy-port 5560
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

LOOP_RATE = 50          # Hz
ACTION_TIMEOUT = 1.0    # seconds — stale action stops arm commands
POLICY_TCP_PORT = 5570

EPISODE_PUB_PORT = 5568
EPISODE_TOPIC = "teleop/episode"

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

class JoystickPolicyTeleopPi0Fast:
    def __init__(self, policy_port: int):
        pygame.init()
        if pygame.joystick.get_count() < 1:
            raise RuntimeError("No joystick detected")
        self.joystick = Joystick(0)
        print(f"[Pi0Fast] Controller: {self.joystick.get_name()}")

        self.control_loop_running = False
        self.policy_enabled = False
        self._last_a_state = False

        # Robot RPC
        print("[Pi0Fast] Connecting to YOR RPC at localhost:5557...")
        self.yor = RPCClient("localhost", 5557)
        self.yor.init()
        self.has_arms = True
        try:
            print("[Pi0Fast] Homing arms...")
            self.yor.home_left_arm()
            self.yor.home_right_arm()
        except Exception as e:
            print(f"[Pi0Fast] Warning: Could not home arms: {e}")
            self.has_arms = False

        # Policy action subscriber (ZMQ sub/pub from inference server)
        self._action_lock = threading.Lock()
        self._latest_action: dict | None = None
        self._latest_action_time: float | None = None
        self._action_thread = threading.Thread(
            target=self._action_worker, args=(policy_port,), daemon=True
        )
        self._action_thread.start()

        # Episode publisher — raw zmq.PUB so Maxwell can subscribe with plain pyzmq
        _ctx = zmq.Context.instance()
        self._episode_socket = _ctx.socket(zmq.PUB)
        self._episode_socket.bind(f"tcp://*:{EPISODE_PUB_PORT}")
        print(f"[Pi0Fast] Subscribing to inference/left_delta_joints on port {policy_port}")
        print(f"[Pi0Fast] Publishing joint states on port {EPISODE_PUB_PORT}")

    def _action_worker(self, port: int):
        ctx = zmq.Context.instance()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.SUBSCRIBE, b"inference/left_delta_joints")
        sock.connect(f"tcp://127.0.0.1:{port}")
        print(f"[Pi0Fast] Connected to inference server on 127.0.0.1:{port}")
        import json as _json
        while True:
            try:
                topic, msg = sock.recv_multipart()
                pkt = _json.loads(msg)
                with self._action_lock:
                    self._latest_action = pkt
                    self._latest_action_time = pkt.get("timestamp", time.time())
            except Exception as e:
                print(f"[Pi0Fast] Subscriber error: {e}")
                time.sleep(0.1)

    def _get_latest_action(self) -> tuple[dict | None, float | None]:
        with self._action_lock:
            age = None if self._latest_action_time is None else time.time() - self._latest_action_time
            return self._latest_action, age

    def _btn(self, name: str) -> bool:
        return bool(self.joystick.get_button(XBOX_CONTROLLER_MAP[name]))

    def control_loop(self):
        rate = RateLimiter(LOOP_RATE, name="joystick_policy_teleop_pi0fast")
        print("[Pi0Fast] Ready.")
        print("  Start      — enable control loop")
        print("  Back       — disable control loop")
        print("  A          — toggle policy rollout ON/OFF")
        print("  X          — home left arm")
        print("  B          — home right arm")

        while True:
            pygame.event.pump()

            # --- Start / stop ---
            if not self.control_loop_running and self._btn("start"):
                self.control_loop_running = True
                print("[Pi0Fast] Control started")

            if self.control_loop_running and self._btn("back"):
                self.control_loop_running = False
                self.policy_enabled = False
                self._last_a_state = False
                print("[Pi0Fast] Control stopped")

            if self.control_loop_running:
                # --- A: toggle policy (edge-triggered) ---
                a_state = self._btn("a")
                if a_state and not self._last_a_state:
                    self.policy_enabled = not self.policy_enabled
                    state_str = "ENABLED" if self.policy_enabled else "DISABLED"
                    print(f"[Pi0Fast] Policy rollout: {state_str}")
                self._last_a_state = a_state

                # --- X: home left arm ---
                if self._btn("x") and self.has_arms:
                    print("[Pi0Fast] Homing left arm")
                    self.yor.home_left_arm()
                    self.policy_enabled = False
                    time.sleep(0.3)

                # --- B: home right arm ---
                if self._btn("b") and self.has_arms:
                    print("[Pi0Fast] Homing right arm")
                    self.yor.home_right_arm()
                    self.policy_enabled = False
                    time.sleep(0.3)

                # --- Publish joint states continuously while control is running ---
                curr_pose = self.yor.get_bimanual_state()

                # --- Execute policy actions ---
                left_joints = None
                left_gripper = None
                right_joints = None
                right_gripper = None
                if self.policy_enabled and self.has_arms:
                    action, age = self._get_latest_action()

                    if action is not None and age <= ACTION_TIMEOUT:
                        delta = np.array(action["left_delta_joints"], dtype=float)
                        delta_joints = delta[:7]
                        delta_gripper = float(delta[7]) if len(delta) > 7 else 0.0
                        current_joints = np.array(curr_pose[8:15], dtype=float)
                        current_gripper = float(curr_pose[15])
                        left_joints = (current_joints + delta_joints).tolist()
                        left_gripper = float(np.clip(current_gripper + delta_gripper, 0.0, 1.0))
                        preview_time = 1.0 / LOOP_RATE

                        print(f"[Action] age={age:.3f}s  delta={np.round(delta_joints, 4).tolist()}  gripper={left_gripper:.3f}")

                        self.yor.set_left_joint_target(
                            joint_target=np.array(left_joints, dtype=float),
                            gripper_target=left_gripper,
                            preview_time=preview_time,
                        )
                    elif action is not None and age > ACTION_TIMEOUT:
                        print("[Pi0Fast] Action timed out — holding position")

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
                }
                self._episode_socket.send_multipart([
                    EPISODE_TOPIC.encode(),
                    json.dumps(frame, default=lambda x: x.tolist() if hasattr(x, "tolist") else float(x)).encode(),
                ])

            rate.sleep()


def main():
    parser = argparse.ArgumentParser(
        description="Joystick-gated pi0 fast teleop: toggle rollout with A, home arms with X/B"
    )
    parser.add_argument("--policy-port", type=int, default=POLICY_TCP_PORT,
                        help=f"TCP port to listen on for policy actions (default {POLICY_TCP_PORT})")
    args = parser.parse_args()

    node = JoystickPolicyTeleopPi0Fast(policy_port=args.policy_port)
    atexit.register(pygame.quit)
    node.control_loop()


if __name__ == "__main__":
    main()
