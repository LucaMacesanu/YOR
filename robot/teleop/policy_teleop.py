"""
policy_teleop.py — Execute joint-space actions streamed from a policy over TCP.

A VR controller (Oculus) is used as a safety interlock:
  Left thumbstick (click) — toggle policy control ON / OFF

TCP protocol: newline-delimited JSON on the configured listening port.
Each message must contain:
  {
    "left_joints":   [7 floats],   # rad, joint-space target for left arm
    "left_gripper":  float,        # 0.0 (closed) – 1.0 (open); omit to leave unchanged
    "right_joints":  [7 floats],   # rad, joint-space target for right arm
    "right_gripper": float,        # 0.0 (closed) – 1.0 (open); omit to leave unchanged
    "preview_time":  float         # optional, seconds (default 1/30)
  }
  Omitting left_joints or right_joints skips that arm for that step.

Usage:
    python robot/teleop/policy_teleop.py
    python robot/teleop/policy_teleop.py --policy-port 6000 --vr-host 192.168.0.195
"""

import argparse
import atexit
import json
import socket
import threading
import time

import numpy as np
import zmq
from loop_rate_limiters import RateLimiter

from commlink import RPCClient
from robot.config import VR_IP
from robot.teleop.oculus_msgs import parse_controller_state

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

VR_TCP_HOST = VR_IP
VR_TCP_PORT = 5555
VR_CONTROLLER_TOPIC = b"oculus_controller"

EPISODE_PUB_PORT = 5568
EPISODE_TOPIC = "teleop/episode"

LOOP_RATE = 30              # Hz
ACTION_TIMEOUT = 1.0        # seconds — stale action stops arm commands
POLICY_TCP_PORT = 5570      # default port to listen on for policy actions


# ---------------------------------------------------------------------------
# Policy action TCP server (policy connects to us)
# ---------------------------------------------------------------------------

class PolicyActionServer:
    """Listens on a TCP port for newline-delimited JSON action messages."""

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
        """Return (action_dict, age_seconds); age is None if no message received yet."""
        with self._lock:
            age = None if self._latest_time is None else time.time() - self._latest_time
            return self.latest, age

    def _handle_client(self, conn, addr):
        print(f"[Policy] Connection from {addr}")
        buf = ""
        try:
            while True:
                chunk = conn.recv(4096)
                if not chunk:
                    break
                buf += chunk.decode("utf-8")
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        msg = json.loads(line)
                        with self._lock:
                            self.latest = msg
                            self._latest_time = time.time()
                    except json.JSONDecodeError as e:
                        print(f"[Policy] JSON parse error: {e}")
        except OSError:
            pass
        finally:
            conn.close()
            print(f"[Policy] Connection from {addr} closed")

    def _run(self):
        while True:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                    sock.connect((self.host, self.port))
                    print(f"[Policy] Connected to action server at {self.host}:{self.port}")
                    self._handle_client(sock, (self.host, self.port))
            except OSError as e:
                print(f"[Policy] Connection failed ({e}), retrying in 2s...")
                time.sleep(2)


# ---------------------------------------------------------------------------
# Main teleop node
# ---------------------------------------------------------------------------

class PolicyTeleop:
    def __init__(self, vr_host: str, policy_port: int):
        self.policy_enabled = False
        self.stop_event = threading.Event()

        # Robot RPC
        print("[PolicyTeleop] Connecting to YOR RPC at localhost:5557...")
        self.yor = RPCClient("localhost", 5557)
        self.yor.init()
        self.has_arms = True
        try:
            print("[PolicyTeleop] Homing arms...")
            self.yor.home_left_arm()
            self.yor.home_right_arm()
        except Exception as e:
            print(f"[PolicyTeleop] Warning: Could not home arms: {e}")
            self.has_arms = False

        # VR controller background reader
        self.latest_controller_state = None
        self.controller_state_lock = threading.Lock()
        self._vr_host = vr_host
        self._vr_thread = threading.Thread(target=self._oculus_worker, daemon=True)
        self._vr_thread.start()

        # Policy action server
        self.action_server = PolicyActionServer("localhost", policy_port)
        self.action_server.start()

        # State publisher: JSON over ZMQ PUB, same wire format as vr_teleop
        # (on its own port so both can run at once).
        self._zmq_ctx = zmq.Context()
        self._pub_sock = self._zmq_ctx.socket(zmq.PUB)
        self._pub_sock.bind(f"tcp://*:{EPISODE_PUB_PORT}")
        print(f"[PolicyTeleop] State publisher on port {EPISODE_PUB_PORT}")

    def _publish(self, topic: str, data: dict):
        self._pub_sock.send_multipart([topic.encode(), json.dumps(data).encode()])

    def _oculus_worker(self):
        print(f"[PolicyTeleop] Connecting to Oculus at {self._vr_host}:{VR_TCP_PORT}...")
        zmq_context = zmq.Context()
        stick_socket = zmq_context.socket(zmq.SUB)
        stick_socket.setsockopt(zmq.RCVTIMEO, 200)  # so stop() isn't blocked by a silent headset
        stick_socket.connect(f"tcp://{self._vr_host}:{VR_TCP_PORT}")
        stick_socket.subscribe(VR_CONTROLLER_TOPIC)

        while not self.stop_event.is_set():
            try:
                _, message = stick_socket.recv_multipart()
                controller_state = parse_controller_state(message.decode())
                with self.controller_state_lock:
                    self.latest_controller_state = controller_state
            except zmq.Again:
                continue
            except zmq.ZMQError:
                time.sleep(0.1)
            except Exception:
                pass

        stick_socket.close(linger=0)
        zmq_context.term()

    def control_loop(self):
        rate = RateLimiter(LOOP_RATE, name="policy_teleop")
        print("[PolicyTeleop] Ready.")
        print("  Left thumbstick (click) — toggle policy control ON/OFF")
        print(f"  Listening for policy actions on port {self.action_server.port}")

        prev_pressed = False
        while not self.stop_event.is_set():
            with self.controller_state_lock:
                controller_state = self.latest_controller_state

            if controller_state is None:
                print("[PolicyTeleop] Waiting for VR controller...")
                rate.sleep()
                continue

            # --- Toggle policy control via left thumbstick click (rising edge) ---
            pressed = controller_state.left_thumbstick
            if pressed and not prev_pressed:
                self.policy_enabled = not self.policy_enabled
                state_str = "ENABLED" if self.policy_enabled else "DISABLED"
                print(f"[PolicyTeleop] Policy control: {state_str}")
            prev_pressed = pressed

            if not self.policy_enabled or not self.has_arms:
                rate.sleep()
                continue

            # --- Execute policy actions ---
            action, age = self.action_server.get_latest()

            if action is None:
                rate.sleep()
                continue

            if age > ACTION_TIMEOUT:
                print("[PolicyTeleop] Action timed out — holding position")
                rate.sleep()
                continue

            left_joints = action.get("left_joints")
            # A missing gripper key leaves the gripper where it is (None), rather than closing it.
            left_gripper = action.get("left_gripper")
            left_gripper = None if left_gripper is None else float(left_gripper)
            right_joints = action.get("right_joints")
            right_gripper = action.get("right_gripper")
            right_gripper = None if right_gripper is None else float(right_gripper)
            preview_time = float(action.get("preview_time", 1.0 / LOOP_RATE))

            if left_joints is not None:
                self.yor.set_left_joint_target(
                    joint_target=np.array(left_joints, dtype=float),
                    gripper_target=left_gripper,
                    preview_time=preview_time,
                )
            if right_joints is not None:
                self.yor.set_right_joint_target(
                    joint_target=np.array(right_joints, dtype=float),
                    gripper_target=right_gripper,
                    preview_time=preview_time,
                )

            # --- Publish state frame (same format as vr_teleop episode topic) ---
            curr_pose = self.yor.get_bimanual_state()
            self._publish(EPISODE_TOPIC, {
                "timestamp":        curr_pose[0],
                "left_ee":          curr_pose[1:8],
                "left_joints":      curr_pose[8:15],
                "left_gripper":     curr_pose[15],
                "right_ee":         curr_pose[16:23],
                "right_joints":     curr_pose[23:30],
                "right_gripper":    curr_pose[30],
                "left_joints_target":  left_joints,
                "right_joints_target": right_joints,
                "left_gripper_target": left_gripper,
                "right_gripper_target": right_gripper,
            })

            rate.sleep()

    def stop(self):
        self.stop_event.set()
        self._vr_thread.join(timeout=2.0)
        self._pub_sock.close(linger=0)
        self._zmq_ctx.term()


def main():
    parser = argparse.ArgumentParser(
        description="Execute joint-space actions from a policy over TCP, toggled via VR left thumbstick"
    )
    parser.add_argument("--policy-port", type=int, default=POLICY_TCP_PORT,
                        help=f"TCP port to listen on for policy actions (default {POLICY_TCP_PORT})")
    parser.add_argument("--vr-host", default=VR_TCP_HOST,
                        help=f"VR controller ZMQ host (default {VR_TCP_HOST})")
    args = parser.parse_args()

    teleop = PolicyTeleop(vr_host=args.vr_host, policy_port=args.policy_port)
    atexit.register(teleop.stop)
    teleop.control_loop()


if __name__ == "__main__":
    main()
