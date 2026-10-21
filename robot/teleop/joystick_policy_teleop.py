"""
joystick_policy_teleop.py — Execute policy actions via joystick interlock.

Controls (Xbox layout):
  Start      — enable control loop
  Back       — disable control loop / stop policy
  A          — toggle policy rollout ON / OFF (edge-triggered)
  X          — home left arm
  B          — home right arm

Three action topics are accepted on the same ZMQ SUB socket (port 5572), since only one
inference backend ever runs at a time:
  - "inference/left_joints_action" (ACT/pi0.5/RICL, joint-space, left arm only):
      {"left_joints": [7 floats], "left_gripper": float}
    -> self.yor.set_left_joint_target(...)
  - "inference/bimanual_joints_action" (added 2026-09-08, for absolute-joint-space checkpoints
    like yor_icl_pi05_easy_pnp_v2_absolute_joint_sanity15k, joint-space, bimanual):
      {"left_joints": [7 floats], "right_joints": [7 floats],
       "left_gripper": float | null, "right_gripper": float | null}
    -> self.yor.set_left_joint_target(...) and self.yor.set_right_joint_target(...), independently,
       same non-blocking path as the left-only topic above. Deliberately no lift here even though
       these checkpoints predict a lift dim: self.yor.set_lift_position() BLOCKS (waits up to 30s
       for the lift to physically converge) unlike the joint/EE target setters, which just fire a
       non-blocking target at the motor controller -- calling it every tick from this loop would
       stall this loop's own Back-button e-stop read. The inference server (see
       inference_server_openpi.py's is_absolute_joint publish block) never sends left_gripper/
       right_gripper either -- passing None down to set_*_joint_target is that function's
       documented "leave gripper alone" no-op, not a missing feature.
  - "inference/bimanual_ee_action" (plain openpi/pi0.5-native, EE-space, bimanual):
      {"left_ee": [qw,qx,qy,qz,x,y,z], "right_ee": [qw,qx,qy,qz,x,y,z],
       "left_gripper": float, "right_gripper": float}
    -> mink.SE3(wxyz_xyz=...) for each arm, then self.yor.set_bimanual_ee_target(...),
       which runs the robot's own IK (ik_solver.py) -- same path vr_teleop.py/yor_teleop.py
       already use. No IK happens in this script or on the inference server.
If more than one happens to be fresh at once (shouldn't normally occur -- only possible right
after switching which backend is running), priority is EE action, then bimanual-joints action,
then left-joints-only action -- each is a superset of what comes after it in that list.

Usage:
    python robot/teleop/joystick_policy_teleop.py
    python robot/teleop/joystick_policy_teleop.py --policy-port 5560
    python robot/teleop/joystick_policy_teleop.py --action-host 192.168.123.89
"""

import argparse
import atexit
import threading
import time

import json

import mink
import numpy as np
import pygame
import zmq
from loop_rate_limiters import RateLimiter
from pygame.joystick import Joystick

from commlink import RPCClient

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

LOOP_RATE = 30          # Hz — must match inference publish rate and training FPS
ACTION_TIMEOUT = 1.0    # seconds — stale action stops arm commands
# All 4 inference_server_*.py backends default --action-port to 5572; this used to be
# 5570 here, a stale mismatch left over from whenever these two sides last diverged.
POLICY_TCP_PORT = 5572
# Inference server's LAN IP. Previously always 127.0.0.1 because the inference server
# reached the Pi through an SSH reverse tunnel; now that the inference machine (lairpc)
# is on the same LAN as the robot, connect to it directly instead.
DEFAULT_ACTION_HOST = "192.168.123.89"

EPISODE_PUB_PORT = 5568
EPISODE_TOPIC = "teleop/episode"

JOINT_ACTION_TOPIC = b"inference/left_joints_action"
BIMANUAL_JOINT_ACTION_TOPIC = b"inference/bimanual_joints_action"
EE_ACTION_TOPIC = b"inference/bimanual_ee_action"

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

class JoystickPolicyTeleop:
    def __init__(self, policy_port: int, action_host: str = DEFAULT_ACTION_HOST):
        pygame.init()
        if pygame.joystick.get_count() < 1:
            raise RuntimeError("No joystick detected")
        self.joystick = Joystick(0)
        print(f"[JoystickPolicy] Controller: {self.joystick.get_name()}")

        self.control_loop_running = False
        self.policy_enabled = False
        self._last_a_state = False

        # Robot RPC
        print("[JoystickPolicy] Connecting to YOR RPC at localhost:5557...")
        self.yor = RPCClient("localhost", 5557)
        self.yor.init()
        self.has_arms = True
        try:
            print("[JoystickPolicy] Homing arms...")
            self.yor.home_left_arm()
            self.yor.home_right_arm()
        except Exception as e:
            print(f"[JoystickPolicy] Warning: Could not home arms: {e}")
            self.has_arms = False

        # Policy action subscriber (ZMQ sub/pub from inference server). All three action topics
        # arrive on this one socket -- see module docstring.
        self._action_lock = threading.Lock()
        self._latest_action: dict | None = None
        self._latest_action_time: float | None = None
        self._latest_bimanual_joint_action: dict | None = None
        self._latest_bimanual_joint_action_time: float | None = None
        self._latest_ee_action: dict | None = None
        self._latest_ee_action_time: float | None = None
        self._action_thread = threading.Thread(
            target=self._action_worker, args=(action_host, policy_port), daemon=True
        )
        self._action_thread.start()

        # Episode publisher — raw zmq.PUB so the inference server can subscribe with plain pyzmq
        _ctx = zmq.Context.instance()
        self._episode_socket = _ctx.socket(zmq.PUB)
        self._episode_socket.bind(f"tcp://*:{EPISODE_PUB_PORT}")
        print(f"[JoystickPolicy] Subscribing to policy actions on {action_host}:{policy_port} "
              f"(topics: {JOINT_ACTION_TOPIC.decode()!r}, {BIMANUAL_JOINT_ACTION_TOPIC.decode()!r}, "
              f"{EE_ACTION_TOPIC.decode()!r})")
        print(f"[JoystickPolicy] Publishing joint states on port {EPISODE_PUB_PORT}")

    def _action_worker(self, host: str, port: int):
        ctx = zmq.Context.instance()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.SUBSCRIBE, JOINT_ACTION_TOPIC)
        sock.setsockopt(zmq.SUBSCRIBE, BIMANUAL_JOINT_ACTION_TOPIC)
        sock.setsockopt(zmq.SUBSCRIBE, EE_ACTION_TOPIC)
        sock.connect(f"tcp://{host}:{port}")
        print(f"[Policy] Connected to inference server on {host}:{port}")
        import json as _json
        while True:
            try:
                topic, msg = sock.recv_multipart()
                pkt = _json.loads(msg)
                now = time.time()  # Pi-local receive time, avoids cross-machine clock skew
                with self._action_lock:
                    if topic == EE_ACTION_TOPIC:
                        self._latest_ee_action = pkt
                        self._latest_ee_action_time = now
                    elif topic == BIMANUAL_JOINT_ACTION_TOPIC:
                        self._latest_bimanual_joint_action = pkt
                        self._latest_bimanual_joint_action_time = now
                    else:
                        self._latest_action = pkt
                        self._latest_action_time = now
            except Exception as e:
                print(f"[Policy] Subscriber error: {e}")
                time.sleep(0.1)

    def _get_latest_action(self) -> tuple[dict | None, float | None]:
        with self._action_lock:
            age = None if self._latest_action_time is None else time.time() - self._latest_action_time
            return self._latest_action, age

    def _get_latest_bimanual_joint_action(self) -> tuple[dict | None, float | None]:
        with self._action_lock:
            age = (None if self._latest_bimanual_joint_action_time is None
                    else time.time() - self._latest_bimanual_joint_action_time)
            return self._latest_bimanual_joint_action, age

    def _get_latest_ee_action(self) -> tuple[dict | None, float | None]:
        with self._action_lock:
            age = None if self._latest_ee_action_time is None else time.time() - self._latest_ee_action_time
            return self._latest_ee_action, age

    def _btn(self, name: str) -> bool:
        return bool(self.joystick.get_button(XBOX_CONTROLLER_MAP[name]))

    def control_loop(self):
        rate = RateLimiter(LOOP_RATE, name="joystick_policy_teleop")
        print("[JoystickPolicy] Ready.")
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
                print("[JoystickPolicy] Control started")

            if self.control_loop_running and self._btn("back"):
                self.control_loop_running = False
                self.policy_enabled = False
                self._last_a_state = False
                print("[JoystickPolicy] Control stopped")

            if self.control_loop_running:
                # --- A: toggle policy (edge-triggered) ---
                a_state = self._btn("a")
                if a_state and not self._last_a_state:
                    self.policy_enabled = not self.policy_enabled
                    state_str = "ENABLED" if self.policy_enabled else "DISABLED"
                    print(f"[JoystickPolicy] Policy rollout: {state_str}")
                self._last_a_state = a_state

                # --- X: home both arms ---
                if self._btn("x") and self.has_arms:
                    print("[JoystickPolicy] Homing both arms")
                    self.yor.home_left_arm()
                    self.yor.home_right_arm()
                    self.policy_enabled = False
                    time.sleep(0.3)

                # --- B: home right arm ---
                if self._btn("b") and self.has_arms:
                    print("[JoystickPolicy] Homing right arm")
                    self.yor.home_right_arm()
                    self.policy_enabled = False
                    time.sleep(0.3)

                # --- Publish joint states continuously while control is running ---
                curr_pose = self.yor.get_bimanual_state()

                # --- Execute policy actions ---
                # Priority when more than one topic happens to be fresh at once: EE action, then
                # bimanual-joints action, then left-joints-only action -- each is a superset of
                # what comes after it in that list. Only one inference backend runs at a time in
                # practice, so this ordering rarely matters, but it's a deliberate choice rather
                # than an accident of dict iteration order. See module docstring.
                left_joints = left_gripper = right_joints = right_gripper = None
                left_ik_solved = right_ik_solved = None
                if self.policy_enabled and self.has_arms:
                    ee_action, ee_age = self._get_latest_ee_action()
                    bij_action, bij_age = self._get_latest_bimanual_joint_action()
                    action, age = self._get_latest_action()

                    if ee_action is not None and ee_age <= ACTION_TIMEOUT:
                        left_ee = mink.SE3(wxyz_xyz=np.array(ee_action["left_ee"], dtype=float))
                        right_ee = mink.SE3(wxyz_xyz=np.array(ee_action["right_ee"], dtype=float))
                        left_gripper = ee_action.get("left_gripper")
                        right_gripper = ee_action.get("right_gripper")
                        if left_gripper is not None:
                            left_gripper = float(left_gripper)
                        if right_gripper is not None:
                            right_gripper = float(right_gripper)

                        print(f"[Action/EE] age={ee_age:.3f}s  "
                              f"L_ee={np.round(left_ee.wxyz_xyz, 3)}  R_ee={np.round(right_ee.wxyz_xyz, 3)}  "
                              f"L_grip={left_gripper}  R_grip={right_gripper}")

                        self.yor.set_bimanual_ee_target(
                            L_ee_target=left_ee,
                            R_ee_target=right_ee,
                            L_gripper_target=left_gripper,
                            L_preview_time=1/30,
                            R_gripper_target=right_gripper,
                            R_preview_time=1/30,
                        )
                        # IK convergence check (2026-09-10) -- the EE-action path is the only one
                        # that runs IK (set_left/right_joint_target below go straight to the
                        # motor controller, no solve involved), so this is only meaningful here.
                        # See yor.py's get_ik_status()/ArmNode.last_ik_solved.
                        left_ik_solved, right_ik_solved = self.yor.get_ik_status()
                        if left_ik_solved is False or right_ik_solved is False:
                            print(f"[JoystickPolicy] WARNING: IK not converged -- "
                                  f"left_solved={left_ik_solved} right_solved={right_ik_solved}")
                    elif bij_action is not None and bij_age <= ACTION_TIMEOUT:
                        left_joints = np.array(bij_action["left_joints"], dtype=float)
                        right_joints = np.array(bij_action["right_joints"], dtype=float)
                        left_gripper = bij_action.get("left_gripper")
                        right_gripper = bij_action.get("right_gripper")
                        if left_gripper is not None:
                            left_gripper = float(left_gripper)
                        if right_gripper is not None:
                            right_gripper = float(right_gripper)
                        preview_time = 1/30

                        print(f"[Action/BimanualJoints] age={bij_age:.3f}s  L={left_joints}  "
                              f"R={right_joints}  L_grip={left_gripper}  R_grip={right_gripper}")

                        # Two independent non-blocking calls, same as set_bimanual_ee_target does
                        # internally for its own two arms -- no lift here, see module docstring.
                        self.yor.set_left_joint_target(
                            joint_target=left_joints,
                            gripper_target=left_gripper,
                            preview_time=preview_time,
                        )
                        self.yor.set_right_joint_target(
                            joint_target=right_joints,
                            gripper_target=right_gripper,
                            preview_time=preview_time,
                        )
                    elif action is not None and age <= ACTION_TIMEOUT:
                        left_joints = np.array(action["left_joints"], dtype=float)
                        left_gripper = action.get("left_gripper")
                        if left_gripper is not None:
                            left_gripper = float(left_gripper)
                        preview_time = 1/30

                        print(f"[Action] age={age:.3f}s  joints={left_joints}  gripper={left_gripper}")

                        self.yor.set_left_joint_target(
                            joint_target=left_joints,
                            gripper_target=left_gripper,
                            preview_time=preview_time,
                        )
                    elif ((action is not None and age > ACTION_TIMEOUT)
                          or (bij_action is not None and bij_age > ACTION_TIMEOUT)
                          or (ee_action is not None and ee_age > ACTION_TIMEOUT)):
                        print("[JoystickPolicy] Action timed out — holding position")

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
                    "left_ik_solved":       left_ik_solved,
                    "right_ik_solved":      right_ik_solved,
                    "is_recording":         self.policy_enabled,
                }
                self._episode_socket.send_multipart([
                    EPISODE_TOPIC.encode(),
                    json.dumps(frame, default=lambda x: x.tolist() if hasattr(x, "tolist") else float(x)).encode(),
                ])
                # print(f"[Publish] L={np.round(curr_pose[8:15], 3).tolist()}  R={np.round(curr_pose[23:30], 3).tolist()}")

            rate.sleep()


def main():
    parser = argparse.ArgumentParser(
        description="Joystick-gated policy teleop: toggle rollout with A, home arms with X/B"
    )
    parser.add_argument("--policy-port", type=int, default=POLICY_TCP_PORT,
                        help=f"TCP port to listen on for policy actions (default {POLICY_TCP_PORT})")
    parser.add_argument("--action-host", default=DEFAULT_ACTION_HOST,
                        help=f"Inference server LAN IP to pull actions from (default {DEFAULT_ACTION_HOST})")
    args = parser.parse_args()

    node = JoystickPolicyTeleop(policy_port=args.policy_port, action_host=args.action_host)
    atexit.register(pygame.quit)
    node.control_loop()


if __name__ == "__main__":
    main()
