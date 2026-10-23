import csv
from datetime import datetime
import json
import zmq
import numpy as np
import atexit
import time
import threading
import os

from loop_rate_limiters import RateLimiter
import mink

from robot.teleop.oculus_msgs import parse_controller_state

from commlink import RPCClient
import argparse

from robot.config import VR_IP, ZED_IP, ZED_PORT, EPISODE_PUB_HOST, EPISODE_PUB_PORT
from robot.teleop.episode_recorder import EpisodeRecorder

# Defaults only — overridable per-run via the CLI flags at the bottom of this
# file, or the matching env vars (see robot/config.py).
EPISODE_TOPIC = "teleop/episode"
EPISODE_RESULT_TOPIC = "teleop/episode_result"
EPISODE_SIGNAL_TOPIC = "episode/signal"


def apply_deadzone(arr, deadzone_size=0.05):
    return np.where(np.abs(arr) <= deadzone_size, 0, np.sign(arr) * (np.abs(arr) - deadzone_size) / (1 - deadzone_size))


# VR Constants
VR_TCP_HOST = VR_IP
ZED_POSE_HOST = ZED_IP
VR_TCP_PORT = 5555
VR_CONTROLLER_TOPIC = b"oculus_controller"
VR_RECV_TIMEOUT_MS = 200  # lets the worker notice stop_event instead of blocking forever

LOOP_RATE = 30  # Hz
CSV_FLUSH_EVERY = 30  # rows; ~1 s of data at risk on a crash
EPISODE_DATA_DIR = "teleop_data/episodes"

# Base speed presets, chosen per run with --speed (not toggled by recording).
SPEED_PRESETS = {
    "teleop":    np.array([0.5, 0.5, 1.57]),
    "slow":      np.array([0.25, 0.25, 1.57]),
    "fast":      np.array([0.75, 0.75, 1.57]),
    "recording": np.array([0.15, 0.15, 1.57 / 2.5]),
}

ARMS = ("left", "right")
WXYZ_XYZ = ['w', 'qx', 'qy', 'qz', 'x', 'y', 'z']


def _gripper_error(state: dict, side: str):
    """Why state has no measured gripper opening for this arm, or None when it has one."""
    if state.get(f"{side}_gripper") is not None:
        return None
    return state.get(f"{side}_gripper_error") or "no measured gripper opening from yor.py"


def _csv_header(zed_pose: bool) -> list:
    header = ['timestamp']
    for side in ARMS:
        header += [f'{side}_ee_actual_{i}' for i in WXYZ_XYZ]
        header += [f'{side}_joint_actual_{i}' for i in range(7)]
        header += [f'{side}_gripper_actual', f'{side}_gripper_error']
    for side in ARMS:
        header += [f'{side}_ee_target_{i}' for i in WXYZ_XYZ]
        header += [f'{side}_gripper_cmd', f'{side}_ik_ok']
    header += ['base_vx_cmd', 'base_vy_cmd', 'base_w_cmd']
    header += ['base_vx_enc', 'base_vy_enc', 'base_w_enc']
    if zed_pose:
        header += ['zed_timestamp']
        header += [f'base_zed_{i}' for i in WXYZ_XYZ]
        header += [f'cam_zed_{i}' for i in WXYZ_XYZ]
    return header


class YORAdvancedTeleop:
    def __init__(self, zed_pose: bool = False, zed_image: bool = False, reset_base_after_data_collection: bool = False, no_gripper: bool = False, act: bool = False,
                 pub_host: str = None, pub_port: int = None, vr_ip: str = None, zed_ip: str = None,
                 speed: str = "teleop", session_meta: dict | None = None,
                 episode_dir: str = EPISODE_DATA_DIR, episode_zed_pose: bool = True):
        # Network endpoints: None keeps the module-level default, so an omitted
        # argument reproduces the pre-flag behaviour exactly.
        self.pub_host = pub_host or EPISODE_PUB_HOST
        self.pub_port = pub_port or EPISODE_PUB_PORT
        self.vr_ip = vr_ip or VR_TCP_HOST
        self.zed_ip = zed_ip or ZED_POSE_HOST
        self.reset_base_after_data_collection = reset_base_after_data_collection
        self.no_gripper = no_gripper
        self.act = act  # kept for CLI compatibility; cmd_vel/enc_vel are now always published

        # teleop state
        self.zed_pose = zed_pose
        self.zed_image = zed_image
        print(f"[YORTeleop] ZED base pose recording: {self.zed_pose}, ZED image recording: {self.zed_image}")
        self.start_teleop_left = False
        self.start_teleop_right = False
        self.start_base_control = False

        # CSV recording state (rows stream to disk as they're captured)
        self.record_data = False
        self.csv_file = None
        self.csv_writer = None
        self.csv_filename = None
        self.csv_rows = 0
        self.video_writer = None
        self.video_path = None
        self.base_cmd_history = []  # base velocities sent while recording, for --reset_base

        # Episode state. session_meta (task prompt, operator, object, locations...)
        # is attached to every episode's meta.json and start signal.
        self.collect_episode = False
        self.episode_id = None
        self.episode_seq = 0
        self.session_meta = session_meta or {}
        self.recorder = EpisodeRecorder(episode_dir, self.session_meta, LOOP_RATE)
        if not self.session_meta.get("task"):
            print("[YORTeleop] WARNING: no --task given; episodes can't be started without a task prompt")

        # Position transform: controller -> robot world frame
        self.H = mink.SE3.from_rotation(mink.SO3.from_matrix(np.array([[0, -1, 0], [0, 0, 1], [-1, 0, 0]])))
        # Rotation transform (permutation 5): correctly maps controller rotation axes to robot EE axes
        # All 6 row permutations of H rows: [0,-1,0],[0,0,1],[-1,0,0]
        # 1: (0,1,2) original          [[0,-1,0], [0,0,1],  [-1,0,0]] — position ✓, rotation ✗
        # 2: (1,0,2) swap rows 0,1     [[0,0,1],  [0,-1,0], [-1,0,0]] — no
        # 3: (0,2,1) swap rows 1,2     [[0,-1,0], [-1,0,0], [0,0,1]]  — no
        # 4: (2,1,0) swap rows 0,2     [[-1,0,0], [0,0,1],  [0,-1,0]] — no
        # 5: (1,2,0) cyclic            [[0,0,1],  [-1,0,0], [0,-1,0]] — rotation ✓
        # 6: (2,0,1) cyclic other      [[-1,0,0], [0,-1,0], [0,0,1]]  — untested
        self.H_rot = mink.SO3.from_matrix(np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]))
        self.X_Cinit = {"left": None, "right": None}
        self.X_ee_init = {"left": None, "right": None}

        # base control state
        self.max_vel = SPEED_PRESETS[speed]
        self.vel_alpha = 0.9
        self.last_target_velocity = np.zeros(3)

        # arm smoothing (lerp)
        self.ee_alpha = 0.5      # EE pose smoothing (0=no smooth, 1=frozen)
        self.gripper_alpha = 0.3  # gripper EMA smoothing (noise filter)
        self.gripper_max_delta = 0.05  # max gripper change per step (slew rate, at 30 Hz: ~0.67s full travel)
        self.last_ee_target = {"left": None, "right": None}
        # 1.0 = open. Re-seeded from the actual gripper on every engage so the
        # slew limiter starts where the jaws are instead of snapping them shut.
        self.last_gripper = {"left": 1.0, "right": 1.0}

        # Robot RPC connection
        print("[YORTeleop] Connecting to YOR RPC at localhost:5557...")
        self.yor = RPCClient("localhost", 5557)
        self.yor.init()
        self.has_arms = True
        try:
            print("[YORTeleop] Homing arms...")
            self.yor.home_left_arm()
            self.yor.home_right_arm()
        except (ConnectionError, AttributeError, RuntimeError, Exception) as e:
            print(f"[YORTeleop] Warning: Could not home arms: {e}")
            self.has_arms = False

        self._zmq_ctx = zmq.Context()
        self._pub_sock = self._zmq_ctx.socket(zmq.PUB)
        self._pub_sock.bind(f"tcp://{self.pub_host}:{self.pub_port}")
        print(f"[YORTeleop] Episode publisher on tcp://{self.pub_host}:{self.pub_port}")

        # The ZED stream is optional: episodes record base_pose=None until it publishes.
        self.pose_stream = None
        if self.zed_pose or self.zed_image or episode_zed_pose:
            from robot.teleop.zed_reader import ZedSub
            self.pose_stream = ZedSub(host=self.zed_ip, port=ZED_PORT, pose_only=not self.zed_image)
            print(f"[YORTeleop] ZED pose from {self.zed_ip}:{ZED_PORT}")
        self.stop_event = threading.Event()

        self.latest_controller_state = None
        self.controller_state_lock = threading.Lock()
        self.thread = threading.Thread(target=self._oculus_worker, daemon=True)
        self.thread.start()

    def _oculus_worker(self):
        print(f"[YORTeleop] Connecting to Oculus at {self.vr_ip}:{VR_TCP_PORT}...")
        zmq_context = zmq.Context()
        stick_socket = zmq_context.socket(zmq.SUB)
        stick_socket.setsockopt(zmq.RCVTIMEO, VR_RECV_TIMEOUT_MS)
        stick_socket.connect("tcp://{}:{}".format(self.vr_ip, VR_TCP_PORT))
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

    def _publish(self, topic: str, data: dict):
        self._pub_sock.send_multipart([topic.encode(), json.dumps(data).encode()])

    def _lerp_se3(self, target: mink.SE3, last: mink.SE3, alpha: float) -> mink.SE3:
        blended = (1 - alpha) * target.wxyz_xyz + alpha * last.wxyz_xyz
        blended[:4] /= np.linalg.norm(blended[:4])
        return mink.SE3(blended)

    def _compute_ee_target(self, X_Cinit, X_ee_init, X_Ctarget):
        """Compute the EE target pose with independent position and rotation transforms."""
        X_Cdelta = X_Cinit.inverse().multiply(X_Ctarget)

        pos_delta = self.H.rotation().inverse() @ X_Cdelta.translation()
        target_pos = X_ee_init.translation() + pos_delta

        rot_delta = self.H_rot.inverse() @ X_Cdelta.rotation() @ self.H_rot
        target_rot = X_ee_init.rotation() @ rot_delta

        return mink.SE3(np.concatenate([target_rot.wxyz, target_pos]))

    # ------------------------------------------------------------------ arms

    def _toggle_arm(self, side: str, controller_SE3):
        engaged = getattr(self, f"start_teleop_{side}")
        label = side.capitalize()
        if engaged:
            setattr(self, f"start_teleop_{side}", False)
            print(f"[{label}] disengaged")
            return
        ee_pose = getattr(self.yor, f"get_{side}_ee_pose")()
        if ee_pose is None:
            print(f"[{label}] ERROR: could not get EE pose — arm may not be initialised (try homing)")
            return
        gripper = getattr(self.yor, f"get_{side}_gripper_pose")()
        self.last_gripper[side] = float(gripper) if gripper is not None else 1.0
        self.last_ee_target[side] = None
        self.X_ee_init[side] = ee_pose
        self.X_Cinit[side] = controller_SE3
        setattr(self, f"start_teleop_{side}", True)
        print(f"[{label}] engaged")

    def _home_arm(self, side: str):
        print(f"[{side.capitalize()}] Homing")
        getattr(self.yor, f"home_{side}_arm")()
        setattr(self, f"start_teleop_{side}", False)
        self.last_gripper[side] = 1.0  # home opens the gripper

    def _arm_target(self, side: str, controller_SE3, index_trigger: float):
        """Smoothed EE target and gripper command for an engaged arm."""
        x_Re = self._compute_ee_target(self.X_Cinit[side], self.X_ee_init[side], controller_SE3)
        if self.last_ee_target[side] is not None:
            x_Re = self._lerp_se3(x_Re, self.last_ee_target[side], self.ee_alpha)
        self.last_ee_target[side] = x_Re

        if self.no_gripper:
            return x_Re, None
        last = self.last_gripper[side]
        gripper = 1.0 - index_trigger  # trigger 0=open, 1=closed
        gripper = (1 - self.gripper_alpha) * gripper + self.gripper_alpha * last
        gripper = last + np.clip(gripper - last, -self.gripper_max_delta, self.gripper_max_delta)
        self.last_gripper[side] = float(gripper)
        return x_Re, float(gripper)

    # -------------------------------------------------------------- episodes

    def _start_episode(self):
        if not self.session_meta.get("task"):
            print("[Episode] NOT STARTED: restart vr_teleop with --task \"<instruction>\" (pi0.5 needs a prompt)")
            return
        now = time.time()
        self.episode_id = datetime.now().strftime("%Y-%m-%d--%H-%M-%S")
        try:
            self.recorder.start(self.episode_id, started_at=now)
        except FileExistsError:
            print(f"[Episode] NOT STARTED: {self.episode_id} already exists (pressed twice within a second?)")
            return
        self.collect_episode = True
        self.episode_seq = 0
        # The Orin camera recorder keys its files on episode_id and copies this metadata.
        self._publish(EPISODE_SIGNAL_TOPIC, {
            "timestamp": now,
            "active": True,
            "episode_id": self.episode_id,
            **self.session_meta,
        })
        print(f"[Episode] STARTED {self.episode_id}: {self.session_meta['task']!r}")

    def _end_episode(self, success: bool):
        self.collect_episode = False
        now = time.time()
        self._publish(EPISODE_SIGNAL_TOPIC, {
            "timestamp": now,
            "active": False,
            "success": success,
            "episode_id": self.episode_id,
            "num_frames": self.episode_seq,
        })
        self._publish(EPISODE_RESULT_TOPIC, {
            "timestamp": now,
            "result": "success" if success else "fail",
            "episode_id": self.episode_id,
        })
        self.recorder.end(success, ended_at=now)
        print(f"[Episode] ENDED {self.episode_id} — {'success' if success else 'fail'} ({self.episode_seq} frames)")

    def _episode_frame(self, state: dict, ee_targets: dict, gripper_cmds: dict, base_cmd: np.ndarray) -> dict:
        frame = {
            "timestamp": state["timestamp"],
            "episode_id": self.episode_id,
            "seq": self.episode_seq,
            "active": True,
            "base_vel": base_cmd.tolist(),
            "cmd_vel": base_cmd.tolist(),
            "enc_vel": state.get("enc_vel"),
        }
        for side in ARMS:
            measured = state.get(f"{side}_gripper")
            cmd_sent = gripper_cmds[side]
            commanded = cmd_sent if cmd_sent is not None else state.get(f"{side}_gripper_cmd")
            frame[f"{side}_ee"] = state.get(f"{side}_ee")
            frame[f"{side}_joints"] = state.get(f"{side}_joints")
            frame[f"{side}_joints_cmd"] = state.get(f"{side}_joints_cmd")  # absolute-joint action
            # Measured and commanded stay separate: a dropped reading is None plus an error.
            frame[f"{side}_gripper"] = measured
            frame[f"{side}_gripper_cmd"] = commanded
            frame[f"{side}_gripper_error"] = _gripper_error(state, side)
            frame[f"{side}_ee_target"] = ee_targets[side] if ee_targets[side] is not None else state.get(f"{side}_ee")
            frame[f"{side}_ik_ok"] = state.get(f"{side}_ik_ok")

        # ZED base pose (global, from the ZED's visual-inertial tracking). None until
        # the ZED publisher is up. zed_age_s compares Pi and Orin clocks, so it is
        # only meaningful when both are NTP-synced.
        base_pose = cam_pose = zed_ts = None
        if self.pose_stream is not None:
            base_pose, cam_pose, zed_ts = self.pose_stream.get_quat_pose()
        frame["base_pose"] = base_pose
        frame["cam_pose"] = cam_pose
        frame["zed_timestamp"] = zed_ts
        frame["zed_age_s"] = None if zed_ts is None else round(state["timestamp"] - zed_ts, 4)
        return frame

    # ------------------------------------------------------------- recording

    def _start_recording(self):
        timestamp_str = datetime.now().strftime("%Y-%m-%d--%H-%M-%S")
        os.makedirs('teleop_data', exist_ok=True)
        if self.zed_pose or self.zed_image:
            if not self.pose_stream.ready():
                print("[Record] NOT STARTED: ZED stream not ready")
                return
        self.csv_filename = f'teleop_data/yor_teleop_log_{timestamp_str}.csv'
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(_csv_header(self.zed_pose))
        self.csv_rows = 0
        self.base_cmd_history = []
        if self.zed_image:
            image_dir = f'teleop_data/images_{timestamp_str}'
            os.makedirs(image_dir, exist_ok=True)
            self.video_path = os.path.join(image_dir, "images.mp4")
            self.video_writer = None  # opened on the first frame, once the size is known
        self.record_data = True
        print(f"[Record] STARTED: {self.csv_filename}")

    def _record_row(self, state: dict, ee_targets: dict, gripper_cmds: dict, base_cmd: np.ndarray):
        row = [state["timestamp"]]
        for side in ARMS:
            measured = state.get(f"{side}_gripper")
            row += state.get(f"{side}_ee") or [float("nan")] * 7
            row += state.get(f"{side}_joints") or [float("nan")] * 7
            row += [float("nan") if measured is None else measured, _gripper_error(state, side) or ""]
        for side in ARMS:
            commanded = gripper_cmds[side] if gripper_cmds[side] is not None else state.get(f"{side}_gripper_cmd")
            row += ee_targets[side] if ee_targets[side] is not None else (state.get(f"{side}_ee") or [float("nan")] * 7)
            ik_ok = state.get(f"{side}_ik_ok")
            row += [commanded, "" if ik_ok is None else int(ik_ok)]
        row += base_cmd.tolist()
        row += state.get("enc_vel") or [float("nan")] * 3

        if self.zed_pose:
            base_wxyz_xyz, cam_wxyz_xyz, ts = self.pose_stream.get_quat_pose()
            row += [ts] + list(base_wxyz_xyz or [float("nan")] * 7) + list(cam_wxyz_xyz or [float("nan")] * 7)
        if self.zed_image:
            image, _ = self.pose_stream.get_image()
            self._write_video_frame(image)

        self.csv_writer.writerow(row)
        self.csv_rows += 1
        if self.csv_rows % CSV_FLUSH_EVERY == 0:
            self.csv_file.flush()
        self.base_cmd_history.append(base_cmd.copy())

    def _write_video_frame(self, image):
        import cv2
        if self.video_writer is None:
            h, w = image.shape[:2]
            self.video_writer = cv2.VideoWriter(self.video_path, cv2.VideoWriter_fourcc(*'mp4v'), LOOP_RATE, (w, h))
        self.video_writer.write(image)

    def _close_recording(self):
        """Flush and close output files. Safe to call more than once."""
        self.record_data = False
        if self.csv_file is not None:
            self.csv_file.close()
            print(f"[Record] Saved {self.csv_rows} rows to {self.csv_filename}")
            self.csv_file = None
            self.csv_writer = None
        if self.video_writer is not None:
            self.video_writer.release()
            print(f"[Record] Video saved: {self.video_path}")
            self.video_writer = None

    def _stop_recording(self):
        self._close_recording()
        self.start_teleop_left = False
        self.start_teleop_right = False

        if self.reset_base_after_data_collection and self.base_cmd_history:
            # Open-loop: replay the base commands negated and in reverse. Drift is
            # expected, so treat this as a rough reset, not a return to the start pose.
            reset_rate = RateLimiter(LOOP_RATE, name="yor_advanced_teleop_reset")
            reset_start_time = time.time()
            for v in reversed(self.base_cmd_history):
                self.yor.set_base_velocity(-v)
                reset_rate.sleep()
            self.yor.set_base_velocity(np.zeros(3))
            print(f"[Reset] Base reset in {time.time() - reset_start_time:.2f}s")
        self.base_cmd_history = []

        if self.has_arms:
            print("[YORTeleop] Homing both arms after recording")
            self._home_arm("left")
            self._home_arm("right")

    # ------------------------------------------------------------ main loop

    def control_loop(self):
        rate = RateLimiter(LOOP_RATE, name="yor_advanced_teleop")

        print("[YORTeleop] Ready.")
        print(" Controls:")
        print("  - Both hand triggers: Toggle base control")
        print("  - Left Stick: Rotate Base   Right Stick: Move Base")
        print("  - Left X / Right A: Engage/disengage arm")
        print("  - Left Y / Right B: Home arm")
        print("  - Index triggers: Close Grippers")
        print("  - Right Thumbstick click: Start/End episode (success)")
        print("  - Left Thumbstick click: End episode (fail)")
        print("  - Left Menu: Toggle CSV recording")

        prev_state = None
        waiting_logged = False

        while not self.stop_event.is_set():
            with self.controller_state_lock:
                controller_state = self.latest_controller_state
            if controller_state is None:
                if not waiting_logged:
                    print("Waiting for VR controller...")
                    waiting_logged = True
                rate.sleep()
                continue

            # Rising-edge helper: fires only on False→True transition
            def rose(attr):
                return getattr(controller_state, attr) and (prev_state is None or not getattr(prev_state, attr))

            # --- Arm teleop state machine ---
            if self.has_arms:
                if rose('left_x'):
                    self._toggle_arm("left", controller_state.left_SE3)
                if rose('left_y'):
                    self._home_arm("left")
                if rose('right_a'):
                    self._toggle_arm("right", controller_state.right_SE3)
                if rose('right_b'):
                    self._home_arm("right")

            if rose('right_thumbstick'):
                if not self.collect_episode:
                    self._start_episode()
                else:
                    self._end_episode(success=True)

            if rose('left_thumbstick') and self.collect_episode:
                self._end_episode(success=False)

            # --- Compute and send arm targets ---
            preview_time = 2 / 15
            ee_targets = {"left": None, "right": None}
            gripper_cmds = {"left": None, "right": None}
            x_Re = {}
            for side, SE3, trigger in (("left", controller_state.left_SE3, controller_state.left_index_trigger),
                                       ("right", controller_state.right_SE3, controller_state.right_index_trigger)):
                if getattr(self, f"start_teleop_{side}"):
                    x_Re[side], gripper_cmds[side] = self._arm_target(side, SE3, trigger)
                    ee_targets[side] = x_Re[side].wxyz_xyz.tolist()

            if self.start_teleop_left and self.start_teleop_right:
                self.yor.set_bimanual_ee_target(
                    L_ee_target=x_Re["left"], L_gripper_target=gripper_cmds["left"], L_preview_time=preview_time,
                    R_ee_target=x_Re["right"], R_gripper_target=gripper_cmds["right"], R_preview_time=preview_time,
                )
            elif self.start_teleop_left:
                self.yor.set_left_ee_target(ee_target=x_Re["left"], gripper_target=gripper_cmds["left"], preview_time=preview_time)
            elif self.start_teleop_right:
                self.yor.set_right_ee_target(ee_target=x_Re["right"], gripper_target=gripper_cmds["right"], preview_time=preview_time)

            # --- Base control ---
            both_triggers = controller_state.left_hand_trigger > 0.5 and controller_state.right_hand_trigger > 0.5
            prev_both_triggers = prev_state is not None and prev_state.left_hand_trigger > 0.5 and prev_state.right_hand_trigger > 0.5
            if both_triggers and not prev_both_triggers:
                self.start_base_control = not self.start_base_control
                print(f"[Base] Control {'ENABLED' if self.start_base_control else 'DISABLED'}")
                if not self.start_base_control:
                    self.yor.set_base_velocity(np.zeros(3))
                    self.last_target_velocity = np.zeros(3)

            target_velocity = np.zeros(3)
            if self.start_base_control:
                vy = -controller_state.right_thumbstick_axes[0]  # Right stick horizontal
                vx = controller_state.right_thumbstick_axes[1]   # Right stick vertical
                w = -controller_state.left_thumbstick_axes[0]    # Left stick horizontal
                target_velocity = apply_deadzone(np.array([vx, vy, w]))
                target_velocity = self.max_vel * target_velocity
                target_velocity = (1 - self.vel_alpha) * target_velocity + self.vel_alpha * self.last_target_velocity
                target_velocity[np.abs(target_velocity) < 1e-3] = 0.0  # let the EMA tail reach exactly zero
                self.last_target_velocity = target_velocity
                # Sent every tick (zeros included): the robot stops the base if commands stop arriving.
                self.yor.set_base_velocity(target_velocity)

            # --- Data recording / episode publishing ---
            if self.record_data or self.collect_episode:
                state = self.yor.get_collection_state()
                if state is not None:
                    if self.record_data:
                        self._record_row(state, ee_targets, gripper_cmds, target_velocity)
                    if self.collect_episode:
                        frame = self._episode_frame(state, ee_targets, gripper_cmds, target_velocity)
                        self._publish(EPISODE_TOPIC, frame)
                        self.recorder.write(frame)
                        self.episode_seq += 1

            # --- Toggle recording ---
            if rose('left_menu'):
                if not self.record_data:
                    self._start_recording()
                else:
                    self._stop_recording()

            prev_state = controller_state
            rate.sleep()

    def stop(self):
        self.stop_event.set()
        self._close_recording()
        if self.collect_episode:
            self.collect_episode = False
            self.recorder.end(success=False, reason="teleop exited mid-episode")
        try:
            self.yor.set_base_velocity(np.zeros(3))
        except Exception:
            pass
        self.thread.join(timeout=2.0)
        self._pub_sock.close(linger=0)
        self._zmq_ctx.term()


def main(zed_pose, zed_image, reset_base, no_gripper, act,
         pub_host=None, pub_port=None, vr_ip=None, zed_ip=None, speed="teleop",
         session_meta=None, episode_dir=EPISODE_DATA_DIR, episode_zed_pose=True):
    teleop = YORAdvancedTeleop(
        reset_base_after_data_collection=reset_base,
        zed_pose=zed_pose,
        zed_image=zed_image,
        no_gripper=no_gripper,
        act=act,
        pub_host=pub_host,
        pub_port=pub_port,
        vr_ip=vr_ip,
        zed_ip=zed_ip,
        speed=speed,
        session_meta=session_meta,
        episode_dir=episode_dir,
        episode_zed_pose=episode_zed_pose,
    )
    atexit.register(teleop.stop)
    try:
        teleop.control_loop()
    except KeyboardInterrupt:
        pass


def session_meta_from_args(args) -> dict:
    """Session metadata stamped on every episode: --meta-file first, then CLI flags on top."""
    meta = {}
    if args.meta_file:
        with open(args.meta_file) as f:
            meta.update(json.load(f))
    for key in ("task", "operator", "object", "scene", "start_location", "goal_location", "notes"):
        value = getattr(args, key)
        if value is not None:
            meta[key] = value
    return meta


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--reset_base', '-r', action='store_true', help='Reset base after data collection')
    parser.add_argument('--zed_pose', '-b', action='store_true', help='Also record ZED base pose in the CSV log')
    parser.add_argument('--zed_image', '-i', action='store_true', help='Record ZED image and depth data')
    parser.add_argument('--no-gripper', action='store_true', help='Disable gripper control')
    parser.add_argument('--act', action='store_true', help='No-op, kept for compatibility: cmd_vel and enc_vel are always published')
    parser.add_argument('--speed', choices=sorted(SPEED_PRESETS), default='teleop',
                        help='Base speed preset for the whole run (default: teleop)')
    # ── Episode metadata (stamped on every episode in this session) ───────────
    meta = parser.add_argument_group('episode metadata')
    meta.add_argument('--task', help='Language instruction for the policy, e.g. "pick up the red cup and place it on the table" (required to start episodes)')
    meta.add_argument('--operator', help='Who is teleoperating')
    meta.add_argument('--object', help='Object being manipulated')
    meta.add_argument('--scene', help='Room / environment name')
    meta.add_argument('--start-location', dest='start_location', help='Where the robot starts')
    meta.add_argument('--goal-location', dest='goal_location', help='Where the object is placed')
    meta.add_argument('--notes', help='Free-form notes')
    meta.add_argument('--meta-file', help='JSON file of metadata fields; CLI flags override it')
    parser.add_argument('--episode-dir', default=EPISODE_DATA_DIR, help=f'Where Pi-side episodes are saved (default: {EPISODE_DATA_DIR})')
    parser.add_argument('--no-episode-zed-pose', action='store_true', help='Do not subscribe to the ZED pose for episode frames')
    # ── Network endpoints (all default to robot/config.py / env vars) ─────────
    parser.add_argument('--pub-host', default=EPISODE_PUB_HOST,
                        help=f'Interface the episode publisher binds to '
                             f'(default: {EPISODE_PUB_HOST} = all; env EPISODE_PUB_HOST)')
    parser.add_argument('--pub-port', type=int, default=EPISODE_PUB_PORT,
                        help=f'Port the episode publisher binds to '
                             f'(default: {EPISODE_PUB_PORT}; env EPISODE_PUB_PORT)')
    parser.add_argument('--vr-ip', default=VR_TCP_HOST,
                        help=f'Oculus controller-stream address '
                             f'(default: {VR_TCP_HOST}; env VR_IP)')
    parser.add_argument('--zed-ip', default=ZED_POSE_HOST,
                        help=f'ZED pose-stream address '
                             f'(default: {ZED_POSE_HOST}; env ZED_IP)')
    args = parser.parse_args()
    main(args.zed_pose, args.zed_image, args.reset_base, args.no_gripper, args.act,
         pub_host=args.pub_host, pub_port=args.pub_port,
         vr_ip=args.vr_ip, zed_ip=args.zed_ip, speed=args.speed,
         session_meta=session_meta_from_args(args), episode_dir=args.episode_dir,
         episode_zed_pose=not args.no_episode_zed_pose)
