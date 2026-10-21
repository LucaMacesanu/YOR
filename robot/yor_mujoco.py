"""
MuJoCo stand-in for robot/yor.py: serves the same RPC methods on the same port
(5557), so teleop clients (telestick, joystick, vr_teleop, yor_teleop, ...) run
against the simulation unchanged.

    python robot/yor_mujoco.py [--headless] [--port 5557]

Physics model: yor-description/scene-full.mjcf (swerve base, lift, Nero arms,
grippers, pan/tilt head with ZED cameras). Where a client can observe it, the
sim behaves like the real robot:
  - arms: the same IK model and solver as ArmNode (nero-welded-base-and-lift.mjcf,
    SingleArmIK), the same home pose, and joint targets reached over
    preview_time at nerolib's joint speed limit. EE poses are in that IK model's
    world frame (robot frame, lift-independent), exactly as ArmNode reports them.
  - grippers: 0 = closed, 1 = open (CanGripper convention); enable/disable like the
    SameHand (a disabled gripper ignores targets).
  - neck: yor.py's set_neck_position / get_neck_position / get_neck_limits /
    home_neck, in the robot's convention (degrees, + pan = head right, + tilt =
    head down, 0 = straight ahead and level).
  - base: [vx, vy, omega] in the robot frame, held until the next command (the
    real BaseController re-sends its last target), acceleration-limited like
    base_motor.py, driven through the 4 swerve modules of the model.
  - lift: up / down / stop at a fixed speed, like the Pico lift.
The calls vr-teleop's yor_bridge.py uses are listed in
vr-teleop/docs/YOR_RPC_INTERFACE.md. Sim-only: get_sim_pose, and get_lift_height
returns the real height.

Robot frame = the IK model's world frame: x forward, y left, z up, with its
origin 0.21 m to the right of the chassis centre (where the CAD put it).
"""
import argparse
import atexit
import functools
import math
import sys
import threading
import time
from pathlib import Path

import mink
import mujoco
import mujoco.viewer
import numpy as np
from loop_rate_limiters import RateLimiter

_HERE = Path(__file__).parent
_ROOT = _HERE.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from commlink import RPCServer  # noqa: E402

from robot.arm.ik_solver import SingleArmIK  # noqa: E402

YOR_PORT = 5557
SIM_MODEL = _HERE / "yor-description" / "scene-full.mjcf"
IK_MODEL = _HERE / "yor-description" / "nero-welded-base-and-lift.mjcf"  # same as yor.py

CONTROL_HZ = 100  # command updates per second; physics substeps fill the gap
VIEWER_HZ = 50

SIDES = ("left", "right")
# ArmNode.home_position (robot/arm/arm.py) -- keep in sync
HOME_Q = {
    "left": np.array([1.8317, 1.3773, -1.7357, 1.5211, -0.1130, -0.0954, 0.0973]),
    "right": np.array([-1.8391, 1.3285, 1.7432, 1.5277, -0.0006, 0.1011, 0.1097]),
}
JOINT_VEL_MAX = 3.0  # rad/s, ArmNode's config.joint_vel_max
HOME_DURATION_S = 2.0  # ArmNode.home() waits 2 s after reset_to_home()
IK_FAIL_HOLD_THRESHOLD = 10  # ArmNode: after this many failed solves in a row, hold instead of sending

# Gripper finger (joint1) angle at "closed". +q closes the finger onto the fixed
# jaw on both sides (output/grippers/build_gripper.py); 0 is open. Placeholder:
# the real servo travel is unknown, this is 95% of the collision-free range.
GRIPPER_CLOSED_FRACTION = 0.95
GRIPPER_SPEED = 2.0  # full open<->close strokes per second, placeholder

# base_motor.py: per-axis acceleration limit of the S-curve profile, and the
# raw drive-velocity unit (raw / DRIVE_VEL_SCALE = m/s)
BASE_ACCEL_MAX = np.array([1.9, 1.9, 6.5])
DRIVE_VEL_SCALE = 2.0
# Swerve modules in base_motor.py order [FL, FR, RR, RL]: (steer joint, drive joint)
MODULES = (
    ("front_left_steer", "drive_front_left"),
    ("front_right_steer", "drive_front_right"),
    ("back_right_steer", "drive_back_right"),
    ("back_left_steer", "drive_back_left"),
)

LIFT_SPEED_MPS = 0.05  # Pico lift speed is unknown -- placeholder

# Simulated ZED stereo for the VR headset: rendered here, published locally, and
# streamed by vr-teleop's video server through its "zmq" camera source.
VIDEO_PUB_ADDR = "tcp://127.0.0.1:5560"
ZED_CAMERAS = ("head_zed_left", "head_zed_right")


def require_initialization(func):
    @functools.wraps(func)
    def wrapper(self, *args, **kwargs):
        if not self._initialized:
            print(f"Warning: {func.__name__} called before YOR was initialized")
            return None
        return func(self, *args, **kwargs)

    return wrapper


def _wrap(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def _pose_matrix(pos, mat):
    T = np.eye(4)
    T[:3, :3] = np.asarray(mat).reshape(3, 3)
    T[:3, 3] = pos
    return T


class _SimArm:
    """One Nero arm + gripper: ArmNode's IK, nerolib-like joint interpolation."""

    def __init__(self, side, model, ik_model_path):
        self.side = side
        joint_names = [f"{side}_arm_joint{i}" for i in range(1, 8)]
        self.ik = SingleArmIK(ik_model_path, joint_names=joint_names, ee_frame=f"{side}_arm_ee")
        self.ik_lock = threading.Lock()
        self.qpos_adr = np.array([model.jnt_qposadr[model.joint(n).id] for n in joint_names])
        self.act_ids = np.array([model.actuator(n + "_pos").id for n in joint_names])

        self.finger_adr = model.jnt_qposadr[model.joint(f"{side}_gripper_joint1").id]
        self.finger_act = model.actuator(f"{side}_gripper_joint1_pos").id
        self.knuckle_act = model.actuator(f"{side}_gripper_joint2_pos").id
        self.finger_closed = GRIPPER_CLOSED_FRACTION * model.jnt_range[model.joint(f"{side}_gripper_joint1").id][1]

        self.q_cmd = HOME_Q[side].copy()  # interpolated command sent to the actuators
        self.q_goal = HOME_Q[side].copy()
        self.q_vel = np.zeros(7)
        self.gripper_cmd = 1.0
        self.gripper_goal = 1.0
        self.gripper_enabled = True
        self.last_joint_target = None
        self.last_ik_solved = None
        self.ik_fail_streak = 0

    def set_goal(self, q, duration):
        q = np.asarray(q, dtype=float)
        duration = max(float(duration), float(np.max(np.abs(q - self.q_cmd))) / JOINT_VEL_MAX, 1e-3)
        self.q_goal = q
        self.q_vel = np.abs(q - self.q_cmd) / duration

    def tick(self, data, dt):
        step = np.clip(self.q_goal - self.q_cmd, -self.q_vel * dt, self.q_vel * dt)
        self.q_cmd = self.q_cmd + step
        data.ctrl[self.act_ids] = self.q_cmd

        g_step = GRIPPER_SPEED * dt
        self.gripper_cmd += float(np.clip(self.gripper_goal - self.gripper_cmd, -g_step, g_step))
        data.ctrl[self.finger_act] = (1.0 - self.gripper_cmd) * self.finger_closed
        data.ctrl[self.knuckle_act] = 0.0

    def gripper_position(self, data):
        return float(np.clip(1.0 - data.qpos[self.finger_adr] / self.finger_closed, 0.0, 1.0))


class YORMujoco:
    def __init__(self, model_path=SIM_MODEL, ik_model_path=IK_MODEL, headless=False):
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        # Room for the ZED renders. Set before the viewer starts: viewer.sync() copies
        # the viewer's visual settings back into the model, so a later change is lost.
        self.model.vis.global_.offwidth = max(self.model.vis.global_.offwidth, 1920)
        self.model.vis.global_.offheight = max(self.model.vis.global_.offheight, 1080)
        self.data = mujoco.MjData(self.model)
        self._lock = threading.RLock()  # guards self.data and every command below
        self._initialized = False
        self.no_arms = False

        m = self.model
        mujoco.mj_resetDataKeyframe(m, self.data, m.key("home").id)
        self.arms = {side: _SimArm(side, m, str(ik_model_path)) for side in SIDES}
        for arm in self.arms.values():
            self.data.qpos[arm.qpos_adr] = HOME_Q[arm.side]
            self.data.ctrl[arm.act_ids] = HOME_Q[arm.side]
        mujoco.mj_forward(m, self.data)

        # Robot frame = base body pose at home, so T_world_robot = T_world_base @ inv(T_base_home).
        self._base_id = m.body("base").id
        T_base_home = _pose_matrix(self.data.xpos[self._base_id], self.data.xmat[self._base_id])
        self._T_base_robot = np.linalg.inv(T_base_home)

        # Swerve module positions relative to the chassis centre, robot frame.
        self._steer_adr = np.array([m.jnt_qposadr[m.joint(s).id] for s, _ in MODULES])
        self._drive_dof = np.array([m.jnt_dofadr[m.joint(dr).id] for _, dr in MODULES])
        self._steer_act = np.array([m.actuator(s + "_pos").id for s, _ in MODULES])
        self._drive_act = np.array([m.actuator(dr + "_vel").id for _, dr in MODULES])
        self._steer_range = np.array([m.jnt_range[m.joint(s).id] for s, _ in MODULES])
        anchors = np.array([self.data.xanchor[m.joint(s).id][:2] for s, _ in MODULES])
        self._chassis_center = anchors.mean(axis=0)
        self._module_xy = anchors - self._chassis_center
        wheel_geom = m.geom("MK4_Colson_Hub_4_X_1_5_collision").id
        self._wheel_radius = float(m.geom_aabb[wheel_geom][4])  # half-extent across the tyre
        self.base_target = np.zeros(3)  # [vx, vy, omega], robot frame
        self.base_vel = np.zeros(3)  # after the acceleration limit

        self._lift_adr = m.jnt_qposadr[m.joint("lift_joint").id]
        self._lift_act = m.actuator("Lift").id
        self._lift_range = m.jnt_range[m.joint("lift_joint").id].copy()
        self.lift_setpoint = float(self.data.qpos[self._lift_adr])
        self.lift_dir = 0  # +1 up, -1 down, 0 hold

        self._pan_adr = m.jnt_qposadr[m.joint("head_pan_joint").id]
        self._tilt_adr = m.jnt_qposadr[m.joint("head_tilt_joint").id]
        self._pan_act = m.actuator("head_pan_pos").id
        self._tilt_act = m.actuator("head_tilt_pos").id
        self._neck_cmd = None  # last commanded [pan, tilt], robot convention

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._physics_loop, daemon=True)
        self.viewer = None
        if not headless:
            self.viewer = mujoco.viewer.launch_passive(m, self.data, show_left_ui=False, show_right_ui=False)

    # ------------------------------------------------------------- lifecycle
    def init(self):
        if self._initialized:
            print("Warning: YOR already initialized")
            return
        for arm in self.arms.values():
            arm.ik.init(HOME_Q[arm.side])
        self._thread.start()
        self._initialized = True

    def stop(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self.viewer is not None:
            self.viewer.close()

    def is_running(self):
        return not self._stop.is_set() and (self.viewer is None or self.viewer.is_running())

    def _physics_loop(self):
        timestep = self.model.opt.timestep
        max_substeps = max(1, round(0.1 / timestep))  # if we fall >100 ms behind, slow down instead
        rate = RateLimiter(CONTROL_HZ, name="yor_mujoco", warn=False)
        wall0, sim0 = time.monotonic(), self.data.time
        last_view = 0.0
        while not self._stop.is_set():
            # Step as much sim time as wall time has passed: sleep() on Windows only
            # wakes every ~15 ms, so a fixed step per tick would run slower than real time.
            with self._lock:
                behind = (time.monotonic() - wall0) - (self.data.time - sim0)
                substeps = int(np.clip(behind / timestep, 1, max_substeps))
                dt = substeps * timestep
                for arm in self.arms.values():
                    arm.tick(self.data, dt)
                self._tick_base(dt)
                self._tick_lift(dt)
                mujoco.mj_step(self.model, self.data, nstep=substeps)
                if substeps == max_substeps:
                    wall0, sim0 = time.monotonic(), self.data.time
                now = time.monotonic()
                if self.viewer is not None and now - last_view >= 1.0 / VIEWER_HZ:
                    self.viewer.sync()
                    last_view = now
            rate.sleep()

    # ------------------------------------------------------------------ base
    def _tick_base(self, dt):
        step = BASE_ACCEL_MAX * dt
        self.base_vel = self.base_vel + np.clip(self.base_target - self.base_vel, -step, step)
        vx, vy, w = self.base_vel
        steer = _wrap(self.data.qpos[self._steer_adr])
        if np.allclose(self.base_vel, 0.0, atol=1e-4):
            self.data.ctrl[self._drive_act] = 0.0  # hold the wheels where they point
            return
        # Body velocity at each module: v + omega x r (base_motor.py's swerve IK).
        wx = vx - w * self._module_xy[:, 1]
        wy = vy + w * self._module_xy[:, 0]
        speed = np.hypot(wx, wy)
        angle = np.arctan2(wy, wx)
        err = _wrap(angle - steer)
        flip = np.abs(err) > np.pi / 2  # turn the short way and drive backwards instead
        angle = np.where(flip, _wrap(angle + np.pi), angle)
        speed = np.where(flip, -speed, speed)
        speed *= np.cos(_wrap(angle - steer))  # don't push while still steering
        target = self.data.qpos[self._steer_adr] + _wrap(angle - steer)
        self.data.ctrl[self._steer_act] = np.clip(target, self._steer_range[:, 0], self._steer_range[:, 1])
        self.data.ctrl[self._drive_act] = speed / self._wheel_radius

    def _T_world_robot(self):
        T_world_base = _pose_matrix(self.data.xpos[self._base_id], self.data.xmat[self._base_id])
        return T_world_base @ self._T_base_robot

    # ------------------------------------------------------------------ lift
    def _tick_lift(self, dt):
        if self.lift_dir:
            self.lift_setpoint = float(
                np.clip(self.lift_setpoint + self.lift_dir * LIFT_SPEED_MPS * dt, *self._lift_range)
            )
        self.data.ctrl[self._lift_act] = self.lift_setpoint

    # =================================================== RPC: base (yor.py)
    @require_initialization
    def set_base_velocity(self, velocity: np.ndarray):
        with self._lock:
            self.base_target = np.asarray(velocity, dtype=float).reshape(3)

    @require_initialization
    def get_cmd_vel(self):
        with self._lock:
            return self.base_target.tolist(), time.time()

    @require_initialization
    def get_base_encoders(self) -> dict:
        with self._lock:
            steer = self.data.qpos[self._steer_adr]
            drive = self.data.qvel[self._drive_dof] * self._wheel_radius * DRIVE_VEL_SCALE
            return {
                "timestamp": time.time(),
                "steer_rad": steer.tolist(),
                "steer_deg": np.degrees(steer).tolist(),
                "steer_counts": [0.0] * 4,
                "drive_vel": drive.tolist(),
                "drive_counts": [0.0] * 4,
                "lift_height_m": float(self.data.qpos[self._lift_adr]),
            }

    @require_initialization
    def get_pose(self) -> dict:
        """Ground-truth pose of the chassis centre in the sim world (the real robot gets
        this from the ZED). The base turns about this point."""
        with self._lock:
            T = self._T_world_robot()
        x, y = T[:2, 3] + T[:2, :2] @ self._chassis_center
        return {"x": float(x), "y": float(y), "theta": float(math.atan2(T[1, 0], T[0, 0]))}

    def _not_simulated(self, name):
        print(f"[YORMujoco] {name} is not simulated")
        return None

    def follow_path(self, path=None):
        return self._not_simulated("follow_path")

    def move_to(self, goal=None):
        return self._not_simulated("move_to")

    def move_by(self, deltas=None):
        return self._not_simulated("move_by")

    def get_nav_debug(self):
        return self._not_simulated("get_nav_debug")

    # =================================================== RPC: lift (yor.py)
    @require_initialization
    def lift_up(self) -> None:
        with self._lock:
            self.lift_dir = 1

    @require_initialization
    def lift_down(self) -> None:
        with self._lock:
            self.lift_dir = -1

    @require_initialization
    def lift_stop(self) -> None:
        with self._lock:
            self.lift_dir = 0
            self.lift_setpoint = float(self.data.qpos[self._lift_adr])

    @require_initialization
    def lift_home(self) -> None:
        self.lift_to_height(0.0)

    @require_initialization
    def get_lift_height(self) -> float:
        with self._lock:
            return float(self.data.qpos[self._lift_adr])

    @require_initialization
    def lift_to_height(self, target_m, tolerance_m=0.002, timeout_s=30.0, min_height_m=0.0, max_height_m=0.5) -> bool:
        target_m = float(np.clip(target_m, max(min_height_m, self._lift_range[0]), min(max_height_m, self._lift_range[1])))
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            h = self.get_lift_height()
            if abs(target_m - h) <= tolerance_m:
                self.lift_stop()
                return True
            with self._lock:
                self.lift_dir = 1 if target_m > h else -1
            time.sleep(0.01)
        self.lift_stop()
        return False

    @require_initialization
    def lift_delta_height(self, delta_m, tolerance_m=0.002, timeout_s=30.0, min_height_m=0.0, max_height_m=0.5) -> bool:
        return self.lift_to_height(self.get_lift_height() + float(delta_m), tolerance_m, timeout_s, min_height_m,
                                   max_height_m)

    # =================================================== RPC: arms (yor.py / ArmNode)
    def _set_joint_target(self, side, joint_target, gripper_target, preview_time):
        arm = self.arms[side]
        with self._lock:
            arm.set_goal(joint_target, preview_time)
            arm.last_joint_target = np.array(joint_target, dtype=float)
            if gripper_target is not None and arm.gripper_enabled:
                arm.gripper_goal = float(np.clip(gripper_target, 0.0, 1.0))

    def _set_ee_target(self, side, ee_target: mink.SE3, gripper_target, preview_time):
        # ArmNode.set_ee_target: re-sync IK to the measured joints, solve, and after
        # IK_FAIL_HOLD_THRESHOLD failures in a row hold the last converged command.
        arm = self.arms[side]
        q = self._measured_q(side)
        with arm.ik_lock:
            arm.ik.update_configuration(q)
            qd, solved = arm.ik.solve_ik(ee_target)
        arm.last_ik_solved = bool(solved)
        arm.ik_fail_streak = 0 if solved else arm.ik_fail_streak + 1
        if arm.ik_fail_streak >= IK_FAIL_HOLD_THRESHOLD:
            return
        self._set_joint_target(side, qd, gripper_target, preview_time)

    def _measured_q(self, side):
        with self._lock:
            return self.data.qpos[self.arms[side].qpos_adr].copy()

    def _get_ee_pose(self, side) -> mink.SE3:
        # ArmNode.get_ee_pose: load the measured joints into the IK model, then FK.
        q = self._measured_q(side)
        arm = self.arms[side]
        with arm.ik_lock:
            arm.ik.update_configuration(q)
            return arm.ik.forward_kinematics()

    def _home(self, side, gripper_target):
        # ArmNode.home: reset_to_home, gripper to gripper_target, wait 2 s, re-init IK
        self._set_joint_target(side, HOME_Q[side], gripper_target, HOME_DURATION_S)
        time.sleep(HOME_DURATION_S)
        arm = self.arms[side]
        q = self._measured_q(side)
        with arm.ik_lock:
            arm.ik.init(q)

    def _gripper_pose(self, side):
        with self._lock:
            return self.arms[side].gripper_position(self.data)

    def _set_gripper(self, side, value):
        with self._lock:
            if self.arms[side].gripper_enabled:
                self.arms[side].gripper_goal = float(np.clip(value, 0.0, 1.0))

    def _enable_gripper(self, side, on):
        with self._lock:
            self.arms[side].gripper_enabled = bool(on)
        return True

    @require_initialization
    def set_left_joint_target(self, joint_target: np.ndarray, gripper_target: float | None = None,
                              preview_time: float = 1.0):
        self._set_joint_target("left", joint_target, gripper_target, preview_time)

    @require_initialization
    def set_left_ee_target(self, ee_target: mink.SE3, gripper_target: float | None = None, preview_time: float = 0.3):
        self._set_ee_target("left", ee_target, gripper_target, preview_time)

    @require_initialization
    def set_left_gripper(self, gripper_pos: float):
        self._set_gripper("left", gripper_pos)

    @require_initialization
    def set_left_gain(self, kp: np.ndarray, kd: np.ndarray):
        pass  # the sim's position actuators have fixed gains

    @require_initialization
    def home_left_arm(self, gripper_target: float = 1.0):
        self._home("left", gripper_target)

    @require_initialization
    def enable_left_gripper(self) -> bool:
        return self._enable_gripper("left", True)

    @require_initialization
    def disable_left_gripper(self):
        self._enable_gripper("left", False)

    @require_initialization
    def tuck_left_arm(self):
        self._set_joint_target("left", np.zeros(7), 1.0, 2.0)

    @require_initialization
    def open_left_gripper(self):
        self._set_gripper("left", 1.0)

    @require_initialization
    def close_left_gripper(self):
        self._set_gripper("left", 0.0)

    @require_initialization
    def get_left_ee_pose(self) -> mink.SE3:
        return self._get_ee_pose("left")

    @require_initialization
    def get_left_joint_positions(self) -> np.ndarray:
        return self._measured_q("left")

    @require_initialization
    def get_left_gripper_pose(self):
        return self._gripper_pose("left")

    @require_initialization
    def set_right_joint_target(self, joint_target: np.ndarray, gripper_target: float | None = None,
                               preview_time: float = 1.0):
        self._set_joint_target("right", joint_target, gripper_target, preview_time)

    @require_initialization
    def set_right_ee_target(self, ee_target: mink.SE3, gripper_target: float | None = None, preview_time: float = 0.3):
        self._set_ee_target("right", ee_target, gripper_target, preview_time)

    @require_initialization
    def set_right_gripper(self, gripper_pos: float):
        self._set_gripper("right", gripper_pos)

    @require_initialization
    def set_right_gain(self, kp: np.ndarray, kd: np.ndarray):
        pass

    @require_initialization
    def home_right_arm(self, gripper_target: float = 1.0):
        self._home("right", gripper_target)

    @require_initialization
    def enable_right_gripper(self) -> bool:
        return self._enable_gripper("right", True)

    @require_initialization
    def disable_right_gripper(self):
        self._enable_gripper("right", False)

    @require_initialization
    def tuck_right_arm(self):
        self._set_joint_target("right", np.zeros(7), 1.0, 2.0)

    @require_initialization
    def open_right_gripper(self):
        self._set_gripper("right", 1.0)

    @require_initialization
    def close_right_gripper(self):
        self._set_gripper("right", 0.0)

    @require_initialization
    def get_right_ee_pose(self) -> mink.SE3:
        return self._get_ee_pose("right")

    @require_initialization
    def get_right_joint_positions(self) -> np.ndarray:
        return self._measured_q("right")

    @require_initialization
    def get_right_gripper_pose(self):
        return self._gripper_pose("right")

    @require_initialization
    def get_arm_relative_pose(self) -> tuple[mink.SE3, mink.SE3]:
        left_ee_pose = self.get_left_ee_pose()
        right_ee_pose = self.get_right_ee_pose()
        l2r = right_ee_pose.inverse() @ left_ee_pose
        r2l = left_ee_pose.inverse() @ right_ee_pose
        return r2l, l2r

    @require_initialization
    def get_bimanual_state(self) -> list:
        """Same row layout as yor.py: [t, L_ee(7), L_q(7), L_grip, R_ee(7), R_q(7), R_grip, lift].
        Unlike the real robot (no lift feedback, 0.0), the lift entry is the sim height."""
        row = [0.0] * (1 + 7 + 7 + 1 + 7 + 7 + 1 + 1)
        row[0] = time.time()
        row[1:8] = self.get_left_ee_pose().wxyz_xyz.tolist()
        row[8:15] = self.get_left_joint_positions().tolist()
        row[15] = self.get_left_gripper_pose()
        row[16:23] = self.get_right_ee_pose().wxyz_xyz.tolist()
        row[23:30] = self.get_right_joint_positions().tolist()
        row[30] = self.get_right_gripper_pose()
        row[31] = self.get_lift_height()
        return row

    @require_initialization
    def set_bimanual_ee_target(self, L_ee_target: mink.SE3, R_ee_target: mink.SE3,
                               L_gripper_target: float | None = None, L_preview_time: float = 0.1,
                               R_gripper_target: float | None = None, R_preview_time: float = 0.1):
        self._set_ee_target("left", L_ee_target, L_gripper_target, L_preview_time)
        self._set_ee_target("right", R_ee_target, R_gripper_target, R_preview_time)

    @require_initialization
    def get_ik_status(self):
        return tuple(self.arms[side].last_ik_solved for side in SIDES)

    @require_initialization
    def get_collection_state(self) -> dict:
        """yor.py's one-call snapshot (vr-teleop's HUD reads it)."""
        with self._lock:
            state = {"timestamp": time.time(), "base_cmd_vel": self.base_target.tolist(),
                     "enc_vel": self.base_vel.tolist(), "neck_deg": self._neck_deg(),
                     "neck_cmd_deg": None if self._neck_cmd is None else list(self._neck_cmd)}
        for side in SIDES:
            arm = self.arms[side]
            state[f"{side}_ee"] = self._get_ee_pose(side).wxyz_xyz.tolist()
            state[f"{side}_joints"] = self._measured_q(side).tolist()
            state[f"{side}_joints_cmd"] = None if arm.last_joint_target is None else arm.last_joint_target.tolist()
            state[f"{side}_gripper"] = self._gripper_pose(side)
            state[f"{side}_gripper_cmd"] = arm.gripper_goal
            state[f"{side}_ik_ok"] = arm.last_ik_solved
        return state

    @require_initialization
    def set_lift_position(self, target_m) -> bool:
        height = float(target_m) if np.isscalar(target_m) else float(np.asarray(target_m).ravel()[0])
        return self.lift_to_height(height)

    # =================================================== RPC: neck (yor.py)
    # Robot convention: degrees, + pan = head right, + tilt = head down. In the model
    # + head_pan_joint turns the camera left and + head_tilt_joint turns it down.
    def _neck_deg(self):
        return [-math.degrees(self.data.qpos[self._pan_adr]), math.degrees(self.data.qpos[self._tilt_adr])]

    @require_initialization
    def set_neck_position(self, pan: float | None = None, tilt: float | None = None):
        lim = self.get_neck_limits()
        with self._lock:
            cmd = list(self._neck_cmd or self._neck_deg())
            if pan is not None:
                cmd[0] = float(np.clip(pan, *lim["pan"]))
            if tilt is not None:
                cmd[1] = float(np.clip(tilt, *lim["tilt"]))
            self._neck_cmd = cmd
            self.data.ctrl[self._pan_act] = -math.radians(cmd[0])
            self.data.ctrl[self._tilt_act] = math.radians(cmd[1])
        return {"pan": cmd[0], "tilt": cmd[1]}

    @require_initialization
    def get_neck_position(self):
        with self._lock:
            pan, tilt = self._neck_deg()
        return {"pan": pan, "tilt": tilt}

    @require_initialization
    def get_neck_limits(self):
        m = self.model
        pan_lo, pan_hi = np.degrees(m.jnt_range[m.joint("head_pan_joint").id])
        tilt_lo, tilt_hi = np.degrees(m.jnt_range[m.joint("head_tilt_joint").id])
        return {"pan": (float(-pan_hi), float(-pan_lo)), "tilt": (float(tilt_lo), float(tilt_hi))}

    @require_initialization
    def home_neck(self) -> bool:
        self.set_neck_position(0.0, 0.0)
        return True

    @require_initialization
    def set_neck_torque(self, on: bool):
        pass  # the sim's neck actuators can't go limp

    # =================================================== RPC: sim-only
    @require_initialization
    def get_sim_pose(self) -> list:
        """Robot frame in the sim world, wxyz_xyz."""
        with self._lock:
            T = self._T_world_robot()
        return mink.SE3.from_matrix(T).wxyz_xyz.tolist()


class StereoPublisher:
    """Renders the head ZED pair and publishes [topic, jpeg (left|right side by side),
    stereo_info json] over ZMQ PUB -- the frames the real ZED would give the Orin."""

    def __init__(self, sim: YORMujoco, address=VIDEO_PUB_ADDR, width=960, height=540, fps=30, jpeg_quality=75):
        self.sim, self.address = sim, address
        self.width, self.height, self.fps, self.quality = width, height, fps, jpeg_quality
        m = sim.model
        cam_l, cam_r = (m.camera(name).id for name in ZED_CAMERAS)
        fovy = math.radians(m.cam_fovy[cam_l])
        f = (height / 2.0) / math.tan(fovy / 2.0)  # MuJoCo cameras: square pixels, centred
        self.stereo_info = {
            "type": "stereo_info", "width": width, "height": height, "fx": f, "fy": f,
            "cx": width / 2.0, "cy": height / 2.0,
            "baseline_m": float(np.linalg.norm(m.cam_pos[cam_l] - m.cam_pos[cam_r])), "model": "ZED 2 (sim)",
        }
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)

    def _run(self):
        import json

        import cv2
        import zmq

        sock = zmq.Context.instance().socket(zmq.PUB)
        sock.setsockopt(zmq.SNDHWM, 2)
        sock.bind(self.address)
        info = json.dumps(self.stereo_info).encode()
        m = self.sim.model
        scratch = mujoco.MjData(m)
        renderer = mujoco.Renderer(m, self.height, self.width)  # GL context lives on this thread
        rate = RateLimiter(self.fps, name="zed_sim", warn=False)
        print(f"[YORMujoco] ZED stereo {self.width}x{self.height}/eye @ {self.fps} fps on {self.address}")
        try:
            while not self._stop.is_set():
                with self.sim._lock:
                    mujoco.mj_copyData(scratch, m, self.sim.data)  # both eyes from one instant
                eyes = []
                for cam in ZED_CAMERAS:
                    renderer.update_scene(scratch, camera=cam)
                    eyes.append(renderer.render())
                frame = cv2.cvtColor(np.hstack(eyes), cv2.COLOR_RGB2BGR)
                ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
                if ok:
                    sock.send_multipart([b"head_stereo", jpeg.tobytes(), info])
                rate.sleep()
        finally:
            renderer.close()
            sock.close(linger=0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true", help="no viewer window")
    parser.add_argument("--port", type=int, default=YOR_PORT)
    parser.add_argument("--model", default=str(SIM_MODEL), help="physics MJCF (default: scene-full.mjcf)")
    parser.add_argument("--no-video", action="store_true", help=f"don't publish ZED frames on {VIDEO_PUB_ADDR}")
    args = parser.parse_args()

    yor = YORMujoco(model_path=args.model, headless=args.headless)
    yor.init()
    video = None if args.no_video else StereoPublisher(yor)
    if video:
        video.start()
    server = RPCServer(yor, port=args.port, threaded=True)

    def graceful_shutdown():
        print("\nRPC Server stopping...")
        server.stop()
        if video:
            video.stop()
        yor.stop()

    atexit.register(graceful_shutdown)
    server.start()
    print(f"[YORMujoco] RPC server on port {args.port}")
    try:
        while yor.is_running():
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
