# yor.py
import os
import sys
import functools
import time
import numpy as np
import mink
import atexit
from pathlib import Path

# Add project root to sys.path
_HERE = Path(__file__).parent
_ROOT = _HERE.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

# Import Base from either package layout (robot/base.py) or flat (base.py)
from robot.base import Base

from robot.arm.arm import ArmNode
from robot.base import BaseController
from robot.config import LIFT_AUTO_HOME, LIFT_MAX_M, LIFT_MIN_M
from commlink import RPCServer, Subscriber
import threading

THOR_IP = '192.168.1.11'
ZED_PUB_PORT = 6000
POSE_TOPIC = "zed/pose"

YOR_PORT = 5557

def require_initialization(func):
    @functools.wraps(func)
    def wrapper(self, *args, **kwargs):
        if not self._initialized:
            print(f"Warning: {func.__name__} called before YOR was initialized")
            return None
        return func(self, *args, **kwargs)

    return wrapper

def require_zed(func):
    @functools.wraps(func)
    def wrapper(self, *args, **kwargs):
        if not self._zed_initialized:
            print(f"Warning: {func.__name__} called before Zed Subscriber was initialized")
            return None
        return func(self, *args, **kwargs)

    return wrapper


class YOR():
    def __init__(
        self,
        base_max_vel=np.array((1.0, 1.0, 1.57)),
        base_max_accel=np.array((1.0, 1.0, 1.57)),
        no_arms: bool = False,
        # AgileX CAN grippers (CanGripper, CAN 0x159/0x2A8) on both arms.
        no_gripper: bool = False,
        # SameHand fingers on the right arm instead (via the SameHand bridge over Zenoh);
        # the SameHand setup lives on the umami branch.
        samehand_gripper: bool = False,
        no_neck: bool = False,
        # Neck servo speed limits (STS units); None = robot/neck/neck.py defaults.
        neck_servo_speed: int | None = None,
        neck_servo_accel: int | None = None,
        # Home the lift at init() when it isn't homed yet (only after a Pico power-on:
        # the homed mark survives yor.py restarts). Default: robot/config.py LIFT_AUTO_HOME.
        lift_auto_home: bool | None = None,
    ):
        self._initialized = False
        self.lift_auto_home = LIFT_AUTO_HOME if lift_auto_home is None else lift_auto_home
        self._zed_initialized = False

        self.zed_sub = None
        self._reset_nav = False
        

        self.pose = None        # tuple of ((x,y,z), theta_z, 4x4_pose)


        self.base_controller = BaseController(
            yor=self,
            base_max_vel=base_max_vel,
            base_max_accel=base_max_accel,
            origin=(0.0, 0.0),
            grid_res=0.05,
            control_hz=20,
        )
        self.base = self.base_controller.base
        self.no_arms = no_arms

        # Neck (2-DOF Feetech pan/tilt). Optional: if the adapter or calibration
        # is missing, warn and run without it.
        self.neck = None
        self._neck_cmd = None  # last commanded {"pan", "tilt"} in degrees
        self._lift_cmd = 0.0  # last manual lift command: +1 up, -1 down, 0 stop
        if not no_neck:
            try:
                from robot.neck.neck import Neck
                neck = Neck()
                if not neck.is_calibrated:
                    raise RuntimeError("not calibrated (run robot/neck/neck_setup.py calibrate)")
                neck.connect(velocity=neck_servo_speed, accel=neck_servo_accel)
                self.neck = neck
                print(f"[YOR] Neck connected on {neck.bus.port}")
            except Exception as e:
                print(f"[YOR] Neck disabled: {e}")
        if not self.no_arms:
            _HERE = Path(__file__).parent
            self.left_arm = ArmNode(
                can_port="can_left",
                mjcf_path=(_HERE / "yor-description/nero-welded-base-and-lift.mjcf").as_posix(),
                dynamixel_gripper=False,
                no_gripper=no_gripper,
            )
            self.right_arm = ArmNode(
                can_port="can_right",
                mjcf_path=(_HERE / "yor-description/nero-welded-base-and-lift.mjcf").as_posix(),
                is_left_arm=False,
                dynamixel_gripper=False,
                no_gripper=no_gripper,
                samehand_gripper=samehand_gripper,
            )
     

    def init(self):
        if self._initialized:
            print("Warning: YOR already initialized")
            return

        # Start the SparkFlex control loop
        self.base.start_control()
        time.sleep(0.5)

        # An unhomed lift (Pico power-on) is homed here, before the arms init and before the
        # RPC server starts, so nothing else can command the lift while it runs. Homing drives
        # the carriage down to the bottom stop: the space below the arms must be clear.
        time.sleep(0.5)
        lift = self.base.get_lift_status()
        if lift["ref"] is not None and not lift["homed"]:
            if self.lift_auto_home:
                self._auto_home_lift()
            else:
                print("[YOR] WARNING: lift NOT homed: lift_height is relative to where the Pico powered on. "
                      "Clear the space below the arms and run lift_home (python robot/lift_home.py).")

        if self.neck is not None and self.neck.home_deg:
            try:
                reached = self.neck.go_home()
                self._neck_cmd = dict(self.neck.home_deg)
                print(f"[YOR] Neck {'homed' if reached else 'homing timed out'}: {self.neck.get_position_deg()}")
            except Exception as e:
                print(f"[YOR] Neck homing failed: {e}")

        # Arms remain optional
        if not self.no_arms:
            self.left_arm.init()
            self.right_arm.init()

        self._initialized = True

    # Base
    @require_initialization
    def set_base_velocity(self, velocity: np.ndarray):
        self.base_controller.mode = "BASE_VEL"
        self.base_controller.target_velocity = velocity
        self.base_controller.vel_cmd_time = time.monotonic()

    
    @require_initialization
    def follow_path(self, path=None):
        self.base_controller.zed_sub_init()

        if path is None:
            self.base_controller._path_world = None
            self.base_controller.mode = "BASE_VEL"
            self.base_controller.target_velocity = np.zeros(3, dtype=float)
            print("[YOR] follow_path: cleared")
            return True

        clean = [(float(p[0]), float(p[1])) for p in path]
        self.base_controller._path_world = clean
        self.base_controller.mode = "PATH_FOLLOWING"
        print(f"[YOR] follow_path: n={len(clean)} first={clean[0]} last={clean[-1]}")
        return True
    
    @require_initialization
    def get_nav_debug(self):
        if hasattr(self.base_controller, "get_nav_debug"):
            return self.base_controller.get_nav_debug()
        return None

    @require_initialization  
    def move_to(self, goal = None):
        self.base_controller.zed_sub_init()
        self.base_controller._goal = goal
        self.base_controller.mode = "MOVE_TO"
    
    @require_initialization   
    def move_by(self, deltas = None):
        self.base_controller.zed_sub_init()
        if self.pose is None:
            print("Warning: move_by called before pose is available")
            return
        if deltas is None:
            print("Warning: move_by called without deltas")
            return
        translation, theta, T_base = self.pose               # (x,y,z), theta_z, 4x4 transform
        x, y = float(translation[0]), float(translation[2])  # (x,z) plane

        self.base_controller._goal = (x+deltas[0], y+deltas[1], theta+deltas[2])
        self.base_controller.mode = "MOVE_TO"


    # Lift controls (lift uses Pico serial up/down/stop)
    @require_initialization
    def lift_up(self) -> None:
        if hasattr(self.base, "lift_up"):
            self.base.lift_up()
            self._lift_cmd = 1.0

    @require_initialization
    def lift_down(self) -> None:
        if hasattr(self.base, "lift_down"):
            self.base.lift_down()
            self._lift_cmd = -1.0

    @require_initialization
    def lift_stop(self) -> None:
        if hasattr(self.base, "lift_stop"):
            self.base.lift_stop()
            self._lift_cmd = 0.0

    def _auto_home_lift(self) -> None:
        """init(): home the lift and wait (up to ~32 s). Ctrl+C stops the lift; a failed home
        only warns, so the robot still comes up (with relative lift heights)."""
        print("[YOR] Lift not homed: homing it now (down to the bottom stop; Ctrl+C stops it, "
              "--no-lift-home / YOR_LIFT_AUTOHOME=0 skips this)")
        if not self.base.lift_home():
            print("[YOR] Lift homing already running")
        try:
            time.sleep(0.2)
            while self.base.get_lift_status()["homing"]:
                time.sleep(0.2)
        except KeyboardInterrupt:
            self.base.lift_stop()
            print("[YOR] Lift homing interrupted: lift stopped, NOT homed")
            raise
        st = self.base.get_lift_status()
        if st["homed"]:
            print(f"[YOR] Lift homed: height {self.base.get_lift_height()} m above the bottom stop")
        else:
            print(f"[YOR] WARNING: lift homing FAILED ({st['home_error']}): lift_height stays relative to "
                  "where the Pico powered on. Fix it and run python robot/lift_home.py.")

    @require_initialization
    def lift_set_home_here(self) -> bool:
        """Mark the lift's current position as the bottom stop (height 0, homed) without moving.
        Only use it when the carriage really is at the bottom stop."""
        return self.base.lift_set_home_here()

    @require_initialization
    def lift_home(self) -> bool:
        """Start homing the lift: up ~13 mm, then down until it stalls at the bottom stop
        (up to ~32 s). Returns at once; False if already homing. Watch get_lift_status()."""
        return self.base.lift_home()

    @require_initialization
    def get_lift_status(self) -> dict:
        """{"homed", "ref", "homing", "home_error"}: see Base.get_lift_status."""
        return self.base.get_lift_status()

    @require_initialization
    def get_lift_height(self):
        """Measured lift height in m (Pico, polled at 50 Hz), or None when it is not a live reading."""
        return self.base.get_lift_height()

    @require_initialization
    def get_lift_height_error(self):
        """Why get_lift_height() is None, or None when it is a live reading."""
        return self.base.get_lift_height_error()

    @require_initialization
    def lift_delta_height(
        self,
        delta_m: float,
        tolerance_m: float = 0.002,
        timeout_s: float = 30.0,
        min_height_m: float = LIFT_MIN_M,
        max_height_m: float = LIFT_MAX_M,
    ) -> bool:
        if not hasattr(self.base, "lift_delta_height"):
            print("[YOR] base has no lift_delta_height()")
            return False
        try:
            return bool(self.base.lift_delta_height(
                delta_m,
                tolerance_m=tolerance_m,
                timeout_s=timeout_s,
                min_height_m=min_height_m,
                max_height_m=max_height_m,
            ))
        except TypeError:
            return bool(self.base.lift_delta_height(delta_m))


    @require_initialization
    def set_lift_position(self, target_m) -> bool:
        """Alias used by yor_teleop / vr_teleop reset path."""
        height = float(target_m) if np.isscalar(target_m) else float(np.asarray(target_m).ravel()[0])
        return self.lift_to_height(height)

    @require_initialization
    def lift_to_height(
        self,
        target_m: float,
        tolerance_m: float = 0.002,
        timeout_s: float = 30.0,
        min_height_m: float = LIFT_MIN_M,
        max_height_m: float = LIFT_MAX_M,
    ) -> bool:
        if not hasattr(self.base, "lift_to_height"):
            print("[YOR] base has no lift_to_height()")
            return False
        return bool(self.base.lift_to_height(
            target_m,
            tolerance_m=tolerance_m,
            timeout_s=timeout_s,
            min_height_m=min_height_m,
            max_height_m=max_height_m,
        ))


    #-------------------ARMS---------------------
    # Left arm
    @require_initialization
    def set_left_joint_target(
        self, joint_target: np.ndarray, gripper_target: float | None = None, preview_time: float = 1.0
    ):
        if self.no_arms:
            print("left arm disabled")
            return
        self.left_arm.set_joint_target(joint_target, gripper_target, preview_time)

    @require_initialization
    def set_left_ee_target(self, ee_target: mink.SE3, gripper_target: float | None = None, preview_time: float = 0.3):
        if self.no_arms:
            print("left arm disabled")
            return
        self.left_arm.set_ee_target(ee_target, gripper_target, preview_time)

    @require_initialization
    def set_left_gripper(self, gripper_pos: float):
        if self.no_arms:
            print("left arm disabled")
            return
        self.left_arm.set_gripper(gripper_pos)

    @require_initialization
    def set_left_gain(self, kp: np.ndarray, kd: np.ndarray):
        if self.no_arms:
            print("left arm disabled")
            return
        self.left_arm.set_gain(kp, kd)

    @require_initialization
    def home_left_arm(self, gripper_target: float = 1.0):
        if self.no_arms:
            print("left arm disabled")
            return
        # Delegate to ArmNode if it provides homing; ignore if not available
        if hasattr(self.left_arm, "home"):
            self.left_arm.home(gripper_target)
        else:
            print("left arm: home() not available")

    @require_initialization
    def tuck_left_arm(self):
        if self.no_arms:
            print("left arm disabled")
            return
        self.left_arm.tuck_arms()

    @require_initialization
    def disable_left_gripper(self):
        """Let the left gripper go limp (for hardware debugging/calibration); arm joints unaffected."""
        if self.no_arms:
            print("left arm disabled")
            return
        self.left_arm.disable_gripper()

    @require_initialization
    def enable_left_gripper(self):
        """Re-enable the left gripper after disable_left_gripper(). Returns True on success."""
        if self.no_arms:
            print("left arm disabled")
            return False
        return self.left_arm.enable_gripper()

    @require_initialization
    def open_left_gripper(self):
        if self.no_arms:
            print("left arm disabled")
            return
        self.left_arm.open_gripper()

    @require_initialization
    def close_left_gripper(self):
        if self.no_arms:
            print("left arm disabled")
            return
        self.left_arm.close_gripper()

    @require_initialization
    def get_left_ee_pose(self) -> mink.SE3:
        """
        Returns the pose of the left arm's end effector in the world frame (qw, qx, qy, qz, x, y, z).
        """
        if self.no_arms:
            print("left arm disabled")
            return None
        if hasattr(self.left_arm, "get_ee_pose"):
            return self.left_arm.get_ee_pose()
        print("left arm: get_ee_pose() not available")
        return None

    @require_initialization
    def get_left_joint_positions(self) -> np.ndarray:
        if self.no_arms:
            print("left arm disabled")
            return None
        return self.left_arm.get_joint_positions()

    @require_initialization
    def get_left_gripper_pose(self):
        if self.no_arms:
            print("left arm disabled")
            return None
        return self.left_arm.get_gripper_pose()

    # Right arm
    @require_initialization
    def set_right_joint_target(
        self, joint_target: np.ndarray, gripper_target: float | None = None, preview_time: float = 1.0
    ):
        if self.no_arms:
            print("right arm disabled")
            return
        self.right_arm.set_joint_target(joint_target, gripper_target, preview_time)

    @require_initialization
    def set_right_ee_target(self, ee_target: mink.SE3, gripper_target: float | None = None, preview_time: float = 0.3):
        if self.no_arms:
            print("right arm disabled")
            return
        self.right_arm.set_ee_target(ee_target, gripper_target, preview_time)

    @require_initialization
    def set_right_gripper(self, gripper_pos: float):
        if self.no_arms:
            print("right arm disabled")
            return
        self.right_arm.set_gripper(gripper_pos)

    @require_initialization
    def set_right_gain(self, kp: np.ndarray, kd: np.ndarray):
        if self.no_arms:
            print("right arm disabled")
            return
        self.right_arm.set_gain(kp, kd)

    @require_initialization
    def home_right_arm(self, gripper_target: float = 1.0):
        if self.no_arms:
            print("right arm disabled")
            return
        if hasattr(self.right_arm, "home"):
            self.right_arm.home(gripper_target)
        else:
            print("right arm: home() not available")

    @require_initialization
    def tuck_right_arm(self):
        if self.no_arms:
            print("right arm disabled")
            return
        self.right_arm.tuck_arms()

    @require_initialization
    def disable_right_gripper(self):
        """Let the right gripper go limp (for hardware debugging/calibration); arm joints unaffected."""
        if self.no_arms:
            print("right arm disabled")
            return
        self.right_arm.disable_gripper()

    @require_initialization
    def enable_right_gripper(self):
        """Re-enable the right gripper after disable_right_gripper(). Returns True on success."""
        if self.no_arms:
            print("right arm disabled")
            return False
        return self.right_arm.enable_gripper()

    @require_initialization
    def open_right_gripper(self):
        if self.no_arms:
            print("right arm disabled")
            return
        self.right_arm.open_gripper()

    @require_initialization
    def close_right_gripper(self):
        if self.no_arms:
            print("right arm disabled")
            return
        self.right_arm.close_gripper()

    @require_initialization
    def get_right_ee_pose(self) -> mink.SE3:
        if self.no_arms:
            print("right arm disabled")
            return None
        if hasattr(self.right_arm, "get_ee_pose"):
            return self.right_arm.get_ee_pose()
        print("right arm: get_ee_pose() not available")
        return None

    @require_initialization
    def get_right_joint_positions(self) -> np.ndarray:
        if self.no_arms:
            print("right arm disabled")
            return None
        return self.right_arm.get_joint_positions()

    @require_initialization
    def get_right_gripper_pose(self):
        if self.no_arms:
            print("right arm disabled")
            return None
        return self.right_arm.get_gripper_pose()

    @require_initialization
    def get_arm_relative_pose(self) -> tuple[mink.SE3, mink.SE3]:
        left_ee_pose = self.get_left_ee_pose()
        right_ee_pose = self.get_right_ee_pose()
        l2r = right_ee_pose.inverse() @ left_ee_pose
        r2l = left_ee_pose.inverse() @ right_ee_pose

        return r2l, l2r

    @require_initialization
    def get_bimanual_state(self) -> dict:
        """
        Get all bimanual state data in a single call for high-speed data logging.
        Returns dict with left/right ee poses and joint positions and lift position.
        Note: Lift position is 0.0 as new hardware doesn't support position feedback.
        """
        row = [0.0] * (1 + 7 + 7 + 1 + 7 + 7 + 1 + 1)
        if not self.no_arms:
            row[0] = time.time()
            row[1:8] = self.left_arm.get_ee_pose().wxyz_xyz.tolist()
            row[8:15] = self.left_arm.get_joint_positions().tolist()
            row[15] = self.left_arm.get_gripper_pose()
            row[16:23] = self.right_arm.get_ee_pose().wxyz_xyz.tolist()
            row[23:30] = self.right_arm.get_joint_positions().tolist()
            row[30] = self.right_arm.get_gripper_pose()
            row[31] = 0.0  # lift position not available on new hardware
            return row
        else:
            row[0] = time.time()
            row[1:8] = [0.90724, -0.41142, 0.075, -0.04495, 0.10741, 0.11358, 0.89066] # roughly tucked position
            row[8:15] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # tucked position
            row[15] = 1.0 # fully open
            row[16:23] = [0.90029, 0.42914, 0.06059, 0.04051, 0.10338, -0.53731, 0.89969]
            row[23:30] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # tucked position
            row[30] = 1.0
            row[31] = 0.0  
            return row

    @require_initialization
    def get_collection_state(self) -> dict:
        """Everything a data-collection frame needs, sampled in one RPC call.

        Gripper values are never filled in from each other: {side}_gripper is the measured
        opening (None when feedback drops, and {side}_gripper_error then says why) and
        {side}_gripper_cmd the commanded one (None while the gripper is limp).
        enc_vel is body velocity [vx, vy, wz] from swerve forward kinematics on
        the measured wheel encoders. lift_cmd is the last manual lift command
        (+1 up, -1 down, 0 stop); lift_height is the measured height in m (None when
        the Pico reading drops, and lift_height_error then says why): travel above the
        bottom stop when lift_homed, else relative to the Pico's power-on position;
        lift_ref names which ("bottom_hard_stop" / "pico_power_on"), lift_homing is True
        while lift_home runs.
        """
        state = {"timestamp": time.time(), "base_cmd_vel": np.asarray(
            self.base_controller.target_velocity, dtype=float).tolist(), "lift_cmd": self._lift_cmd}
        state["lift_height"] = self.base.get_lift_height()
        state["lift_height_error"] = None if state["lift_height"] is not None else (
            self.base.get_lift_height_error() or "lift height not available")
        lift = self.base.get_lift_status()
        state.update(lift_homed=lift["homed"], lift_ref=lift["ref"], lift_homing=lift["homing"])
        state["enc_vel"] = self.base.get_measured_body_velocity()
        if self.neck is not None:
            try:
                pos = self.neck.get_position_deg()
                state["neck_deg"] = [pos["pan"], pos["tilt"]]
            except Exception:
                state["neck_deg"] = None
            cmd = self._neck_cmd
            state["neck_cmd_deg"] = None if not cmd else [cmd.get("pan"), cmd.get("tilt")]
        if self.no_arms:
            return state
        for side, arm in (("left", self.left_arm), ("right", self.right_arm)):
            ee = arm.get_ee_pose()
            measured, commanded, gripper_error = arm.get_gripper_state()
            state[f"{side}_ee"] = ee.wxyz_xyz.tolist() if ee is not None else None
            state[f"{side}_joints"] = arm.get_joint_positions().tolist()
            q_cmd = arm.last_joint_target
            state[f"{side}_joints_cmd"] = None if q_cmd is None else q_cmd.tolist()
            state[f"{side}_gripper"] = None if measured is None else float(measured)
            state[f"{side}_gripper_cmd"] = None if commanded is None else float(commanded)
            state[f"{side}_gripper_error"] = gripper_error
            state[f"{side}_ik_ok"] = None if arm.last_ik_solved is None else bool(arm.last_ik_solved)
        return state

    @require_initialization
    def get_ik_status(self) -> tuple[bool | None, bool | None]:
        """(left_is_solved, right_is_solved) from each arm's most recent EE-target IK solve --
        see ArmNode.set_ee_target()/last_ik_solved. None if that arm has never been given an EE
        target yet (joint-space-only commands so far, or nothing at all) -- kept distinct from
        True/False so "no data" isn't read as "solved". Exposed as its own RPC call (2026-09-10)
        rather than folded into get_bimanual_state()'s row, since that method's fixed-width list
        layout is depended on elsewhere (recording/replay) and this is a genuinely separate,
        optional signal, not part of the robot's physical state."""
        if self.no_arms:
            return (None, None)
        return (self.left_arm.last_ik_solved, self.right_arm.last_ik_solved)

    @require_initialization
    def set_bimanual_ee_target(self,
                           L_ee_target: mink.SE3, R_ee_target: mink.SE3,
                           L_gripper_target: float | None = None, L_preview_time: float = 0.1,
                           R_gripper_target: float | None = None, R_preview_time: float = 0.1):
        self.left_arm.set_ee_target(L_ee_target, L_gripper_target, L_preview_time)
        self.right_arm.set_ee_target(R_ee_target, R_gripper_target, R_preview_time)

    # Neck (degrees from the centre of each joint's calibrated range)
    @require_initialization
    def set_neck_position(self, pan: float | None = None, tilt: float | None = None):
        """Move the neck; None leaves that joint unchanged. Goals are clipped to the
        calibrated range. Returns the goal actually sent, or None without a neck."""
        if self.neck is None:
            return None
        goal = self.neck.set_position_deg(pan=pan, tilt=tilt)
        self._neck_cmd = {**(self._neck_cmd or {}), **goal}
        return goal

    @require_initialization
    def get_neck_position(self):
        """{"pan": deg, "tilt": deg}, or None without a neck."""
        return None if self.neck is None else self.neck.get_position_deg()

    @require_initialization
    def get_neck_limits(self):
        """{"pan": (min, max), "tilt": (min, max)} in degrees, or None without a neck."""
        return None if self.neck is None else self.neck.get_limits_deg()

    @require_initialization
    def home_neck(self) -> bool:
        if self.neck is None:
            return False
        reached = self.neck.go_home()
        self._neck_cmd = dict(self.neck.home_deg)
        return reached

    @require_initialization
    def set_neck_torque(self, on: bool):
        """Torque off lets the neck be posed by hand; any set_neck_position() turns it back on."""
        if self.neck is None:
            return
        self.neck.torque(bool(on))

    @require_initialization
    def get_cmd_vel(self):
        # returns ([vx, vy, omega], timestamp)
        v = np.asarray(self.base_controller.target_velocity, dtype=float)
        return v.tolist(), time.time()

    @require_initialization
    def get_base_encoders(self) -> dict:
        """Return steer positions (rad) and drive velocities (raw) for all 4 modules."""
        base = self.base
        return {
            "timestamp": time.time(),
            "steer_rad":    [m.get_position_rad()    for m in base.rotation_motors],
            "steer_deg":    [m.get_position_deg()    for m in base.rotation_motors],
            "steer_counts": [m.get_position_counts() for m in base.rotation_motors],
            "drive_vel":    [m.get_velocity_raw()    for m in base.drive_motors],
            "drive_counts": [m.get_position_counts() for m in base.drive_motors],
            "lift_height_m": base.get_lift_height(),
        }

    @require_initialization
    def get_pose(self) -> dict:
        """Return ZED IMU-fused pose: x, y, theta (yaw in radians).
        x = translation[0], y = translation[2] (robot moves in XZ plane).
        """
        if self.pose is None:
            return {"x": None, "y": None, "theta": None}
        translation, theta, _ = self.pose
        return {
            "x": float(translation[0]),
            "y": float(translation[2]),
            "theta": float(theta),
        }


def main():
    import argparse
    import signal
    parser = argparse.ArgumentParser()
    parser.add_argument('--no-gripper', action='store_true',
                        help='Run without the AgileX CAN grippers')
    parser.add_argument('--samehand', action='store_true',
                        help='SameHand fingers on the right arm instead of its AgileX gripper')
    parser.add_argument('--no-arms', action='store_true', help='Disable arm initialization (base-only mode)')
    parser.add_argument('--no-neck', action='store_true', help='Run without the Feetech neck')
    parser.add_argument('--neck-servo-speed', type=int, default=None,
                        help='Neck servo max speed, steps/s (4096/rev; default 2000 ~ 175 deg/s; 0 = no limit)')
    parser.add_argument('--neck-servo-accel', type=int, default=None,
                        help='Neck servo acceleration, 0-254 in 100 steps/s^2 (default 80; 0 = no limit)')
    parser.add_argument('--no-lift-home', action='store_true',
                        help="don't home the lift at startup when it isn't homed (env YOR_LIFT_AUTOHOME=0)")
    args = parser.parse_args()

    yor = YOR(no_arms=args.no_arms, no_gripper=args.no_gripper,
              samehand_gripper=args.samehand, no_neck=args.no_neck,
              neck_servo_speed=args.neck_servo_speed, neck_servo_accel=args.neck_servo_accel,
              lift_auto_home=False if args.no_lift_home else None)
    yor.init()
    server = RPCServer(yor, port=YOR_PORT, threaded=True)

    _shutdown_called = False
    stop_requested = threading.Event()

    def graceful_shutdown():
        nonlocal _shutdown_called
        if _shutdown_called:
            return
        _shutdown_called = True

        print("\n[YOR] Shutting down...")
        try:
            server.stop()
        except Exception as e:
            print(f"[YOR] Server stop failed: {e}")

        # Base: stop the nav/velocity worker, then the motor loop. The SparkFlex
        # motors stop on their own once heartbeats stop.
        try:
            yor.base_controller.stop()
            yor.base.stop_control()
        except Exception as e:
            print(f"[YOR] Base stop failed: {e}")
        try:
            yor.base.lift_stop()
        except Exception as e:
            print(f"[YOR] Lift stop failed: {e}")

        if yor.neck is not None:
            # Leave torque ON so the head holds its pose instead of dropping.
            # `python robot/neck/neck_setup.py read` releases it.
            try:
                yor.neck.close(disable_torque=False)
            except Exception as e:
                print(f"[YOR] Neck close failed: {e}")

        if not yor.no_arms:
            for label, arm in [("LEFT", getattr(yor, "left_arm", None)),
                                ("RIGHT", getattr(yor, "right_arm", None))]:
                if arm is not None:
                    print(f"[YOR] Stopping {label} arm...")
                    try:
                        arm.stop()
                    except Exception as e:
                        print(f"[YOR] {label} arm stop failed: {e}")

        print("[YOR] Shutdown complete.")

    def on_signal(signum, frame):
        if stop_requested.is_set():
            # Second Ctrl-C while shutdown is in progress: force quit.
            print("\n[YOR] Forced exit.", flush=True)
            os._exit(1)
        stop_requested.set()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    atexit.register(graceful_shutdown)

    server.start()
    # Shutdown runs here in the main thread, not inside the signal handler, so
    # the process actually exits afterwards.
    while not stop_requested.wait(1.0):
        pass
    graceful_shutdown()
    sys.exit(0)



if __name__ == "__main__":
    main()
