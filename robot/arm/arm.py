
import json
import os
import threading
import time
from pathlib import Path
from typing import Optional
import numpy as np
import mink

try:
    from nerolib import NeroController, ControllerConfig, JointState, Gain, ControlMode, MoveMode
except ImportError:
    print("nerolib not found. Please install it or use the 'nerolib' conda environment.")
    raise

try:
    from robot.arm.gripper import Gripper
except ImportError:
    Gripper = None  # dynamixel_sdk not installed; Gripper unavailable
from robot.arm.can_gripper import CanGripper
from robot.arm.ik_solver import SingleArmIK

# Dynamixel gripper caliberated in dxl.py
# DXL_ID_RIGHT = 2, DXL_ID_LEFT = 3
BAUDRATE = 1000000 
DXL_ID_RIGHT = 2
DXL_ID_LEFT = 3

# Opt-in per-solve IK log (2026-10-19), for debugging orientation failures offline.
# YOR_IK_LOG=1 writes ~/ik_log_<timestamp>.jsonl; any other non-empty value is used as the path.
# One JSON line per set_ee_target() call, both arms in the same file. Poses are in the MJCF
# world frame as returned by mink.SE3, quaternions in w,x,y,z order, positions in meters,
# joints in radians (joint1..joint7, the frame get_joint_positions() returns).
_IK_LOG_ENV = os.environ.get("YOR_IK_LOG", "")
_ik_log_file = None
_ik_log_lock = threading.Lock()


def _ik_log(record: dict):
    # A logging problem (bad path, full disk) disables the log; it must never break arm control.
    global _ik_log_file, _IK_LOG_ENV
    if not _IK_LOG_ENV:
        return
    with _ik_log_lock:
        try:
            if _ik_log_file is None:
                path = (os.path.expanduser(f"~/ik_log_{time.strftime('%Y%m%d_%H%M%S')}.jsonl")
                        if _IK_LOG_ENV == "1" else _IK_LOG_ENV)
                _ik_log_file = open(path, "a", buffering=1)
                print(f"[ArmNode] IK log -> {path}")
            _ik_log_file.write(json.dumps(record) + "\n")
        except OSError as e:
            print(f"[ArmNode] IK log disabled ({e})")
            _IK_LOG_ENV = ""


class ArmNode:
    # After this many CONSECUTIVE non-converged IK solves, set_ee_target() stops sending the
    # unconverged qd and holds the last position that DID converge instead. At this arm's
    # typical ~30Hz command rate that's ~0.3s of genuine, persistent failure -- not a single
    # noisy iteration -- chosen to react quickly without treating one bad solve as a crisis.
    IK_FAIL_HOLD_THRESHOLD = 10

    def __init__(
        self,
        can_port: str,
        mjcf_path: str,
        solver_dt: float = 0.01,
        is_left_arm: bool = True,
        dynamixel_gripper: bool = False,
        no_gripper: bool = False,
        samehand_gripper: bool = False,
        default_kp: Optional[float | list[float]] = 15.0,
        default_kd: Optional[float | list[float]] = 0.8,
        gravity_comp_scale: float = 1.0,
    ):
        _ROOT = Path(__file__).parent.parent.parent
        self.can_port = can_port
        self.is_left_arm = is_left_arm
        if is_left_arm:
            self.urdf_path = (_ROOT / "nerolib/urdf/nero_cone-e_left_fixed.urdf").as_posix()
        else:
            self.urdf_path = (_ROOT / "nerolib/urdf/right_arm_final.urdf").as_posix()
        self.solver_dt = solver_dt

        # Initialize nerolib NeroController
        self.control_mode_set = False
        try:
            print(f"[ArmNode] Initializing {can_port} with nerolib...")
            
            self.config = ControllerConfig()
            self.config.interface_name = can_port
            self.config.urdf_path = self.urdf_path
            
            # Defines home position (originally in ControllerConfig)
            # UPDATED 2026-09-11: set to the per-arm MEAN INITIAL JOINT POSITION over the
            # icl-dataset-fixed-action episodes, so a policy rollout starts from the middle of the
            # distribution the policy was trained on instead of an arbitrary safe pose. The
            # previous hand-picked home sat below the dataset mean on 11 of 14 joints (RMS 0.59
            # sigma), worst at the right wrist: rj4 and rj5 were each ~10 deg / ~1 sigma off.
            #
            # The two arms are NO LONGER an exact mirror, deliberately. The dataset's own start
            # poses are mirror-symmetric for j0-j3 and j6 (left/right gap < 0.05 rad) but not at
            # the wrist, where j5 differs by 0.197 rad and j4 by 0.112 between the arms. The old
            # mirrored home could not represent that, which is where its largest errors came from.
            #
            # All 14 values checked against the URDF limits. Every margin stays wide; the tightest
            # is left j1 at 0.363 rad (20.8 deg) from its +-1.74 limit, down from 0.464 rad. That
            # is the joint the previous comment flagged, so it is the one to watch if homing ever
            # misbehaves. Previous values kept in arm.py.bak_20260911_pre_home_dataset_means.
            self.home_position = (
                [1.8317, 1.3773, -1.7357, 1.5211, -0.1130, -0.0954, 0.0973]
                if is_left_arm
                else [-1.8391, 1.3285, 1.7432, 1.5277, -0.0006, 0.1011, 0.1097]
            )
            
            self.config.home_position = self.home_position
            
            self.config.joint_vel_max = [3.0] * 7
            self.config.joint_acc_max = [15.0] * 7
            
            if default_kp is not None:
                if isinstance(default_kp, (int, float)):
                    self.config.default_kp = [float(default_kp)] * 7
                else:
                    self.config.default_kp = [float(x) for x in default_kp]
            
            if default_kd is not None:
                if isinstance(default_kd, (int, float)):
                    self.config.default_kd = [float(default_kd)] * 7
                else:
                    self.config.default_kd = [float(x) for x in default_kd]

            self.config.gravity_compensation = False
            self.config.gravity_comp_scale = gravity_comp_scale
            self.config.gripper_on = False  # CanGripper owns all 0x159 traffic

            self.nero = NeroController(self.config)

            if not self.nero.start():
                 print(f"[ArmNode] Failed to start NeroController on {can_port}")
                 self.nero = None
            else:
                 print(f"[ArmNode] {can_port} initialized and started.")
                 self.control_mode_set = True
            
        except Exception as e:
            print(f"[ArmNode] Failed to initialize arm on {can_port}: {e}")
            self.nero = None

        self.target: Optional[mink.SE3] = None
        self.gripper_target: Optional[float] = None
        self.q_offset = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # IK convergence tracking (2026-09-10) -- set_ee_target() used to silently discard
        # solve_ik()'s is_solved flag (qd, _ = ...), so a failed-to-converge target would still
        # get sent with no signal anywhere that it happened. last_ik_solved is exposed to
        # yor.py's get_ik_status() RPC method so joystick_policy_teleop.py (a separate process,
        # talking to yor.py over RPC, not a direct import of this module) can see it too.
        # None = this arm has never been given an EE target (joint-space-only commands so far,
        # or nothing yet) -- distinct from True/False so "no data" isn't confused with "solved".
        self.last_ik_solved: Optional[bool] = None
        # Last joint target actually sent to nerolib (same frame as get_joint_positions()),
        # recorded as the absolute-joint action for policy training. None until first command.
        self.last_joint_target: Optional[np.ndarray] = None
        self.ik_fail_streak = 0
        if is_left_arm:
            joint_names = [
                "left_arm_joint1",
                "left_arm_joint2",
                "left_arm_joint3",
                "left_arm_joint4",
                "left_arm_joint5",
                "left_arm_joint6",
                "left_arm_joint7",
            ]
            ee_frame = "left_arm_ee"
        else:
            joint_names = [
                "right_arm_joint1",
                "right_arm_joint2",
                "right_arm_joint3",
                "right_arm_joint4",
                "right_arm_joint5",
                "right_arm_joint6",
                "right_arm_joint7",
            ]
            ee_frame = "right_arm_ee"

        self.ik_solver = None
        if mjcf_path:
            try:
                self.ik_solver = SingleArmIK(
                    mjcf_path,
                    solver_dt=self.solver_dt,
                    joint_names=joint_names,
                    ee_frame=ee_frame,
                )
            except Exception as e:
                print(f"[ArmNode] IK solver init failed ({e}). EE pose methods unavailable.")

        self.dynamixel_gripper = dynamixel_gripper
        self.gripper = None
        self.can_gripper: CanGripper | None = None

        if self.dynamixel_gripper and Gripper is not None:
            # If your Gripper class uses /dev/ttyUSB0 internally, this will fail when unplugged.
            # Auto-disable instead of crashing.
            try:
                dxl_id = DXL_ID_LEFT if is_left_arm else DXL_ID_RIGHT
                self.gripper = Gripper(baudrate=BAUDRATE, dxl_id=dxl_id)

                open_gripper_value, close_gripper_value = self.gripper.dxl.calibrate_motor()
                self.open_gripper_value = open_gripper_value
                self.close_gripper_value = close_gripper_value
                self.gripper_range = open_gripper_value - close_gripper_value

                print("[ArmNode] Dynamixel gripper enabled")

            except (FileNotFoundError, OSError) as e:
                print(f"[ArmNode] Dynamixel gripper not found ({e}). Continuing without gripper.")
                self.dynamixel_gripper = False
                self.gripper = None

        if samehand_gripper:
            # SameHand fingers via the SameHand glove bridge (Zenoh). Same API as CanGripper.
            # Left limp here: torque comes on with enable_gripper() (whole-body teleop does
            # that on the first clutch with teleop on).
            try:
                from robot.arm.samehand_gripper import SameHandGripper
                self.can_gripper = SameHandGripper()
                print("[ArmNode] SameHand gripper connected (limp until enable_gripper())")
            except Exception as e:
                print(f"[ArmNode] SameHand gripper init failed ({e}). Continuing without gripper.")
                self.can_gripper = None
        elif not self.dynamixel_gripper and self.nero is not None and not no_gripper:
            try:
                # Both grippers' raw-zero references have been calibrated (AgileX
                # AgxGripper zero-point cal, CAN 0x159 set_zero) so raw 0 is
                # directly closed on both units; no per-side inversion needed.
                self.can_gripper = CanGripper(can_port, inverted=False, close_overtravel=0.10)
                self.can_gripper.enable()
                print(f"[ArmNode] CanGripper enabled on {can_port}")
            except Exception as e:
                print(f"[ArmNode] CanGripper init failed ({e}). Continuing without gripper.")
                self.can_gripper = None







    def _is_arm_broadcasting(self) -> bool:
        """Return True if any joint position is non-zero (arm is sending CAN state)."""
        if self.nero is None:
            return False
        q = np.array(self.nero.get_current_state().pos)
        return bool(np.any(q != 0.0))

    def init(self):
        if self.nero is None:
            print("[ArmNode] Warning: Nero not initialized, init() skipping hardware calls.")
            return

        # If the arm didn't enable during start() its joint positions stay at 0.0 (no CAN
        # feedback received). Wait up to 8s for the controller to wake up and start
        # broadcasting, then try again before homing.
        if not self._is_arm_broadcasting():
            print(f"[ArmNode] {self.can_port}: arm not broadcasting CAN state — waiting 8s for controller to wake up...")
            time.sleep(8.0)
            if not self._is_arm_broadcasting():
                print(f"[ArmNode] {self.can_port}: HARDWARE ISSUE — arm still not broadcasting after 8s delay.")
                print(f"[ArmNode]   → Power-cycle the {self.can_port} arm and restart yor.py.")
                return

        # Move to home position
        print(f"[ArmNode] Moving to home position on {self.can_port}...")
        self.nero.reset_to_home()

        # Sync IK solver with current state
        q = self.get_joint_positions()
        print(f"q_reached: {np.round(q, 4)}")

        if self.ik_solver is not None:
            self.ik_solver.init(q)
            self.target = self.ik_solver.forward_kinematics()
        

    def home(self, gripper_target: float = 1.0):
        if self.nero:
            try:
                self.nero.reset_to_home()
            except Exception as e:
                print(f"[ArmNode] Home failed: {e}")
        
        arm_side = "left" if self.is_left_arm else "right"
        if self.dynamixel_gripper:
            self.gripper.move_to_pos(int(gripper_target * self.gripper_range + self.close_gripper_value))
        elif self.can_gripper:
            print(f"[ArmNode] {arm_side} gripper: set_position({gripper_target})")
            self.can_gripper.set_position(float(gripper_target))
        else:
            print(f"[ArmNode] {arm_side} gripper: can_gripper is None, skipping")
        time.sleep(2.0)
        q = self.get_joint_positions()
        if self.ik_solver is not None:
            self.ik_solver.init(q)

    def tuck_arms(self):
        self.set_joint_target(np.zeros(7), gripper_target=1.00, preview_time=2.0)

    def set_joint_target(
        self, joint_target: np.ndarray, gripper_target: float | None = None, preview_time: float = 0.1
    ):
        if self.nero:
            try:
                # nerolib expects lists or std::vectors, typically python lists/arrays work with pybind11
                # The API signature: set_target(new_target_pos, new_target_gripper_pos, minimum_duration, new_target_vel, new_target_acc)
                # We map joint_target to new_target_pos.
                
                target_pos = (joint_target + self.q_offset).tolist()
                self.last_joint_target = np.array(joint_target, dtype=float)

                self.nero.set_target(
                    new_target_pos=target_pos,
                    new_target_gripper_pos=1.0,  # nerolib gripper_on=False; unused
                    minimum_duration=preview_time
                )

            except Exception as e:
                 print(f"Set target failed: {e}")

        if gripper_target is not None:
            if self.dynamixel_gripper:
                self.gripper.move_to_pos(int(gripper_target * self.gripper_range + self.close_gripper_value))
            elif self.can_gripper:
                self.can_gripper.set_position(float(gripper_target))

    def disable_gripper(self):
        """Let the gripper go limp without touching arm joint control (for hardware debugging/calibration)."""
        if self.can_gripper:
            self.can_gripper.disable()

    def enable_gripper(self) -> bool:
        """Re-enable the gripper after disable_gripper(). False if it couldn't (e.g. SameHand bridge down)."""
        if not self.can_gripper:
            return False
        try:
            self.can_gripper.enable()
            return True
        except Exception as e:
            print(f"[ArmNode] gripper enable failed: {e}")
            return False

    def open_gripper(self):
        if self.dynamixel_gripper:
            self.gripper.move_to_pos(self.open_gripper_value)
        elif self.can_gripper:
            self.can_gripper.set_position(1.0)

    def close_gripper(self):
        if self.dynamixel_gripper:
            self.gripper.move_to_pos(self.close_gripper_value)
        elif self.can_gripper:
            self.can_gripper.set_position(0.0)

    def set_ee_target(self, ee_target: mink.SE3, gripper_target: Optional[float] = None, preview_time: float = 0.0):
        if self.ik_solver is None:
            print("[ArmNode] set_ee_target: IK solver not available")
            return
        self.target = ee_target
        q_measured = self.get_joint_positions()
        self.ik_solver.update_configuration(q_measured)  # re-sync IK config with actual CAN positions before solving
        qd, is_solved = self.ik_solver.solve_ik(self.target)
        self.last_ik_solved = is_solved

        side = "left" if self.is_left_arm else "right"
        if _IK_LOG_ENV:
            target = np.asarray(ee_target.wxyz_xyz)
            achieved = np.asarray(self.ik_solver.forward_kinematics().wxyz_xyz)
            _ik_log({
                "t": time.time(),
                "arm": side,
                "q_measured": np.round(q_measured, 6).tolist(),
                "target_pos": np.round(target[4:], 6).tolist(),
                "target_quat_wxyz": np.round(target[:4], 6).tolist(),
                "qd": np.round(qd, 6).tolist(),
                "is_solved": bool(is_solved),
                # FK of qd, so position/orientation error can be computed without rerunning IK
                "achieved_pos": np.round(achieved[4:], 6).tolist(),
                "achieved_quat_wxyz": np.round(achieved[:4], 6).tolist(),
                # False when the fail-streak hold below suppresses the command
                "sent": bool(is_solved or self.ik_fail_streak + 1 < self.IK_FAIL_HOLD_THRESHOLD),
                "preview_time": float(preview_time),
            })
        if is_solved:
            if self.ik_fail_streak >= 3:  # only worth a recovery log if it was a real streak
                print(f"[ArmNode] IK recovered (side={side}) after {self.ik_fail_streak} "
                      f"consecutive failed solves.")
            self.ik_fail_streak = 0
        else:
            self.ik_fail_streak += 1
            print(f"[ArmNode] WARNING: IK did not converge (side={side}, "
                  f"consecutive_failures={self.ik_fail_streak})")
            if self.ik_fail_streak >= self.IK_FAIL_HOLD_THRESHOLD:
                # Sending qd here would command a target we know didn't converge -- e.g. an
                # unreachable pose, near a singularity, or a target that's drifted far from
                # anything the arm can actually track. Holding the last converged position is
                # the safe default: no new command means nero just keeps executing whatever
                # trajectory it already has, not a fresh reach toward an unvalidated solution.
                print(f"[ArmNode] IK failing to converge for {self.ik_fail_streak} consecutive "
                      f"solves (side={side}) -- holding last converged position, NOT sending "
                      f"this target.")
                return
        self.set_joint_target(qd, gripper_target, preview_time)
            
    def set_gripper(self, gripper_pos: float):
        if self.dynamixel_gripper:
            self.gripper.move_to_pos(int(gripper_pos * self.gripper_range + self.close_gripper_value))
        elif self.can_gripper:
            self.can_gripper.set_position(float(gripper_pos))

    def set_gain(self, kp, kd):
        """
        Set Kp and Kd gains for the controller.
        kp, kd can be float (applied to all joints) or 7-element lists/arrays.
        """
        if self.nero:
            gain = Gain()
            if isinstance(kp, (int, float)):
                gain.kp = [float(kp)] * 7
            else:
                gain.kp = [float(x) for x in kp]
            
            if isinstance(kd, (int, float)):
                gain.kd = [float(kd)] * 7
            else:
                gain.kd = [float(x) for x in kd]
            
            self.nero.set_gain(gain)

    def set_gravity_comp(self, enable: bool):
        if self.nero:
            self.nero.enable_gravity_compensation(enable)

    def set_gravity_comp_scale(self, scale: float):
        if self.nero:
            self.nero.set_gravity_comp_scale(scale)

    def set_mode(self, control_mode: ControlMode, move_mode: MoveMode):
        if self.nero:
            self.nero.set_mode(control_mode, move_mode)

    def set_compliant_mode(self, kp=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], kd=[0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]):
        """preset for manual guidance. Default: Stiffness=0, Damping=0.5."""
        self.sync_target()
        self.set_gain(kp, kd)
        self.set_gravity_comp(True)
        self.set_gravity_comp_scale(1.0)

    def set_spring_mode(self, kp=[2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0], kd=[0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3]):
        """preset for springy behavior. Default: Stiffness=2, Damping=0.3."""
        self.sync_target()
        self.set_gain(kp, kd)
        self.set_gravity_comp(True)
        self.set_gravity_comp_scale(1.0)

    def set_firm_mode(self, kp=[15.0, 15.0, 15.0, 15.0, 15.0, 15.0, 15.0], kd=[0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8]):
        """preset for accurate tracking. Default: Stiffness=15, Damping=0.8."""
        self.sync_target()
        self.set_gain(kp, kd)
        self.set_gravity_comp(True)
        self.set_gravity_comp_scale(1.0 )

    def set_admittance_mode(self, kp=[25.0, 25.0, 20.0, 20.0, 15.0, 15.0, 15.0], kd=[1.2, 1.2, 1.0, 1.0, 0.8, 0.8, 0.8]):
        """
        High stiffness and damping, optimized for a high-level admittance loop.
        """
        self.sync_target()
        self.set_gain(kp, kd)
        self.set_gravity_comp(True)
        self.set_gravity_comp_scale(1.0)

    def sync_target(self):
        """Update the controller's target to match the current arm position."""
        if self.nero:
            q = self.get_joint_positions()
            self.nero.set_target(
                new_target_pos=(q + self.q_offset).tolist(),
                new_target_gripper_pos=self.get_gripper_pose(),
                minimum_duration=0.0
            )
            # Give the C++ control loop (500Hz) a few cycles to ingest this 
            # and 'snap' its internal trajectory state before we change gains.
            time.sleep(0.02)

    def get_joint_state(self):
        """Return raw nerolib JointState (pos, vel, torque arrays)."""
        if self.nero is None:
            return None
        return self.nero.get_current_state()

    def get_joint_positions(self) -> np.ndarray:
        if self.nero is None:
             return np.zeros(7)

        try:
            state = self.nero.get_current_state()
            q = np.array(state.pos)
            return q - self.q_offset
        except Exception:
            return np.zeros(7)

    def get_joint_velocities(self) -> np.ndarray:
        if self.nero is None:
             return np.zeros(7)
             
        try:
            state = self.nero.get_current_state()
            return np.array(state.vel)
        except Exception:
            return np.zeros(7)

    def get_joint_torques(self) -> np.ndarray:
        if self.nero is None:
             return np.zeros(7)
             
        try:
            state = self.nero.get_current_state()
            return np.array(state.torque)
        except Exception:
            return np.zeros(7)

    def get_ee_pose(self) -> mink.SE3:
        if self.ik_solver is None:
            print("[ArmNode] get_ee_pose: IK solver not available")
            return None
        self.update_joint_positions()
        return self.ik_solver.forward_kinematics()

    def get_gripper_pose(self):
        if self.dynamixel_gripper:
            unnormalized_pos = self.gripper.dxl.get_present_position()
            return float(unnormalized_pos - self.close_gripper_value) / float(self.gripper_range)
        elif self.can_gripper:
            return self.can_gripper.get_position()
        return 0.0

    def get_gripper_state(self) -> tuple[float | None, float | None]:
        """(measured, commanded) opening in [0, 1]; measured is None when the
        gripper gives no live feedback, so callers never mistake a command for a reading."""
        if self.dynamixel_gripper:
            return self.get_gripper_pose(), self.gripper_target
        if self.can_gripper:
            return self.can_gripper.get_measured(), self.can_gripper.get_commanded()
        return None, None

    def update_joint_positions(self):
        if self.ik_solver is None:
            return
        q = self.get_joint_positions()
        self.ik_solver.update_configuration(q)

    def set_q_offset(self, q_offset: np.ndarray):
        self.q_offset = q_offset

    def stop(self):
        if self.can_gripper:
            try:
                if hasattr(self.can_gripper, "enabled"):  # SameHandGripper
                    self.can_gripper.stop()
                else:
                    self.can_gripper.disable()
            except Exception as e:
                print(f"[ArmNode] gripper disable failed: {e}")
            self.can_gripper = None

        if self.nero:
            try:
                self.nero.stop()  # holds current pos briefly, then disables motors
            except Exception as e:
                print(f"[ArmNode] nero.stop() failed: {e}")
            self.nero = None

if __name__ == "__main__":
    _HERE = Path(__file__).parent.parent
    left_arm = ArmNode(
        can_port="can_left",
        mjcf_path=(_HERE / "yor-description/nero-welded-base-and-lift.mjcf").as_posix(),
        is_left_arm=True,
        dynamixel_gripper=False,
    )
    # right_arm = ArmNode(
    #     can_port="can_right",
    #     mjcf_path=(_HERE / "yor-description/nero-welded-base-and-lift.mjcf").as_posix(),
    #     is_left_arm=False,
    # )
    input("Press enter to init left arm")
    left_arm.init()
    # input("Press enter to init right arm")
    # right_arm.init()
    input("Press enter to close gripper")
    left_arm.close_gripper()
    # right_arm.close_gripper()
    input("Press enter to open gripper")
    left_arm.open_gripper()
    # right_arm.open_gripper()
    input("Press enter to exit")
    left_arm.stop()
    # right_arm.stop()
    exit()
