#!/usr/bin/env python3
"""Direct nerolib test — no gripper/dynamixel deps."""
import sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import nerolib
from nerolib import NeroController, ControllerConfig, ControlMode, MoveMode

_HERE = Path(__file__).parent.parent

def main():
    cfg = ControllerConfig()
    cfg.interface_name = "can_left"
    cfg.gripper_on = False
    cfg.urdf_path = str(_HERE / "nerolib/urdf/nero_cone-e_left_fixed.urdf")
    cfg.gravity_compensation = False
    cfg.controller_freq_hz = 100
    cfg.default_kp = [15.0] * 7
    cfg.default_kd = [0.8] * 7
    cfg.home_position = [1.71, 1.32, -1.71, 1.31, 0.0, 0.0, 0.0]
    cfg.joint_vel_max = [1.5] * 7
    cfg.joint_acc_max = [5.0] * 7
    cfg.over_current_cnt_max = 10

    print("Creating NeroController...")
    ctrl = NeroController(cfg)

    print("Starting controller...")
    ok = ctrl.start()
    if not ok:
        print("ERROR: controller start failed")
        return

    time.sleep(0.5)

    state0 = ctrl.get_current_state()
    pos0 = list(state0.pos)
    print(f"Initial pos: {[round(p,4) for p in pos0]}")

    target = list(pos0)
    target[0] += 0.15
    print(f"Commanding target[0] = {target[0]:.4f} rad (delta +0.15)...")
    ctrl.set_target(target, 1.0, 2.0)  # pos, gripper, min_duration
    time.sleep(4.0)

    state1 = ctrl.get_current_state()
    pos1 = list(state1.pos)
    delta = pos1[0] - pos0[0]
    print(f"After move:  {[round(p,4) for p in pos1]}")
    print(f"joint[0] delta = {delta:.4f} rad")
    if abs(delta) > 0.05:
        print("✓ nerolib JOINT mode movement WORKS!")
    else:
        print("✗ nerolib still not moving")

    print("Stopping...")
    ctrl.stop()

if __name__ == "__main__":
    main()
