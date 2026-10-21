#!/usr/bin/env python3
"""Quick test: can nerolib move the arm via JOINT mode?"""
import sys, time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))
from robot.arm.arm import ArmNode

def main():
    _ROOT = Path(__file__).parent.parent
    arm = ArmNode(
        can_port="can_left",
        mjcf_path=str(_ROOT / "robot/yor-description/nero-welded-base-and-lift.mjcf"),
        is_left_arm=True,
        no_gripper=True,
    )
    print("Starting controller...")
    arm.init()
    time.sleep(0.5)

    pos0 = arm.get_joint_positions()
    print(f"Initial pos: {[round(p,4) for p in pos0]}")

    target = pos0.copy()
    target[0] += 0.15
    print(f"Commanding target[0] = {target[0]:.4f} rad (delta +0.15)...")
    arm.set_joint_target(target, gripper_target=None, preview_time=2.0)
    time.sleep(4.0)

    pos1 = arm.get_joint_positions()
    delta = pos1[0] - pos0[0]
    print(f"After move:  {[round(p,4) for p in pos1]}")
    print(f"joint[0] delta = {delta:.4f} rad")
    if abs(delta) > 0.05:
        print("✓ nerolib movement WORKS!")
    else:
        print("✗ nerolib still not moving")

    print("Homing...")
    arm.home()
    time.sleep(3.0)
    arm.stop()

if __name__ == "__main__":
    main()
