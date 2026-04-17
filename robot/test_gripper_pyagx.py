"""
Test gripper using pyAgxArm high-level API.
Usage: python robot/test_gripper_pyagx.py --can can_right
"""
import argparse
import time
from pyAgxArm import AgxArmFactory, create_agx_arm_config

parser = argparse.ArgumentParser()
parser.add_argument('--can', default='can_right')
args = parser.parse_args()

print(f"Connecting to Nero on {args.can}...")
robot_cfg = create_agx_arm_config(robot="nero", interface="socketcan", channel=args.can)
robot = AgxArmFactory.create_arm(robot_cfg)
robot.connect()

print("Enabling motors and waiting for arm to stabilize...")
for _ in range(5):
    robot.enable()
    time.sleep(0.5)

print("Initializing AGX gripper effector...")
effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)

print("Waiting 2s for gripper feedback...")
time.sleep(2.0)

gs = effector.get_gripper_status()
if gs is None:
    print("Gripper status: None (gripper not responding on 0x2A8)")
else:
    print(f"Gripper status: value={gs.msg.value:.4f} force={gs.msg.force:.3f} "
          f"enabled={gs.msg.foc_status.driver_enable_status} mode={gs.msg.mode}")

print("\nCommands: o=open  c=close  s=status  q=quit")
while True:
    key = input("> ").strip().lower()
    if key == 'o':
        effector.move_gripper_m(value=0.07, force=5.0)
        print("Sent: open (0.07m)")
    elif key == 'c':
        effector.move_gripper_m(value=0.0, force=5.0)
        print("Sent: close (0.0m)")
    elif key == 's':
        gs = effector.get_gripper_status()
        if gs is None:
            print("Gripper status: None")
        else:
            print(f"value={gs.msg.value:.4f}m  force={gs.msg.force:.3f}N  "
                  f"enabled={gs.msg.foc_status.driver_enable_status}  "
                  f"homed={gs.msg.foc_status.homing_status}  mode={gs.msg.mode}")
    elif key == 'q':
        break

robot.disconnect()
