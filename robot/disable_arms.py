"""
disable_arms.py — Make both arms go limp, then stop the CAN controller.

Run this standalone to test that arms properly go compliant on shutdown.
Does NOT call init() (which would home the arms first).
Each arm is handled independently — a failure on one does not block the other.
"""
import struct
import time
import sys
from pathlib import Path

import can

_HERE = Path(__file__).parent
_ROOT = _HERE.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from nerolib import NeroController, ControllerConfig, Gain

GRIPPER_CMD_ID = 0x159


def make_nero(can_port: str, urdf_path: str) -> NeroController | None:
    try:
        config = ControllerConfig()
        config.interface_name = can_port
        config.urdf_path = urdf_path
        config.joint_vel_max = [3.0] * 7
        config.joint_acc_max = [15.0] * 7
        config.default_kp = [15.0] * 7
        config.default_kd = [0.8] * 7
        config.gravity_compensation = False
        config.gripper_on = False
        nero = NeroController(config)
        if not nero.start():
            print(f"[{can_port}] NeroController.start() failed")
            return None
        return nero
    except Exception as e:
        print(f"[{can_port}] Failed to connect: {e}")
        return None


def go_limp(nero: NeroController, label: str):
    try:
        print(f"[{label}] Setting kp=0, kd=0 (limp)...")
        gain = Gain()
        gain.kp = [0.0] * 7
        gain.kd = [0.0] * 7
        nero.set_gain(gain)
        nero.enable_gravity_compensation(False)
        time.sleep(0.1)
    except Exception as e:
        print(f"[{label}] go_limp failed: {e}")


def disable_gripper(can_port: str, label: str):
    print(f"[{label}] Sending gripper disable frames...")
    try:
        bus = can.interface.Bus(channel=can_port, interface="socketcan")
        data = struct.pack(">iH", 0, 0) + bytes([0x00, 0x00])  # pos=0, effort=0, status=0
        msg = can.Message(arbitration_id=GRIPPER_CMD_ID, data=data, is_extended_id=False)
        for _ in range(10):
            bus.send(msg)
            time.sleep(0.005)
        bus.shutdown()
        print(f"[{label}] Gripper disabled.")
    except Exception as e:
        print(f"[{label}] Gripper disable failed: {e}")


def disable_arm(can_port: str, urdf_path: str, label: str):
    nero = make_nero(can_port, urdf_path)

    if nero is not None:
        go_limp(nero, label)
    else:
        print(f"[{label}] Skipping go_limp (no nerolib connection).")

    disable_gripper(can_port, f"{label} GRIPPER")

    if nero is not None:
        try:
            nero.stop()
            print(f"[{label}] CAN controller stopped.")
        except Exception as e:
            print(f"[{label}] nero.stop() failed: {e}")


def main():
    urdf_left  = (_ROOT / "nerolib/urdf/nero_cone-e_left_fixed.urdf").as_posix()
    urdf_right = (_ROOT / "nerolib/urdf/right_arm_final.urdf").as_posix()

    input("Press ENTER to go LIMP and disable both arms...")

    disable_arm("can_left",  urdf_left,  "LEFT")
    disable_arm("can_right", urdf_right, "RIGHT")

    print("[disable_arms] Done.")


if __name__ == "__main__":
    main()
