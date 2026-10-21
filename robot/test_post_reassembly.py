"""Non-destructive health check after the 2026-10 reassembly
(custom right gripper, left gripper removed, lift re-added).

    python robot/test_post_reassembly.py            # probe arm buses (motors stay disabled)
    python robot/test_post_reassembly.py --passive  # listen only, send nothing

What it checks:
  1. Live CAN interface names vs. the serials in /etc/udev/rules.d/80-can-yor.rules
     (an installed rule only takes effect after a reboot or replug).
  2. Which arm bus actually has the Nero. Unless --passive, it sends the same
     frames as nerolib's set_normal_mode() (0x470 + 0x151 move_mode=0xFF,
     enable_can_push=1). That turns on feedback streaming only; it never sends
     the 0x471 enable, so the motors are not powered.
  3. Arm status (0x2A1), driver faults (0x261-0x267) and what the gripper
     feedback frame (0x2A8) says now that the AgileX gripper is gone.
  4. Lift config vs. the Pico serial device.

Do not run while robot/yor.py is running.
"""

import argparse
import os
import re
import struct
import sys
import time
from collections import Counter
from pathlib import Path

import can

UDEV_RULE = Path("/etc/udev/rules.d/80-can-yor.rules")
ARM_IFACES = ("can_left", "can_right")
ARM_STATUS = {
    0: "NORMAL", 1: "EMERGENCY_STOP", 2: "NO_SOLUTION", 3: "SINGULARITY",
    4: "TARGET_ANGLE_EXCEEDS_LIMIT", 5: "JOINT_COMMUNICATION_EXCEPTION",
    6: "JOINT_BRAKE_NOT_RELEASED", 7: "COLLISION", 9: "JOINT_STATUS_ABNORMAL",
    10: "OTHER_EXCEPTION",
}

problems = []


def ok(msg):
    print(f"  [ok]   {msg}")


def bad(msg):
    print(f"  [FAIL] {msg}")
    problems.append(msg)


def note(msg):
    print(f"  [note] {msg}")


def iface_serial(iface):
    try:
        return (Path(f"/sys/class/net/{iface}/device/..") / "serial").resolve().read_text().strip()
    except OSError:
        return None


def check_naming():
    print("\n1. CAN interface naming")
    if not UDEV_RULE.exists():
        bad(f"{UDEV_RULE} missing")
        return
    want = dict((name, serial) for serial, name in
                re.findall(r'ATTRS\{serial\}=="(\w+)", NAME="(\w+)"', UDEV_RULE.read_text()))
    for iface in ARM_IFACES:
        live = iface_serial(iface)
        if live is None:
            note(f"{iface}: interface not present")
        elif live == want.get(iface):
            ok(f"{iface} -> adapter {live} (matches udev rule)")
        else:
            bad(f"{iface} is adapter {live} but the udev rule says {want.get(iface)}. "
                "Rule not applied yet: reboot or replug the USB-CAN adapters.")


def listen(iface, seconds, probe):
    try:
        bus = can.interface.Bus(channel=iface, interface="socketcan")
    except OSError as e:
        return None, f"cannot open ({e})"
    frames = {}
    counts = Counter()
    try:
        if probe:
            bus.send(can.Message(arbitration_id=0x470, data=bytes(8), is_extended_id=False))
            time.sleep(0.05)
            bus.send(can.Message(arbitration_id=0x151,
                                 data=bytes([0x01, 0xFF, 50, 0, 0, 0, 0x01, 0]),
                                 is_extended_id=False))
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            msg = bus.recv(timeout=0.1)
            if msg is None or msg.is_error_frame:
                continue
            counts[msg.arbitration_id] += 1
            frames[msg.arbitration_id] = bytes(msg.data)
    except can.CanError as e:
        return None, f"CAN error ({e})"
    finally:
        bus.shutdown()
    return (counts, frames), None


def check_buses(probe):
    print(f"\n2. Arm buses ({'probe: feedback only, motors stay off' if probe else 'passive listen'})")
    arm_on = []
    for iface in ARM_IFACES:
        res, err = listen(iface, 1.5, probe)
        if err:
            note(f"{iface}: {err}")
            continue
        counts, frames = res
        joint_frames = sum(counts[i] for i in range(0x251, 0x258))
        other = sorted(i for i in counts if not (0x251 <= i <= 0x257 or 0x261 <= i <= 0x267
                                                  or 0x2A1 <= i <= 0x2A9))
        if joint_frames:
            arm_on.append(iface)
            q = [struct.unpack(">i", frames[0x251 + j][4:8])[0] / 1000.0
                 for j in range(7) if 0x251 + j in frames]
            print(f"  {iface}: Nero arm streaming ({joint_frames} joint frames), "
                  f"q[rad]={[round(x, 3) for x in q]}")
            report_arm_state(iface, frames)
        else:
            print(f"  {iface}: no Nero feedback ({sum(counts.values())} frames total)")
        if other:
            note(f"{iface}: non-Nero CAN IDs seen: {[hex(i) for i in other]}")
    print()
    if arm_on == ["can_left", "can_right"]:
        ok("arms found on both can_left and can_right")
    elif arm_on:
        note(f"arm found only on {arm_on[0]}. Is the other arm powered and its CAN cable plugged in?")
    else:
        bad("no arm found on either bus" + ("" if probe else " (passive mode: an idle arm "
            "may not stream; rerun without --passive)"))
    note("this check can't tell which arm is physically left/right. To verify, unplug the "
         "right arm's CAN cable and rerun: can_right should go quiet.")


def report_arm_state(iface, frames):
    if 0x2A1 in frames:
        d = frames[0x2A1]
        print(f"    0x2A1 ctrl_mode={d[0]} arm_status={d[1]} "
              f"({ARM_STATUS.get(d[1], '?')}) move_mode={d[2]}")
        if d[1] not in (0, 6):  # 6 = brakes engaged, normal while disabled
            bad(f"{iface}: arm status {ARM_STATUS.get(d[1], d[1])}")
    faults = []
    for j in range(7):
        d = frames.get(0x261 + j)
        if d is None:
            continue
        b = d[5]
        names = [n for bit, n in ((1, "overcurrent"), (2, "overheat"), (3, "collision"),
                                   (4, "driver_error"), (7, "stall")) if b >> bit & 1]
        if names:
            faults.append(f"j{j + 1}:{'/'.join(names)}")
    (bad if faults else ok)(f"{iface}: driver faults: {faults or 'none'}")
    if 0x2A8 in frames:
        d = frames[0x2A8]
        width_um = struct.unpack(">i", d[0:4])[0]
        force = struct.unpack(">h", d[4:6])[0]
        note(f"{iface}: 0x2A8 (AgileX gripper feedback) still present: width={width_um} um "
             f"force={force} status=0x{d[6]:02x} mode=0x{d[7]:02x}. With the AgileX gripper "
             "removed this is not a real reading, but CanGripper will report it as one.")
    else:
        note(f"{iface}: no 0x2A8 gripper feedback. CanGripper.get_measured() will return None.")


def check_lift():
    print("\n3. Lift")
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    try:
        from robot.config import LIFT_ENABLED
    except Exception as e:
        note(f"could not import robot.config ({e})")
        return
    pico = sorted(Path("/dev/serial/by-id").glob("*MicroPython*")) if Path("/dev/serial/by-id").exists() else []
    if pico:
        ok(f"Pico present: {pico[0].name} -> {os.path.realpath(pico[0])}")
    else:
        bad("no MicroPython (Pico lift) serial device found")
    if LIFT_ENABLED:
        ok("LIFT_ENABLED=True")
    else:
        bad("LIFT_ENABLED is False (YOR_LIFT=0 is set), so yor.py will make every lift call a no-op.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--passive", action="store_true", help="listen only, send nothing")
    args = ap.parse_args()
    check_naming()
    check_buses(probe=not args.passive)
    check_lift()
    print(f"\n{len(problems)} problem(s) found." if problems else "\nAll checks passed.")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
