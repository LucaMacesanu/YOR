"""Neck servo setup tool. Run each step in a real terminal (it waits for Enter).

    python robot/neck/neck_setup.py scan        # find the adapter, servo IDs, baud rate
    python robot/neck/neck_setup.py identify    # torque OFF: move ONE joint for 10 s, see which ID moved
    python robot/neck/neck_setup.py calibrate   # torque OFF: centre, then sweep each joint's range
    python robot/neck/neck_setup.py set-home    # torque OFF: pose the neck by hand, save as home
    python robot/neck/neck_setup.py read        # stream angles (torque OFF)
    python robot/neck/neck_setup.py home        # torque ON: move gently to the saved home pose

Calibration writes each servo's homing offset and position limits to the servo
itself and saves them, with the home pose, to robot/neck/neck_calibration.json.
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from lerobot.motors.feetech import FeetechMotorsBus  # noqa: E402

from robot.neck.neck import (  # noqa: E402
    CALIB_PATH, MOTOR_IDS, MOTOR_MODEL, Neck, find_port, load_calibration, save_calibration,
)


def cmd_scan(args):
    port = args.port or find_port()
    print(f"Scanning {port} at every baud rate...")
    found = FeetechMotorsBus.scan_port(port)
    if not found:
        print("No servos answered. Check servo power and the adapter's bus wiring.")
        return 1
    for baud, ids in found.items():
        print(f"  baud {baud}: IDs {ids}")
    print(f"Driver expects IDs {MOTOR_IDS} ({MOTOR_MODEL}) at 1000000 baud.")
    return 0


def cmd_identify(args):
    """Works before calibration: reads raw encoder counts by servo ID."""
    from lerobot.motors import Motor, MotorNormMode
    port = args.port or find_port()
    bus = FeetechMotorsBus(port, {f"id{i}": Motor(i, MOTOR_MODEL, MotorNormMode.DEGREES)
                                  for i in MOTOR_IDS.values()})
    bus.connect()
    try:
        bus.disable_torque()
        names = list(bus.motors)
        start = bus.sync_read("Present_Position", names, normalize=False)
        lo, hi = dict(start), dict(start)
        print(f"Torque OFF. Move ONLY the pan joint back and forth for {args.seconds:.0f} s...")
        t0 = time.monotonic()
        while time.monotonic() - t0 < args.seconds:
            pos = bus.sync_read("Present_Position", names, normalize=False)
            for n in names:
                lo[n], hi[n] = min(lo[n], pos[n]), max(hi[n], pos[n])
            time.sleep(0.02)
        moved = {n: hi[n] - lo[n] for n in names}
        for n in names:
            print(f"  {n}: moved {moved[n]} counts ({moved[n] * 360 / 4095:.1f} deg)")
        top = max(moved, key=moved.get)
        if moved[top] < 50:
            print("Nothing moved noticeably. Try again.")
        else:
            pan_id = bus.motors[top].id
            tilt_id = next(i for i in MOTOR_IDS.values() if i != pan_id)
            print(f"=> pan is ID {pan_id}, tilt is ID {tilt_id}. Driver currently has {MOTOR_IDS}.")
    finally:
        bus.disconnect(disable_torque=True)
    return 0


def cmd_calibrate(args):
    neck = Neck(port=args.port)
    neck.bus.calibration = {}
    neck.connect()
    bus = neck.bus
    _, home = load_calibration()
    try:
        bus.disable_torque()
        input("\nTorque is OFF. Move the neck by hand to the MIDDLE of its travel "
              "(both pan and tilt), then press Enter...")
        offsets = bus.set_half_turn_homings()
        print(f"Homing offsets: {offsets}")
        print("\nNow move EACH joint slowly through its full range, end to end. "
              "Press Enter when done.")
        mins, maxes = bus.record_ranges_of_motion()
        calib = {}
        from lerobot.motors import MotorCalibration
        for name, m in bus.motors.items():
            calib[name] = MotorCalibration(id=m.id, drive_mode=0, homing_offset=offsets[name],
                                           range_min=mins[name], range_max=maxes[name])
        bus.write_calibration(calib)
        save_calibration(calib, None)
        print(f"\nSaved calibration to {CALIB_PATH}")
        for name, c in calib.items():
            span = (c.range_max - c.range_min) * 360 / 4095
            print(f"  {name}: range {c.range_min}..{c.range_max} ({span:.1f} deg)")
        if home:
            print("Note: the old home pose was cleared, since it was relative to the old calibration. "
                  "Run set-home again.")
    finally:
        neck.close(disable_torque=True)
    return 0


def cmd_set_home(args):
    neck = Neck(port=args.port)
    neck._require_calibration()
    neck.connect()
    try:
        neck.torque(False)
        input("\nTorque is OFF. Move the neck by hand to the HOME pose, then press Enter...")
        home = {k: round(v, 2) for k, v in neck.get_position_deg().items()}
        save_calibration(neck.bus.calibration, home)
        print(f"Saved home pose {home} to {CALIB_PATH}")
    finally:
        neck.close(disable_torque=True)
    return 0


def cmd_read(args):
    neck = Neck(port=args.port)
    neck.connect()
    try:
        neck.torque(False)
        print(f"Limits (deg): {neck.get_limits_deg()}   home: {neck.home_deg}")
        print("Streaming angles; Ctrl-C to stop.")
        while True:
            pos = neck.get_position_deg()
            print("\r" + "  ".join(f"{k}={v:7.2f}" for k, v in pos.items()), end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()
    finally:
        neck.close(disable_torque=True)
    return 0


def cmd_home(args):
    neck = Neck(port=args.port)
    neck.connect()
    try:
        ok = neck.go_home()
        print(f"Home {'reached' if ok else 'NOT reached within timeout'}: {neck.get_position_deg()}")
        if args.hold:
            input("Holding home with torque ON. Press Enter to release...")
    finally:
        neck.close(disable_torque=True)
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (default: NECK_PORT or auto-detect)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("scan")
    i = sub.add_parser("identify")
    i.add_argument("--seconds", type=float, default=10.0)
    sub.add_parser("calibrate")
    sub.add_parser("set-home")
    sub.add_parser("read")
    h = sub.add_parser("home")
    h.add_argument("--hold", action="store_true", help="keep torque on until Enter")
    args = ap.parse_args()
    return {"scan": cmd_scan, "identify": cmd_identify, "calibrate": cmd_calibrate, "set-home": cmd_set_home,
            "read": cmd_read, "home": cmd_home}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
