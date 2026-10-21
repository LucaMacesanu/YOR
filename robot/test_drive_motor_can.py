"""
test_drive_motor_can.py — Auto-detect REV SparkFlex motors on CAN and verify Pi control.

Scans IDs 1–8, reports which motors are alive, then commands them via joystick.
Supports two drive modes selectable at startup:

  --mode duty    Duty-cycle (-1.0 … +1.0).  Works with factory defaults.
                 Use this first to confirm the motor physically spins.

  --mode velocity  Velocity PID (RPM).  Requires kF/kP gains set in REV Hardware Client.
                   This is the mode used by the real robot stack.

Controls (Xbox layout):
  Hold LB              — deadman: engage all detected motors
  Left OR right stick  — up = forward, down = reverse
  Hold RB              — boost ×10 (scale / unit test)
  Back                 — quit

Usage:
    python robot/test_drive_motor_can.py                      # duty-cycle, can0
    python robot/test_drive_motor_can.py --mode velocity      # velocity PID
    python robot/test_drive_motor_can.py --can can0 --max-speed 0.3 --scan-max 12

Known CAN IDs:
  Drive    : FL=1  FR=4  RR=3  RL=2
  Rotation : FL=5  FR=8  RR=7  RL=6

NOTE — velocity mode PID setup (REV Hardware Client):
  Connect each motor via USB-C, open REV Hardware Client, go to the Control tab:
    kF  (feed-forward) — start around 0.0002  (most important: this alone makes it spin)
    kP  (proportional) — start around 0.0001
  Without kF the motor will not spin in velocity mode even if commands are received.
"""

import argparse
import atexit
import math
import sys
import time

import pygame
from loop_rate_limiters import RateLimiter

from sparkcan_py import SparkFlex

# --- Enum import with integer fallbacks ---
# REV SparkFlex control type integers (same as Java/C++ API)
#   kDutyCycle = 0   kVelocity = 1   kVoltage = 2   kPosition = 3
try:
    from sparkcan_py import CtrlType
    CTRL_DUTY     = CtrlType.kDutyCycle
    CTRL_VELOCITY = CtrlType.kVelocity
    _CTRL_SOURCE  = "enum"
except Exception:
    CtrlType      = None
    CTRL_DUTY     = 0
    CTRL_VELOCITY = 1
    _CTRL_SOURCE  = "raw int (enum import failed)"

try:
    from sparkcan_py import IdleMode
    IDLE_COAST = IdleMode.kCoast
    _IDLE_SOURCE = "enum"
except Exception:
    IdleMode     = None
    IDLE_COAST   = 0          # kCoast = 0 on REV controllers
    _IDLE_SOURCE = "raw int (enum import failed)"

LOOP_RATE = 50
DEADZONE  = 0.05

XBOX_BTN_LB   = 4
XBOX_BTN_RB   = 5
XBOX_BTN_BACK = 6
XBOX_AXIS_LY  = 1
XBOX_AXIS_RY  = 4

KNOWN_IDS = {
    1: "Drive  FL",
    2: "Drive  RL",
    3: "Drive  RR",
    4: "Drive  FR",
    5: "Rot    FL",
    6: "Rot    RL",
    7: "Rot    RR",
    8: "Rot    FR",
}


def apply_deadzone(v: float, dz: float = DEADZONE) -> float:
    if abs(v) <= dz:
        return 0.0
    return (abs(v) - dz) / (1.0 - dz) * (1.0 if v > 0 else -1.0)


def probe_motor(can_if: str, can_id: int) -> tuple[bool, dict]:
    """
    Return (alive, info_dict).

    Alive detection strategy:
      1. GetBusVoltage() > 5 V  — most reliable: non-existent IDs never receive
         a CAN frame so the binding returns 0.0; real motors return ~24 V.
      2. Fall back to GetVelocity() not-NaN only if GetBusVoltage is unavailable.

    GetVelocity() == 0.0 is NOT sufficient — the binding initialises the cached
    value to 0.0, so absent motors also return 0.0 (false positive).
    """
    info = dict(open=False, velocity=float("nan"), voltage=None, faults=None,
                alive_reason="", error=None)
    try:
        m = SparkFlex(can_if, can_id)
        info["open"] = True
    except Exception as e:
        info["error"] = str(e)
        return False, info

    try:
        m.Heartbeat()
        # Wait long enough for the motor to broadcast at least one status frame.
        # REV SparkFlex status0 period is 10 ms by default; 0.25 s is generous.
        time.sleep(0.25)
        info["velocity"] = float(m.GetVelocity())
    except Exception as e:
        info["error"] = str(e)
        return False, info

    # --- primary alive check: bus voltage ---
    if hasattr(m, "GetBusVoltage"):
        try:
            v = float(m.GetBusVoltage())
            info["voltage"] = v
            if v > 5.0:
                info["alive_reason"] = f"voltage={v:.1f}V"
                alive = True
            else:
                info["alive_reason"] = f"voltage={v:.1f}V (<5 V — not on bus)"
                alive = False
        except Exception as e:
            info["alive_reason"] = f"GetBusVoltage error: {e}"
            alive = False
    else:
        # fallback: NaN check on velocity
        alive = not math.isnan(info["velocity"])
        info["alive_reason"] = "velocity not-NaN (no GetBusVoltage)" if alive else "velocity=NaN"

    if hasattr(m, "GetFaults"):
        try:
            info["faults"] = m.GetFaults()
        except Exception:
            pass

    return alive, info


def configure_motor(motor: SparkFlex, ctrl_int: int) -> tuple[list[str], list[str]]:
    """
    Apply idle mode and control type.
    Returns (ok_lines, warning_lines) for printing.

    REV idle mode integers:  kCoast=0  kBrake=1
    REV ctrl type integers:  kDutyCycle=0  kVelocity=1

    NOTE: SetIdleMode only affects runtime behaviour.  The value burned in flash
    (via REV Hardware Client) takes effect on power-up.  If the motor reverts to
    brake mode after a restart, use REV Hardware Client → Config → Idle Mode = Coast
    → Burn Flash.
    """
    oks, warns = [], []

    # --- Idle mode ---
    try:
        motor.SetIdleMode(IDLE_COAST)
        oks.append(f"SetIdleMode({IDLE_COAST} = kCoast)  OK  "
                   f"[runtime only — flash default may still be kBrake]")
    except Exception as e:
        warns.append(f"SetIdleMode({IDLE_COAST}) FAILED: {e}  "
                     f"→ motor may stay in brake mode")

    # --- Control type ---
    if hasattr(motor, "SetCtrlType"):
        try:
            motor.SetCtrlType(ctrl_int)
            oks.append(f"SetCtrlType({ctrl_int})  OK")
        except Exception as e:
            warns.append(f"SetCtrlType({ctrl_int}) FAILED: {e}")
    else:
        warns.append("SetCtrlType not in binding — control type unchanged")

    # --- Status frame rate ---
    if hasattr(motor, "SetPeriodicStatus2Period"):
        try:
            motor.SetPeriodicStatus2Period(20)
        except Exception:
            pass

    return oks, warns


def send_command(motor: SparkFlex, mode: str, value: float) -> str | None:
    """Send value to motor in the given mode. Returns error string or None."""
    try:
        if mode == "duty":
            motor.SetDutyCycle(float(value)) if hasattr(motor, "SetDutyCycle") \
                else motor.SetVelocity(float(value))
        else:
            motor.SetVelocity(float(value))
        return None
    except Exception as e:
        return str(e)


def sep(char="=", width=66):
    print(char * width)


def main():
    parser = argparse.ArgumentParser(description="REV SparkFlex CAN test from Raspberry Pi")
    parser.add_argument("--can",       default="can0")
    parser.add_argument("--mode",      choices=["duty", "velocity"], default="duty",
                        help="duty = percent output (factory default OK); "
                             "velocity = RPM PID (needs kF configured in REV Hardware Client)")
    parser.add_argument("--max-speed", type=float, default=0.3,
                        help="Max command value at full stick. "
                             "duty: 0.0–1.0 (default 0.3 = 30%%). "
                             "velocity: RPM (default 0.3 — you probably want 100–500 RPM)")
    parser.add_argument("--scan-max",  type=int, default=8,
                        help="Scan CAN IDs 1..N (default 8)")
    args = parser.parse_args()

    ctrl_int  = CTRL_VELOCITY if args.mode == "velocity" else CTRL_DUTY
    ctrl_name = "VELOCITY (RPM PID)" if args.mode == "velocity" else "DUTY CYCLE (% output)"

    # ------------------------------------------------------------------ #
    #  ENUM / BINDING REPORT                                               #
    # ------------------------------------------------------------------ #
    sep()
    print("  SPARKCAN_PY BINDING CHECK")
    sep()
    print(f"  CtrlType source  : {_CTRL_SOURCE}")
    print(f"  IdleMode source  : {_IDLE_SOURCE}")
    print(f"  Drive mode       : {ctrl_name}  (ctrl_int={ctrl_int})")
    if args.mode == "velocity":
        print()
        print("  *** VELOCITY MODE — motor will NOT spin unless PID gains are set ***")
        print("  Set kF (feed-forward) in REV Hardware Client via USB-C.")
        print("  Suggested starting values:  kF=0.0002  kP=0.0001")
        print("  Without kF the output is 0 even if commands arrive correctly.")

    # ------------------------------------------------------------------ #
    #  SCAN                                                                #
    # ------------------------------------------------------------------ #
    sep()
    print(f"  CAN SCAN — interface: {args.can}   IDs 1..{args.scan_max}")
    sep()
    print(f"  {'ID':>3}  {'Label':<12}  {'Alive':^6}  "
          f"{'Voltage':>8}  {'Velocity':>9}  {'Faults':>6}  Reason / Notes")
    print("  " + "-" * 76)

    alive_ids: list[int] = []

    for can_id in range(1, args.scan_max + 1):
        label = KNOWN_IDS.get(can_id, f"ID {can_id:02d}   ")
        alive, r = probe_motor(args.can, can_id)

        alive_s = "  YES " if alive else ("   NO " if r["open"] else "  --- ")
        vel_s   = f"{r['velocity']:+.3f}" if not math.isnan(r["velocity"]) else "    NaN"
        volt_s  = f"{r['voltage']:.1f}V"  if r["voltage"] is not None else "    ---"
        flt_s   = str(r["faults"])        if r["faults"]  is not None else "   ---"
        note    = r["alive_reason"] or r["error"] or ""
        if alive and r["faults"] and r["faults"] != 0:
            note = f"FAULT={r['faults']}  " + note

        print(f"  {can_id:>3}  {label:<12}  {alive_s}  "
              f"{volt_s:>8}  {vel_s:>9}  {flt_s:>6}  {note}")

        if alive:
            alive_ids.append(can_id)

    sep("-")
    if not alive_ids:
        print(f"  RESULT: NO motors detected on {args.can}")
        print()
        print("  Troubleshooting:")
        print("    1. Are the motors powered?")
        print("    2. CAN_H/CAN_L correctly wired? (not swapped)")
        print("    3. 120Ω termination resistor at BOTH ends of the CAN chain?")
        print("    4. CAN interface up?  Run:  ip link show | grep can")
        print("       If down:  sudo ip link set can0 up type can bitrate 1000000")
        sys.exit(1)
    else:
        print(f"  RESULT: {len(alive_ids)} motor(s) alive — IDs: {alive_ids}")

    # ------------------------------------------------------------------ #
    #  CONFIGURE                                                           #
    # ------------------------------------------------------------------ #
    sep()
    print(f"  CONFIGURING — mode: {ctrl_name}")
    sep()

    motors: dict[int, SparkFlex] = {}
    all_warnings: list[str] = []

    for can_id in alive_ids:
        label = KNOWN_IDS.get(can_id, f"ID {can_id}")
        try:
            m = SparkFlex(args.can, can_id)
            warns = configure_motor(m, ctrl_int)
            motors[can_id] = m
            status = "OK" if not warns else f"{len(warns)} warning(s)"
            print(f"  ID {can_id:>2}  {label:<12}  {status}")
            for w in warns:
                print(f"              ! {w}")
                all_warnings.append(f"ID {can_id}: {w}")
        except Exception as e:
            print(f"  ID {can_id:>2}  {label:<12}  FAILED: {e}")

    if not motors:
        print("  ERROR: could not open any motor. Exiting.")
        sys.exit(1)

    def stop_all():
        for m in motors.values():
            try:
                send_command(m, args.mode, 0.0)
            except Exception:
                pass

    atexit.register(stop_all)

    # ------------------------------------------------------------------ #
    #  JOYSTICK                                                            #
    # ------------------------------------------------------------------ #
    sep()
    print("  JOYSTICK")
    sep()
    pygame.init()
    if pygame.joystick.get_count() < 1:
        print("  ERROR: no joystick detected")
        sys.exit(1)
    joy = pygame.joystick.Joystick(0)
    print(f"  Controller : {joy.get_name()}")
    print(f"  Axes: {joy.get_numaxes()}   Buttons: {joy.get_numbuttons()}")
    atexit.register(pygame.quit)

    # ------------------------------------------------------------------ #
    #  CONTROL LOOP                                                        #
    # ------------------------------------------------------------------ #
    sep()
    print(f"  RUNNING — {len(motors)} motor(s)  mode={args.mode}  max={args.max_speed}")
    sep()
    print("  Hold LB              — engage motors (deadman)")
    print("  Left OR right stick  — up = forward, down = reverse")
    print("  Hold RB + LB         — boost ×10")
    print("  Back                 — quit")
    if args.mode == "velocity" and any("SetCtrlType" in w or "kF" in w for w in all_warnings):
        print()
        print("  *** WARNING: velocity mode may not be active — see warnings above ***")
        print("  *** Run with --mode duty first to confirm the motor can spin       ***")
    sep("-")

    header_line = (f"  {'ID':>3}  {'Label':<12}  {'Cmd':>8}  {'GetVel':>8}  "
                   f"{'Alive':^5}  Notes")
    HEADER_EVERY = 15
    block_n = 0
    rate = RateLimiter(LOOP_RATE, name="can-test")
    last_print = 0.0

    while True:
        pygame.event.pump()

        if joy.get_button(XBOX_BTN_BACK):
            print("\n  Back — exiting.")
            break

        deadman = bool(joy.get_button(XBOX_BTN_LB))
        boost   = bool(joy.get_button(XBOX_BTN_RB))
        raw_ly  = float(joy.get_axis(XBOX_AXIS_LY))
        raw_ry  = float(joy.get_axis(XBOX_AXIS_RY))
        raw_y   = raw_ly if abs(raw_ly) >= abs(raw_ry) else raw_ry
        dz_y    = apply_deadzone(raw_y)

        cmd = 0.0
        if deadman:
            cmd = -dz_y * args.max_speed
            if boost:
                cmd *= 10.0

        errs: dict[int, str] = {}
        for can_id, m in motors.items():
            m.Heartbeat()
            err = send_command(m, args.mode, cmd)
            if err:
                errs[can_id] = err

        now = time.monotonic()
        if now - last_print >= 0.25:
            dm_s = f"LB={'ON ' if deadman else 'OFF'}  RB={'ON' if boost else 'OFF'}"
            print(f"\n  stick={raw_y:+.3f}  dz={dz_y:+.3f}  cmd={cmd:+.5f}  [{dm_s}]  mode={args.mode}")

            if block_n % HEADER_EVERY == 0:
                print(header_line)
                print("  " + "-" * 58)
            block_n += 1

            for can_id, m in motors.items():
                label = KNOWN_IDS.get(can_id, f"ID {can_id:02d}")
                try:
                    vel = float(m.GetVelocity())
                    alive = not math.isnan(vel)
                    vel_s = f"{vel:+.4f}"
                except Exception:
                    alive = False
                    vel_s = "   ERR"

                note = ""
                if can_id in errs:
                    note = f"send ERR: {errs[can_id]}"
                elif args.mode == "velocity" and cmd != 0.0 and alive and abs(vel) < 0.5:
                    note = "vel~0 despite cmd — set kF in REV Hardware Client"
                elif args.mode == "duty" and cmd != 0.0 and alive and abs(vel) < 0.5:
                    note = "vel~0 — motor may be stalled or brake mode active"
                elif boost:
                    note = "BOOST x10"

                alive_s = "YES" if alive else "NO "
                print(f"  {can_id:>3}  {label:<12}  {cmd:>8.4f}  {vel_s:>8}  "
                      f"{alive_s:^5}  {note}")

            last_print = now

        rate.sleep()

    print()
    stop_all()


if __name__ == "__main__":
    main()
