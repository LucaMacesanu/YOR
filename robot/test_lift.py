"""
Standalone hardware test for the Pico serial lift (robot.base_motor.PicoLift).

Exercises the same PicoLift class the Base uses, without the base, arms or RPC server.

Stages:
  1. connect   - open the serial port, query `status`, parse a height reading   (no motion)
  2. up        - jog up for --pulse seconds, verify height increased            (--move)
  3. stop      - verify height holds after `stop`                               (--move)
  4. down      - jog down for --pulse seconds, verify height decreased          (--move)

Usage (from repo root):
  python robot/test_lift.py                  # stage 1 only, lift does not move
  python robot/test_lift.py --move           # full test, asks before moving
  python robot/test_lift.py --move --pulse 1.0 --yes

Make sure nothing else (robot/yor.py with YOR_LIFT=1) has the port open.
"""
import argparse
import sys
import time
from pathlib import Path
from typing import Optional

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from robot.base_motor import PicoLift  # noqa: E402

POLL_HZ = 5
MIN_TRAVEL_MM = 1.0  # a jog must move at least this much to count as motion
HOLD_TOL_MM = 1.0  # max drift allowed after `stop`


def read_height_mm(lift: PicoLift, timeout: float = 1.0) -> Optional[float]:
    """Ask the Pico for `status` and wait for a fresh parsed height."""
    with lift._height_lock:
        lift._height_m = None
    lift._send("status")
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        h = lift.get_height()
        if h is not None:
            return h * 1000.0
        time.sleep(0.02)
    return None


def jog(lift: PicoLift, direction: str, duration: float) -> list[float]:
    """Drive up/down for `duration` seconds, then stop. Returns heights sampled during the jog."""
    samples = []
    (lift.up if direction == "up" else lift.down)()
    t_end = time.monotonic() + duration
    try:
        while time.monotonic() < t_end:
            h = read_height_mm(lift, timeout=1.0 / POLL_HZ)
            if h is not None:
                samples.append(h)
    finally:
        lift.stop()
    return samples


def check(name: str, ok: bool, msg: str, results: list) -> bool:
    print(f"\n[{'PASS' if ok else 'FAIL'}] {name}: {msg}")
    results.append((name, ok))
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--move", action="store_true", help="run the motion stages (lift WILL move)")
    ap.add_argument("--pulse", type=float, default=0.5, help="seconds per up/down jog (default 0.5)")
    ap.add_argument("--yes", action="store_true", help="don't ask for confirmation before moving")
    args = ap.parse_args()

    results: list = []
    lift = PicoLift(device_path=args.port, baud=args.baud)
    time.sleep(0.5)  # let the Pico print its banner after the port opens

    try:
        # ---- 1. connect ----
        if not check("connect", lift._ser is not None, f"serial {args.port}", results):
            return 1
        h0 = read_height_mm(lift)
        if not check("telemetry", h0 is not None,
                     f"height = {h0:.2f} mm" if h0 is not None else "no 'Height: ... mm' reply to `status`",
                     results):
            return 1

        if not args.move:
            print("\nNo-motion checks passed. Re-run with --move to jog the lift.")
            return 0

        if not args.yes:
            print(f"\nAbout to jog the lift UP then DOWN for {args.pulse:.2f} s each.")
            print("Keep clear of the lift and have the e-stop / power ready.")
            if input("Type 'y' to continue: ").strip().lower() != "y":
                print("Aborted.")
                return 1

        # ---- 2. up ----
        start = read_height_mm(lift)
        samples = jog(lift, "up", args.pulse)
        time.sleep(0.3)
        after_up = read_height_mm(lift)
        print(f"  up samples (mm): {[round(s, 2) for s in samples]}")
        if start is None or after_up is None:
            check("up", False, "lost height telemetry", results)
            return 1
        d_up = after_up - start
        if d_up <= -MIN_TRAVEL_MM:
            check("up", False, f"moved {d_up:+.2f} mm: 'up' moves DOWN (motor or encoder wiring reversed?)", results)
        else:
            check("up", d_up >= MIN_TRAVEL_MM,
                  f"{start:.2f} -> {after_up:.2f} mm ({d_up:+.2f})"
                  + ("" if d_up >= MIN_TRAVEL_MM else ": no motion (driver power, wiring, upper limit?)"),
                  results)

        # ---- 3. stop / hold ----
        held = []
        t_end = time.monotonic() + 1.0
        while time.monotonic() < t_end:
            h = read_height_mm(lift, timeout=1.0 / POLL_HZ)
            if h is not None:
                held.append(h)
        drift = (max(held) - min(held)) if held else float("nan")
        check("stop", bool(held) and drift <= HOLD_TOL_MM,
              f"drift over 1 s after stop = {drift:.2f} mm (tol {HOLD_TOL_MM} mm)", results)

        # ---- 4. down ----
        start = read_height_mm(lift)
        samples = jog(lift, "down", args.pulse)
        time.sleep(0.3)
        after_down = read_height_mm(lift)
        print(f"  down samples (mm): {[round(s, 2) for s in samples]}")
        if start is None or after_down is None:
            check("down", False, "lost height telemetry", results)
            return 1
        d_down = after_down - start
        check("down", d_down <= -MIN_TRAVEL_MM,
              f"{start:.2f} -> {after_down:.2f} mm ({d_down:+.2f})"
              + ("" if d_down <= -MIN_TRAVEL_MM else ": did not move down (lower limit, wiring?)"),
              results)

    except KeyboardInterrupt:
        print("\nInterrupted.")
        results.append(("interrupted", False))
    finally:
        lift.stop()
        lift._drain_stop.set()

    print("\n==== Summary ====")
    for name, ok in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    return 0 if all(ok for _, ok in results) else 1


if __name__ == "__main__":
    sys.exit(main())
