"""
Home the lift through the running robot/yor.py, so lift_height becomes absolute:
carriage travel in metres above the bottom hard stop (0 = fully down, up is +).

The lift goes up ~13 mm, then down until it stalls at the bottom stop (up to ~32 s).
Clear the space below the arms first. Ctrl+C stops the lift.

Usage (from repo root, with robot/yor.py running):
  python robot/lift_home.py            # home, then print the reading
  python robot/lift_home.py --status   # just print height / homed / ref
"""
import argparse
import time

from commlink import RPCClient


def show(yor):
    st = yor.get_lift_status()
    h = yor.get_lift_height()
    print(f"lift_height = {'None' if h is None else f'{h:.4f} m'}  homed={st['homed']}  ref={st['ref']}"
          + (f"  last home error: {st['home_error']}" if st["home_error"] else ""))
    return st


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=5557)
    ap.add_argument("--status", action="store_true", help="print the lift status and exit (no motion)")
    args = ap.parse_args()

    yor = RPCClient(args.host, args.port)
    if args.status:
        show(yor)
        return 0
    if not yor.lift_home():
        print("A lift_home is already running")
    print("Homing (Ctrl+C stops the lift)...")
    try:
        time.sleep(0.2)
        while yor.get_lift_status()["homing"]:
            time.sleep(0.2)
    except KeyboardInterrupt:
        yor.lift_stop()
        print("\nStopped")
        return 1
    return 0 if show(yor)["homed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
