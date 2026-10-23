"""
Checks for the simulated robot (robot/yor_mujoco.py), no hardware needed:

    python robot/test_yor_mujoco.py          # drives YORMujoco directly, headless
    python robot/test_yor_mujoco.py --gripper agx   # same, with the stock AgileX grippers
    python robot/test_yor_mujoco.py --rpc    # same checks through commlink, against a
                                             # running `python robot/yor_mujoco.py`

Each check commands one thing and asserts the robot moved the right way, in the
robot frame (x forward, y left, z up).
"""
import argparse
import sys
import time
from pathlib import Path

import mink
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


def wait(s):
    time.sleep(s)


def check(name, ok, detail=""):
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}")
    return ok


def run(yor):
    results = []
    wait(1.0)  # let the arms settle under gravity before measuring start poses

    # --- arms: EE target 5 cm forward / up, measured pose follows
    for side in ("left", "right"):
        get = getattr(yor, f"get_{side}_ee_pose")
        set_ = getattr(yor, f"set_{side}_ee_target")
        start = get()
        for axis, name in ((0, "forward"), (2, "up")):
            target = start.wxyz_xyz.copy()
            target[4 + axis] += 0.05
            for _ in range(30):  # stream like a 30 Hz teleop client
                set_(ee_target=mink.SE3(target), gripper_target=None, preview_time=2 / 15)
                wait(1 / 30)
            wait(0.5)
            moved = get().translation() - start.translation()
            results.append(check(f"{side} arm {name} 5 cm", abs(moved[axis] - 0.05) < 0.01 and
                                 np.linalg.norm(np.delete(moved, axis)) < 0.01, f"moved {np.round(moved, 4)}"))
            getattr(yor, f"home_{side}_arm")()
            wait(0.5)
            back = np.linalg.norm(get().translation() - start.translation())
            results.append(check(f"{side} arm home after {name}", back < 0.01, f"err {back:.4f} m"))

    # --- grippers
    for side in ("left", "right"):
        getattr(yor, f"close_{side}_gripper")()
        wait(1.0)
        closed = getattr(yor, f"get_{side}_gripper_pose")()
        getattr(yor, f"open_{side}_gripper")()
        wait(1.0)
        opened = getattr(yor, f"get_{side}_gripper_pose")()
        results.append(check(f"{side} gripper close/open", closed < 0.1 and opened > 0.9,
                             f"closed {closed:.2f} open {opened:.2f}"))

    # --- lift
    h0 = yor.get_lift_height()
    yor.lift_up()
    wait(1.0)
    yor.lift_stop()
    h1 = yor.get_lift_height()
    yor.lift_down()
    wait(1.0)
    yor.lift_stop()
    h2 = yor.get_lift_height()
    results.append(check("lift up then down", h1 > h0 + 0.03 and h2 < h1 - 0.03,
                         f"{h0:.3f} -> {h1:.3f} -> {h2:.3f} m"))

    # --- neck
    yor.set_neck_position(pan=30.0, tilt=15.0)  # robot convention: head right, head down
    wait(1.0)
    pos = yor.get_neck_position()
    pan, tilt = pos["pan"], pos["tilt"]
    results.append(check("neck pan 30 / tilt 15", abs(pan - 30) < 3 and abs(tilt - 15) < 3,
                         f"pan {pan:.1f} tilt {tilt:.1f}"))
    yor.home_neck()

    # --- base: each axis for 2 s, check the displacement in the starting robot frame
    for vel, name in (([0.2, 0, 0], "forward"), ([0, 0.2, 0], "left"), ([0, 0, 0.5], "turn left")):
        p0 = yor.get_pose()
        yor.set_base_velocity(np.array(vel, dtype=float))
        wait(2.0)
        yor.set_base_velocity(np.zeros(3))
        wait(1.0)
        p1 = yor.get_pose()
        c, s = np.cos(p0["theta"]), np.sin(p0["theta"])
        dx, dy = p1["x"] - p0["x"], p1["y"] - p0["y"]
        d = np.array([c * dx + s * dy, -s * dx + c * dy, (p1["theta"] - p0["theta"] + np.pi) % (2 * np.pi) - np.pi])
        i = int(np.argmax(np.abs(vel)))
        ok = d[i] > 0.5 * vel[i] * 2.0 and all(abs(d[j]) < 0.1 for j in range(3) if j != i)
        results.append(check(f"base {name}", ok, f"moved [dx dy dtheta] = {np.round(d, 3)}"))

    print(f"\n{sum(results)}/{len(results)} passed")
    return all(results)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rpc", action="store_true", help="test a running yor_mujoco.py over commlink")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=5557)
    parser.add_argument("--gripper", choices=("custom", "agx"), default="custom", help="model to test (not with --rpc)")
    args = parser.parse_args()

    if args.rpc:
        from commlink import RPCClient

        ok = run(RPCClient(args.host, args.port))
    else:
        from robot.yor_mujoco import SIM_MODELS, YORMujoco

        yor = YORMujoco(model_path=SIM_MODELS[args.gripper], headless=True)
        yor.init()
        try:
            ok = run(yor)
        finally:
            yor.stop()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
