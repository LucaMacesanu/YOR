"""
Whole-body VR teleop for YOR. The code lives in vr-teleop/server/yor_bridge.py
(shared with the simulator); this launcher only supplies YOR's defaults: the
Orin video address and the neck calibration file.

Start robot/yor.py first, then:

    python robot/whole_body_teleop.py                 # live: neck + arms + base + lift + gripper
    python robot/whole_body_teleop.py --no-arms       # leave out arms (or --no-base / --no-neck / --no-lift)
    python robot/whole_body_teleop.py --dry-run       # no robot: print commands only
    # headset browser: open the URL this prints -> accept cert -> Enter VR

Teleop starts OFF. Controls:
  A + X (one per hand), tap   teleop on/off  (off: arms hold, base stops, neck holds)
  A + X, hold 1.5 s           home both arms + neck, teleop off
  grip (hold)                 move that arm; release and it holds
  B + Y (one per hand)        base mode on/off;  left stick drive, right stick turn
  head                  neck pan/tilt;  left stick click re-centers
  right trigger         SameHand fingers: 0 open .. full pull = closed (touching)
  A + Y (hold)          lift down;  X + B (hold) lift up;  release stops
The lift's automatic head-height retargeting stays off. The left trigger is unused (no left gripper).
The Pico lift keeps moving until it gets "stop": the bridge sends stop on release, teleop off,
stale/lost headset, home, and exit -- but a hard crash (kill -9) mid-move would not.

SameHand fingers: start the hand first (~/SameHand-UMI/device/software/run.sh start calib
bridge). They stay limp until the first right-arm clutch with teleop on, then hold their last
target while unclutched (so a grasp survives a re-clutch), and go limp again when teleop turns
off, on home, on headset disconnect, and when this script exits. --no-gripper leaves them alone.

Needs: aiohttp, mink (yor-nero env) and the openssl CLI (self-signed cert).
"""
import os
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "vr-teleop" / "server"))
sys.path.insert(1, str(REPO))

from robot.config import ORIN_IP  # noqa: E402
import yor_bridge  # noqa: E402  (vr-teleop/server/yor_bridge.py)

if __name__ == "__main__":
    yor_bridge.main(
        default_video=os.environ.get("VR_VIDEO", f"{ORIN_IP}:8444"),
        neck_calib=str(REPO / "robot" / "neck" / "neck_calibration.json"),
    )
