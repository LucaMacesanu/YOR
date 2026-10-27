"""
Whole-body VR teleop for YOR. The code lives in vr-teleop/server/yor_bridge.py
(shared with the simulator); this launcher only supplies YOR's defaults: the
Orin video address and the neck calibration file.

Start robot/yor.py first, then:

    python robot/whole_body_teleop.py                 # live: neck + arms + base + lift + gripper
    python robot/whole_body_teleop.py --no-arms       # leave out arms (or --no-base / --no-neck / --no-lift)
    python robot/whole_body_teleop.py --dry-run       # no robot: print commands only
    python robot/whole_body_teleop.py --task "pick up the cup"   # task prompt sent with each episode
    # headset browser: open the URL this prints -> accept cert -> Enter VR
    # laptop browser: https://<pi-ip>:8443/monitor -> read-only live view of everything

Teleop starts OFF. Controls:
  A + X (one per hand), tap   teleop on/off  (off: arms hold, base stops, neck holds)
  A + X, hold 1.5 s           home both arms + neck, teleop off
  grip (hold)                 move that arm; release and it holds
  B + Y (one per hand)        base mode on/off;  left stick drive, right stick turn
  head                  neck pan/tilt; straight ahead re-zeroes each time teleop turns on
  right stick click     episode start / end as success;  left stick click  end as fail
                        (in a cycle from the Orin: success moves to the next subtask; left click
                        with no episode open goes back one subtask)
  left stick (teleop off)  move the headset HUD up/down
  right stick up/down   IPD;  left/right (teleop off)  view tilt: the robot looks that much further
                        down (or up) than your head, so you don't crane your neck at a table
                        (teleop on or off, so policy rollouts can be marked too)
  trigger               that arm's AgileX gripper: 0 open .. full pull = closed
  A + Y (hold)          lift down;  X + B (hold) lift up;  release stops
The lift's automatic head-height retargeting stays off.
The Pico lift keeps moving until it gets "stop": the bridge sends stop on release, teleop off,
stale/lost headset, home, and exit -- but a hard crash (kill -9) mid-move would not.

Grippers stay limp until that arm's first clutch with teleop on, then hold their last target
while unclutched (so a grasp survives a re-clutch), and go limp again when teleop turns off, on
home, on headset disconnect, and when this script exits. --no-gripper leaves them alone.

Episodes go out on ZMQ :5558 (EPISODE_PUB_PORT), same topics as robot/teleop/vr_teleop.py:
episode/signal + teleop/episode_result on start/end, and a 30 Hz teleop/episode robot-state
frame in between, for the Orin collector. Each episode is also saved compressed as
teleop_data/episodes/<episode_id>.yep (read: python vr-teleop/server/episode_store.py FILE).
Don't run vr_teleop.py at the same time (same port).

Cycles: when the Orin collector announces a cycle of subtasks on collection/cycle
(CYCLE_SOURCE, default ORIN_IP:5559), each episode's task is the cycle's current subtask and
the HUD shows it (k/n, READY / REC / RETRY); progress is kept in teleop_data/cycle_state.json.
Format and click rules: vr-teleop/docs/EPISODE_FORMAT.md, "Cycles". Test without the Orin:
python vr-teleop/tools/announce_cycle.py CYCLES.json NAME --bind 127.0.0.1:5559 with
CYCLE_SOURCE=127.0.0.1:5559.

Needs: aiohttp, mink (yor-nero env) and the openssl CLI (self-signed cert).
"""
import os
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "vr-teleop" / "server"))
sys.path.insert(1, str(REPO))

from robot.config import CYCLE_SOURCE, EPISODE_PUB_HOST, EPISODE_PUB_PORT, ORIN_IP  # noqa: E402
import yor_bridge  # noqa: E402  (vr-teleop/server/yor_bridge.py)

if __name__ == "__main__":
    yor_bridge.main(
        default_video=os.environ.get("VR_VIDEO", f"{ORIN_IP}:8444"),
        neck_calib=str(REPO / "robot" / "neck" / "neck_calibration.json"),
        episode_host=EPISODE_PUB_HOST,
        episode_port=EPISODE_PUB_PORT,
        episode_dir=str(REPO / "teleop_data" / "episodes"),
        cycle_source=CYCLE_SOURCE,
        cycle_state=str(REPO / "teleop_data" / "cycle_state.json"),
    )
