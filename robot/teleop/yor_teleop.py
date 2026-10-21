"""Deprecated entry point, kept so existing commands and docs keep working.

This used to be a separate copy of the VR teleop client. It had drifted from
vr_teleop.py (old rotation mapping, no fail button, the same recording bugs),
so it now just runs vr_teleop with the same flags. The previous version is in
git history.
"""
import runpy
import sys
from pathlib import Path

if __name__ == "__main__":
    print("[yor_teleop] Deprecated: running robot/teleop/vr_teleop.py instead", file=sys.stderr)
    sys.argv[0] = str(Path(__file__).with_name("vr_teleop.py"))
    runpy.run_module("robot.teleop.vr_teleop", run_name="__main__")
