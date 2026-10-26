"""Shared runtime network configuration.

The values below are defaults only. Each can be overridden without editing this
file, in increasing order of precedence:

    1. this file                        (the fallback baked in below)
    2. the matching environment variable  `VR_IP=10.0.0.9 python -m robot.teleop.vr_teleop`
    3. the script's CLI flag              `python -m robot.teleop.vr_teleop --vr-ip 10.0.0.9`

EPISODE_PUB_HOST is the interface the teleop episode publisher binds to; "*"
means every interface, so the Orin can reach it on whatever address the Pi
currently holds. Set it only to pin the publisher to one NIC.
"""

import os

ORIN_IP = os.environ.get("ORIN_IP", "192.168.123.117")
VR_IP = os.environ.get("VR_IP", "192.168.123.227")

EPISODE_PUB_HOST = os.environ.get("EPISODE_PUB_HOST", "*")
EPISODE_PUB_PORT = int(os.environ.get("EPISODE_PUB_PORT", "5558"))

# ZED pose/image publisher host. The ZED is on the Orin, so default to it;
# the Pi has no route to the old 192.168.1.x camera network.
ZED_IP = os.environ.get("ZED_IP", ORIN_IP)
ZED_PORT = int(os.environ.get("ZED_PORT", "6000"))

# The Pico serial lift (/dev/ttyACM0) is installed and enabled by default.
# Set YOR_LIFT=0 to run without it: the Base then never opens the serial port
# and every lift call is a no-op.
LIFT_ENABLED = os.environ.get("YOR_LIFT", "1") == "1"

# Lift travel and speed, measured 2026-10-23 (joystick full-range runs, 50 Hz
# get_lift_height polling): hard stops at 0.000 m and 0.6726 m. Speed is
# constant with no visible ramp or coast (~30 ms command lag), so for IK the
# lift is height += rate * dt while commanded. Down is ~16% faster than up.
# LIFT_MAX_M keeps ~1 cm off the top stop.
LIFT_MIN_M = 0.0
LIFT_MAX_M = 0.66
LIFT_TRAVEL_M = 0.6726  # bottom stop -> top stop
LIFT_UP_RATE_MPS = 0.0265
LIFT_DOWN_RATE_MPS = 0.0308

# The lift encoder is incremental: the Pico zeroes it wherever the carriage is at
# power-on. Heights are absolute only after the carriage has stalled against the
# bottom hard stop (lift_home(), or driving down until it stops), which re-zeroes
# the Pico; see PicoLift for how "homed" is tracked.

# BaseController zeroes the base if no set_base_velocity() arrives within this
# window (seconds), so a stopped or crashed teleop client can't leave it driving.
BASE_VEL_CMD_TIMEOUT = float(os.environ.get("BASE_VEL_CMD_TIMEOUT", "0.3"))
