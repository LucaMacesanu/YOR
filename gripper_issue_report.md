# Nero Arm Gripper — Troubleshooting Report
**Date:** 2026-04-16  
**Hardware:** Two Nero arms (left: `can_left`, right: `can_right`) on a Raspberry Pi 5 running Linux  
**Software:** nerolib (C++/pybind11), pyAgxArm, python-can (socketcan)

---

## Problem Summary

The AGX gripper attached to the right Nero arm does not respond to `0x159` position/width control commands. The gripper motor can be toggled stiff/soft via `0x471` motor-8 enable/disable, but no position control is achievable. The gripper feedback message (`0x2A8`) never appears on the CAN bus.

---

## Hardware Setup

- Raspberry Pi 5
- Three SocketCAN interfaces: `can0` (swerve drive), `can_left` (left arm), `can_right` (right arm)
- AGX gripper connected to the right arm's wrist end-effector connector via a cable carrying power and CAN

---

## Observed Behavior

| Action | Result |
|--------|--------|
| Run nerolib (`yor.py`) | Gripper goes stiff and partially closes |
| Press enter to stop nerolib | Gripper goes soft |
| Send `0x471` motor-8 enable (raw CAN) | Gripper goes stiff on first send only |
| Send `0x471` motor-8 disable (raw CAN) | Gripper goes soft reliably |
| Send `0x159` open/close (any status code) | No response whatsoever |
| `0x2A8` gripper feedback | Never appears in candump |
| Plug/unplug gripper | No change in CAN IDs seen (same 22 IDs with or without gripper) |

---

## CAN Bus State

Running `candump can_right` with arm initialized shows 22 unique CAN IDs:

```
0x2A1  arm status
0x2A2–0x2A7  end-effector pose
0x2A9  joint 7 angle
0x251–0x257  joint low-speed feedback (joints 1–7)
0x261–0x267  joint high-speed feedback (joints 1–7)
```

**`0x2A8` (gripper feedback) never appears** — with or without the gripper physically connected.

---

## Initialization Sequence Used (nerolib)

From `nero_interface.cpp`:

1. `set_arm_mode(CAN_COMMAND, JOINT, 50, PV)` → `0x151`
2. `0x471` with `data=[0x08, 0x02, 0, 0, 0, 0, 0, 0]`
3. `set_arm_mode(CAN_COMMAND, MIT, 100, MIT)` → `0x151`
4. `reset_gripper()`:
   - Send `0x159` with status_code=`0x02` (disable/clear/width), pos=0, effort=0
   - Wait for `0x2A8` (times out — changed to warning instead of throw)
   - Send `0x159` with status_code=`0x01` (enable/width), pos=0, effort=0
   - Wait for `0x2A8` (times out)
5. Control loop starts, sends `0x159` at 500Hz with `target_gripper_pos=1.0` (open), effort=0.1

Despite step 4 timing out, the gripper physically stiffens and partially closes on startup.

---

## Everything We Tried

### 1. Raw CAN via `test_gripper_can.py`
- Sent `0x151` mode command + `0x471` motor enable + `0x159` open/close in various sequences
- Tried status codes: `0x01`, `0x02`, `0x03` on `0x159`
- Tried one-shot and continuous 200Hz loop
- Tried matching nerolib's exact init sequence byte-for-byte
- **Result:** No gripper movement, `0x2A8` never appeared

### 2. Plug/unplug comparison
- Captured `candump can_right` with gripper unplugged vs plugged in
- **Result:** Identical 22 CAN IDs both times — gripper adds no new IDs to the bus

### 3. MIT joint commands to joints 7 and 8
- Sent MIT frames to `0x160` (joint 7) and `0x161` (joint 8) with various positions
- `0x160` moved the wrist joint; `0x161` had no effect
- **Result:** Wrist rotation is joint 7, not the gripper mechanism

### 4. pyAgxArm high-level API (`test_gripper_pyagx.py`)
- Used `AgxArmFactory.create_arm()` + `robot.init_effector(AGX_GRIPPER)`
- Called `end_effector.move_gripper_m(value=0.07, force=5.0)`
- **Result:** Gripper status always `None`, no movement

### 5. pyAgxArm calibration sequence
- Called `end_effector.disable_gripper()`, `set_gripper_teaching_pendant_param(max_range_config=0.07)`, manually closed gripper, `calibrate_gripper()`
- `calibrate_gripper()` timed out waiting for `0x476` ACK
- **Result:** Gripper went stiff during the attempt (possibly received the command) but calibration not confirmed, and subsequent `move_gripper_m` still had no effect

### 6. Manufacturer demo (`detect_nero_series.py` adapted)
- Modified `test_can_right.py` to use `channel="can_right"`, called `robot.enable()` + `end_effector.move_gripper_m()`
- **Result:** No gripper movement

### 7. `0x471` motor-8 enable/disable (raw CAN)
- Discovered that `0x471 [0x08, 0x01]` (disable) reliably makes gripper go soft
- `0x471 [0x08, 0x02]` (enable) only works on first call after power-on — does not re-home on subsequent calls
- Full arm disable → re-enable cycle does re-home the gripper but also moves arm joints
- **Result:** Binary soft/stiff control confirmed. No position control.

### 8. `nerolib` continuous `0x159` at 500Hz
- nerolib's control loop sends `set_gripper(target_gripper_pos, 0.1, ENABLE)` at 500Hz
- Changed `target_gripper_pos` between `0.0` (closed) and `1.0` (open) while loop running
- **Result:** No observable gripper movement

### 9. Raw `0x471` disable→enable while nerolib loop active
- Added raw CAN socket in `arm.py` alongside running nerolib
- `open_gripper()`: send `0x471` motor-8 disable
- `close_gripper()`: send disable, wait 150ms, send enable
- **Result:** Open (disable) works; re-enable while loop active did not re-trigger homing

---

## Key Observations

1. **`0x2A8` feedback never appears** regardless of gripper state, suggesting either the gripper is not transmitting or the arm controller is not relaying it.

2. **`0x159` commands are completely ignored** — no position control, no mode change, no error response.

3. **`0x471` motor-8 commands work** for stiff/soft toggling, but only the very first enable triggers a homing/close motion.

4. **No USB/RS485 devices** (`/dev/ttyUSB*`) found — gripper is confirmed to be on the CAN bus, not serial.

5. **Only three CAN interfaces** exist on the system (`can0`, `can_left`, `can_right`) — no separate gripper CAN bus.

---

## Questions for the Manufacturer

1. Why does `0x2A8` gripper feedback never appear on `can_right` even with the gripper physically connected and the arm enabled?

2. Is there a specific initialization sequence required before `0x159` position commands are accepted? The nerolib `reset_gripper` sequence (disable + enable via `0x159`) does not produce `0x2A8` feedback and times out.

3. Does the gripper require calibration (`calibrate_gripper` / `0x159` with `set_zero=0xAE`) before position commands work? If so, why does the calibration ACK (`0x476`) never appear?

4. Is `0x2A8` feedback only sent in response to `0x159` commands, or does it broadcast periodically once enabled?

5. Is there a hardware configuration (e.g., CAN termination jumper) on the last arm joint (joint 7) that needs to be changed when the gripper is attached?

6. Does this gripper variant support `0x159` width-mode position control, or is it only controllable via motor enable/disable (`0x471`)?

---

## Software Versions / Environment

- Platform: Raspberry Pi 5, Linux 6.12.47+rpt-rpi-2712
- Python: 3.x (conda environment)
- nerolib: local build (C++/pybind11)
- pyAgxArm: installed from `~/pyAgxArm` (development mode)
- python-can: socketcan interface
