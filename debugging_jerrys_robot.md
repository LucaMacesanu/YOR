# Debugging Jerry's Robot — CAN Bus & Motor Verification

## Platform
- YOR bimanual robot (NYU GRAIL), swerve drive base
- Raspberry Pi as host controller
- REV SparkFlex motors on CAN bus

---

## CAN Interfaces (Raspberry Pi)

```
can0      UP  qlen=1000  — swerve drive base (all 8 motors)
can_left  UP  qlen=10    — left arm (nerolib)
can_right UP  qlen=10    — right arm (nerolib)
```

Command to verify:
```bash
ip link show | grep can
```

---

## Motors on `can0`

All 8 REV SparkFlex motors are daisy-chained on `can0` and actively broadcasting.

| CAN ID | Role     | Position   | Status frame IDs          |
|--------|----------|------------|---------------------------|
| 1      | Drive    | Front Left | `02050001`  `02052C81`    |
| 2      | Drive    | Rear Left  | `02050002`  `02052C82`    |
| 3      | Drive    | Rear Right | `02050003`  `02052C83`    |
| 4      | Drive    | Front Right| `02050004`  `02052C84`    |
| 5      | Rotation | Front Left | `02050105`  `02052C85`    |
| 6      | Rotation | Rear Left  | `02050106`  `02052C86`    |
| 7      | Rotation | Rear Right | `02050107`  `02052C87`    |
| 8      | Rotation | Front Right| `02050108`  `02052C88`    |

Each motor broadcasts two frame types:
- `02050X0X` — Status 0: velocity, position
- `02052CXX` — Status 2: applied output, faults

Command used to detect:
```bash
candump -ta can0 &  sleep 2; kill %1
```

Device ID extraction: lowest 6 bits of arbitration ID (`arb & 0x3F`).

---

## CAN Scan — False Positive Bug (Fixed)

**Problem:** Script reported 8 motors alive even when fewer were physically connected.

**Root cause:** `SparkFlex.GetVelocity()` returns `0.0` (not `NaN`) for CAN IDs with no
motor on the bus — the binding initialises its cache to `0.0`. The `not math.isnan(vel)`
check therefore passed for every ID.

**Fix:** Use `GetBusVoltage() > 5.0 V` as the alive indicator instead.
- Real motor (powered, on bus): returns ~24 V
- Non-existent ID (no frame received): returns 0.0 V

---

## Motor Not Spinning — Diagnosis

**Symptom:** Motors detected as alive, commands sent, `GetVelocity()` stays at ~0.

### Issue 1 — `CtrlType` enum import fails
`from sparkcan_py import CtrlType` fails on this system.
The script skips `SetCtrlType`, so the motor stays in its default control mode.

**Fix in script:** Fall back to raw integers (REV API values):
- `kDutyCycle = 0`
- `kVelocity  = 1`

Call `motor.SetCtrlType(0)` or `motor.SetCtrlType(1)` directly.

### Issue 2 — Brake mode set in flash
`SetIdleMode(kCoast)` at runtime does not override the value burned in flash.
If the motor was configured with **Idle Mode = Brake** via REV Hardware Client,
it will revert to brake mode on every power cycle.

**Fix:** Connect each motor via **USB-C** to a laptop running **REV Hardware Client**:
1. Select the SparkFlex
2. Go to **Config** tab
3. Set **Idle Mode = Coast**
4. Click **Burn Flash**

### Issue 3 — Velocity PID gains not set (velocity mode only)
In velocity mode the motor runs a PID loop. Factory default gains are all zero,
so the motor will not spin even if the setpoint and control type are correct.

**Fix via REV Hardware Client → Control tab:**
| Gain | Starting value | Purpose                          |
|------|---------------|----------------------------------|
| kF   | 0.0002        | Feed-forward — **must be non-zero to spin** |
| kP   | 0.0001        | Proportional tracking             |

---

## Test Script

**File:** `robot/test_drive_motor_can.py`

```bash
# Duty-cycle mode (works with factory defaults — test this first)
python robot/test_drive_motor_can.py --mode duty --max-speed 0.3

# Velocity mode (requires kF set in REV Hardware Client)
python robot/test_drive_motor_can.py --mode velocity --max-speed 200

# Scan more IDs or a different interface
python robot/test_drive_motor_can.py --can can0 --scan-max 12
```

### Joystick controls (Xbox layout)
| Input | Action |
|-------|--------|
| Hold **LB** | Deadman — engages all detected motors |
| Left stick up / down | Forward / reverse |
| Right stick up / down | Same — use either hand |
| Hold **RB** + LB | Boost ×10 (raw unit scale test) |
| **Back** | Quit |

### Live output columns
| Column | Meaning |
|--------|---------|
| `Alive` | `GetBusVoltage > 5V` — motor on bus and powered |
| `Cmd`   | Exact value sent to `SetVelocity` / `SetDutyCycle` |
| `GetVel`| Velocity reported back by motor |
| `Notes` | Auto-diagnosis (brake mode hint, PID hint, boost active) |

### Diagnostic notes in loop output
| Note | Meaning |
|------|---------|
| `vel~0 despite cmd` (velocity mode) | kF=0 — set feed-forward in REV Hardware Client |
| `vel~0` (duty mode) | Brake mode active in flash, or stall at low duty — try `--max-speed 0.9` |
| `BOOST x10` | RB held — command scaled ×10 for unit test |

---

## REV SparkFlex CAN Protocol Notes

- Extended 29-bit arbitration IDs
- Device ID = `arb_id & 0x3F` (lowest 6 bits)
- Manufacturer = REV (0x05), Device type = motor controller (0x02)
- Status frames broadcast continuously at ~2 ms (Status 0) and ~20 ms (Status 2)
- Control type integers: `kDutyCycle=0  kVelocity=1  kVoltage=2  kPosition=3`
- Idle mode integers:    `kCoast=0  kBrake=1`
- Heartbeat must be sent at ≥10 Hz or the controller faults and stops output

---

## Next Steps

- [ ] Set **Idle Mode = Coast** on all 8 motors via REV Hardware Client (USB-C)
- [ ] Confirm motors spin in duty-cycle mode at `--max-speed 0.9`
- [ ] If velocity mode needed: set `kF=0.0002` and `kP=0.0001` per motor
- [ ] Verify rotation motors (IDs 5-8) respond to position commands separately
