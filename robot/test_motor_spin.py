"""
Brief spin test for each motor on can0.
Pulses each motor at low duty cycle for 1.5 s, reports velocity response.
"""
import math, time, sys
from sparkcan_py import SparkFlex

try:
    from sparkcan_py import CtrlType
    CTRL_DUTY = CtrlType.kDutyCycle
except Exception:
    CTRL_DUTY = 0

try:
    from sparkcan_py import IdleMode
    IDLE_COAST = IdleMode.kCoast
except Exception:
    IDLE_COAST = 0

CAN_IF  = "can0"
IDS     = [1, 2, 5, 6]
DUTY    = 0.15   # 15% — gentle
PULSE_S = 1.5
HZ      = 50

KNOWN = {1: "Drive FL", 2: "Drive RL", 5: "Rot   FL", 6: "Rot   RL"}

def pulse(can_id):
    label = KNOWN.get(can_id, f"ID {can_id}")
    print(f"\n[Motor {can_id} — {label}]")

    try:
        m = SparkFlex(CAN_IF, can_id)
    except Exception as e:
        print(f"  OPEN FAILED: {e}")
        return

    # warm-up: send heartbeats so the controller doesn't fault
    for _ in range(10):
        m.Heartbeat()
        time.sleep(0.05)

    try:
        m.SetIdleMode(IDLE_COAST)
    except Exception:
        pass
    if hasattr(m, "SetCtrlType"):
        try:
            m.SetCtrlType(CTRL_DUTY)
        except Exception:
            pass

    print(f"  Spinning at duty={DUTY:+.2f} for {PULSE_S:.1f} s ...")
    interval = 1.0 / HZ
    t_end = time.monotonic() + PULSE_S
    vels = []
    while time.monotonic() < t_end:
        m.Heartbeat()
        if hasattr(m, "SetDutyCycle"):
            m.SetDutyCycle(DUTY)
        else:
            m.SetVelocity(DUTY)
        vel = float(m.GetVelocity())
        if not math.isnan(vel):
            vels.append(vel)
        time.sleep(interval)

    # stop
    for _ in range(10):
        m.Heartbeat()
        if hasattr(m, "SetDutyCycle"):
            m.SetDutyCycle(0.0)
        else:
            m.SetVelocity(0.0)
        time.sleep(0.02)

    if vels:
        peak = max(vels, key=abs)
        avg  = sum(vels) / len(vels)
        moved = abs(peak) > 0.5
        status = "SPUN OK" if moved else "NO MOVEMENT — possible brake mode in flash (use REV Hardware Client to set Coast + Burn Flash)"
        print(f"  peak vel: {peak:+.3f}   avg: {avg:+.3f}   -> {status}")
    else:
        print("  No velocity readings returned.")

    time.sleep(0.5)  # pause between motors

if __name__ == "__main__":
    print(f"Spin test — IDs {IDS} on {CAN_IF}  duty={DUTY}")
    for cid in IDS:
        pulse(cid)
    print("\nDone.")
