"""
Quick non-interactive motor pulse test.
Spins each specified motor at low duty cycle for 1 second, reports velocity.
"""
import math
import time
import sys

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

CAN_IF   = "can0"
IDS      = [1, 2, 6]
DUTY     = 0.25   # 25% — enough to spin freely, gentle enough to be safe
PULSE_S  = 1.0    # seconds per motor
HZ       = 50

KNOWN = {1: "Drive FL", 2: "Drive RL", 6: "Rot   RL"}

def pulse(can_id: int):
    label = KNOWN.get(can_id, f"ID {can_id}")
    print(f"\n--- Motor {can_id} ({label}) ---")

    m = SparkFlex(CAN_IF, can_id)
    # Send heartbeat and wait for status frames to populate binding cache
    for _ in range(5):
        m.Heartbeat()
        time.sleep(0.1)

    v = float(m.GetBusVoltage()) if hasattr(m, "GetBusVoltage") else 0.0
    print(f"  Bus voltage: {v:.1f} V  (candump confirmed alive — proceeding regardless)")

    try:
        m.SetIdleMode(IDLE_COAST)
    except Exception:
        pass
    if hasattr(m, "SetCtrlType"):
        try:
            m.SetCtrlType(CTRL_DUTY)
        except Exception:
            pass

    print(f"  Pulsing duty={DUTY:+.2f} for {PULSE_S:.1f} s ...")
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
    for _ in range(5):
        m.Heartbeat()
        if hasattr(m, "SetDutyCycle"):
            m.SetDutyCycle(0.0)
        else:
            m.SetVelocity(0.0)
        time.sleep(0.02)

    if vels:
        peak = max(vels, key=abs)
        avg  = sum(vels) / len(vels)
        spun = abs(peak) > 0.5
        print(f"  Velocity — peak: {peak:+.3f}  avg: {avg:+.3f}  {'MOVED' if spun else 'NO MOVEMENT (brake mode or stall?)'}")
    else:
        print("  No velocity readings.")

if __name__ == "__main__":
    print(f"Pulse test: IDs {IDS} on {CAN_IF}  duty={DUTY}")
    for cid in IDS:
        pulse(cid)
    print("\nDone.")
