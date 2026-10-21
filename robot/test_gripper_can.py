"""
Gripper MIT control via 0x159.
Usage: python robot/test_gripper_can.py --can can_left

Commands:
  i                       - init arm
  mv<vel>[,kd[,torque]]   - MIT velocity command, e.g. mv0.5  mv-0.5,0.5,0.1
  mp<pos>[,kp[,kd]]       - MIT position command, e.g. mp1.5  mp-1.0,10,0.5
  stop                    - send zero vel/torque
  d                       - exit MIT mode (motor goes limp)
  ge                      - enable gripper (high-level protocol, status=3 kick)
  gp<mm>[,effort]         - go to raw position in mm, e.g. gp20  gp-10,0.1
  sweep [start [end [step [effort [hold_s]]]]] - sweep mm range, default: -300 300 5 0.05 0.5
  w                       - stream all CAN frames until Enter
  wp<hexid>               - stream single CAN ID, e.g. wp2a8
  q                       - quit
"""
import argparse
import struct
import threading
import time
import can


MOTOR_ENABLE_ID = 0x471
GRIPPER_CMD_ID  = 0x159

POS_MIN, POS_MAX = -12.5, 12.5
VEL_MIN, VEL_MAX = -45.0,  45.0
KP_MIN,  KP_MAX  =   0.0, 500.0
KD_MIN,  KD_MAX  =  -5.0,   5.0
T_MIN,   T_MAX   = -25.0,  25.0

MIT_EXIT = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])


def _fi(val, min_, max_, bits):
    return int((val - min_) / (max_ - min_) * ((1 << bits) - 1))


def make_mit_frame(pos=0.0, vel=0.0, kp=0.0, kd=0.5, torque=0.0) -> bytes:
    pos_ref = _fi(max(POS_MIN, min(POS_MAX, pos)), POS_MIN, POS_MAX, 16)
    spd_ref = _fi(max(VEL_MIN, min(VEL_MAX, vel)), VEL_MIN, VEL_MAX, 12)
    kp_int  = _fi(max(KP_MIN,  min(KP_MAX,  kp)),  KP_MIN,  KP_MAX,  12)
    kd_int  = _fi(max(KD_MIN,  min(KD_MAX,  kd)),  KD_MIN,  KD_MAX,  12)
    t_ref   = _fi(max(T_MIN,   min(T_MAX,   torque)), T_MIN, T_MAX,    8)
    d = [0] * 8
    d[0] = (pos_ref >> 8) & 0xFF
    d[1] = pos_ref & 0xFF
    d[2] = (spd_ref >> 4) & 0xFF
    d[3] = (((spd_ref & 0xF) << 4) & 0xF0) | ((kp_int >> 8) & 0x0F)
    d[4] = kp_int & 0xFF
    d[5] = (kd_int >> 4) & 0xFF
    d[6] = (((kd_int & 0xF) << 4) & 0xF0) | ((t_ref >> 4) & 0x0F)
    crc   = (d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[5] ^ d[6]) & 0x0F
    d[7] = ((t_ref << 4) & 0xF0) | crc
    return bytes(d)


def make_grip_frame(pos_mm: float, effort: float = 0.1, status: int = 1) -> bytes:
    pos_ref = int(pos_mm * 1000)        # mm → µm
    effort_ref = int(effort * 5000)     # 0-1 → mNm
    return struct.pack(">iH", pos_ref, effort_ref) + bytes([status, 0x00])


def init_arm(bus):
    def mode_frame(move_mode, speed_rate, arm_controller):
        return bytes([0x01, move_mode, speed_rate, arm_controller, 0x00, 0x00, 0x01, 0x00])
    bus.send(can.Message(arbitration_id=0x151, data=mode_frame(0x01, 50, 0x00), is_extended_id=False))
    time.sleep(0.3)
    bus.send(can.Message(arbitration_id=MOTOR_ENABLE_ID, data=bytes([0x08, 0x02, 0, 0, 0, 0, 0, 0]), is_extended_id=False))
    time.sleep(0.6)
    bus.send(can.Message(arbitration_id=0x151, data=mode_frame(0x04, 100, 0xAD), is_extended_id=False))
    time.sleep(0.2)
    print("Arm initialized")


def watch(bus, can_id=None):
    stop = threading.Event()
    threading.Thread(target=lambda: (input(), stop.set()), daemon=True).start()
    print(f"Streaming {'0x%03X' % can_id if can_id else 'all'} — press Enter to stop")
    while not stop.is_set():
        msg = bus.recv(timeout=0.1)
        if msg and (can_id is None or msg.arbitration_id == can_id):
            raw = int.from_bytes(msg.data[0:4], 'big', signed=True)
            print(f"  0x{msg.arbitration_id:03X}  {msg.data.hex()}  int32={raw}")


def scan(bus, duration=10):
    print(f"Scanning for {duration}s — move the gripper now...")
    first = {}
    changed = {}
    deadline = time.time() + duration
    while time.time() < deadline:
        msg = bus.recv(timeout=0.1)
        if msg:
            aid = msg.arbitration_id
            data = bytes(msg.data)
            if aid not in first:
                first[aid] = data
            elif data != first[aid] and aid not in changed:
                changed[aid] = (first[aid], data)
    if changed:
        print("IDs whose data changed:")
        for aid, (before, after) in sorted(changed.items()):
            print(f"  0x{aid:03X}  was={before.hex()}  now={after.hex()}")
    else:
        print("No IDs changed — gripper may not be broadcasting position feedback")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--can', default='can_left')
    args = parser.parse_args()

    print(f"Opening {args.can}...")
    with can.interface.Bus(channel=args.can, interface='socketcan') as bus:
        print("i=init  mv<vel>[,kd[,t]]  mp<pos>[,kp[,kd]]  stop  d=disable  scan  w=watch  wp<hexid>  q=quit")
        while True:
            key = input("> ").strip().lower()
            if key == 'i':
                init_arm(bus)
            elif key.startswith('mv'):
                try:
                    parts = key[2:].split(',')
                    vel    = float(parts[0])
                    kd     = float(parts[1]) if len(parts) > 1 else 0.5
                    torque = float(parts[2]) if len(parts) > 2 else 0.0
                    frame  = make_mit_frame(vel=vel, kp=0.0, kd=kd, torque=torque)
                    for _ in range(20):
                        bus.send(can.Message(arbitration_id=GRIPPER_CMD_ID, data=frame, is_extended_id=False))
                        time.sleep(0.005)
                    print(f"MIT vel={vel} kd={kd} torque={torque}  frame: {frame.hex()}")
                except ValueError:
                    print("Usage: mv<vel>[,kd[,torque]]  e.g. mv0.5  mv-0.5,0.5,0.1")
            elif key.startswith('mp'):
                try:
                    parts = key[2:].split(',')
                    pos = float(parts[0])
                    kp  = float(parts[1]) if len(parts) > 1 else 10.0
                    kd  = float(parts[2]) if len(parts) > 2 else 0.5
                    frame = make_mit_frame(pos=pos, kp=kp, kd=kd)
                    bus.send(can.Message(arbitration_id=GRIPPER_CMD_ID, data=frame, is_extended_id=False))
                    print(f"MIT pos={pos} kp={kp} kd={kd}  frame: {frame.hex()}")
                except ValueError:
                    print("Usage: mp<pos>[,kp[,kd]]  e.g. mp1.5  mp-1.0,10,0.5")
            elif key == 'stop':
                frame = make_mit_frame(vel=0.0, kp=0.0, kd=0.5, torque=0.0)
                bus.send(can.Message(arbitration_id=GRIPPER_CMD_ID, data=frame, is_extended_id=False))
                print(f"Sent stop  frame: {frame.hex()}")
            elif key == 'd':
                for _ in range(10):
                    bus.send(can.Message(arbitration_id=GRIPPER_CMD_ID, data=MIT_EXIT, is_extended_id=False))
                    time.sleep(0.005)
                print("Sent MIT exit x10 — motor limp")
            elif key == 'ge':
                frame = make_grip_frame(0.0, effort=0.1, status=3)
                for _ in range(20):
                    bus.send(can.Message(arbitration_id=GRIPPER_CMD_ID, data=frame, is_extended_id=False))
                    time.sleep(0.005)
                print(f"Gripper enabled (status=3)  frame: {frame.hex()}")
            elif key.startswith('gp'):
                try:
                    parts = key[2:].split(',')
                    pos_mm = float(parts[0])
                    effort = float(parts[1]) if len(parts) > 1 else 0.1
                    frame = make_grip_frame(pos_mm, effort=effort, status=1)
                    for _ in range(40):
                        bus.send(can.Message(arbitration_id=GRIPPER_CMD_ID, data=frame, is_extended_id=False))
                        time.sleep(0.005)
                    print(f"Grip pos={pos_mm}mm effort={effort}  frame: {frame.hex()}")
                except ValueError:
                    print("Usage: gp<mm>[,effort]  e.g. gp20  gp-10,0.05")
            elif key.startswith('sweep'):
                try:
                    parts = key.split()
                    start  = float(parts[1]) if len(parts) > 1 else -300.0
                    end    = float(parts[2]) if len(parts) > 2 else  300.0
                    step   = float(parts[3]) if len(parts) > 3 else    5.0
                    effort = float(parts[4]) if len(parts) > 4 else    0.05
                    hold_s = float(parts[5]) if len(parts) > 5 else    0.5
                    pos = start
                    direction = 1 if end > start else -1
                    print(f"Sweeping {start}mm → {end}mm step={step}mm effort={effort} hold={hold_s}s  (Enter to stop)")
                    stop = threading.Event()
                    threading.Thread(target=lambda: (input(), stop.set()), daemon=True).start()
                    while not stop.is_set() and (direction * pos) <= (direction * end):
                        frame = make_grip_frame(pos, effort=effort, status=1)
                        deadline = time.time() + hold_s
                        print(f"  pos={pos:+.1f}mm  frame: {frame.hex()}")
                        while not stop.is_set() and time.time() < deadline:
                            bus.send(can.Message(arbitration_id=GRIPPER_CMD_ID, data=frame, is_extended_id=False))
                            time.sleep(0.005)
                        pos += direction * step
                    print("Sweep done")
                except ValueError:
                    print("Usage: sweep [start [end [step [effort]]]]  e.g. sweep -100 100 5 0.1")
            elif key == 'scan':
                scan(bus)
            elif key.startswith('wp'):
                try:
                    watch(bus, int(key[2:], 16))
                except ValueError:
                    print("Usage: wp<hexid>  e.g. wp2a8 or wp2a9")
            elif key == 'w':
                watch(bus)
            elif key == 'q':
                break


if __name__ == '__main__':
    main()
