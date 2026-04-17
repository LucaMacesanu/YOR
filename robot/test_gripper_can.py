"""
Minimal gripper test — sends raw CAN frames to actuate the gripper.
Usage: python test_gripper_can.py --can can_left

Commands:
  i       - init arm (CAN_COMMAND mode + enable all motors)
  e       - enable gripper motor (0x471, motor index 8)
  d       - disable gripper motor (0x471, motor index 8) — may open gripper
  r       - reset gripper via 0x159 handshake (disable+clear then enable, like nerolib does)
  z       - set zero / calibrate (PHYSICALLY CLOSE GRIPPER FIRST, then press z)
  o       - open gripper via 0x159
  c       - close gripper via 0x159
  j7o     - send MIT command to joint 7 (0x160) pos=+1.5 rad
  j7c     - send MIT command to joint 7 (0x160) pos=-1.5 rad
  j7,<a>  - send MIT command to joint 7 with explicit angle (rad), e.g. j7,0.5
  j8o     - send MIT command to joint 8 (0x161) pos=+1.5 rad
  j8c     - send MIT command to joint 8 (0x161) pos=-1.5 rad
  p<id>,<pos> - send MIT to any CAN ID, e.g. p160,1.5
  l       - listen for 5s and print unique CAN IDs seen
  q       - quit
"""
import argparse
import struct
import time
import can


GRIPPER_CMD_ID      = 0x159
GRIPPER_FEEDBACK_ID = 0x2A8
MOTOR_ENABLE_ID     = 0x471
GRIPPER_MOTOR_INDEX = 0x08
JOINT7_MIT_ID       = 0x160  # joint 7 MIT command
JOINT8_MIT_ID       = 0x161  # joint 8 MIT command (potential gripper)

POS_MIN, POS_MAX = -12.5, 12.5
VEL_MIN, VEL_MAX = -45.0, 45.0
KP_MIN,  KP_MAX  = 0.0, 500.0
KD_MIN,  KD_MAX  = -5.0, 5.0
T_MIN,   T_MAX   = -25.0, 25.0


def make_gripper_frame(position: float, effort: float = 0.5, status_code: int = 1) -> bytes:
    # position: 0.0 = closed, 1.0 = open (width mode)
    # status_code: 1=enable/width, 3=enable/clear/width
    pos_ref    = int(position * 0.07 * 1e6)  # 0-1 -> meters -> micrometers
    effort_ref = int(effort * 5000)           # 0-1 -> mNm
    return struct.pack('>iH', pos_ref, effort_ref) + bytes([status_code, 0x00])


def float_to_int(val, min_, max_, bits):
    return int((val - min_) / (max_ - min_) * ((1 << bits) - 1))


def make_joint7_mit_frame(pos: float, kp: float = 15.0, kd: float = 0.8) -> bytes:
    pos_ref = float_to_int(max(POS_MIN, min(POS_MAX, pos)), POS_MIN, POS_MAX, 16)
    spd_ref = float_to_int(0.0, VEL_MIN, VEL_MAX, 12)
    kp_int  = float_to_int(kp,  KP_MIN,  KP_MAX,  12)
    kd_int  = float_to_int(kd,  KD_MIN,  KD_MAX,  12)
    t_ref   = float_to_int(0.0, T_MIN,   T_MAX,   8)
    d = [0] * 8
    d[0] = (pos_ref >> 8) & 0xFF
    d[1] = pos_ref & 0xFF
    d[2] = (spd_ref >> 4) & 0xFF
    d[3] = (((spd_ref & 0xF) << 4) & 0xF0) | ((kp_int >> 8) & 0x0F)
    d[4] = kp_int & 0xFF
    d[5] = (kd_int >> 4) & 0xFF
    d[6] = (((kd_int & 0xF) << 4) & 0xF0) | ((t_ref >> 4) & 0x0F)
    crc  = (d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[5] ^ d[6]) & 0x0F
    d[7] = ((t_ref << 4) & 0xF0) | crc
    return bytes(d)


def make_enable_frame(enable: bool = True) -> bytes:
    return bytes([GRIPPER_MOTOR_INDEX, 0x02 if enable else 0x01, 0, 0, 0, 0, 0, 0])


def make_mode_frame(move_mode, speed_rate, arm_controller):
    # ctrl_mode=0x01 (CAN_COMMAND), enable_can_push=0x01
    return bytes([0x01, move_mode, speed_rate, arm_controller, 0x00, 0x00, 0x01, 0x00])


def init_arm(bus):
    # Mirrors nerolib enable_arm() exactly:
    # 1. CAN_COMMAND + JOINT mode (move_mode=0x01, arm_controller=0x00 PV)
    bus.send(can.Message(arbitration_id=0x151, data=make_mode_frame(0x01, 50, 0x00), is_extended_id=False))
    print("  Sent: CAN_COMMAND + JOINT mode")
    time.sleep(0.3)

    # 2. Motor enable via 0x471, data[0]=0x08 (nerolib uses 0x08, not 0xFF)
    bus.send(can.Message(arbitration_id=MOTOR_ENABLE_ID, data=bytes([0x08, 0x02, 0, 0, 0, 0, 0, 0]), is_extended_id=False))
    print("  Sent: 0x471 motor enable (0x08)")
    time.sleep(0.6)

    # 3. Switch to CAN_COMMAND + MIT mode (move_mode=0x04, arm_controller=0xAD MIT)
    bus.send(can.Message(arbitration_id=0x151, data=make_mode_frame(0x04, 100, 0xAD), is_extended_id=False))
    print("  Sent: CAN_COMMAND + MIT mode")
    time.sleep(0.2)


def decode_gripper_feedback(data: bytes) -> str:
    if len(data) < 8:
        return "too short"
    pos_um = struct.unpack('>i', data[0:4])[0]
    effort = struct.unpack('>H', data[4:6])[0]
    status = data[6]
    pos_m  = pos_um / 1e6
    STATUS_NAMES = {0: "DISABLED/IDLE", 1: "ENABLED", 2: "CLEAR", 3: "ENABLED+CLEAR", 0xAE: "SET_ZERO"}
    return f"pos={pos_m*1000:.2f}mm  effort={effort}  status=0x{status:02X}({STATUS_NAMES.get(status, '?')})"


def listen(bus, duration=5.0):
    seen = {}
    deadline = time.time() + duration
    print(f"Listening for {duration}s...")
    while time.time() < deadline:
        msg = bus.recv(timeout=0.1)
        if msg:
            if msg.arbitration_id not in seen:
                seen[msg.arbitration_id] = msg
                if msg.arbitration_id == GRIPPER_FEEDBACK_ID:
                    print(f"  0x{msg.arbitration_id:03X}  {msg.data.hex()}  <-- {decode_gripper_feedback(msg.data)}")
                else:
                    print(f"  0x{msg.arbitration_id:03X}  {msg.data.hex()}")
    print(f"Saw {len(seen)} unique CAN IDs")


import threading

gripper_target = None  # None = loop not running, float = active target
loop_running = False


def gripper_loop(bus):
    """Send 0x159 continuously at ~200Hz like nerolib's control loop."""
    global gripper_target, loop_running
    while loop_running:
        t = gripper_target
        if t is not None:
            bus.send(can.Message(
                arbitration_id=GRIPPER_CMD_ID,
                data=make_gripper_frame(t, status_code=1),
                is_extended_id=False,
            ))
        time.sleep(0.005)  # 200Hz


def main():
    global gripper_target, loop_running
    parser = argparse.ArgumentParser()
    parser.add_argument('--can', default='can_left')
    args = parser.parse_args()

    print(f"Opening {args.can}...")
    with can.interface.Bus(channel=args.can, interface='socketcan') as bus:
        loop_running = True
        t = threading.Thread(target=gripper_loop, args=(bus,), daemon=True)
        t.start()
        print("i=init  e=enable  d=disable  r=reset  z=set_zero(calibrate)  o=open  c=close  l=listen  s=start_loop  x=stop_loop  q=quit")
        while True:
            key = input("> ").strip().lower()
            if key == 'i':
                print("Initializing arm...")
                init_arm(bus)
                listen(bus, duration=2.0)
            elif key == 'e':
                bus.send(can.Message(arbitration_id=MOTOR_ENABLE_ID, data=make_enable_frame(True), is_extended_id=False))
                print("Sent: ENABLE gripper motor (0x471, motor 8)")
                time.sleep(0.5)
                listen(bus, duration=2.0)
            elif key == 'd':
                gripper_target = None
                for _ in range(10):
                    bus.send(can.Message(arbitration_id=GRIPPER_CMD_ID, data=make_gripper_frame(0.0, effort=0.0, status_code=0), is_extended_id=False))
                    time.sleep(0.005)
                print("Sent: loop stopped + 0x159 status=0 (disable/width) x10 — gripper should go limp")
                time.sleep(0.5)
                listen(bus, duration=2.0)
            elif key == 'z':
                print("SET ZERO — make sure gripper is physically CLOSED before confirming.")
                confirm = input("Gripper closed? Type 'yes' to send calibrate command: ").strip().lower()
                if confirm == 'yes':
                    # status_code=0xAE = set_zero/calibrate
                    frame = make_gripper_frame(0.0, effort=0.0, status_code=0xAE)
                    bus.send(can.Message(arbitration_id=GRIPPER_CMD_ID, data=frame, is_extended_id=False))
                    print(f"Sent 0x159 set_zero (0xAE): {frame.hex()}")
                    listen(bus, duration=2.0)
                else:
                    print("Cancelled.")
            elif key == 'r':
                # Mirror nerolib's reset_gripper: disable+clear then enable via 0x159
                print("Step 1: sending disable+clear (status_code=0x02)...")
                bus.send(can.Message(arbitration_id=GRIPPER_CMD_ID, data=make_gripper_frame(0.0, status_code=2), is_extended_id=False))
                listen(bus, duration=1.0)
                print("Step 2: sending enable (status_code=0x01)...")
                bus.send(can.Message(arbitration_id=GRIPPER_CMD_ID, data=make_gripper_frame(0.0, status_code=1), is_extended_id=False))
                listen(bus, duration=1.0)
                print("Gripper reset done. Try 'o' to open.")
            elif key == 'o':
                # Kick with status_code=3 (enable+clear) then switch loop to status_code=1
                bus.send(can.Message(arbitration_id=GRIPPER_CMD_ID, data=make_gripper_frame(1.0, status_code=3), is_extended_id=False))
                time.sleep(0.05)
                gripper_target = 1.0
                print("Loop target: OPEN (1.0) — sent status=3 kick, loop running at 200Hz")
                listen(bus, duration=2.0)
            elif key == 'c':
                bus.send(can.Message(arbitration_id=GRIPPER_CMD_ID, data=make_gripper_frame(0.0, status_code=3), is_extended_id=False))
                time.sleep(0.05)
                gripper_target = 0.0
                print("Loop target: CLOSE (0.0) — sent status=3 kick, loop running at 200Hz")
                listen(bus, duration=2.0)
            elif key == 's':
                gripper_target = 1.0
                print("Gripper loop started at OPEN (1.0)")
            elif key == 'x':
                gripper_target = None
                print("Gripper loop paused")
            elif key == 'j7o':
                bus.send(can.Message(arbitration_id=JOINT7_MIT_ID, data=make_joint7_mit_frame(1.5), is_extended_id=False))
                print("Sent: joint 7 MIT pos=+1.5 rad")
            elif key == 'j7c':
                bus.send(can.Message(arbitration_id=JOINT7_MIT_ID, data=make_joint7_mit_frame(-1.5), is_extended_id=False))
                print("Sent: joint 7 MIT pos=-1.5 rad")
            elif key.startswith('j7,'):
                try:
                    angle = float(key[3:])
                    bus.send(can.Message(arbitration_id=JOINT7_MIT_ID, data=make_joint7_mit_frame(angle), is_extended_id=False))
                    print(f"Sent: joint 7 MIT pos={angle} rad")
                except Exception:
                    print("Usage: j7,<angle>  e.g. j7,0.5 or j7,-1.0")
            elif key == 'j8o':
                bus.send(can.Message(arbitration_id=JOINT8_MIT_ID, data=make_joint7_mit_frame(1.5), is_extended_id=False))
                print("Sent: joint 8 MIT pos=+1.5 rad")
            elif key == 'j8c':
                bus.send(can.Message(arbitration_id=JOINT8_MIT_ID, data=make_joint7_mit_frame(-1.5), is_extended_id=False))
                print("Sent: joint 8 MIT pos=-1.5 rad")
            elif key.startswith('p'):
                try:
                    can_id, pos = key[1:].split(',')
                    can_id = int(can_id, 16)
                    pos = float(pos)
                    bus.send(can.Message(arbitration_id=can_id, data=make_joint7_mit_frame(pos), is_extended_id=False))
                    print(f"Sent: CAN 0x{can_id:03X} MIT pos={pos} rad")
                except Exception:
                    print("Usage: p<hex_id>,<pos>  e.g. p160,1.5 or p159,-1.0")
            elif key == 'l':
                listen(bus)
            elif key == 'q':
                loop_running = False
                gripper_target = None
                for _ in range(10):
                    bus.send(can.Message(arbitration_id=GRIPPER_CMD_ID, data=make_gripper_frame(0.0, effort=0.0, status_code=0), is_extended_id=False))
                    time.sleep(0.005)
                break


if __name__ == '__main__':
    main()
