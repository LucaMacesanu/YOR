#!/usr/bin/env python3
"""
Diagnostic test for arm movement.
Tests both pyAgxArm (JOINT mode) and nerolib (MIT mode) to identify which works.
"""
import sys
import time
import struct
import socket
import ctypes
import math

CAN_IFACE = "can_left"

# ── raw CAN helpers ──────────────────────────────────────────────────────────

AF_CAN   = 29
CAN_RAW  = 1
SOCK_RAW = 3

class CANFrame(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("can_id",  ctypes.c_uint32),
        ("can_dlc", ctypes.c_uint8),
        ("pad",     ctypes.c_uint8 * 3),
        ("data",    ctypes.c_uint8 * 8),
    ]

def open_can(iface):
    s = socket.socket(AF_CAN, SOCK_RAW, CAN_RAW)
    s.bind((iface,))
    s.settimeout(0.1)
    return s

def send_frame(s, can_id, data):
    f = CANFrame()
    f.can_id  = can_id & 0x1FFFFFFF
    f.can_dlc = 8
    for i, b in enumerate(data[:8]):
        f.data[i] = b
    s.send(bytes(f))

def recv_frames(s, n=10):
    frames = []
    for _ in range(n):
        try:
            raw = s.recv(16)
            f = CANFrame.from_buffer_copy(raw)
            frames.append((f.can_id, bytes(f.data[:f.can_dlc])))
        except socket.timeout:
            break
    return frames

# ── nerolib raw MIT frame builder ────────────────────────────────────────────

POS_MIN, POS_MAX = -12.5, 12.5
VEL_MIN, VEL_MAX = -45.0, 45.0
KP_MIN,  KP_MAX  = 0.0, 500.0
KD_MIN,  KD_MAX  = 0.0, 5.0
T_MIN,   T_MAX   = -18.0, 18.0

def float_to_int(val, mn, mx, bits):
    return int((val - mn) / (mx - mn) * ((1 << bits) - 1))

def mit_frame(motor_id, pos, vel, kp, kd, torque):
    pos_ref = float_to_int(pos,    POS_MIN, POS_MAX, 16)
    spd_ref = float_to_int(vel,    VEL_MIN, VEL_MAX, 12)
    kp_int  = float_to_int(kp,     KP_MIN,  KP_MAX,  12)
    kd_int  = float_to_int(kd,     KD_MIN,  KD_MAX,  12)
    t_ref   = float_to_int(torque, T_MIN,   T_MAX,   8)

    d0 = (pos_ref >> 8) & 0xFF
    d1 = pos_ref & 0xFF
    d2 = (spd_ref >> 4) & 0xFF
    d3 = (((spd_ref & 0xF) << 4) & 0xF0) | ((kp_int >> 8) & 0x0F)
    d4 = kp_int & 0xFF
    d5 = (kd_int >> 4) & 0xFF
    d6 = (((kd_int & 0xF) << 4) & 0xF0) | ((t_ref >> 4) & 0x0F)
    crc = (d0 ^ d1 ^ d2 ^ d3 ^ d4 ^ d5 ^ d6) & 0x0F
    d7  = ((t_ref << 4) & 0xF0) | crc
    return bytes([d0, d1, d2, d3, d4, d5, d6, d7])

# ── joint position frame builder (JOINT mode, 0x155-0x157, 0x170) ───────────

def joint_ctrl_frames(pos_rad):
    """Returns list of (can_id, data) for 7 joints in 0.001 deg format."""
    mdeg = [int(round(p * 180.0 / math.pi * 1000)) for p in pos_rad]
    def pack2(j1, j2):
        return struct.pack(">ii", j1, j2)
    return [
        (0x155, pack2(mdeg[0], mdeg[1])),
        (0x156, pack2(mdeg[2], mdeg[3])),
        (0x157, pack2(mdeg[4], mdeg[5])),
        (0x170, struct.pack(">i", mdeg[6]) + b'\x00' * 4),
    ]

# ── helpers ──────────────────────────────────────────────────────────────────

def read_joint_pos(s):
    """Read 0x251-0x257 frames for 0.5s and return joint positions (rad)."""
    pos = [None] * 7
    deadline = time.time() + 0.5
    while time.time() < deadline:
        for can_id, data in recv_frames(s, 20):
            if 0x251 <= can_id <= 0x257:
                idx = can_id - 0x251
                if len(data) >= 8:
                    pos[idx] = struct.unpack(">i", data[4:8])[0] / 1000.0
    return pos

def send_normal_mode(s):
    send_frame(s, 0x470, [0x00]*8)
    time.sleep(0.05)
    send_frame(s, 0x151, [0x01, 0xFF, 50, 0x00, 0x00, 0x00, 0x01, 0x00])
    time.sleep(0.2)

def send_enable(s):
    send_frame(s, 0x471, [0x08, 0x02, 0, 0, 0, 0, 0, 0])
    time.sleep(0.3)

def is_arm_enabled(s):
    enabled = [False] * 7
    deadline = time.time() + 0.5
    while time.time() < deadline:
        for can_id, data in recv_frames(s, 20):
            if 0x261 <= can_id <= 0x267:
                idx = can_id - 0x261
                if len(data) >= 6:
                    enabled[idx] = bool(data[5] & 0x40)
    return enabled

# ── main test ────────────────────────────────────────────────────────────────

def main():
    print(f"Opening CAN socket on {CAN_IFACE}...")
    s = open_can(CAN_IFACE)

    # ── Phase 1: Enable arm (mirroring pyAgxArm) ──────────────────────────
    print("\n=== Phase 1: Enable arm ===")
    for attempt in range(8):
        send_enable(s)
        enabled = is_arm_enabled(s)
        print(f"  Attempt {attempt+1}: per-joint enabled = {enabled}")
        if all(enabled):
            print("  All joints enabled!")
            break
        print("  Not all enabled, sending set_normal_mode...")
        send_normal_mode(s)
    else:
        print("  WARNING: failed to enable all joints after 8 attempts")

    # ── Phase 2: Read initial position ───────────────────────────────────
    print("\n=== Phase 2: Initial joint positions ===")
    pos0 = read_joint_pos(s)
    print(f"  pos = {[round(p,4) if p else None for p in pos0]}")
    if pos0[0] is None:
        print("  ERROR: no position feedback — arm not broadcasting 0x251-0x257")
        s.close()
        return

    # ── Phase 3A: Test JOINT mode (0x155-0x157) ──────────────────────────
    print("\n=== Phase 3A: JOINT mode (move_mode=0x01, mit_mode=0xAD) ===")
    send_frame(s, 0x151, [0x01, 0x01, 50, 0xAD, 0x00, 0x00, 0x01, 0x00])
    time.sleep(0.2)
    print("  Mode set. Sending target pos[0] = pos0[0] + 0.1 rad...")
    target = list(pos0)
    target[0] = pos0[0] + 0.1

    for _ in range(20):  # send for 2 seconds
        for can_id, data in joint_ctrl_frames(target):
            send_frame(s, can_id, data)
        time.sleep(0.1)

    pos_after_joint = read_joint_pos(s)
    delta_joint = (pos_after_joint[0] or 0) - (pos0[0] or 0)
    print(f"  pos after: {[round(p,4) if p else None for p in pos_after_joint]}")
    print(f"  joint[0] delta = {delta_joint:.4f} rad")

    if abs(delta_joint) > 0.01:
        print("  ✓ JOINT mode WORKS!")
    else:
        print("  ✗ JOINT mode did NOT move arm")

    # ── Phase 3B: Test MIT mode (0x15A-0x160) ────────────────────────────
    print("\n=== Phase 3B: MIT mode (move_mode=0x04, mit_mode=0xAD) ===")
    pos1 = read_joint_pos(s)
    send_frame(s, 0x151, [0x01, 0x04, 100, 0xAD, 0x00, 0x00, 0x01, 0x00])
    time.sleep(0.2)
    target_mit = (pos1[0] or 0) + 0.1
    print(f"  Mode set. Sending MIT target pos[0] = {target_mit:.4f} rad, kp=15, kd=0.5...")

    for _ in range(20):  # send for 2 seconds
        for motor_id in range(7):
            tgt = target_mit if motor_id == 0 else (pos1[motor_id] or 0.0)
            data = mit_frame(motor_id, tgt, 0.0, 15.0, 0.5, 0.0)
            send_frame(s, 0x15A + motor_id, data)
        time.sleep(0.1)

    pos_after_mit = read_joint_pos(s)
    delta_mit = (pos_after_mit[0] or 0) - (pos1[0] or 0)
    print(f"  pos after: {[round(p,4) if p else None for p in pos_after_mit]}")
    print(f"  joint[0] delta = {delta_mit:.4f} rad")

    if abs(delta_mit) > 0.01:
        print("  ✓ MIT mode WORKS!")
    else:
        print("  ✗ MIT mode did NOT move arm")

    # ── Disable ───────────────────────────────────────────────────────────
    print("\n=== Disabling arm ===")
    send_frame(s, 0x151, [0x00, 0x00, 0, 0, 0, 0, 0x02, 0x00])  # STANDBY
    time.sleep(0.1)
    send_frame(s, 0x471, [0x08, 0x01, 0, 0, 0, 0, 0, 0])  # disable
    time.sleep(0.3)
    s.close()
    print("Done.")

if __name__ == "__main__":
    main()
