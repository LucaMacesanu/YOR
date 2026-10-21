"""
Probe the teleop episode publisher (vr_teleop.py / collect_nav_data.py).
Run while a publisher is active; with vr_teleop, start an episode first
(right thumbstick click).

Usage:
    python robot/teleop/probe_episode_5558.py                 # on the Pi
    python robot/teleop/probe_episode_5558.py --host <pi-ip>  # from another machine
"""
import argparse
import json

import zmq

from robot.config import EPISODE_PUB_PORT

parser = argparse.ArgumentParser()
parser.add_argument("--host", default="localhost", help="Address of the machine running the publisher")
parser.add_argument("--port", type=int, default=EPISODE_PUB_PORT)
parser.add_argument("-n", type=int, default=50, help="Number of frames to print")
args = parser.parse_args()

ctx = zmq.Context()
sock = ctx.socket(zmq.SUB)
sock.setsockopt(zmq.RCVTIMEO, 5000)
sock.connect(f"tcp://{args.host}:{args.port}")
for topic in (b"teleop/episode", b"episode/signal"):
    sock.setsockopt(zmq.SUBSCRIBE, topic)

print(f"Waiting for frames on tcp://{args.host}:{args.port} ...")
frames = 0
last_seq = None
while frames < args.n:
    try:
        topic, payload = sock.recv_multipart()
    except zmq.Again:
        print("  (no frames for 5 s)")
        continue
    msg = json.loads(payload)
    if topic == b"episode/signal":
        print(f"[signal] {msg}")
        continue
    if frames == 0:
        print("\nKeys:", sorted(msg.keys()))
        print("\nFirst frame:")
        for k, v in sorted(msg.items()):
            print(f"  {k}: {v}")
    else:
        seq = msg.get("seq")
        gap = "" if last_seq is None or seq is None or seq == last_seq + 1 else f"  <-- GAP (prev {last_seq})"
        fmt = lambda xs: None if xs is None else [round(x, 4) for x in xs]
        print(f"  ep={msg.get('episode_id')} seq={seq} cmd_vel={fmt(msg.get('cmd_vel'))} "
              f"enc_vel={fmt(msg.get('enc_vel'))}{gap}")
    last_seq = msg.get("seq")
    frames += 1
