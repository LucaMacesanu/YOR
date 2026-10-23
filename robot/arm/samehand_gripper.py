"""SameHand fingers as a drop-in for CanGripper (0.0 = closed, 1.0 = open).

YOR never opens the hand's USB port. The SameHand glove bridge owns it; this class talks to
the bridge over Zenoh (samehand/fingers/glove/cmd and .../state). Start the hand first:

    ~/SameHand-UMI/device/software/run.sh start calib bridge

Behaviour (see ~/SameHand-UMI/device/integration.md):
  - Limp until enable(). enable() holds the fingers where they are, then set_position()
    moves them. disable() sends a few enable=false commands and then goes silent.
  - While enabled, the current target is re-sent at `cmd_hz`; the firmware drops torque
    250 ms after the stream stops, so a crashed process leaves the fingers limp.
  - set_position(0.0) closes to the recorded grasp pose (touching, no squeeze) unless
    squeeze=True, which maps 0.0 to the full press (s = 1).
"""
import importlib.util
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np
import zenoh

ROOT = Path(os.environ.get("SAMEHAND_ROOT", Path.home() / "SameHand-UMI")) / "device" / "onboard"

COMMANDER_ID = 0x0E01  # YOR; SameHand's own Robot class is 0x0B07
PRIORITY = 2  # same as SameHand's Robot: neither preempts the other
STATE_FRESH_S = 0.25


def _load(name, path):
    # Load by path: SameHand's onboard/robot/ has a robot.py that would shadow YOR's
    # `robot` package if that directory went on sys.path.
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


pb = _load("samehand_glove_pb2", ROOT / "glove_bridge" / "glove_pb2.py")
grip = _load("samehand_grip", ROOT / "robot" / "grip.py")


class SameHandGripper:
    def __init__(self, tau_max=0.4, dq_max=1.5, slew=2.0, cmd_hz=100.0, squeeze=False,
                 device_id="glove", connect=None):
        """
        tau_max  effort ceiling, 0..1 of stall (grip force). 0.6 is SameHand's default;
                 0.4 keeps further from the servos' overcurrent trip for first runs.
        dq_max   servo speed cap, rad/s.
        slew     max change of the grip value s per second (s spans 0..1).
        squeeze  False: fully closed = touching (s = contact_s). True: fully closed = s = 1.
        connect  optional Zenoh endpoints, e.g. ["tcp/192.168.1.5:7447"], if the bridge
                 runs on another host.
        """
        self.g = grip.GripInterpolator.load()  # device/onboard/robot/grip_waypoints.json
        self.squeeze = bool(squeeze)
        self.s_max = 1.0 if squeeze else self.g.contact_s
        self.tau_max, self.dq_max, self.slew, self.hz = float(tau_max), dq_max, slew, cmd_hz

        self._lock = threading.Lock()  # state + targets
        self._pub_lock = threading.Lock()  # one publisher at a time (loop vs disable)
        self._q = None
        self._sts = None
        self._state = None  # last full State message, see telemetry()
        self._q_t = 0.0
        self._target = None  # grip value s (0 open .. s_max closed)
        self._cmd = None  # slew-limited s actually sent
        self._on = False
        self._seq = 0
        self._last_faults = None

        conf = zenoh.Config()
        if connect:
            import json
            conf.insert_json5("connect/endpoints", json.dumps(list(connect)))
        self._z = zenoh.open(conf)
        self._pub = self._z.declare_publisher(f"samehand/fingers/{device_id}/cmd")
        self._sub = self._z.declare_subscriber(f"samehand/fingers/{device_id}/state", self._on_state)
        self._run = True
        self._thread = threading.Thread(target=self._loop, name="samehand-cmd", daemon=True)
        self._thread.start()

    # ---------------------------------------------------------------- CanGripper API
    def enable(self, timeout=0.05):
        """Torque on, holding the current finger pose. Raises if the bridge isn't streaming.

        Short timeout on purpose: the bridge streams state at 200 Hz, so if it is up a fresh
        sample is already here. This runs inside yor.py's single-threaded RPC server, so a long
        wait here stalls every other call (arm targets included) while the bridge is down."""
        deadline = time.monotonic() + timeout
        q = self._fresh_q()
        while q is None:
            if time.monotonic() > deadline:
                raise RuntimeError("SameHand: no finger state. Is the bridge running? "
                                   "(~/SameHand-UMI/device/software/run.sh status)")
            time.sleep(0.02)
            q = self._fresh_q()
        s0 = min(self.g.s_from_q(q), self.s_max)
        with self._lock:
            self._cmd = s0
            self._target = s0
            self._on = True
            self._last_faults = None

    def disable(self):
        """Fingers limp: a few enable=false commands, then silence (never stream disables)."""
        with self._pub_lock:
            with self._lock:
                was_on = self._on
                self._on = False
                self._cmd = None
            if not was_on:
                return
            for _ in range(3):
                self._publish(None)
                time.sleep(0.02)

    def set_position(self, pos):
        """1.0 = open, 0.0 = closed. Only moves the fingers while enabled."""
        s = (1.0 - float(np.clip(pos, 0.0, 1.0))) * self.s_max
        with self._lock:
            self._target = s

    def get_commanded(self):
        with self._lock:
            t = self._target
        return 1.0 if t is None else 1.0 - t / self.s_max

    def get_measured(self):
        q = self._fresh_q()
        if q is None:
            return None
        return float(np.clip(1.0 - self.g.s_from_q(q, n=501) / self.s_max, 0.0, 1.0))

    def get_position(self):
        m = self.get_measured()
        return self.get_commanded() if m is None else m

    def stop(self):
        self.disable()
        self._run = False
        self._thread.join(timeout=1.0)
        try:
            self._sub.undeclare()
            self._pub.undeclare()
        except Exception:
            pass
        self._z.close()

    # ---------------------------------------------------------------- extras
    @property
    def enabled(self):
        return self._on

    def set_tau_max(self, tau_max):
        """Effort ceiling, 0..1 of stall; applies from the next command (<= 10 ms)."""
        self.tau_max = float(np.clip(tau_max, 0.05, 1.0))
        return self.tau_max

    def set_squeeze(self, squeeze):
        """True: fully closed = full press (s = 1); False: fully closed = touching (contact_s).
        The current target keeps its grip value s, so toggling never moves the fingers."""
        with self._lock:
            self.squeeze = bool(squeeze)
            self.s_max = 1.0 if self.squeeze else self.g.contact_s
            if self._target is not None:
                self._target = min(self._target, self.s_max)
        return self.squeeze

    def telemetry(self):
        """Last finger state from the bridge plus our command side; None if no fresh state."""
        with self._lock:
            st, t, target, cmd = self._state, self._q_t, self._target, self._cmd
        if st is None or time.monotonic() - t > STATE_FRESH_S:
            return None
        st = dict(st)
        st.update(enabled=self._on, tau_max=self.tau_max, squeeze=self.squeeze,
                  contact_s=float(self.g.contact_s), s_target=target, s_cmd=cmd)
        return st

    def faults(self):
        """Per-joint HLS status bits; nonzero = servo protection tripped (bit 3 = overcurrent)."""
        with self._lock:
            return None if self._sts is None else list(self._sts)

    # ---------------------------------------------------------------- internals
    def _fresh_q(self):
        with self._lock:
            if self._q is None or time.monotonic() - self._q_t > STATE_FRESH_S:
                return None
            return self._q.copy()

    def _on_state(self, sample):
        m = pb.State()
        m.ParseFromString(bytes(sample.payload))
        if len(m.actuators) < self.g.nq:
            return
        a = m.actuators[: self.g.nq]
        with self._lock:
            self._q = np.array([x.position for x in a])
            self._sts = [int(x.status) for x in a]
            self._state = {"t_us": int(m.t_us), "torque_on": bool(m.torque_on),
                           **{k: [float(getattr(x, k)) for x in a] for k in
                              ("position", "velocity", "effort", "current", "voltage", "temperature")},
                           "status": list(self._sts)}
            self._q_t = time.monotonic()

    def _publish(self, q):
        self._seq += 1
        m = pb.Command(seq=self._seq, enable=q is not None, priority=PRIORITY, commander_id=COMMANDER_ID)
        if q is not None:
            m.pos.extend(map(float, q))
            m.vel_max.extend([self.dq_max] * len(q))
            m.effort_max.extend([self.tau_max] * len(q))
        self._pub.put(m.SerializeToString())

    def _check_faults(self):
        f = self.faults()
        if f is not None and f != self._last_faults:
            if any(f):
                print(f"[SameHand] servo fault status {f} (bit 3 = overcurrent): grip may have released")
            elif self._last_faults is not None and any(self._last_faults):
                print("[SameHand] servo faults cleared")
            self._last_faults = f

    def _loop(self):
        dt = 1.0 / self.hz
        step = self.slew * dt
        nxt = time.monotonic()
        while self._run:
            nxt += dt
            time.sleep(max(0.0, nxt - time.monotonic()))
            nxt = max(nxt, time.monotonic() - dt)  # don't burst after a stall
            with self._pub_lock:
                with self._lock:
                    if not self._on or self._cmd is None:
                        continue  # idle = silent
                    self._cmd += float(np.clip(self._target - self._cmd, -step, step))
                    s = self._cmd
                self._publish(self.g(s))
            self._check_faults()
