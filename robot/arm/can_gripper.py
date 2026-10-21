import struct
import threading
import time
import can

GRIPPER_CMD_ID = 0x159
GRIPPER_FEEDBACK_ID = 0x2A8   # int32 BE width (um) | int16 force (mN) | status | mode
GRIPPER_STROKE_M = 0.07       # full travel; matches the 0.07 scale in _make_frame
FEEDBACK_STALE_S = 0.25       # treat feedback older than this as missing
EFFORT = 0.4  # 0-1, maps to 0-5 Nm; lower = less grip force

# The nominal position=0.0/1.0 (fully closed/open) endpoints land a little short
# of the actual mechanical hard stops, so a "closed"/"open" command leaves a
# visible gap. Command slightly past each endpoint on full-travel requests so
# the jaws drive fully into the hard stop; safe because torque is still capped
# by EFFORT. The two gripper units differ in how short they land, so these are
# tunable per CanGripper instance rather than shared constants.
CLOSE_OVERTRAVEL = 0.10
OPEN_OVERTRAVEL = 0.10


def _make_frame(
    position: float, status_code: int, effort: float = EFFORT, inverted: bool = True,
    close_overtravel: float = CLOSE_OVERTRAVEL, open_overtravel: float = OPEN_OVERTRAVEL,
) -> bytes:
    p = (1.0 - position) if inverted else position
    if position <= 0.0:
        p += close_overtravel if inverted else -close_overtravel
    elif position >= 1.0:
        p += -open_overtravel if inverted else open_overtravel
    pos_ref = int(p * 0.07 * 1e6)              # 0-1 → meters → micrometers
    effort_ref = int(effort * 5000)             # 0-1 → mNm
    return struct.pack(">iH", pos_ref, effort_ref) + bytes([status_code, 0x00])


class CanGripper:
    """200 Hz raw-CAN gripper controller for the Nero integrated gripper.

    nerolib must be configured with gripper_on=False so its loop doesn't
    conflict. This class owns all 0x159 traffic.
    """

    def __init__(
        self, can_port: str, inverted: bool = True,
        close_overtravel: float = CLOSE_OVERTRAVEL, open_overtravel: float = OPEN_OVERTRAVEL,
    ):
        self._can_port = can_port
        self._inverted = inverted
        self._close_overtravel = close_overtravel
        self._open_overtravel = open_overtravel
        self._target: float | None = None
        self._running = False
        self._bus: can.interface.Bus | None = None
        self._thread: threading.Thread | None = None
        self._rx_thread: threading.Thread | None = None
        self._measured: float | None = None
        self._measured_t = 0.0

    def _frame(self, position: float, status_code: int, effort: float = EFFORT) -> bytes:
        return _make_frame(
            position, status_code, effort=effort, inverted=self._inverted,
            close_overtravel=self._close_overtravel, open_overtravel=self._open_overtravel,
        )

    def enable(self):
        """Open the bus, kick with status=3 (enable+clear), start the loop."""
        if self._running:
            return
        # The filter only affects what this socket receives; we never need to
        # read anything but the gripper's own feedback frame.
        self._bus = can.interface.Bus(
            channel=self._can_port, interface="socketcan",
            can_filters=[{"can_id": GRIPPER_FEEDBACK_ID, "can_mask": 0x7FF, "extended": False}],
        )
        # Kick with ENABLE_AND_CLEAR_ERROR so the gripper activates immediately
        self._bus.send(can.Message(
            arbitration_id=GRIPPER_CMD_ID,
            data=self._frame(1.0, status_code=3),
            is_extended_id=False,
        ))
        time.sleep(0.05)
        self._target = 1.0
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def disable(self):
        """Stop the loop and send status=0 frames to let the gripper go limp."""
        self._target = None
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
            self._rx_thread = None
        if self._bus:
            for _ in range(10):
                try:
                    self._bus.send(can.Message(
                        arbitration_id=GRIPPER_CMD_ID,
                        data=self._frame(0.0, status_code=0, effort=0.0),
                        is_extended_id=False,
                    ))
                except Exception:
                    pass
                time.sleep(0.005)
            self._bus.shutdown()
            self._bus = None

    def set_position(self, pos: float):
        """Set gripper target (0.0 = closed, 1.0 = open)."""
        self._target = max(0.0, min(1.0, pos))

    def get_commanded(self) -> float:
        return self._target if self._target is not None else 0.0

    def get_measured(self) -> float | None:
        """Measured opening (0.0 = closed, 1.0 = open) from 0x2A8, or None if stale."""
        if self._measured is None or time.monotonic() - self._measured_t > FEEDBACK_STALE_S:
            return None
        return self._measured

    def get_position(self) -> float:
        """Measured opening when feedback is live, otherwise the commanded target."""
        measured = self.get_measured()
        return measured if measured is not None else self.get_commanded()

    def stop(self):
        self.disable()

    def _loop(self):
        while self._running:
            t = self._target
            if t is not None and self._bus:
                try:
                    self._bus.send(can.Message(
                        arbitration_id=GRIPPER_CMD_ID,
                        data=self._frame(t, status_code=1),
                        is_extended_id=False,
                    ))
                except Exception:
                    pass
            time.sleep(0.005)  # 200 Hz

    def _rx_loop(self):
        while self._running:
            bus = self._bus
            if bus is None:
                break
            try:
                msg = bus.recv(timeout=0.1)
            except Exception:
                continue
            if msg is None or msg.arbitration_id != GRIPPER_FEEDBACK_ID or len(msg.data) < 8:
                continue
            if msg.data[7] != 0x00:  # 0x01 = angle mode; we only command width
                continue
            width_m = struct.unpack(">i", bytes(msg.data[0:4]))[0] * 1e-6
            p = width_m / GRIPPER_STROKE_M
            pos = (1.0 - p) if self._inverted else p
            self._measured = max(0.0, min(1.0, pos))
            self._measured_t = time.monotonic()
