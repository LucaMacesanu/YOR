import struct
import threading
import time
import can

GRIPPER_CMD_ID = 0x159
GRIPPER_FEEDBACK_ID = 0x2A8   # int32 BE width (um) | int16 force (mN) | status | mode
GRIPPER_STROKE_M = 0.07       # full travel; matches the 0.07 scale in _make_frame
FEEDBACK_STALE_S = 0.25       # treat feedback older than this as missing
STATUS_DRIVER_ENABLED = 0x40  # 0x2A8 byte 6, bit 6 (nerolib GripperStatus.driver_enable_status)
DISABLE_AND_CLEAR_ERROR = 0x02  # plain DISABLE (0x00) was ignored by the right gripper on 2026-10-23
DISABLE_TRIES = 5
DISABLE_CONFIRM_S = 0.1       # wait this long for the enable bit to drop before resending
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
        self._running_rx = False
        self._bus: can.interface.Bus | None = None
        self._thread: threading.Thread | None = None
        self._rx_thread: threading.Thread | None = None
        self._measured: float | None = None
        self._measured_t = 0.0
        self._width_um: int | None = None  # raw width from the last 0x2A8 frame
        self._status: int | None = None    # status byte from the last 0x2A8 frame
        self._status_t = 0.0

    def _frame(self, position: float, status_code: int, effort: float = EFFORT) -> bytes:
        return _make_frame(
            position, status_code, effort=effort, inverted=self._inverted,
            close_overtravel=self._close_overtravel, open_overtravel=self._open_overtravel,
        )

    def _open_bus(self):
        if self._bus is None:
            # The filter only affects what this socket receives; we never need to
            # read anything but the gripper's own feedback frame.
            self._bus = can.interface.Bus(
                channel=self._can_port, interface="socketcan",
                can_filters=[{"can_id": GRIPPER_FEEDBACK_ID, "can_mask": 0x7FF, "extended": False}],
            )

    def _start_rx(self):
        """Read 0x2A8 feedback from now until close(), limp or not."""
        self._open_bus()
        if self._rx_thread is None:
            self._running_rx = True
            self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._rx_thread.start()

    def enable(self):
        """Open the bus, kick with status=3 (enable+clear), start the loop."""
        if self._running:
            return
        self._start_rx()
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

    def disable(self) -> bool:
        """Let the gripper go limp. Returns True once its feedback confirms the driver is off.

        The gripper moves to the position in a disable frame (nerolib's set_gripper notes
        this too), so the frame carries the measured width: a disable never moves the jaws.
        Works even when the gripper was left enabled with our loop stopped. Feedback keeps
        being read afterwards (get_measured() stays live); close() releases the bus."""
        last_target = self._target if self._target is not None else 1.0
        self._target = None
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        try:
            if self._rx_thread is None:  # need feedback to confirm
                self._start_rx()
                time.sleep(DISABLE_CONFIRM_S)
        except Exception as e:
            print(f"[CanGripper] {self._can_port}: can't open bus to disable ({e})")
            return False
        confirmed = False
        for _ in range(DISABLE_TRIES):
            if self._width_um is not None and time.monotonic() - self._measured_t <= FEEDBACK_STALE_S:
                data = struct.pack(">iH", self._width_um, 0) + bytes([DISABLE_AND_CLEAR_ERROR, 0x00])
            else:  # no feedback: last commanded opening, kept off the overtravel endpoints
                data = self._frame(min(0.99, max(0.01, last_target)), DISABLE_AND_CLEAR_ERROR, effort=0.0)
            try:
                self._bus.send(can.Message(arbitration_id=GRIPPER_CMD_ID, data=data, is_extended_id=False))
            except Exception as e:
                print(f"[CanGripper] {self._can_port}: disable frame send failed ({e})")
            t_sent = time.monotonic()
            while time.monotonic() - t_sent < DISABLE_CONFIRM_S:
                time.sleep(0.01)
                if (self._status_t > t_sent and self._status is not None
                        and not self._status & STATUS_DRIVER_ENABLED):
                    confirmed = True
                    break
            if confirmed:
                break
        if not confirmed:
            print(f"[CanGripper] {self._can_port}: gripper did not confirm disable (status {self._status})")
        return confirmed

    def close(self):
        """Go limp, stop reading feedback and release the bus."""
        self.disable()
        self._running_rx = False
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
            self._rx_thread = None
        if self._bus:
            self._bus.shutdown()
            self._bus = None

    def is_enabled(self) -> bool:
        """Our command loop is running (the gripper is being held at a target)."""
        return self._running

    def set_position(self, pos: float):
        """Set gripper target (0.0 = closed, 1.0 = open)."""
        self._target = max(0.0, min(1.0, pos))

    def get_commanded(self) -> float | None:
        """Commanded opening (0.0 = closed, 1.0 = open), or None while limp (nothing commanded)."""
        return self._target

    def get_measured(self) -> float | None:
        """Measured opening (0.0 = closed, 1.0 = open) from 0x2A8; None exactly when
        feedback_error() is not None."""
        return None if self.feedback_error() is not None else self._measured

    def feedback_error(self) -> str | None:
        """None while get_measured() is live, otherwise why it is None."""
        if self._rx_thread is None:
            return f"{self._can_port}: gripper feedback not being read"
        if self._measured is None:
            return f"{self._can_port}: no 0x2A8 gripper feedback received"
        age = time.monotonic() - self._measured_t
        if age > FEEDBACK_STALE_S:
            return f"{self._can_port}: gripper feedback stale ({age:.2f} s old)"
        return None

    def get_position(self) -> float:
        """Measured opening when feedback is live, otherwise the commanded target (1.0 if limp)."""
        measured = self.get_measured()
        if measured is not None:
            return measured
        return self._target if self._target is not None else 1.0

    def stop(self):
        self.close()

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
        while self._running_rx:
            bus = self._bus
            if bus is None:
                break
            try:
                msg = bus.recv(timeout=0.1)
            except Exception:
                continue
            if msg is None or msg.arbitration_id != GRIPPER_FEEDBACK_ID or len(msg.data) < 8:
                continue
            self._status, self._status_t = msg.data[6], time.monotonic()
            if msg.data[7] != 0x00:  # 0x01 = angle mode; we only command width
                continue
            self._width_um = struct.unpack(">i", bytes(msg.data[0:4]))[0]
            width_m = self._width_um * 1e-6
            p = width_m / GRIPPER_STROKE_M
            pos = (1.0 - p) if self._inverted else p
            self._measured = max(0.0, min(1.0, pos))
            self._measured_t = time.monotonic()
