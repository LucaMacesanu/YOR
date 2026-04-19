import struct
import threading
import time
import can

GRIPPER_CMD_ID = 0x159
EFFORT = 0.5  # 0-1, maps to 0-5 Nm; 0.5 gave reliable actuation in testing


def _make_frame(position: float, status_code: int, effort: float = EFFORT) -> bytes:
    pos_ref = int(position * 0.07 * 1e6)       # 0-1 → meters → micrometers
    effort_ref = int(effort * 5000)             # 0-1 → mNm
    return struct.pack(">iH", pos_ref, effort_ref) + bytes([status_code, 0x00])


class CanGripper:
    """200 Hz raw-CAN gripper controller for the Nero integrated gripper.

    nerolib must be configured with gripper_on=False so its loop doesn't
    conflict. This class owns all 0x159 traffic.
    """

    def __init__(self, can_port: str):
        self._can_port = can_port
        self._target: float | None = None
        self._running = False
        self._bus: can.interface.Bus | None = None
        self._thread: threading.Thread | None = None

    def enable(self):
        """Open the bus, kick with status=3 (enable+clear), start the loop."""
        if self._running:
            return
        self._bus = can.interface.Bus(channel=self._can_port, interface="socketcan")
        # Kick with ENABLE_AND_CLEAR_ERROR so the gripper activates immediately
        self._bus.send(can.Message(
            arbitration_id=GRIPPER_CMD_ID,
            data=_make_frame(1.0, status_code=3),
            is_extended_id=False,
        ))
        time.sleep(0.05)
        self._target = 1.0
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def disable(self):
        """Stop the loop and send status=0 frames to let the gripper go limp."""
        self._target = None
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._bus:
            for _ in range(10):
                try:
                    self._bus.send(can.Message(
                        arbitration_id=GRIPPER_CMD_ID,
                        data=_make_frame(0.0, status_code=0, effort=0.0),
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

    def get_position(self) -> float:
        return self._target if self._target is not None else 0.0

    def stop(self):
        self.disable()

    def _loop(self):
        while self._running:
            t = self._target
            if t is not None and self._bus:
                try:
                    self._bus.send(can.Message(
                        arbitration_id=GRIPPER_CMD_ID,
                        data=_make_frame(t, status_code=1),
                        is_extended_id=False,
                    ))
                except Exception:
                    pass
            time.sleep(0.005)  # 200 Hz
