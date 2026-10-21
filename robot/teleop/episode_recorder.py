"""Pi-side episode recorder for policy datasets (pi0.5 / LeRobot conversion).

Each episode is a directory keyed by the same episode_id that vr_teleop publishes
on episode/signal, so the Orin's camera recording for that episode can be joined
to it later:

    <root>/<episode_id>/
        meta.json      task prompt, session metadata, outcome, timing, schema
        frames.jsonl   one JSON object per control tick (the published episode frame)

frames.jsonl is streamed and flushed as it is written, so a crash loses at most
about a second. meta.json is written atomically at start ("status":
"in_progress") and rewritten at the end ("success" / "fail"); an episode still
marked in_progress after collection was interrupted and should be discarded.

Timestamps are Pi wall-clock seconds (time.time()); joining with Orin camera
frames relies on both machines being NTP-synced, which is recorded per episode.
"""

import json
import os
import subprocess
import time
from pathlib import Path

SCHEMA_VERSION = 1
FLUSH_EVERY = 30  # frames


def _git_commit() -> str | None:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"], cwd=Path(__file__).parent,
            stderr=subprocess.DEVNULL, text=True,
        ).strip() + ("-dirty" if subprocess.call(
            ["git", "diff", "--quiet"], cwd=Path(__file__).parent, stderr=subprocess.DEVNULL) else "")
    except Exception:
        return None


def _clock_synced() -> bool | None:
    try:
        out = subprocess.check_output(["timedatectl", "show", "-p", "NTPSynchronized", "--value"],
                                      stderr=subprocess.DEVNULL, text=True, timeout=2)
        return out.strip() == "yes"
    except Exception:
        return None


def _write_json_atomic(path: Path, data: dict):
    tmp = path.with_suffix(path.suffix + ".tmp")
    with open(tmp, "w") as f:
        json.dump(data, f, indent=2)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


class EpisodeRecorder:
    def __init__(self, root: str, session_meta: dict, control_hz: float):
        self.root = Path(root)
        self.session_meta = dict(session_meta)
        self.control_hz = control_hz
        self.git_commit = _git_commit()
        self._dir: Path | None = None
        self._frames = None
        self._meta: dict | None = None
        self._num_frames = 0
        self._zed_frames = 0

    @property
    def active(self) -> bool:
        return self._frames is not None

    def start(self, episode_id: str, started_at: float):
        if self.active:
            self.end(success=False, reason="superseded by a new episode")
        self._dir = self.root / episode_id
        self._dir.mkdir(parents=True, exist_ok=False)
        self._num_frames = 0
        self._zed_frames = 0
        self._meta = {
            "schema_version": SCHEMA_VERSION,
            "episode_id": episode_id,
            "status": "in_progress",
            **self.session_meta,
            "robot": "YOR",
            "recorded_on": "pi",
            "git_commit": self.git_commit,
            "control_hz": self.control_hz,
            "clock": "unix wall time (s), Pi",
            "clock_ntp_synced": _clock_synced(),
            "started_at": started_at,
        }
        _write_json_atomic(self._dir / "meta.json", self._meta)
        self._frames = open(self._dir / "frames.jsonl", "w")
        print(f"[Recorder] {self._dir}")

    def write(self, frame: dict):
        if not self.active:
            return
        if self._num_frames == 0:
            self._meta["frame_keys"] = sorted(frame.keys())
        self._frames.write(json.dumps(frame) + "\n")
        self._num_frames += 1
        if frame.get("base_pose") is not None:
            self._zed_frames += 1
        if self._num_frames % FLUSH_EVERY == 0:
            self._frames.flush()

    def end(self, success: bool, ended_at: float | None = None, reason: str | None = None):
        if not self.active:
            return
        self._frames.close()
        self._frames = None
        ended_at = ended_at if ended_at is not None else time.time()
        self._meta.update({
            "status": "success" if success else "fail",
            "success": success,
            "ended_at": ended_at,
            "duration_s": round(ended_at - self._meta["started_at"], 3),
            "num_frames": self._num_frames,
            "zed_pose_frames": self._zed_frames,
        })
        if reason:
            self._meta["end_reason"] = reason
        _write_json_atomic(self._dir / "meta.json", self._meta)
        print(f"[Recorder] saved {self._num_frames} frames ({self._meta['status']}) to {self._dir}")
