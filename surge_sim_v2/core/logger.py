"""走行ログの記録・再生。

WebUI の「REC」ボタンで記録開始・停止する。
フォーマット: JSON Lines（1行1フレーム）。保存先: logs/YYYYMMDD_HHMMSS.jsonl。
再生は記録データを SharedState に書き込むことで実現する。
"""
from __future__ import annotations

import json
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from .interfaces import (
    ConnectionStatus,
    DriveMode,
    LidarScan,
    LocalizationResult,
    OccupancyGrid,
    SystemState,
    VehicleState,
)
from .shared_state import SharedState

LOG_DIR = Path(__file__).resolve().parent.parent / "logs"


def _vehicle_to_dict(v: VehicleState) -> dict:
    return {
        "x": v.x, "y": v.y, "heading": v.heading, "speed": v.speed,
        "acceleration": v.acceleration, "steer_angle": v.steer_angle,
        "timestamp": v.timestamp,
    }


def _lidar_to_dict(s: LidarScan) -> dict:
    return {
        "distances": np.asarray(s.distances).round(4).tolist(),
        "angles": np.asarray(s.angles).round(2).tolist(),
        "timestamp": s.timestamp,
    }


def _loc_to_dict(l: LocalizationResult) -> dict:
    return {
        "x": l.x, "y": l.y, "heading": l.heading,
        "confidence": l.confidence, "source": l.source, "timestamp": l.timestamp,
    }


def serialize_frame(state: SystemState) -> dict:
    return {
        "mode": state.mode.value,
        "vehicle": _vehicle_to_dict(state.vehicle),
        "lidar": _lidar_to_dict(state.lidar),
        "localization": _loc_to_dict(state.localization),
        "timestamp": state.timestamp,
    }


class Logger:
    def __init__(self, shared_state: SharedState | None = None) -> None:
        self._shared = shared_state
        self._fh = None
        self._path: str | None = None
        self._lock = threading.Lock()

    def start_recording(self) -> str:
        with self._lock:
            if self._fh is not None:
                return self._path or ""
            LOG_DIR.mkdir(parents=True, exist_ok=True)
            name = datetime.now().strftime("%Y%m%d_%H%M%S") + ".jsonl"
            self._path = str(LOG_DIR / name)
            self._fh = open(self._path, "w", encoding="utf-8")
        if self._shared is not None:
            self._shared.set_recording(True)
        return self._path

    def stop_recording(self) -> None:
        with self._lock:
            if self._fh is not None:
                self._fh.flush()
                self._fh.close()
                self._fh = None
        if self._shared is not None:
            self._shared.set_recording(False)

    def record_frame(self, state: SystemState) -> None:
        with self._lock:
            if self._fh is None:
                return
            self._fh.write(json.dumps(serialize_frame(state)) + "\n")

    def is_recording(self) -> bool:
        with self._lock:
            return self._fh is not None


class LogReplayer:
    def __init__(self, log_path: str, shared_state: SharedState) -> None:
        self.log_path = log_path
        self.shared = shared_state
        self._thread: threading.Thread | None = None
        self._running = False

    @staticmethod
    def get_available_logs() -> list[str]:
        if not LOG_DIR.exists():
            return []
        return sorted(str(p) for p in LOG_DIR.glob("*.jsonl"))

    def _load_frames(self) -> list[dict]:
        frames: list[dict] = []
        with open(self.log_path, "r", encoding="utf-8") as fh:
            for line in fh:
                line = line.strip()
                if line:
                    frames.append(json.loads(line))
        return frames

    def start(self, speed_multiplier: float = 1.0) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._run, args=(speed_multiplier,), daemon=True
        )
        self._thread.start()

    def _run(self, speed_multiplier: float) -> None:
        frames = self._load_frames()
        if not frames:
            self._running = False
            return
        prev_ts = frames[0]["timestamp"]
        for f in frames:
            if not self._running:
                break
            dt = max(0.0, f["timestamp"] - prev_ts) / max(speed_multiplier, 1e-6)
            prev_ts = f["timestamp"]
            time.sleep(min(dt, 1.0))

            v = f["vehicle"]
            self.shared.update_vehicle(VehicleState(
                v["x"], v["y"], v["heading"], v["speed"],
                v["acceleration"], v["steer_angle"], v["timestamp"]))
            ld = f["lidar"]
            self.shared.update_lidar(LidarScan(
                np.array(ld["distances"], dtype=float),
                np.array(ld["angles"], dtype=float), ld["timestamp"]))
            lo = f["localization"]
            self.shared.update_localization(LocalizationResult(
                lo["x"], lo["y"], lo["heading"], lo["confidence"],
                lo["source"], lo["timestamp"]))
            self.shared.set_mode(DriveMode(f.get("mode", "manual")))
        self._running = False

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
