"""SharedState から WebSocket へ SystemState を配信する（20Hz）。

- SystemState を JSON シリアライズして broadcast_hz で全クライアントへ送信
- クライアントからの command / ping / emergency_stop メッセージを受け取る
"""
from __future__ import annotations

import asyncio
import time

import numpy as np
from fastapi import WebSocket

from core.interfaces import SystemState


def _vehicle_json(v) -> dict:
    return {
        "x": round(v.x, 4), "y": round(v.y, 4), "heading": round(v.heading, 2),
        "speed": round(v.speed, 3), "acceleration": round(v.acceleration, 3),
        "steer_angle": round(v.steer_angle, 2), "timestamp": v.timestamp,
    }


def _lidar_json(s) -> dict:
    return {
        "distances": np.asarray(s.distances).round(3).tolist(),
        "angles": np.asarray(s.angles).round(1).tolist(),
        "timestamp": s.timestamp,
    }


def _loc_json(l) -> dict:
    return {
        "x": round(l.x, 4), "y": round(l.y, 4), "heading": round(l.heading, 2),
        "confidence": round(l.confidence, 3), "source": l.source,
        "timestamp": l.timestamp,
    }


def _grid_json(g) -> dict | None:
    if g is None:
        return None
    return {
        "grid": np.asarray(g.grid).astype(int).tolist(),
        "resolution": g.resolution,
        "origin_x": g.origin_x,
        "origin_y": g.origin_y,
    }


def _course_json(cm) -> dict | None:
    if cm is None:
        return None

    def _pts(a):
        return np.asarray(a).round(3).tolist() if a is not None and len(a) else []

    return {
        "left_wall": _pts(cm.left_wall),
        "right_wall": _pts(cm.right_wall),
        "center_line": _pts(cm.center_line),
        "racing_line": _pts(cm.racing_line),
    }


def _conn_json(c) -> dict:
    return {
        "websocket_connected": c.websocket_connected,
        "last_received_at": c.last_received_at,
        "uart_connected": c.uart_connected,
        "lidar_receiving": c.lidar_receiving,
        "stm32_connected": c.stm32_connected,
        "latency_ms": round(c.latency_ms, 2),
    }


def serialize_state(state: SystemState) -> dict:
    """SystemState を配信用 JSON 辞書に変換する。"""
    return {
        "type": "state",
        "mode": state.mode.value,
        "vehicle": _vehicle_json(state.vehicle),
        "lidar": _lidar_json(state.lidar),
        "localization": _loc_json(state.localization),
        "slam_map": _grid_json(state.slam_map),
        "course_map": _course_json(state.course_map),
        "connection": _conn_json(state.connection),
        "is_paused": state.is_paused,
        "is_recording": state.is_recording,
        "speed_multiplier": state.speed_multiplier,
        "autonomous_running": state.autonomous_running,
        "timestamp": state.timestamp,
    }


class ConnectionManager:
    """接続中の WebSocket クライアントを束ねる。"""

    def __init__(self) -> None:
        self._clients: set[WebSocket] = set()
        self._lock = asyncio.Lock()

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        async with self._lock:
            self._clients.add(ws)

    async def disconnect(self, ws: WebSocket) -> None:
        async with self._lock:
            self._clients.discard(ws)

    @property
    def client_count(self) -> int:
        return len(self._clients)

    async def broadcast(self, message: dict) -> None:
        async with self._lock:
            targets = list(self._clients)
        dead = []
        for ws in targets:
            try:
                await ws.send_json(message)
            except Exception:  # noqa: BLE001
                dead.append(ws)
        if dead:
            async with self._lock:
                for ws in dead:
                    self._clients.discard(ws)


class Broadcaster:
    """20Hz で SharedState を全クライアントへ配信するループ。"""

    def __init__(self, shared_state, manager: ConnectionManager,
                 broadcast_hz: float = 20.0) -> None:
        self.shared = shared_state
        self.manager = manager
        self.period = 1.0 / broadcast_hz
        self._task: asyncio.Task | None = None
        self._running = False

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._task = asyncio.create_task(self._run())

    async def _run(self) -> None:
        while self._running:
            start = time.perf_counter()
            if self.manager.client_count > 0:
                state = self.shared.get_system_state()
                # WebSocket 接続フラグを反映
                state.connection.websocket_connected = True
                await self.manager.broadcast(serialize_state(state))
            elapsed = time.perf_counter() - start
            await asyncio.sleep(max(0.0, self.period - elapsed))

    def stop(self) -> None:
        self._running = False
        if self._task is not None:
            self._task.cancel()
