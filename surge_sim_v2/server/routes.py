"""REST API エンドポイント。"""
from __future__ import annotations

import time
from pathlib import Path

from fastapi import APIRouter
from pydantic import BaseModel

from core.interfaces import DriveMode
from core.logger import LogReplayer

from .context import get_context

router = APIRouter(prefix="/api")


class ModeBody(BaseModel):
    mode: str


class SpeedBody(BaseModel):
    multiplier: float


class TargetSpeedBody(BaseModel):
    target_speed: float | None = None


class MapNameBody(BaseModel):
    name: str


@router.post("/mode")
def set_mode(body: ModeBody):
    ctx = get_context()
    try:
        mode = DriveMode(body.mode)
    except ValueError:
        return {"ok": False, "error": f"unknown mode: {body.mode}"}
    ctx.controller.set_mode(mode)
    return {"ok": True, "mode": mode.value}


@router.post("/emergency_stop")
def emergency_stop():
    ctx = get_context()
    ctx.controller.emergency_stop()
    return {"ok": True}


@router.post("/autonomous/start")
def autonomous_start(body: TargetSpeedBody | None = None):
    ctx = get_context()
    target = body.target_speed if body is not None else None
    if hasattr(ctx.controller, "set_autonomous_target"):
        ctx.controller.set_autonomous_target(target)
    # Phase3: SLAM 地図があれば SLAM 由来経路に切り替える
    if hasattr(ctx.controller, "prepare_autonomous"):
        ctx.controller.prepare_autonomous()
    ctx.shared_state.set_emergency_stop(False)
    ctx.shared_state.set_autonomous_running(True)
    return {"ok": True, "autonomous_running": True, "target_speed": target}


@router.post("/autonomous/stop")
def autonomous_stop():
    ctx = get_context()
    ctx.shared_state.set_autonomous_running(False)
    return {"ok": True, "autonomous_running": False}


@router.post("/sim/pause")
def sim_pause():
    ctx = get_context()
    new_state = not ctx.shared_state.is_paused()
    ctx.shared_state.set_paused(new_state)
    return {"ok": True, "is_paused": new_state}


@router.post("/sim/reset")
def sim_reset():
    ctx = get_context()
    ctx.controller.backend.reset()
    ctx.shared_state.set_emergency_stop(False)
    return {"ok": True}


@router.post("/sim/speed")
def sim_speed(body: SpeedBody):
    ctx = get_context()
    ctx.shared_state.set_speed_multiplier(body.multiplier)
    return {"ok": True, "speed_multiplier": body.multiplier}


@router.post("/map/save")
def map_save(body: MapNameBody):
    ctx = get_context()
    if ctx.on_map_save is not None:
        path = ctx.on_map_save(body.name)
        return {"ok": True, "path": path}
    return {"ok": False, "error": "地図生成は Phase3 で実装"}


@router.post("/map/reset")
def map_reset():
    ctx = get_context()
    if ctx.on_map_reset is not None:
        ctx.on_map_reset()
        return {"ok": True}
    ctx.shared_state.update_slam_map(None)  # type: ignore[arg-type]
    return {"ok": True}


@router.post("/map/load")
def map_load(body: MapNameBody):
    ctx = get_context()
    if ctx.on_map_load is not None:
        ok = ctx.on_map_load(body.name)
        if ok is False:
            return {"ok": False, "error": f"地図が見つかりません: {body.name}"}
        return {"ok": True}
    return {"ok": False, "error": "地図読み込みは Phase3 で実装"}


@router.get("/maps")
def list_maps():
    ctx = get_context()
    d = Path(ctx.saved_maps_dir)
    if not d.exists():
        return {"maps": []}
    maps = sorted(p.stem for p in d.glob("*.npz")) + \
        sorted(p.stem for p in d.glob("*.json"))
    return {"maps": maps}


@router.post("/log/start")
def log_start():
    ctx = get_context()
    path = ctx.logger.start_recording()
    return {"ok": True, "path": path}


@router.post("/log/stop")
def log_stop():
    ctx = get_context()
    ctx.logger.stop_recording()
    return {"ok": True}


@router.get("/logs")
def list_logs():
    return {"logs": [Path(p).name for p in LogReplayer.get_available_logs()]}


@router.get("/courses")
def list_courses():
    ctx = get_context()
    return {
        "courses": [
            {"id": cid, "name": c["name"]} for cid, c in ctx.courses.items()
        ]
    }


@router.post("/course")
def set_course(body: MapNameBody):
    ctx = get_context()
    if body.name not in ctx.courses:
        return {"ok": False, "error": f"unknown course: {body.name}"}
    if ctx.on_course_change is not None:
        ctx.on_course_change(body.name)
    return {"ok": True, "course": body.name}
