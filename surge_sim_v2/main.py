"""SURGE Mark.2 シミュレータ＆自律走行システム エントリポイント。

  python main.py --mode sim                              # シミュレーション
  python main.py --mode real                             # 実機
  python main.py --mode replay --log logs/xxxx.jsonl     # ログ再生

起動処理：
  1. --mode に応じてバックエンドを選択・初期化
  2. SharedState を生成
  3. Logger を生成
  4. FastAPI サーバーを別スレッドで起動（uvicorn）
  5. controller の制御ループを別スレッドで起動
  6. sim のみ pygame ウィンドウをメインスレッドで起動
  7. real は pygame を起動しない（FastAPI + 制御ループのみ）
  8. replay は LogReplayer を起動して SharedState に書き込む
"""
from __future__ import annotations

import argparse
import threading
from pathlib import Path

import yaml

from core.controller import Controller
from core.logger import Logger, LogReplayer
from core.shared_state import SharedState
from maps import get_all_courses
from server import context as server_context

ROOT = Path(__file__).resolve().parent
CONFIG_DIR = ROOT / "config"


def _load_yaml(name: str) -> dict:
    with open(CONFIG_DIR / name, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _start_server_thread(host: str, port: int) -> threading.Thread:
    import uvicorn
    from server.app import app

    config = uvicorn.Config(app, host=host, port=port, log_level="warning")
    server = uvicorn.Server(config)

    def _run() -> None:
        server.run()

    t = threading.Thread(target=_run, daemon=True)
    t.start()
    return t


def run_sim(localization_mode: str | None = None) -> None:
    sim_cfg = _load_yaml("sim.yaml")
    vehicle_cfg = _load_yaml("vehicle.yaml")["vehicle"]
    sim = sim_cfg["simulation"]
    srv = sim_cfg["server"]
    wd = sim_cfg.get("watchdog", {})
    loc_mode = localization_mode or sim_cfg.get("localization", {}).get("mode", "slam")

    courses = get_all_courses()
    current = {"id": "default_course" if "default_course" in courses else next(iter(courses))}
    cur_course = courses[current["id"]]

    shared = SharedState()
    logger = Logger(shared)

    from backend.sim.sim_backend import SimBackend
    backend = SimBackend(vehicle_cfg, sim, shared,
                         cur_course["walls"], cur_course["start_pose"])

    controller = Controller(backend, shared, logger,
                            command_timeout_ms=wd.get("command_timeout_ms", 500),
                            is_sim=True,
                            vehicle_cfg=vehicle_cfg,
                            planner_cfg=sim_cfg.get("planner", {}),
                            slam_cfg=sim_cfg.get("slam", {}),
                            racing_cfg=sim_cfg.get("racing_line", {}),
                            localization_mode=loc_mode,
                            saved_maps_dir=str(ROOT / "saved_maps"))
    controller.set_course(cur_course)   # 初期コース（slam時は経路は探索後に生成）

    # --- コース切替フック（sim） ---
    def on_course_change(cid: str) -> None:
        if cid not in courses:
            return
        current["id"] = cid
        c = courses[cid]
        backend.set_course(c["walls"], c["start_pose"])
        controller.set_course(c)        # Phase2: 追従経路を再構築
        shared.set_autonomous_running(False)

    def get_course_render() -> dict:
        c = courses[current["id"]]
        path = controller.path
        return {
            "walls": c["walls"],
            "center_line": c.get("center_line") or [],
            "racing_line": path.tolist() if path is not None else [],
        }

    ctx = server_context.AppContext(
        shared_state=shared, controller=controller, logger=logger,
        courses=courses, is_sim=True,
        saved_maps_dir=str(ROOT / "saved_maps"),
        on_course_change=on_course_change,
        on_map_save=lambda name: controller.save_map(name),
        on_map_reset=lambda: controller.reset_map(),
        on_map_load=lambda name: controller.load_map(name),
    )
    server_context.set_context(ctx)

    _start_server_thread(srv["host"], srv["port"])
    controller.start()

    print(f"[main] SIM 起動（自己位置推定: {loc_mode}）。"
          f"WebUI: http://localhost:{srv['port']}  WS: ws://localhost:{srv['port']}/ws")
    if loc_mode == "slam":
        print("[main] 実機相当モード：真値なし。MapBuilding で1周以上探索→地図構築後に Autonomous 可能。")

    # pygame をメインスレッドで起動
    from backend.sim.renderer import SimRenderer
    renderer = SimRenderer(
        config=sim, shared_state=shared,
        courses={cid: {"name": c["name"]} for cid, c in courses.items()},
        current_course_id=current["id"],
        get_course_render=get_course_render,
        callbacks={
            "course_change": on_course_change,
            "pause_toggle": lambda: shared.set_paused(not shared.is_paused()),
            "reset": lambda: backend.reset(),
            "speed_change": lambda m: shared.set_speed_multiplier(m),
        },
    )
    try:
        renderer.run()
    finally:
        controller.stop()
        print("[main] 終了")


def run_real(localization_mode: str | None = None) -> None:
    real_cfg = _load_yaml("real.yaml")
    vehicle_cfg = _load_yaml("vehicle.yaml")["vehicle"]
    srv = real_cfg["server"]
    wd = real_cfg.get("watchdog", {})
    uart = real_cfg["uart"]
    loc_mode = localization_mode or real_cfg.get("localization", {}).get("mode", "slam")

    shared = SharedState()
    logger = Logger(shared)

    from backend.real import RealBackend
    backend = RealBackend(uart["port"], uart.get("baudrate", 250000), shared)
    backend.start_watchdog()

    controller = Controller(backend, shared, logger,
                            command_timeout_ms=wd.get("command_timeout_ms", 500),
                            is_sim=False,
                            vehicle_cfg=vehicle_cfg,
                            planner_cfg=real_cfg.get("planner", {}),
                            slam_cfg=real_cfg.get("slam", {}),
                            racing_cfg=real_cfg.get("racing_line", {}),
                            localization_mode=loc_mode,
                            saved_maps_dir=str(ROOT / "saved_maps"))

    courses = get_all_courses()
    ctx = server_context.AppContext(
        shared_state=shared, controller=controller, logger=logger,
        courses=courses, is_sim=False, saved_maps_dir=str(ROOT / "saved_maps"),
        on_map_save=lambda name: controller.save_map(name),
        on_map_reset=lambda: controller.reset_map(),
        on_map_load=lambda name: controller.load_map(name),
    )
    server_context.set_context(ctx)

    _start_server_thread(srv["host"], srv["port"])
    controller.start()
    print(f"[main] REAL 起動。WebUI(遠隔): http://<raspi-ip>:{srv['port']}")
    print("[main] pygame は起動しません。Ctrl+C で終了。")
    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        controller.stop()
        backend.stop_watchdog()
        print("[main] 終了")


def run_replay(log_path: str) -> None:
    sim_cfg = _load_yaml("sim.yaml")
    sim = sim_cfg["simulation"]
    srv = sim_cfg["server"]

    shared = SharedState()
    logger = Logger(shared)
    courses = get_all_courses()
    current = {"id": "default_course" if "default_course" in courses else next(iter(courses))}

    # replay は backend なし。Controller は使わず LogReplayer が SharedState を駆動。
    class _NullController:
        backend = None

        def set_command(self, *a, **k): ...
        def emergency_stop(self): ...
        def set_mode(self, *a, **k): ...
        def start(self): ...
        def stop(self): ...

    replayer = LogReplayer(log_path, shared)

    ctx = server_context.AppContext(
        shared_state=shared, controller=_NullController(), logger=logger,  # type: ignore[arg-type]
        courses=courses, is_sim=True, saved_maps_dir=str(ROOT / "saved_maps"),
    )
    server_context.set_context(ctx)
    _start_server_thread(srv["host"], srv["port"])

    replayer.start(speed_multiplier=shared.get_speed_multiplier())
    print(f"[main] REPLAY 起動: {log_path}  WebUI: http://localhost:{srv['port']}")

    def get_course_render() -> dict:
        c = courses[current["id"]]
        return {"walls": c["walls"], "center_line": c.get("center_line") or [], "racing_line": []}

    from backend.sim.renderer import SimRenderer
    renderer = SimRenderer(
        config=sim, shared_state=shared,
        courses={cid: {"name": c["name"]} for cid, c in courses.items()},
        current_course_id=current["id"],
        get_course_render=get_course_render,
        callbacks={
            "course_change": lambda cid: current.update(id=cid),
            "pause_toggle": lambda: shared.set_paused(not shared.is_paused()),
            "reset": lambda: None,
            "speed_change": lambda m: shared.set_speed_multiplier(m),
        },
    )
    try:
        renderer.run()
    finally:
        replayer.stop()
        print("[main] 終了")


def main() -> None:
    parser = argparse.ArgumentParser(description="SURGE Mark.2 Simulator & Autonomous System")
    parser.add_argument("--mode", choices=["sim", "real", "replay"], default="sim")
    parser.add_argument("--log", type=str, default=None, help="replay 時のログファイル")
    parser.add_argument("--localization", choices=["slam", "cheat"], default=None,
                        help="自己位置推定モード（既定: 設定ファイル。slam=実機相当, cheat=真値）")
    args = parser.parse_args()

    if args.mode == "sim":
        run_sim(localization_mode=args.localization)
    elif args.mode == "real":
        run_real(localization_mode=args.localization)
    elif args.mode == "replay":
        if not args.log:
            parser.error("--mode replay には --log が必要です")
        run_replay(args.log)


if __name__ == "__main__":
    main()
