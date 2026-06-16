"""SURGE Mark.2 シミュレータ エントリポイント。

使い方:
    python main.py --mode sim                  # ローカルpygameシミュレータ
    python main.py --mode sim --serve          # ヘッドレス＋WebSocket配信
    python main.py --mode sim --serve --port 8000
    python main.py --mode sim --course "L-Shape Course"
    python main.py --mode real                 # 実機（Phase以降で実装）
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

CONFIG_DIR = Path(__file__).parent / "config"


def load_yaml(name: str) -> dict:
    with open(CONFIG_DIR / name, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _build_sim(course_name: str | None):
    """SIMスタックを構築して共通部品を返す。

    Returns:
        dict: backend, controller, courses, scene_holder, get_view,
              on_course_change, get_scene, start_name, merged_cfg
    """
    from backend.sim.sim_backend import SimBackend
    from core.controller import Controller
    from core.telemetry import build_scene, build_view
    from maps import get_all_courses

    vehicle_cfg = load_yaml("vehicle.yaml")
    sim_cfg = load_yaml("sim.yaml")

    courses = get_all_courses()
    if not courses:
        raise RuntimeError("コースが見つかりません (maps/)。")

    if course_name and course_name in courses:
        start_name = course_name
    else:
        start_name = "Default Rectangle" if "Default Rectangle" in courses \
            else next(iter(courses))
    current = courses[start_name]

    merged_cfg = dict(vehicle_cfg)
    merged_cfg.update(sim_cfg)

    backend = SimBackend(vehicle_cfg, sim_cfg, current)
    controller = Controller(backend, merged_cfg,
                            control_hz=1.0 / sim_cfg["simulation"]["dt"])
    controller.set_course(current)

    scene_holder = {"scene": build_scene("sim", walls=backend.walls,
                                         center_line=current.get("center_line"))}

    def get_scene():
        # 運用UI(Web)向け＝実機相当。真の壁・カンニング中心線は載せず、
        # SLAMで構築・導出したもの（占有格子は別binary、中心線/走行ライン）のみ。
        return build_scene("sim", walls=None, center_line=None,
                           racing_line=controller.racing_line,
                           slam_center=controller.slam_center)

    def get_view():
        # pygameデバッグビューア向け＝真の壁・真値を含むシーン
        return build_view(controller.get_snapshot(), scene_holder["scene"],
                          source="sim", include_truth=True)

    def on_course_change(name: str) -> None:
        if name not in courses:
            return
        backend.load_course(courses[name])
        controller.set_course(courses[name])
        controller.reset()
        scene_holder["scene"] = build_scene("sim", walls=backend.walls,
                                            center_line=courses[name].get("center_line"))

    return {
        "backend": backend, "controller": controller, "courses": courses,
        "scene_holder": scene_holder, "get_view": get_view, "get_scene": get_scene,
        "on_course_change": on_course_change, "start_name": start_name,
        "merged_cfg": merged_cfg,
    }


def run_sim(course_name: str | None) -> int:
    """ローカルpygameでシミュレータを起動する。"""
    from backend.sim.renderer import SimRenderer

    try:
        s = _build_sim(course_name)
    except RuntimeError as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        return 1

    controller = s["controller"]
    renderer = SimRenderer(
        config=s["merged_cfg"],
        course_names=list(s["courses"].keys()),
        get_view=s["get_view"],
        controller=controller,
    )
    renderer.set_current_course(s["start_name"])

    print("[INFO] pygameはビューア専用です。操作するには --serve でWeb UIを使ってください。")
    controller.start()
    try:
        renderer.run()
    finally:
        controller.stop()
    return 0


def run_serve(course_name: str | None, host: str, port: int,
              window: bool = False) -> int:
    """シミュレータを動かしWebSocket配信する。window=True で pygame も同時表示。"""
    from backend.telemetry_server import TelemetryServer

    try:
        s = _build_sim(course_name)
    except RuntimeError as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        return 1

    controller = s["controller"]
    ui_dir = Path(__file__).parent / "ui_web"
    server = TelemetryServer(
        controller=controller,
        get_scene=s["get_scene"],
        source="sim",
        on_course_change=s["on_course_change"],
        get_courses=lambda: list(s["courses"].keys()),
        get_grid=lambda: controller.occupancy,   # SLAM占有格子(binary配信)
        host=host, port=port,
        static_dir=str(ui_dir) if ui_dir.is_dir() else None,
    )

    controller.start()

    if window:
        # サーバをバックグラウンドスレッド、pygameをメインスレッドで同時実行
        import threading
        import uvicorn
        from backend.sim.renderer import SimRenderer

        uconfig = uvicorn.Config(server.app, host=host, port=port, log_level="warning")
        userver = uvicorn.Server(uconfig)
        threading.Thread(target=userver.run, daemon=True).start()
        print(f"[INFO] Web+pygame 同時起動: http://{host}:{port}/  (ws:/ws)")

        renderer = SimRenderer(
            config=s["merged_cfg"], course_names=list(s["courses"].keys()),
            get_view=s["get_view"], controller=controller,
        )
        renderer.set_current_course(s["start_name"])
        try:
            renderer.run()
        finally:
            userver.should_exit = True
            controller.stop()
        return 0

    print(f"[INFO] TelemetryServer 起動: http://{host}:{port}/  (ws:/ws, Ctrl+Cで停止)")
    try:
        server.run()
    finally:
        controller.stop()
    return 0


def run_real() -> int:
    print("[INFO] 実機モードはPhase以降で実装予定です（backend/real.py スタブ）。")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="SURGE Mark.2 Simulator")
    parser.add_argument("--mode", choices=["sim", "real"], default="sim")
    parser.add_argument("--course", default=None, help="起動時のコース名")
    parser.add_argument("--serve", action="store_true",
                        help="WebSocket配信で起動（既定はヘッドレス）")
    parser.add_argument("--window", action="store_true",
                        help="--serve と併用でpygameウィンドウも同時表示")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()

    if args.mode == "real":
        return run_real()
    if args.serve:
        return run_serve(args.course, args.host, args.port, window=args.window)
    return run_sim(args.course)


if __name__ == "__main__":
    sys.exit(main())
