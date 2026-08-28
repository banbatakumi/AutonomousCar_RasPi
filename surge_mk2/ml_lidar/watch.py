"""ml_lidar/watch.py — 学習中の方策を複数パネルで眺めるビューア（pygame）。

    .venv/bin/python ml_lidar/watch.py
    .venv/bin/python ml_lidar/watch.py --panels 9 --model ml_lidar/runs/ppo_e2e/best_model.zip

`ml_lidar/train_rl.py` とは完全に別プロセス。**学習の`SubprocVecEnv`には一切触れない**
——訓練は速度優先でヘッドレスに回すべきもので、そこに毎ステップ描画を挟むと学習が
遅くなる。ここは学習と切り離した「観戦専用」の軽量な環境をパネル数ぶん自分で持ち、
数秒おきに学習中のチェックポイント（既定は`EvalCallback`が新記録のたびに上書きする
`best_model.zip`）を読み直して、そのときの方策で走らせて見せるだけ。

**パネル数は学習の並列数（`train_rl.py`の`--n-envs`）とは無関係。** 見たい数を
自由に選べる（学習は裏で何並列走っていても関係ない）。

見た目は `sim/ui.py`（`sim/gui.py`・`sim/editor.py`と共通の部品）をそのまま使う——
地図の再描画（`map_surface`）も姿勢マーカー（`arrow`）も車体色も新しく作らない。
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # surge_mk2/

import numpy as np  # noqa: E402
import pygame  # noqa: E402
from stable_baselines3 import PPO  # noqa: E402

from ml_lidar.env import GymSurgeEnv  # noqa: E402
from sim import ui  # noqa: E402
from sim.course import Course, DEFAULT_COURSE_DIR  # noqa: E402
from sim.random_course import generate_random_course  # noqa: E402

__all__ = ["Panel", "build_courses"]

TRAIL_LEN = 150


def build_courses(n: int, *, seed: int = 0) -> list[Course]:
    """`circuit`/`fuji`（評価に使う既知コース）を先頭に置き、残りをランダムで埋める。
    `train_rl.py`の評価コースと同じものを見せることで、「未知コースへの汎化」を
    そのまま目視できるようにしてある。"""
    known = [Course.load(DEFAULT_COURSE_DIR / "circuit.json"),
            Course.load(DEFAULT_COURSE_DIR / "fuji.json")]
    courses = known[:n]
    if n > len(courses):
        rng = np.random.default_rng(seed)
        courses += [generate_random_course(rng, name=f"rand{i}")
                   for i in range(n - len(courses))]
    return courses


class Panel:
    """1コース分の観戦用インスタンス。学習ワーカーとは無関係の独立した環境。"""

    def __init__(self, course: Course, *, max_steps: int, max_speed: float,
                max_steer: float, seed: int) -> None:
        self.course = course
        self.env = GymSurgeEnv([course], max_steps=max_steps, max_speed=max_speed,
                               max_steer=max_steer, seed=seed)
        self.obs, _ = self.env.reset()
        self.trail: deque[tuple[float, float]] = deque(maxlen=TRAIL_LEN)

    def step(self, model: PPO | None) -> None:
        if model is not None:
            action, _ = model.predict(self.obs, deterministic=True)
        else:
            # `GymSurgeEnv`の行動は[-1,1]正規化（0は速度レンジの中央=半速）なので、
            # 「モデル未取得の間は静止」には speed=-1（物理速度0への写像）を使う
            action = np.array([0.0, -1.0], dtype=np.float32)
        self.obs, _, terminated, truncated, _ = self.env.step(action)
        v = self.env.sim.vehicle
        self.trail.append((v.x, v.y))
        if terminated or truncated:
            self.obs, _ = self.env.reset()
            self.trail.clear()


def _draw_panel(screen: pygame.Surface, p: Panel, x0: int, y0: int, w: int, h: int,
                font: pygame.font.Font, px_per_m: float) -> None:
    """`px_per_m` は全パネル共通の縮尺。**パネルごとに引き伸ばして表示すると、
    5.5m四方のコースも9m四方のコースも同じ大きさに見えてしまう**（バンビの指摘で判明）
    ので、共通の縮尺で描いてから空いた余白を中央寄せする。"""
    pad = 4
    avail_w, avail_h = max(1, w - pad * 2), max(1, h - pad * 2 - 16)

    cw, chh = p.course.size_m
    dw = max(1, min(avail_w, round(cw * px_per_m)))
    dh = max(1, min(avail_h, round(chh * px_per_m)))
    ox_px = x0 + pad + (avail_w - dw) // 2
    oy_px = y0 + pad + (avail_h - dh) // 2

    surf = ui.map_surface(p.course, dw, dh, key=p.course.name)
    screen.blit(surf, (ox_px, oy_px))

    ox, oy = p.course.origin

    def to_px(x: float, y: float) -> tuple[float, float]:
        return (ox_px + (x - ox) / cw * dw, oy_px + dh - (y - oy) / chh * dh)

    if len(p.trail) > 1:
        pygame.draw.aalines(screen, ui.TRAIL, False, [to_px(x, y) for x, y in p.trail])

    v = p.env.sim.vehicle
    px, py = to_px(v.x, v.y)
    ui.arrow(screen, px, py, v.yaw, 10, ui.BAD if v.collided else ui.OK)

    screen.blit(font.render(p.course.name, True, ui.DIM), (x0 + pad, y0 + h - 16))
    pygame.draw.rect(screen, ui.HAIR, (x0, y0, w, h), width=1)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", type=Path, default=Path("ml_lidar/runs/ppo_e2e/best_model.zip"))
    ap.add_argument("--panels", type=int, default=6)
    ap.add_argument("--max-steps", type=int, default=1500)
    ap.add_argument("--max-speed", type=float, default=1.5)
    ap.add_argument("--max-steer", type=float, default=0.45)
    ap.add_argument("--reload-interval", type=float, default=3.0,
                    help="モデルの再読込を試みる間隔 [s]")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--window", type=int, default=960, help="ウィンドウの一辺 [px]")
    args = ap.parse_args()

    panels = [Panel(c, max_steps=args.max_steps, max_speed=args.max_speed,
                    max_steer=args.max_steer, seed=i)
             for i, c in enumerate(build_courses(args.panels))]

    pygame.init()
    pygame.display.set_caption("E2E LiDAR — 観戦ビューア")
    screen = pygame.display.set_mode((args.window, args.window), pygame.RESIZABLE)
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Menlo", 13)

    cols = math.ceil(math.sqrt(len(panels)))
    rows = math.ceil(len(panels) / cols)
    #: 全パネルで最大のコースがちょうどパネルに収まる縮尺を1回だけ決める。
    #: コース自体は起動後に変わらないので、毎フレーム計算し直す必要はない
    max_extent = max(max(p.course.size_m) for p in panels)

    model: PPO | None = None
    model_mtime = 0.0
    last_reload = 0.0

    running = True
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.KEYDOWN and e.key in (pygame.K_ESCAPE, pygame.K_q):
                running = False

        now = time.monotonic()
        if now - last_reload >= args.reload_interval:
            last_reload = now
            if args.model.exists():
                mtime = args.model.stat().st_mtime
                if mtime != model_mtime:
                    try:
                        loaded = PPO.load(str(args.model), device="cpu")
                        # ★観測の形が今の環境と合っているか確認する。合わないまま
                        # predict() を呼ぶと（例: 観測の次元を変える改修の前後で
                        # チェックポイントだけ古いままだと）ここではなく毎フレームの
                        # 描画ループ側で例外になり、ビューアごと落ちる
                        want = panels[0].env.observation_space.shape
                        if loaded.observation_space.shape != want:
                            print(f"# {args.model}: 観測の形が合わない "
                                  f"({loaded.observation_space.shape} != {want})。"
                                  f"古いチェックポイントの可能性——無視して待機します")
                            model_mtime = mtime    # 同じファイルで警告を連発しない
                        else:
                            model = loaded
                            model_mtime = mtime
                    except Exception:                          # noqa: BLE001
                        pass    # 書き込み中の中途半端なファイルを掴んだだけかもしれない

        for p in panels:
            p.step(model)

        w, h = screen.get_size()
        cell_w, cell_h = w // cols, h // rows
        # 余白(pad*2+ラベル16px)ぶんを引いた「地図に使える一辺」を基準に共通縮尺を出す
        px_per_m = (min(cell_w, cell_h) - 4 * 2 - 16) / max_extent
        screen.fill(ui.BG)
        for i, p in enumerate(panels):
            r, c = divmod(i, cols)
            _draw_panel(screen, p, c * cell_w, r * cell_h, cell_w, cell_h, font, px_per_m)

        status = ("モデル: 未取得（学習の最初の評価を待っています）" if model is None else
                  f"モデル: {args.model} （{time.strftime('%H:%M:%S', time.localtime(model_mtime))} 更新）")
        screen.blit(font.render(status, True, ui.DIM), (8, h - 18))

        pygame.display.flip()
        clock.tick(args.fps)

    pygame.quit()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
