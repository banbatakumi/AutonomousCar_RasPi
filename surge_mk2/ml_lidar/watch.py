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
from functools import partial
from pathlib import Path
from typing import Callable

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # surge_mk2/

import numpy as np  # noqa: E402
import pygame  # noqa: E402
from stable_baselines3 import PPO  # noqa: E402

from ml_lidar.env import GymSurgeEnv  # noqa: E402
from ml_lidar.export_onnx_rl import load_run_config_defaults  # noqa: E402
from sim import ui  # noqa: E402
from sim.course import Course, DEFAULT_COURSE_DIR  # noqa: E402
from sim.random_course import generate_diverse_course  # noqa: E402

__all__ = ["Panel", "build_panels"]

TRAIL_LEN = 150


def build_panels(n: int, *, max_steps: int, max_speed: float, steer_tau: float,
                 steer_rate_weight: float, seed: int = 0) -> list["Panel"]:
    """`circuit`/`fuji`（評価に使う既知コース、幅1.0m固定）を先頭に置き、残りは
    **`train_rl.py`の学習と全く同じ`generate_diverse_course`**（形状・道幅に加えて
    アーキタイプ=organic/circuit/corridor/narrow/obstacleも毎エピソード引き直す。
    2026-08-31、コース多様化）で埋める。

    以前は`generate_random_course`（幅1.0m固定）を起動時に1回だけ生成し、以後
    ずっと同じコースを使い回していたため、`circuit`/`fuji`はもちろんランダム
    埋めのコースまで含めて**観戦ビューアには常に幅1.0mの道しか映らなかった**
    （2026-08-29、バンビの指摘で発覚）。実際の学習が経験している「毎エピソード
    形も道幅も変わる」感覚に近づけるため、ランダム埋めのパネルは`course_fn`
    （`Panel`が`env.reset()`のたびに新しいコースを作る）に切り替えた。
    `circuit`/`fuji`はこれまで通り固定のまま——`train_rl.py`の評価コースと
    同じものを見せることで「未知コース形状への汎化」を目視できる意図は変えない。
    """
    known = [("circuit", Course.load(DEFAULT_COURSE_DIR / "circuit.json")),
            ("fuji", Course.load(DEFAULT_COURSE_DIR / "fuji.json"))][:n]
    panels = [Panel(label, course=c, max_steps=max_steps, max_speed=max_speed,
                    steer_tau=steer_tau, steer_rate_weight=steer_rate_weight, seed=i)
             for i, (label, c) in enumerate(known)]
    for i in range(len(known), n):
        label = f"rand{i}"
        panels.append(Panel(label, course_fn=partial(generate_diverse_course, name=label),
                            max_steps=max_steps, max_speed=max_speed, steer_tau=steer_tau,
                            steer_rate_weight=steer_rate_weight, seed=seed + i))
    return panels


class Panel:
    """1コース分の観戦用インスタンス。学習ワーカーとは無関係の独立した環境。

    `course`（固定）と`course_fn`（`env.reset()`のたびに新しいコースを作る）は
    `SimE2EEnv`と同じく排他——どちらか一方を渡す。`course_fn`を渡した場合、
    コースはエピソードごとに変わるので**`self.course`は固定属性ではなくプロパティ**
    にしてある（常に今のエピソードのコースを指す）。
    """

    def __init__(self, label: str, *, course: Course | None = None,
                course_fn: Callable[[np.random.Generator], Course] | None = None,
                max_steps: int, max_speed: float, steer_tau: float,
                steer_rate_weight: float, seed: int) -> None:
        self.label = label
        self.env = GymSurgeEnv([course] if course is not None else None,
                               course_fn=course_fn, max_steps=max_steps,
                               max_speed=max_speed, steer_tau=steer_tau,
                               steer_rate_weight=steer_rate_weight, seed=seed)
        self.obs, _ = self.env.reset()
        self.trail: deque[tuple[float, float]] = deque(maxlen=TRAIL_LEN)
        #: コースが変わるたびに増える（`_draw_panel`の地図キャッシュキーに使う。
        #: `course_fn`パネルは同じ`label`のまま中身が変わるので、`label`だけを
        #: キーにすると古い地図が描画され続けてしまう）
        self.episode = 0

    @property
    def course(self) -> Course:
        return self.env.sim.course

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
            self.episode += 1


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

    # `key`は`p.episode`込み——`course_fn`パネルは`p.label`が同じまま中身（形・道幅）
    # が変わるので、`label`だけをキーにすると前のエピソードの地図が残ってしまう
    surf = ui.map_surface(p.course, dw, dh, key=f"{p.label}-{p.episode}")
    screen.blit(surf, (ox_px, oy_px))

    ox, oy = p.course.origin

    def to_px(x: float, y: float) -> tuple[float, float]:
        return (ox_px + (x - ox) / cw * dw, oy_px + dh - (y - oy) / chh * dh)

    # ★obstacle/narrowアーキタイプ（`sim/random_course.py`）は壁と同系色の
    # グレーで焼き込まれるだけなので、このサムネイル解像度だと肉眼では
    # ほぼ判別できない（2026-08-31、バンビの指摘）。学習環境そのもの
    # （車体の走行ではなく「今このコースに何があるか」）が一目で分かるよう、
    # `Course.obstacles`/`Course.width`(配列)から直接ハイライトを重ねて描く
    if p.course.obstacles is not None:
        for obs_x, obs_y, obs_r in p.course.obstacles:
            ox_, oy_ = to_px(float(obs_x), float(obs_y))
            r_px = max(3, round(float(obs_r) * px_per_m))
            pygame.draw.circle(screen, ui.BAD, (round(ox_), round(oy_)), r_px, width=2)

    if isinstance(p.course.width, np.ndarray):
        narrow_mask = p.course.width < float(p.course.width.max()) - 1e-6
        for nx_, ny_ in p.course.centerline[narrow_mask, :2]:
            px_, py_ = to_px(float(nx_), float(ny_))
            pygame.draw.circle(screen, ui.WARN, (round(px_), round(py_)), 2)

    if len(p.trail) > 1:
        pygame.draw.aalines(screen, ui.TRAIL, False, [to_px(x, y) for x, y in p.trail])

    v = p.env.sim.vehicle
    px, py = to_px(v.x, v.y)
    ui.arrow(screen, px, py, v.yaw, 10, ui.BAD if v.collided else ui.OK)

    course_width = p.course.width
    w_txt = (f"{course_width.min():.2f}-{course_width.max():.2f}"
             if isinstance(course_width, np.ndarray) else f"{(course_width or 1.0):.2f}")
    label = f"{p.label} 幅{w_txt}m"
    screen.blit(font.render(label, True, ui.DIM), (x0 + pad, y0 + h - 16))
    pygame.draw.rect(screen, ui.HAIR, (x0, y0, w, h), width=1)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", type=Path, default=Path("ml_lidar/runs/ppo_e2e/best_model.zip"))
    ap.add_argument("--panels", type=int, default=6)
    ap.add_argument("--max-steps", type=int, default=None,
                    help="省略時は--modelと同じディレクトリのrun_config.json"
                         "（train_rl.pyが書く）から読む。無ければ1500")
    ap.add_argument("--max-speed", type=float, default=None,
                    help="省略時はrun_config.jsonから読む。無ければ1.5"
                         "（★ここが学習時の値とズレると、モデルの速度出力を"
                         "違う物理レンジで解釈してしまう。`export_onnx_rl.py`の"
                         "--max-speedと同じ注意点）")
    ap.add_argument("--steer-tau", type=float, default=None,
                    help="省略時はrun_config.jsonから読む。無ければ0.10"
                         "（`SimE2EEnv`の既定と同じ）")
    ap.add_argument("--steer-rate-weight", type=float, default=None,
                    help="省略時はrun_config.jsonから読む。無ければ0.2"
                         "（`SimE2EEnv`の既定と同じ）")
    ap.add_argument("--reload-interval", type=float, default=3.0,
                    help="モデルの再読込を試みる間隔 [s]")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--window", type=int, default=960, help="ウィンドウの一辺 [px]")
    args = ap.parse_args()

    # ★観戦環境を学習環境になるべく揃える（2026-08-29、バンビの指摘で追加）。
    # `--model`と同じディレクトリの`run_config.json`（`train_rl.py`が書く）から
    # 実際にその run が使った値を拾う。CLI指定があればそちらを優先
    defaults = load_run_config_defaults(args.model)
    max_steps = args.max_steps if args.max_steps is not None else int(defaults.get("max_steps", 1500))
    max_speed = args.max_speed if args.max_speed is not None else float(defaults.get("max_speed", 1.5))
    steer_tau = args.steer_tau if args.steer_tau is not None else float(defaults.get("steer_tau", 0.10))
    steer_rate_weight = (args.steer_rate_weight if args.steer_rate_weight is not None
                         else float(defaults.get("steer_rate_weight", 0.2)))
    print(f"# max_steps={max_steps} max_speed={max_speed} steer_tau={steer_tau} "
          f"steer_rate_weight={steer_rate_weight}"
          f"{'（run_config.jsonから）' if defaults else '（run_config.json無し、既定値）'}")

    panels = build_panels(args.panels, max_steps=max_steps, max_speed=max_speed,
                          steer_tau=steer_tau, steer_rate_weight=steer_rate_weight)

    pygame.init()
    pygame.display.set_caption("E2E LiDAR — 観戦ビューア")
    screen = pygame.display.set_mode((args.window, args.window), pygame.RESIZABLE)
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Menlo", 13)

    cols = math.ceil(math.sqrt(len(panels)))
    rows = math.ceil(len(panels) / cols)

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
        # ★`rand*`パネルは`course_fn`でエピソードごとにコースが変わる（形も道幅も）
        # ため、`max_extent`はもう起動時の1回だけでは済まず毎フレーム出し直す
        # （2026-08-29。パネル数は多くても9個程度なので負荷は無視できる）
        max_extent = max(max(p.course.size_m) for p in panels)
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
