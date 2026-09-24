"""ml_lidar/live_watch.py — 学習中のrunをリアルタイムに観戦する（複数コース並列表示）。

    .venv/bin/python -m ml_lidar.live_watch --run-name v1

`ml_lidar/watch.py`（エクスポート済みONNX＋実車の`raspi/auto/e2e_lidar.py`を通す、
実車投入前の最終確認用）とは別物。こちらは学習中のSB3チェックポイント（`.zip`）を
`PPO.predict()`で直接動かす軽量版——ONNXエクスポート＋パリティ検証のコストを
毎回払わずに、学習の進み具合をリアルタイムで覗くための道具に振っている。
**実車投入前の最終確認には引き続き`watch.py`を使うこと**（推論経路の忠実性が違う）。

## 既存の成果物を自動で追う（観戦専用の保存は増やさない）

`train_rl.py`の`EvalCallback`が既定`--eval-freq`2万step間隔で更新する
`eval/best_model.zip`と、`checkpoints/`（resume用、既定10万step間隔）の
`find_latest_checkpoint()`のうち、**実際にmtimeが新しい方**を使う
（`train_rl.py`の`find_watch_source()`）。`best_model.zip`は「評価で改善した
ときだけ」更新されるので、改善が止まっている間は`checkpoints/`の方が
新しくなることもある——両方見て新しい方を使えば、観戦用に専用の保存機構を
足さずに済む。`--poll-interval-s`（既定5秒）間隔でこの2つのmtimeをポーリングし、
更新されていれば読み込み直す。書き込み中のzipを掴んでロードに失敗しても
例外を投げず、次のポーリングで再試行する。

## 複数コースを同じ画面で並列表示

`train_rl.py`の`EvalCallback`が使うのと同じアーキタイプ（`EVAL_COURSE_PARAMS`の全て）を
1画面に並べる。学習の「今の実力」を、`best_model`だけに頼らず複数の形状で目視できる。

並べ方はウィンドウの大きさで決める（`_best_grid()`）。起動時は画面の約9割の大きさで開き、
リサイズのたびに「1枚が最も大きくなる行×列」を選び直す（横長なら2行×5列、縦長なら
3行×3列など）。タイトルの文字もパネルの大きさに合わせて縮める。

## 学習を邪魔しないための配慮

- 表示するコースは`EVAL_COURSE_PARAMS`の本数に連動する（増やすほどCPUを食う）
- モデルの読み込み直し自体はONNXエクスポートよりずっと軽い（SB3 zipの
  デシリアライズのみ、パリティ検証も無し）
- とはいえ学習プロセスとCPUを取り合うことに変わりはない。ラップトップ1台で
  学習と同時に見ると、学習側のfpsは多少落ちる。気になるなら`--interval-ms`を
  上げて描画頻度を下げること

## 障害物パネル（障害物ありで学習したrunだけ）

`env_config.json`の`obstacle_prob>0`なら、固定コースに静的障害物を刻んだパネルを
`_OBSTACLE_PANELS`の本数だけ足す（`ml_lidar/obstacles.py`の`add_obstacles`、
通過可能性を確認済みの配置）。障害物なしのパネルは残すので、回避の様子と
障害物なしの走りを並べて見られる。配置はコースのシードで決めるので毎回同じ。

理想ライン（MCL）は2026-09-24に表示をやめた。学習には一切使っておらず
（報酬は`progress`＋衝突のみ）、しかも広いコースや障害物のあるコースでは
理想ラインが実際に通れる経路を表せない（PROGRESS.md 2026-09-24節）ため、
観戦画面で比較の基準に見えるのは誤解のもとになる。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ML_LIDAR_DIR = Path(__file__).resolve().parent
REPO_ROOT = ML_LIDAR_DIR.parent
sys.path.insert(0, str(REPO_ROOT))

import numpy as np  # noqa: E402
from gymnasium.wrappers import TimeLimit  # noqa: E402
from stable_baselines3 import PPO  # noqa: E402

from sim.vehicle import VehicleSpec  # noqa: E402

from ml_lidar.env import EnvConfig, LidarE2EEnv  # noqa: E402
from ml_lidar.obstacles import add_obstacles  # noqa: E402
from ml_lidar.train_rl import EVAL_COURSE_PARAMS, RUNS_DIR, find_watch_source, \
    load_saved_env_config  # noqa: E402
from ml_lidar.viz_support import catchup_step_count, set_japanese_font  # noqa: E402

__all__ = ["Panel", "LiveWatcher", "parse_args", "main"]

DEFAULT_POLL_INTERVAL_S = 5.0
DEFAULT_INTERVAL_MS = 150
#: 障害物を刻んで追加表示するeval固定コースの名前（狭い1.0mと広い2.2mを1本ずつ）。
#: 1枚増えるごとに全パネルが小さくなるので、道幅の両端の2本に絞ってある
_OBSTACLE_PANELS: tuple[str, ...] = ("medium", "wide")


class Panel:
    """1コースぶんの表示状態。`LidarE2EEnv`をそのまま1本持って進める。"""

    def __init__(self, name: str, course, env_cfg: EnvConfig, max_episode_steps: int) -> None:
        self.name = name
        self.base_env = LidarE2EEnv(env_cfg, course=course)
        self.env = TimeLimit(self.base_env, max_episode_steps=max_episode_steps)
        self.obs, _ = self.env.reset()
        self.status = "待機中"

    def step(self, model: PPO) -> None:
        action, _ = model.predict(self.obs, deterministic=True)
        self.obs, _reward, terminated, truncated, info = self.env.step(action)
        if info["collided"]:
            self.status = "衝突"
        elif terminated:
            self.status = "周回達成"
        elif truncated:
            self.status = "打ち切り"
        else:
            self.status = "走行中"
        if terminated or truncated:
            self.obs, _ = self.env.reset()


class LiveWatcher:
    """`find_watch_source()`（観戦用の軽量スナップショット→無ければcheckpoints）を
    定期ポーリングし、最新モデルで全パネルを進める。"""

    def __init__(self, run_dir: Path, panels: list[Panel], poll_interval_s: float) -> None:
        self.run_dir = run_dir
        self.panels = panels
        self.poll_interval_s = poll_interval_s
        self.model: PPO | None = None
        self.model_path: Path | None = None
        self.model_mtime: float | None = None
        self._last_poll = 0.0
        self.reload()

    def reload(self) -> bool:
        found = find_watch_source(self.run_dir)
        if found is None:
            return False
        path, mtime = found
        # `live/latest.zip`は同じパスへ上書きされ続けるので、パス一致だけでは
        # 「更新されたか」を判定できない。mtimeが進んでいるかで見る
        if path == self.model_path and self.model_mtime is not None and mtime <= self.model_mtime:
            return False
        try:
            model = PPO.load(str(path), device="cpu")
        except Exception:                                              # noqa: BLE001
            # 学習プロセスが書き込み中のzipを掴んだ可能性。次のpollで再試行する
            return False
        self.model = model
        self.model_path = path
        self.model_mtime = mtime
        return True

    def maybe_reload(self, now: float) -> bool:
        if now - self._last_poll < self.poll_interval_s:
            return False
        self._last_poll = now
        return self.reload()

    def step_all(self) -> None:
        if self.model is not None:
            for panel in self.panels:
                panel.step(self.model)


def _build_panels(env_cfg: EnvConfig, max_episode_steps: int) -> list[Panel]:
    """eval固定コース（`EVAL_COURSE_PARAMS`）ぶんのパネルを作る。障害物ありで学習した
    runなら、`_OBSTACLE_PANELS`のコースに障害物を刻んだパネルを後ろに足す。

    ★固定コースを渡した`LidarE2EEnv`は障害物を置かない（`obstacle_prob`を無視する）ので、
    ここで刻んでから渡す。`build_course()`は毎回新しい`Course`を返すので、障害物なしの
    パネルと共有して汚すことはない
    """
    panels = [Panel(spec.name, spec.build_course(), env_cfg, max_episode_steps)
              for spec in EVAL_COURSE_PARAMS]
    if env_cfg.obstacle_prob > 0.0:
        for spec in EVAL_COURSE_PARAMS:
            if spec.name not in _OBSTACLE_PANELS:
                continue
            course = spec.build_course()
            add_obstacles(course, np.random.default_rng(spec.seed), env_cfg.max_obstacles)
            panels.append(Panel(f"{spec.name}+障害物", course, env_cfg, max_episode_steps))
    return panels


#: 図全体のタイトル（suptitle）ぶんに取っておく高さ [inch]
_SUPTITLE_H_IN = 0.45
#: 起動時のウィンドウを画面の何割にするか（メニューバー・Dockぶん縦は控えめ）
_SCREEN_FRAC = (0.92, 0.85)
_FALLBACK_FIGSIZE_IN = (12.0, 8.0)


def _best_grid(n: int, width: float, height: float, panel_aspect: float = 1.0) -> tuple[int, int]:
    """`n`枚のパネルを`width×height`の領域に並べたとき、1枚の大きさが最大になる`(行, 列)`。

    パネルは縦横比`panel_aspect`（幅/高さ）で描かれる（`set_aspect("equal")`のコースは
    ほぼ正方形）ので、セルの幅と高さのうち律速する側で1枚の大きさが決まる。
    同じ大きさなら空き枠の少ない方を選ぶ。
    """
    best: tuple[float, int, int, int] | None = None
    for cols in range(1, n + 1):
        rows = -(-n // cols)
        cell_w, cell_h = width / cols, height / rows
        scale = min(cell_w / panel_aspect, cell_h)
        key = (round(scale, 6), -(rows * cols - n), rows, cols)
        if best is None or key[:2] > best[:2]:
            best = key
    return best[2], best[3]


def _panel_area(fig) -> tuple[float, float]:
    """パネルに使える図の領域 [inch]（suptitleぶんを除く）。"""
    w, h = fig.get_size_inches()
    return float(w), max(0.1, float(h) - _SUPTITLE_H_IN)


def _title_fontsize(fig, rows: int, cols: int) -> float:
    """パネル1枚の大きさに比例したタイトルの文字サイズ [pt]。
    「medium+障害物 | 走行中 | v=1.23m/s」（約30字）がセル幅に収まる程度に縮める。"""
    w, h = _panel_area(fig)
    cell_in = min(w / cols, h / rows)
    # 約30字×文字幅0.6em ≒ 18em。係数3.0ならセル幅の約75%に収まる
    return float(np.clip(cell_in * 3.0, 6.0, 12.0))


def _initial_figsize(dpi: float) -> tuple[float, float]:
    """画面に収まる初期ウィンドウの大きさ [inch]。画面サイズはtkinterで取る
    （matplotlibのバックエンドに依らない。macOSの既定`macosx`にも画面サイズを問う口が無い）。"""
    try:
        import tkinter
        root = tkinter.Tk()
        root.withdraw()
        sw, sh = root.winfo_screenwidth(), root.winfo_screenheight()
        root.destroy()
    except Exception:                                                  # noqa: BLE001
        return _FALLBACK_FIGSIZE_IN
    return sw * _SCREEN_FRAC[0] / dpi, sh * _SCREEN_FRAC[1] / dpi


def run(args: argparse.Namespace) -> None:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.patches import Circle, Polygon

    set_japanese_font()
    plt.style.use("dark_background")

    run_dir = RUNS_DIR / args.run_name
    env_cfg = load_saved_env_config(run_dir)
    if env_cfg is None:
        print(f"!! {run_dir} にenv_config.jsonが無い（学習がまだ始まっていないのでは）",
             file=sys.stderr)
        raise SystemExit(1)
    max_episode_steps = max(1, int(round(args.episode_s / env_cfg.dt)))

    panels = _build_panels(env_cfg, max_episode_steps)
    watcher = LiveWatcher(run_dir, panels, poll_interval_s=args.poll_interval_s)
    if watcher.model is None:
        print(f"!! {run_dir} にまだeval/best_model.zipもcheckpointsも無い。"
             "学習が最初の--eval-freq（既定2万step）か--checkpoint-freqへ"
             "到達するまで待つこと", file=sys.stderr)

    footprint = np.asarray(VehicleSpec.load().footprint)

    # 画面に収まる大きさで開き、ウィンドウのリサイズのたびに行×列を選び直す。
    # 固定の3列だと9枚で縦12.6インチになり、Macの画面(高さ956pt)に収まらなかった。
    # 余白・タイトル位置はconstrained_layoutが再計算する
    fig = plt.figure(figsize=_initial_figsize(plt.rcParams["figure.dpi"]), layout="constrained")
    layout = [_best_grid(len(panels), *_panel_area(fig))]
    gs = fig.add_gridspec(*layout[0])
    axes_flat = [fig.add_subplot(gs[i // layout[0][1], i % layout[0][1]])
                 for i in range(len(panels))]
    fig.suptitle(f"ml_lidar ライブ観戦 — run={args.run_name}")

    artists = []
    for ax, panel in zip(axes_flat, panels):
        ax.set_aspect("equal")
        # 座標の目盛りは観戦に不要。小さい画面では目盛りの余白が地図を圧迫する
        ax.set_xticks([])
        ax.set_yticks([])
        course = panel.base_env.course
        h, w = course.grid.shape
        extent = (course.origin[0], course.origin[0] + w * course.resolution,
                 course.origin[1], course.origin[1] + h * course.resolution)
        ax.imshow(course.grid, extent=extent, origin="lower", cmap="Greys",
                 vmin=0, vmax=1, alpha=0.6)
        # 障害物は格子にも刻まれているが、半径5cm程度だと灰色の点に埋もれるので輪郭を重ねる
        if course.obstacles is not None:
            for ox, oy, r in course.obstacles:
                ax.add_patch(Circle((ox, oy), r, facecolor="none", edgecolor="tab:orange",
                                    linewidth=1.5, zorder=3))
        body_poly = Polygon(np.zeros((4, 2)), closed=True, facecolor="tab:blue",
                            edgecolor="white", zorder=5)
        ax.add_patch(body_poly)
        scatter = ax.scatter([], [], s=3, c="tab:red", zorder=4)
        title = ax.set_title(panel.name)
        artists.append((body_poly, scatter, title))

    def _relayout(_event=None) -> None:
        rows, cols = _best_grid(len(panels), *_panel_area(fig))
        if (rows, cols) != layout[0]:
            layout[0] = (rows, cols)
            new_gs = fig.add_gridspec(rows, cols)
            for i, ax in enumerate(axes_flat):
                ax.set_subplotspec(new_gs[i // cols, i % cols])
        size = _title_fontsize(fig, rows, cols)
        for _, _, title in artists:
            title.set_fontsize(size)
        fig.canvas.draw_idle()

    _relayout()
    fig.canvas.mpl_connect("resize_event", _relayout)

    # 直前フレームからの実経過時間ぶんだけシムを進める（fixed-timestep-with-catchup）。
    # 「毎フレーム必ず1step」だと、描画・学習プロセスとのCPU競合でフレーム間隔が
    # 伸びた瞬間だけ見かけの速度が遅くなり、次に間隔が戻ると急に追いつく——
    # これが「かくつく」の正体（`ml_lidar/watch.py`の`_update()`と同じ対策）
    last_time = [time.monotonic()]

    def _update(_frame):
        now = time.monotonic()
        real_dt = now - last_time[0]
        last_time[0] = now
        n_steps = catchup_step_count(real_dt, env_cfg.dt)

        watcher.maybe_reload(now)
        for _ in range(n_steps):
            watcher.step_all()

        model_label = watcher.model_path.stem if watcher.model_path else "未読込"
        fig.suptitle(f"ml_lidar ライブ観戦 — run={args.run_name} — model={model_label}")

        for panel, (body_poly, scatter, title) in zip(panels, artists):
            v = panel.base_env.vehicle
            c, s = np.cos(v.yaw), np.sin(v.yaw)
            rot = np.array([[c, -s], [s, c]])
            body_poly.set_xy(footprint @ rot.T + np.array([v.x, v.y]))

            scan = panel.base_env._scan
            if scan is not None:
                pts = []
                for deg in range(0, 360, 4):
                    d = scan.dist[deg]
                    if d > 0:
                        a = v.yaw + np.radians(deg)
                        pts.append((v.x + d * np.cos(a), v.y + d * np.sin(a)))
                scatter.set_offsets(np.asarray(pts) if pts else np.empty((0, 2)))

            title.set_text(f"{panel.name} | {panel.status} | v={v.speed:.2f}m/s")
        return [a for tup in artists for a in tup]

    anim = FuncAnimation(fig, _update, interval=max(1, args.interval_ms), blit=False)
    plt.show()


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="ml_lidar 学習中runのライブ観戦（複数コース並列）")
    p.add_argument("--run-name", required=True)
    p.add_argument("--episode-s", type=float, default=20.0)
    p.add_argument("--poll-interval-s", type=float, default=DEFAULT_POLL_INTERVAL_S,
                   help="checkpoints/の最新ファイルを確認する間隔[s]")
    p.add_argument("--interval-ms", type=int, default=DEFAULT_INTERVAL_MS,
                   help="描画更新間隔[ms]。大きいほど滑らかさは落ちるがCPU負荷が下がる")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    run(parse_args(argv))


if __name__ == "__main__":
    main()
