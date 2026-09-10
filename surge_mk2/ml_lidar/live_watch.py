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

## 学習を邪魔しないための配慮

- 表示するコースは`EVAL_COURSE_PARAMS`の本数に連動する（増やすほどCPUを食う）
- モデルの読み込み直し自体はONNXエクスポートよりずっと軽い（SB3 zipの
  デシリアライズのみ、パリティ検証も無し）
- とはいえ学習プロセスとCPUを取り合うことに変わりはない。ラップトップ1台で
  学習と同時に見ると、学習側のfpsは多少落ちる。気になるなら`--interval-ms`を
  上げて描画頻度を下げること
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
from ml_lidar.train_rl import EVAL_COURSE_PARAMS, RUNS_DIR, find_watch_source, \
    load_saved_env_config  # noqa: E402
from ml_lidar.viz_support import catchup_step_count, set_japanese_font  # noqa: E402

__all__ = ["Panel", "LiveWatcher", "parse_args", "main"]

DEFAULT_POLL_INTERVAL_S = 5.0
DEFAULT_INTERVAL_MS = 150


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
    """5本のeval固定コース（直線/コーナー/中間/ヘアピン/連続シケイン主体）ぶんのパネルを作る。"""
    return [Panel(spec.name, spec.build_course(), env_cfg, max_episode_steps)
           for spec in EVAL_COURSE_PARAMS]


def run(args: argparse.Namespace) -> None:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.patches import Polygon

    set_japanese_font()

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

    fig, axes = plt.subplots(1, len(panels), figsize=(5 * len(panels), 5.5))
    if len(panels) == 1:
        axes = [axes]
    fig.suptitle(f"ml_lidar ライブ観戦 — run={args.run_name}")

    artists = []
    for ax, panel in zip(axes, panels):
        ax.set_aspect("equal")
        course = panel.base_env.course
        h, w = course.grid.shape
        extent = (course.origin[0], course.origin[0] + w * course.resolution,
                 course.origin[1], course.origin[1] + h * course.resolution)
        ax.imshow(course.grid, extent=extent, origin="lower", cmap="Greys",
                 vmin=0, vmax=1, alpha=0.6)
        body_poly = Polygon(np.zeros((4, 2)), closed=True, facecolor="tab:blue",
                            edgecolor="black", zorder=5)
        ax.add_patch(body_poly)
        scatter = ax.scatter([], [], s=3, c="tab:red", zorder=4)
        title = ax.set_title(panel.name)
        artists.append((body_poly, scatter, title))

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
