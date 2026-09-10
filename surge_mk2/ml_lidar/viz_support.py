"""ml_lidar/viz_support.py — `watch.py`・`live_watch.py`が共通で使う描画まわりの小物。

どちらも matplotlib の `FuncAnimation` で「直前フレームからの実経過時間ぶんだけ
シムを進める」（fixed-timestep-with-catchup）方式を使っている。「毎フレーム必ず
1step」だと、描画・CPU競合でフレーム間隔が伸びた瞬間だけ見かけの速度が遅くなり、
次に間隔が戻ると急に追いつく——これが「かくつく」の正体なので、両者とも同じ
対策を採る。
"""

from __future__ import annotations

__all__ = ["MAX_CATCHUP_STEPS", "set_japanese_font", "catchup_step_count"]

#: 1フレームで追いつくために進めてよいシムstepの上限。無制限にすると、
#: 描画がひどく詰まった直後に「早送り」してしまう
MAX_CATCHUP_STEPS = 10


def set_japanese_font() -> None:
    """既定フォント(DejaVu Sans)は日本語グリフを持たず豆腐（□□□）になるため、
    macOS標準の日本語フォントを明示する。matplotlibをimportした呼び出し側で使う。
    """
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = "sans-serif"
    plt.rcParams["font.sans-serif"] = ["Hiragino Sans", "Arial Unicode MS", "DejaVu Sans"]


def catchup_step_count(real_dt: float, dt: float, max_steps: int = MAX_CATCHUP_STEPS) -> int:
    """直前フレームからの実経過時間`real_dt`ぶんを埋めるのに必要なstep数（1以上）。"""
    return max(1, min(max_steps, round(real_dt / dt))) if real_dt > 0 else 1
