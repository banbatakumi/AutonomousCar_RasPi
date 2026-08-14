"""Follow the Gap — LiDAR だけで「一番広く空いている方向」へ走る。

地図も自己位置も要らない**反射的な**走り方。SLAM を作る前に、点群が本当に
使える品質で届いているかを実車で確かめるための最初の自動運転として置いている。

## 手順

    ① 視野を切り出す         前方 ±fov/2 度だけを見る
    ② 前処理                 欠測・飽和・測距不能を安全側に倒し、最小値フィルタを掛ける
    ③ 安全バブル             最近傍の点の周りを「侵入禁止」で塗り潰す
    ④ ギャップ探索           gap_min[m] 以上が続く最長の区間を選ぶ
    ⑤ 狙点                   その区間の**真ん中**を向く
    ⑥ 速度                   正面の余裕と舵角の大きさで落とす

⑤ を「区間の中で一番遠い点」にする流儀もある。**真ん中を採っている**のは、
一番遠い点を狙うと壁際すれすれを縫う挙動になり、車幅ぶんの余裕が無い
ミニカーだと外輪が壁に当たるため。真ん中なら通路の中央に寄る。

## 前処理（②）は `base.scan_window()` にある

欠測・飽和・測距不能をどう読むかは**点群の読み方の契約**そのもので、
`Disparity Extender`（`auto/disparity_extender.py`）と共有している。
片方だけ直すともう片方が古い読み方のまま走るので、**ここには書き写さない。**
`stop_dist` が「測距不能を空き扱いにしている穴」を受ける側であることも同様。
"""

from __future__ import annotations

import math

from ..msgs.types import AutoState, Scan, VehicleState
from .base import ParamSpec, Planner, min_filter, scan_window

__all__ = ["FollowTheGap"]

#: 視野内でセクタが見えている割合がこれを下回ったら計画を放棄する。
#: **パラメータにしていない。** 「点群が来ていないのに走る」を人間が
#: スライダで許可できてしまう場所を作らないため
MIN_SEEN_RATIO = 0.6


class FollowTheGap(Planner):
    id = "ftg"
    name = "Follow the Gap"
    description = "点群の中で最も広く空いた方向へ走る。地図も自己位置も使わない"

    params = (
        ParamSpec(key="fov_deg", label="視野角", min=60, max=300, step=10,
                  default=180, unit="°",
                  note="前方 ±この半分だけを見る。広げると横の壁に反応しやすくなる"),
        ParamSpec(key="max_range", label="見る距離", min=0.5, max=8.0, step=0.1,
                  default=3.0, unit="m",
                  note="これより遠い点は「空き」として同じ扱いにする。遠くまで見ると直線的に走る"),
        ParamSpec(key="min_filter_deg", label="最小値フィルタ幅", min=0, max=5, step=1,
                  default=2, unit="°",
                  note="±この範囲の最小値を取る。障害物を太らせる方向にだけ間違える"),
        ParamSpec(key="bubble_m", label="安全バブル半径", min=0.05, max=1.0, step=0.01,
                  default=0.25, unit="m",
                  note="最近傍の点の周りを侵入禁止にする半径。車幅の半分＋余裕から決める"),
        ParamSpec(key="gap_min", label="ギャップの下限", min=0.2, max=4.0, step=0.05,
                  default=1.0, unit="m",
                  note="この距離以上が続く区間だけを「通れる隙間」と数える"),
        ParamSpec(key="front_deg", label="正面とみなす幅", min=5, max=45, step=1,
                  default=20, unit="°",
                  note="速度を決める前方余裕をこの範囲の最小距離で測る。車幅と見る距離から決まる"),
        ParamSpec(key="stop_dist", label="停止する前方距離", min=0.1, max=1.5, step=0.01,
                  default=0.35, unit="m",
                  note="正面余裕がこれを切ったら制動する。★測距不能を空き扱いにしている穴を受けるのはここ"),
        ParamSpec(key="slow_dist", label="減速を始める距離", min=0.3, max=5.0, step=0.05,
                  default=1.5, unit="m",
                  note="正面余裕がこれ以下で最高速度から最低速度へ線形に落とす"),
        ParamSpec(key="max_speed", label="最高速度", min=0.05, max=1.5, step=0.01,
                  default=0.40, unit="m/s",
                  note="★io_node の --max-speed を超えても Pi 側で切り捨てられるだけ"),
        ParamSpec(key="min_speed", label="最低速度", min=0.0, max=1.0, step=0.01,
                  default=0.12, unit="m/s",
                  note="減速しきってもこれ以下にはしない。0 にすると詰まった所で動けなくなる"),
        ParamSpec(key="max_steer", label="最大舵角", min=0.1, max=1.047, step=0.005,
                  default=0.60, unit="rad",
                  note="★io_node の --max-steer を超えても切り捨てられるだけ"),
        ParamSpec(key="steer_gain", label="舵角ゲイン", min=0.1, max=2.0, step=0.05,
                  default=0.80, unit="",
                  note="狙う方位[rad]に掛けて舵角にする。上げると食いつくが振動しやすい"),
        ParamSpec(key="turn_slow", label="旋回時の減速", min=0.0, max=1.0, step=0.05,
                  default=0.50, unit="",
                  note="舵角いっぱいで速度をこの割合ぶん落とす。1.0 で全舵時に停止"),
        ParamSpec(key="steer_tau", label="舵の平滑化", min=0.0, max=0.5, step=0.01,
                  default=0.10, unit="s",
                  note="舵指令の1次遅れの時定数。0 で平滑化なし。上げると滑らかだが反応が鈍る"),
    )

    def __init__(self) -> None:
        self._steer = 0.0

    def reset(self) -> None:
        # **モード切替・disengage のたびに呼ばれる。** 残しておくと、次に engage
        # した瞬間に前回の舵の続きから動き出す
        self._steer = 0.0

    # ── 本体 ──

    def plan(self, scan: Scan, vs: VehicleState | None,
             p: dict[str, float], dt: float) -> AutoState:
        st = AutoState(mode=self.id, planner=self.name)

        max_range = p["max_range"]

        # ── ① / ② 視野の切り出しと前処理（`base.scan_window`）──
        w = scan_window(scan, p["fov_deg"], max_range)
        degs, usable, measured = w.degs, list(w.dist), w.measured
        st.valid_ratio = w.valid_ratio

        if w.seen_ratio < MIN_SEEN_RATIO:
            st.reason = (f"点群の欠測が多すぎる（視野の {w.seen_ratio * 100:.0f}% しか"
                         f"受信できていない）")
            return st                      # ready=False のまま返す ＝ 制動

        # ── ③ 安全バブル ──
        #
        # **最近傍はフィルタを掛ける前の値から探す。** 最小値フィルタは欠測区間の
        # 0.0 を隣へ広げるので、フィルタ後の配列で探すと「欠測の隣にある壁」が
        # 距離 0 の実測点に化け、バブルが 90°（＝半面）に開いて全部塞いでしまう
        nearest_j = -1
        nearest = max_range
        for j, ok in enumerate(measured):
            if ok and usable[j] < nearest:
                nearest = usable[j]
                nearest_j = j
        st.nearest = nearest
        st.nearest_deg = float(degs[nearest_j]) if nearest_j >= 0 else 0.0

        usable = min_filter(usable, int(p["min_filter_deg"]))

        # 正面の余裕。**バブルを塗る前の値で測る**（バブルは進路選択のための
        # 加工であって、正面に何 m あるかという事実ではない）。
        # 欠測が正面に掛かれば 0 になり、そのまま停止条件に落ちる ＝ 前が見えない
        # なら止まる、で正しい
        fw = int(p["front_deg"])
        front = [usable[j] for j, d in enumerate(degs) if abs(d) <= fw]
        st.free_ahead = min(front) if front else 0.0

        if nearest_j >= 0 and nearest < max_range:
            # 距離 `nearest` の点から半径 `bubble_m` を見込む角度。至近では
            # 90°（＝半面）で頭打ちにする。atan2 は nearest→0 で 90° に収束する
            span = math.degrees(math.atan2(p["bubble_m"], max(nearest, 1e-3)))
            span = min(span, 90.0)
            lo = degs[nearest_j] - span
            hi = degs[nearest_j] + span
            st.bubble_start_deg = lo
            st.bubble_end_deg = hi
            for j, d in enumerate(degs):
                if lo <= d <= hi:
                    usable[j] = 0.0
        else:
            # `start > end` で「バブル無し」を表す（`AutoState` の約束）
            st.bubble_start_deg = 0.0
            st.bubble_end_deg = -1.0

        # ── ④ 最長ギャップ ──
        gap = _longest_run(usable, p["gap_min"])
        if gap is not None:
            a, b = gap
            st.gap_start_deg = float(degs[a])
            st.gap_end_deg = float(degs[b])

        # 正面が詰まっているときは、ギャップが見つかっていても**まず止める**。
        # 「隙間が無い」より「正面 20cm」の方が、人が読んで原因が分かる
        stop_d = p["stop_dist"]
        if st.free_ahead <= stop_d:
            st.ready = True                # 計画はできている。**意図して止めている**
            st.brake = True
            st.target_speed = 0.0
            st.target_steer = self._steer   # 舵は保持（曲がりながら止まれる）
            st.reason = (f"正面 {st.free_ahead * 100:.0f}cm で停止"
                         f"（停止距離 {stop_d * 100:.0f}cm）")
            return st

        if gap is None:
            st.reason = (f"進める隙間が無い（最近傍 {nearest * 100:.0f}cm・"
                         f"下限 {p['gap_min']:.2f}m）")
            return st                      # ready=False ＝ 制動

        a, b = gap
        st.ready = True

        # ── ⑤ 狙点はギャップの真ん中 ──
        mid = (degs[a] + degs[b]) / 2.0
        st.heading = math.radians(mid)

        max_steer = p["max_steer"]
        target = max(-max_steer, min(max_steer, p["steer_gain"] * st.heading))
        # 時間ベースの1次遅れ。フレームレートに依存させない
        tau = p["steer_tau"]
        alpha = 1.0 if tau <= 1e-3 or dt <= 0 else 1.0 - math.exp(-dt / tau)
        self._steer += (target - self._steer) * alpha
        st.target_steer = self._steer

        # ── ⑥ 速度 ──
        slow_d = max(p["slow_dist"], stop_d + 0.01)
        v_max = p["max_speed"]
        v_min = min(p["min_speed"], v_max)

        ratio = min(1.0, (st.free_ahead - stop_d) / (slow_d - stop_d))
        v = v_min + (v_max - v_min) * ratio
        turn = abs(st.target_steer) / max_steer if max_steer > 0 else 0.0
        v *= 1.0 - p["turn_slow"] * min(1.0, turn)
        st.target_speed = max(0.0, v)

        width = degs[b] - degs[a]
        st.reason = (f"{mid:+.0f}° のギャップ（幅 {width:.0f}°）へ・"
                     f"正面 {st.free_ahead:.2f}m")
        return st


# ── 小道具 ────────────────────────────────────────────────────────────

def _longest_run(r: list[float], threshold: float) -> tuple[int, int] | None:
    """`threshold` 以上が連続する最長区間 `(start, end)`（両端含む）。

    同じ長さが並んだときは**平均距離が大きい方**を採る。左右対称な通路の
    真ん中に立ったときに、毎周期どちらを選ぶか揺れて舵が振動するのを防ぐ
    ……ためではなく（それは平滑化の仕事）、単純に**より奥まで抜けている方**が
    通路として正しいため。
    """
    best: tuple[int, int] | None = None
    best_len = 0
    best_mean = 0.0
    j = 0
    n = len(r)
    while j < n:
        if r[j] < threshold:
            j += 1
            continue
        k = j
        while k + 1 < n and r[k + 1] >= threshold:
            k += 1
        length = k - j + 1
        mean = sum(r[j:k + 1]) / length
        if length > best_len or (length == best_len and mean > best_mean):
            best, best_len, best_mean = (j, k), length, mean
        j = k + 1
    return best
