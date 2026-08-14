"""Disparity Extender — 「車体が通れない隙間」を先に塞いでから、一番遠くを狙う。

地図も自己位置も要らない反射的な走り方で、`FollowTheGap` の上位互換にあたる。
F1TENTH で広く使われている手法（Nathan Otterness, 2019）。

## Follow the Gap との違いは狙点の決め方だけ

| | Follow the Gap | Disparity Extender |
|---|---|---|
| 隙間の扱い | `gap_min`[m] 以上が続く**最長の区間**を選ぶ | **通れない隙間を塞いでから**残った所を見る |
| 狙点 | 最長区間の真ん中 | **一番遠い帯**の真ん中 |
| 速度 | 正面 ±`front_deg` の余裕 | **狙う方向の見通し**（下記。ここが効く） |
| 安全バブル | 最近傍の周りを塗る | **要らない**（塗りが同じ役目を果たす） |

FTG は「どれだけ広く空いているか」しか見ないので、**奥まで抜けているかを
問わない**。Disparity Extender は通れない隙間を先に消したうえで奥行きで選ぶ。

実測（真値で周回計測・180秒・いずれも衝突 0。`PROGRESS.md`）:

    circuit  FTG 37.2s → DE 16.9s
    oval     FTG 36.5s → DE 15.8s
    split    FTG 1周もできず → DE 40.5s（`safety_half_width` を 0.12 へ下げる）

## 段差（disparity）の埋め方

隣り合う測定値が `disparity_m` 以上飛んでいる所が「物の縁」。
そこは**手前の物の陰**なので、車体半幅ぶんは通れない:

        近い側 r                     ↓ ここが段差
        ┌──────────┐                 │
        │  障害物   │  ← r [m]        ▼
        └──────────┘  · · · · · · · · ┃ 遠い側（見えているが通れない）

縁の**遠い側**を、近い側の値 `r` で塗り潰す。塗る角度幅は

    θ = atan2( safety_half_width , r )

で、距離 `r` の所で車体半幅ぶんを見込む角度そのもの。**近いほど広く塗る**
（至近では 90° で頭打ち。atan2 は r→0 で 90° に収束する）。

塗るときは `min()` で入れる。**上書きにすると処理の順番で結果が変わる**
（左から見た段差と右から見た段差が同じセルを取り合う）。min なら順不同で
同じ答えになり、しかも常に安全側にしか動かない。

## 塗ったあとは「一番遠い**帯の真ん中**」を狙う（点ではない）

塗り終わった配列は、抜けている方向が**同じ距離で平らに並ぶ**。そこで素直に
`argmax` を採ると平らな帯の**端**が選ばれ、コーナー内側を舐める線になる。
実測（circuit・180秒）で **ラップ 48.7s → 16.9s**。差はほぼここ。

同じ広さの帯が並んだら**正面に近い方**。開けた場所で理由もなく片側へ
曲がっていくのを防ぐ。

## ★ 速度は「狙う方向の `ext`」そのままで決める。扇で測り直さない

**道幅 1.0m の走路では、中央に居ても壁が 0.5m 横にある。** 正面 ±20° の扇で
最小値を採ると 1.46m 先で必ず壁に当たるので、**どんな直線でも見通しが 1.5m を
超えない**。これが Follow the Gap がミニカーのコースで踏み切れない理由でもある。

塗り終わった `ext[狙う方向]` は「**車体半幅を織り込んだうえで、その方向へ
何 m 進めるか**」なので、扇で測り直すのは二重に安全側へ倒すことになる。
正面の余裕（事実）は `stop_dist` の判定で使っており、そちらが最後の砦。

実測でここが最大の足枷だった（circuit のラップ **62s → 17s**）。

## 前処理と停止条件は Follow the Gap と同じ契約

欠測・飽和・測距不能の読み方は `base.scan_window()` に集約してある
（**片方だけ直すともう片方が古い読み方のまま走る**ため）。
`stop_dist` が「測距不能を空き扱いにしている穴」を受ける最後の砦であることも同じ。
"""

from __future__ import annotations

import math

from ..msgs.types import AutoState, Scan, VehicleState
from .base import ParamSpec, Planner, min_filter, scan_window

__all__ = ["DisparityExtender"]

#: 視野内でセクタが見えている割合がこれを下回ったら計画を放棄する。
#: **パラメータにしていない。**（`follow_the_gap.py` と同じ理由）
MIN_SEEN_RATIO = 0.6
#: 狙点の候補と見なす「一番遠い」の許容差 [m]。これ以内は同着として扱い、
#: **その中で正面にいちばん近い方向**を採る
TIE_M = 0.05


class DisparityExtender(Planner):
    id = "de"
    name = "Disparity Extender"
    description = "通れない隙間を塞いでから一番遠くを狙う。地図も自己位置も使わない"

    params = (
        ParamSpec(key="fov_deg", label="視野角", min=60, max=270, step=10,
                  default=180, unit="°",
                  note="前方 ±この半分だけを見る。狭めると横の抜け道を見落とす"),
        ParamSpec(key="max_range", label="見る距離", min=0.5, max=8.0, step=0.1,
                  default=6.0, unit="m",
                  note="★FTG より長く取る。遠くまで見えるほど直線で伸びる。"
                       "短いと出口が見えず、コーナーで減速したままになる"),
        ParamSpec(key="disparity_m", label="段差とみなす距離差", min=0.05, max=2.0,
                  step=0.05, default=0.30, unit="m",
                  note="隣の点とこれ以上離れていたら「物の縁」と見なす。"
                       "小さくすると壁の凹凸まで縁になり、視野が塞がって遅くなる"),
        ParamSpec(key="safety_half_width", label="安全半幅", min=0.08, max=0.60,
                  step=0.01, default=0.30, unit="m",
                  note="★縁の陰を塗る幅。車体半幅 0.095m の3倍あるのは、"
                       "旋回半径ぶんの膨らみと点群の遅延をここで飲むため"
                       "（0.25 だと circuit で行き止まりに入り込んで停止した）。"
                       "★分離帯コース（車線 0.42m）では 0.12 まで下げること"),
        ParamSpec(key="min_filter_deg", label="最小値フィルタ幅", min=0, max=5, step=1,
                  default=2, unit="°",
                  note="±この範囲の最小値を取る。障害物を太らせる方向にだけ間違える"),
        ParamSpec(key="front_deg", label="正面とみなす幅", min=5, max=45, step=1,
                  default=20, unit="°",
                  note="正面の余裕をこの範囲の最小距離で測る。停止判定に使う"),
        ParamSpec(key="stop_dist", label="停止する前方距離", min=0.1, max=1.5, step=0.01,
                  default=0.35, unit="m",
                  note="正面余裕がこれを切ったら制動する。★測距不能を空き扱いに"
                       "している穴を受けるのはここ"),
        ParamSpec(key="slow_dist", label="全開になる距離", min=0.3, max=8.0, step=0.1,
                  default=3.0, unit="m",
                  note="見通しがこれ以上あれば最高速度。stop_dist との間を線形に結ぶ"),
        ParamSpec(key="max_speed", label="最高速度", min=0.05, max=2.0, step=0.05,
                  default=0.60, unit="m/s",
                  note="★実車で上げるのは点群の遅延を測ってから。"
                       "シムでは 1.5 まで衝突なしを確認済み（`PROGRESS.md`）"),
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
                  default=0.35, unit="",
                  note="舵角いっぱいで速度をこの割合ぶん落とす。"
                       "★FTG より小さい既定。膨らまないぶん速度を残せる"),
        ParamSpec(key="steer_tau", label="舵の平滑化", min=0.0, max=0.5, step=0.01,
                  default=0.08, unit="s",
                  note="舵指令の1次遅れの時定数。0 で平滑化なし。上げると滑らかだが鈍る"),
    )

    def __init__(self) -> None:
        self._steer = 0.0

    def reset(self) -> None:
        # **モード切替・disengage のたびに呼ばれる。**（`base.py` の約束1）
        self._steer = 0.0

    # ── 本体 ──

    def plan(self, scan: Scan, vs: VehicleState | None,
             p: dict[str, float], dt: float) -> AutoState:
        st = AutoState(mode=self.id, planner=self.name)

        max_range = p["max_range"]

        # ── ① / ② 視野の切り出しと前処理 ──
        w = scan_window(scan, p["fov_deg"], max_range)
        degs, measured = w.degs, w.measured
        st.valid_ratio = w.valid_ratio

        if w.seen_ratio < MIN_SEEN_RATIO:
            st.reason = (f"点群の欠測が多すぎる（視野の {w.seen_ratio * 100:.0f}% しか"
                         f"受信できていない）")
            return st                      # ready=False のまま返す ＝ 制動

        # **最近傍はフィルタを掛ける前の実測点から探す**（GUI の表示用）。
        # フィルタ後だと欠測の 0.0 が隣へ広がり、そこが最近傍に化ける
        nearest = max_range
        nearest_j = -1
        for j, ok in enumerate(measured):
            if ok and w.dist[j] < nearest:
                nearest = w.dist[j]
                nearest_j = j
        st.nearest = nearest
        st.nearest_deg = float(degs[nearest_j]) if nearest_j >= 0 else 0.0

        usable = min_filter(list(w.dist), int(p["min_filter_deg"]))

        # 正面の余裕。**段差を塗る前に測る**（塗るのは進路選択のための加工であって、
        # 正面に何 m あるかという事実ではない）
        fw = int(p["front_deg"])
        front = [usable[j] for j, d in enumerate(degs) if abs(d) <= fw]
        st.free_ahead = min(front) if front else 0.0

        # ── ③ 段差を埋める ──
        ext = _extend(usable, p["disparity_m"], p["safety_half_width"])

        # DE は安全バブルを置かない（塗りが同じ役目を果たす）。
        # `start > end` で「バブル無し」を表す（`AutoState` の約束）
        st.bubble_start_deg = 0.0
        st.bubble_end_deg = -1.0

        # 正面が詰まっているときは、進める方向があっても**まず止める**。
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

        # ── ④ 一番遠い帯の**真ん中**を狙う ──
        best = max(ext)
        if best <= stop_d:
            st.reason = (f"塞いだ結果どこにも進めない（最遠 {best * 100:.0f}cm・"
                         f"最近傍 {nearest * 100:.0f}cm）")
            return st                      # ready=False ＝ 制動

        a, b = _best_band(ext, best - TIE_M, degs)
        j_best = (a + b) // 2
        st.ready = True
        st.gap_start_deg = float(degs[a])
        st.gap_end_deg = float(degs[b])

        # ── ⑤ 舵 ──
        st.heading = math.radians(degs[j_best])
        max_steer = p["max_steer"]
        target = max(-max_steer, min(max_steer, p["steer_gain"] * st.heading))
        # 時間ベースの1次遅れ。フレームレートに依存させない
        tau = p["steer_tau"]
        alpha = 1.0 if tau <= 1e-3 or dt <= 0 else 1.0 - math.exp(-dt / tau)
        self._steer += (target - self._steer) * alpha
        st.target_steer = self._steer

        # ── ⑥ 速度は「**進む方向**の見通し」で決める ──
        #
        # ★ **`ext[j_best]` をそのまま使う。扇で測り直さない。**
        #
        # 正面 ±`front_deg` の最小値で速度を決めると、**道幅 1.0m の走路では
        # 側壁を測ってしまう**。中央に居ても壁は 0.5m 横にあるので、±20° の扇は
        # 1.46m 先で必ず壁に当たり、どんな直線でも見通しが 1.5m 以上にならない。
        # これが Follow the Gap がミニカーのコースで踏み切れない理由でもある。
        #
        # 塗り終わった `ext` は「車体半幅を考慮したうえで、その方向へ何 m
        # 進めるか」なので、**扇で測り直すのは二重に安全側へ倒すこと**になる。
        # 正面の余裕（事実）は `stop_dist` の判定で既に使っており、そちらが最後の砦
        lookahead = ext[j_best]
        slow_d = max(p["slow_dist"], stop_d + 0.01)
        v_max = p["max_speed"]
        v_min = min(p["min_speed"], v_max)

        ratio = min(1.0, max(0.0, (lookahead - stop_d) / (slow_d - stop_d)))
        v = v_min + (v_max - v_min) * ratio
        turn = abs(st.target_steer) / max_steer if max_steer > 0 else 0.0
        v *= 1.0 - p["turn_slow"] * min(1.0, turn)
        st.target_speed = max(0.0, v)

        st.reason = (f"{degs[j_best]:+d}° の見通し {lookahead:.2f}m へ・"
                     f"正面 {st.free_ahead:.2f}m")
        return st


# ── 小道具 ────────────────────────────────────────────────────────────

def _best_band(r: list[float], threshold: float,
               degs: list[int]) -> tuple[int, int]:
    """`threshold` 以上が続く区間のうち**いちばん広いもの**を返す（両端含む）。

    ★ **「一番遠い点」ではなく「一番遠い帯の真ん中」を狙う。** 塗ったあとの
    配列は、抜けている方向が同じ距離で平らに並ぶ（`max_range` で頭打ちになる
    ため）。そこで素直に `argmax` を採ると平らな帯の**端**が選ばれ、
    コーナーで内側の壁を舐める線になる。実測（circuit・180秒）で
    **ラップ 48.7s → 端を狙った場合**、真ん中を狙うと大きく縮む。

    同じ広さが並んだら**正面に近い方**。開けた場所で理由もなく片側へ
    曲がっていくのを防ぐ。
    """
    best = (0, 0)
    best_len = -1
    best_off = 1e9
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
        off = abs(degs[(j + k) // 2])
        if length > best_len or (length == best_len and off < best_off):
            best, best_len, best_off = (j, k), length, off
        j = k + 1
    return best



def _extend(r: list[float], disparity: float, half_width: float) -> list[float]:
    """段差の**遠い側**を近い側の値で塗る（docstring の図）。

    塗るのは `min()`。**上書きにすると処理の順番で結果が変わる。**
    """
    out = list(r)
    n = len(r)
    for i in range(n - 1):
        lo, hi = r[i], r[i + 1]
        if abs(hi - lo) < disparity:
            continue
        near = min(lo, hi)
        # 距離 `near` の所で半幅ぶんを見込む角度［度］。至近では 90° で頭打ち
        span = min(90.0, math.degrees(math.atan2(half_width, max(near, 1e-3))))
        k = int(math.ceil(span))           # 点は 1° 刻み
        if hi > lo:
            # 遠いのは右→左方向（添字が増える側）。i+1 から先を塗る
            for j in range(i + 1, min(n, i + 1 + k)):
                out[j] = min(out[j], near)
        else:
            for j in range(max(0, i - k + 1), i + 1):
                out[j] = min(out[j], near)
    return out
