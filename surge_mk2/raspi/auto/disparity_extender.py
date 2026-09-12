"""Disparity Extender — 「車体が通れない隙間」を先に塞いでから、一番遠くを狙う。

地図も自己位置も要らない反射的な走り方で、`FollowTheGap` の上位互換にあたる。
F1TENTH で広く使われている手法（Nathan Otterness, 2019）。

## Follow the Gap との違いは狙点の決め方だけ

| | Follow the Gap | Disparity Extender |
|---|---|---|
| 隙間の扱い | `gap_min`[m] 以上が続く**最長の区間**を選ぶ | **通れない隙間を塞いでから**残った所を見る |
| 狙点 | 最長区間の真ん中 | **一番遠い帯**の真ん中 |
| 速度の基礎 | 正面 ±`front_deg` の余裕 | **狙う方向の見通し**（下記。ここが効く） |
| 安全バブル | 最近傍の周りを塗る | **要らない**（塗りが同じ役目を果たす） |

速度の頭打ちは両方とも同じ式（実舵角から曲率 κ=tan(δ)/L を求め
`v ≤ sqrt(a_lat_max/κ)`）を使う。**以前は DE だけ `turn_slow`（舵角いっぱいで
一律%減速）という発見的な調整だったが、2026-09-12 に FTG と同じ曲率ベースへ
揃えた**——緩いカーブもきついカーブも同じ割合でしか区別できない
`turn_slow` の弱点は FTG の方の docstring（⑥）参照。

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

## 前処理は Follow the Gap と同じ契約

欠測・飽和・測距不能の読み方は `base.scan_window()` に集約してある
（**片方だけ直すともう片方が古い読み方のまま走る**ため）。

## 緊急停止は持たない（2026-09-12）

`FollowTheGap` と同じ理由（`follow_the_gap.py` docstring参照）で、正面距離
しきい値によるハード停止（旧 `stop_dist`）は撤去した。STM32 の `auto_stop`
（速度に応じて伸びる動的停止距離）に任せる。塗った結果どこにも進めない
（`best <= _STUCK_M`）判定は残す——これは「衝突しそう」ではなく「計画その
ものが失敗している」ケースなので `ready=False` として残す。
"""

from __future__ import annotations

import math

from ..core.vehicle import Vehicle
from ..msgs.types import AutoState, Scan, VehicleState
from .base import ParamSpec, Planner, extend_disparity, min_filter, scan_window

__all__ = ["DisparityExtender"]

#: 視野内でセクタが見えている割合がこれを下回ったら計画を放棄する。
#: **パラメータにしていない。**（`follow_the_gap.py` と同じ理由）
MIN_SEEN_RATIO = 0.6
#: 狙点の候補と見なす「一番遠い」の許容差 [m]。これ以内は同着として扱い、
#: **その中で正面にいちばん近い方向**を採る
TIE_M = 0.05
#: 塗った結果の最遠距離がこれ以下なら「どこにも進めない」（`ready=False`）。
#: **緊急停止のしきい値ではない**——実質ゼロ（車体が何かに接触している）を
#: 検出するためだけの固定値。停止そのものは STM32 の `auto_stop` に任せる
_STUCK_M = 0.05


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
                  note="★縁の陰を塗る幅。車体半幅 0.09m の3倍あるのは、"
                       "旋回半径ぶんの膨らみと点群の遅延をここで飲むため"
                       "（0.25 だと circuit で行き止まりに入り込んで停止した）。"
                       "★分離帯コース（車線 0.42m）では 0.12 まで下げること"),
        ParamSpec(key="min_filter_deg", label="最小値フィルタ幅", min=0, max=5, step=1,
                  default=2, unit="°",
                  note="±この範囲の最小値を取る。障害物を太らせる方向にだけ間違える"),
        ParamSpec(key="front_deg", label="正面とみなす幅", min=5, max=45, step=1,
                  default=20, unit="°",
                  note="GUI表示用の正面の余裕をこの範囲の最小距離で測る（診断用。速度には使わない）"),
        ParamSpec(key="slow_dist", label="全開になる距離", min=0.3, max=8.0, step=0.1,
                  default=3.0, unit="m",
                  note="見通しがこれ以上あれば最高速度。0m（接触寸前）との間を線形に結ぶ"),
        ParamSpec(key="max_speed", label="最高速度", min=0.05, max=3.0, step=0.05,
                  default=0.60, unit="m/s",
                  note="★実車で上げるのは点群の遅延を測ってから。"
                       "シムでは 1.5 まで衝突なしを確認済み（`PROGRESS.md`）"),
        ParamSpec(key="min_speed", label="最低速度", min=0.0, max=1.0, step=0.01,
                  default=0.12, unit="m/s",
                  note="減速しきってもこれ以下にはしない。0 にすると詰まった所で動けなくなる"),
        ParamSpec(key="steer_gain", label="舵角ゲイン", min=0.1, max=2.0, step=0.05,
                  default=0.80, unit="",
                  note="狙う方位[rad]に掛けて舵角にする。上げると食いつくが振動しやすい"),
        ParamSpec(key="a_lat_max", label="旋回時の横加速度上限", min=0.5, max=8.0, step=0.1,
                  default=3.0, unit="m/s²",
                  note="★実車未計測の暫定値。実際に切る舵角から曲率 κ=tan(δ)/L を求め、"
                       "v ≤ sqrt(これ/κ) で速度を抑える（`FollowTheGap` と同じ式。"
                       "2026-09-12に`turn_slow`から置き換え）"),
        ParamSpec(key="steer_tau", label="舵の平滑化", min=0.0, max=0.5, step=0.01,
                  default=0.08, unit="s",
                  note="舵指令の1次遅れの時定数。0 で平滑化なし。上げると滑らかだが鈍る"),
    )

    def __init__(self) -> None:
        self.vehicle = Vehicle.load()
        self._steer = 0.0

    def reset(self) -> None:
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
        ext = extend_disparity(usable, p["disparity_m"], p["safety_half_width"])

        # DE は安全バブルを置かない（塗りが同じ役目を果たす）。
        # `start > end` で「バブル無し」を表す（`AutoState` の約束）
        st.bubble_start_deg = 0.0
        st.bubble_end_deg = -1.0

        # ── ④ 一番遠い帯の**真ん中**を狙う ──
        best = max(ext)
        if best <= _STUCK_M:
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
        max_steer = self.vehicle.max_steer
        target = max(-max_steer, min(max_steer, p["steer_gain"] * st.heading))
        # 時間ベースの1次遅れ。フレームレートに依存させない
        tau = p["steer_tau"]
        alpha = 1.0 if tau <= 1e-3 or dt <= 0 else 1.0 - math.exp(-dt / tau)
        self._steer += (target - self._steer) * alpha
        st.target_steer = self._steer

        # ── ⑥ 速度は「**進む方向**の見通し」で決める ──
        # ★ `ext[j_best]` をそのまま使う。扇で測り直さない
        #   （理由はモジュール docstring「速度は『狙う方向の ext』そのままで決める」参照）
        lookahead = ext[j_best]
        slow_d = max(p["slow_dist"], 1e-3)
        v_max = p["max_speed"]
        v_min = min(p["min_speed"], v_max)

        ratio = max(0.0, min(1.0, lookahead / slow_d))
        v = v_min + (v_max - v_min) * ratio

        # 曲率ベースの物理的な上限（`FollowTheGap`の⑥と同じ式）。実際に切る
        # 舵角（クランプ後の`target`。平滑化前）から曲率を求める
        kappa = abs(math.tan(target) / self.vehicle.wheelbase)
        v_curve = math.sqrt(p["a_lat_max"] / kappa) if kappa > 1e-6 else math.inf
        st.target_speed = max(v_min, min(v, v_curve, v_max))

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
