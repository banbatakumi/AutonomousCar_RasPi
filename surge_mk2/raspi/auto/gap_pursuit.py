"""Disparity Pursuit — Disparity Extender の安全マージンと Follow the Gap の
Pure Pursuit 舵角を合わせ、さらに3つの弱点に手を入れた反射型プランナ。

地図も自己位置も要らない非SLAM・O(n)のリアルタイム処理という前提は
`FollowTheGap`／`DisparityExtender` と同じ。この2つを読み比べると、
「安全マージンの取り方・狙点の選び方」は Disparity Extender が優れ
（`disparity_extender.py` の実測: circuit 37.2s→16.9s 等）、「狙点を舵角へ
変換する式」は Follow the Gap 側が Pure Pursuit へ移行済み（速度比例の前方
注視距離で、コーナーで曲がりきれず壁に当たる欠陥を修正済み）——という
ねじれた状態にある。**どちらか片方を持ってきても最良にはならない。**

## この Planner が両者から引き継ぐもの

- 前処理: `base.scan_window()` / `min_filter()`（欠測→壁、飽和→空き、
  測距不能→空き。3プランナで共有する契約。片方だけ直さない）
- 安全マージン: `DisparityExtender._extend()` と同じ「段差の遠い側を
  車体半幅ぶん塗る」処理。最近傍1点だけを塗る円形バブル（FTG）と違い、
  視野内の**すべての段差**を同時に処理できる
- 狙点: 塗った後の「一番遠い**帯の真ん中**」（DEと同じ。端を狙うとコーナー
  内側を舐める）
- 速度の基礎: 狙う方向の塗り後距離をそのまま使い、正面の扇で測り直さない
  （DEの docstring 参照。測り直すと道幅ぶんの壁を拾って直線でも減速し続ける）
- 舵角: `nav.purepursuit.steer_for_target()`（FTGと同じ。η=狙点方位、
  Ld=速度比例で、ギャップの奥行き＝塗り後距離を安全上限にする）

## ここから新規に追加した3つ

### ① 狙点のヒステリシス

DEの同着判定（`TIE_M`以内なら「正面に近い方」を採る）は、**前回どちらを
向いていたか**を見ない。通路が湾曲している場面では、僅差で最遠帯が
フレームごとに入れ替わりうる場所で「正面に近い方」という基準自体が
毎回同じ2択の間で振動する引き金になりうる。ここでは基準を「前回向いて
いた方位に一番近い帯」に一般化する。前回ヘディングが0付近（直進中）なら
DEと同じ挙動に自然収束するので、直線・素直なカーブでの走りは変えない。

### ② 曲率ベースの速度上限

FTG/DEはどちらも「舵角いっぱいで`turn_slow`割ぶん一律減速」という発見的な
調整で、緩いカーブもきついカーブも同じ割合でしか区別できない。ここでは
Pure Pursuit が使う舵角 `δ` から実際に描く円弧の曲率 `κ=tan(δ)/L` を求め、
円運動の横加速度制限 `v ≤ sqrt(a_lat_max/κ)` を速度の物理的な上限にする。
緩い旋回は落としすぎず、きつい旋回は根拠のある分だけ確実に落とす。
`a_lat_max`（既定 3.0 m/s²）は実車未計測のため暫定値。**実測して詰めること。**

### ③ TTC（衝突余裕時間）による追加の停止条件

`stop_dist` は固定距離のしきい値で、どれだけの速さで近づいているかを見ない。
前フレームとの正面余裕の差分から接近速度を推定し、
`正面余裕 / 接近速度 < ttc_min` なら `stop_dist` の手前でも即座に制動する。
低速では効かない（`stop_dist`のほうが先に効く）が、将来 `max_speed` を
上げていく段になるほど効いてくる安全層として先に入れておく。

## 実車・シムでの計測はまだ行っていない

`disparity_m` / `safety_half_width` / `look_k` / `look_min` / `steer_tau` の
既定値は `DisparityExtender` / `FollowTheGap` の実測値をそのまま引き継いだ
ものだが、`a_lat_max` / `ttc_min` / `max_speed` はこの Planner 独自の値で
未計測。FTG/DEと同じ手順（シムで衝突なしを確認 → 実車で上げる）を踏むこと。
"""

from __future__ import annotations

import math

from ..core.vehicle import Vehicle
from ..msgs.types import AutoState, Scan, VehicleState
from ..nav.purepursuit import steer_for_target
from .base import ParamSpec, Planner, min_filter, scan_window

__all__ = ["DisparityPursuit"]

#: 視野内でセクタが見えている割合がこれを下回ったら計画を放棄する。
#: **パラメータにしていない。**（`follow_the_gap.py` と同じ理由）
MIN_SEEN_RATIO = 0.6
#: 狙点の候補と見なす「一番遠い」の許容差 [m]。これ以内は同着として扱い、
#: **その中で前回ヘディングにいちばん近い方向**を採る（①ヒステリシス）
TIE_M = 0.05


class DisparityPursuit(Planner):
    id = "dp"
    name = "Disparity Pursuit"
    description = "段差を塞いでから最遠の帯を追う。舵はPure Pursuit、速度は旋回曲率と接近速度で決める"

    params = (
        ParamSpec(key="fov_deg", label="視野角", min=60, max=270, step=10,
                  default=180, unit="°",
                  note="前方 ±この半分だけを見る。狭めると横の抜け道を見落とす"),
        ParamSpec(key="max_range", label="見る距離", min=0.5, max=8.0, step=0.1,
                  default=6.0, unit="m",
                  note="遠くまで見えるほど直線で伸びる。短いと出口が見えずコーナーで減速したままになる"),
        ParamSpec(key="disparity_m", label="段差とみなす距離差", min=0.05, max=2.0,
                  step=0.05, default=0.30, unit="m",
                  note="隣の点とこれ以上離れていたら「物の縁」と見なす。"
                       "小さくすると壁の凹凸まで縁になり、視野が塞がって遅くなる"),
        ParamSpec(key="safety_half_width", label="安全半幅", min=0.08, max=0.60,
                  step=0.01, default=0.30, unit="m",
                  note="縁の陰を塗る幅。車体半幅 0.09m の3倍あるのは、"
                       "旋回半径ぶんの膨らみと点群の遅延をここで飲むため"),
        ParamSpec(key="min_filter_deg", label="最小値フィルタ幅", min=0, max=5, step=1,
                  default=2, unit="°",
                  note="±この範囲の最小値を取る。障害物を太らせる方向にだけ間違える"),
        ParamSpec(key="front_deg", label="正面とみなす幅", min=5, max=45, step=1,
                  default=20, unit="°",
                  note="正面の余裕をこの範囲の最小距離で測る。停止判定とTTCに使う"),
        ParamSpec(key="stop_dist", label="停止する前方距離", min=0.1, max=1.5, step=0.01,
                  default=0.35, unit="m",
                  note="正面余裕がこれを切ったら制動する。★測距不能を空き扱いにしている穴を受けるのはここ"),
        ParamSpec(key="slow_dist", label="全開になる距離", min=0.3, max=8.0, step=0.1,
                  default=3.0, unit="m",
                  note="見通しがこれ以上あれば最高速度。stop_dist との間を線形に結ぶ"),
        ParamSpec(key="max_speed", label="最高速度", min=0.05, max=2.0, step=0.05,
                  default=0.50, unit="m/s",
                  note="★この Planner は未計測。まずシムで衝突なしを確認してから上げること"),
        ParamSpec(key="min_speed", label="最低速度", min=0.0, max=1.0, step=0.01,
                  default=0.12, unit="m/s",
                  note="減速しきってもこれ以下にはしない。0 にすると詰まった所で動けなくなる"),
        ParamSpec(key="look_k", label="前方注視の速度係数", min=0.0, max=2.0, step=0.05,
                  default=0.7, unit="s",
                  note="Ld = 係数×速度 + 最小値。上げると滑らかだがコーナーで曲がりきれなくなる"),
        ParamSpec(key="look_min", label="前方注視の最小値", min=0.15, max=1.5, step=0.05,
                  default=0.35, unit="m",
                  note="低速時の注視距離。小さすぎると舵が振動する"),
        ParamSpec(key="a_lat_max", label="旋回時の横加速度上限", min=0.5, max=8.0, step=0.1,
                  default=3.0, unit="m/s²",
                  note="★未計測の暫定値。v ≤ sqrt(これ/曲率) で速度を抑える。"
                       "上げるほど旋回中に速度が残るが横滑りしやすくなる"),
        ParamSpec(key="ttc_min", label="衝突余裕時間の下限", min=0.1, max=3.0, step=0.05,
                  default=0.6, unit="s",
                  note="正面余裕÷接近速度がこれを切ったら stop_dist の手前でも即座に制動する"),
        ParamSpec(key="steer_tau", label="舵の平滑化", min=0.0, max=0.5, step=0.01,
                  default=0.08, unit="s",
                  note="舵指令の1次遅れの時定数。0 で平滑化なし。上げると滑らかだが鈍る"),
    )

    def __init__(self) -> None:
        self.vehicle = Vehicle.load()
        self._steer = 0.0
        self._heading_deg = 0.0            # ①ヒステリシス: 前回向いていた方位
        self._prev_front: float | None = None  # ③TTC: 前フレームの正面余裕

    def reset(self) -> None:
        # **モード切替・disengage のたびに呼ばれる。**（`base.py` の約束1）
        self._steer = 0.0
        self._heading_deg = 0.0
        self._prev_front = None

    # ── 本体 ──

    def plan(self, scan: Scan, vs: VehicleState | None,
             p: dict[str, float], dt: float) -> AutoState:
        st = AutoState(mode=self.id, planner=self.name)

        max_range = p["max_range"]

        # ── 視野の切り出しと前処理（`base.scan_window`）──
        w = scan_window(scan, p["fov_deg"], max_range)
        degs, measured = w.degs, w.measured
        st.valid_ratio = w.valid_ratio

        if w.seen_ratio < MIN_SEEN_RATIO:
            st.reason = (f"点群の欠測が多すぎる（視野の {w.seen_ratio * 100:.0f}% しか"
                         f"受信できていない）")
            # ★TTCの前フレーム値はここでは更新しない。信頼できない周で
            # 「急に空いた」と誤検出して閉ループさせないため
            return st                      # ready=False のまま返す ＝ 制動

        # **最近傍はフィルタを掛ける前の実測点から探す**（GUI の表示用）
        nearest = max_range
        nearest_j = -1
        for j, ok in enumerate(measured):
            if ok and w.dist[j] < nearest:
                nearest = w.dist[j]
                nearest_j = j
        st.nearest = nearest
        st.nearest_deg = float(degs[nearest_j]) if nearest_j >= 0 else 0.0

        usable = min_filter(list(w.dist), int(p["min_filter_deg"]))

        # 正面の余裕。**段差を塗る前に測る**（事実であって進路選択の加工ではない）
        fw = int(p["front_deg"])
        front = [usable[j] for j, d in enumerate(degs) if abs(d) <= fw]
        st.free_ahead = min(front) if front else 0.0

        stop_d = p["stop_dist"]

        # ── ③ TTC: 接近速度が速ければ stop_dist の手前でも止める ──
        ttc_trigger = False
        closing = 0.0
        ttc = math.inf
        if self._prev_front is not None and dt > 0.0:
            closing = (self._prev_front - st.free_ahead) / dt
            if closing > 1e-3:
                ttc = st.free_ahead / closing
                ttc_trigger = ttc < p["ttc_min"]
        self._prev_front = st.free_ahead

        # ── 段差を埋める（安全マージン。DEと同じ処理）──
        ext = _extend(usable, p["disparity_m"], p["safety_half_width"])
        st.bubble_start_deg = 0.0
        st.bubble_end_deg = -1.0            # DEと同じく円形バブルは置かない

        if st.free_ahead <= stop_d or ttc_trigger:
            st.ready = True                # 計画はできている。**意図して止めている**
            st.brake = True
            st.target_speed = 0.0
            st.target_steer = self._steer   # 舵は保持（曲がりながら止まれる）
            if ttc_trigger and st.free_ahead > stop_d:
                st.reason = (f"正面が {closing:.2f}m/s で接近中・衝突余裕 {ttc:.2f}s"
                             f"（しきい値 {p['ttc_min']:.2f}s）で停止")
            else:
                st.reason = (f"正面 {st.free_ahead * 100:.0f}cm で停止"
                             f"（停止距離 {stop_d * 100:.0f}cm）")
            return st

        # ── 一番遠い帯の真ん中を狙う（①同着はヒステリシスで解く）──
        best = max(ext)
        if best <= stop_d:
            st.reason = (f"塞いだ結果どこにも進めない（最遠 {best * 100:.0f}cm・"
                         f"最近傍 {nearest * 100:.0f}cm）")
            return st                      # ready=False ＝ 制動

        a, b = _best_band(ext, best - TIE_M, degs, self._heading_deg)
        j_best = (a + b) // 2
        st.ready = True
        st.gap_start_deg = float(degs[a])
        st.gap_end_deg = float(degs[b])

        # ── 舵: Pure Pursuit。η=狙点方位、Ld=速度比例（安全上限は塗り後距離）──
        st.heading = math.radians(degs[j_best])
        self._heading_deg = float(degs[j_best])   # 次フレームのヒステリシス基準を更新

        max_steer = self.vehicle.max_steer
        v_now = vs.speed if vs is not None else 0.0
        lookahead_cap = ext[j_best]
        ld = min(lookahead_cap, p["look_k"] * v_now + p["look_min"])
        target = steer_for_target(st.heading, ld, self.vehicle.wheelbase, max_steer)
        # 時間ベースの1次遅れ。フレームレートに依存させない
        tau = p["steer_tau"]
        alpha = 1.0 if tau <= 1e-3 or dt <= 0 else 1.0 - math.exp(-dt / tau)
        self._steer += (target - self._steer) * alpha
        st.target_steer = self._steer

        # ── ② 速度: 見通しベースと曲率ベースの小さい方を採る ──
        #
        # 見通しベース: `ext[j_best]` をそのまま使う。DEと同じ理由で扇で測り
        # 直さない（正面 ±front_deg の最小値だと道幅ぶんの側壁を拾い、
        # どんな直線でも見通しが頭打ちになる）
        slow_d = max(p["slow_dist"], stop_d + 0.01)
        v_max = p["max_speed"]
        v_min = min(p["min_speed"], v_max)
        ratio = min(1.0, max(0.0, (lookahead_cap - stop_d) / (slow_d - stop_d)))
        v_range = v_min + (v_max - v_min) * ratio

        # 曲率ベース: 実際に切る舵角（クランプ後の `target`）から曲率を出す。
        # `steer_for_target` の式 tan(δ)=2L·sinη/Ld の左辺そのものなので、
        # 舵が頭打ちになっている場面でも実際に描く円弧に対して正しい
        kappa = abs(math.tan(target) / self.vehicle.wheelbase)
        v_curve = math.sqrt(p["a_lat_max"] / kappa) if kappa > 1e-6 else math.inf

        v = max(v_min, min(v_range, v_curve, v_max))
        st.target_speed = v

        st.reason = (f"{degs[j_best]:+d}° の見通し {lookahead_cap:.2f}m へ・"
                     f"曲率制限 {min(v_curve, v_max):.2f}m/s・正面 {st.free_ahead:.2f}m")
        return st


# ── 小道具 ────────────────────────────────────────────────────────────

def _best_band(r: list[float], threshold: float, degs: list[int],
                prev_heading_deg: float = 0.0) -> tuple[int, int]:
    """`threshold` 以上が続く区間のうち**いちばん広いもの**を返す（両端含む）。

    `DisparityExtender._best_band()` の一般化。同じ広さが並んだら
    **前回向いていた方位に一番近い方**を採る（①ヒステリシス）。
    `prev_heading_deg=0.0`（直進中の既定）なら DisparityExtender と
    まったく同じ「正面に近い方」に一致する。
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
        off = abs(degs[(j + k) // 2] - prev_heading_deg)
        if length > best_len or (length == best_len and off < best_off):
            best, best_len, best_off = (j, k), length, off
        j = k + 1
    return best


def _extend(r: list[float], disparity: float, half_width: float) -> list[float]:
    """段差の**遠い側**を近い側の値で塗る（`DisparityExtender._extend()` と同一）。

    塗るのは `min()`。**上書きにすると処理の順番で結果が変わる。**
    """
    out = list(r)
    n = len(r)
    for i in range(n - 1):
        lo, hi = r[i], r[i + 1]
        if abs(hi - lo) < disparity:
            continue
        near = min(lo, hi)
        span = min(90.0, math.degrees(math.atan2(half_width, max(near, 1e-3))))
        k = int(math.ceil(span))           # 点は 1° 刻み
        if hi > lo:
            for j in range(i + 1, min(n, i + 1 + k)):
                out[j] = min(out[j], near)
        else:
            for j in range(max(0, i - k + 1), i + 1):
                out[j] = min(out[j], near)
    return out
