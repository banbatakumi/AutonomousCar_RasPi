"""Disparity Pursuit — Disparity Extender の安全マージンと Follow the Gap の
Pure Pursuit 舵角を合わせ、さらに狙点のヒステリシスと安全半幅の速度依存化を
足した反射型プランナ。

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
- 安全マージン: `base.extend_disparity()`（DEと共有する共通実装）による
  「段差の遠い側を車体半幅ぶん塗る」処理。最近傍1点だけを塗る円形バブル
  （FTG）と違い、視野内の**すべての段差**を同時に処理できる
- 狙点: 塗った後の「一番遠い**帯の真ん中**」（DEと同じ。端を狙うとコーナー
  内側を舐める）
- 速度の基礎: 狙う方向の塗り後距離をそのまま使い、正面の扇で測り直さない
  （DEの docstring 参照。測り直すと道幅ぶんの壁を拾って直線でも減速し続ける）
- 舵角: `nav.purepursuit.steer_for_target()`（FTGと同じ。η=狙点方位、
  Ld=速度比例で、ギャップの奥行き＝塗り後距離を安全上限にする）
- 曲率ベースの速度上限: Pure Pursuit が使う舵角 `δ` から実際に描く円弧の
  曲率 `κ=tan(δ)/L` を求め、円運動の横加速度制限 `v ≤ sqrt(a_lat_max/κ)` を
  速度の物理的な上限にする（2026-09-12 に DE 側もこの式へ揃えたので、今は
  FTG/DE/DP の3つとも共通）。`a_lat_max`（既定 3.0 m/s²）は実車未計測のため
  暫定値。**実測して詰めること。**

## ここから新規に追加したもの①: 狙点のヒステリシス

DEの同着判定（`TIE_M`以内なら「正面に近い方」を採る）は、**前回どちらを
向いていたか**を見ない。通路が湾曲している場面では、僅差で最遠帯が
フレームごとに入れ替わりうる場所で「正面に近い方」という基準自体が
毎回同じ2択の間で振動する引き金になりうる。ここでは基準を「前回向いて
いた方位に一番近い帯」に一般化する。前回ヘディングが0付近（直進中）なら
DEと同じ挙動に自然収束するので、直線・素直なカーブでの走りは変えない。

## ここから新規に追加したもの②: 安全半幅の速度依存化

Pure Pursuit の前方注視距離 `Ld = look_k・v + look_min` は速度に比例して
伸ばしているのに、`extend_disparity` の `safety_half_width` は速度に
関わらず固定だった。しかし高速ほど「操舵の応答遅れ（`tau_steer_s` +
`dead_time_s`、`vehicle.toml` 実測値）の間に進む距離」も「旋回時に外側へ
膨らむ量」も大きくなるはずで、DEが既定値を車体半幅 0.09m の3倍に
決め打ちしているのも「旋回半径ぶんの膨らみをここで飲むため」（DEの
docstring参照）——つまり**低速前提で決めた固定マージンを高速域まで
流用している**。

    有効安全半幅 = safety_half_width + safety_width_k・v

`Ld` の式と対になる形にし、`v=0` では従来（DEの実測値）と完全に一致する
よう加算式にした（掛け算にすると低速域の実測結果まで変わってしまう）。
`safety_width_k` は実車・シム未計測の暫定値。**小さめに倒してある** ——
大きくしすぎると悪影響のほうが先に出る（段差の陰が塗り広がりすぎて、
本来は通れる隙間まで塞いでしまう）ため、上げるときはシムで隙間を
塞ぎすぎていないか確認しながら詰めること。

## 緊急停止は持たない（2026-09-12）

以前は固定距離 `stop_dist` によるハード停止と、接近速度から見積もる
`ttc_min`（TTC・衝突余裕時間）の2段構えの緊急停止を持っていたが、STM32 の
`auto_stop`（★v0.12・速度に応じて伸びる動的停止距離 `d_stop`）の方が
高性能なため両方とも撤去した（`follow_the_gap.py` docstring参照）。
塗った結果どこにも進めない判定（`_STUCK_M`）は残す——これは「衝突しそう」
ではなく「計画そのものが失敗している」ケース。

## 実車・シムでの計測はまだ行っていない

`disparity_m` / `safety_half_width` / `look_k` / `look_min` / `steer_tau` の
既定値は `DisparityExtender` / `FollowTheGap` の実測値をそのまま引き継いだ
ものだが、`a_lat_max` / `max_speed` / `safety_width_k` はこの Planner
独自の値で未計測。FTG/DEと同じ手順（シムで衝突なしを確認 → 実車で上げる）
を踏むこと。
"""

from __future__ import annotations

import math

from ..core.vehicle import Vehicle
from ..msgs.types import AutoState, Scan, VehicleState
from ..nav.purepursuit import steer_for_target
from .base import ParamSpec, Planner, extend_disparity, min_filter, scan_window

__all__ = ["DisparityPursuit"]

#: 視野内でセクタが見えている割合がこれを下回ったら計画を放棄する。
#: **パラメータにしていない。**（`follow_the_gap.py` と同じ理由）
MIN_SEEN_RATIO = 0.6
#: 狙点の候補と見なす「一番遠い」の許容差 [m]。これ以内は同着として扱い、
#: **その中で前回ヘディングにいちばん近い方向**を採る（ヒステリシス）
TIE_M = 0.05
#: 塗った結果の最遠距離がこれ以下なら「どこにも進めない」（`ready=False`）。
#: **緊急停止のしきい値ではない**（`disparity_extender.py`の`_STUCK_M`と同じ）
_STUCK_M = 0.05


class DisparityPursuit(Planner):
    id = "dp"
    name = "Disparity Pursuit"
    description = "段差を塞いでから最遠の帯を追う。舵はPure Pursuit、速度は旋回曲率で決める"

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
        ParamSpec(key="safety_width_k", label="安全半幅の速度係数", min=0.0, max=0.15,
                  step=0.005, default=0.02, unit="s",
                  note="★未計測の暫定値。有効安全半幅 = safety_half_width + これ×速度。"
                       "Ld=look_k×v+look_minと対になる発想で、高速ほど操舵応答遅れの間に"
                       "進む距離や旋回時の膨らみが増える分を反映する。上げすぎると"
                       "本来通れる隙間まで塞ぐので、シムで詰まっていないか確認しながら上げること"),
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
        ParamSpec(key="steer_tau", label="舵の平滑化", min=0.0, max=0.5, step=0.01,
                  default=0.08, unit="s",
                  note="舵指令の1次遅れの時定数。0 で平滑化なし。上げると滑らかだが鈍る"),
    )

    def __init__(self) -> None:
        self.vehicle = Vehicle.load()
        self._steer = 0.0
        self._heading_deg = 0.0            # ヒステリシス: 前回向いていた方位

    def reset(self) -> None:
        self._steer = 0.0
        self._heading_deg = 0.0

    # ── 本体 ──

    def plan(self, scan: Scan, vs: VehicleState | None,
             p: dict[str, float], dt: float) -> AutoState:
        st = AutoState(mode=self.id, planner=self.name)

        # `extend_disparity` の安全半幅にも使うので、狙点を決める前に確定させる
        v_now = vs.speed if vs is not None else 0.0

        max_range = p["max_range"]

        # ── 視野の切り出しと前処理（`base.scan_window`）──
        w = scan_window(scan, p["fov_deg"], max_range)
        degs, measured = w.degs, w.measured
        st.valid_ratio = w.valid_ratio

        if w.seen_ratio < MIN_SEEN_RATIO:
            st.reason = (f"点群の欠測が多すぎる（視野の {w.seen_ratio * 100:.0f}% しか"
                         f"受信できていない）")
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

        # ── 段差を埋める（安全マージン。DEと同じ処理＋速度で半幅を伸ばす）──
        half_width = p["safety_half_width"] + p["safety_width_k"] * v_now
        ext = extend_disparity(usable, p["disparity_m"], half_width)
        st.bubble_start_deg = 0.0
        st.bubble_end_deg = -1.0            # DEと同じく円形バブルは置かない

        # ── 一番遠い帯の真ん中を狙う（同着はヒステリシスで解く）──
        best = max(ext)
        if best <= _STUCK_M:
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
        lookahead_cap = ext[j_best]
        ld = min(lookahead_cap, p["look_k"] * v_now + p["look_min"])
        target = steer_for_target(st.heading, ld, self.vehicle.wheelbase, max_steer)
        # 時間ベースの1次遅れ。フレームレートに依存させない
        tau = p["steer_tau"]
        alpha = 1.0 if tau <= 1e-3 or dt <= 0 else 1.0 - math.exp(-dt / tau)
        self._steer += (target - self._steer) * alpha
        st.target_steer = self._steer

        # ── 速度: 見通しベースと曲率ベースの小さい方を採る ──
        #
        # 見通しベース: `ext[j_best]` をそのまま使う。DEと同じ理由で扇で測り
        # 直さない（正面 ±front_deg の最小値だと道幅ぶんの側壁を拾い、
        # どんな直線でも見通しが頭打ちになる）。0m（接触寸前）で最低速度
        slow_d = max(p["slow_dist"], 1e-3)
        v_max = p["max_speed"]
        v_min = min(p["min_speed"], v_max)
        ratio = max(0.0, min(1.0, lookahead_cap / slow_d))
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
