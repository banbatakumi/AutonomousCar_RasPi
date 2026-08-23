"""Follow the Gap — LiDAR だけで「一番広く空いている方向」へ走る。

地図も自己位置も要らない**反射的な**走り方。SLAM を作る前に、点群が本当に
使える品質で届いているかを実車で確かめるための最初の自動運転として置いている。

## 手順

    ① 視野を切り出す         前方 ±fov/2 度だけを見る
    ② 前処理                 欠測・飽和・測距不能を安全側に倒し、最小値フィルタを掛ける
    ③ 安全バブル             最近傍の点の周りを「侵入禁止」で塗り潰す
    ④ ギャップ探索           gap_min[m] 以上が続く最長の区間を選ぶ
    ⑤ 狙点                   その区間の**真ん中**を向く
    ⑥ 速度                   正面の余裕と、実際に切る舵から求めた曲率で落とす

⑤ を「区間の中で一番遠い点」にする流儀もある。**真ん中を採っている**のは、
一番遠い点を狙うと壁際すれすれを縫う挙動になり、車幅ぶんの余裕が無い
ミニカーだと外輪が壁に当たるため。真ん中なら通路の中央に寄る。

角度から舵角への変換は `nav.purepursuit.steer_for_target()`（Pure Pursuit の
δ = atan(2L·sinη / Ld)）を使う。**η はギャップ中央の方位。** 地図も経路点列も
要らない — Pure Pursuit の式は自分を原点とした目標点の方位と距離さえあれば
決まるので、LiDAR から毎周期直接それを作れる FTG とは相性がよい。

## Ld（前方注視距離）は速度比例。ギャップの奥行きではない

`Ld = look_k·v + look_min`（`RaceLine` の RACE フェーズ・`nav/purepursuit.py` と
同じ式）。速度が上がるほど遠くを狙って滑らかに、下がるほど近くを狙って鋭く曲がる。

★ **最初はギャップの奥行き（区間の平均距離）を Ld にしていたが、これは
コーナーで曲がりきれず壁に当たる欠陥があった。** 交差点やコーナーでは曲がる
方向のギャップが奥まで開けていることが多く、奥行きをそのまま Ld にすると
「奥まで見えるから急がなくていい」という判断になる。しかし実際にはその場の
通路幅の中で曲がりきらないといけないので、舵が緩すぎて外側の壁に当たる。
速度比例なら、壁に近づいて減速するとき（`slow_dist`／`turn_slow`）に Ld も
一緒に縮むので、**減速している＝鋭く曲がりたい瞬間ほど鋭く曲がる**、という
向きが自然に揃う。

ギャップの奥行きは完全に捨てるわけではなく、Ld の**安全上限**として残す
（`min(奥行き, look_k·v + look_min)`）。見えている範囲より先を目標点にしない
ため。`_longest_run()` が選ぶ区間は既に `gap_min` 以上なので、この上限が
効くのは高速で `look_k·v + look_min` が奥行きを超えるような場合だけ。

## 速度は「正面の余裕」と「曲率」の両方で頭打ちにする

正面の余裕（`free_ahead`）による減速は従来通り。★ それとは別に、実際に切る
舵角から円運動の曲率 `κ = tan(δ)/L` を求め、横加速度制限
`v ≤ sqrt(a_lat_max/κ)` を速度のもう一つの上限にする（`DisparityPursuit`
の②と同じ式・同じ理由。`auto/gap_pursuit.py` の docstring 参照）。

**以前はここが `turn_slow`（舵角いっぱいで一律 x% 減速）という発見的な調整
だった。** 緩いカーブもきついカーブも同じ割合でしか区別できず、`turn_slow`
の意味も車体（ホイールベース・最大舵角）や `a_lat` の実測値と無関係だった。
曲率ベースにすると、緩い旋回は落としすぎず、きつい旋回は物理的な根拠がある
分だけ確実に落ちる。κ は**クランプ後の `target`**（平滑化前）から求める。
舵が頭打ちのときでも実際に描く円弧に対して正しく、しかも平滑化の遅れぶん
速度側は先回りして落ちる（安全側）。

## 前処理（②）は `base.scan_window()` にある

欠測・飽和・測距不能をどう読むかは**点群の読み方の契約**そのもので、
`Disparity Extender`（`auto/disparity_extender.py`）と共有している。
片方だけ直すともう片方が古い読み方のまま走るので、**ここには書き写さない。**
`stop_dist` が「測距不能を空き扱いにしている穴」を受ける側であることも同様。
"""

from __future__ import annotations

import math

from ..core.vehicle import Vehicle
from ..msgs.types import AutoState, Scan, VehicleState
from ..nav.purepursuit import steer_for_target
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
        ParamSpec(key="look_k", label="前方注視の速度係数", min=0.0, max=2.0, step=0.05,
                  default=0.7, unit="s",
                  note="Ld = 係数×速度 + 最小値。上げると滑らかだがコーナーで曲がりきれなくなる"),
        ParamSpec(key="look_min", label="前方注視の最小値", min=0.15, max=1.5, step=0.05,
                  default=0.35, unit="m",
                  note="低速時の注視距離。小さすぎると舵が振動する"),
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
        ParamSpec(key="max_steer", label="最大舵角", min=0.1, max=0.524, step=0.005,
                  default=0.50, unit="rad",
                  note="★io_node の --max-steer を超えても切り捨てられるだけ"),
        ParamSpec(key="a_lat_max", label="旋回時の横加速度上限", min=0.5, max=8.0, step=0.1,
                  default=3.0, unit="m/s²",
                  note="★実車未計測の暫定値。実際に切る舵角から曲率 κ=tan(δ)/L を求め、"
                       "v ≤ sqrt(これ/κ) で速度を抑える（`DisparityPursuit` と同じ式）。"
                       "上げるほど旋回中に速度が残るが横滑りしやすくなる"),
        ParamSpec(key="steer_tau", label="舵の平滑化", min=0.0, max=0.5, step=0.01,
                  default=0.10, unit="s",
                  note="舵指令の1次遅れの時定数。0 で平滑化なし。上げると滑らかだが反応が鈍る"),
    )

    def __init__(self) -> None:
        self.vehicle = Vehicle.load()
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
            a, b, _depth = gap
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

        a, b, depth = gap
        st.ready = True

        # ── ⑤ 狙点はギャップの真ん中。舵角は Pure Pursuit の式で作る ──
        # η ＝ ギャップ中央の方位。Ld は速度比例（`vs` が届いていなければ
        # 速度不明 ＝ 0 とみなす。**Ld が最小になる ＝ 最も鋭く曲がる側に倒れる**
        # ので安全側）。ギャップの奥行きは Ld の安全上限としてだけ残す
        mid = (degs[a] + degs[b]) / 2.0
        st.heading = math.radians(mid)

        max_steer = p["max_steer"]
        v_now = vs.speed if vs is not None else 0.0
        ld = min(depth, p["look_k"] * v_now + p["look_min"])
        target = steer_for_target(st.heading, ld, self.vehicle.wheelbase, max_steer)
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

        # 曲率ベースの物理的な上限。実際に切る舵角（クランプ後の `target`。
        # 平滑化前）から曲率を求め、円運動の横加速度制限で頭打ちにする
        # （`DisparityPursuit` の②と同じ式。`gap_pursuit.py` の docstring）
        kappa = abs(math.tan(target) / self.vehicle.wheelbase)
        v_curve = math.sqrt(p["a_lat_max"] / kappa) if kappa > 1e-6 else math.inf
        st.target_speed = max(v_min, min(v, v_curve, v_max))

        width = degs[b] - degs[a]
        st.reason = (f"{mid:+.0f}° のギャップ（幅 {width:.0f}°）へ・"
                     f"正面 {st.free_ahead:.2f}m・曲率制限 {min(v_curve, v_max):.2f}m/s")
        return st


# ── 小道具 ────────────────────────────────────────────────────────────

def _longest_run(r: list[float], threshold: float) -> tuple[int, int, float] | None:
    """`threshold` 以上が連続する最長区間 `(start, end, mean_depth)`（両端含む）。

    同じ長さが並んだときは**平均距離が大きい方**を採る。左右対称な通路の
    真ん中に立ったときに、毎周期どちらを選ぶか揺れて舵が振動するのを防ぐ
    ……ためではなく（それは平滑化の仕事）、単純に**より奥まで抜けている方**が
    通路として正しいため。

    `mean_depth`（区間の平均距離）は呼び出し側で Pure Pursuit の前方注視距離
    `Ld` として使う。区間は必ず `threshold` 以上の値だけで構成されるので、
    `mean_depth` も自動的に `threshold` 以上になる
    （`Ld` の下限に専用パラメータが要らない理由）。
    """
    best: tuple[int, int, float] | None = None
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
            best, best_len, best_mean = (j, k, mean), length, mean
        j = k + 1
    return best
