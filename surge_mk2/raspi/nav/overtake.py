"""障害物・他車を追い抜く一時経路（Frenet 格子 × 時刻つきの当たり判定）。

レーシングラインに対する横ずれ `d(s)` の候補をまとめて作り、障害物の**その時刻の位置**と
重ならないものの中から、窓を抜ける時間がいちばん短いものを選ぶ。Werling ら (2010) の
Frenet 格子、TUM の graph-based local planner (Stahl 2019)、ForzaETH の Predictive Spliner
(Baumann 2024) と同じ考え方。

    d(s) = 5次多項式  今の状態 (d0, d0', d0'') → (dT, 0, 0)        0 ≤ s < L1   （入る）
         = dT                                                     L1 ≤ s < S2  （並ぶ）
         = dT · (1 − smoothstep((s − S2) / L3))                   S2 ≤ s       （戻る）

## `avoid.plan_offset` との違い

- **今の横ずれから出発する。** 避けている途中・戻る途中でも毎周期引き直せる（前の計画の上に
  居れば前の計画の状態から出発するので、同じ候補を選べば形は変わらない）
- **時刻で判定する。** 候補の速度（元の経路の速度・曲率で決まる速度・今の速度から加減速できる
  速度）から各点を通る時刻を出し、障害物をその時刻の位置（`s + u·t`、`d + w·t`）に置いて重なりを
  見る。静止物は `u = w = 0` で同じ式。同方向の遅い車は「抜く間に相手が進む」ぶん並ぶ区間が伸び、
  抜く前に追いつかない候補（相手が先へ逃げる）は塞がれない
- **形を決め打ちにしない。** 入る長さ・並ぶ長さ・戻る長さを格子で振り、曲率と道幅（`room`）で
  捨てる。曲がれる限界まで使う短い入り方も候補に入る（どれを選ぶかは時間で決まる）
- **避ける間に速度を抑えない。** 速度は曲率と摩擦だけで決まる（安全は横・前後の余裕で持つ）

障害物は Frenet 上の矩形（経路に沿った範囲 × 横の範囲）で扱う。LiDAR に見えるのは手前の面だけ
なので、奥（経路の先の側）へ見えない厚みを足す: 動く物は車の長さ `opp_length`、静止物は見えている幅。
動く物は時間とともに位置の不確かさが増えるとして、範囲を `u_sigma·t`・`w_sigma·t` ずつ広げる。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import NamedTuple

import numpy as np

from .avoid import AvoidPlan
from .centerline import normals
from .obstacles import MOVING_MIN_SPEED, Obstacle
from .raceline import ENV_U, RaceLine, curvature

__all__ = ["OvertakeConfig", "Track", "Cand", "OvertakeResult", "frenet_state", "frenet_tracks",
           "advance_along", "opp_motion", "line_profile", "line_shift", "pred_spread", "calibrate_spread", "plan",
           "eval_cand", "build"]

#: 格子の点の間隔 [m]（経路の点 0.1m の 2〜3 つおき）。速いときは粗くする（車体の長さ 0.37m より
#: 細かければ、静止物を飛び越す判定にはならない）
_STEP = 0.2
_STEP_FAST = 0.3
#: 見る先の長さ: `max(_H_MIN, v·_H_T)`、上限 `_H_MAX` [m]
_H_MIN = 6.0
_H_T = 2.5
_H_MAX = 12.0
#: 障害物を数える経路からの横の範囲 [m]（これより外は道の外か別の道）
_LAT_MAX = 1.5
#: 戻る区間の始まりを振る間隔 [m]
_S2_STEP = 0.4
#: 前の計画の上に居るとみなす横のずれ [m]（超えたら今の姿勢から出発する）
_ON_PLAN_M = 0.08
#: 左右を入れ替える候補に足す時間 [s]（判断のばたつきを抑える）
_SWITCH_COST = 0.15
#: 見る先の終わりで経路から外れている量に掛ける時間 [s/m]（戻れるなら戻る）
_END_COST = 0.4
#: 横ずれの大きさに掛ける時間 [s/m]（同じ時間なら小さくずれる）
_LAT_COST = 0.05
#: 前の計画と同じ形の候補を選び直すときの値引き [s]
_KEEP_BONUS = 0.05
#: 経路の上に居るとみなす横ずれ [m]（ここから外れていなければ「戻る」計画は要らない）。追従の誤差（±2〜4cm）より
#: 大きくする。★0.02 だと線の上を走っているだけで毎周期「線へ戻る」計画が「避ける」として選ばれ、付いていく・
#: 避けるが周期ごとに入れ替わった（sim.bench --mover）
_ON_LINE_M = 0.05
#: 元の線が曲がれる限界を超えている所で、さらに急にしてよい量（限界に対する割合）
_KAPPA_SLACK = 0.1
#: 障害物を経路の枝へ射影するとき、距離の谷から前後に見る範囲 [m]（`frenet_tracks`）
_BRANCH_M = 0.8
#: 動く相手の前側の位置の不確かさ: 相手が予測で進む距離に対する割合（その p90）。場所ごとに覚えた速さで
#: 予測したとき（`line_profile`）と、今の速さと自分の線の速度の比で伸ばしたとき（`opp_motion`）。
#: 記録（sim.bench --mover、追跡は EKF）の経路に沿った位置の誤差 p90 ÷ 進んだ距離から: 覚えた速さ 2 秒先
#: 0.31〜0.63m（約 3m 進む間）、比 0.73〜1.02m（`docs/overtake.md`）
_GROW_LEARNED = 0.2
_GROW_RATIO = 0.3
#: まだ動いたことの無い物の広がりの上限 [m]（見え始めは速度の σ が 2m/s。動いても数 s でこれ以上は離れない道幅の中）
_GROW_UNKNOWN_MAX = 0.6
#: 予測を伸ばす時間の上限 [s]。★追い付いて並び、抜き切るまで（見る先 `_H_OT`）を予測が覆う長さにする。
#: 3 秒では、遅い相手に追い付く地点が予測の外になり、そこで横へ出る候補を評価できなかった
_T_PRED_MAX = 5.0
#: 相手を予測で進める長さ [s]。当たり判定は「横へ出始める時刻＋`_T_PRED_MAX`」まで（`eval_cand`）。区間の入口まで
#: 付いていく候補は、今は付いていくだけで、横へ出るかは入口でその時の予測で確かめ直すので、確かめる範囲は横へ
#: 出始める時刻から数える。予測の不確かさは進む距離に比例して広げる（`_GROW_*`）
_T_PRED_LONG = 10.0
#: 動く相手がいるときの見る先 [m]。追い付くまで（車間 1〜2m を詰める）＋横へ出る＋並ぶ（相手 0.8 倍で約 4.5m）
#: ＋戻るを入れる。速さで決める見る先（`_H_T`、2m/s で 6m）では並ぶ途中で切れ、抜き切る候補が作れなかった
_H_OT = 12.0
#: これより手前で横へ出始める候補は、今その追い抜きを始めるものとして抜き切りまで確かめる [m]（`eval_cand`）
_COMMIT_M = 1.0
#: 横へ出始めるのを遅らせる候補で、出終わる位置を追い付く地点よりどれだけ手前に置くか [m]（`_catch_point`）
_S1_BACKS = (0.0, 0.5, 1.0)
#: 同じく、戻り始めを振る間隔 [m]
_S2_STEP_LATE = 0.8
#: 前の相手に付いていく候補が、当たり判定の前後の余裕（`margin_long`＋予測の広がりの上限）に足す車間 [m]
_FOLLOW_SLACK = 0.05
#: 付いていく候補の「今すでに車間の内側か」の判定で許す量 [m]（相手の後ろ面の観測の揺れ。`_follow_times`）
_INSIDE_TOL = 0.05
#: 乗っている計画を続ける候補の確かめで許す道幅 [m] と曲率の割合（`eval_cand` の `keep_last`）。★時間が最短の候補は
#: 道幅が尽きる直前で戻るので道幅の境目に居り、毎周期の格子のずれ（0.2〜0.3m）による標本化の差で、一度確かめて乗った
#: 計画が落ちて「横へ出ては戻る」を繰り返した（理想化した検証、相手 0.7 倍）。道幅には壁との余裕（`avoid_clear`）が
#: 別に入っている
_KEEP_ROOM_TOL = 0.02
_KEEP_KAPPA_TOL = 0.03


@dataclass(frozen=True)
class OvertakeConfig:
    half_width: float                      #: 車体の半幅 [m]
    front: float                           #: base_link から車体前端 [m]
    rear: float                            #: base_link から車体後端 [m]
    kappa_max: float                       #: 経路に許す曲率の上限 [1/m]
    a_brake: float                         #: 減速度 [m/s²]
    a_accel: float                         #: 加速度 [m/s²]
    #: 前後の上限の包絡 `(加速, 減速)`（`raceline.ENV_U` の各点、横の使い具合 u ごと。走行中に学習した g-g 図、
    #: `grip.LongLearner`）。None なら摩擦円 `a_accel·√(1−u²)`・`a_brake·√(1−u²)`。速度プロファイル（`speed_profile`）と
    #: 同じモデル（`_Long`）
    env: tuple | None = None
    margin_lat: float = 0.10               #: 障害物の横の余裕 [m]
    #: 動く物（他車）の横の余裕 [m]。`None` なら `margin_lat`。★静止物と同じ 0.20m だと、横に並ぶのに
    #: 経路から 0.46m 要り、toyota2 で片側にそれだけ空く区間（3〜8m）が抜き切る距離（相手 0.7 倍で約 9m）
    #: より短く、完全な知覚でもどこからも抜けなかった。0.12m なら全ての出発点から抜けた（0.15m でも 0）。
    #: 相手の横の動きは `w_sigma`・`w_grow_max` で別に足す
    margin_lat_moving: float | None = None
    margin_long: float = 0.10              #: 障害物の前後の余裕 [m]
    #: 動く物（他車）の長さ [m]。後ろ面しか見えないので、奥へこれだけ足す
    opp_length: float = 0.40
    #: 動く物の位置の不確かさが時間とともに増える速さ [m/s]（前後・横）と、その上限 [m]。
    #: 横は道の中で走っている車なので大きくは動かない（上限を小さく）
    u_sigma: float = 0.15
    w_sigma: float = 0.04
    u_grow_max: float = 0.40
    w_grow_max: float = 0.06
    #: 抜いた動く相手の前へ戻るときに空ける時間 [s]（相手の速さ × これを前後の余裕に足す）。
    #: 以前は 0.3（相手の速度を直線の回帰で出していた頃、曲がる相手の速さが弦のぶん遅く出て、抜いた直後に
    #: 前へ戻ると後ろから当たられた）。追跡を EKF にして速さの偏りが消えた（真値の 0.97〜0.99 倍。回帰は
    #: 0.87〜0.89 倍）ので 0。★0.3 のままだと、相手 0.8 倍で並ぶ区間が 4.9 → 7.9m になり、toyota2 の
    #: 片側が空く最長の区間（約 6.8m）に収まらない（`docs/overtake.md`）
    cutin_headway: float = 0.0
    #: 前の相手に付いていく最小の車間（車体前端から相手の後ろ面）[m]。`None` なら前後の余裕＋予測の広がりの
    #: 上限＋`_FOLLOW_SLACK`。**付いていく速さはこの計画が決める**（`OvertakeResult.speeds` が速度指令になる）
    follow_keep: float | None = None
    #: 計画し直しが効くまでの遅れ [s]（同定値。`Slam2dRaceLine` は `react_s`）。車間にこの間の相手の予測の不確かさ
    #: （`_GROW_*` × 相手の速さ × これ）を足す。★速度そのものの遅れは制御の先読み（`speedctl.ref_command` の
    #: `lead_s`）が受け持つので、計画で「この間は今の速さのまま」と二重に置かない（置くと制御は毎周期その保持区間を
    #: 読み、減速の指令が出ずに追突した。相手 0.75 倍で 1 本に 19 回）
    latency_s: float = 0.0
    #: 減速の指令が実際の減速になるまでの時間 [s]（`Slam2dRaceLine`: 指令の先読み `_ref_lead` ＋車速制御の時定数
    #: `speed_loop_tau_s`。どれも同定値）。付いていく車間に「近づく速さ × これ」を足す。★計画は「ぎりぎりまで保って減速度の
    #: 上限で落とす」形になる一方、実際の減速は指令から遅れて立ち上がる（減速を求めている間に出たのは 1.83m/s²、車の
    #: 減速の上限は 2.54m/s²）。見込まないと、相手に近づきながら付く所で毎回間に合わず追突した（車間 0.3m・相手 0.75 倍で
    #: 1 本に 16〜19 回）
    cmd_lead_s: float = 0.0
    #: 覚えた速さで予測する相手の前側の広がり（予測で進む距離に対する割合）。走行中に予測の外れを実測した値
    #: （`calibrate_spread`。呼び出し側が 1 秒に 1 回ほど出す）。None なら一定の `_GROW_LEARNED`
    grow_learned: float | None = None



def _margin(cfg: OvertakeConfig, moving: bool) -> float:
    """障害物の横の余裕（動く物は `margin_lat_moving`）。"""
    if moving and cfg.margin_lat_moving is not None:
        return cfg.margin_lat_moving
    return cfg.margin_lat


class Track(NamedTuple):
    """Frenet 上の障害物（車の今の位置を s=0 とした経路に沿った範囲と、経路からの横の範囲）。"""

    s_lo: float
    s_hi: float
    d_lo: float
    d_hi: float
    u: float                               #: 経路に沿った速度 [m/s]（負 = 逆走）
    w: float                               #: 経路から横へ離れる速度 [m/s]（左が正）
    moving: bool
    src: Obstacle
    #: 速度の推定の標準偏差 [m/s]（追跡 `VelocityTracker` の EKF）。None なら追跡を通っていない（手で作った物。
    #: 予測の広がりは `u_sigma` の決め打ち）
    su: float | None = None


class Cand(NamedTuple):
    """1 つの候補（`s` は出発点 `s0` からの距離）。`s0` は経路の始点からの弧長 [m]。"""

    s0: float
    d0: float
    d1: float
    d2: float
    dT: float
    L1: float
    S2: float                              #: 戻り始め（`inf` = 見る先の中では戻らない）
    L3: float
    #: 横へ出始める位置（出発点からの距離）。それまでは出発点の横ずれのまま（線の上で相手の後ろを走る）。
    #: 0 なら今すぐ出る（`_catch_point`）
    S1: float = 0.0

    def at(self, x: float) -> tuple[float, float, float]:
        d, d1, d2 = _profile(np.array([x]), *(np.array([v]) for v in self[1:]))
        return float(d[0, 0]), float(d1[0, 0]), float(d2[0, 0])


class OvertakeResult(NamedTuple):
    #: "clear" = 経路のまま通れる / "avoid" = `plan` に乗る / "blocked" = 通れる候補が無い
    status: str
    plan: AvoidPlan | None
    cand: Cand | None
    #: 使える候補を良い順に（`cand` が先頭）。呼び出し側が直交座標で確かめ、落ちたら次を使う
    ranked: list
    #: 経路のままだと当たる障害物のうち最初のもの（"blocked" のとき、付いていく相手）
    lead: Track | None
    n_cands: int
    why: str
    #: 選んだのが前の計画をそのまま続ける候補か（形は前と同じ。作り直さなくてよい）
    kept: bool = False
    #: 速度の計画: 経路の添字 `sel`（見る先の格子の点。出発点から順）と、その点ごとの速さ [m/s] の列（`ranked` の
    #: 候補ごと。"clear"・"blocked" では線のまま付いていく最善の 1 本）。呼び出し側は走る経路の速さをこれで抑え、
    #: 速度指令にする（付いていく速さを別の式で決めない）。障害物が無く計算しなかったときは None
    sel: np.ndarray | None = None
    speeds: list | None = None
    #: 線のまま前の相手に付いていく最善の速度の計画（`speeds` と同じ点。避ける候補が直交座標の確かめで全部落ちた
    #: ときの付いていく速さ）
    line_speed: np.ndarray | None = None
    #: 付いていく相手（`lead`）の今の場所での予測の速さ [m/s]（付いていく計画と同じ、覚えた速さの遅い側。覚えていない
    #: 場所は今の速さの比で伸ばした値）。付いていく制御が追跡の生の速さの代わりに使う
    lead_u: float | None = None


def _quintic(d0, d1, d2, dT, L):
    """(d0, d1, d2) → (dT, 0, 0) を長さ L で結ぶ5次多項式の 3〜5 次の係数（配列）。"""
    h0 = dT - (d0 + d1 * L + 0.5 * d2 * L * L)
    h1 = -(d1 + d2 * L)
    h2 = -d2
    c3 = (10.0 * h0 - 4.0 * h1 * L + 0.5 * h2 * L * L) / L ** 3
    c4 = (-15.0 * h0 + 7.0 * h1 * L - h2 * L * L) / L ** 4
    c5 = (6.0 * h0 - 3.0 * h1 * L + 0.5 * h2 * L * L) / L ** 5
    return c3, c4, c5


def _profile(x, d0, d1, d2, dT, L1, S2, L3, S1=0.0):
    """候補 (C,) を点 x (K,) で評価した `(d, d', d'')`（どれも (C, K)）。

    `S1` > 0 の候補は、`S1` まで出発点の状態のまま（`d1 = d2 = 0` の候補だけに使う）、そこから `L1` で入る。"""
    x = np.asarray(x, dtype=float)[None, :]
    d0, d1, d2, dT, L1, S2, L3, S1 = (np.asarray(a, dtype=float)[:, None] for a in (d0, d1, d2, dT, L1, S2, L3, S1))
    c3, c4, c5 = _quintic(d0, d1, d2, dT, L1)
    h2 = 0.5 * d2
    # 入る区間（ホーナー法）。S1 より手前は始点の値、S1 + L1 より先は端の値 (dT, 0, 0) になるよう、x を止める
    xe = np.clip(x - S1, 0.0, L1)
    d = d0 + xe * (d1 + xe * (h2 + xe * (c3 + xe * (c4 + xe * c5))))
    dd = d1 + xe * (d2 + xe * (3 * c3 + xe * (4 * c4 + xe * 5 * c5)))
    ddd = d2 + xe * (6 * c3 + xe * (12 * c4 + xe * 20 * c5))
    # 戻る区間: S2 から L3 で dT → 0。入る区間の先（端の値 dT, 0, 0）を上書きする
    with np.errstate(invalid="ignore"):
        u = (x - S2) / L3
    back = (u > 0.0) & (x >= S1 + L1)
    if back.any():
        u = np.clip(np.nan_to_num(u, nan=0.0), 0.0, 1.0)
        sm = u * u * u * (10.0 + u * (-15.0 + 6.0 * u))
        sm1 = 30.0 * u * u * (1.0 - u) ** 2 / L3
        sm2 = u * (60.0 + u * (-180.0 + 120.0 * u)) / (L3 * L3)
        d = np.where(back, dT * (1.0 - sm), d)
        dd = np.where(back, -dT * sm1, dd)
        ddd = np.where(back, -dT * sm2, ddd)
    return d, dd, ddd


#: Frenet の初期値として扱う向きのずれの上限 [rad]。超えたら追い抜きの経路を作らない（`plan`）
PSI_MAX = 1.2


def frenet_state(path: RaceLine, index: int, x: float, y: float, yaw: float):
    """車の (経路の始点からの弧長 s, 横ずれ d, d' = tan(向きのずれ))。"""
    n = len(path)
    j, j2 = index % n, (index + 1) % n
    t = path.xy[j2] - path.xy[j]
    seg = max(float(np.hypot(*t)), 1e-9)
    t = t / seg
    dx, dy = x - path.xy[j, 0], y - path.xy[j, 1]
    along = dx * t[0] + dy * t[1]
    d = -dx * t[1] + dy * t[0]
    psi = (yaw - math.atan2(t[1], t[0]) + math.pi) % (2 * math.pi) - math.pi
    # tan は ±90° で発散するので有限に抑える。抑えた値（`PSI_MAX` ちょうど）なら Frenet の多項式の初期値として
    # 意味が無いので、`plan` は追い抜きの経路を作らない
    return along, d, math.tan(max(-PSI_MAX, min(PSI_MAX, psi)))


def frenet_tracks(path: RaceLine, order: np.ndarray, s_rel: np.ndarray, nrm: np.ndarray,
                  obstacles: list[Obstacle], cfg: OvertakeConfig) -> list[Track]:
    """障害物を Frenet の矩形にする。`order`・`s_rel` は車の少し後ろから見る先までの経路の添字と
    車からの弧長（負 = 後ろ）。経路から遠い物・窓の外の物は落とす。

    ★ 経路が折り返している所（ヘアピン）では、1 つの障害物が窓の中の 2 つの枝のどちらにも近い。
    いちばん近い点へ射影すると反対側の枝に乗り、線の上の少し先に居る相手が「足元の右 0.9m」に
    見えて、抜いた直後に戻った線で当たった（sim.bench --mover）。重心から経路までの距離の谷ごとに、
    その枝の近く（±`_BRANCH_M`）だけで射影して別々の矩形にする（どちらの枝の上にも居るものとして避ける）。
    """
    out: list[Track] = []
    xy = path.xy[order]
    tn = np.column_stack([nrm[order, 1], -nrm[order, 0]])           # 接線
    K = len(order)
    step = max(float(np.median(np.diff(s_rel))) if K > 1 else 0.1, 1e-3)
    half = max(2, int(round(_BRANCH_M / step)))
    for o in obstacles:
        pts = o.pts if o.pts is not None and len(o.pts) else None
        if pts is None:
            a = np.linspace(0.0, 2 * math.pi, 9)[:-1]
            pts = np.column_stack([o.x + o.r * np.cos(a), o.y + o.r * np.sin(a)])
        cx, cy = float(pts[:, 0].mean()), float(pts[:, 1].mean())
        dc = np.hypot(xy[:, 0] - cx, xy[:, 1] - cy)
        # 距離の谷（両隣以下、端は除く）で、経路から `_LAT_MAX` 以内のもの
        mins = [k for k in range(1, K - 1)
                if dc[k] <= dc[k - 1] and dc[k] <= dc[k + 1] and dc[k] < _LAT_MAX + 0.3]
        taken: list[int] = []
        for m in sorted(mins, key=lambda k: dc[k]):
            if any(abs(m - q) <= half for q in taken):
                continue
            taken.append(m)
            lo, hi = max(0, m - half), min(K, m + half + 1)
            sub = np.arange(lo, hi)
            d2 = ((pts[:, None, :] - xy[None, sub, :]) ** 2).sum(axis=2)
            k = sub[np.argmin(d2, axis=1)]
            if np.all((k == 0) | (k == K - 1)):
                continue
            rel = pts - xy[k]
            s = s_rel[k] + (rel * tn[k]).sum(axis=1)
            d = (rel * nrm[order[k]]).sum(axis=1)
            if float(np.abs(d).min()) > _LAT_MAX:
                continue
            kap = float(path.kappa[order[m]]) if len(path.kappa) == len(path.xy) else 0.0
            out.append(_track(o, s, d, tn[m], nrm[order[m]], cfg, kap))
    return out


def advance_along(path: RaceLine, s_cum: np.ndarray, nrm: np.ndarray, pts: np.ndarray,
                  vx: float, vy: float, dt: float) -> np.ndarray:
    """点 `pts` (N, 2) を、速度 (vx, vy) の経路に沿った成分で `dt` 秒ぶん**経路に沿って**進めた位置。

    各点を経路へ射影して (s, d) にし、s だけ進めて同じ横ずれ d で直交座標へ戻す。★直線で
    `pts + v·dt` と進めると、カーブを走る相手の残像がカーブの外へはみ出し、相手が 20〜30cm 太って
    見えた（外側から抜こうとする自車の候補がすべて重なる。sim.bench --mover）。射影は点群の重心に
    いちばん近い経路の点の前後だけで探す（ヘアピンの反対の枝へ乗らないため）。
    """
    n = len(path)
    L = path.length
    c = pts.mean(axis=0)
    k0 = int(np.argmin(np.hypot(path.xy[:, 0] - c[0], path.xy[:, 1] - c[1])))
    w = max(2, int(round(_BRANCH_M / max(L / n, 1e-3))))
    cand = (k0 + np.arange(-w, w + 1)) % n
    d2 = ((pts[:, None, :] - path.xy[None, cand, :]) ** 2).sum(axis=2)
    k = cand[np.argmin(d2, axis=1)]
    tn = np.column_stack([nrm[k, 1], -nrm[k, 0]])
    rel = pts - path.xy[k]
    s = s_cum[k] + (rel * tn).sum(axis=1)
    d = (rel * nrm[k]).sum(axis=1)
    sn = (s + (vx * tn[:, 0] + vy * tn[:, 1]) * dt) % L
    j = np.clip(np.searchsorted(s_cum, sn, side="right") - 1, 0, n - 1)
    j2 = (j + 1) % n
    seg = np.where(j2 == 0, L - s_cum[j], s_cum[j2] - s_cum[j])
    f = ((sn - s_cum[j]) / np.maximum(seg, 1e-9))[:, None]
    xy = path.xy[j] + f * (path.xy[j2] - path.xy[j])
    nn = nrm[j] + f * (nrm[j2] - nrm[j])
    nn /= np.maximum(np.hypot(nn[:, 0], nn[:, 1]), 1e-9)[:, None]
    return xy + nn * d[:, None]


#: 横の速度を使う経路の曲率の上限 [1/m]（`_track`）
_W_KAPPA_MAX = 0.3


def _track(o: Obstacle, s: np.ndarray, d: np.ndarray, t, n, cfg: OvertakeConfig,
           kappa: float = 0.0) -> Track:
    """射影した点の Frenet 座標 `(s, d)` と、その場所の接線 `t`・法線 `n`・経路の曲率 `kappa` から矩形を作る。"""
    s_lo, s_hi = float(s.min()), float(s.max())
    d_lo, d_hi = float(d.min()), float(d.max())
    if math.isfinite(o.sv):
        # 追跡（EKF）を通った物: 動く・動かないの二択で切らず、速度の推定値とその不確かさで予測する。
        # 車らしい扱い（見えない奥行き・横の余裕・相手の線と速さの予測）は「一度でも確かに動いたか」で決める
        moving = bool(o.moved)
        u = float(o.ex * t[0] + o.ey * t[1])
        w = 0.0 if moving else float(o.ex * n[0] + o.ey * n[1])   # 車の横の動きは覚えた線（`line_shift`）で
        su = float(o.sv)
    else:
        moving, su = o.speed >= MOVING_MIN_SPEED, None
        u = float(o.vx * t[0] + o.vy * t[1])
        w = float(o.vx * n[0] + o.vy * n[1])
    if su is not None:
        pass
    elif not moving:
        u = w = 0.0
    elif abs(kappa) >= _W_KAPPA_MAX and u > 0.0:
        # ★ カーブでは横の速度を使わない。速度は過去 0.8s の位置の直線の回帰（`VelocityTracker`）なので、
        #   向きが今の接線より 0.4s ぶん遅れ（17〜33°）、線の上を走る相手が 0.5m/s で外へ流れていくように
        #   見えた。外から抜く候補がすべて重なり、ヘアピンで抜けなかった（sim.bench --mover。カーブで
        #   前の車の 7 割の周期）。向きが当てにならないので、前へ進む速さは速度の大きさで取る
        u, w = o.speed, 0.0
    elif abs(w) < 0.3:
        w = 0.0                            # 道の中を走る車の横の速度は小さい。推定の揺れを落とす
    # 見えない奥行き（車から遠い側。前の物は経路の先、後ろの物はさらに後ろ）。
    # 動く物は車の長さ、静止物は見えている幅ぶん
    width = d_hi - d_lo
    depth = cfg.opp_length if moving else min(max(width, 0.05), 0.30)
    if 0.5 * (s_lo + s_hi) >= 0.0:
        s_hi = max(s_hi, s_lo + depth)
    else:
        s_lo = min(s_lo, s_hi - depth)
    # 点が少ないと横幅を小さく見積もる（遠くの物）。最低 10cm にする
    if width < 0.10:
        c = 0.5 * (d_lo + d_hi)
        d_lo, d_hi = c - 0.05, c + 0.05
    return Track(s_lo, s_hi, d_lo, d_hi, u, w, moving, o, su)


class _Long(NamedTuple):
    """前後に出せる加速度のモデル（速度プロファイル `raceline.speed_profile` と同じ）: 横の使い具合
    u = v²|κ| ÷ 横の限界 `grip` ごとに、包絡 `env` があればその u での値、無ければ摩擦円 `a·√(1−u²)`。

    ★以前は一定の `a_accel`・`a_brake` で計算していた。横を大きく使う所（ヘアピンの入口）でも上限いっぱい減速できる
    前提になり、付いていく計画が「ぎりぎりのブレーキ」になって実際には間に合わず追突した（計画 2.54m/s² に対し、
    減速を求めている間に実際に出たのは約 1.96m/s²）"""

    grip: np.ndarray                       #: (K,) 横の限界（摩擦円の半径）[m/s²]
    a_accel: float
    a_brake: float
    env: tuple | None

    def acc(self, v, kap, k=None):
        g = self.grip if k is None else self.grip[k]
        u = v * v * np.abs(kap) / np.maximum(g, 1e-6)
        if self.env is not None:
            return np.interp(u, ENV_U, self.env[0])
        return self.a_accel * np.sqrt(np.maximum(1.0 - u * u, 0.0))

    def dec(self, v, kap, k=None):
        g = self.grip if k is None else self.grip[k]
        u = v * v * np.abs(kap) / np.maximum(g, 1e-6)
        if self.env is not None:
            return np.interp(u, ENV_U, self.env[1])
        return self.a_brake * np.sqrt(np.maximum(1.0 - u * u, 0.0))


def _path_ds(ds: np.ndarray, kr: np.ndarray, d: np.ndarray, dd: np.ndarray) -> np.ndarray:
    """横ずれ `d(x)` の経路が格子の点の間に走る道のり (C, K-1)。経路の弧長 `ds` に `√((1−κr·d)² + d'²)` を掛けた
    もの。★以前は時刻を経路の弧長で出していて、カーブの外側を回る候補の時刻を短く（κ=0.45・外へ 0.4m で 18%）、
    内側を長く見積もっていた。外から抜く候補は追い付く時刻が早すぎ、内から抜く候補は遅すぎた"""
    f = np.sqrt((1.0 - kr[None, :] * d) ** 2 + dd * dd)
    return ds[None, :] * 0.5 * (f[:, :-1] + f[:, 1:])


def _speeds(kap_c, v_cap, a_lat, ds, v_now, cfg: OvertakeConfig, lon: _Long | None = None):
    """候補 (C, K) の速度: 曲率で決まる上限 → 後ろから減速 → 今の速度から加速。点 k から出る加減速はその点の
    横の使い具合で決まる（`_Long`。速度プロファイルと同じ）。今の速度から減速し切れない候補は `feasible=False`。"""
    if lon is None:
        lon = _Long(np.full(len(v_cap), 1e9), cfg.a_accel, cfg.a_brake, None)
    with np.errstate(divide="ignore"):
        v = np.minimum(v_cap[None, :], np.sqrt(a_lat[None, :] / np.maximum(np.abs(kap_c), 1e-6)))
    K = v.shape[1]
    # 道のり: 候補ごと (C, K-1)（`_path_ds`）でも経路の弧長 (K-1,) でもよい
    dsm = np.broadcast_to(ds, (v.shape[0], K - 1))
    for k in range(K - 2, -1, -1):
        a = lon.dec(v[:, k + 1], kap_c[:, k + 1], k + 1)
        v[:, k] = np.minimum(v[:, k], np.sqrt(v[:, k + 1] ** 2 + 2.0 * a * dsm[:, k]))
    ok = v[:, 0] >= v_now - 0.3
    v[:, 0] = np.minimum(v[:, 0], max(v_now, 0.0))
    for k in range(K - 1):
        a = lon.acc(v[:, k], kap_c[:, k], k)
        v[:, k + 1] = np.minimum(v[:, k + 1], np.sqrt(v[:, k] ** 2 + 2.0 * a * dsm[:, k]))
    return np.maximum(v, 0.05), ok


def _collide(x, d, dd, t, tracks: list[Track], cfg: OvertakeConfig, start_ok: np.ndarray,
             d_start: float = 0.0, motion: list | None = None, grow_rel: list | None = None,
             t_lim: np.ndarray | None = None, slow: list | None = None):
    """候補ごとに当たる障害物があるか (C,) と、当たる最初の障害物の番号 (C,)（無ければ -1）。

    `grow_rel[j]` があれば、動く相手 j の前後の不確かさを**前側だけ**、予測で進む距離の `grow_rel[j]` 倍とする
    （抜く車に危ないのは相手が予測より速いときだけ。遅ければ早く抜けるだけで、後ろに付いている間は毎周期
    引き直す）。無ければ前後とも `u_sigma·t`（上限 `u_grow_max`）。

    `motion[j]` が `(t_arr, s_arr[, lat_arr])` なら、障害物 j は時刻 t に `s_arr` を `t_arr` で補間した距離だけ
    進んでいる（`opp_motion`。経路の速度プロファイルに沿って走る相手）。`lat_arr` があれば横へもそれだけ
    ずれる（`line_shift`。覚えた相手の線）。`None` なら `u·t`。
    """
    C = d.shape[0]
    hit = np.zeros(C, dtype=bool)
    first = np.full(C, -1)
    #: 前後で重なる間の、横の間隔の最小 [m]（余裕を引いた後。負 = 重なる）。通れる候補が無いときに
    #: 「いちばん離れていられる」候補を選ぶのに使う
    gap = np.full(C, np.inf)
    swing = cfg.half_width + cfg.front * np.abs(dd)              # 向きのずれで角が横へ出るぶん
    # 点と点の間（0.2〜0.3m）でも横ずれは変わる。前後の点との間で取りうる範囲を車体の横の範囲にする
    d_up = np.maximum(d, np.maximum(np.roll(d, 1, axis=1), np.roll(d, -1, axis=1)))
    d_dn = np.minimum(d, np.minimum(np.roll(d, 1, axis=1), np.roll(d, -1, axis=1)))
    d_up[:, 0], d_dn[:, 0] = np.maximum(d[:, 0], d[:, 1]), np.minimum(d[:, 0], d[:, 1])
    d_up[:, -1], d_dn[:, -1] = np.maximum(d[:, -1], d[:, -2]), np.minimum(d[:, -1], d[:, -2])
    car_lo, car_hi = x[None, :] - cfg.rear, x[None, :] + cfg.front
    tt = np.minimum(t, _T_PRED_MAX)
    lim = np.full((t.shape[0], 1), _T_PRED_MAX) if t_lim is None else np.asarray(t_lim, dtype=float)[:, None]
    for j, tr in enumerate(tracks):
        if tr.su is not None and not tr.moving:
            # 追跡を通った「まだ動いたことの無い」物: 速度の推定の不確かさ（p90 = 1.28σ）で前後・横に広げる。
            # 止まっている物は σ が小さく広がらない。見え始めたばかりの物は σ が大きく、どちらへ動くか分からない分だけ広がる
            grow_s = grow_d = np.minimum(1.28 * tr.su * tt, _GROW_UNKNOWN_MAX)
        else:
            grow_s = np.minimum(cfg.u_sigma * tt, cfg.u_grow_max) if tr.moving else 0.0
            grow_d = np.minimum(cfg.w_sigma * tt, cfg.w_grow_max) if tr.moving else 0.0
        m_lat = _margin(cfg, tr.moving)
        if tr.s_lo - cfg.margin_long < cfg.front and tr.s_hi + cfg.margin_long > -cfg.rear:
            # ★ もう並んでいる相手。横の余裕を今の間隔まで縮める（今より近づかなければよい）。
            #   縮めないと、余裕の内側に並んだ時点で全候補が初手から重なり「通れない」になる
            gap0 = max(tr.d_lo - (d_start + cfg.half_width), (d_start - cfg.half_width) - tr.d_hi)
            if gap0 >= 0.0:
                m_lat = min(m_lat, max(gap0 - 0.01, 0.0))
                grow_d = np.minimum(grow_d, max(gap0 - 0.01 - m_lat, 0.0)) if tr.moving else 0.0
        mo = motion[j] if motion is not None else None
        # 予測で進めた相手は、予測の長さの内側（`t_lim` まで）で時刻どおりに置く
        tm = np.minimum(t, lim) if mo is not None else tt
        adv = np.interp(tm, mo[0], mo[1]) if mo is not None else tr.u * tt
        # 覚えた相手の線から出した横のずれ（`line_shift`）。無ければ `w·t`
        lat = np.interp(tm, mo[0], mo[2]) if mo is not None and len(mo) > 2 else 0.0
        g_rel = grow_rel[j] if grow_rel is not None else None
        if tr.moving and g_rel is not None:
            # 広がりは横へ出始める時刻（`t_lim` − `_T_PRED_MAX`）から数える。それまでは付いていくだけで、出る前に
            # その時の観測で確かめ直す
            adv0 = np.interp(np.maximum(lim - _T_PRED_MAX, 0.0), mo[0], mo[1]) if mo is not None else 0.0
            grow_lo, grow_hi = 0.0, g_rel * np.maximum(adv - adv0, 0.0)
            ms = slow[j] if slow is not None else None
            if ms is not None:
                # 後ろ側は、場所ごとに測った遅い側（p10）の予測まで広げる（相手が予測より遅くても当たらない候補だけ使う。
                # 遅かったときに取りやめると、もう減速が間に合わない所で取りやめになった）。広がりは横へ出始める時刻から
                a_s = np.interp(tm, ms[0], ms[1])
                a_s0 = np.interp(np.maximum(lim - _T_PRED_MAX, 0.0), ms[0], ms[1])
                grow_lo = np.maximum((adv - adv0) - (a_s - a_s0), 0.0)
        else:
            grow_lo = grow_hi = grow_s
        s_lo = tr.s_lo + adv - grow_lo - cfg.margin_long
        s_hi = tr.s_hi + adv + grow_hi + cfg.margin_long + cfg.cutin_headway * max(tr.u, 0.0)
        d_lo = tr.d_lo + tr.w * tt + lat - grow_d - m_lat
        d_hi = tr.d_hi + tr.w * tt + lat + grow_d + m_lat
        s_ov = (car_hi > s_lo) & (car_lo < s_hi) & start_ok
        if tr.moving:
            # ★ 動く相手は予測の上限（`_T_PRED_MAX`）までだけ確かめる（TUM の IAC の planner と同じく、当たり判定は
            #   予測が信頼できる範囲に限る）。上限で相手を止めて見ると、5 秒より後に着く点で付いていく候補が止まった
            #   相手に当たったことになり、付いていく候補が全部「重なる」になった（相手の後ろ 1〜1.5m で "blocked"）。
            #   抜く候補が上限より後に戻るなら確かめきれないので、`eval_cand` が使わない
            s_ov &= t <= lim
        lat = np.maximum(d_lo - (d_up + swing), (d_dn - swing) - d_hi)
        gap = np.minimum(gap, np.where(s_ov, lat, np.inf).min(axis=1))
        over = s_ov & (lat < 0.0)
        h = over.any(axis=1)
        first = np.where(h & (first < 0), j, first)
        hit |= h
    return hit, first, gap


def _follow_times(t: np.ndarray, x: np.ndarray, ds: np.ndarray, rows: np.ndarray, follow,
                  a_brake: float, upto: np.ndarray | None = None,
                  a_accel: float = math.inf, lon: _Long | None = None,
                  kap: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """`rows` の候補の各点の時刻を、前を走る相手の後ろに付いていく時刻まで遅らせる。

    `upto`（`rows` の候補ごと）より先の点は付いていかない（横へ出始めた後。`Cand.S1`）。付いていった後に
    元の速さへ戻るところは加速度 `a_accel` で上げる。`lon`（と `rows` の候補の曲率 `kap`）があれば、加減速は点ごとの
    横の使い具合で決まる値（`_Long`。速度プロファイルと同じ）。

    `follow = (s_lo, t_arr, s_arr, keep, 前端)`: 相手の後ろ面（今は車から `s_lo`）は時刻 t に `s_lo + s_arr(t)`。
    車の前端が相手の後ろ面から `keep` 以上離れているように、点 x に着く時刻を `t_opp(x + 前端 + keep − s_lo)`
    以降にする（`t[k] = max(t[k−1] + 元の間隔, それ)`）。戻り値は `(時刻, 使えるか, 確かめる点)`:
    - 速さは今の速さから減速度の上限でしか落とせない（間に合わなければ相手に近づく時刻になり、重なれば当たり判定が落とす）
    - 今すでに車間の内側に居る候補は使えない（呼び出し側が車間を今の車間までに抑える）
    - 相手の予測（`t_arr` の終わり＝`_T_PRED_MAX`）より後にしか着けない点は確かめない（当たり判定は
      予測の終わりで相手を止めて見るので、付いていく候補が止まった相手に当たったことになる）
    """
    s_lo, t_arr, s_arr, keep, front = follow[:5]
    keep_now = follow[5] if len(follow) > 5 else keep
    t = t.copy()
    ok = np.ones(len(t), dtype=bool)
    check = np.ones(t.shape, dtype=bool)
    if not rows.any():
        return t, ok, check
    need = x + front + keep - s_lo
    beyond = need > s_arr[-1]
    t_req = np.where(need <= 0.0, 0.0, np.interp(np.minimum(need, s_arr[-1]), s_arr, t_arr))
    tr_ = t[rows]
    dt0 = np.diff(tr_, axis=1)
    # 道のり: `rows` の候補ごと (R, K-1)（`_path_ds`）か経路の弧長 (K-1,)
    ds = ds[rows] if np.ndim(ds) == 2 else np.broadcast_to(ds, (len(tr_), len(x) - 1))
    out = np.empty_like(tr_)
    up = np.full(len(tr_), np.inf) if upto is None else np.asarray(upto, dtype=float)
    req_m = np.where((x[None, :] <= up[:, None]) & ~beyond[None, :], t_req[None, :], 0.0)
    # 最初の点は今ここ（時刻 0）。今すでに車間の内側なら、その先の点へ遅く着く（＝減速度の上限で下がる）計画になる。
    # ★起点の時刻を遅らせると、その先の時刻は遅れた起点からの相対になり、速さの列は今の速さのまま減速を表さず、
    #   車間 0.3m の内側でも減速の指令が出ずに追突した
    out[:, 0] = tr_[:, 0]
    for k in range(1, tr_.shape[1]):
        out[:, k] = np.maximum(out[:, k - 1] + dt0[:, k - 1], req_m[:, k])
    # ★遅らせた時刻は速度が段で落ちる（付いていく制約が効き始める点で瞬時に減速する）。その段を減速度の
    #   上限で後ろから均し（`v[k]² ≤ v[k+1]² + 2a·ds`）、今の速さからそれだけ減速できない候補は使えない。
    #   均さずに最初の区間だけ確かめると、横に並んだ所から「相手の後ろへ戻る」計画が瞬時に下がれる前提に
    #   なり、戻る途中で相手に当てた（sim.bench --mover、1 本に 4〜7 回）
    v_seg = ds / np.maximum(np.diff(out, axis=1), 1e-6)
    v_pt = np.empty_like(out)
    v_pt[:, 0] = v_seg[:, 0]
    v_pt[:, -1] = v_seg[:, -1]
    v_pt[:, 1:-1] = np.minimum(v_seg[:, :-1], v_seg[:, 1:])
    for k in range(v_pt.shape[1] - 2, -1, -1):
        a = lon.dec(v_pt[:, k + 1], kap[:, k + 1], k + 1) if lon is not None else a_brake
        v_pt[:, k] = np.minimum(v_pt[:, k], np.sqrt(v_pt[:, k + 1] ** 2 + 2.0 * a * ds[:, k]))
    v_first = float(np.max(ds[:, 0] / np.maximum(dt0[:, 0], 1e-6))) if dt0.shape[1] else 0.0
    v_now = np.minimum(ds[:, 0] / np.maximum(dt0[:, 0], 1e-6), v_first)
    # 今の速さからは減速度の上限でしか落とせない（物理的に走れる速さで時刻を出し、相手に重なるかは当たり判定が決める）。
    # ★以前は「狙いの車間を守れるだけ減速できるか」で候補を捨てていた。車間 0.3m で付いていくと余裕が無く、相手が
    #   コーナーの手前で減速するたびに付いていく候補も区間の入口まで付いていく候補も全部落ち、一度も仕掛けなかった
    v_pt[:, 0] = v_now
    for k in range(v_pt.shape[1] - 1):
        a = lon.dec(v_pt[:, k], kap[:, k], k) if lon is not None else a_brake
        v_pt[:, k + 1] = np.maximum(v_pt[:, k + 1], np.sqrt(np.maximum(v_pt[:, k] ** 2 - 2.0 * a * ds[:, k], 0.0)))
    # 付いていった後に元の速さへ戻るところは加速度の上限で上げる（瞬時に戻る前提だと、遅らせて横へ出る候補が
    # 追い付く時刻を早く見積もる）
    if lon is not None or math.isfinite(a_accel):
        for k in range(v_pt.shape[1] - 1):
            a = lon.acc(v_pt[:, k], kap[:, k], k) if lon is not None else a_accel
            v_pt[:, k + 1] = np.minimum(v_pt[:, k + 1], np.sqrt(v_pt[:, k] ** 2 + 2.0 * a * ds[:, k]))
    dt_new = 2.0 * ds / np.maximum(v_pt[:, :-1] + v_pt[:, 1:], 1e-6)
    out2 = np.concatenate([out[:, :1], out[:, :1] + np.cumsum(dt_new, axis=1)], axis=1)
    t[rows] = out2 if (lon is not None or math.isfinite(a_accel)) else np.maximum(out, out2)
    # 今すでに車間の内側なら、車間を守る候補としては使えない。★「今」は付いていく制御が保つ車間（上乗せ前の
    #   `keep_now`。`slam2d_raceline._follow_speed` と同じ）で見る。上乗せ（予測の不確かさ × 遅れ・近づく速さ × 指令の
    #   遅れ）込みの車間で見ると、制御どおりに付いていくほど常に内側に居ることになり、「区間の入口まで付いていって
    #   から抜く」候補が全部捨てられていた（理想化した検証で、続ける計画が落ちる理由のいちばん）。上乗せ分は先の点の
    #   時刻（少し下がる計画）で扱う。観測の揺れ（後ろ面の位置 ±数 cm）の分は許す
    need_now = x[0] + front + keep_now - s_lo
    ok[rows] = need_now <= _INSIDE_TOL
    check[np.ix_(rows, beyond)] = False
    return t, ok, check


def eval_cand(cands: np.ndarray, x, kr, v_cap, a_lat, ds, room_l, room_r, v_now,
              tracks, cfg: OvertakeConfig, d_start: float, motion: list | None = None,
              follow=None, grow_rel: list | None = None, lon: _Long | None = None, slow: list | None = None,
              keep_last: bool = False):
    """候補 (C, 9) をまとめて評価する。`keep_last` なら最後の候補（乗っている計画を続けるもの）の道幅・曲率の確かめに
    `_KEEP_ROOM_TOL`・`_KEEP_KAPPA_TOL` だけ許す（一度確かめて乗った形を、格子のずれによる標本化の差で落とさない）。戻り値 `(使えるか, 時間のコスト, 当たる最初の障害物, d, 落ちた理由, 横の間隔,
    各点の時刻)`（時刻は計算しなかった候補が nan）。

    `follow` があれば、経路へ戻る候補（`dT = 0`）は前の相手に付いていく速さで走る（`_follow_times`）。
    """
    d, dd, ddd = _profile(x, *cands[:, 1:].T)
    one = 1.0 - kr[None, :] * d
    kap_c = kr[None, :] / np.maximum(one, 0.2) + ddd
    # 道幅: 出発点が `room` の外に居るなら、最初の 1m は出発点の横ずれまで許す（戻っていく途中）
    fade = np.clip(1.0 - x / 1.0, 0.0, 1.0)
    lim_l = np.maximum(room_l, max(d_start, 0.0) * fade + 0.005)
    lim_r = np.maximum(room_r, max(-d_start, 0.0) * fade + 0.005)
    tol = np.zeros((len(cands), 1))
    if keep_last:
        tol[-1] = _KEEP_ROOM_TOL
    in_room = ((d <= lim_l[None, :] + tol) & (-d <= lim_r[None, :] + tol)).all(axis=1)
    k_tol = 1.0 + tol / max(_KEEP_ROOM_TOL, 1e-9) * _KEEP_KAPPA_TOL
    # Frenet の曲率は近似なので（直交座標で測ると数 % 大きい）、限界の 95% までにする。元の線が
    # すでに限界を超えている所（道幅が足りず半径を開けられないヘアピン）は、元の曲率に限界の
    # `_KAPPA_SLACK` まで上乗せしてよい（`avoid.plan_offset` と同じ。+0.001 しか許さないと、
    # そこでは少しでも横へずれた候補がすべて落ち、他車の後ろを 1 周近く付いて回った）
    k_lim = np.maximum(0.95 * cfg.kappa_max, np.abs(kr) + _KAPPA_SLACK * cfg.kappa_max)
    k_ok = ((np.abs(kap_c) <= np.where(np.abs(kr) > 0.95 * cfg.kappa_max, k_lim, 0.95 * cfg.kappa_max)[None, :] * k_tol)
            .all(axis=1) & (one > 0.2).all(axis=1))
    # 格子の点の間（中点）でも曲率を確かめる。★点だけで見ると、毎周期の格子のずれで同じ候補の曲率が限界の前後を
    #   行き来し、乗っている計画が落ちて別の候補へ入れ替わった
    g0 = np.nonzero(k_ok & in_room)[0]
    if g0.size:
        xm = 0.5 * (x[:-1] + x[1:])
        km = 0.5 * (kr[:-1] + kr[1:])
        dm_, _, dddm = _profile(xm, *cands[g0, 1:].T)
        kap_m = km[None, :] / np.maximum(1.0 - km[None, :] * dm_, 0.2) + dddm
        lim_m = np.maximum(0.95 * cfg.kappa_max, np.abs(km) + _KAPPA_SLACK * cfg.kappa_max)
        k_ok[g0] = (np.abs(kap_m) <= np.where(np.abs(km) > 0.95 * cfg.kappa_max, lim_m, 0.95 * cfg.kappa_max)[None, :]
                    * k_tol[g0]).all(axis=1)
    # ★ 速度・時刻・当たり判定は、道幅と曲率を満たす候補だけで計算する（結果は全候補で計算したときと同じ）。
    #   相手 2 台で「通れない」と出る周期は 1500 本中 1200 本がここで落ち、残りの計算が無駄だった
    C = len(cands)
    geo = in_room & k_ok
    # 経路へ戻る候補（数本）は形によらず含める（`_follow_times` が戻る候補全体の時刻を見るため、外すと結果が変わる）
    sub = np.nonzero(geo | (cands[:, 4] == 0.0))[0]
    ok = np.zeros(C, dtype=bool)
    cost = np.full(C, np.inf)
    first = np.full(C, -1)
    gap = np.full(C, -np.inf)
    v_ok = np.zeros(C, dtype=bool)
    hit = np.zeros(C, dtype=bool)
    tm = np.full((C, len(x)), np.nan)
    if sub.size:
        dsc = _path_ds(ds, kr, d[sub], dd[sub])
        v, vk = _speeds(kap_c[sub], v_cap, a_lat, dsc, v_now, cfg, lon)
        dt = 2.0 * dsc / (v[:, :-1] + v[:, 1:])
        t = np.concatenate([np.zeros((sub.size, 1)), np.cumsum(dt, axis=1)], axis=1)
        check = np.ones((1, len(x)), dtype=bool)
        if follow is not None:
            # 線のまま戻る候補は見る先の終わりまで、区間の入口まで付いていく候補は横へ出始める点（S1）まで、前の相手に付いていく
            late = cands[sub, 8] > 0.0 if cands.shape[1] > 8 else np.zeros(sub.size, dtype=bool)
            rows_f = (cands[sub, 4] == 0.0) | late
            upto = np.where(late[rows_f], cands[sub, 8][rows_f] if cands.shape[1] > 8 else np.inf, np.inf)
            t, f_ok, check = _follow_times(t, x, dsc, rows_f, follow, cfg.a_brake, upto, cfg.a_accel,
                                           lon, kap_c[sub][rows_f] if lon is not None else None)
            vk = vk & f_ok
        ds_ = d[sub]
        # 確かめる範囲: 横へ出始める時刻（`S1`。今すぐ出る候補は 0）から `_T_PRED_MAX`
        s1 = cands[sub, 8] if cands.shape[1] > 8 else np.zeros(sub.size)
        t_s1 = np.array([np.interp(a, x, t[i]) if a > 0.0 else 0.0 for i, a in enumerate(s1)])
        t_lim = t_s1 + _T_PRED_MAX
        h, f1, g1 = _collide(x, ds_, dd[sub], t, tracks, cfg, check, d_start, motion, grow_rel, t_lim, slow)
        g = geo[sub]
        # 動く相手を横から抜く候補は、予測の上限（`_T_PRED_MAX`）の内側で抜き切りが確かめられるものだけ使う:
        # 確かめる時点（戻り終える位置か見る先の終わりの早い方）がその内側で、戻り終えるのが見る先の外なら、
        # その時点で相手（前側の不確かさ込み）より前に出ていること
        mv = [j for j, tr in enumerate(tracks) if tr.moving]
        if mv:
            side = cands[sub, 4] != 0.0
            x_end = np.where(np.isfinite(cands[sub, 6]), cands[sub, 6] + cands[sub, 7], np.inf)
            x_c = np.minimum(x_end, x[-1])
            t_c = np.array([np.interp(xc, x, t[i]) for i, xc in enumerate(x_c)])
            unver = side & (t_c > t_lim)
            # 見る先の外で戻る候補の「前に出ているか」は、すぐ横へ出る候補（出始めが `_COMMIT_M` 以内）だけで確かめる。
            # 出始めがまだ先の候補は今は付いていくだけで、出る直前の計画がその時の見る先で確かめ直す（★確かめないと、
            # 区間の入口まで付いていく候補は抜き切りが見る先 12m の外になり、全部落ちて一度も仕掛けなかった）
            beyond_h = side & (x_end > x[-1]) & (s1 <= _COMMIT_M)
            if beyond_h.any():
                for j in mv:
                    tr, mo = tracks[j], (motion[j] if motion is not None else None)
                    adv = np.interp(np.minimum(t_c, t_lim), mo[0], mo[1]) if mo is not None else tr.u * t_c
                    g_rel = grow_rel[j] if grow_rel is not None and grow_rel[j] is not None else 0.0
                    adv0 = np.interp(t_s1, mo[0], mo[1]) if mo is not None else tr.u * t_s1
                    opp_front = tr.s_hi + adv + g_rel * np.maximum(adv - adv0, 0.0) + cfg.margin_long
                    unver |= beyond_h & (x_c - cfg.rear < opp_front) & (tr.s_hi + cfg.margin_long > -cfg.rear)
            vk = vk & ~unver
        v_ok[sub], hit[sub], first[sub], gap[sub] = vk, h, f1, np.where(g, g1, -np.inf)
        ok[sub] = vk & ~h & g
        tm[sub] = t
        # 順位付けの時間: 線より長い道のりは損として数えるが、短い道のり（カーブの内側）は得にしない。★本当の道のりで
        #   比べると、速度の上限（`v_max`・見通し）で抑えた所では内側へずれる候補が線より速くなり、障害物が見えるたびに
        #   線を外れて内側を走った。線はその速度の上限の下で引いた最善の線として扱う（時刻・当たり判定は本当の道のり）
        w = np.maximum(1.0, ds[None, :] / np.maximum(dsc, 1e-9))
        cost[sub] = ((np.diff(t, axis=1) * w).sum(axis=1) + _END_COST * np.abs(ds_[:, -1])
                     + _LAT_COST * np.abs(ds_).max(axis=1))
    fails = {"道幅": int((~in_room).sum()), "曲率": int((~k_ok).sum()), "減速": int((geo & ~v_ok).sum()),
             "重なる": int(hit.sum())}
    return ok, cost, first, d, fails, gap, tm


def opp_motion(tr: Track, s_abs: float, s_cum: np.ndarray, v_line: np.ndarray, L: float,
               t_max: float = _T_PRED_MAX, v_learn: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray] | None:
    """同じ向きに走る相手が `t_max` 秒の間に進む距離 `(t_arr, s_arr)`（`s_arr[0] = 0`）。

    相手は経路の速度プロファイル `v_line` に、今の速さとの比 `k = u / v_line(今の場所)` を掛けた速さで
    走るとする。★一定の速さ `u` で進むとすると、コーナーの手前で自分の計画は減速するのに相手は
    減速しないことになり、「経路のままで当たらない」と出て横へ出た計画を取り下げ、相手が減速して
    追いつくとまた横へ出る、を繰り返して抜けなかった（sim.bench --mover。相手 0.7 倍で 120 秒抜けない）。
    `s_abs` は相手（の後ろ面）の経路の始点からの弧長。逆向き・遅い相手は `None`（一定速度のまま）。
    """
    if not tr.moving or tr.u < MOVING_MIN_SPEED:
        return None
    n = len(s_cum)
    step = max(L / n, 1e-3)
    j0 = int(np.searchsorted(s_cum, s_abs % L, side="right") - 1) % n
    v_line = np.asarray(v_line, dtype=float)
    k = min(max(tr.u / max(float(v_line[j0]), 0.1), 0.2), 1.5)
    # 見る長さは `t_max` の間に最も速く走っても届く距離（足りないと補間の端で相手が止まって見える）
    v_top = k * float(v_line.max())
    if v_learn is not None and np.isfinite(v_learn).any():
        v_top = max(v_top, float(np.nanmax(v_learn)))
    span = int(math.ceil((v_top * t_max + 1.0) / step)) + 2
    idx = (j0 + np.arange(min(span, n))) % n
    v = v_line[idx]
    vv = k * v
    if v_learn is not None:
        # 覚えた場所の速さ（`line_profile`）があればそれを使う。覚えていない場所は比で伸ばす
        vl = v_learn[idx]
        vv = np.where(np.isfinite(vl), vl, vv)
    vv = np.maximum(vv, MOVING_MIN_SPEED)
    ds = np.full(len(idx) - 1, step)
    t_arr = np.concatenate([[0.0], np.cumsum(2.0 * ds / (vv[:-1] + vv[1:]))])
    s_arr = np.concatenate([[0.0], np.cumsum(ds)])
    return t_arr, s_arr


#: 相手の線を覚える区間の長さ [m]・使うのに要る観測の数・横のずれの予測の上限 [m]
_LINE_BIN = 0.2
_LINE_MIN_N = 2
#: 覚えた速さを使うのに要る観測の数。1 周で 1 区間に約 1 つ（10Hz・2m/s で 0.2m おき）しか入らないので 1。
#: EKF の速さは 1 つでも真値の ±数 %（追跡の `VelocityTracker`）
_SPEED_MIN_N = 1
#: 場所ごとの相手の速さに使う直近の観測の数（中央値。1 周で 1 区間に約 1 つなので、直近の約 3 周）
_SPEED_RECENT = 3
#: 場所ごとの相手の速さ・ばらつきを均す前後の区間の数（±0.4m）
_SPEED_SMOOTH = 2
#: 相手の軌跡から速さを差分で出すとき、前後の観測の時刻の間隔の上限 [s] と、結んでよい速さの上限 [m/s]（`line_profile`）
_DIFF_GAP_S = 0.15
_DIFF_VMAX = 6.0
_LINE_SHIFT_MAX = 0.6
#: 予測の当たり具合を測る（`pred_spread`）: 予測の長さ [s]（追い付いてから抜き切るまで 1〜3 秒）・後の観測の時刻の
#: 許容 [s]（点群 1 周 0.1s の半分）・使うのに要る組の数・下限（観測の位置の揺れ 2〜3cm ÷ 1〜3 秒で進む 2〜7m）
_CAL_TAU = (1.0, 2.0, 3.0)
_CAL_DT = 0.05
_CAL_MIN_N = 30
_CAL_FLOOR = 0.03
#: 予測の当たり具合を測る直近の時間 [s]（約 2 周。それより前の観測で覚えた速さで、この間の観測を予測する）
_CAL_RECENT_S = 20.0


def line_profile(path: RaceLine, order: np.ndarray, s_rel: np.ndarray, nrm: np.ndarray,
                 trail: np.ndarray) -> tuple | None:
    """これまでに見た動く相手の位置 `trail` を、窓（`order`・`s_rel`。`frenet_tracks` と同じ）の経路へ射影し、
    `_LINE_BIN` ごとの横ずれの平均 `(始まりの s, 平均, 数, 経路に沿った速さの平均, その数)` にする。
    `trail` が (N, 4)（位置と速度 vx, vy）でなければ速さは None。

    相手の速さも**場所の関数**として覚える（Predictive Spliner, Baumann ら 2024 と同じ考え）。今の速さと自分の線の
    速度の比で先へ伸ばす（`opp_motion`）と、相手の線・加減速の仕方が自分と違う所でずれる。記録で 2 秒先の
    経路に沿った位置の誤差 p90 が 1.2〜1.5m（比）対 0.8m（場所ごとの平均）だった

    ★相手の、自分の線からの横ずれは時間の一次式ではなく**場所の関数**（別の線を毎周なぞる）。直前の
    傾きで先へ伸ばすと「同じ横ずれのまま」より外れた（記録で 1s 先の誤差 p90 66cm 対 43cm）。前の周に
    その場所で見た横ずれの変化を使うと 39cm（自分の線と同じ線の相手では変わらない）。
    自分の線は周ごとに替わる（`slam2d_route`）ので、覚えるのは map 上の位置で、射影は毎回ここでする。
    """
    xy = path.xy[order]
    lo, hi = xy.min(axis=0) - 1.0, xy.max(axis=0) + 1.0
    m = (trail[:, 0] > lo[0]) & (trail[:, 0] < hi[0]) & (trail[:, 1] > lo[1]) & (trail[:, 1] < hi[1])
    pts = trail[m, :2]
    if len(pts) == 0:
        return None
    K = len(order)
    d2 = ((pts[:, None, :] - xy[None, :, :]) ** 2).sum(axis=2)
    k = np.argmin(d2, axis=1)
    tn = np.column_stack([nrm[order, 1], -nrm[order, 0]])
    rel = pts - xy[k]
    s = s_rel[k] + (rel * tn[k]).sum(axis=1)
    d = (rel * nrm[order[k]]).sum(axis=1)
    keep = (k > 0) & (k < K - 1) & (np.abs(d) < 1.0)
    if not keep.any():
        return None
    u = None
    if trail.shape[1] >= 4:
        u = trail[m, 2] * tn[k, 0] + trail[m, 3] * tn[k, 1]
    if trail.shape[1] >= 5:
        # 時刻があれば、経路に沿った位置の前後の差（中心差分）から速さを出す。★追跡（EKF）の速さは相手が減速している最中に
        #   遅れて高めに出て（コーナーの手前で真値 2.4 に対し 2.8〜3.2m/s）、その平均で覚えると減速区間で毎回同じ向きに偏り、
        #   付いていく計画が近づきすぎて追突した。覚えるのは過去の軌跡なので遅れの無い差分が使える。前後の観測が
        #   `_DIFF_GAP_S` 以内で、別の相手を結ばない（経路に沿った差が速さの上限で進める距離以内）ものだけ
        tt = trail[m, 4]
        sd_ = s
        ok_d = np.zeros(len(tt), dtype=bool)
        ud = np.full(len(tt), np.nan)
        if len(tt) >= 3:
            dt2 = tt[2:] - tt[:-2]
            dsd = sd_[2:] - sd_[:-2]
            good = (dt2 > 1e-3) & (dt2 <= 2.0 * _DIFF_GAP_S) & (np.abs(dsd) <= _DIFF_VMAX * dt2) & (dsd > 0.0)
            ud[1:-1] = np.where(good, dsd / np.maximum(dt2, 1e-6), np.nan)
            ok_d[1:-1] = good
        u = np.where(ok_d, ud, u if u is not None else np.nan)
    s0 = float(s_rel[0])
    nb = int(math.ceil((float(s_rel[-1]) - s0) / _LINE_BIN)) + 1
    b = np.clip(((s[keep] - s0) / _LINE_BIN).astype(int), 0, nb - 1)
    cnt = np.bincount(b, minlength=nb).astype(float)
    tot = np.bincount(b, weights=d[keep], minlength=nb)
    mean_u = cnt_u = sd_u = None
    if u is not None:
        uk = u[keep]
        mv = uk >= MOVING_MIN_SPEED
        bm, um = b[mv], uk[mv]
        if trail.shape[1] >= 5 and len(bm):
            # ★場所ごとに**直近の `_SPEED_RECENT` 回**の観測だけを使い、その中央値を覚えた速さにする。以前は 3 分ぶん全部の
            #   平均で、相手の線や速さが周ごとに変わると（シムの他車は自車の経路に乗り、自車の経路が周ごとに替わる）古い周の
            #   値で予測が外れた（1 周目は s=4〜10m で 0.7m/s、以降 1.2〜1.5m/s。予測の外れの p90 が標本外で進む距離の 52%）。
            #   中央値は、別の相手を結んだ差分などの外れ値にも強い
            tm_ = trail[m, 4][keep][mv]
            o = np.lexsort((-tm_, bm))                   # 区間ごとに新しい順
            bo = bm[o]
            st_ = np.r_[0, np.nonzero(np.diff(bo))[0] + 1]
            rank = np.arange(len(bo)) - np.repeat(st_, np.diff(np.r_[st_, len(bo)]))
            sel = rank < _SPEED_RECENT
            bm, um = bo[sel], um[o][sel]
            o2 = np.lexsort((um, bm))                    # 区間ごとに値の順
            bm, um = bm[o2], um[o2]
            cnt_u = np.bincount(bm, minlength=nb).astype(float)
            first = np.r_[0, np.cumsum(cnt_u)[:-1]].astype(int)
            c_ = cnt_u.astype(int)
            lo_i = np.clip(first + (c_ - 1) // 2, 0, max(len(um) - 1, 0))
            hi_i = np.clip(first + c_ // 2, 0, max(len(um) - 1, 0))
            mean_u = np.where(cnt_u > 0, 0.5 * (um[lo_i] + um[hi_i]), np.nan)
        else:
            cnt_u = np.bincount(bm, minlength=nb).astype(float)
            mean_u = np.where(cnt_u > 0, np.bincount(bm, weights=um, minlength=nb) / np.maximum(cnt_u, 1.0), np.nan)
        # 場所ごとのばらつき（同じ観測で 2 つ以上。無ければ nan）。付いていく計画は遅い側（`_slow_side`）を使う
        sum_u = np.bincount(bm, weights=um, minlength=nb)
        sum_u2 = np.bincount(bm, weights=um ** 2, minlength=nb)
        mu_ = sum_u / np.maximum(cnt_u, 1.0)
        var = np.where(cnt_u >= 2, np.maximum(sum_u2 / np.maximum(cnt_u, 1.0) - mu_ ** 2, 0.0)
                       * cnt_u / np.maximum(cnt_u - 1.0, 1.0), np.nan)
        # ★経路に沿って ±`_SPEED_SMOOTH` 区間で均す（覚えていない区間へは広げない）。区間（0.2m）ごとの値は直近の数回の
        #   観測だけで決まり、区間ごとに ±0.2m/s ほどばらつく。その値で付いていく制御が相手の速さを読むと周期ごとに
        #   0.16m/s（中央）変わり、自車の速さも揺れた。左右対称の平均なので、減速区間の一次の変化は保たれる
        mean_u = _smooth_bins(mean_u, _SPEED_SMOOTH)
        sd_u = np.sqrt(_smooth_bins(var, _SPEED_SMOOTH))
    # 射影した観測（経路に沿った位置・時刻・その時の経路に沿った速さ）。予測の当たり具合を測るのに使う（`pred_spread`）
    samples = None
    if trail.shape[1] >= 5:
        samples = (s[keep], trail[m, 4][keep], u[keep] if u is not None else np.full(int(keep.sum()), np.nan))
    return s0, np.where(cnt > 0, tot / np.maximum(cnt, 1.0), 0.0), cnt, mean_u, cnt_u, sd_u, samples


def pred_spread(prof: tuple) -> tuple[float, int] | None:
    """覚えた速さ（`line_profile`）で相手の位置を予測したときの、前側の外れの p90 ÷ 予測で進む距離と、測った組の数。

    これまでに見た相手の観測の組（時刻が `_CAL_TAU` 秒離れたもの）で、前の観測から覚えた速さで進めた位置と、後の観測
    の位置を比べる。★以前は過去の 1 回のベンチの記録から決めた一定の割合（`_GROW_LEARNED` = 0.2）で前側を広げて
    いた。予測どおりに走る相手でも 3 秒先で約 1m 前へ広がり、抜き切って前へ戻る所が確かめられず、知覚と制御が完全な
    理想化した検証でも 0.65 倍より速い相手を一度も抜けなかった（広がりを 0.02 にすると 0.7 倍まで全部抜けた）。
    予測がよく当たる相手には小さく、揺れる相手には大きくなる（測れなければ None で、一定の割合のまま）。
    覚えた速さはその組自身も含めて平均したもの（同じ所を何周も見ていれば影響は小さい）"""
    if prof is None or len(prof) < 7 or prof[6] is None or prof[3] is None:
        return None
    s_pts, t_pts, u_pts = prof[6]
    if len(s_pts) < 2 * _CAL_MIN_N:
        return None
    s0, mean_u = prof[0], prof[3]
    # 区間（`_LINE_BIN`）ごとの通過時間の累積 T(s)。覚えていない区間を挟む組は使わない
    v = np.where(np.isfinite(mean_u) & (mean_u >= MOVING_MIN_SPEED), mean_u, np.nan)
    dtb = _LINE_BIN / v
    Tc = np.concatenate([[0.0], np.cumsum(np.nan_to_num(dtb, nan=0.0))])
    gap = np.concatenate([[0], np.cumsum(~np.isfinite(dtb))])        # 覚えていない区間の数の累積
    edges = s0 + _LINE_BIN * np.arange(len(Tc))
    o = np.argsort(t_pts)
    s_pts, t_pts, u_pts = s_pts[o], t_pts[o], u_pts[o]
    rel: list[np.ndarray] = []
    n = 0
    for tau in _CAL_TAU:
        jb = np.searchsorted(t_pts, t_pts + tau)
        # 後の観測の候補: 時刻がほぼ τ 後のもの（同時に複数の相手がいれば、等速で進めた位置に近い方）
        best = np.full(len(t_pts), -1)
        err_b = np.full(len(t_pts), np.inf)
        for dj in (-1, 0, 1, 2):
            jj = np.clip(jb + dj, 0, len(t_pts) - 1)
            okt = np.abs(t_pts[jj] - (t_pts + tau)) <= _CAL_DT
            guess = s_pts + np.nan_to_num(u_pts, nan=1.0) * tau
            e = np.abs(s_pts[jj] - guess)
            take = okt & (e < err_b)
            best = np.where(take, jj, best)
            err_b = np.where(take, e, err_b)
        a = np.nonzero(best >= 0)[0]
        if not len(a):
            continue
        b = best[a]
        sa, sb = s_pts[a], s_pts[b]
        adv_obs = sb - sa
        ia = np.clip(((sa - s0) / _LINE_BIN), 0, len(Tc) - 1)
        Ta = np.interp(ia, np.arange(len(Tc)), Tc)
        s_pred = np.interp(Ta + tau, Tc, edges)
        ip = np.clip(((s_pred - s0) / _LINE_BIN), 0, len(Tc) - 1)
        good = ((adv_obs > 0.0) & (adv_obs < _DIFF_VMAX * tau) & (s_pred > sa + 0.1) & (s_pred < edges[-1] - 1e-6)
                & (np.interp(ip, np.arange(len(gap)), gap) - np.interp(ia, np.arange(len(gap)), gap) < 0.5))
        if not good.any():
            continue
        rel.append((sb[good] - s_pred[good]) / (s_pred[good] - sa[good]))
        n += int(good.sum())
    if n < _CAL_MIN_N:
        return None
    r = np.concatenate(rel)
    return float(max(np.percentile(r, 90), _CAL_FLOOR)), n


def calibrate_spread(path: RaceLine, s_cum: np.ndarray, nrm: np.ndarray, trail: np.ndarray) -> tuple[float, int] | None:
    """これまでに見た相手の軌跡 `trail` (N, 5) で、覚えた速さによる予測の前側の外れ（`pred_spread`）を 1 周ぶん全体で
    標本外に測る: 直近 `_CAL_RECENT_S` 秒の観測を、それより前の観測で覚えた速さで予測する（予測に使う速さは直近の
    観測まで含むので、実際の予測はこれより当たる側＝安全側）。測れなければ None。重いので毎周期は呼ばない"""
    if trail is None or len(trail) < 2 * _CAL_MIN_N or trail.shape[1] < 5:
        return None
    order = np.arange(len(path))
    t_cut = float(trail[:, 4].max()) - _CAL_RECENT_S
    old, new = trail[trail[:, 4] < t_cut], trail[trail[:, 4] >= t_cut]
    if len(old) < _CAL_MIN_N or len(new) < _CAL_MIN_N:
        return None
    p_old = line_profile(path, order, s_cum, nrm, old)
    p_new = line_profile(path, order, s_cum, nrm, new)
    if p_old is None or p_new is None or p_old[3] is None:
        return None
    return pred_spread(p_old[:6] + (p_new[6],))


def _smooth_bins(v: np.ndarray, w: int) -> np.ndarray:
    """区間の値の、前後 `w` 区間の平均（nan は数えない）。もとが nan の区間は nan のまま。"""
    if v is None or w <= 0:
        return v
    m = np.isfinite(v)
    k = np.ones(2 * w + 1)
    num = np.convolve(np.where(m, v, 0.0), k, mode="same")
    den = np.convolve(m.astype(float), k, mode="same")
    return np.where(m & (den > 0), num / np.maximum(den, 1e-9), np.nan)


def line_shift(prof, s_now: float, s_adv: np.ndarray) -> np.ndarray:
    """相手（今 `s_now`）が `s_adv` だけ進んだときの横ずれの変化（覚えた線の差）。覚えていない場所は
    手前で分かっている値のまま（今の場所を覚えていなければすべて 0）。"""
    s0, mean, cnt = prof[:3]
    nb = len(mean)
    i0 = int((s_now - s0) / _LINE_BIN)
    out = np.zeros(len(s_adv))
    if not (0 <= i0 < nb) or cnt[i0] < _LINE_MIN_N:
        return out
    idx = ((s_now + s_adv - s0) / _LINE_BIN).astype(int)
    last = 0.0
    for q, i in enumerate(idx):
        if 0 <= i < nb and cnt[i] >= _LINE_MIN_N:
            last = float(mean[i] - mean[i0])
        out[q] = last
    return np.clip(out, -_LINE_SHIFT_MAX, _LINE_SHIFT_MAX)


def _lengths(delta: float, v: float, kappa_budget: float, kappa_max: float) -> list[float]:
    """横に `delta` ずれる長さの候補 [m]（短い順）。

    最短は**車の限界の曲率**で決まる長さ（カーブの外へ出る向きなら元の曲率が減るので、
    `kappa_budget`＝限界から元の曲率を引いた残り、より短く入れる）。その上に `kappa_budget` で
    決まる長さとその倍、速さに合わせた長さ。使えるかは `eval_cand` が曲率で確かめる。
    """
    dl = max(abs(delta), 1e-3)
    l_phys = max(0.3, math.sqrt(5.77 * dl / max(0.95 * kappa_max, 1e-3)))
    l_bud = max(0.3, math.sqrt(5.77 * dl / max(kappa_budget, 1e-3)))
    ls = {round(l_phys, 2), round(l_bud, 2), round(1.5 * l_bud, 2), round(2.5 * l_bud, 2),
          round(max(l_bud, 0.8 * max(v, 0.5)), 2)}
    return sorted(ls)


def _catch_point(x, kr, v_cap, al, ds, v0, cfg: OvertakeConfig, tr: Track, mo,
                 lon: _Long | None = None) -> float | None:
    """線のまま全速で走ったとき、車の前端が前の相手の後ろ面（＋前後の余裕）に届く点 [m]（出発点から）。
    見る先の中で届かなければ None。相手は `mo`（`opp_motion`）か一定の速さ `tr.u` で進む。"""
    v, _ = _speeds(kr[None, :], v_cap, al, ds, v0, cfg, lon)
    t = np.concatenate([[0.0], np.cumsum(2.0 * ds / (v[0, :-1] + v[0, 1:]))])
    tt = np.minimum(t, _T_PRED_MAX)
    adv = np.interp(tt, mo[0], mo[1]) if mo is not None else tr.u * tt
    reach = x + cfg.front + cfg.margin_long >= tr.s_lo + adv
    if not reach.any() or t[int(np.argmax(reach))] > _T_PRED_MAX:
        return None
    return float(x[int(np.argmax(reach))])


def plan(path: RaceLine, index: int, pose: tuple[float, float, float], v_now: float,
         v_base: np.ndarray, a_lat, room: tuple[np.ndarray, np.ndarray],
         obstacles: list[Obstacle], cfg: OvertakeConfig, *, prev: Cand | None = None,
         s_cum: np.ndarray | None = None, trail: np.ndarray | None = None,
         a_grip=None) -> OvertakeResult:
    """今の位置から先の一時経路を引く（閉じた経路だけ）。

    :param index: 車の今いる経路の添字
    :param v_base: 元の経路の速度（点ごと。見通しの上限などを掛けたもの）
    :param a_lat: 横加速度の上限（スカラーか点ごと）
    :param room: 経路の各点から左・右へずらせる量 [m]（壁と車体の余裕を引いたもの）
    :param prev: 前の周期に選んだ候補（その上に居れば、その状態から出発する）
    :param s_cum: 経路の始点からの弧長（点ごと。省略すると作る）
    :param trail: これまでに見た動く相手の位置 (N, 2)（map フレーム）。相手の線を経路に沿って覚え、
        横のずれを予測する（`line_profile`・`line_shift`）
    :param a_grip: 横の限界（摩擦円の半径）[m/s²]。スカラーか点ごと。None なら `a_lat`。速度プロファイルに渡す値と同じ
        （前後に出せる加速度を横の使い具合で決める `_Long`）
    """
    n = len(path)
    if s_cum is None:
        seg = np.hypot(*np.diff(np.vstack([path.xy, path.xy[:1]]), axis=0).T)
        s_cum = np.concatenate([[0.0], np.cumsum(seg[:-1])])
    L = path.length
    v0 = max(v_now, 0.0)
    H = min(_H_MAX, max(_H_MIN, v0 * _H_T))
    if any(o.speed >= MOVING_MIN_SPEED for o in obstacles):
        H = max(H, _H_OT)
    nrm = normals(path.xy)
    along, d_act, d1_act = frenet_state(path, index, *pose)
    s_car = (float(s_cum[index]) + along) % L

    # 見る先の点（0.2m おき）と、障害物を Frenet に直すための少し後ろからの点
    step_n = max(1, int(round((_STEP if v0 < 1.5 else _STEP_FAST) / max(L / n, 1e-3))))
    back = int(round(1.0 / max(L / n, 1e-3)))
    span = int(math.ceil(H / max(L / n, 1e-3))) + 2
    # ★ 窓の弧長は始まり（車の少し後ろ）から積み上げて単調にする。±L/2 へ折り返すと、窓（後ろ 1m＋前 H）が
    #   半周を超える短いコース・高い速度で先の方が負（＝後ろ）になり、先の障害物を落とし、`line_profile` が
    #   例外で止まった（家のコース 1周 17m・3.4m/s 以上。実機 surge_20261120_115744 で 3 回、そのたびに急制動）。
    #   1 周を超える窓は同じ点を二度見るので 1 周で打ち切る
    full = (index - back + np.arange(min(back + span + 1, n))) % n
    s_head = ((s_cum[full[0]] - s_car + 0.5 * L) % L) - 0.5 * L
    s_full = s_head + np.concatenate([[0.0], np.cumsum(np.diff(s_cum[full]) % L)])
    tracks = frenet_tracks(path, full, s_full, nrm, obstacles, cfg)
    # ★ 車より後ろの物は数えない。後ろから速い車が来ても、こちらが止まって避けることはない
    #   （数えると予測で全候補が重なり、止まったところへ突っ込まれた。sim.bench --mover）
    tracks = [tr for tr in tracks if tr.s_hi > -cfg.rear and tr.s_lo < H + 1.0]

    # 出発点: 前の計画の上に居ればその状態、外れていれば今の姿勢
    start = (d_act, d1_act, 0.0)
    x_prev = None
    if prev is not None:
        x_prev = (s_car - prev.s0) % L
        if x_prev < 0.5 * L:
            dp, d1p, d2p = prev.at(x_prev)
            if abs(dp - d_act) < _ON_PLAN_M:
                start = (dp, d1p, d2p)
            else:
                x_prev = None
        else:
            x_prev = None
    if not tracks and abs(start[0]) < _ON_LINE_M:
        return OvertakeResult("clear", None, None, [], None, 0, "")
    if x_prev is None and abs(math.atan(d1_act)) >= PSI_MAX - 1e-9:
        # 経路に対して大きく横を向いている（線形の範囲外）。切った値から引いた経路は静かに誤るので作らない（レビュー L12）
        if not tracks:
            return OvertakeResult("clear", None, None, [], None, 0, "")
        return OvertakeResult("blocked", None, None, [], None, 0,
                              f"向きのずれが {math.degrees(PSI_MAX):.0f}° を超え、追い抜きの経路を作れない")

    sel = (index + np.arange(0, span, step_n)) % n
    x = ((s_cum[sel] - s_car) % L)
    # ★ x と sel は同じ印で絞る。最も近い点（`index`）が車の少し後ろだと、その点の x は約 1 周になり落ちる。以前は
    #   `sel[:len(x)]` と先頭から数で切っていたので、x と sel が 1 点（0.2〜0.3m）ずれ、曲率・道幅・速度の上限と、計画の
    #   速度を当てはめる経路の点が後ろへずれていた（付いていく速度が本来より 0.3m 後ろで効き、追突した）
    keep_x = x <= H + 1e-6
    x, sel = x[keep_x], sel[keep_x]
    if len(x) < 4:
        return OvertakeResult("clear", None, None, [], None, 0, "")
    ds = np.maximum(np.diff(x), 1e-3)
    kr = path.kappa[sel] if len(path.kappa) == n else curvature(path.xy, True)[sel]
    v_cap = np.asarray(v_base, dtype=float)[sel]
    al = np.broadcast_to(np.asarray(a_lat, dtype=float), (n,))[sel] if np.ndim(a_lat) else \
        np.full(len(sel), float(a_lat))
    # 道幅は格子の点の前後の区間（経路の細かい点）の最小で見る。★格子の点（0.2〜0.3m おき）だけで見ると、道幅が狭まる
    #   所では毎周期の格子のずれで同じ候補が通ったり落ちたりし、乗っている追い抜きの計画が道幅で落ちて別の候補へ
    #   入れ替わった（理想化した検証、相手 0.7 倍で横へ出ては戻るを繰り返した）
    fine = (sel[:, None] + np.arange(-step_n, step_n + 1)[None, :]) % n
    room_l, room_r = room[0][fine].min(axis=1), room[1][fine].min(axis=1)
    # 前後に出せる加速度のモデル（速度プロファイルと同じ。横の限界 `a_grip` は無ければ `a_lat`）
    if a_grip is None:
        grip = al
    elif np.ndim(a_grip):
        grip = np.asarray(a_grip, dtype=float)[sel]
    else:
        grip = np.full(len(sel), float(a_grip))
    lon = _Long(np.maximum(grip, al), cfg.a_accel, cfg.a_brake, cfg.env)

    # 相手の予測（候補を作る前に出す。横へ出始める位置を、追い付く地点から決める）
    v_line = np.asarray(v_base, dtype=float)
    prof = line_profile(path, full, s_full, nrm, trail) if trail is not None and len(trail) else None
    v_learn = v_slow = None
    if prof is not None and prof[3] is not None:
        b = np.clip(((s_full - prof[0]) / _LINE_BIN).astype(int), 0, len(prof[3]) - 1)
        good = prof[4][b] >= _SPEED_MIN_N
        if good.any():
            v_learn = np.full(n, np.nan)
            v_learn[full[good]] = prof[3][b[good]]
            # 遅い側（p10）: 平均 − 1.28σ。ばらつきがまだ分からない場所（観測 1 つ）は平均（不確かさは車間の
            # `latency_s` の項だけ。予測の全時間に全体の比を掛けると、前を速く走る車まで遅いかもしれないと扱って塞がれる）
            sd = prof[5][b[good]]
            mu = prof[3][b[good]]
            v_slow = np.full(n, np.nan)
            v_slow[full[good]] = np.where(np.isfinite(sd), mu - 1.28 * sd, mu)
    motion = [opp_motion(tr, s_car + tr.s_lo, s_cum, v_line, L, t_max=_T_PRED_LONG, v_learn=v_learn) for tr in tracks]
    slow = ([None if mo is None else opp_motion(tr, s_car + tr.s_lo, s_cum, v_line, L, t_max=_T_PRED_LONG, v_learn=v_slow)
             for tr, mo in zip(tracks, motion)] if v_slow is not None else None)

    def _learned_at(tr: Track) -> bool:
        if v_learn is None:
            return False
        j0 = int(np.searchsorted(s_cum, (s_car + tr.s_lo) % L, side="right") - 1) % n
        return bool(np.isfinite(v_learn[j0]))

    # 覚えた速さで予測する相手の前側の広がり: 予測の当たり具合を実測した値（測れなければ一定の割合）
    g_learned = cfg.grow_learned if cfg.grow_learned is not None else _GROW_LEARNED
    grow_rel = [None if mo is None else (g_learned if _learned_at(tr) else _GROW_RATIO)
                for tr, mo in zip(tracks, motion)]
    if prof is not None:
        motion = [mo if mo is None else (*mo, line_shift(prof, 0.5 * (tr.s_lo + tr.s_hi), mo[1]))
                  for tr, mo in zip(tracks, motion)]
    # 線の上を前に走る相手（いちばん近いもの）。経路へ戻る候補はこの相手に付いていく速さで走らせる。
    # ★戻る候補を最高速のまま評価すると、付いていく間は「経路のままで当たらない」(clear) か「全部当たる」
    #   (blocked) のどちらかになり、抜く・付いていくを所要時間で比べられなかった。並んだ後に抜き切れないと
    #   blocked → 車間を空けて付いていく速さに落ち、横に出たまま相手の速さで走って下がった（sim.bench --mover）
    follow = None
    ahead = [j for j, tr in enumerate(tracks)
             if motion[j] is not None and tr.s_lo > cfg.front
             and tr.d_lo - _margin(cfg, True) < cfg.half_width and tr.d_hi + _margin(cfg, True) > -cfg.half_width]
    if ahead:
        j = min(ahead, key=lambda q: tracks[q].s_lo)
        keep = cfg.follow_keep if cfg.follow_keep is not None else cfg.margin_long + cfg.u_grow_max + _FOLLOW_SLACK
        keep_base = keep
        g_lead = grow_rel[j] if grow_rel[j] is not None else _GROW_RATIO
        # 計画し直しが効くまでの相手の予測の不確かさと、減速の指令が実際の減速になるまでに詰まる距離
        # （近づく速さ × 応答の時間。相手と同じ速さで付いているときは 0）
        keep += g_lead * max(tracks[j].u, 0.0) * cfg.latency_s + max(v0 - tracks[j].u, 0.0) * cfg.cmd_lead_s
        # ★ 今の車間が狙いより短くても、狙いの車間は下げない（その候補は使えないが、物理的に走れる最善の減速の計画が
        #   速度指令になる。`line_speed`）。以前は「今の車間まで抑える」としていたが、誤差で近づくたびに狙いが縮み、
        #   毎周期少しずつ詰まって追突した（相手 0.75 倍で 1 本に 16 回）
        # ★ 付いていくのは、場所ごとに覚えた相手の速さの遅い側（p10）で進む相手の後ろ。付いていく車に危ないのは相手が
        #   予測より遅いとき（抜く車に危ないのは速いとき。`_collide` の前側の広がり）。平均で進む相手に付いていくと、
        #   ヘアピンで相手の見える面が後ろから横へ変わる所（速さが 1.8〜1.9m/s に出る。真値 1.1〜1.3）で近づきすぎて追突した
        mo_f = slow[j] if slow is not None and slow[j] is not None else motion[j]
        follow = (tracks[j].s_lo, mo_f[0], mo_f[1], keep, cfg.front, keep_base)
    j_lead = None if follow is None else min(ahead, key=lambda q: tracks[q].s_lo)
    lead_u = None
    if follow is not None and len(follow[1]) > 1 and follow[1][1] > follow[1][0]:
        lead_u = float((follow[2][1] - follow[2][0]) / (follow[1][1] - follow[1][0]))

    d0, d1, d2 = start
    k_budget = max(0.3 * cfg.kappa_max, cfg.kappa_max - float(np.abs(kr).max()))
    k_budget = min(k_budget, max(0.5, float(al.min()) / max(v0, 0.5) ** 2))
    rows: list[tuple] = []

    def add(dT: float, L1s, S2s, L3s) -> None:
        for L1 in L1s:
            for S2 in S2s:
                if S2 < L1:
                    continue
                for L3 in L3s:
                    rows.append((s_car, d0, d1, d2, dT, L1, S2, L3, 0.0))

    # 経路へ戻る（＝経路のまま）。並ぶ区間は無い
    for L1 in _lengths(d0, v0, k_budget, cfg.kappa_max):
        add(0.0, [L1], [L1], [1.0])
    targets: list[tuple[float, bool]] = []
    for tr in tracks:
        for side in (+1, -1):
            base = tr.d_hi if side > 0 else tr.d_lo
            for e in ((0.07, 0.14) if tr.moving else (0.0, 0.08)):
                targets.append((base + side * (cfg.half_width + _margin(cfg, tr.moving) + 0.01 + e), tr.moving))
    if prev is not None and x_prev is not None and abs(prev.dT) > 1e-3:
        targets.append((prev.dT, False))
    seen: list[float] = []
    for dT, moving in targets:
        if any(abs(dT - q) < 0.01 for q in seen):
            continue
        seen.append(dT)
        L1s = _lengths(dT - d0, v0, k_budget, cfg.kappa_max)[:5]
        L3s = _lengths(dT, v0, k_budget, cfg.kappa_max)[2:4]
        # 戻り始めは、いちばん手前の障害物の手前より前にしても意味が無い（避ける前に戻ってしまう）
        s_first = max(0.0, min(tr.s_lo for tr in tracks)) if tracks else 0.0
        for L1 in L1s:
            a = max(L1, s_first)
            S2s = [a + k * _S2_STEP for k in range(int(max(H - a, 0.0) / _S2_STEP) + 1)]
            # ★ 動く相手には、見る先の中で戻り終える（抜き切る）候補だけを使う。戻らない候補（S2 = ∞）は
            #   並んだ先で抜き切れるかを確かめずに横へ出ることになり、道幅が尽きて線へ戻る途中で大きく失速した
            #   （相手の 0.78 倍の速さの車の後ろで、目標 1.05m/s に対し相手 2.46m/s。sim.bench --mover）
            add(dT, [L1], S2s if moving else S2s + [math.inf], L3s)
    # 追い付く地点の手前で横へ出始める候補（Predictive Spliner と同じ考え: 抜くのは追い付いてから並ぶ区間だけで
    # よい）。★今すぐ横へ出る候補だけだと、相手の後ろ 1〜2m から追い付くまでの間も横に出たまま走ることになり、
    # その区間ぶん長く道幅が要った。toyota2 の片側が空く区間（2.5〜5.5m）では抜き切る前に道幅が尽き、
    # 相手の 0.6 倍の車でも時間の半分は後ろに詰まっていた（sim.bench --mover）
    x_c = None
    if j_lead is not None and abs(d0) < 0.05 and abs(d1) < 0.1:
        x_c = _catch_point(x, kr, v_cap, al, ds, v0, cfg, tracks[j_lead], motion[j_lead], lon)
    if j_lead is not None and abs(d0) < 0.05 and abs(d1) < 0.1:
        tr = tracks[j_lead]
        for side in (+1, -1):
            base = tr.d_hi if side > 0 else tr.d_lo
            room_s = room_l if side > 0 else room_r
            for e in (0.07, 0.14):
                dT = base + side * (cfg.half_width + _margin(cfg, True) + 0.01 + e)
                # 横へ出終わるべき位置: その側に dT の道幅がある区間の入口（★区間の手前まで付いていき、入口で
                # 横へ出て全速で抜く。抜き切れる区間でだけ仕掛ける）と、全速で追い付く地点
                fits = room_s >= abs(dT) - 0.005
                entries = [float(x[k]) for k in range(1, len(x)) if fits[k] and not fits[k - 1]]
                L1s = _lengths(dT, v0, k_budget, cfg.kappa_max)[:2]
                L3s = _lengths(dT, v0, k_budget, cfg.kappa_max)[2:4]
                for L1 in L1s:
                    # 入口の手前から出始める（道幅がなだらかに広がる所）〜入口で出始める（段になっている所）
                    starts = [e - f * L1 for e in entries for f in (1.0, 0.5, 0.0)]
                    if x_c is not None:
                        starts += [x_c - bk - L1 for bk in _S1_BACKS]
                    for S1 in starts:
                        if S1 < 0.3:
                            continue
                        a = S1 + L1
                        S2s = [a + k * _S2_STEP_LATE for k in range(int(max(H - a, 0.0) / _S2_STEP_LATE) + 1)]
                        for S2 in S2s:
                            for L3 in L3s:
                                rows.append((s_car, d0, 0.0, 0.0, dT, L1, S2, L3, S1))
    # 前の計画をそのまま続ける候補（同じ多項式になる）
    n_before = len(rows)
    if prev is not None and x_prev is not None:
        L1r = prev.S1 + prev.L1 - x_prev
        if prev.S1 - x_prev > 0.05:
            # まだ横へ出始めていない（線の上で相手の後ろ）
            rows.append((s_car, d0, 0.0, 0.0, prev.dT, prev.L1, prev.S2 - x_prev, prev.L3, prev.S1 - x_prev))
        elif L1r > 0.05:
            rows.append((s_car, d0, d1, d2, prev.dT, L1r, prev.S2 - x_prev, prev.L3, 0.0))
        elif prev.S2 - x_prev > 0.0:
            rows.append((s_car, d0, d1, d2, prev.dT, 0.05, prev.S2 - x_prev, prev.L3, 0.0))
        elif prev.S2 + prev.L3 - x_prev > 0.05:
            L1r = prev.S2 + prev.L3 - x_prev
            rows.append((s_car, d0, d1, d2, 0.0, L1r, L1r, 1.0, 0.0))
    cands = np.array(rows, dtype=float)

    ok, cost, first, dmat, fails, gap, tm = eval_cand(cands, x, kr, v_cap, al, ds, room_l, room_r, v0, tracks, cfg,
                                                      d0, motion, follow, grow_rel, lon, slow,
                                                      keep_last=len(rows) > n_before)
    line = cands[:, 4] == 0.0

    def _vel(i: int) -> np.ndarray | None:
        """候補 i の点ごとの速さ（時刻の差から。計算していなければ None）。"""
        t = tm[i]
        if not np.isfinite(t).all():
            return None
        vs = ds / np.maximum(np.diff(t), 1e-6)
        return np.concatenate([[vs[0]], vs])

    # 線のまま付いていく最善（"clear"・"blocked" の速度の計画）: 使える線の候補の中で、無ければ線の候補全体で所要時間最小
    li = np.nonzero(line)[0]
    fin = li[np.isfinite(cost[li])]
    i_line = (int(fin[np.argmin(np.where(ok[fin], cost[fin], cost[fin] + 1e3))]) if len(fin) else None)
    v_line_best = _vel(i_line) if i_line is not None else None

    def _done(r: OvertakeResult, speeds: list | None = None) -> OvertakeResult:
        return r._replace(sel=sel, speeds=speeds if speeds is not None else [v_line_best], line_speed=v_line_best,
                          lead_u=lead_u)
    # ★ 横へ出る計画に乗っている間は、それが当たり判定で有効なら乗り続ける（ForzaETH の Overtake 状態と同じ。
    #   無効になったときだけ選び直す）。所要時間の小さな差で毎周期選び直すと、右の 7m 先・左の 2m 先…と入れ替わり、
    #   どれにも乗り切れずに相手の後ろに留まった（sim.bench --mover）
    if len(rows) > n_before and abs(prev.dT) > 1e-3 and ok[-1]:
        c = Cand(*cands[-1])
        lead = tracks[j_lead] if j_lead is not None else None
        return _done(OvertakeResult("avoid", None, c, [c], lead, len(cands), "", True), [_vel(len(cands) - 1)])
    # 経路のまま（今ほぼ経路の上・戻る候補のどれか）が通れて、それがいちばん早いなら避けない
    if abs(d0) < _ON_LINE_M and (ok & line).any():
        best = int(np.argmin(np.where(ok, cost, np.inf)))
        if line[best]:
            # 線のまま（前の相手がいれば付いていく）が最善。付いていく相手を `lead` で返す（表示用）
            return _done(OvertakeResult("clear", None, None, [], tracks[j_lead] if j_lead is not None else None,
                                        len(cands), ""))
    if prev is not None:
        side_prev = math.copysign(1.0, prev.dT) if abs(prev.dT) > 1e-3 else 0.0
        side = np.sign(cands[:, 4])
        cost = cost + np.where((side != 0) & (side_prev != 0) & (side != side_prev), _SWITCH_COST, 0.0)
        if len(rows) > n_before:
            cost[-1] -= _KEEP_BONUS
    lead = None
    hit_line = first[line]
    if (hit_line >= 0).any():
        lead = tracks[int(hit_line[hit_line >= 0][0])]
    elif j_lead is not None:
        # 線の候補が当たりでなく減速しきれないなどで落ちた。付いていく相手を返す（呼び出し側が止まらずに付いていく）
        lead = tracks[j_lead]
    if not ok.any():
        # 通れる候補が無い。道幅と曲率を満たす候補のうち、相手との横の間隔がいちばん大きいものを
        # 逃げ道として返す（並んでいる相手の側へ元の線で戻っていかないため）。速度は呼び出し側が抑える
        k = int(np.argmax(gap))
        ranked = [Cand(*cands[k])] if np.isfinite(gap[k]) or gap[k] == np.inf else []
        return _done(OvertakeResult("blocked", None, ranked[0] if ranked else None, ranked, lead, len(cands),
                                    f"候補 {len(cands)} 本すべて不可（"
                                    + "・".join(f"{k}{v}" for k, v in fails.items() if v) + "）"))
    order = np.argsort(np.where(ok, cost, np.inf))
    top = [int(i) for i in order[:min(6, int(ok.sum()))]]
    ranked = [Cand(*cands[i]) for i in top]
    kept = len(rows) > n_before and int(order[0]) == len(rows) - 1
    return _done(OvertakeResult("avoid", None, ranked[0], ranked, lead, len(cands), "", kept), [_vel(i) for i in top])


def build(path: RaceLine, index: int, cand: Cand, s_cum: np.ndarray, obstacle: Obstacle | None,
          clearance: float = math.inf) -> AvoidPlan:
    """候補を経路の点へ当てはめた一時経路（点の数と添字は元の経路と同じ）。"""
    n = len(path)
    L = path.length
    nrm = normals(path.xy)
    if not math.isfinite(cand.S2):
        # 見る先の中では戻らない候補。経路の点へ当てはめるときは、見る先の終わりから滑らかに戻す
        # （段になって 0 へ落ちると、そこで曲率が跳ねる）。次の周期に引き直すので、ここは通らない
        cand = cand._replace(S2=max(cand.S1 + cand.L1, _H_MAX), L3=max(cand.L3, 2.5))
    s_end = cand.L1 if cand.dT == 0.0 else cand.S2 + cand.L3
    s_end = min(s_end, 0.9 * L)
    back = 10
    span = int(math.ceil(s_end / max(L / n, 1e-3))) + 2
    idx = (index - back + np.arange(back + span)) % n
    x = (s_cum[idx] - cand.s0 + 0.5 * L) % L - 0.5 * L
    d, _, _ = _profile(np.maximum(x, 0.0), *(np.array([v]) for v in cand[1:]))
    d = d[0]
    xy = path.xy.copy()
    xy[idx] += nrm[idx] * d[:, None]
    rl = RaceLine(xy=xy, v=path.v.copy(), kappa=curvature(xy, True), length=path.length, closed=True)
    # 窓は車の位置から先（後ろの点も今の横ずれでずらすが、確かめ・戻ったかの判定には使わない）
    keep = np.abs(d[back:]) > 1e-4
    ahead = idx[back:]
    win = ahead[keep] if keep.any() else ahead[:1]
    return AvoidPlan(rl, win, float(cand.dT if cand.dT != 0.0 else cand.d0), obstacle, clearance)

