"""動的障害物 — 凍結した地図に**無いもの**を見つける。

地図を作る段では「動く物を壁として焼かない」（`grid.py` の hits/misses）。
ここは走る段の話で、**凍結した地図と今の点群の差**を取る。

## 判定は「空きだと確信しているセルに点が落ちた」だけ

    候補 = known_free なセル に落ちた点  −  壁のすぐ近く  −  飽和点

**「未知」のセルを空き扱いにしない**のが誤検出を出さない最大のコツ。1周目に
見えなかった場所（壁の陰、コースの外側）は unknown のままで、そこを通るたびに
「障害物だ」と言い出す planner は使い物にならない。

## 壁の近くは見ない

自己位置は数 cm〜数十 cm ずれる。壁の点がその隣の空きセルに落ちるのは
**日常的に起きる**。壁を `wall_pad` [m] 太らせた領域の中は、何が落ちても
動的障害物と呼ばない。太らせるぶん、壁際に立つ本物の障害物は見逃すが、
そちらは経路が壁際を通らない限り問題にならない。

**セル数ではなく距離で指定する。** 格子を 5cm から 2.5cm に細かくしたとき、
セル数のままだと膨張幅が黙って半分になり、**壁が動的障害物として検出されて
「前方 0.0m に障害物」で止まり続けた**。

## 1周期では信じない

`confirm()` に前の周期の結果を渡すと、**近い位置に2周期連続で出たものだけ**を
返す。10Hz なので 200ms の確認遅延が乗るが、その代わり単発のノイズで
急ブレーキを踏まなくなる。3 m/s なら 60cm 進むので、`stop_dist` はその
ぶんを含めて決めること。
"""

from __future__ import annotations

import math
from typing import NamedTuple

import numpy as np

from .deskew import Points
from .grid import OccGrid
from .local_map import _edt

__all__ = ["Obstacle", "VelocityTracker", "swept", "free_mask", "wall_distance", "near_pad", "detect", "confirm", "blocking"]

#: クラスタを切る点間距離 [m]。これより離れていたら別の物体
_SPLIT_M = 0.18
#: 障害物の点として数える帯を、`wall_pad` よりどれだけ壁へ寄せるか [m]（`near_pad`）
_NEAR_M = 0.10
#: 同じく帯の下限 [m]（これより壁に近い点は数えない）
_NEAR_MIN_M = 0.10
#: 「壁のずれ」とみなす塊の条件（`_is_wall_shift`）。長さ（点の並びの主方向の標準偏差）[m]・
#: 細さ（副方向/主方向）・壁からの平均距離 [m]
_WALL_SHIFT_SPREAD = 0.12
_WALL_SHIFT_THIN = 0.15
_WALL_SHIFT_DIST = 0.30


class Obstacle(NamedTuple):
    x: float                               #: 重心 [m]（map フレーム）
    y: float
    r: float                               #: 半径 [m]。点群の広がりから出す
    n: int                                 #: 構成する点の数
    #: 構成する点 (N, 2) [m]（map フレーム）。`detect` が入れる。手で作った物は None
    #: （重心と半径は見えている面だけから出すので、見る角度で 10cm 近く動く。車体が当たるか
    #: どうかはこの点で直接測る — `avoid.clearance_to_points`）
    pts: np.ndarray | None = None
    #: 速度 [m/s]（map フレーム）。`VelocityTracker` が入れる。静止していると見なせば 0
    #: （動いているかを二択で使う所: 止まって待つか迂回するか・残像・相手の線の学習など）
    vx: float = 0.0
    vy: float = 0.0
    #: 速度の推定値 [m/s]（map フレーム。静止と見なしても 0 にしない）と、その標準偏差 [m/s]（大きい方の向き。
    #: 観測が少ないほど大きい。`VelocityTracker` が無いときは inf）。追い抜きの予測（`overtake`）は二択で
    #: 切らずにこれを使う（`docs/overtake.md`）
    ex: float = 0.0
    ey: float = 0.0
    sv: float = math.inf
    #: この物の追跡が一度でも確かに動いたか（粘り付く。車らしい扱い — 見えない奥行き・横の余裕 — に使う。
    #: 周期ごとの判定で揺れない）
    moved: bool = False

    @property
    def speed(self) -> float:
        return math.hypot(self.vx, self.vy)


def free_mask(grid: OccGrid, wall_pad: float) -> np.ndarray:
    """障害物の候補にしてよいセル（空きと確信している かつ 壁から `wall_pad` 以上離れている）。

    格子全体の膨張なので重い（Pi で数ms）。凍結地図では変わらないので、周期ごとに
    呼ぶ側は `grid.seq` と `wall_pad` で覚えておいて `detect(free=...)` へ渡すこと。
    """
    # ★ 壁からの**ユークリッド距離**で切る。4近傍の膨張（`dilate`）はひし形に太るので、
    #   格子に対して斜めの壁は指定の 71% しか除外されず（0.15m に対して 10.6cm）、自己位置が
    #   その幅を超えてずれると斜めの壁の点が障害物の候補になった
    pad = max(1, int(round(wall_pad / grid.resolution))) * grid.resolution
    return grid.known_free_mask() & (wall_distance(grid) > pad + 1e-6)


def wall_distance(grid: OccGrid) -> np.ndarray:
    """各セルから最寄りの壁までのユークリッド距離 [m]（格子と同じ形）。"""
    return _edt(~grid.wall_mask(), grid.resolution)


def near_pad(wall_pad: float) -> float:
    """障害物の点として**数える**帯の、壁からの距離 [m]（`detect` の `near`）。

    実機（道幅 約0.6m）で `wall_pad=0.25` だと、壁から 25cm に置いた障害物は
    毎周期 1〜2 点しか `wall_pad` の外に落ちず、`min_points` に届かないまま 2.3m/s で当たった。
    道の半幅が 30cm だと `wall_pad` の外は中央の 10cm しか無い。そこで「`wall_pad` の外に
    1点でもある塊」に限って、もう 10cm 壁寄りの点まで数える。壁の点だけでできた塊は
    `wall_pad` の外に点を持たないので、誤検出は `wall_pad` で決まるまま。
    """
    return min(wall_pad, max(_NEAR_MIN_M, wall_pad - _NEAR_M))


def _is_wall_shift(px: np.ndarray, py: np.ndarray, wall_dist: np.ndarray, grid: OccGrid) -> bool:
    """塊が「地図の壁が動いた」ものか: 壁の近くにある、**長くて細い線**（標準偏差 12cm = 長さ約 40cm 超）。

    ダクトの壁は他車の衝突でへこみ・ずれるので、地図を作った後は壁が数 cm〜30cm ずれうる。
    ずれた壁の面は壁の脇（`wall_pad` の外）に一本の線として写る。物体（他車・人・箱）は
    塊でコンパクトか、壁から離れている。2026-11-14 `surge_20261114_074938`: 毎周同じ 2 か所に
    長さ 0.7m・厚さ 1cm の線が 6m 先から障害物として出た（自己位置は一致度 0.9 以上で正常）。
    同じ地図の別の記録の本物の障害物は、長さが 0.06〜0.12・壁から 0.3m 以上離れていた。
    """
    col, row = grid.to_cell(px, py)
    ok = grid.inside(col, row)
    if not ok.any() or float(wall_dist[row[ok], col[ok]].mean()) > _WALL_SHIFT_DIST:
        return False
    cov = np.cov(np.vstack([px, py]))
    minor, major = np.sqrt(np.maximum(np.sort(np.linalg.eigvalsh(cov)), 0.0))
    return bool(major >= _WALL_SHIFT_SPREAD and minor <= _WALL_SHIFT_THIN * major)


def detect(grid: OccGrid, pts: Points, pose: tuple[float, float, float], *,
           wall_pad: float = 0.15, min_points: int = 3,
           max_obstacles: int = 8, free: np.ndarray | None = None,
           near: np.ndarray | None = None, behind=None,
           wall_dist: np.ndarray | None = None) -> list[Obstacle]:
    """凍結地図に無いものを拾う。**`grid.frozen` でなければ何も返さない。**

    地図を作っている最中は「地図に無い」が当たり前なので、判定に意味が無い。

    :param free: `free_mask(grid, wall_pad)` の結果（省略すると毎回作る）
    :param near: `free_mask(grid, near_pad(wall_pad))` の結果。渡すと、`free` に1点でも落ちた
        塊はこの帯の点まで数える（省略すると `free` の点だけで数える）
    :param wall_dist: `wall_distance(grid)` の結果。渡すと、壁の脇の長く細い線（壁のずれ。`_is_wall_shift`）を捨てる
    :param behind: 車体座標の点 `(x, y)` → 地図の壁の向こうに見えた点か（`Frontend.behind_wall`）。渡すと、
        地図の空きに落ちた候補のうち壁の向こうの点を捨てる。LiDAR の光は壁を通らないので、そういう点は
        低い壁の上を越えて見えた奥か、壁での反射。2026-11-06 `surge_20261106_024246`: 木の仕切り壁（LiDAR の
        上に約 3cm）の向こうの点が、ヘアピンの空き（y≈5.02）に一直線に出て 5 周で障害物になり、「経路へ戻っている」と
        横グリップの学習の停止を起こした。本物の障害物は壁の手前にあるので消えない（壁越しに早く見つけることだけ失う）
    """
    if not grid.frozen or len(pts) == 0:
        return []
    hit = pts.hit
    if not hit.any():
        return []

    x0, y0, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    wx = x0 + pts.x[hit] * c - pts.y[hit] * s
    wy = y0 + pts.x[hit] * s + pts.y[hit] * c

    col, row = grid.to_cell(wx, wy)
    ok = grid.inside(col, row)
    if free is None:
        free = free_mask(grid, wall_pad)
    core = np.zeros(wx.size, dtype=bool)
    core[ok] = free[row[ok], col[ok]]
    if not core.any():
        return []
    cand = core
    if near is not None:
        cand = np.zeros(wx.size, dtype=bool)
        cand[ok] = near[row[ok], col[ok]]
        cand |= core

    if behind is not None:
        i = np.flatnonzero(cand)
        drop = np.asarray(behind(pts.x[hit][i], pts.y[hit][i]), dtype=bool)
        cand[i[drop]] = False
        core[i[drop]] = False
        if not core.any():
            return []
    idx = np.flatnonzero(cand)
    px, py = wx[idx], wy[idx]
    strict = core[idx]
    # 点は方位順に並んでいるので、隣同士の距離が開いたところで切ればよい
    gap = np.hypot(np.diff(px), np.diff(py)) > _SPLIT_M
    # ★ 方位 0°（**真正面**）が配列の継ぎ目。末尾と先頭がつながっているなら、最初の切れ目が
    #   先頭に来るよう回してから切る。回さないと、他に候補点があるだけで正面の障害物が
    #   2つに割れ（中心が横へ寄り、半径は半分以下）、遠くでは点数が足りず検出されなかった
    if gap.any() and math.hypot(px[0] - px[-1], py[0] - py[-1]) <= _SPLIT_M:
        k = int(np.flatnonzero(gap)[0]) + 1
        px, py, strict = np.roll(px, -k), np.roll(py, -k), np.roll(strict, -k)
        gap = np.hypot(np.diff(px), np.diff(py)) > _SPLIT_M
    starts = np.concatenate([[0], np.flatnonzero(gap) + 1, [idx.size]])

    out: list[Obstacle] = []
    for a, b in zip(starts[:-1], starts[1:]):
        if b - a < min_points or not strict[a:b].any():
            continue
        if (wall_dist is not None and b - a >= 5
                and _is_wall_shift(px[a:b], py[a:b], wall_dist, grid)):
            continue
        cx, cy = float(px[a:b].mean()), float(py[a:b].mean())
        r = float(np.hypot(px[a:b] - cx, py[a:b] - cy).max())
        out.append(Obstacle(cx, cy, max(r, grid.resolution), int(b - a),
                            np.column_stack([px[a:b], py[a:b]])))

    out.sort(key=lambda o: -o.n)
    return out[:max_obstacles]


def confirm(now: list[Obstacle], prev: list[Obstacle], *,
            tol: float = 0.35) -> list[Obstacle]:
    """**2周期連続で同じ場所に出たものだけ**を残す。単発のノイズを落とす。"""
    if not prev:
        return []
    out = []
    for o in now:
        if any(math.hypot(o.x - p.x, o.y - p.y) <= tol + o.r + p.r for p in prev):
            out.append(o)
    return out


def blocking(obstacles: list[Obstacle], path_xy: np.ndarray, index: int,
             *, half_width: float, ahead_m: float, step: float,
             skip_m: float = 0.35, closed: bool = True) -> tuple[Obstacle | None, float]:
    """これから通る帯に入っている障害物と、そこまでの距離 [m]。

    経路上の `index` から `ahead_m` 先までの点だけを見る。**後ろや、1周先の
    同じ場所に居る物で止まらない**ため（閉ループなので添字は巡回する）。

    :param skip_m: 自車の足元ぶん、手前を飛ばす距離 [m]

    ★ **自分の真横や後ろを数えてはいけない。** `index` は自車が今いる場所なので、
    そこから数えると**自車のすぐ横にある壁**（＝地図の誤差で動的障害物と誤検出
    されたもの）が「前方 0.0m の障害物」になり、走り出した瞬間に止まる。
    実測でこれに引っ掛かった（`+138°`＝左後方の物で停止していた）。

    戻り値の距離は経路に沿った弧長の近似（点数 × 間隔）。**車体前端ではなく
    base_link からの距離**なので、停止判定に使う側で前オーバーハングを足すこと。
    """
    if not obstacles or len(path_xy) == 0:
        return None, math.inf
    n = len(path_xy)
    span = max(1, int(ahead_m / max(step, 1e-3)))
    span = min(span, n - 1)
    skip = min(span, max(0, int(skip_m / max(step, 1e-3))))
    idx = index + np.arange(skip, span + 1)
    # 開いた経路は終点の先を見ない（`closed=False`、`nav/route.py`）
    idx = idx % n if closed else idx[idx < n]
    if len(idx) == 0:
        return None, math.inf
    seg = path_xy[idx]

    best: Obstacle | None = None
    best_d = math.inf
    for o in obstacles:
        d2 = (seg[:, 0] - o.x) ** 2 + (seg[:, 1] - o.y) ** 2
        k = int(np.argmin(d2))
        if math.sqrt(float(d2[k])) <= half_width + o.r:
            d = (k + skip) * step
            if d < best_d:
                best, best_d = o, d
    return best, best_d


#: 動いていると見なす最小の速さ [m/s]。見える面が変わると重心が数 cm ずれる（10Hz で 0.2〜0.5m/s 相当）
#: ので、それより十分上に置く
MOVING_MIN_SPEED = 0.3
#: 見えなかった周期も履歴を残す時間 [s]。★1 周期見落とす（壁際の無視する帯に点が落ちる・2 つに割れる）
#: たびに履歴を捨てると、速度が分かるまでの間は静止物に見える。シムで前の車の 18% の周期が静止物に見え、
#: 追い抜きの計画が止まる・動くで入れ替わった（sim.bench --mover）
_TRACK_KEEP_S = 0.3
#: 追跡の EKF の雑音: 重心の観測の揺れ [m]・速度を変える白色加速度 [m/s²]・ヨーレートの変化 [rad/s²]
_EKF_SIGMA_MEAS = 0.05
_EKF_SIGMA_ACC = 3.0
_EKF_SIGMA_WDOT = 6.0
#: 新しい追跡の速度・ヨーレートの初期の標準偏差と、ヨーレートの上限 [rad/s]
_EKF_P0_VEL = 2.0
_EKF_P0_W = 1.5
_EKF_W_MAX = 4.0
#: 動いていると言うのに要る観測の数・最初の観測からの時間 [s]
_MOVING_MIN_UPDATES = 3
_MOVING_MIN_AGE_S = 0.2
#: 速さが推定の標準偏差のこの倍に満たなければ静止とする
_MOVING_NSIG = 1.0
#: 動いたかを確かめる観測の窓 [s] と、窓の前半・後半の平均位置の差の下限 [m]・その差の速さの下限
#: （`MOVING_MIN_SPEED` に対する割合）。★EKF の速さだけで決めると、重心が白色に 5cm 揺れる静止物の
#: 17% の周期が動いて見えた。平均を取ると揺れは打ち消され 3% になる（3cm なら 0%）
_MOVING_WIN_S = 0.6
_MOVING_DISP_M = 0.08
_MOVING_DISP_SPEED = 0.5
#: 同じ追跡に結ばれた検出を 1 つの物にまとめる、点どうしの距離の上限 [m]（`VelocityTracker.update`）
_MERGE_M = 0.30


def _ct_predict(x: np.ndarray, T: float) -> tuple[np.ndarray, np.ndarray]:
    """協調旋回（CT）モデルで状態 `[px, py, vx, vy, ω]` を `T` 秒進めた値と、そのヤコビアン。"""
    _, _, vx, vy, w = x
    wT = w * T
    s, c = math.sin(wT), math.cos(wT)
    if abs(w) < 1e-4:
        s_w, c_w = T, 0.5 * w * T * T                  # sin(ωT)/ω, (1−cos ωT)/ω の ω→0 の近似
        ds_w, dc_w = -w * T ** 3 / 3.0, 0.5 * T * T     # それぞれの ω による微分
    else:
        s_w, c_w = s / w, (1.0 - c) / w
        ds_w = (T * c * w - s) / (w * w)
        dc_w = (T * s * w - (1.0 - c)) / (w * w)
    xn = np.array([x[0] + s_w * vx - c_w * vy, x[1] + c_w * vx + s_w * vy,
                   c * vx - s * vy, s * vx + c * vy, w])
    F = np.eye(5)
    F[0, 2], F[0, 3], F[0, 4] = s_w, -c_w, ds_w * vx - dc_w * vy
    F[1, 2], F[1, 3], F[1, 4] = c_w, s_w, dc_w * vx + ds_w * vy
    F[2, 2], F[2, 3], F[2, 4] = c, -s, -T * (s * vx + c * vy)
    F[3, 2], F[3, 3], F[3, 4] = s, c, T * (c * vx - s * vy)
    return xn, F


def _ct_q(T: float) -> np.ndarray:
    """白色加速度（x・y 独立）とヨーレートのランダムウォークを `T` 秒で離散化したプロセス雑音。"""
    q = _EKF_SIGMA_ACC ** 2
    Q = np.zeros((5, 5))
    for i, j in ((0, 2), (1, 3)):
        Q[i, i] = q * T ** 3 / 3.0
        Q[i, j] = Q[j, i] = q * T ** 2 / 2.0
        Q[j, j] = q * T
    Q[4, 4] = _EKF_SIGMA_WDOT ** 2 * T
    return Q


class _Track:
    """1 つの追跡（EKF の状態・共分散と、動いたかを確かめる観測の履歴）。"""

    __slots__ = ("x", "P", "t", "t0", "n", "hist", "moved")

    def __init__(self, t: float, px: float, py: float) -> None:
        sm, sv = _EKF_SIGMA_MEAS ** 2, _EKF_P0_VEL ** 2
        self.x = np.array([px, py, 0.0, 0.0, 0.0])
        self.P = np.diag([sm, sm, sv, sv, _EKF_P0_W ** 2])
        self.t = self.t0 = t
        self.n = 1
        self.hist = [(t, px, py)]
        self.moved = False

    def predicted(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        T = t - self.t
        if T <= 0.0:
            return self.x, self.P
        x, F = _ct_predict(self.x, T)
        return x, F @ self.P @ F.T + _ct_q(T)

    def correct(self, t: float, x: np.ndarray, P: np.ndarray, zx: float, zy: float) -> None:
        S = P[:2, :2] + np.eye(2) * _EKF_SIGMA_MEAS ** 2
        K = P[:, :2] @ np.linalg.inv(S)
        x = x + K @ np.array([zx - x[0], zy - x[1]])
        P = P - K @ P[:2, :]
        x[4] = max(-_EKF_W_MAX, min(_EKF_W_MAX, x[4]))
        self.x, self.P, self.t = x, 0.5 * (P + P.T), t
        self.n += 1
        self.hist = [h for h in self.hist if t - h[0] <= _MOVING_WIN_S] + [(t, zx, zy)]

    def velocity(self, t: float) -> tuple[float, float]:
        """外へ出す速度。動いていると言える条件を満たさなければ (0, 0)。"""
        if self.n < _MOVING_MIN_UPDATES or t - self.t0 < _MOVING_MIN_AGE_S:
            return 0.0, 0.0
        vx, vy = float(self.x[2]), float(self.x[3])
        sp = math.hypot(vx, vy)
        if sp < MOVING_MIN_SPEED:
            return 0.0, 0.0
        u = np.array([vx, vy]) / sp
        if sp < _MOVING_NSIG * math.sqrt(max(float(u @ self.P[2:4, 2:4] @ u), 0.0)):
            return 0.0, 0.0
        m = len(self.hist) // 2
        if m < 1:
            return 0.0, 0.0
        a, b = np.asarray(self.hist[:m]), np.asarray(self.hist[m:])
        dt = float(b[:, 0].mean() - a[:, 0].mean())
        disp = math.hypot(float(b[:, 1].mean() - a[:, 1].mean()), float(b[:, 2].mean() - a[:, 2].mean()))
        if dt <= 0.0 or disp < _MOVING_DISP_M or disp / dt < _MOVING_DISP_SPEED * MOVING_MIN_SPEED:
            return 0.0, 0.0
        return vx, vy


class VelocityTracker:
    """周期ごとの検出を結んで、障害物の速度を出す（協調旋回モデルの EKF）。

    状態は map フレームの `[px, py, vx, vy, ω]`、観測は検出の重心。毎周期、新しい検出を各追跡の
    「今の時刻への予測位置」に近いものへ結ぶ（距離 `tol + 半径`）。結ばれなかった追跡も
    `_TRACK_KEEP_S` の間は予測だけで残す。

    **速度は直交成分で持つ**（向きと速さで持つ CTRV は、止まった物で向きが観測できず線形化が特異になる。
    障害物には静止物も混ざる）。★それまでの直線の回帰（直近 0.8s）は、曲がる相手の向きが 0.4s ぶん遅れ
    （カーブで中央 28°）、速さも弦のぶん遅く出た。EKF は同じ検出の列（sim.bench --mover の記録）で
    向き 4〜5°・速度ベクトルの誤差 0.87 → 0.30m/s・0.5 秒先の予測 0.65 → 0.35m。

    静止とする（速度 0 で返す）のは: 観測が少ない・新しい、速さが `MOVING_MIN_SPEED` 未満か推定の
    標準偏差の `_MOVING_NSIG` 倍未満、窓の中で実際に動いていない（`_MOVING_DISP_*`）とき。
    """

    def __init__(self, tol: float = 0.5) -> None:
        self._tol = tol
        self._tracks: list[_Track] = []

    def reset(self) -> None:
        self._tracks = []

    def update(self, t: float, now: list[Obstacle]) -> list[Obstacle]:
        preds = [tr.predicted(t) for tr in self._tracks]
        now = self._merge_split(now, preds)
        used: set[int] = set()
        new_tracks: list[_Track] = []
        out: list[Obstacle] = []
        for o in now:
            best, best_d = -1, math.inf
            for i, (x, _) in enumerate(preds):
                if i in used:
                    continue
                d = math.hypot(o.x - x[0], o.y - x[1])
                if d <= self._tol + o.r and d < best_d:
                    best, best_d = i, d
            if best >= 0:
                used.add(best)
                tr = self._tracks[best]
                tr.correct(t, *preds[best], o.x, o.y)
            else:
                tr = _Track(t, o.x, o.y)
            new_tracks.append(tr)
            vx, vy = tr.velocity(t)
            tr.moved = tr.moved or bool(vx or vy)
            ex, ey = float(tr.x[2]), float(tr.x[3])
            sv = math.sqrt(max(float(np.linalg.eigvalsh(tr.P[2:4, 2:4]).max()), 0.0))
            if not tr.moved and tr.n >= _MOVING_MIN_UPDATES and t - tr.t0 >= _MOVING_MIN_AGE_S:
                # 観測が溜まり、窓の中で動いていない（`velocity` が 0）物は速度 0・不確かさ 0。★EKF の速度の σ は
                # プロセス雑音（加速度 3m/s²）のため止まった物でも 0.3〜0.5m/s から下がらず、推定値も 0 の周りで揺れる。
                # それをそのまま予測に使うと静止物が「ゆっくり動く・大きく広がる物」に見え、避けにくくなった
                # （sim.bench toyota2 の静止物 3 つ: 平均ラップ 10.5 → 16.1 秒・Hybrid A* 3 → 7 回）
                ex = ey = sv = 0.0
            out.append(o._replace(vx=vx, vy=vy, ex=ex, ey=ey, sv=sv, moved=tr.moved))
        for i, tr in enumerate(self._tracks):
            if i not in used and t - tr.t <= _TRACK_KEEP_S:
                new_tracks.append(tr)
        self._tracks = new_tracks
        return out

    def _merge_split(self, now: list[Obstacle], preds) -> list[Obstacle]:
        """1 つの物が 2 つ以上の検出に割れたもの（車の後ろ面と側面が点の間隔で切れる等）を 1 つにまとめる。

        各検出を最も近い追跡（門 `tol + 半径` の内側）に割り当て、同じ追跡に割り当たった検出のうち、最も近いもの
        と点どうしが `_MERGE_M` 以内のものを合わせる。★割れたまま 1 対 1 で結ぶと、余った方は毎周期新しい追跡に
        なって静止物と出続け、動く車の位置に静止物が居ることになった（付いていく候補がそれに当たって "blocked"、
        「避けられない障害物」で急減速。sim.bench --mover、相手 0.65 倍の後ろで 1 周に何度も）。静止物は自分の
        追跡が最も近いので、そばを通る車の追跡には入らない。"""
        if len(now) < 2 or not preds:
            return now
        groups: dict[int, list[tuple[float, int]]] = {}
        for k, o in enumerate(now):
            best, best_d = -1, math.inf
            for i, (x, _) in enumerate(preds):
                d = math.hypot(o.x - x[0], o.y - x[1])
                if d <= self._tol + o.r and d < best_d:
                    best, best_d = i, d
            if best >= 0:
                groups.setdefault(best, []).append((best_d, k))
        drop: set[int] = set()
        merged: dict[int, Obstacle] = {}
        for members in groups.values():
            if len(members) < 2:
                continue
            members.sort()
            k0 = members[0][1]
            base = now[k0]
            if base.pts is None:
                continue
            pts = [base.pts]
            for _, k in members[1:]:
                o = now[k]
                if o.pts is None:
                    continue
                d2 = ((o.pts[:, None, :] - base.pts[None, :, :]) ** 2).sum(axis=2)
                if float(np.sqrt(d2.min())) <= _MERGE_M:
                    pts.append(o.pts)
                    drop.add(k)
            if len(pts) > 1:
                p = np.vstack(pts)
                cx, cy = float(p[:, 0].mean()), float(p[:, 1].mean())
                r = float(np.hypot(p[:, 0] - cx, p[:, 1] - cy).max())
                merged[k0] = base._replace(x=cx, y=cy, r=max(r, base.r), n=len(p), pts=p)
        return [merged.get(k, o) for k, o in enumerate(now) if k not in drop]


def swept(o: Obstacle, horizon_s: float, step: float) -> list[Obstacle]:
    """`o` が等速で `horizon_s` 秒動く間の通り道を、円の列（間隔 `step` 以下）で返す。
    静止していれば `[o]`。"""
    sp = o.speed
    if sp < MOVING_MIN_SPEED or horizon_s <= 0.0:
        return [o]
    length = sp * horizon_s
    n = max(1, int(math.ceil(length / max(step, 1e-3))))
    return [Obstacle(o.x + o.vx * horizon_s * k / n, o.y + o.vy * horizon_s * k / n, o.r, o.n)
            for k in range(n + 1)]
