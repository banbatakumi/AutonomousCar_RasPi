"""駐車ベンチの世界 — 縦列/車庫入れ/斜め/袋小路の配置を`Course`として組み立てる。

## なぜPNGではなくコードで作るのか

`sim/courses/*.png`はコース（周回路）の表現で、**駐車の評価軸とは形が違う**。
駐車で振りたいのは「隙間の長さ」「車庫の奥行き」「通路幅」という数値パラメータ
であって、絵ではない。`Course`はdataclassなので`grid`（bool配列）を直接渡せば
PNGを経由せずに作れる——`raycast()`/`collides()`/`occupied()`はすべて`grid`
だけを見るので、PNG由来のコースと完全に同じ扱いになる。

## 車体寸法との関係（`config/vehicle.toml`実測値）

    全長 0.37m（base_link から前 0.30・後 0.07）・全幅 0.18m
    最小旋回半径 wheelbase/tan(max_steer) = 0.23/tan(30°) = 0.398m

隙間の既定範囲はこの寸法に対して「実車の駐車として妥当な難しさ」になるよう
選んである（縦列の隙間は車長の1.6〜2.7倍、実車の教習が目安にする1.5倍前後を
下限側に含む）。**難しすぎて誰も解けない配置を並べても、planner の改善が
測れない**ので、`target_is_clear()`で目標姿勢自体の余裕を検査し、満たさない
配置は生成し直す。

## 真値のクリアランス

`clearance_field()`は壁からの距離場（EDT）を返す。**これは評価専用の真値**で、
planner が見るESDF（`raspi/`側、`cv2.distanceTransform`）とは別物——真値は
`scipy`で作る（`sim/requirements.txt`に既にあり、Mac専用なのでPiには影響しない）。
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

import numpy as np

from .course import Course

__all__ = ["ParkScenario", "SCENARIOS", "make", "clearance_field",
           "body_clearance", "target_is_clear"]

#: 世界の広さ [m]と解像度 [m/px]。駐車は数m四方で完結するのでコースより小さく細かく
WORLD_M = 6.0
RES = 0.01


@dataclass
class ParkScenario:
    """1つの駐車配置。`target`は**世界座標**の目標姿勢。"""

    name: str
    course: Course
    start: tuple[float, float, float]       #: 車両の初期姿勢（世界座標）
    target: tuple[float, float, float]      #: 目標姿勢（世界座標）
    #: 生成に使ったパラメータ（隙間長など）。失敗例を再現するためログに残す
    params: dict


# ── 矩形のラスタライズ ────────────────────────────────────────────────

def _blank() -> np.ndarray:
    n = int(round(WORLD_M / RES))
    return np.zeros((n, n), dtype=bool)


def _fill_rect(grid: np.ndarray, cx: float, cy: float, w: float, h: float,
               yaw: float = 0.0) -> None:
    """中心`(cx,cy)`・幅`w`(x方向)・高さ`h`(y方向)・`yaw`回転の矩形を壁で塗る。

    回転に対応するのは斜め駐車のため。回転後のbounding box内の全セルを
    ローカル座標へ戻して内外判定する（矩形なので範囲比較だけで済む）。
    """
    n = grid.shape[0]
    r = math.hypot(w, h) / 2.0 + RES
    c0 = max(0, int((cx - r) / RES))
    c1 = min(n, int((cx + r) / RES) + 1)
    r0 = max(0, int((cy - r) / RES))
    r1 = min(n, int((cy + r) / RES) + 1)
    if c0 >= c1 or r0 >= r1:
        return
    cols = (np.arange(c0, c1) + 0.5) * RES - cx
    rows = (np.arange(r0, r1) + 0.5) * RES - cy
    gx, gy = np.meshgrid(cols, rows)
    c, s = math.cos(-yaw), math.sin(-yaw)
    lx = gx * c - gy * s
    ly = gx * s + gy * c
    inside = (np.abs(lx) <= w / 2.0) & (np.abs(ly) <= h / 2.0)
    grid[r0:r1, c0:c1] |= inside


def _clear_x_band(grid: np.ndarray, y0: float, y1: float,
                  x0: float, x1: float) -> None:
    """`y0..y1` × `x0..x1` の矩形を空きへ戻す（壁の帯に開口を作る）。"""
    n = grid.shape[0]
    r0, r1 = max(0, int(y0 / RES)), min(n, int(y1 / RES) + 1)
    c0, c1 = max(0, int(x0 / RES)), min(n, int(x1 / RES) + 1)
    grid[r0:r1, c0:c1] = False


def _course(grid: np.ndarray, name: str) -> Course:
    from pathlib import Path
    return Course(name=name, path=Path(f"<{name}>"), resolution=RES,
                  origin=(0.0, 0.0), start=(0.0, 0.0, 0.0),
                  grid=np.ascontiguousarray(grid))


# ── 配置 ──────────────────────────────────────────────────────────────
# すべて「通路は x 方向、駐車スペースは y の小さい側」に揃えてある。
# 車両の初期姿勢は通路上で +x 向き。base_link は後輪車軸なので、車体を
# 目標位置の中央に収めるには base_link を車体中心より 0.115m 手前に置く
# （前0.30・後0.07 → 中心は base_link+0.115）。

_BODY_FWD = 0.30
_BODY_BACK = 0.07
_BODY_HALF_W = 0.09
_BODY_MID = (_BODY_FWD - _BODY_BACK) / 2.0     # base_link から車体中心まで


def _parallel(rng: random.Random) -> ParkScenario:
    """縦列駐車 — 縁石沿いに並んだ2台の隙間へ、通路と平行に入れる。"""
    gap = rng.uniform(0.60, 1.00)           # 隙間の長さ（車長0.37の1.6〜2.7倍）
    lane = rng.uniform(0.70, 1.00)          # 通路幅
    gx = rng.uniform(2.2, 3.2)              # 隙間の中心x
    curb_y = 2.00                           # 縁石の壁面（この上に駐車帯）
    slot_d = 0.34                           # 駐車帯の奥行き（車幅0.18＋左右8cm）
    grid = _blank()
    # 縁石（駐車帯の奥の壁）
    _fill_rect(grid, WORLD_M / 2, curb_y - 0.10, WORLD_M, 0.20)
    # 前後の駐車車両
    for sgn in (-1.0, 1.0):
        cx = gx + sgn * (gap / 2.0 + 0.30)
        _fill_rect(grid, cx, curb_y + slot_d / 2.0, 0.60, slot_d)
    # 通路の向かい側の壁
    far = curb_y + slot_d + lane
    _fill_rect(grid, WORLD_M / 2, far + 0.10, WORLD_M, 0.20)
    target = (gx - _BODY_MID, curb_y + slot_d / 2.0, 0.0)
    start = (gx + rng.uniform(0.5, 1.0), curb_y + slot_d + lane / 2.0, 0.0)
    return ParkScenario("parallel", _course(grid, "parallel"), start, target,
                        dict(gap=gap, lane=lane, gx=gx))


def _garage(rng: random.Random) -> ParkScenario:
    """車庫入れ — 通路に直交する車庫へ後退で入れる（前向きでは出られない奥行き）。"""
    width = rng.uniform(0.38, 0.54)         # 車庫の間口（車幅0.18＋左右10cm〜）
    depth = rng.uniform(0.45, 0.65)         # 車庫の奥行き
    lane = rng.uniform(0.75, 1.10)
    bx = rng.uniform(2.2, 3.2)
    mouth_y = 2.20                          # 通路と車庫の境界
    grid = _blank()
    back = mouth_y - depth
    # 車庫の奥の壁
    _fill_rect(grid, bx, back - 0.10, width + 0.60, 0.20)
    # 車庫の側壁（＋通路側の縁石を兼ねる帯）
    for sgn in (-1.0, 1.0):
        cx = bx + sgn * (width / 2.0 + 0.15)
        _fill_rect(grid, cx, back + depth / 2.0, 0.30, depth)
    _fill_rect(grid, WORLD_M / 2, mouth_y - 0.05, WORLD_M, 0.10)   # 通路側の縁石
    _clear_x_band(grid, mouth_y - 0.11, mouth_y + 0.01, bx - width / 2, bx + width / 2)
    # 向かい側の壁
    _fill_rect(grid, WORLD_M / 2, mouth_y + lane + 0.10, WORLD_M, 0.20)
    # 後退で入れる → 車体の後端が奥、前端が通路側（yaw=+90°）
    ty = back + _BODY_BACK + 0.08
    target = (bx, ty, math.pi / 2.0)
    start = (bx - rng.uniform(0.6, 1.0), mouth_y + lane / 2.0, 0.0)
    return ParkScenario("garage", _course(grid, "garage"), start, target,
                        dict(width=width, depth=depth, lane=lane, bx=bx))


def _angled(rng: random.Random) -> ParkScenario:
    """斜め駐車 — 通路に対して45°傾いた枠へ入れる。"""
    ang = rng.choice([math.radians(45), math.radians(-45), math.radians(60)])
    width = rng.uniform(0.40, 0.56)
    depth = rng.uniform(0.50, 0.70)
    lane = rng.uniform(0.80, 1.10)
    bx = rng.uniform(2.2, 3.2)
    mouth_y = 2.20
    grid = _blank()
    # 枠の中心線方向（通路から奥へ向かうベクトル）
    dirx, diry = math.cos(-math.pi / 2 + ang), math.sin(-math.pi / 2 + ang)
    cx = bx + dirx * depth / 2.0
    cy = mouth_y + diry * depth / 2.0
    # 側壁2枚（枠の中心線に平行）
    for sgn in (-1.0, 1.0):
        ox = -diry * sgn * (width / 2.0 + 0.06)
        oy = dirx * sgn * (width / 2.0 + 0.06)
        _fill_rect(grid, cx + ox, cy + oy, 0.12, depth,
                   yaw=math.atan2(diry, dirx) - math.pi / 2)
    # 奥の壁
    _fill_rect(grid, bx + dirx * (depth + 0.06), mouth_y + diry * (depth + 0.06),
               width + 0.30, 0.12, yaw=math.atan2(diry, dirx) - math.pi / 2)
    # 通路側の縁石。間口は枠の軸が縁石線を切る幅（width/|diry|）ぶん開ける
    _fill_rect(grid, WORLD_M / 2, mouth_y - 0.05, WORLD_M, 0.10)
    half = width / 2.0 / max(abs(diry), 0.3) + 0.05
    _clear_x_band(grid, mouth_y - 0.11, mouth_y + 0.01, bx - half, bx + half)
    _fill_rect(grid, WORLD_M / 2, mouth_y + lane + 0.10, WORLD_M, 0.20)
    # 前向きに入れる（枠の奥を向く）姿勢
    tyaw = math.atan2(diry, dirx)
    tx = bx + dirx * (depth - _BODY_FWD - 0.08)
    ty = mouth_y + diry * (depth - _BODY_FWD - 0.08)
    target = (tx, ty, tyaw)
    start = (bx - rng.uniform(0.7, 1.1), mouth_y + lane / 2.0, 0.0)
    return ParkScenario("angled", _course(grid, "angled"), start, target,
                        dict(ang_deg=math.degrees(ang), width=width,
                             depth=depth, lane=lane, bx=bx))


def _deadend(rng: random.Random) -> ParkScenario:
    """袋小路 — 行き止まりの通路で向きを180°変えて戻る。切り返し必須。"""
    lane = rng.uniform(0.80, 1.20)
    end_x = rng.uniform(3.2, 3.8)
    y0 = 2.00
    grid = _blank()
    _fill_rect(grid, WORLD_M / 2, y0 - 0.10, WORLD_M, 0.20)            # 下の壁
    _fill_rect(grid, WORLD_M / 2, y0 + lane + 0.10, WORLD_M, 0.20)     # 上の壁
    _fill_rect(grid, end_x + 0.10, y0 + lane / 2.0, 0.20, lane + 0.4)  # 行き止まり
    cy = y0 + lane / 2.0
    start = (end_x - rng.uniform(0.7, 1.0), cy, 0.0)                   # 行き止まりを向く
    target = (start[0] - rng.uniform(0.8, 1.3), cy, math.pi)           # 反転して手前へ
    return ParkScenario("deadend", _course(grid, "deadend"), start, target,
                        dict(lane=lane, end_x=end_x))


SCENARIOS = {
    "parallel": _parallel,
    "garage": _garage,
    "angled": _angled,
    "deadend": _deadend,
}


def make(name: str, rng: random.Random, *, min_target_clearance: float = 0.06,
         tries: int = 40) -> ParkScenario:
    """`name`の配置を1つ作る。**目標姿勢に余裕が無い配置は作り直す。**

    解けない配置を並べても planner の改善が測れないため（モジュール
    docstring参照）。`tries`回試して満たせなければ最後の候補を返す
    （生成器のパラメータ範囲が間違っていることを呼び出し側で気付けるように、
    例外にはしない）。
    """
    gen = SCENARIOS[name]
    sc = gen(rng)
    for _ in range(tries):
        if target_is_clear(sc, min_target_clearance):
            return sc
        sc = gen(rng)
    return sc


# ── 真値のクリアランス（評価専用） ──────────────────────────────────

def clearance_field(course: Course) -> np.ndarray:
    """壁からの距離場 [m]。`grid`と同じ形。**評価専用の真値。**"""
    from scipy import ndimage
    return ndimage.distance_transform_edt(~course.grid) * course.resolution


def body_clearance(course: Course, field: np.ndarray, x: float, y: float,
                   yaw: float, body: np.ndarray) -> float:
    """車体外形サンプル`body`（`Course.body_samples()`の戻り値）の最小クリアランス [m]。

    車体が壁に重なっていれば0.0。**衝突したかどうかではなく「どれだけ
    ぎりぎりだったか」**を測るための指標（成功/失敗の2値では、たまたま
    擦らなかった経路と余裕のある経路が区別できない）。
    """
    c, s = math.cos(yaw), math.sin(yaw)
    wx = x + body[:, 0] * c - body[:, 1] * s
    wy = y + body[:, 0] * s + body[:, 1] * c
    cols = (wx / course.resolution).astype(np.int32)
    rows = (wy / course.resolution).astype(np.int32)
    h, w = field.shape
    if not ((cols >= 0) & (cols < w) & (rows >= 0) & (rows < h)).all():
        return 0.0
    return float(field[rows, cols].min())


def footprint(margin: float = 0.0) -> list[list[float]]:
    """車体外形ポリゴン（`config/vehicle.toml`の実測値）を`margin`ぶん太らせたもの。"""
    return [[_BODY_FWD + margin, _BODY_HALF_W + margin],
            [_BODY_FWD + margin, -_BODY_HALF_W - margin],
            [-_BODY_BACK - margin, -_BODY_HALF_W - margin],
            [-_BODY_BACK - margin, _BODY_HALF_W + margin]]


def target_is_clear(sc: ParkScenario, margin: float) -> bool:
    """目標姿勢に車体を置いたとき、壁まで`margin`以上あるか。

    距離場（`clearance_field`）は600×600のEDTで、生成のやり直しループで
    毎回作ると割に合わない。**外形を`margin`ぶん太らせて衝突判定する**だけで
    同じ合否が出る（角の丸めぶんだけ厳しくなる側の近似）。
    """
    body = sc.course.body_samples(footprint(margin))
    return not sc.course.collides(*sc.target, body)
