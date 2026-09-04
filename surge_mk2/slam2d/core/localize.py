"""グローバルローカリゼーション — 直前の姿勢という手がかり無しで、地図全体から
自己位置を復元する。

`core/frontend.py`の`Frontend`はスキャンマッチングによる**追跡型**の局所推定
しか持たない（見失ったときの`RELOC_STAGES`再探索も位置0.4m・角度20°程度の
近傍限定）。保存済み地図を読み込んでRACEから走り始める場面では、直前の姿勢が
無いので、この追跡型の探索では初期姿勢に辿り着けない。

## 候補は「地図の空きセル × 粗い角度刻み」

地図の`known_free_mask()`（空きだと確信できるセル）の上に間隔`spacing`で
候補位置をばら撒き、角度は`angle_step`刻みで全周を回す。各候補で今のスキャンを
`score_map()`（尤度場）に対して採点し、最良のものを採用する。

## 1周期に1角度だけ進める

候補位置×候補角度の全探索を1回でやると、大きい地図では数百ms〜秒のブロッキング
になり、他のバス通信を止める。`step()`は1角度ぶんだけ評価して返す設計にして、
呼び出し側（`raspi/auto/slam2d_raceline.py`の`_locate()`）が`plan()`の1周期
ごとに1回呼ぶ想定（車両は`ready=False`で静止させたまま呼ぶこと。**動きながら
呼ぶ設計にはなっていない**——候補位置は固定座標系の点で、動いている間の姿勢は
候補と比較できない）。

## 左右対称・繰り返し形状のコースでは誤認識しうる

似た形の場所が複数あると、得点の1位と2位がほぼ並ぶ。`LocalizeResult.ambiguous`
で検出し、**自動では採用しない**（`core/frontend.py`の「見失ったら黙って走らない」
と同じ哲学）。呼び出し側がGUIのクリックヒント（`hint`）で探索範囲を絞れば、
同じ理由で解消しやすくなる。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import NamedTuple

import numpy as np

from .grid import OccGrid
from .types import ScanPoints

__all__ = ["LocalizeConfig", "LocalizeResult", "GlobalLocalizer"]


@dataclass(frozen=True, slots=True)
class LocalizeConfig:
    #: 候補位置の間隔 [m]。★ 荒すぎると量子化誤差で無関係な候補が偶然
    #: 僅差の得点になり、`ambiguous`が誤検出される（合成部屋での実測で、
    #: 0.3mでは非対称な部屋でも得点差0.04まで縮む場面があった。0.2mだと
    #: 同条件で0.18まで開く。実測に基づく値であり、コース形状によっては
    #: 再チューニングが要る——`ambiguous_gap`との組で決まる）
    spacing: float = 0.2
    angle_step: float = math.radians(15.0)  #: 候補角度の刻み
    max_points: int = 120                  #: 採点に使う点数の上限（`scanmatch.MAX_POINTS`と同じ考え方）
    #: 1位と（別の場所にある）2位の一致度差がこれ未満なら「対称の疑い」。
    #: `spacing`の量子化誤差より十分大きく取る（上のコメント参照）
    ambiguous_gap: float = 0.1
    #: これ以上離れていれば「別の場所」とみなす（同じ場所の近傍候補は数えない）
    ambiguous_min_dist: float = 1.0
    #: クリックヒントで絞り込む半径 [m]。★ `ambiguous_min_dist`の半分未満に
    #: すること。ヒント後の候補はどの2点も最大`2×hint_radius`しか離れられない
    #: ので、これを守ると「ヒントを送ったのに離れた2箇所を比べてまた
    #: ambiguousになる」が原理的に起きなくなる（クリックの目的そのものが
    #: 「対称の解消」なので、これが保証されないと機能として成立しない）
    hint_radius: float = 0.4


class LocalizeResult(NamedTuple):
    x: float
    y: float
    yaw: float
    score: float
    ambiguous: bool


class GlobalLocalizer:
    """1回の自己位置復元につき1個作る（使い捨て）。`step()`を`done`になるまで呼ぶ。"""

    def __init__(self, grid: OccGrid, *, hint: tuple[float, float] | None = None,
                config: LocalizeConfig = LocalizeConfig()) -> None:
        self._grid = grid
        self._config = config

        free = grid.known_free_mask()
        stride = max(1, round(config.spacing / grid.resolution))
        rows, cols = np.nonzero(free[::stride, ::stride])
        rows, cols = rows * stride, cols * stride
        xs, ys = grid.to_world(cols, rows)
        if hint is not None:
            keep = (xs - hint[0]) ** 2 + (ys - hint[1]) ** 2 <= config.hint_radius ** 2
            xs, ys = xs[keep], ys[keep]

        self._xs = np.asarray(xs, dtype=np.float64)
        self._ys = np.asarray(ys, dtype=np.float64)
        n_ang = max(1, round(2.0 * math.pi / config.angle_step))
        self._angles = np.linspace(-math.pi, math.pi, n_ang, endpoint=False)
        self._ai = 0
        #: 候補ごとの「これまで試した角度のうちの最良得点・その角度」
        self._best_score = np.zeros(self._xs.size, dtype=np.float64)
        self._best_yaw = np.zeros(self._xs.size, dtype=np.float64)
        #: 候補が1つも無い（ヒント範囲に空きセルが無い等）。呼び出し側は
        #: `step()`を呼ばずにこれだけ見て諦めてよい
        self.done = self._xs.size == 0

    @property
    def progress(self) -> float:
        return self._ai / max(1, len(self._angles))

    def step(self, pts: ScanPoints) -> bool:
        """候補角度1つぶんを全候補位置に対して採点する。戻り値は`done`と同じ。"""
        if self.done:
            return True

        sx = pts.x[pts.hit]
        sy = pts.y[pts.hit]
        if sx.size == 0:
            self._ai += 1
            self.done = self._ai >= len(self._angles)
            return self.done
        if sx.size > self._config.max_points:
            take = np.linspace(0, sx.size - 1, self._config.max_points).astype(np.int64)
            sx, sy = sx[take], sy[take]

        score_map = self._grid.score_map()
        yaw = float(self._angles[self._ai])
        c, s = math.cos(yaw), math.sin(yaw)
        rx = sx * c - sy * s
        ry = sx * s + sy * c

        fx = ((self._xs[:, None] + rx[None, :] - self._grid.origin[0])
              / self._grid.resolution - 0.5)
        fy = ((self._ys[:, None] + ry[None, :] - self._grid.origin[1])
              / self._grid.resolution - 0.5)
        totals = _bilinear_candidates(score_map, fx, fy) / sx.size

        better = totals > self._best_score
        self._best_score = np.where(better, totals, self._best_score)
        self._best_yaw = np.where(better, yaw, self._best_yaw)

        self._ai += 1
        # ★ 最良得点が十分でも打ち切らない。全候補は同じ角度列を共有しているため
        # （上のループ参照）、ここで打ち切ると「対称な複製がまだ自分のピーク角度
        # を試せていない」状態のまま`ambiguous`判定に使われ、回転方向にずれた
        # 対称性（同じ位置でも向きが異なる複製等）を見逃す。1位の候補だけが早く
        # ピークに達する場合があるため、`ambiguous`の安全性を保証するには全角度
        # を評価し切る必要がある
        self.done = self._ai >= len(self._angles)
        return self.done

    @property
    def result(self) -> LocalizeResult:
        """`done`になってから読む。候補が無ければ`ambiguous=True`・得点0で返す。"""
        if self._xs.size == 0:
            return LocalizeResult(0.0, 0.0, 0.0, 0.0, True)

        order = np.argsort(-self._best_score)
        i0 = int(order[0])
        x0, y0 = float(self._xs[i0]), float(self._ys[i0])
        score0 = float(self._best_score[i0])
        yaw0 = float(self._best_yaw[i0])

        # **最初に見つかった「離れた」候補とだけ比べる。** 1位のすぐ近くの
        # 候補（同じ場所のサブセル違い）は対称性の判定に数えない
        ambiguous = False
        for i in order[1:]:
            d = math.hypot(self._xs[i] - x0, self._ys[i] - y0)
            if d < self._config.ambiguous_min_dist:
                continue
            ambiguous = (score0 - float(self._best_score[i])) < self._config.ambiguous_gap
            break
        return LocalizeResult(x0, y0, yaw0, score0, ambiguous)


def _bilinear_candidates(sm: np.ndarray, fx: np.ndarray, fy: np.ndarray) -> np.ndarray:
    """`sm`を候補ごとに独立な(x, y)でバイリニア補間し、点について合計する。

    `fx`/`fy`は共に(M, N)（M候補×N点）。戻りは(M,)。`core/scanmatch.py`の
    `_bilinear`はグリッド状(X×Y)の候補向け（`match()`の並進探索がX/Yを独立に
    格子で振るため）だが、こちらは候補ごとに(x, y)が組で決まる点群探索なので
    形が合わない——単純な要素ごとの補間で足りる分、こちらのほうが軽い。
    """
    h, w = sm.shape
    x0 = np.floor(fx).astype(np.int32)
    y0 = np.floor(fy).astype(np.int32)
    tx = (fx - x0).astype(np.float32)
    ty = (fy - y0).astype(np.float32)

    okx = (x0 >= 0) & (x0 < w - 1)
    oky = (y0 >= 0) & (y0 < h - 1)
    ok = okx & oky
    cc = np.where(okx, x0, 0)
    rr = np.where(oky, y0, 0)

    v00 = sm[rr, cc]
    v01 = sm[rr, cc + 1]
    v10 = sm[rr + 1, cc]
    v11 = sm[rr + 1, cc + 1]
    val = ((v00 * (1 - tx) + v01 * tx) * (1 - ty)
           + (v10 * (1 - tx) + v11 * tx) * ty)
    return np.where(ok, val, 0.0).sum(axis=1)
