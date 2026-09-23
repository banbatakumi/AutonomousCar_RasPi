"""面要素の地図 — 壁のセルごとに「点の平均位置と法線」を持ち、最寄りの壁を表引きする。

`core/register.py`（位置合わせ）が点ごとに「どの壁のどの向きの面に当たったはずか」
を引くための構造。点対線ICP（PL-ICP）の対応付けを、KD木ではなく格子の表引きで
行う（Pi には scipy が無く、KD木を毎周期組み直すのも割に合わない）。

## なぜ距離場（距離変換そのもの）を残差にしないのか

最初は「最寄りの壁セル中心までの距離」を双一次補間して残差にしていた。
ところが複数のスキャンを重ねた地図では、測距ノイズ（1〜2cm）のぶん壁が
2〜3セルの厚みになり、**壁の内側は距離0の平らな台地**になる。そこに落ちた点は
残差も勾配も0で、姿勢を何も拘束しない。回頭方向の点群側の情報が推測航法の
事前分布と同程度まで落ち、ジャイロのゼロ点ずれ0.5°/sがそのまま地図ごと
回っていった（`sim/slam_bench.py` normal/real: 2周で向き17°ずれ）。

ここでは壁のセルごとに**そこへ落ちた点の平均**（サブセル精度の壁の位置）と
**近傍の点の主成分から作った法線**を持ち、残差を

    r = n · (p − q)          （q: 最寄りの壁セルの平均位置, n: その法線）

にする。厚い壁の内側でも最寄りのセル（＝自分のいるセル）の平均から外れた分だけ
残差が出るので、台地にならない。

## 表引き

`cv2.distanceTransformWithLabels(..., DIST_LABEL_PIXEL)` は、各セルについて
最寄りの壁セルの番号を返す（壁セルはラスタ順に 1, 2, ... と番号が振られる）。
番号→壁セルの表を1本持てば、任意の点の対応先がO(1)で引ける。cv2 が無い環境は
Jump Flooding（log2(n)回の9近傍伝播）で同じ表を作る。

## 平面かどうか

近傍の点の広がり（共分散の固有値比）が細長ければ**平面**（点対平面の1残差）、
丸ければ**点**（角・柱・孤立した障害物。点対点の2残差）として扱う。

★ **近傍セルが`min_support`個に満たないセルは、そもそも対応に使わない。**
遠くの壁は角度分解能のぶん点がまばらになり（8m先で1°＝14cm間隔）、孤立した
セルが「角のような特徴」に見えてしまう。それを点対点で拘束すると、
**平行な壁しか無い通路なのに進行方向が決まっているかのような情報**が出て、
推測航法の事前分布を押しのけてしまう（実測: 通路で進行方向の情報が横方向の
7倍に化けた）。裏付けの無いセルは「そこに何かある」以上のことを言えない。
"""

from __future__ import annotations

import math

import numpy as np

__all__ = ["SurfaceMap", "Association"]

try:                                         # pragma: no cover - 環境依存
    import cv2 as _cv2
except ImportError:                          # pragma: no cover
    _cv2 = None

from typing import NamedTuple


class Association(NamedTuple):
    qx: np.ndarray        #: 対応先の壁の位置（世界座標）
    qy: np.ndarray
    nx: np.ndarray        #: 対応先の法線（単位ベクトル）
    ny: np.ndarray
    planar: np.ndarray    #: True なら点対平面、False なら点対点
    #: 対応が有効（格子の内側・既知・最寄りの壁まで`max_dist`未満）
    valid: np.ndarray
    #: 点から対応先までの距離（平面なら法線方向の距離の絶対値）[m]
    dist: np.ndarray


class SurfaceMap:
    """壁の面要素（平均位置・法線）と最寄りの壁の表。

    :param occ: 壁セルのブールマスク（`grid[row, col]`）
    :param mean_x, mean_y: 壁セルごとの点の平均位置[m]（世界座標）。None なら
        セル中心
    :param known: 観測済みのセル。None なら全て既知。**未知のセルに落ちた点は
        対応を付けない**（見ていない場所の壁は「無い」のではなく「知らない」）
    """

    def __init__(self, occ: np.ndarray, *, resolution: float, origin: tuple[float, float],
                 mean_x: np.ndarray | None = None, mean_y: np.ndarray | None = None,
                 known: np.ndarray | None = None, max_dist: float = 0.4,
                 normal_radius: int = 2, planar_ratio: float = 0.25,
                 min_support: int = 3, tangent_gate: float | None = None) -> None:
        self.resolution = float(resolution)
        #: 接線方向のはみ出しの許容[m]（既定は1セル）
        self.tangent_gate = float(tangent_gate) if tangent_gate is not None else float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.max_dist = float(max_dist)
        self.height, self.width = occ.shape
        self.known = known
        self.empty = not bool(occ.any())
        self.cells = int(occ.sum())
        if self.empty:
            return

        rows, cols = np.nonzero(occ)
        cx = self.origin[0] + (cols + 0.5) * self.resolution
        cy = self.origin[1] + (rows + 0.5) * self.resolution
        if mean_x is not None and mean_y is not None:
            qx = mean_x[rows, cols].astype(np.float64)
            qy = mean_y[rows, cols].astype(np.float64)
        else:
            qx, qy = cx, cy

        # ── 法線: 近傍 (2r+1)² セルの壁点の共分散の最小固有ベクトル ──
        n_occ = rows.size
        idx_map = np.full(occ.shape, -1, dtype=np.int64)
        idx_map[rows, cols] = np.arange(n_occ)
        sx = np.zeros(n_occ); sy = np.zeros(n_occ)
        sxx = np.zeros(n_occ); syy = np.zeros(n_occ); sxy = np.zeros(n_occ)
        cnt = np.zeros(n_occ)
        r_ = normal_radius
        for dr in range(-r_, r_ + 1):
            for dc in range(-r_, r_ + 1):
                rr, cc = rows + dr, cols + dc
                ok = (rr >= 0) & (rr < self.height) & (cc >= 0) & (cc < self.width)
                j = np.full(n_occ, -1, dtype=np.int64)
                j[ok] = idx_map[rr[ok], cc[ok]]
                has = j >= 0
                if not has.any():
                    continue
                px, py = qx[j[has]] - qx[has], qy[j[has]] - qy[has]
                sx[has] += px; sy[has] += py
                sxx[has] += px * px; syy[has] += py * py; sxy[has] += px * py
                cnt[has] += 1.0
        m = np.maximum(cnt, 1.0)
        mx, my = sx / m, sy / m
        a = sxx / m - mx * mx
        c = syy / m - my * my
        b = sxy / m - mx * my
        # 2x2 対称行列 [[a,b],[b,c]] の固有値（閉形式）
        tr = a + c
        disc = np.sqrt(np.maximum(0.0, (a - c) ** 2 / 4.0 + b * b))
        lmax = tr / 2.0 + disc
        lmin = tr / 2.0 - disc
        # 最大固有値の固有ベクトル（接線）→ 90°回して法線
        tx = np.where(np.abs(b) > 1e-12, lmax - c, np.where(a >= c, 1.0, 0.0))
        ty = np.where(np.abs(b) > 1e-12, b, np.where(a >= c, 0.0, 1.0))
        tn = np.hypot(tx, ty)
        tn = np.where(tn > 1e-12, tn, 1.0)
        self._nx = (-ty / tn)
        self._ny = (tx / tn)
        self._planar = (cnt >= min_support) & (lmin <= planar_ratio * np.maximum(lmax, 1e-12))
        #: 近傍の裏付けがあるセルだけを対応に使う（docstring参照）
        self._support = cnt >= min_support
        self._qx, self._qy = qx, qy

        # ── 最寄りの壁セルの表 ──
        self._nearest = _nearest_occupied(occ, idx_map)        # (H, W) → 壁index or -1

    def associate(self, wx: np.ndarray, wy: np.ndarray) -> Association:
        """世界座標の点ごとに、最寄りの壁の面要素を引く。"""
        wx = np.asarray(wx, dtype=np.float64)
        wy = np.asarray(wy, dtype=np.float64)
        shape = wx.shape
        if self.empty:
            z = np.zeros(shape)
            return Association(z, z, z, z, np.zeros(shape, bool), np.zeros(shape, bool),
                               np.full(shape, np.inf))
        res = self.resolution
        c = np.floor((wx - self.origin[0]) / res).astype(np.int64)
        r = np.floor((wy - self.origin[1]) / res).astype(np.int64)
        ok = (c >= 0) & (c < self.width) & (r >= 0) & (r < self.height)
        cc = np.where(ok, c, 0)
        rr = np.where(ok, r, 0)
        j = np.where(ok, self._nearest[rr, cc], -1)
        if self.known is not None:
            ok &= self.known[rr, cc]
        ok &= j >= 0
        jj = np.where(ok, j, 0)
        qx, qy = self._qx[jj], self._qy[jj]
        nx, ny = self._nx[jj], self._ny[jj]
        planar = self._planar[jj]
        ok &= self._support[jj]
        ex, ey = wx - qx, wy - qy
        d = np.where(planar, np.abs(nx * ex + ny * ey), np.hypot(ex, ey))
        # ★ 面要素から**接線方向**にはみ出した点は対応させない。最寄りの壁セルが
        #   接線方向に1セル以上離れているのは「地図にある壁の端より先」に落ちた点で、
        #   その壁が先で曲がっているかどうかは地図に無い。対応させると壁を直線に
        #   延長した面へ引っ張られ、**曲がり角で必ず回頭を弱める向きに偏る**
        #   （`sim/slam_bench.py` normal: 回頭中に1周期0.015°ずつ回り不足になり、
        #   2周で向き4.7°。推測航法だけなら0.87°）
        tang = np.abs(-ny * ex + nx * ey)
        beyond = planar & (tang > self.tangent_gate)
        far = np.hypot(ex, ey) > self.max_dist
        valid = ok & ~far & ~beyond & (d < self.max_dist)
        return Association(qx, qy, nx, ny, planar, valid, np.where(valid, d, np.inf))

    def known_at(self, wx: np.ndarray, wy: np.ndarray) -> np.ndarray:
        res = self.resolution
        c = np.floor((np.asarray(wx) - self.origin[0]) / res).astype(np.int64)
        r = np.floor((np.asarray(wy) - self.origin[1]) / res).astype(np.int64)
        ok = (c >= 0) & (c < self.width) & (r >= 0) & (r < self.height)
        if self.known is None:
            return ok
        out = np.zeros(ok.shape, dtype=bool)
        out[ok] = self.known[r[ok], c[ok]]
        return out

    # ── 作り方 ──

    @classmethod
    def from_points(cls, xs: np.ndarray, ys: np.ndarray, *, center: tuple[float, float],
                    radius: float, resolution: float, max_dist: float = 0.4,
                    min_count: int = 1) -> "SurfaceMap":
        """世界座標の点群から、`center`の周り`radius`の正方形の面要素地図を作る。"""
        n = int(math.ceil(2 * radius / resolution))
        ox = math.floor((center[0] - radius) / resolution) * resolution
        oy = math.floor((center[1] - radius) / resolution) * resolution
        c = np.floor((xs - ox) / resolution).astype(np.int64)
        r = np.floor((ys - oy) / resolution).astype(np.int64)
        ok = (c >= 0) & (c < n) & (r >= 0) & (r < n)
        flat = r[ok] * n + c[ok]
        cnt = np.bincount(flat, minlength=n * n).reshape(n, n)
        sx = np.bincount(flat, weights=xs[ok], minlength=n * n).reshape(n, n)
        sy = np.bincount(flat, weights=ys[ok], minlength=n * n).reshape(n, n)
        occ = cnt >= min_count
        m = np.maximum(cnt, 1)
        return cls(occ, resolution=resolution, origin=(ox, oy), mean_x=sx / m, mean_y=sy / m,
                   max_dist=max_dist)


def _nearest_occupied(occ: np.ndarray, idx_map: np.ndarray) -> np.ndarray:
    """各セルについて最寄りの壁セルの index（`idx_map`の値）。"""
    if _cv2 is not None:
        src = np.where(occ, 0, 1).astype(np.uint8)
        _d, labels = _cv2.distanceTransformWithLabels(
            src, _cv2.DIST_L2, 5, labelType=_cv2.DIST_LABEL_PIXEL)
        zero = np.flatnonzero(occ.reshape(-1))
        lut = np.full(int(labels.max()) + 1, -1, dtype=np.int64)
        lut[labels.reshape(-1)[zero]] = idx_map.reshape(-1)[zero]
        return lut[labels]
    return _jump_flood(occ, idx_map)


def _jump_flood(occ: np.ndarray, idx_map: np.ndarray) -> np.ndarray:  # pragma: no cover
    """Jump Flooding による最寄り壁セルの表（cv2 が無い環境の予備）。"""
    h, w = occ.shape
    rows, cols = np.nonzero(occ)
    seed_r = np.full((h, w), -1, dtype=np.int64)
    seed_c = np.full((h, w), -1, dtype=np.int64)
    seed_r[rows, cols] = rows
    seed_c[rows, cols] = cols
    rr, cc = np.mgrid[0:h, 0:w]
    step = 1 << int(math.ceil(math.log2(max(h, w))))
    while step >= 1:
        for dr in (-step, 0, step):
            for dc in (-step, 0, step):
                if dr == 0 and dc == 0:
                    continue
                sr = np.full((h, w), -1, dtype=np.int64)
                sc = np.full((h, w), -1, dtype=np.int64)
                ys = slice(max(0, dr), h + min(0, dr))
                yd = slice(max(0, -dr), h + min(0, -dr))
                xs = slice(max(0, dc), w + min(0, dc))
                xd = slice(max(0, -dc), w + min(0, -dc))
                sr[yd, xd] = seed_r[ys, xs]
                sc[yd, xd] = seed_c[ys, xs]
                cand = sr >= 0
                d_new = np.where(cand, (sr - rr) ** 2 + (sc - cc) ** 2, np.iinfo(np.int64).max)
                d_old = np.where(seed_r >= 0, (seed_r - rr) ** 2 + (seed_c - cc) ** 2,
                                 np.iinfo(np.int64).max)
                better = d_new < d_old
                seed_r = np.where(better, sr, seed_r)
                seed_c = np.where(better, sc, seed_c)
        step //= 2
    out = np.full((h, w), -1, dtype=np.int64)
    ok = seed_r >= 0
    out[ok] = idx_map[seed_r[ok], seed_c[ok]]
    return out
