"""全域の自己位置探索 — 多解像度の分枝限定法（Cartographer の FastCorrelativeScanMatcher の考え方）。

`core/localize.py` の `GlobalLocalizer` は「空きセル 0.2m 間隔 × 15° 刻み」の総当たりを 1 周期 1 角度ずつ
進めるので、全周で最低 24 周期（10Hz で 2.4 秒）かかり、刻みの間に真の姿勢が落ちると候補に入らない。
こちらは地図全体・全周を **`step` [m]・`angle_step` 刻みで厳密に**探し、最良の候補を 1 回の呼び出しで返す。

## しくみ

尤度の格子 L（壁からの距離 d の exp(−d²/2σ²)、`step` 刻み）を、2^h 四方の窓の最大値で縮めた格子 P_h を
地図ごとに 1 度だけ作る。2^h 四方の平行移動の候補をまとめた「節」の得点の上界は Σ_k P_h[点 k のセル] なので、
上界が今の候補より低い節は子を見ない（`_native.c` の `bb_match`）。車が居られない所（既知の空きでない）を含まない
節も見ない。得点は `GlobalLocalizer` と同じく点ごとの尤度の平均。

## 返すもの

場所（`sep_m`）か向き（`sep_deg`）が離れた候補を上位 `k` 個（`LocalizeResult`）。呼び出し側は今までどおり
`Frontend.refine()` で仕上げて `pick_locate` で決める。候補は `step`・`angle_step` の精度なので、仕上げの窓は
狭くてよい（`REFINE_TRANS`・`REFINE_ROT`）。

C が組めない環境では使えない（`available()` が False）。呼び出し側は `GlobalLocalizer` に落ちる。
"""

from __future__ import annotations

import ctypes
import itertools
import math

import numpy as np

from .grid import OccGrid
from .localize import LocalizeResult
from .native import addr, lib
from .types import ScanPoints

__all__ = ["REFINE_ROT", "REFINE_TRANS", "GlobalMatcher", "MatchJob", "available"]

#: 候補を仕上げる窓（`Frontend.refine` の `trans`・`rot`）。探索の刻みの 2〜3 倍
REFINE_TRANS = 0.12
REFINE_ROT = math.radians(4.5)
#: 角度を分けて探すとき、前の塊までの最良のこの割合未満の候補は探さない（`MatchJob._floor`）
KEEP_RATIO = 0.5


def available() -> bool:
    return lib is not None and hasattr(lib, "bb_match")


def _maxpool_fwd(a: np.ndarray, size: int, pad: int) -> np.ndarray:
    """`out[r + pad, c + pad] = max(a[r:r+size, c:c+size])`（r, c は −pad 以上。はみ出しは 0）。"""
    h, w = a.shape
    p = np.zeros((h + 2 * pad, w + 2 * pad), dtype=a.dtype)
    p[pad:pad + h, pad:pad + w] = a
    # 縦横それぞれ、前向きの窓 [i, i+size) の最大（倍々に広げる）
    def run(x: np.ndarray, axis: int) -> np.ndarray:
        out = x.copy()
        span = 1
        while span < size:
            s = min(span, size - span)
            sh = np.zeros_like(out)
            if axis == 0:
                sh[:-s] = out[s:]
            else:
                sh[:, :-s] = out[:, s:]
            out = np.maximum(out, sh)
            span += s
        return out
    m = run(run(p, 0), 1)
    return m[:h + pad, :w + pad]


class GlobalMatcher:
    """1 つの凍結地図に対する探索器（前処理を持つので、地図が変わるまで使い回す）。"""

    def __init__(self, grid: OccGrid, *, step: float = 0.04, angle_step_deg: float = 1.0,
                 sigma: float = 0.08, levels: int = 7, max_points: int = 120,
                 max_range: float = 8.0) -> None:
        import cv2

        self.seq = grid.seq
        f = max(1, round(step / grid.resolution))
        self.step = f * grid.resolution
        self.origin = (float(grid.origin[0]), float(grid.origin[1]))
        wall = grid.wall_mask()
        dist = cv2.distanceTransform((~wall).astype(np.uint8), cv2.DIST_L2, 5) * grid.resolution
        like = np.exp(-(dist.astype(np.float64) ** 2) / (2.0 * sigma * sigma)).astype(np.float32)
        free = grid.known_free_mask()
        h0, w0 = like.shape
        H, W = (h0 + f - 1) // f, (w0 + f - 1) // f
        # 粗い格子へ: 尤度は窓の最大（上界を保つ）・車が居られるかは窓のどれか
        lp = np.zeros((H * f, W * f), dtype=np.float32)
        lp[:h0, :w0] = like
        fp = np.zeros((H * f, W * f), dtype=bool)
        fp[:h0, :w0] = free
        L = lp.reshape(H, f, W, f).max(axis=(1, 3))
        F = fp.reshape(H, f, W, f).any(axis=(1, 3)).astype(np.uint8)
        self.H, self.W = H, W
        self.levels = max(1, min(levels, math.ceil(math.log2(max(H, W))) + 1))
        pyr, frees, offs = [], [], []
        off = 0
        for hlev in range(self.levels):
            size = 1 << hlev
            pad = size - 1
            pyr.append(_maxpool_fwd(L, size, pad).ravel())
            frees.append(_maxpool_fwd(F, size, pad).ravel())
            offs.append(off)
            off += pyr[-1].size
        self._pyr = np.ascontiguousarray(np.concatenate(pyr), dtype=np.float32)
        self._free = np.ascontiguousarray(np.concatenate(frees), dtype=np.uint8)
        self._off = np.asarray(offs, dtype=np.int64)
        n_ang = max(1, round(360.0 / angle_step_deg))
        self.angles = np.linspace(-math.pi, math.pi, n_ang, endpoint=False)
        self.max_points = max_points
        self.max_range = max_range
        self.last_visited = 0

    def _offsets(self, pts: ScanPoints) -> tuple[np.ndarray, int]:
        sx, sy = pts.x[pts.hit], pts.y[pts.hit]
        keep = np.hypot(sx, sy) <= self.max_range
        sx, sy = sx[keep], sy[keep]
        if sx.size > self.max_points:
            take = np.linspace(0, sx.size - 1, self.max_points).astype(np.int64)
            sx, sy = sx[take], sy[take]
        c, s = np.cos(self.angles)[:, None], np.sin(self.angles)[:, None]
        rx = (c * sx[None, :] - s * sy[None, :]) / self.step
        ry = (s * sx[None, :] + c * sy[None, :]) / self.step
        off = np.stack([np.floor(rx + 0.5), np.floor(ry + 0.5)], axis=-1).astype(np.int32)
        return np.ascontiguousarray(off), int(sx.size)

    def begin(self, pts: ScanPoints, k: int = 12, *, hint: tuple[float, float] | None = None,
              radius: float | None = None, min_score: float = 0.2, sep_m: float = 1.0,
              sep_deg: float = 60.0, chunks: int = 1) -> MatchJob:
        """探索を始める。`chunks` に分けた角度を `MatchJob.step()` 1 回に 1 つずつ探す（Pi で 1 周期に収める）。"""
        return MatchJob(self, pts, k, hint, radius, min_score, sep_m, sep_deg, chunks)

    def match(self, pts: ScanPoints, k: int = 12, **kw) -> list[LocalizeResult]:
        """点群（車体基準）の全域の候補を、得点の高い順に最大 `k` 個（1 回で全部探す）。"""
        job = self.begin(pts, k, **kw)
        while not job.step():
            pass
        return job.result()

    def score_at(self, pts: ScanPoints, pose: tuple[float, float, float]) -> float:
        """同じ物差し（L の平均）で、決まった姿勢の得点（今の姿勢と全域の最良を比べる見張り用）。"""
        sx, sy = pts.x[pts.hit], pts.y[pts.hit]
        keep = np.hypot(sx, sy) <= self.max_range
        sx, sy = sx[keep], sy[keep]
        if sx.size == 0:
            return 0.0
        if sx.size > self.max_points:
            take = np.linspace(0, sx.size - 1, self.max_points).astype(np.int64)
            sx, sy = sx[take], sy[take]
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        wx, wy = x + c * sx - s * sy, y + s * sx + c * sy
        col = np.floor((wx - self.origin[0]) / self.step).astype(np.int64)
        row = np.floor((wy - self.origin[1]) / self.step).astype(np.int64)
        ok = (col >= 0) & (row >= 0) & (col < self.W) & (row < self.H)
        L = self._pyr[:self.H * self.W].reshape(self.H, self.W)
        return float(L[row[ok], col[ok]].sum() / sx.size)


class MatchJob:
    """`GlobalMatcher.begin` が返す探索の途中経過。点群は始めたときのものを使い続ける（車は止めておく）。"""

    def __init__(self, gm: GlobalMatcher, pts: ScanPoints, k: int, hint, radius, min_score: float,
                 sep_m: float, sep_deg: float, chunks: int) -> None:
        self.gm = gm
        self.off, self.n = gm._offsets(pts)
        self.k = int(k)
        self.use_hint = hint is not None and radius is not None
        self.hx = (hint[0] - gm.origin[0]) / gm.step - 0.5 if self.use_hint else 0.0
        self.hy = (hint[1] - gm.origin[1]) / gm.step - 0.5 if self.use_hint else 0.0
        self.hr = (radius / gm.step) if self.use_hint else 0.0
        self.min_sum = float(min_score * self.n)
        self.sep_cells = sep_m / gm.step
        self.sep_ang = math.radians(sep_deg)
        na = len(gm.angles)
        cuts = np.linspace(0, na, max(1, int(chunks)) + 1).round().astype(int)
        self._ranges = [(int(a), int(b)) for a, b in itertools.pairwise(cuts) if b > a]
        self._i = 0
        self._rows: list[list[float]] = []
        self.visited = 0
        self._ang = np.ascontiguousarray(gm.angles, dtype=np.float64)
        self.done = self.n == 0

    @property
    def progress(self) -> float:
        return self._i / max(1, len(self._ranges))

    def step(self) -> bool:
        """角度の塊を 1 つ探す。全部終わったら True。"""
        if self.done:
            return True
        a0, a1 = self._ranges[self._i]
        vis = np.zeros(1, dtype=np.int64)
        gm = self.gm
        # 塊ごとに独立に上位を出し、最後に点数順でまとめる（`result`）。C の中の重複のまとめは探す順に
        # 依存するので、前の塊の候補を持ち越すと 2 位以下が一括の探索と変わる（1 位は同じ）
        out = np.zeros((self.k, 4), dtype=np.float64)
        cnt = lib.bb_match(gm.H, gm.W, gm.levels, addr(gm._pyr), addr(gm._off), addr(gm._free),
                           len(gm.angles), addr(self._ang), self.n, addr(self.off),
                           ctypes.c_int(1 if self.use_hint else 0), self.hx, self.hy, self.hr,
                           self._floor(), self.k, self.sep_cells, self.sep_ang, 0,
                           a0, a1, 0, addr(out), addr(vis))
        if cnt > 0:
            self._rows.extend(out[:cnt].tolist())
        self.visited += int(vis[0])
        gm.last_visited = self.visited
        self._i += 1
        self.done = self._i >= len(self._ranges)
        return self.done

    def _floor(self) -> float:
        """この塊の足切り（点数の和）。これまでの最良の `KEEP_RATIO` 倍未満の候補は、曖昧さの判定（2 位が 1 位の
        7 割以上か）にも関わらないので探さない。前の塊で良い候補が見つかっていれば枝刈りが早く効く。"""
        best = max((r[0] for r in self._rows), default=0.0)
        return max(self.min_sum, KEEP_RATIO * best)

    def result(self) -> list[LocalizeResult]:
        """点数の高い順に、場所か向きが離れたものだけを最大 `k` 個（決定的なまとめ）。"""
        gm = self.gm
        picked: list[list[float]] = []
        for r in sorted(self._rows, key=lambda r: (-r[0], r[3], r[2], r[1])):
            a = gm.angles[int(r[3])]
            if any((r[1] - q[1]) ** 2 + (r[2] - q[2]) ** 2 < self.sep_cells ** 2
                   and abs((gm.angles[int(q[3])] - a + math.pi) % (2.0 * math.pi) - math.pi) < self.sep_ang
                   for q in picked):
                continue
            picked.append(r)
            if len(picked) >= self.k:
                break
        return [LocalizeResult(gm.origin[0] + (cx + 0.5) * gm.step, gm.origin[1] + (cy + 0.5) * gm.step,
                               float(gm.angles[int(ai)]), s / self.n, False)
                for s, cx, cy, ai in picked]
