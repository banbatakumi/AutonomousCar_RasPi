"""画像（コース写真・図・PNG）→ 占有格子コース。`sim/editor.py` の「画像」モードの中身。

pygame を持ち込まない純粋な画像処理だけを置く（テストできるように分けてある）。

## パイプライン

    色で壁を拾う → 穴埋め（管の縞・影の隙間を閉じる） → 小さい塊を捨てる
                 → 手動の編集（ブロブ削除・矩形で消す・矩形で塗る）を順に適用

**自動処理の後に手動編集を重ねる。** 手動編集は「画像のピクセル座標」で記録するので、
色の許容や穴埋めを後から変えても、消した場所は消えたまま残る。

## 座標

画像の左下を世界座標の `origin` に置き、`1 px = 1/scale [m]`。行は上下反転する
（画像は行0が上、世界は y が上に増える）。**切り出しは保存時だけ**行う——編集中に
外接矩形で基準を動かすと、ノイズを消すたびにスタートや障害物の座標がずれる。
"""

from __future__ import annotations

import colorsys
from dataclasses import dataclass

import numpy as np
from scipy import ndimage as ndi

__all__ = ["Op", "guess_wall_color", "color_mask", "build_mask", "sample_grid",
           "crop_for_save", "load_rgb"]

RES = 0.02
MARGIN = 0.4

#: 手動編集の1操作。("blob", x, y) / ("erase", x0, y0, x1, y1) / ("fill", x0, y0, x1, y1)
#: （座標は画像のピクセル、x=列・y=行）
Op = tuple


def load_rgb(path) -> np.ndarray:
    """画像ファイル → `(H, W, 3)` uint8。透過PNGは白背景に載せる。"""
    from PIL import Image

    im = Image.open(path)
    if im.mode in ("RGBA", "LA", "P"):
        im = im.convert("RGBA")
        bg = Image.new("RGBA", im.size, (255, 255, 255, 255))
        im = Image.alpha_composite(bg, im)
    return np.asarray(im.convert("RGB"), dtype=np.uint8)


def _is_binary(rgb: np.ndarray) -> bool:
    """白黒（に近い）画像か。壁=黒・走行可=白のこのリポジトリの PNG 規約。"""
    lum = rgb.mean(axis=2)
    return float(((lum < 60) | (lum > 195)).mean()) > 0.9 and \
        float((rgb.max(axis=2).astype(int) - rgb.min(axis=2)).mean()) < 12


def guess_wall_color(rgb: np.ndarray) -> tuple[int, int, int]:
    """壁の色の当て推量。外れたら人がスポイトで直す前提の初期値。

    - 白黒画像: 暗い方が壁（`sim/courses/*.png` と同じ規約）
    - カラー: 彩度の高い色ほど、数が多いほど壁らしいとみなす（背景と床は普通は
      くすんだ色で、壁は目立つ色で描かれる）
    """
    if _is_binary(rgb):
        return (0, 0, 0)
    sub = rgb[::2, ::2]
    q = (sub >> 4).reshape(-1, 3).astype(np.int32)                 # 16 階調に丸める
    key = (q[:, 0] << 8) | (q[:, 1] << 4) | q[:, 2]
    vals, counts = np.unique(key, return_counts=True)
    total = float(len(key))
    cands = []
    for v, c in zip(vals, counts):
        if c < total * 0.003:
            continue
        r, g, b = ((v >> 8) & 15) * 16 + 8, ((v >> 4) & 15) * 16 + 8, (v & 15) * 16 + 8
        _h, s, val = colorsys.rgb_to_hsv(r / 255, g / 255, b / 255)
        if val < 0.25:
            continue
        cands.append((s ** 3 * c, (r, g, b)))
    cands.sort(reverse=True)
    best, best_score = (128, 128, 128), -1.0
    for score, ref in cands[:8]:
        near = np.abs(sub.astype(np.float64) - np.array(ref)).max(axis=2) <= 16
        # 壁は細長い（管・板）、床や背景は広い。広い塊に属する画素が多い色は壁ではない
        # （距離変換が大きい画素の割合で測る。サブサンプル済みなので 12 ≒ 元の 24px）
        blob = float((ndi.distance_transform_edt(near) > 12).sum()) / max(1, int(near.sum()))
        score *= max(0.0, 1.0 - blob * 2.0)
        if score > best_score and near.any():
            best, best_score = tuple(int(round(v)) for v in sub[near].mean(axis=0)), score
    return best


def _rgb_to_hsv(rgb: np.ndarray):
    x = rgb.astype(np.float32) / 255.0
    mx, mn = x.max(axis=2), x.min(axis=2)
    d = mx - mn
    s = np.where(mx > 1e-6, d / np.maximum(mx, 1e-6), 0.0)
    r, g, b = x[..., 0], x[..., 1], x[..., 2]
    dd = np.maximum(d, 1e-6)
    h = np.where(mx == r, ((g - b) / dd) % 6,
                 np.where(mx == g, (b - r) / dd + 2, (r - g) / dd + 4)) * 60.0
    return h, s, mx


def color_mask(rgb: np.ndarray, ref: tuple[int, int, int], tol: int) -> np.ndarray:
    """`ref` に近い色のピクセル。`tol` は 0〜100 の許容。

    鮮やかな色（管のように陰影で明るさが変わるもの）は**色相**で拾い、明るさの差は
    大目に見る。無彩色（黒い壁・灰色）は明度で拾う。RGB の距離だと、同じ管でも
    陰の側が拾えず縞状に抜ける。
    """
    h, s, v = _rgb_to_hsv(rgb)
    rh, rs, rv = colorsys.rgb_to_hsv(ref[0] / 255, ref[1] / 255, ref[2] / 255)
    if rs >= 0.25:
        dh = np.abs((h - rh * 360.0 + 180.0) % 360.0 - 180.0)
        return (dh <= 4.0 + tol * 0.3) & (s >= rs * (0.75 - tol * 0.004)) & (v >= 0.25)
    lum = rgb.astype(np.float32).mean(axis=2)
    rl = float(np.mean(ref))
    return (np.abs(lum - rl) <= 8.0 + tol * 1.2) & (s <= 0.3)


def _apply_op(mask: np.ndarray, op: Op) -> np.ndarray:
    h, w = mask.shape
    kind = op[0]
    if kind == "blob":
        _, x, y = op
        x, y = int(round(x)), int(round(y))
        lab, n = ndi.label(mask, structure=np.ones((3, 3), dtype=int))
        if n == 0:
            return mask
        # クリックが壁の縁を少し外れても拾えるよう、近傍で最初に当たる塊を使う
        r = 6
        win = lab[max(0, y - r):min(h, y + r + 1), max(0, x - r):min(w, x + r + 1)]
        ids = win[win > 0]
        if ids.size == 0:
            return mask
        # 中心に最も近い画素の塊
        yy, xx = np.nonzero(win > 0)
        cy, cx = y - max(0, y - r), x - max(0, x - r)
        k = int(np.argmin((yy - cy) ** 2 + (xx - cx) ** 2))
        out = mask.copy()
        out[lab == win[yy[k], xx[k]]] = False
        return out
    x0, y0, x1, y1 = (int(round(v)) for v in op[1:5])
    x0, x1 = sorted((max(0, x0), min(w, x1 + 1)))
    y0, y1 = sorted((max(0, y0), min(h, y1 + 1)))
    out = mask.copy()
    out[y0:y1, x0:x1] = (kind == "fill")
    return out


def build_mask(rgb: np.ndarray, ref: tuple[int, int, int], tol: int, close_px: int,
               min_area_px: float, ops: list[Op]) -> np.ndarray:
    """壁マスク `(H, W)` bool を作る（パイプライン全体）。"""
    m = color_mask(rgb, ref, tol)
    if close_px > 0:
        k = np.ones((2 * close_px + 1, 2 * close_px + 1), dtype=bool)
        m = ndi.binary_closing(m, k)
    if min_area_px > 1:
        lab, n = ndi.label(m, structure=np.ones((3, 3), dtype=int))
        if n:
            sizes = ndi.sum(m, lab, index=np.arange(1, n + 1))
            keep = np.concatenate(([False], sizes >= min_area_px))
            m = keep[lab]
    for op in ops:
        m = _apply_op(m, op)
    return m


@dataclass
class Sampled:
    grid: np.ndarray                 #: bool (行0 = y 最小)
    origin: tuple[float, float]


def sample_grid(mask: np.ndarray, scale: float, origin: tuple[float, float] = (0.0, 0.0),
                res: float = RES) -> Sampled:
    """画像マスクを世界座標の占有格子（`res` m/セル）へ最近傍サンプリングする。

    格子の範囲は画像全体。`origin` は画像の左下の世界座標。

    1セルが1画素より大きい（縮小する）ときは、セルが覆う画素に1つでも壁があれば壁にする
    （最近傍だと、セルの中心を外れた細い壁が消える）。
    """
    h, w = mask.shape
    gw = max(1, int(np.ceil(w / scale / res)))
    gh = max(1, int(np.ceil(h / scale / res)))
    step = res * scale                       # 1セルが覆う画素数
    if step > 1.0:
        flipped = mask[::-1].astype(np.int32)       # 行0 = y 最小

        def spans(n_cells, n_px):
            a = np.minimum((np.arange(n_cells) * step).astype(int), n_px - 1)
            b = np.minimum(np.maximum(np.ceil((np.arange(n_cells) + 1) * step).astype(int), a + 1),
                           n_px)
            return a, b

        r0, r1 = spans(gh, h)
        c0, c1 = spans(gw, w)
        ii = np.zeros((h + 1, w + 1), dtype=np.int64)
        ii[1:, 1:] = flipped.cumsum(0).cumsum(1)
        cnt = (ii[r1[:, None], c1[None, :]] - ii[r0[:, None], c1[None, :]]
               - ii[r1[:, None], c0[None, :]] + ii[r0[:, None], c0[None, :]])
        return Sampled(np.ascontiguousarray(cnt > 0), origin)
    cols = np.clip(((np.arange(gw) + 0.5) * res * scale).astype(int), 0, w - 1)
    rows = np.clip(h - 1 - ((np.arange(gh) + 0.5) * res * scale).astype(int), 0, h - 1)
    return Sampled(np.ascontiguousarray(mask[rows[:, None], cols[None, :]]), origin)


def crop_for_save(grid: np.ndarray, origin: tuple[float, float], res: float = RES,
                  margin: float = MARGIN):
    """壁の外接矩形 + `margin` で切り出す。返り値 `(grid, shift)`。

    `shift` は元の世界座標から新しい世界座標（新 origin = (0,0)）への引き算量:
    `new = old - shift`。スタートや障害物の座標に同じだけ適用する。
    壁が1セルも無ければ `ValueError`。
    """
    ys, xs = np.nonzero(grid)
    if ys.size == 0:
        raise ValueError("壁がありません")
    pad = int(round(margin / res))
    r0, r1, c0, c1 = ys.min(), ys.max(), xs.min(), xs.max()
    g = np.pad(grid, pad, constant_values=False)
    out = g[r0:r1 + 2 * pad + 1, c0:c1 + 2 * pad + 1]
    shift = (origin[0] + (c0 - pad) * res, origin[1] + (r0 - pad) * res)
    return np.ascontiguousarray(out), shift
