"""シミュレータの画面部品 — `sim/gui.py` と `sim/editor.py` の共通部分。

**地図の描画をここに置くのが一番の目的。** 占有格子の再着色・壁の縁・1m グリッド・
中心線は、俯瞰ビューとエディタで**同じに見えないと意味がない**（エディタで作った物が
走らせたときと違って見えたら、エディタを信用できなくなる）。2箇所にコピーすると必ずずれる。

見た目の方針は `sim/gui.py` の docstring 参照。要点は
「PNG を生で貼らない」「アンチエイリアスを使う」「余白と字を規則に乗せる」の3つ。
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pygame
import pygame.gfxdraw

__all__ = ["Ui", "map_surface", "make_dot", "shift",
           "BG", "PANEL", "PANEL2", "LINE", "HAIR", "FG", "DIM", "FAINT",
           "ACCENT", "OK", "WARN", "BAD", "POINT", "TRAIL", "PAD"]

PAD = 20

BG      = (14, 17, 22)
PANEL   = (21, 26, 33)
PANEL2  = (27, 33, 41)
LINE    = (38, 46, 56)
HAIR    = (31, 38, 46)
FG      = (230, 235, 242)
DIM     = (124, 136, 153)
FAINT   = (78, 88, 102)
ACCENT  = (224, 170, 42)
OK      = (78, 201, 138)
WARN    = (232, 168, 60)
BAD     = (226, 88, 78)
POINT   = (90, 209, 240)
TRAIL   = (61, 127, 196)

#: 走行可能面を背景より少し明るい「床」にし、壁は沈めて縁だけ光らせる。
#: 逆にすると（壁を明るくすると）走れる場所が背景に溶けて、コースに見えない
MAP_FREE = (27, 34, 43)
MAP_WALL = (16, 20, 26)
MAP_EDGE = (72, 90, 112)
MAP_GRID = (255, 255, 255, 12)
MAP_CENTER = (120, 140, 165, 70)

_UI = ["/System/Library/Fonts/ヒラギノ角ゴシック W4.ttc",
       "/System/Library/Fonts/Supplemental/Arial Unicode.ttf",
       "/System/Library/Fonts/AppleSDGothicNeo.ttc"]
_UI_BOLD = ["/System/Library/Fonts/ヒラギノ角ゴシック W6.ttc", *_UI]
#: 数字専用。**桁が揃わないと計器に見えない。**
#: ただし日本語のグリフが無い（`回` が豆腐になった）ので、単位はラベル側に置くこと
_MONO = ["/System/Library/Fonts/Menlo.ttc",
         "/System/Library/Fonts/Supplemental/Courier New.ttf"]


def _font(cands: list[str], size: int):
    for p in cands:
        if Path(p).exists():
            return pygame.font.Font(p, size)
    return pygame.font.Font(None, size)


class Ui:
    """フォント一式と、繰り返し使う小さな描画。**pygame.init() の後に作る。**"""

    def __init__(self) -> None:
        self.lbl = _font(_UI, 13)
        self.sm = _font(_UI, 11)
        self.hd = _font(_UI_BOLD, 15)
        self.num = _font(_MONO, 14)
        self.big = _font(_MONO, 30)
        self.pill_f = _font(_UI_BOLD, 12)
        self._track: dict[tuple, pygame.Surface] = {}

    def tracked(self, text: str, color, size: int = 11, sp: int = 2) -> pygame.Surface:
        """字間を空けた見出し。**小さい大文字は字間を空けないと詰まって見える。**"""
        key = (text, color, size, sp)
        if key not in self._track:
            f = _font(_UI_BOLD, size)
            glyphs = [f.render(c, True, color) for c in text]
            w = sum(g.get_width() for g in glyphs) + sp * (len(glyphs) - 1)
            s = pygame.Surface((max(1, w), f.get_height()), pygame.SRCALPHA)
            x = 0
            for g in glyphs:
                s.blit(g, (x, 0))
                x += g.get_width() + sp
            self._track[key] = s
        return self._track[key]

    def section(self, screen, text: str, x: int, y: int, inner: int) -> int:
        g = self.tracked(text, FAINT, 10, 3)
        screen.blit(g, (x, y))
        ly = y + g.get_height() // 2
        pygame.draw.line(screen, HAIR, (x + g.get_width() + 10, ly), (x + inner, ly))
        return y + 22

    def kv(self, screen, x: int, y: int, inner: int, label: str, value: str,
           col=FG, mono: bool = True) -> int:
        """左にラベル（UI フォント）、右に値（既定は等幅）。

        **等幅（Menlo）には日本語が無い。** 数字は桁を揃えたいので等幅にするが、
        値に日本語が入るときは `mono=False` にすること（`回` `閉じた` が豆腐になった）。
        単位は原則ラベル側に置く（`衝突 [回]`）。
        """
        screen.blit(self.lbl.render(label, True, DIM), (x, y))
        g = (self.num if mono else self.lbl).render(value, True, col)
        screen.blit(g, (x + inner - g.get_width(), y))
        return y + 22

    def pill(self, screen, text: str, color, x: int, y: int) -> int:
        """枠だけのピル。塗ると強すぎて、並べたときに全部が主役になる。"""
        g = self.pill_f.render(text, True, color)
        w, h = g.get_width() + 20, 22
        # **半透明はアルファ面に描いてから blit する。** 画面に直接 draw.rect すると
        # アルファが無視されてベタ塗りになり、ピルが全部主張してしまう
        s = pygame.Surface((w, h), pygame.SRCALPHA)
        pygame.draw.rect(s, (*color, 26), s.get_rect(), border_radius=h // 2)
        pygame.draw.rect(s, color, s.get_rect(), 1, border_radius=h // 2)
        s.blit(g, (10, (h - g.get_height()) // 2))
        screen.blit(s, (x, y))
        return x + w

    def button(self, screen, rect: pygame.Rect, text: str, *,
               on: bool = False, hover: bool = False, color=None) -> None:
        """パレットのボタン。選択中は枠と字を accent に。

        :param color: 常にこの色で描く（E-STOP のように意味が固定のもの）
        """
        col = color or (ACCENT if on else (FG if hover else DIM))
        bg = PANEL2 if (on or hover) else PANEL
        pygame.draw.rect(screen, bg, rect, border_radius=5)
        pygame.draw.rect(screen, col if on else LINE, rect, 1, border_radius=5)
        g = self.lbl.render(text, True, col)
        screen.blit(g, (rect.centerx - g.get_width() // 2,
                        rect.centery - g.get_height() // 2))


# ── 地図 ────────────────────────────────────────────────────────────────

_MAP_CACHE: dict[tuple, pygame.Surface] = {}


def map_surface(course, dw: int, dh: int, key: str | None = None) -> pygame.Surface:
    """占有格子から**作り直した**地図。PNG を生で貼らない。

    :param key: キャッシュの鍵。エディタのように同じ `path` で中身が変わる場合に渡す
    """
    k = (key or str(course.path), dw, dh)
    if k in _MAP_CACHE:
        return _MAP_CACHE[k]
    if len(_MAP_CACHE) > 24:              # 際限なく増やさない（リサイズで作り直すため）
        _MAP_CACHE.clear()

    g = course.grid
    free = ~g
    # 壁のうち、走行可能セルに接しているものだけを縁として明るくする。
    # これがあるだけで「塗り」ではなく「構造物」に見える
    edge = g & (shift(free, 1, 0) | shift(free, -1, 0)
                | shift(free, 0, 1) | shift(free, 0, -1))
    rgb = np.empty((*g.shape, 3), dtype=np.uint8)
    rgb[free] = MAP_FREE
    rgb[g] = MAP_WALL
    rgb[edge] = MAP_EDGE
    # 格子は行0 = y 最小。画面は上が y 最大なので上下反転してから (W,H,3) に
    surf = pygame.surfarray.make_surface(np.transpose(rgb[::-1], (1, 0, 2)))
    surf = pygame.transform.smoothscale(surf, (max(1, dw), max(1, dh)))

    ov = pygame.Surface((max(1, dw), max(1, dh)), pygame.SRCALPHA)
    cw, chh = course.size_m
    ox, oy = course.origin
    # **世界座標の整数メートルに引く。** 画像の左端から 1m ごとに引くと、
    # 画像の原点（外接矩形から決まる半端な値）ぶんずれて、コースの線と一致しない。
    # エディタで 1m の直線を置いたのにグリッドに乗らない、という気持ち悪さの正体
    for m in range(math.ceil(ox), math.floor(ox + cw) + 1):
        x = (m - ox) / cw * dw
        pygame.draw.line(ov, MAP_GRID, (x, 0), (x, dh))
    for m in range(math.ceil(oy), math.floor(oy + chh) + 1):
        y = dh - (m - oy) / chh * dh
        pygame.draw.line(ov, MAP_GRID, (0, y), (dw, y))
    # センターライン方式なら中心線を薄く。**これは経路追従の正解データそのもの**
    if getattr(course, "centerline", None) is not None and len(course.centerline) > 2:
        pts = [((x - course.origin[0]) / cw * dw,
                dh - (y - course.origin[1]) / chh * dh)
               for x, y, _ in course.centerline]
        pygame.draw.aalines(ov, MAP_CENTER, False, pts)
    surf.blit(ov, (0, 0))

    _MAP_CACHE[k] = surf
    return surf


def clear_map_cache() -> None:
    _MAP_CACHE.clear()


# ── 小物 ────────────────────────────────────────────────────────────────

def shift(a: np.ndarray, dr: int, dc: int) -> np.ndarray:
    """境界を False で埋めるシフト（`np.roll` だと反対側が回り込む）。"""
    out = np.zeros_like(a)
    r0, r1 = max(dr, 0), a.shape[0] + min(dr, 0)
    c0, c1 = max(dc, 0), a.shape[1] + min(dc, 0)
    out[r0:r1, c0:c1] = a[r0 - dr:r1 - dr, c0 - dc:c1 - dc]
    return out


def make_dot(color, r: int = 3) -> pygame.Surface:
    """中心が明るく外側が消えていく点。**1px の set_at より遥かに読みやすい。**"""
    d = r * 2 + 1
    s = pygame.Surface((d, d), pygame.SRCALPHA)
    for rad, a in ((r, 40), (r - 1, 110), (0, 255)):
        pygame.gfxdraw.filled_circle(s, r, r, max(rad, 0), (*color, a))
    return s


def arrow(screen, x: float, y: float, yaw: float, size: float, color) -> None:
    """姿勢マーカー。**向きが分からないと「どこから走り出すか」が読めない。**"""
    tip = (x + size * math.cos(yaw), y - size * math.sin(yaw))
    left = (x + size * 0.45 * math.cos(yaw + 2.4), y - size * 0.45 * math.sin(yaw + 2.4))
    right = (x + size * 0.45 * math.cos(yaw - 2.4), y - size * 0.45 * math.sin(yaw - 2.4))
    pts = [(int(a), int(b)) for a, b in (tip, left, (x, y), right)]
    pygame.gfxdraw.filled_polygon(screen, pts, (*color, 200))
    pygame.gfxdraw.aapolygon(screen, pts, (*color, 255))
