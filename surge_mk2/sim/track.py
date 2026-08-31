"""センターライン方式のコース生成 — 直線と円弧をつないで走路を作る。

    sim/courses/mytrack.json
    {
      "name": "マイコース",
      "width": 1.0,                 // 道幅 [m]
      "resolution": 0.02,           // 格子の刻み [m/px]
      "origin": [1.0, 1.0, 0.0],    // 中心線の始点 x, y, 向き[rad]
      "loop": true,
      "path": [["straight", 3.0], ["arc", 1.2, 90], ...]
    }

`["arc", 半径[m], 角度[deg]]` の角度は**反時計回りが正**（`architecture.md` §5.1 の規約。
左旋回が正）。`["straight", 長さ[m]]`。

## なぜ壁ではなく中心線を定義するのか

壁を1本ずつ置くと、左右を別々に置くことになるので

- 道幅がばらつく／継ぎ目に隙間ができる
- スタート姿勢を人間が決めるので**壁に埋まる**（実際にやった）
- 経路追従の評価に使える中心線が手に入らない

中心線＋道幅なら、左右の壁は常に平行に生成され、スタートは中心線上に置けるので
埋まりようがない。**中心線そのものが Phase 3 の経路追従の正解データになる。**

## ラスタライズは「全部壁から掘る」

格子を全部壁で埋めてから、中心線に沿って半径 `width/2` の円盤で削る。
左右の壁を別々に描くより単純で、**継ぎ目や自己交差（8の字）が自然に処理される。**
"""

from __future__ import annotations

import math

import numpy as np

__all__ = ["centerline", "rasterize", "add_offset_discs", "build"]

#: 中心線を刻む間隔を解像度の何倍にするか。円盤の半径より十分細かければよい
_STEP_RATIO = 1.0


def centerline(path: list, x: float, y: float, yaw: float,
               step: float) -> np.ndarray:
    """区間の並び → 中心線の点列 `(N, 3)`（x, y, 向き）。"""
    pts = [(x, y, yaw)]
    for seg in path:
        kind = seg[0]
        if kind == "straight":
            length = float(seg[1])
            n = max(1, int(round(abs(length) / step)))
            dx, dy = math.cos(yaw), math.sin(yaw)
            for i in range(1, n + 1):
                d = length * i / n
                pts.append((x + dx * d, y + dy * d, yaw))
            x, y = x + dx * length, y + dy * length

        elif kind == "arc":
            r = float(seg[1])
            deg = float(seg[2])
            if r <= 0:
                raise ValueError(f"arc の半径は正の値: {seg}")
            rad = math.radians(deg)
            s = 1.0 if deg >= 0 else -1.0
            # 曲率中心は進行方向の左（左旋回のとき）。右旋回なら反対側
            cx = x - s * r * math.sin(yaw)
            cy = y + s * r * math.cos(yaw)
            a0 = yaw - s * math.pi / 2
            n = max(1, int(round(abs(rad) * r / step)))
            for i in range(1, n + 1):
                phi = rad * i / n
                pts.append((cx + r * math.cos(a0 + phi),
                            cy + r * math.sin(a0 + phi),
                            yaw + phi))
            x = cx + r * math.cos(a0 + rad)
            y = cy + r * math.sin(a0 + rad)
            yaw = yaw + rad

        else:
            raise ValueError(f"未知の区間: {seg!r}（straight か arc）")

    return np.asarray(pts, dtype=np.float64)


def rasterize(pts: np.ndarray, width: float, resolution: float,
              margin: float = 0.4, divider: list | None = None,
              divider_width: float = 0.06) -> tuple[np.ndarray, tuple[float, float]]:
    """中心線 → 占有格子。**全部壁で埋めてから円盤で掘る。**

    :param divider: 分離帯を置く区間の並び `[[s0, s1], ...]`（中心線に沿った
        弧長 [m]）。コースを平行に2分割する薄い壁を、掘ったあとに立て直す
    :param divider_width: 分離帯の厚み [m]

    :returns: (grid, origin)。grid は bool で True = 壁。行0 が y 最小
        （`sim/course.py` の規約に合わせる）

    ## 分離帯は「掘ってから立てる」

    先に立ててから掘ると円盤に削り取られる。順序を逆にすることで、
    **道幅ぶん掘った中に細い壁が残る**（＝車線が2本になる）。
    """
    half = width / 2.0
    xs, ys = pts[:, 0], pts[:, 1]
    pad = half + margin
    x0, y0 = xs.min() - pad, ys.min() - pad
    w = int(math.ceil((xs.max() + pad - x0) / resolution))
    h = int(math.ceil((ys.max() + pad - y0) / resolution))

    grid = np.ones((h, w), dtype=bool)
    r = max(1, int(round(half / resolution)))
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    disc = (xx * xx + yy * yy) <= r * r

    cols = np.round((xs - x0) / resolution).astype(np.int64)
    rows = np.round((ys - y0) / resolution).astype(np.int64)
    for c, rw in zip(cols, rows):
        grid[rw - r:rw + r + 1, c - r:c + r + 1] &= ~disc

    if divider:
        # 中心線に沿った弧長。**始点からの距離**で区間を指定する
        seg = np.hypot(*np.diff(pts[:, :2], axis=0).T)
        arc = np.concatenate([[0.0], np.cumsum(seg)])
        dr = max(1, int(round(divider_width / 2.0 / resolution)))
        yy, xx = np.mgrid[-dr:dr + 1, -dr:dr + 1]
        dot = (xx * xx + yy * yy) <= dr * dr
        for a, b in divider:
            m = (arc >= float(a)) & (arc <= float(b))
            for c, rw in zip(cols[m], rows[m]):
                grid[rw - dr:rw + dr + 1, c - dr:c + dr + 1] |= dot
    return grid, (x0, y0)


def add_offset_discs(grid: np.ndarray, origin: tuple[float, float], resolution: float,
                     pts: np.ndarray, spans: list[tuple[float, float, float, float]]) -> None:
    """占有格子に、中心線から法線方向へオフセットした円盤を追加でOR演算する（in-place）。

    `rasterize()`の`divider`（上の`rasterize()`docstring参照）と同じ「掘ったあとに
    立てる」ロジックを、**中心線上に限らず任意の横オフセット位置**に置けるよう
    一般化したもの。`sim/random_course.py`の`narrow`（両側対称にオフセットして局所的に
    道幅を狭める）・`obstacle`（片側にオフセットした孤立円盤を浮かべる）アーキタイプの
    土台。

    :param pts: `rasterize()`に渡したのと同じ中心線点列 `(N,3)`（x, y, yaw）。
        弧長は`rasterize()`の`divider`と同じ「始点からの累積距離」（ループを
        閉じない）で計算するので、`spans`の`s0`/`s1`はそこと同じ基準で指定する
    :param spans: `(s0, s1, offset_m, radius_m)` のリスト。弧長`[s0, s1]`区間に
        含まれる各点について、その点から法線方向（`pts`のyaw列から計算、左が正）に
        `offset_m`ずれた位置へ半径`radius_m`の円盤を置く。符号を変えれば反対側の
        壁/障害物になる
    """
    xs, ys, yaws = pts[:, 0], pts[:, 1], pts[:, 2]
    seg = np.hypot(*np.diff(pts[:, :2], axis=0).T)
    arc = np.concatenate([[0.0], np.cumsum(seg)])
    x0, y0 = origin
    h, w = grid.shape
    nx, ny = -np.sin(yaws), np.cos(yaws)          # 法線（進行方向左が正）

    for s0, s1, offset_m, radius_m in spans:
        m = (arc >= s0) & (arc <= s1)
        if not m.any():
            continue
        ox = xs[m] + nx[m] * offset_m
        oy = ys[m] + ny[m] * offset_m
        r = max(1, int(round(radius_m / resolution)))
        yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
        disc = (xx * xx + yy * yy) <= r * r
        cols = np.round((ox - x0) / resolution).astype(np.int64)
        rows = np.round((oy - y0) / resolution).astype(np.int64)
        for c, rw in zip(cols, rows):
            # ★narrow/obstacleのオフセットは`rasterize()`本体の壁掘りより外側に
            # 出ることがあるため、格子外へのはみ出しはクリップする（本体の掘り/
            # divider は margin 込みで必ず内側に収まる前提で無警戒だが、ここは
            # 新しい・オフセットが自由な経路なので明示的に守る）
            c0, c1 = max(0, c - r), min(w, c + r + 1)
            rw0, rw1 = max(0, rw - r), min(h, rw + r + 1)
            if c0 >= c1 or rw0 >= rw1:
                continue
            dc0, dc1 = c0 - (c - r), c1 - (c - r)
            dr0, dr1 = rw0 - (rw - r), rw1 - (rw - r)
            grid[rw0:rw1, c0:c1] |= disc[dr0:dr1, dc0:dc1]


def build(meta: dict) -> dict:
    """コース JSON（`path` を持つもの）→ `Course` に渡す材料。

    閉ループを宣言しているのに始点と終点が合っていなければ**警告する**。
    黙って繋げると、繋ぎ目に細い壁が残って「なぜかそこで必ず止まる」になる。

    ## `divider`（分離帯）

    `"divider": [[s0, s1], ...]` でコースを平行に2分割する薄い壁を立てられる
    （単位は中心線に沿った弧長 [m]）。実車のコースに「ある場合がある」ため。

    **これがあると走れる幅が半分になる。** 道幅 0.9m なら車線は 0.45m で、
    車体半幅 0.09m×2 と余裕を引くと振れ幅は 8cm しか残らない。地図の分解能が
    そのまま経路の質に効く領域なので、`raspi/auto/raceline.py` の `MAP_RES` は
    ここに合わせて 2.5cm にしてある。
    """
    res = float(meta.get("resolution", 0.02))
    width = float(meta.get("width", 1.0))
    ox, oy, oyaw = (list(meta.get("origin", (0.0, 0.0, 0.0))) + [0.0, 0.0, 0.0])[:3]

    pts = centerline(meta["path"], float(ox), float(oy), float(oyaw),
                     res * _STEP_RATIO)

    if meta.get("loop"):
        gap = math.hypot(pts[-1, 0] - pts[0, 0], pts[-1, 1] - pts[0, 1])
        dth = abs((pts[-1, 2] - pts[0, 2] + math.pi) % (2 * math.pi) - math.pi)
        if gap > width * 0.25 or dth > math.radians(10):
            print(f"!! loop=true だが始点と終点が合っていない: "
                  f"位置 {gap:.2f}m / 向き {math.degrees(dth):.0f}° ずれ（{meta.get('name', '')}）")

    grid, origin = rasterize(pts, width, res, float(meta.get("margin", 0.4)),
                             divider=meta.get("divider"),
                             divider_width=float(meta.get("divider_width", 0.06)))

    start = meta.get("start")
    if start is None:
        # **中心線の始点に置く。** 人間が決めないので壁に埋まりようがない
        start = (float(pts[0, 0]), float(pts[0, 1]), float(pts[0, 2]))

    return {
        "grid": grid,
        "resolution": res,
        "origin": origin,
        "start": tuple(float(v) for v in start),
        "centerline": pts,
        "width": width,
    }
