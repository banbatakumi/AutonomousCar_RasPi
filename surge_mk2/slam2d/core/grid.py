"""占有格子 — レイを彫って地図を作り、動く物を壁として確定させない。

`raspi/nav/grid.py`（このリポジトリの自律走行ミニカー専用SLAM）から移植した、
完全にセンサ・車体非依存のモジュール。`ScanPoints`（脱スキュー済み点群）と
`Pose2D`（姿勢）だけに依存し、それ以外の外部情報を必要としない。

## log-oddsではなくヒット/ミスの回数を数える

セルごとに「壁として当たった回数`hits`」と「レイが素通りした回数`misses`」を
数えるだけ。log-oddsは同じ情報を1つの実数に潰したもので、確率としては上等だが
**「何回見たか」が消える**。今回いちばん効かせたい判断がまさにそれ:

    壁として確定するのは  hits >= min_hits  かつ  hit_weight × hits >= misses

`hits >= min_hits`が**動く物を壁にしない**ための条件。1周のあいだに一度だけ
横切った物体は`hits == 1`にしかならず、壁にならない。log-oddsだと
「1回の強いヒット」と「3回の弱いヒット」が同じ値になりうるので区別できない。

## 当たりは素通りより重い（`hit_weight`、既定3）

以前は`hits / (hits + misses) >= 0.5`（当たりと素通りを同じ重み）だった。
これだと**車線を仕切る薄い壁（厚さ2cm＝1セル弱）が消える**。薄い壁のセルは、
壁にすれすれの角度で走るレイ（格子に量子化すると壁のセルを通過してしまう）と
反対側の車線からのレイの両方から「素通り」を数えられ、実測（toyota、誤差なし）
で当たり33回に対し素通り38回、壁として残ったのは32%だけだった。中心線・
レーシングラインの生成は壁を境界に使うので、仕切りが消えると隣の車線へ
抜ける経路を引いてしまう。Cartographer も当たり0.55/素通り0.49（log-oddsで
約5:1）と当たりを重く扱っている。実測では重み2〜5で薄い壁は残り、
大きくするほど雑音由来の壁が太る（実測: normal で重み5だとレース時の自己位置
誤差が4.5cm→9.4cmへ悪化）ので既定は2にしてある。

さらに`known_free`（**空きだと確信できる**セル）を`hits + misses >= min_seen`
かつ壁の条件を満たさないもので定義する。「未知」と「空き」を混ぜないことが、
誤検出を出さない唯一のコツ。

## 数えるのは「点の数」ではなく「周の数」

1周のうちに同じセルへ何点落ちても**+1**。これを守らないと`min_hits`が意味を
失う。角度分解能の高いLiDARでは、近距離で1つのセルに複数点が同時に落ちる。
点の数で数えると、動く物体の一時的な横切りが「何度も見た壁」と区別つかなく
なる。`bincount`の結果を1で頭打ちにするだけで実現する（`_add`）。

## 座標系

`grid[row, col]`のrowはoriginからの+y方向、colは+x方向。SI単位系。
"""

from __future__ import annotations

import numpy as np

from .types import Pose2D, ScanPoints

__all__ = ["OccGrid", "dilate", "UNKNOWN", "FREE", "OCCUPIED"]

#: レイを進める刻み幅を解像度の何倍にするか。1.0だと斜めのレイがセルを飛ばす
_STEP_RATIO = 0.5
#: 回数カウンタの上限。uint16の飽和で「昔たくさん見た」が固まるのを防ぐ
_COUNT_MAX = 60000
#: レイの終端の**手前どれだけを彫らないか**［セル］。
#:
#: ここが小さいと**壁が自分のレイで消える**。特にやられるのがコーナーの内側の
#: 壁で、走路に対して浅い角度でレイが舐めていくため、同じセルに「隣を通り
#: 過ぎたミス」が延々と積もる。幅はセンサの測距ノイズ1σ相当を目安に選ぶ
#: （`raspi/nav/grid.py`の実測では1.5セルで網羅率99%・自己位置誤差4.1cmだった）
_END_BACKOFF = 1.5

UNKNOWN, FREE, OCCUPIED = 0, 1, 2


class OccGrid:
    """占有格子。**原点はセル(0,0)の角。**

    `grow=True`（既定）なら、レイが格子の外へ出たとき**その方向にだけ**
    `grow_step_m`ずつ格子を広げる（原点が動く。世界座標[m]は変わらない）。
    以前は固定サイズで、スタート地点を中心に16m四方だったため、スタートが
    コースの端にある11m級のコースで地図がはみ出していた（`sim/slam_bench.py`の
    circuit_chicane_a・course1・course3）。`grow=False`なら固定のまま、入り切ら
    なかった本数を`out_of_bounds`に数える。
    """

    def __init__(self, *, resolution: float = 0.05, size_m: float = 20.0,
                 origin: tuple[float, float] | None = None,
                 min_hits: int = 3, min_seen: int = 3, hit_weight: float = 2.0,
                 grow: bool = False, grow_step_m: float = 4.0,
                 max_size_m: float = 60.0) -> None:
        self.resolution = float(resolution)
        n = max(1, int(round(size_m / resolution)))
        self.width = self.height = n
        #: 原点を指定しなければ**中心が(0,0)**になるように置く
        self.origin = origin if origin is not None else (-size_m / 2.0, -size_m / 2.0)
        self.min_hits = int(min_hits)
        self.min_seen = int(min_seen)
        #: 当たり1回が素通り何回ぶんに相当するか（docstring「当たりは素通りより重い」）
        self.hit_weight = float(hit_weight)
        self.grow = bool(grow)
        self.grow_step_m = float(grow_step_m)
        self.max_size_m = float(max_size_m)

        self.hits = np.zeros((n, n), dtype=np.uint16)
        self.misses = np.zeros((n, n), dtype=np.uint16)
        #: 壁に当たった点の位置の和と個数（**点の数**で数える）。セル内のどこに壁の
        #: 表面があるかをサブセル精度で持つため（`core/surfmap.py`の面要素の位置）
        self.hit_sx = np.zeros((n, n), dtype=np.float32)
        self.hit_sy = np.zeros((n, n), dtype=np.float32)
        self.hit_n = np.zeros((n, n), dtype=np.float32)
        #: 地図の版。変わったときだけ再描画・再配布すればよい、という用途のための番号
        self.seq = 0
        #: 格子の外に出たレイの本数（累積）。増え続けるなら`size_m`が足りない
        self.out_of_bounds = 0
        self.frozen = False

        #: `seq`ごとの読み出し結果のキャッシュ（1周期に何度も呼ばれるため）
        self._cache: dict[str, np.ndarray] = {}
        self._cache_seq = -1

    # ── 座標変換 ──

    def to_cell(self, x: np.ndarray | float, y: np.ndarray | float):
        """世界座標[m] → (col, row)。**範囲検査はしない**（呼び出し側で潰す）。"""
        col = np.floor((np.asarray(x) - self.origin[0]) / self.resolution).astype(np.int32)
        row = np.floor((np.asarray(y) - self.origin[1]) / self.resolution).astype(np.int32)
        return col, row

    def to_world(self, col: np.ndarray | float, row: np.ndarray | float):
        """(col, row) → **セル中心**の世界座標[m]。"""
        x = self.origin[0] + (np.asarray(col) + 0.5) * self.resolution
        y = self.origin[1] + (np.asarray(row) + 0.5) * self.resolution
        return x, y

    def inside(self, col: np.ndarray, row: np.ndarray) -> np.ndarray:
        return (col >= 0) & (col < self.width) & (row >= 0) & (row < self.height)

    # ── 更新 ──

    def integrate(self, pts: ScanPoints, pose: Pose2D) -> None:
        """脱スキュー済みの点群を1周ぶん取り込む。`pose`は共通フレームでの世界姿勢。

        **凍結後は何もしない。** 凍結は「この地図で走る（使う）」という宣言なので、
        使用中に地図が動くと経路との対応が黙って崩れる。
        """
        if self.frozen or len(pts) == 0:
            return
        x0, y0, yaw = pose
        c, s = np.cos(yaw), np.sin(yaw)
        wx = x0 + pts.x * c - pts.y * s
        wy = y0 + pts.x * s + pts.y * c

        if self.grow:
            self._ensure_inside(min(float(wx.min()), x0), min(float(wy.min()), y0),
                                max(float(wx.max()), x0), max(float(wy.max()), y0))
        # 「地図が足りない」の指標は**レイの終端が格子の外に出た本数**。
        # レイ全体が外に出た本数だと、車が格子の中にいる限り一生増えない
        col, row = self.to_cell(wx, wy)
        self.out_of_bounds += int((~self.inside(col, row)).sum())
        self._carve(x0, y0, wx, wy)
        self._mark(wx[pts.hit], wy[pts.hit])
        self.seq += 1

    def _ensure_inside(self, xmin: float, ymin: float, xmax: float, ymax: float) -> None:
        """矩形`[xmin,xmax]×[ymin,ymax]`が収まるよう、足りない側にだけ格子を足す。"""
        res = self.resolution
        step = max(1, int(round(self.grow_step_m / res)))
        margin = 2 * res
        c0 = int(np.floor((xmin - margin - self.origin[0]) / res))
        r0 = int(np.floor((ymin - margin - self.origin[1]) / res))
        c1 = int(np.floor((xmax + margin - self.origin[0]) / res))
        r1 = int(np.floor((ymax + margin - self.origin[1]) / res))
        left = -(-max(0, -c0) // step) * step
        bottom = -(-max(0, -r0) // step) * step
        right = -(-max(0, c1 - (self.width - 1)) // step) * step
        top = -(-max(0, r1 - (self.height - 1)) // step) * step
        if not (left or bottom or right or top):
            return
        max_cells = int(round(self.max_size_m / res))
        if (self.width + left + right > max_cells or self.height + bottom + top > max_cells):
            return                          # 上限。はみ出しは`out_of_bounds`に数える
        pad = ((bottom, top), (left, right))
        self.hits = np.pad(self.hits, pad)
        self.misses = np.pad(self.misses, pad)
        self.hit_sx = np.pad(self.hit_sx, pad)
        self.hit_sy = np.pad(self.hit_sy, pad)
        self.hit_n = np.pad(self.hit_n, pad)
        self.height, self.width = self.hits.shape
        self.origin = (self.origin[0] - left * res, self.origin[1] - bottom * res)
        self._cache_seq = -1

    def _carve(self, ox: float, oy: float,
               wx: np.ndarray, wy: np.ndarray) -> None:
        """レイの手前を「素通り」として数える。**終端セルは含めない。**"""
        dx = wx - ox
        dy = wy - oy
        d = np.hypot(dx, dy)
        live = d > self.resolution
        if not live.any():
            return
        dx, dy, d = dx[live] / d[live], dy[live] / d[live], d[live]

        step = self.resolution * _STEP_RATIO
        n_steps = max(1, int(np.ceil(float(d.max()) / step)))
        t = (np.arange(1, n_steps + 1, dtype=np.float32) * step)[None, :]

        col, row = self.to_cell(ox + dx[:, None] * t, oy + dy[:, None] * t)
        ok = self.inside(col, row)
        # 終端セルの手前まで。**終端は壁かもしれないので空きにしない**（`_END_BACKOFF`）
        ok &= t < (d[:, None] - self.resolution * _END_BACKOFF)
        self._bump(self.misses, row[ok], col[ok])

    def _mark(self, wx: np.ndarray, wy: np.ndarray) -> None:
        """終端に壁を打つ。位置の和も数えておく（`hit_mean`がサブセル精度の壁を作る）。"""
        if wx.size == 0:
            return
        col, row = self.to_cell(wx, wy)
        ok = self.inside(col, row)
        if not ok.any():
            return
        r, c = row[ok], col[ok]
        self._bump(self.hits, r, c)
        r0, r1 = int(r.min()), int(r.max())
        c0, c1 = int(c.min()), int(c.max())
        h, w = r1 - r0 + 1, c1 - c0 + 1
        flat = (r - r0) * w + (c - c0)
        n = w * h
        self.hit_n[r0:r1 + 1, c0:c1 + 1] += np.bincount(flat, minlength=n).reshape(h, w).astype(np.float32)
        self.hit_sx[r0:r1 + 1, c0:c1 + 1] += np.bincount(
            flat, weights=wx[ok], minlength=n).reshape(h, w).astype(np.float32)
        self.hit_sy[r0:r1 + 1, c0:c1 + 1] += np.bincount(
            flat, weights=wy[ok], minlength=n).reshape(h, w).astype(np.float32)

    def hit_mean(self) -> tuple[np.ndarray, np.ndarray] | None:
        """壁セルごとの点の平均位置[m]（点が無いセルはセル中心）。無ければ None。"""
        if not self.hit_n.any():
            return None
        def make():
            n = np.maximum(self.hit_n, 1.0)
            cx, cy = self.to_world(np.arange(self.width)[None, :], np.arange(self.height)[:, None])
            mx = np.where(self.hit_n > 0, self.hit_sx / n, cx).astype(np.float64)
            my = np.where(self.hit_n > 0, self.hit_sy / n, cy).astype(np.float64)
            return np.stack([mx, my])
        m = self._cached("hitmean", make)
        return m[0], m[1]

    def _bump(self, target: np.ndarray, rows: np.ndarray, cols: np.ndarray) -> None:
        """該当セルを**1だけ**増やす。**1周で何点落ちても+1**（docstring参照）。

        触ったセルを囲む矩形の中だけで数える。格子全体に対する`bincount`は
        格子の大きさに比例するコストが1周ごとに掛かり、地図が大きいほど遅く
        なっていた（16m四方で1回3〜5ms、ループ閉じ後の焼き直し640キーフレームで
        3秒以上）。矩形はレイ1本ぶんの広がりしかないので、実質は点の数に比例する。
        """
        if rows.size == 0:
            return
        r0, r1 = int(rows.min()), int(rows.max())
        c0, c1 = int(cols.min()), int(cols.max())
        h, w = r1 - r0 + 1, c1 - c0 + 1
        flat = (rows - r0) * w + (cols - c0)
        touched = np.bincount(flat, minlength=w * h).reshape(h, w) > 0
        sub = target[r0:r1 + 1, c0:c1 + 1]
        # `_COUNT_MAX`で頭打ち（uint16の飽和で「昔たくさん見た」が固まるのを防ぐ）
        np.add(sub, touched, out=sub, where=sub < _COUNT_MAX, casting="unsafe")

    def freeze(self) -> None:
        """地図を確定させる。以降`integrate()`は無視される。"""
        self.frozen = True
        self.seq += 1

    # ── 読み出し ──

    def _cached(self, key: str, make):
        if self._cache_seq != self.seq:
            self._cache = {}
            self._cache_seq = self.seq
        v = self._cache.get(key)
        if v is None:
            v = make()
            v.setflags(write=False)          # キャッシュなので呼び出し側に書き換えさせない
            self._cache[key] = v
        return v

    @property
    def seen(self) -> np.ndarray:
        """観測回数（ヒット＋ミス）。int32に伸ばして返す（uint16の引き算は罠）。"""
        return self._cached("seen", lambda: self.hits.astype(np.int32) + self.misses.astype(np.int32))

    def wall_mask(self) -> np.ndarray:
        """壁として確定したセル。**`hits >= min_hits`が動く物を弾いている。**

        返す配列はキャッシュなので**書き換えないこと**。
        """
        def make():
            h = self.hits.astype(np.float32)
            return (self.hits >= self.min_hits) & (h * self.hit_weight >= self.misses)
        return self._cached("wall", make)

    def known_free_mask(self) -> np.ndarray:
        """**空きだと確信できる**セル。未知（見ていない）とは区別する。

        動的障害物の検出はここだけを使う。未知セルを空き扱いにすると、
        まだ見ていない場所を通るたびに「障害物だ」と言い出す。
        """
        def make():
            h = self.hits.astype(np.float32)
            return (self.seen >= self.min_seen) & (h * self.hit_weight < self.misses)
        return self._cached("free", make)

    def trinary(self) -> np.ndarray:
        """3値の地図（0=未知 1=空き 2=占有）。表示・シリアライズは呼び出し側の責務。"""
        out = np.full((self.height, self.width), UNKNOWN, dtype=np.uint8)
        out[self.known_free_mask()] = FREE
        out[self.wall_mask()] = OCCUPIED
        return out

    # ── レイキャスト ──

    def raycast(self, ox, oy, angles: np.ndarray,
                max_range: float, mask: np.ndarray | None = None,
                fill: int = 1) -> np.ndarray:
        """`angles`（世界座標の絶対角[rad]）方向の壁までの距離[m]。

        `ox`/`oy`はスカラでも`angles`と同じ長さの配列でもよい。原点が点ごとに
        異なる場合は一括計算した方が桁違いに速い。

        当たらなかったレイは**`max_range`**を返す。

        **既定で壁を1セル太らせてから撃つ（`fill`）。** 角度分解能に対して
        セルが粗いと穴の空いた壁ができ、レイがすり抜けて距離を過大に測る。
        太らせたぶん（`fill`セル）は距離に足して返す（穴を塞ぐのが目的で、
        壁を手前に動かすのが目的ではない）。
        """
        grid = self.wall_mask() if mask is None else mask
        if fill > 0:
            grid = dilate(grid, fill)
        step = self.resolution * _STEP_RATIO
        n_steps = max(1, int(max_range / step))
        t = np.arange(1, n_steps + 1, dtype=np.float32) * step

        cos = np.cos(angles).astype(np.float32)[:, None]
        sin = np.sin(angles).astype(np.float32)[:, None]
        x0 = np.asarray(ox, dtype=np.float32).reshape(-1)[:, None]
        y0 = np.asarray(oy, dtype=np.float32).reshape(-1)[:, None]
        col, row = self.to_cell(x0 + cos * t[None, :], y0 + sin * t[None, :])

        ok = self.inside(col, row)
        occ = grid[np.where(ok, row, 0), np.where(ok, col, 0)]
        # 格子の外は壁扱い。地図の外へレイが伸びていくのを防ぐ
        occ = np.where(ok, occ, True)

        hit = occ.any(axis=1)
        first = occ.argmax(axis=1)
        d = t[first] + fill * self.resolution
        return np.where(hit, np.minimum(d, max_range), max_range).astype(np.float64)


def dilate(mask: np.ndarray, cells: int) -> np.ndarray:
    """`mask`を`cells`セルぶん太らせる（4近傍）。

    動的障害物の判定で「壁のすぐ近く」を除外する用途に使う。局在化が1〜2セル
    ずれると壁の点が空きセルに落ち、動く物が居ないのに誤検出する。
    """
    out = mask
    for _ in range(max(0, cells)):
        out = _spread(out, out, np.logical_or)
    return out


def _spread(base: np.ndarray, src: np.ndarray, op) -> np.ndarray:
    """`src`を上下左右に1つずらして`base`に重ねる。

    **`np.roll`を使ってはいけない。** 端が反対側へ回り込むので、地図の下端に
    ある壁が上端に漏れる。スライスなら回り込まないうえ速い。
    """
    out = base.copy()
    op(out[1:, :], src[:-1, :], out=out[1:, :])
    op(out[:-1, :], src[1:, :], out=out[:-1, :])
    op(out[:, 1:], src[:, :-1], out=out[:, 1:])
    op(out[:, :-1], src[:, 1:], out=out[:, :-1])
    return out
