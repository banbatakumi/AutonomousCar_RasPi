"""占有格子 — レイを彫って地図を作り、動く物を壁として確定させない。

## log-odds ではなくヒット/ミスの回数を数える

セルごとに「壁として当たった回数 `hits`」と「レイが素通りした回数 `misses`」を
数えるだけ。log-odds は同じ情報を1つの実数に潰したもので、確率としては上等だが、
**「何回見たか」が消える**。今回いちばん効かせたい判断がまさにそれ:

    壁として確定するのは  hits >= min_hits  かつ  hits / (hits + misses) >= 0.5

`hits >= min_hits` が**動く物を壁にしない**ための条件。1周のあいだに一度だけ
横切った人の脚は `hits == 1` にしかならず、壁にならない。log-odds だと
「1回の強いヒット」と「3回の弱いヒット」が同じ値になりうるので区別できない。

さらに `known_free`（**空きだと確信できる**セル）を `hits + misses >= min_seen`
かつ占有率 0.5 未満で定義する。走行中の動的障害物の検出はこれを使う。
**「未知」と「空き」を混ぜないことが、誤検出を出さない唯一のコツ。**

## ★ 数えるのは「点の数」ではなく「周の数」

1周のうちに同じセルへ何点落ちても **+1**。これを守らないと `min_hits` が意味を失う。
1° 刻みの点群は 1m 先で 1.7cm 間隔なので、**5cm のセルには1周で 3 点以上が同時に
落ちる**。点の数で数えると、目の前を横切った人の脚が1周で `hits = 10` に達し、
「3 回見た壁」と区別がつかなくなる。

実装は `bincount` の結果を 1 で頭打ちにするだけ（`_add(..., once=True)`）。
`np.unique` で潰すより速く、レイが同じセルを 2 回踏む問題も同時に消える。

## レイ彫りは `np.bincount`

レイに沿って解像度の 0.5 倍刻みでサンプルして一括加算する。0.5 倍刻みなのは
1.0 倍だと斜めのレイがセルを飛ばすため（`sim/course.py` の `_STEP_RATIO` と同じ理由）。

## 座標系

`docs/architecture.md` §5.1 のまま（x = 前/右、y = 左/上、SI）。
`grid[row, col]` の **row は origin からの +y 方向**、col は +x 方向。
画像として出すときに上下がひっくり返るのは表示側の都合なので、ここでは直さない
（`sim/course.py` の格子と同じ向きに揃えてある）。
"""

from __future__ import annotations

import zlib

import numpy as np

from .deskew import Points

__all__ = ["OccGrid", "dilate", "pack_trinary"]

#: レイを進める刻み幅を解像度の何倍にするか。1.0 だと斜めのレイがセルを飛ばす
_STEP_RATIO = 0.5
#: 回数カウンタの上限。uint16 の飽和で「昔たくさん見た」が固まるのを防ぐ
_COUNT_MAX = 60000
#: ★ レイの終端の**手前どれだけを彫らないか**［セル］。
#:
#: ここが小さいと**壁が自分のレイで消える**。特にやられるのが
#: **コーナーの内側の壁**で、走路に対して浅い角度でレイが舐めていくため、
#: 同じセルに「隣を通り過ぎたミス」が延々と積もる。実測（circuit・1周・
#: `misses` が中央 83 に対し `hits` は 19）で `ratio < 0.5` に落ちて
#: **3440 セルが壁から外れていた**（GUI で内壁が虫食いになる症状）。
#:
#: 幅は**測距ノイズ 1σ 相当**で決める。σ = 1cm + 0.5%×距離 なので
#: 3m で 2.5cm ＝ 2.5cm 格子の 1 セル。1.5 セル取れば終端のばらつきを覆える。
#:
#: 実測（circuit・150秒・壁の網羅率／自己位置の平均誤差）:
#:
#:     0.5 セル（旧）→ 網羅 89% / 9.3cm
#:     1.5 セル      → 網羅 99% / 4.1cm   ※正確さは 100%→99% でほぼ不変
#:
#: **大きくしすぎない。** 動く物が通り過ぎた跡を彫って消せなくなる
_END_BACKOFF = 1.5

UNKNOWN, FREE, OCCUPIED = 0, 1, 2


class OccGrid:
    """固定サイズの占有格子。**原点はセル(0,0)の角。**

    地図を広げる仕組みは持たない。コースの大きさは走る前に分かっているので、
    動的な拡張を入れるより「入らなかったら `size_m` を上げる」方が読みやすい。
    入り切らなかったことは `out_of_bounds` が数えているので気づける。
    """

    def __init__(self, *, resolution: float = 0.05, size_m: float = 20.0,
                 origin: tuple[float, float] | None = None,
                 min_hits: int = 3, min_seen: int = 3) -> None:
        self.resolution = float(resolution)
        n = max(1, int(round(size_m / resolution)))
        self.width = self.height = n
        #: 原点を指定しなければ**中心が (0,0)** になるように置く
        self.origin = origin if origin is not None else (-size_m / 2.0, -size_m / 2.0)
        self.min_hits = int(min_hits)
        self.min_seen = int(min_seen)

        self.hits = np.zeros((n, n), dtype=np.uint16)
        self.misses = np.zeros((n, n), dtype=np.uint16)
        #: 地図の版。**変わったときだけ GUI へ送る**ための番号
        self.seq = 0
        #: 格子の外に出たレイの本数（累積）。増え続けるなら `size_m` が足りない
        self.out_of_bounds = 0
        self.frozen = False

        self._score: np.ndarray | None = None
        self._score_seq = -1

    # ── 座標変換 ──

    def to_cell(self, x: np.ndarray | float, y: np.ndarray | float):
        """世界座標 [m] → (col, row)。**範囲検査はしない**（呼び出し側で潰す）。"""
        col = np.floor((np.asarray(x) - self.origin[0]) / self.resolution).astype(np.int32)
        row = np.floor((np.asarray(y) - self.origin[1]) / self.resolution).astype(np.int32)
        return col, row

    def to_world(self, col: np.ndarray | float, row: np.ndarray | float):
        """(col, row) → **セル中心**の世界座標 [m]。"""
        x = self.origin[0] + (np.asarray(col) + 0.5) * self.resolution
        y = self.origin[1] + (np.asarray(row) + 0.5) * self.resolution
        return x, y

    def inside(self, col: np.ndarray, row: np.ndarray) -> np.ndarray:
        return ((col >= 0) & (col < self.width) & (row >= 0) & (row < self.height))

    # ── 更新 ──

    def integrate(self, pts: Points, pose: tuple[float, float, float]) -> None:
        """脱スキュー済みの点群を1周ぶん取り込む。`pose` は base_link の世界姿勢。

        **凍結後は何もしない。** 凍結は「この地図で走る」という宣言なので、
        走行中に地図が動くと、経路との対応が黙って崩れる。
        """
        if self.frozen or len(pts) == 0:
            return
        x0, y0, yaw = pose
        c, s = np.cos(yaw), np.sin(yaw)
        wx = x0 + pts.x * c - pts.y * s
        wy = y0 + pts.x * s + pts.y * c

        self._carve(x0, y0, wx, wy)
        self._mark(wx[pts.hit], wy[pts.hit])
        self.seq += 1

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

        flat = row.astype(np.int64) * self.width + col
        self.out_of_bounds += int((~self.inside(col, row)).all(axis=1).sum())
        self._add(self.misses, flat[ok])

    def _mark(self, wx: np.ndarray, wy: np.ndarray) -> None:
        """終端に壁を打つ。"""
        if wx.size == 0:
            return
        col, row = self.to_cell(wx, wy)
        ok = self.inside(col, row)
        self._add(self.hits, (row[ok].astype(np.int64) * self.width + col[ok]))

    def _add(self, target: np.ndarray, flat: np.ndarray) -> None:
        """該当セルを **1 だけ**増やす。**1周で何点落ちても +1**（docstring 参照）。"""
        if flat.size == 0:
            return
        counts = np.bincount(flat, minlength=self.width * self.height)
        np.minimum(counts, 1, out=counts)
        acc = target.reshape(-1).astype(np.int32) + counts.astype(np.int32)
        np.clip(acc, 0, _COUNT_MAX, out=acc)
        target[:] = acc.astype(np.uint16).reshape(target.shape)

    def freeze(self) -> None:
        """地図を確定させる。以降 `integrate()` は無視される。"""
        self.frozen = True
        self.seq += 1

    # ── 読み出し ──

    @property
    def seen(self) -> np.ndarray:
        """観測回数（ヒット＋ミス）。int32 に伸ばして返す（uint16 の引き算は罠）。"""
        return self.hits.astype(np.int32) + self.misses.astype(np.int32)

    def wall_mask(self) -> np.ndarray:
        """壁として確定したセル。**`hits >= min_hits` が動く物を弾いている。**"""
        seen = self.seen
        with np.errstate(invalid="ignore", divide="ignore"):
            ratio = np.where(seen > 0, self.hits / np.maximum(seen, 1), 0.0)
        return (self.hits >= self.min_hits) & (ratio >= 0.5)

    def known_free_mask(self) -> np.ndarray:
        """**空きだと確信できる**セル。未知（見ていない）とは区別する。

        動的障害物の検出はここだけを使う。未知セルを空き扱いにすると、
        1周目に見えなかった場所を通るたびに「障害物だ」と言い出す。
        """
        seen = self.seen
        with np.errstate(invalid="ignore", divide="ignore"):
            ratio = np.where(seen > 0, self.hits / np.maximum(seen, 1), 1.0)
        return (seen >= self.min_seen) & (ratio < 0.5)

    def trinary(self) -> np.ndarray:
        """GUI へ送る 3 値の地図（0=未知 1=空き 2=占有）。"""
        out = np.full((self.height, self.width), UNKNOWN, dtype=np.uint8)
        out[self.known_free_mask()] = FREE
        out[self.wall_mask()] = OCCUPIED
        return out

    def score_map(self, *, radius: int = 2, decay: float = 0.6) -> np.ndarray:
        """スキャンマッチ用の尤度場。**壁の周りをなだらかに盛った地図。**

        壁ちょうどのセルだけを 1 点とすると、1 セル（5cm）ずれた候補姿勢の得点が
        いきなり 0 になり、探索が平坦な谷底で迷子になる。半径 `radius` セルまで
        `decay` で減衰させて裾を作ると、粗い刻みでも正しい方向へ落ちる。

        `seq` が変わるまでキャッシュする（1周期に何度も呼ばれるため）。
        """
        if self._score is not None and self._score_seq == self.seq:
            return self._score
        s = self.wall_mask().astype(np.float32)
        for _ in range(max(0, radius)):
            s = _spread(s, s * decay, np.maximum)
        self._score = s
        self._score_seq = self.seq
        return s

    # ── レイキャスト（中心線の道幅測定に使う） ──

    def raycast(self, ox, oy, angles: np.ndarray,
                max_range: float, mask: np.ndarray | None = None,
                fill: int = 1) -> np.ndarray:
        """`angles`（世界座標の絶対角 [rad]）方向の壁までの距離 [m]。

        `ox`/`oy` はスカラでも `angles` と同じ長さの配列でもよい。**道幅の測定は
        点ごとに原点が違う**ので、1点ずつ呼ぶと呼び出しのたびに (R, S) の配列を
        作り直して桁違いに遅くなる（278点で 130ms → 一括なら数 ms）。

        当たらなかったレイは **`max_range`** を返す（`sim/course.py` の 0.0 とは
        違う）。道幅の測定に使うので、「壁が無い ＝ 幅が無限」ではなく
        「少なくとも max_range はある」と読める方が呼び出し側が素直になる。

        **既定で壁を 1 セル太らせてから撃つ（`fill`）。** 1° 刻みの点群は 3m 先で
        5.2cm 間隔になり、5cm のセルに対して**穴の空いた壁**ができる。そのまま
        撃つとレイが壁をすり抜け、道幅を数 m 過大に測る（＝壁にめり込む
        レーシングラインが「実行可能」に見える）。太らせる側に間違えるのが安全。

        太らせたぶん（`fill` セル）は**距離に足して返す**。穴を塞ぐのが目的で、
        壁を手前に動かすのが目的ではない。足さないと道幅が左右 1 セルずつ狭く出て、
        レーシングラインの振れ幅がそのぶん小さくなる。
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
        # 格子の外は壁扱い。**コースの外へ道幅が伸びていくのを防ぐ**
        occ = np.where(ok, occ, True)

        hit = occ.any(axis=1)
        first = occ.argmax(axis=1)
        d = t[first] + fill * self.resolution
        return np.where(hit, np.minimum(d, max_range), max_range).astype(np.float64)


def pack_trinary(t: np.ndarray) -> bytes:
    """3値の地図を **2bit に詰めて zlib で圧縮**する（`AutoMap.cells`）。

    400×400 の生 uint8 が 160KB、2bit で 40KB、圧縮して数KB。GUI 側の展開は
    ブラウザ標準の `DecompressionStream('deflate')` でできる（`zlib.compress` は
    zlib ヘッダ付きなので `'deflate'`。`'deflate-raw'` ではない）ので、
    フロントに依存を増やさずに済む。
    """
    flat = np.ascontiguousarray(t, dtype=np.uint8).reshape(-1)
    pad = (-flat.size) % 4
    if pad:
        flat = np.concatenate([flat, np.zeros(pad, dtype=np.uint8)])
    packed = (flat[0::4] | (flat[1::4] << 2) | (flat[2::4] << 4) | (flat[3::4] << 6))
    return zlib.compress(packed.tobytes(), 6)


def dilate(mask: np.ndarray, cells: int) -> np.ndarray:
    """`mask` を `cells` セルぶん太らせる（4近傍）。

    動的障害物の判定で「壁のすぐ近く」を除外するために使う。局在化が 1〜2 セル
    ずれると壁の点が空きセルに落ち、**動く物が居ないのに毎周期止まる**。
    """
    out = mask
    for _ in range(max(0, cells)):
        out = _spread(out, out, np.logical_or)
    return out


def _spread(base: np.ndarray, src: np.ndarray, op) -> np.ndarray:
    """`src` を上下左右に1つずらして `base` に重ねる。

    **`np.roll` を使ってはいけない。** 端が反対側へ回り込むので、地図の下端に
    ある壁が上端に漏れる。地図の縁はコースの外なので、そこに偽の壁ができると
    レイキャストが手前で止まり道幅を誤る。スライスなら回り込まないうえ速い。
    """
    out = base.copy()
    op(out[1:, :], src[:-1, :], out=out[1:, :])
    op(out[:-1, :], src[1:, :], out=out[:-1, :])
    op(out[:, 1:], src[:, :-1], out=out[:, 1:])
    op(out[:, :-1], src[:, 1:], out=out[:, :-1])
    return out
