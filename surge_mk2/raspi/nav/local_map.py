"""駐車用の局所環境表現 — 占有格子に溜めて、ESDF（距離場）で余裕を O(1) で引く。

## なぜ瞬時の点群では足りないのか

`park_to_point`は当初、毎周期のスキャンを点群のまま掃引判定へ渡していた。
これには2つの穴がある:

1. **遮蔽と角度分解能で駐車枠の内側が見えていない。** 車庫入れ配置40通りで
   目標フットプリント上の3点のクリアランスを測ると、**3割の地点で実際より
   3cm以上「余裕がある」と誤認**した（最大21cm、2026-09-16計測）。しかも
   欠測を単に除外していたので**見えていない場所へ楽観的に経路を引く**
2. **一度見た壁を忘れる。** 後退中に前方の壁が視野から外れる、手前の物陰に
   入る、といった場面で経路の妥当性が黙って崩れる

格子に溜めればレイ彫り（`OccGrid`のhits/misses）が「空き」「占有」「未知」を
区別して保持する。**未知を空きと混ぜないのがこのモジュールの核心。**

## 硬い拘束は「累積の壁」＋「今見えている点」

`min_hits`（既定2周=0.2s）は測距ノイズの1点が幻の壁になるのを防ぐためだが、
**それだけを硬い拘束にすると、新しく見えた壁を planner が1周期知らない**。
安全層は今のスキャンを見るので、「計画は通ると言い、安全層は当たると言う」
食い違いになり、制動と再計画が交代して進まなくなる（実測）。
そこで`_fresh`（今周期の当たり点のセル）も硬い拘束に含める。単発ノイズは
次の周期には消えるので、幻の壁が残り続けることもない。

## 2つの距離場を持つ

- `clearance()` … **壁まで**の距離。硬い拘束（これを割ったら当たる）
- `clearance(include_unknown=True)` … **壁または未知まで**の距離。
  未知は「当たるかどうか分からない」なので、禁止ではなく**コスト**として
  扱いたい。経路計画は壁で弾き、未知は避ける方向に寄せる

## 矩形フットプリントは円被覆で扱う

ESDFは点の周りの距離しか答えないので、矩形の車体は**中心線上に並べた円**で
覆う。円の数`n`を増やすほど保守性が下がる（半径が車体半幅に近づく）:

    半径 r = √((全長/2n)² + 半幅²)   → 横方向の余分な保守性は r − 半幅

全長0.37m・半幅0.09mのこの車体では n=6 で 0.5cm。**点群を直接なめる方式より
速くて、しかも軸並行bboxの近似より正確**（回転した矩形をbboxで代用すると
最大で対角ぶん太る）。

## 解像度は1cm

駐車の余裕は4〜10cmなので、格子の量子化がそのまま判定の余裕を食う。
1cm格子（6m四方で600×600）＋円被覆のはみ出し1〜1.8cmで、保守性は合計3cm弱。
`cv2.distanceTransform`は600²で数msなので、10Hzでも問題にならない。
**2.5cm格子では真値3cmの余裕が「0cm」と読まれ、「目標が壁と重なっている」と
誤判定した**（2026-09-16実測）。

## 依存

ESDFは`cv2.distanceTransform`（`raspi/requirements.txt`の
`opencv-contrib-python-headless`に含まれる）を使う。**無い環境では
チャンファー近似**へ落ちる（誤差数%）——`raspi/nav/`のモジュールが
importできないと planner ごと起動しなくなるため、硬い依存にはしない。
"""

from __future__ import annotations

import math

import numpy as np

from .deskew import Points
from .grid import OccGrid

__all__ = ["LocalMap", "footprint_circles"]

try:                                          # pragma: no cover - 環境依存
    import cv2
    _HAVE_CV2 = True
except ImportError:                           # pragma: no cover
    _HAVE_CV2 = False


def footprint_circles(footprint: list[tuple[float, float]] | tuple,
                      nx: int = 5, ny: int = 3) -> tuple[np.ndarray, float]:
    """矩形フットプリントを`nx × ny`個の円で覆う。戻り値は `(中心(K,2), 半径)`。

    矩形を`nx × ny`のセルに切り、各セルの外接円を置く:

        セル寸法 dx = 全長/nx、dy = 全幅/ny
        半径 r = hypot(dx/2, dy/2)
        はみ出し量 = 前後 `r − dx/2`、左右 `r − dy/2`

    ★ **中心線上に1列だけ並べてはいけない。** 円は矩形の角を覆うために
    必ず端から外へ出る。1列（ny=1）だと、この車体（全長0.37m・全幅0.18m）
    では前後に**6.4cmもはみ出す**——駐車の余裕は4〜10cmなので、それだけで
    「目標姿勢が壁と重なっている」と誤判定する（2026-09-16、斜め駐車が
    1/8になった原因）。2列以上にすると半径が小さくなり、前後・左右とも
    2cm以下に収まる（既定5×3で前後1.1cm・左右1.8cm）。

    はみ出しは**安全側**（実際より当たりやすく判定する）なので、残る誤差は
    「通れるのに通らない」方向にしか効かない。
    """
    xs = [pt[0] for pt in footprint]
    ys = [pt[1] for pt in footprint]
    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
    dx = (x1 - x0) / nx
    dy = (y1 - y0) / ny
    centers = np.array([[x0 + dx * (i + 0.5), y0 + dy * (j + 0.5)]
                        for i in range(nx) for j in range(ny)])
    return centers, math.hypot(dx / 2.0, dy / 2.0)


def _edt(free: np.ndarray, resolution: float) -> np.ndarray:
    """`free`（True=空き）の各セルから最近傍の非空きセルまでの距離 [m]。"""
    src = free.astype(np.uint8)
    if _HAVE_CV2:
        return cv2.distanceTransform(src, cv2.DIST_L2, 5) * resolution
    return _chamfer(src) * resolution


def _chamfer(src: np.ndarray) -> np.ndarray:
    """2パスのチャンファー距離（cv2が無い環境のフォールバック、誤差数%）。"""
    big = 1e9
    d = np.where(src > 0, big, 0.0)
    h, w = d.shape
    a, b = 1.0, math.sqrt(2.0)
    for i in range(1, h):                     # 前方走査
        row, prev = d[i], d[i - 1]
        row[1:] = np.minimum(row[1:], row[:-1] + a)
        row[:] = np.minimum(row, prev + a)
        row[1:] = np.minimum(row[1:], prev[:-1] + b)
        row[:-1] = np.minimum(row[:-1], prev[1:] + b)
    for i in range(h - 2, -1, -1):            # 後方走査
        row, nxt = d[i], d[i + 1]
        row[:-1] = np.minimum(row[:-1], row[1:] + a)
        row[:] = np.minimum(row, nxt + a)
        row[:-1] = np.minimum(row[:-1], nxt[1:] + b)
        row[1:] = np.minimum(row[1:], nxt[:-1] + b)
    return d


class LocalMap:
    """基準フレームに固定した局所占有格子＋ESDF。

    :param min_hits: 壁として確定するのに必要なヒット数。**1にしない**
        ——測距ノイズの1点が幻の壁になり、`misses`では消えないので経路が
        永久に塞がれる。2なら0.2s（2周）で確定し、単発ノイズは弾ける
    :param min_seen: 空きとして確定するのに必要な観測数。1でよい
        ——「1度素通りした」で空きとみなしても危険側には倒れない（危険側は
        「壁を空きと言う」方であって、こちらは`hits`の比率条件が守っている）
    """

    def __init__(self, *, resolution: float = 0.01, size_m: float = 6.0,
                 min_hits: int = 2, min_seen: int = 1,
                 footprint: list[tuple[float, float]] | tuple = (),
                 circles_x: int = 5, circles_y: int = 3,
                 max_cells: int = 700) -> None:
        self.resolution = float(resolution)
        self.size_m = float(size_m)
        #: 1辺のセル数の上限。**解像度より優先する**——広い地図で1cmを
        #: 維持するとESDFが1周期に収まらないため、広いときは自動で粗くする
        self.max_cells = int(max_cells)
        #: 格子の中心（基準フレーム座標）。既定は基準フレームの原点
        self.center = (0.0, 0.0)
        self._min_hits = int(min_hits)
        self._min_seen = int(min_seen)
        self._circles, self._circle_r = (
            footprint_circles(footprint, circles_x, circles_y) if footprint
            else (np.zeros((1, 2)), 0.1))
        self.reset()

    def configure(self, *, size_m: float, center: tuple[float, float],
                  resolution: float | None = None) -> None:
        """格子の広さと中心を決め直す（`reset()`を含む）。

        ★**目標が格子に収まっていなければならない。** 外周1セルは塞いで
        ある（距離場が格子外へ伸びるのを防ぐため）ので、目標が縁に乗ると
        「目標姿勢が障害物と重なっています」になり、Hybrid A*が即失敗する。
        実測（2026-09-17）: 既定6m・原点中心の格子で3m先を目標にすると
        目標の余裕が-4.8cmと報告され、RS候補フォールバックが壁を無視した
        経路を出した。**目標距離から広さを決める**のが正しい。
        """
        self.size_m = float(size_m)
        self.center = (float(center[0]), float(center[1]))
        if resolution is not None:
            self.resolution = float(resolution)
        # セル数の上限を超えるなら解像度を粗くする（ESDFの計算量を抑える）
        if self.size_m / self.resolution > self.max_cells:
            self.resolution = self.size_m / self.max_cells
        self.reset()

    def reset(self) -> None:
        self.grid = OccGrid(resolution=self.resolution, size_m=self.size_m,
                            origin=(self.center[0] - self.size_m / 2.0,
                                    self.center[1] - self.size_m / 2.0),
                            min_hits=self._min_hits, min_seen=self._min_seen)
        self._wall_edt: np.ndarray | None = None
        self._blocked_edt: np.ndarray | None = None
        self._edt_seq = -1
        #: 今周期の当たり点のセル。**累積の`min_hits`を待たない**
        self._fresh: np.ndarray | None = None
        self._fresh_seq = -1

    @property
    def circle_radius(self) -> float:
        return self._circle_r

    def integrate(self, pts: Points, pose: tuple[float, float, float]) -> None:
        """脱スキュー済み点群を取り込む。`pose`は**基準フレームでの**base_link姿勢。

        累積（`OccGrid`）に加えて、**今周期の当たり点そのもの**を
        `_fresh`として覚える。`min_hits`（既定2周）を待つ累積だけを硬い拘束に
        使うと、**新しく見えた壁を planner が1周期知らないまま経路を引く**
        ——そして安全層（今のスキャンを見る）がそれを拒否する、という食い違いが
        起きる。実測では縦列駐車で「計画→即制動→再計画」が延々と交代した。
        """
        self.grid.integrate(pts, pose)
        x0, y0, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        wx = x0 + pts.x[pts.hit] * c - pts.y[pts.hit] * s
        wy = y0 + pts.x[pts.hit] * s + pts.y[pts.hit] * c
        col, row = self.grid.to_cell(wx, wy)
        h, w = self.grid.height, self.grid.width
        ok = (col >= 0) & (col < w) & (row >= 0) & (row < h)
        self._fresh = np.zeros((h, w), dtype=bool)
        self._fresh[row[ok], col[ok]] = True
        self._fresh_seq = self.grid.seq

    # ── 距離場 ──

    def refresh(self) -> None:
        """ESDFを作り直す。**格子が更新された周期に1回だけ呼ぶ。**"""
        if self._edt_seq == self.grid.seq:
            return
        wall = self.grid.wall_mask()
        if self._fresh is not None:
            wall = wall | self._fresh          # 今見えている壁も硬い拘束に含める
        free = self.grid.known_free_mask()
        # 格子の外は未知。**外周1セルを塞ぐ**ことで、距離場が格子の外側へ
        # 無限に伸びて「端に寄れば余裕がある」と誤答するのを防ぐ
        blocked = wall | ~free
        blocked[0, :] = blocked[-1, :] = True
        blocked[:, 0] = blocked[:, -1] = True
        wall_only = wall.copy()
        wall_only[0, :] = wall_only[-1, :] = True
        wall_only[:, 0] = wall_only[:, -1] = True
        self._wall_edt = _edt(~wall_only, self.resolution)
        self._blocked_edt = _edt(~blocked, self.resolution)
        self._edt_seq = self.grid.seq

    @property
    def ready(self) -> bool:
        """ESDFが作られており、かつ**壁が確定できるだけ観測を重ねたか**。

        ★`min_hits`（既定2）に届くまでは`wall_mask()`が空なので、地図は
        「全部空き」に見える。その状態で経路計画すると障害物を無視した経路が
        出るので、呼び出し側は`ready`が立つまで走り出してはいけない
        （`park_to_point`は「地図を初期化中」として制動する。LiDARは10Hzなので
        待ち時間は0.2s程度）。
        """
        return self._wall_edt is not None and self.grid.seq >= self._min_hits

    @property
    def observations(self) -> int:
        """取り込んだ周の数。`ready`が立つまでの進捗表示に使う。"""
        return self.grid.seq

    @property
    def min_hits(self) -> int:
        return self._min_hits

    def contains(self, x: float, y: float, *, pad: float = 0.0) -> bool:
        """点が格子の内側（外周から`pad`以上内）にあるか。"""
        half = self.size_m / 2.0 - pad
        return (abs(x - self.center[0]) <= half
                and abs(y - self.center[1]) <= half)

    def clearance(self, x: np.ndarray | float, y: np.ndarray | float, *,
                  include_unknown: bool = False) -> np.ndarray:
        """点`(x,y)`（基準フレーム）から最近傍の壁（または未知）までの距離 [m]。

        格子の外は0.0（＝塞がれている扱い）。**未知を含めるかは呼び出し側が
        決める**——経路計画は壁で弾き、未知はコストとして使うため。
        """
        field = self._blocked_edt if include_unknown else self._wall_edt
        if field is None:
            return np.zeros(np.shape(x))
        col, row = self.grid.to_cell(x, y)
        h, w = field.shape
        inside = (col >= 0) & (col < w) & (row >= 0) & (row < h)
        out = np.zeros(np.shape(col), dtype=np.float64)
        cc = np.clip(col, 0, w - 1)
        rr = np.clip(row, 0, h - 1)
        return np.where(inside, field[rr, cc], out)

    # ── 車体の余裕 ──

    def body_clearance(self, x: np.ndarray | float, y: np.ndarray | float,
                       yaw: np.ndarray | float, *,
                       include_unknown: bool = False) -> np.ndarray:
        """姿勢`(x,y,yaw)`に車体を置いたときの余裕 [m]（負なら食い込んでいる）。

        円被覆の各円について `ESDF − 半径` を取り、その最小値。**姿勢を
        配列で渡せる**（Hybrid A*が数千ノードを一括で判定するため）。

        ★ **全ての円をまとめて1回の参照で引く。** 円ごとに`clearance()`を
        呼ぶ実装だと、numpyの呼び出しオーバヘッド（1回20µs程度）が円の数だけ
        掛かり、Hybrid A*の1展開が0.2msを超えて探索が秒オーダーになる
        （2026-09-16の実測で探索が最大10秒かかり、ここが主因だった）。
        """
        field = self._blocked_edt if include_unknown else self._wall_edt
        scalar = np.isscalar(x)
        xa = np.atleast_1d(np.asarray(x, dtype=np.float64))
        ya = np.atleast_1d(np.asarray(y, dtype=np.float64))
        yawa = np.atleast_1d(np.asarray(yaw, dtype=np.float64))
        if field is None:
            out = np.zeros(xa.shape)
            return float(out[0]) if scalar else out

        ox = self._circles[None, :, 0]                     # (1, K)
        oy = self._circles[None, :, 1]
        c = np.cos(yawa)[:, None]
        s = np.sin(yawa)[:, None]
        px = xa[:, None] + ox * c - oy * s                 # (N, K)
        py = ya[:, None] + ox * s + oy * c
        col, row = self.grid.to_cell(px, py)
        h, w = field.shape
        inside = (col >= 0) & (col < w) & (row >= 0) & (row < h)
        vals = field[np.clip(row, 0, h - 1), np.clip(col, 0, w - 1)]
        vals = np.where(inside, vals, 0.0)                 # 格子の外は塞がれている扱い
        out = vals.min(axis=1) - self._circle_r
        return float(out[0]) if scalar else out
    def path_clearance(self, poses, *, include_unknown: bool = False) -> float:
        """姿勢列に沿った最小余裕 [m]。`(N,3)`配列でもタプルのリストでも受ける
        （`reeds_shepp.sample_path_array()`を渡すのが速い）。"""
        arr = np.asarray(poses, dtype=np.float64)
        if arr.size == 0:
            return math.inf
        return float(self.body_clearance(arr[:, 0], arr[:, 1], arr[:, 2],
                                         include_unknown=include_unknown).min())
