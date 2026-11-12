"""ステレオの照合と、障害物の点 → 擬似 `Scan`。Pi の `stereo_node` と Mac の `tools/stereo_bench` が共有する。

    from raspi.core.stereo_match import MatchParams, make_matcher, disparity, obstacle_scan

平行化と座標の約束は `raspi/core/stereo_model.py`。計画は `docs/stereo_plan.md`（段階3・4）。
cv2 はここで遅延 import する（`StereoRectifier` と同じ。テストの一部は cv2 なしで回る）。

## 障害物の擬似 `Scan` の約束（`obstacle_scan`）

- 原点は **base_link**（後輪車軸中心）。添字 i が車両角 i 度（x 前 = 0、反時計回りが正）。`scan/cam` と同じ
- 値は「床より `z_min` 上〜 `z_max` 以下の高さに、距離 `r_min`〜`r_max` で点がまとまって見つかった」方位の、
  そのかたまりの近い方からの `near_pct` パーセント点の距離。かたまりは、1° の方位の点を距離 `slice_m` 刻みで
  近い方から見て、隣の刻みと合わせて `min_points` 点以上あり、**高さの幅（10〜90% 点）が `min_height_span` 以上**の
  最初のもの。高さの幅を見るのは、斜めに見える床の取り違えが「床より数 cm 上に、高さのそろった点の列」として
  出るため（合成画像で、床の誤対応は幅 1.1cm 以下、高さ 10cm の箱は 3〜6cm）。代わりに、高さ
  `z_min + min_height_span`（既定 5cm）より低い物は見つけられない
- **見つからない方位は 0.0**。これは LiDAR の「反射が返らない」と同じ書き方で、**空きという意味ではない**
  （模様の無い物はステレオに写らない）。ステレオは障害物を**足すだけ**の入力として使う
- `sector_seen` は平行化画像の視野（水平 `hfov`）に1°でも入るセクタだけ True
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

__all__ = ["MatchParams", "make_matcher", "to_gray", "disparity", "ObstacleParams",
           "obstacle_scan", "fov_degrees", "colorize_disparity", "DisplayFiller",
           "obstacles_from_disparity", "obstacles_from_disparity_numpy"]


@dataclass(frozen=True)
class MatchParams:
    """照合の設定。既定は 640 幅の StereoSGBM（3WAY・ブロック 5）。

    段階3 の物差しの壁（`docs/stereo_plan.md`）で決めた: StereoBM は無地の面で取り違え、1.5m の壁を 0.7m に出す組があった。
    SGBM は視差の出る割合 97%・距離 ±1〜3%。代わりに Pi 5 で照合が約 2 倍かかる。
    """

    algo: str = "sgbm"                #: "bm" / "sgbm"
    #: 平行化画像（640 幅）に対する縮尺。0.5 なら 320 幅で照合する（距離の誤差は倍になる）
    scale: float = 1.0
    #: 視差の探索幅（縮めた画像の画素。16 の倍数）。最も近い距離は f·B/視差（640 幅・96 で約 0.21m）
    num_disp: int = 96
    #: 照合の窓の一辺 [px]（奇数）。SGBM は 5、BM なら 15 前後
    block: int = 5


def make_matcher(p: MatchParams):
    import cv2

    if p.algo == "bm":
        m = cv2.StereoBM_create(numDisparities=p.num_disp, blockSize=p.block)
        m.setTextureThreshold(10)
        m.setUniquenessRatio(10)
        m.setSpeckleWindowSize(50)
        m.setSpeckleRange(2)
        return m
    if p.algo == "sgbm":
        b = p.block
        return cv2.StereoSGBM_create(minDisparity=0, numDisparities=p.num_disp, blockSize=b,
                                     P1=8 * b * b, P2=32 * b * b, disp12MaxDiff=1,
                                     uniquenessRatio=10, speckleWindowSize=50, speckleRange=2,
                                     mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
    raise ValueError(f"知らない照合の方式: {p.algo}")


def to_gray(img: np.ndarray, fmt: str = "BGR888") -> np.ndarray:
    """共有メモリの画像 → グレースケール。`fmt` は**メモリ上の実際のバイト順**（`ImageRef.fmt`）。"""
    import cv2

    if img.ndim == 2:
        return img
    if img.shape[2] == 4:
        # 4 バイト形式は先頭 3 バイトを色として使う（並びの違いは灰色の重みが少し変わるだけで、照合には効かない）
        img = img[:, :, :3]
    code = cv2.COLOR_RGB2GRAY if fmt.startswith("RGB") else cv2.COLOR_BGR2GRAY
    return cv2.cvtColor(np.ascontiguousarray(img), code)


def disparity(p: MatchParams, matcher, rect_left: np.ndarray, rect_right: np.ndarray) -> np.ndarray:
    """平行化済みの左右（グレースケール、同じ大きさ）→ 視差 [px、縮めた画像の単位]。無効は −1。"""
    import cv2

    l_, r = rect_left, rect_right
    if p.scale != 1.0:
        size = (int(round(l_.shape[1] * p.scale)), int(round(l_.shape[0] * p.scale)))
        l_ = cv2.resize(l_, size, interpolation=cv2.INTER_AREA)
        r = cv2.resize(r, size, interpolation=cv2.INTER_AREA)
    d = matcher.compute(l_, r).astype(np.float32) / 16.0
    d[d <= 0] = -1.0
    return d


@dataclass(frozen=True)
class ObstacleParams:
    #: 障害物とみなす高さ [m]（路面から）。下は床の誤検出を避ける余白、上は計画の「20cm 以下」
    z_min: float = 0.03
    z_max: float = 0.20
    #: 距離の範囲 [m]（base_link 原点から）。遠いほど誤差が増える（2m で ±4cm 程度）
    r_min: float = 0.15
    r_max: float = 2.0
    #: 1° の方位で障害物とみなす点の数の下限（ばらばらの誤対応を捨てる）
    min_points: int = 4
    #: 方位の中の距離の何パーセント点を使うか（近い側。中央値だと物の手前の縁より奥に出る）
    near_pct: float = 20.0
    #: かたまりの高さの幅（10〜90% 点）の下限 [m]。床の取り違えを捨てる（モジュールの docstring）
    min_height_span: float = 0.02
    #: かたまりを探す距離の刻み [m]
    slice_m: float = 0.05


def fov_degrees(rect, cam_pose) -> list[int]:
    """平行化画像の左端〜右端の列が base_link で向く車両角 [deg] の範囲（整数のリスト）。"""
    from .stereo_model import camera_to_base_rotation

    R = camera_to_base_rotation(cam_pose[3], cam_pose[4]) @ rect.R_rect.T
    yaws = []
    for u in (0.0, rect.out_w - 1.0):
        ray = np.array([(u - rect.cx) / rect.f, 0.0, 1.0]) @ R.T
        yaws.append(math.degrees(math.atan2(ray[1], ray[0])))
    lo, hi = math.ceil(min(yaws)), math.floor(max(yaws))
    return [d % 360 for d in range(lo, hi + 1)]


@dataclass
class ObstacleResult:
    dist: list[float]
    sector_seen: list[bool]
    #: 障害物とみなした点の数（診断用）
    n_points: int = 0
    #: 視差の出た画素の割合（左端の探索幅は除く）
    valid_frac: float = 0.0
    extra: dict = field(default_factory=dict)


def obstacle_scan(pts_base: np.ndarray, fov_degs: list[int], q: ObstacleParams = ObstacleParams()
                  ) -> ObstacleResult:
    """base_link の点 (..., 3) → 障害物の擬似 `Scan` の中身（モジュールの docstring の約束）。"""
    from raspi.auto.base import sector_of_deg

    P = pts_base.reshape(-1, 3)
    ok = np.isfinite(P[:, 2]) & (P[:, 2] >= q.z_min) & (P[:, 2] <= q.z_max)
    P = P[ok]
    r = np.hypot(P[:, 0], P[:, 1])
    keep = (r >= q.r_min) & (r <= q.r_max)
    r, P = r[keep], P[keep]
    dist = [0.0] * 360
    if r.size:
        z = P[:, 2]
        deg = np.round(np.degrees(np.arctan2(P[:, 1], P[:, 0]))).astype(int) % 360
        order = np.lexsort((r, deg))                   # 方位ごと・近い順
        deg, r, z = deg[order], r[order], z[order]
        uniq, start, counts = np.unique(deg, return_index=True, return_counts=True)
        for d, s, c in zip(uniq.tolist(), start.tolist(), counts.tolist()):
            if c >= q.min_points:
                got = _nearest_cluster(r[s:s + c], z[s:s + c], q)
                if got is not None:
                    dist[d] = got
    seen = [False] * 12
    for d in fov_degs:
        seen[sector_of_deg(d)] = True
    return ObstacleResult(dist, seen, int(r.size))


def obstacles_from_disparity(rect, d: np.ndarray, cam_pose, fov_degs: list[int],
                             q: ObstacleParams = ObstacleParams(), *, scale: float = 1.0,
                             skip_cols: int = 0, native: bool = True) -> ObstacleResult:
    """視差（`disparity` の結果、行の帯）→ 障害物の擬似 `Scan` の中身。`stereo_node` の後処理の全部。

    C 版（`raspi/core/stereo_native.c`）が使えればそちら、使えなければ numpy 版（`obstacles_from_disparity_numpy`）。
    結果は同じ（`raspi/tests/test_stereo_native.py`）。Pi 5 で numpy 版は 1 組 約 12ms かかっていた。
    """
    if native:
        from .stereo_model import camera_to_base_rotation
        from .stereo_native import native_obstacles

        R = camera_to_base_rotation(cam_pose[3], cam_pose[4]) @ rect.R_rect.T
        got = native_obstacles(rect, d, R, np.asarray(cam_pose[:3], np.float64), q,
                               scale=scale, skip_cols=skip_cols)
        if got is not None:
            from raspi.auto.base import sector_of_deg

            dist, n_points, n_valid = got
            seen = [False] * 12
            for deg in fov_degs:
                seen[sector_of_deg(deg)] = True
            w = d.shape[1] - skip_cols
            return ObstacleResult(dist, seen, n_points,
                                  valid_frac=n_valid / (d.shape[0] * w) if w > 0 else 0.0)
    return obstacles_from_disparity_numpy(rect, d, cam_pose, fov_degs, q, scale=scale,
                                          skip_cols=skip_cols)


def obstacles_from_disparity_numpy(rect, d: np.ndarray, cam_pose, fov_degs: list[int],
                                   q: ObstacleParams = ObstacleParams(), *, scale: float = 1.0,
                                   skip_cols: int = 0) -> ObstacleResult:
    """numpy 版。視差の出た画素だけを点にし（`StereoRectifier.points_from_pixels`）、base_link へ変換して
    `obstacle_scan` に渡す。`valid_frac` は左端の `skip_cols` 列（探索幅。原理的に照合できない）を除いた割合。
    """
    v, u = np.nonzero(d > 0)
    pts = rect.points_from_pixels(u, v, d[v, u], scale=scale)
    res = obstacle_scan(rect.rect_to_base(pts, *cam_pose), fov_degs, q)
    w = d.shape[1] - skip_cols
    res.valid_frac = float(np.count_nonzero(d[:, skip_cols:] > 0)) / (d.shape[0] * w) if w > 0 else 0.0
    return res


def _q(sorted_vals: np.ndarray, pct: float) -> float:
    """並べ替え済みの配列の `pct` パーセント点（近い順位の値。補間しない）。

    `np.percentile` を使わないのは、方位ごと・刻みごとに何百回も呼ぶと Pi では呼び出しの手間だけで
    数十 ms になるため（Pi 5 で抽出が 38ms、Mac の 27 倍だった）。
    """
    return float(sorted_vals[int(round(pct / 100.0 * (len(sorted_vals) - 1)))])


def _nearest_cluster(r: np.ndarray, z: np.ndarray, q: ObstacleParams) -> float | None:
    """近い順の距離 `r` と高さ `z`（1° の方位の点）→ 最初の「縦に広がったかたまり」の距離。無ければ None。"""
    k = ((r - r[0]) / q.slice_m).astype(np.int64)      # r は昇順なので k も昇順
    for kk in np.unique(k).tolist():
        # 刻みの境目で物が割れないよう、隣の刻みと合わせる（昇順なので連続した区間になる）
        lo, hi = np.searchsorted(k, kk, "left"), np.searchsorted(k, kk + 2, "left")
        if hi - lo < q.min_points:
            continue
        zs = np.sort(z[lo:hi])
        if _q(zs, 90.0) - _q(zs, 10.0) >= q.min_height_span:
            return _q(r[lo:hi], q.near_pct)
    return None


def colorize_disparity(d: np.ndarray, num_disp: int) -> np.ndarray:
    """視差 → BGR（近い = 赤、遠い = 青、視差なし = 黒）。"""
    import cv2

    v = np.clip(d / float(num_disp) * 255.0, 0, 255).astype(np.uint8)
    img = cv2.applyColorMap(v, cv2.COLORMAP_JET)
    img[d <= 0] = 0
    return img


class DisplayFiller:
    """**表示専用**の穴埋め（照合できなかった黒い所を、周りの視差で画像の縁を守りながら埋める）。

    OpenCV の WLS フィルタ（`cv2.ximgproc`、案内画像 = 平行化した左）は入力について線形なので、
    「有効画素だけの視差（無効は 0）」と「有効の目印（0/1）」をそれぞれ通して割る（正規化した WLS）。
    無効の画素（値 −1）をそのまま通すと「値」として平均に混ざり、ほとんど埋まらない（合成画像で 39% → 45%）。
    右の視差も計算する信頼度付きの WLS は照合が倍になるので使わない。

    **埋めた値は測った値ではなく推測**なので、障害物の判定（`obstacle_scan`）には使わない。周りに有効な画素が
    無い所（左端の探索幅の帯・照合が全く効かない広い面）は `min_weight` で黒のまま残す。
    """

    def __init__(self, lam: float = 8000.0, sigma_color: float = 1.0, min_weight: float = 0.02) -> None:
        import cv2

        self._wls = cv2.ximgproc.createDisparityWLSFilterGeneric(False)
        self._wls.setLambda(lam)
        self._wls.setSigmaColor(sigma_color)
        self.min_weight = min_weight

    #: 目印の値（int16 に収め、割り算の分解能を確保する）
    _ONE = 1024

    def __call__(self, d: np.ndarray, guide: np.ndarray) -> np.ndarray:
        """視差（無効 −1）と案内画像（同じ大きさの灰色）→ 埋めた視差（埋まらない所は −1）。"""
        valid = d > 0
        num = np.where(valid, d * 16.0, 0.0).astype(np.int16)
        den = (valid * self._ONE).astype(np.int16)
        a = self._wls.filter(num, guide).astype(np.float32)
        b = self._wls.filter(den, guide).astype(np.float32)
        ok = b > self._ONE * self.min_weight
        out = np.full(d.shape, -1.0, np.float32)
        out[ok] = a[ok] / b[ok] * (self._ONE / 16.0)
        out[valid] = d[valid]                      # 測った所は測った値のまま
        return out
