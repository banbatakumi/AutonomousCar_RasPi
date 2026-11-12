"""照合の方式・物差し（LiDAR との比較）・集計。画面を持たない純ロジック。"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field, replace

import numpy as np

from raspi.core.stereo_model import StereoRectifier

from .data import StereoFrame

__all__ = ["MatcherConfig", "PRESETS", "Geometry", "compute_disparity", "compare_with_lidar",
           "BinCompare", "ConfigResult", "run", "wall_check"]

#: 平行化画像の水平画角（`tools/cam_calib` の合否と同じ値）
RECT_HFOV_RAD = math.radians(100)


@dataclass(frozen=True)
class MatcherConfig:
    name: str
    algo: str                         #: "bm" / "sgbm"
    #: 平行化画像（640 幅）に対する縮尺。0.5 なら 320 幅で照合する
    scale: float = 1.0
    #: 視差の探索幅（縮めた画像の画素。16 の倍数）。最も近い距離は f·B/視差（640 幅・96 で約 0.21m）
    num_disp: int = 96
    block: int = 15
    #: 照合する平行化画像の行 `(v0, v1)`（縮める前の座標）。None は全部
    rows: tuple[int, int] | None = None


#: 比べる組の既定。block は BM が大きめ（模様の少ない所で誤対応を減らす）、SGBM は小さめ（平滑化は SGM が担う）
PRESETS: dict[str, MatcherConfig] = {
    "bm": MatcherConfig("bm", "bm", 1.0, 96, 15),
    "sgbm": MatcherConfig("sgbm", "sgbm", 1.0, 96, 5),
    "bm-half": MatcherConfig("bm-half", "bm", 0.5, 48, 9),
    "sgbm-half": MatcherConfig("sgbm-half", "sgbm", 0.5, 48, 3),
}


@dataclass
class Geometry:
    """平行化と、右カメラ・LiDAR の取付。"""

    rect: StereoRectifier
    #: 右（主）カメラの取付 `(x, y, z, pitch, yaw)`（base_link）
    cam_pose: tuple[float, float, float, float, float]
    #: LiDAR の取付 `(x, y, z, yaw)`
    lidar_pose: tuple[float, float, float, float]

    @classmethod
    def from_vehicle(cls, v, hfov: float = RECT_HFOV_RAD) -> "Geometry":
        if v.stereo is None or v.cam_front_fisheye is None or v.cam_left_fisheye is None:
            raise ValueError("vehicle.toml にステレオの校正値が無い（tools/cam_calib の「ステレオ」）")
        rect = StereoRectifier(v.cam_front_fisheye, v.cam_left_fisheye, v.stereo, 640, 480, hfov,
                               crop_right=v.cam_front_bottom_crop, crop_left=v.cam_left_bottom_crop)
        return cls(rect, (v.cam_front_x, v.cam_front_y, v.cam_front_z, v.cam_front_pitch,
                          v.cam_front_yaw),
                   (v.lidar_x, v.lidar_y, v.lidar_z, v.lidar_yaw))


def _match_params(cfg: MatcherConfig):
    from raspi.core.stereo_match import MatchParams

    return MatchParams(cfg.algo, cfg.scale, cfg.num_disp, cfg.block)


def _matcher(cfg: MatcherConfig):
    """照合器（`raspi/core/stereo_match.py` の `make_matcher`。Pi の `stereo_node` と同じ設定）。"""
    from raspi.core.stereo_match import make_matcher

    return make_matcher(_match_params(cfg))


def compute_disparity(cfg: MatcherConfig, matcher, rect_left: np.ndarray,
                      rect_right: np.ndarray) -> tuple[np.ndarray, int]:
    """平行化済みの左右（グレースケール）→ `(視差 [px、縮めた画像の単位], 1行目の行番号)`。無効は −1。"""
    from raspi.core.stereo_match import disparity

    v0, v1 = cfg.rows or (0, rect_left.shape[0])
    return disparity(_match_params(cfg), matcher, rect_left[v0:v1], rect_right[v0:v1]), v0


@dataclass
class BinCompare:
    """1組ぶんの LiDAR との比較（1° の方位ごと）。"""

    deg: np.ndarray                   #: 車両角 [deg]（LiDAR の添字）
    r_stereo: np.ndarray              #: ステレオの距離（LiDAR 原点から、方位内の中央値）[m]
    r_lidar: np.ndarray
    n_points: np.ndarray              #: 方位内のステレオの点の数


def stereo_ranges(geom: Geometry, pts_rect: np.ndarray, *, band_m: float = 0.02,
                  r_min: float = 0.15, r_max: float = 3.0, min_points: int = 5):
    """ステレオの点のうち LiDAR の走査面 ±`band_m` にあるものを、LiDAR から見た 1° の方位ごとの距離に。

    `(deg, r, n)`。方位の中の点が `min_points` 未満のものは除く。
    """
    lx, ly, lz, lyaw = geom.lidar_pose
    P = geom.rect.rect_to_base(pts_rect.reshape(-1, 3), *geom.cam_pose)
    ok = np.isfinite(P[:, 2]) & (np.abs(P[:, 2] - lz) <= band_m)
    P = P[ok]
    dx, dy = P[:, 0] - lx, P[:, 1] - ly
    r = np.hypot(dx, dy)
    keep = (r >= r_min) & (r <= r_max)
    r = r[keep]
    deg = np.round(np.degrees(np.arctan2(dy[keep], dx[keep]) - lyaw)).astype(int) % 360
    if r.size == 0:
        return np.zeros(0, int), np.zeros(0), np.zeros(0, int)
    order = np.argsort(deg, kind="stable")
    deg, r = deg[order], r[order]
    uniq, start, counts = np.unique(deg, return_index=True, return_counts=True)
    med = np.array([np.median(r[s:s + c]) for s, c in zip(start, counts)])
    sel = counts >= min_points
    return uniq[sel], med[sel], counts[sel]


def compare_with_lidar(geom: Geometry, pts_rect: np.ndarray, scan: np.ndarray,
                       **kw) -> BinCompare:
    deg, r, n = stereo_ranges(geom, pts_rect, **kw)
    rl = scan[deg] if deg.size else np.zeros(0)
    ok = rl > 0
    return BinCompare(deg[ok], r[ok], rl[ok], n[ok])


@dataclass
class ConfigResult:
    cfg: MatcherConfig
    times_ms: list[float] = field(default_factory=list)
    valid_frac: list[float] = field(default_factory=list)
    #: 全組の比較をつないだもの（相対誤差 (ステレオ − LiDAR)/LiDAR）
    rel_err: list[np.ndarray] = field(default_factory=list)
    r_lidar: list[np.ndarray] = field(default_factory=list)

    def summary(self) -> dict:
        e = np.concatenate(self.rel_err) if self.rel_err else np.zeros(0)
        rl = np.concatenate(self.r_lidar) if self.r_lidar else np.zeros(0)
        a = np.abs(e)

        def near(lo, hi):
            s = (rl >= lo) & (rl < hi)
            return float(np.median(a[s])) if s.any() else math.nan
        return {
            "name": self.cfg.name,
            "ms": float(np.median(self.times_ms)) if self.times_ms else math.nan,
            "valid": float(np.mean(self.valid_frac)) if self.valid_frac else math.nan,
            "bins": int(e.size),
            "bias": float(np.median(e)) if e.size else math.nan,
            "abs_med": float(np.median(a)) if e.size else math.nan,
            "abs_p90": float(np.percentile(a, 90)) if e.size else math.nan,
            "within5": float(np.mean(a <= 0.05)) if e.size else math.nan,
            "abs_med_lt1": near(0.0, 1.0),
            "abs_med_1to2": near(1.0, 2.0),
            "abs_med_ge2": near(2.0, 99.0),
        }


def _requality(img: np.ndarray, q: int | None) -> np.ndarray:
    if q is None:
        return img
    import cv2

    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, int(q)])
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)


def rectify_gray(geom: Geometry, fr: StereoFrame, requality: int | None = None):
    """`(左, 右)` の平行化したグレースケール。"""
    import cv2

    r, l_ = fr.decode()
    r, l_ = _requality(r, requality), _requality(l_, requality)
    rr, rl = geom.rect(r, l_)
    return cv2.cvtColor(rl, cv2.COLOR_BGR2GRAY), cv2.cvtColor(rr, cv2.COLOR_BGR2GRAY)


def run(frames: list[StereoFrame], geom: Geometry, configs: list[MatcherConfig], *,
        requality: int | None = None, band_m: float = 0.02, max_speed: float | None = 0.05,
        save=None) -> list[ConfigResult]:
    """全組 × 全方式を照合して集計する。`save(i, cfg, rect_left, disp)` は絵を残す呼び出し（任意）。

    LiDAR との比較は、点群があり、車速が `max_speed` 以下（記録に車速が無ければ問わない）の組だけ。
    """
    results = [ConfigResult(c) for c in configs]
    matchers = [_matcher(c) for c in configs]
    for i, fr in enumerate(frames):
        gl, gr = rectify_gray(geom, fr, requality)
        still = max_speed is None or fr.speed is None or abs(fr.speed) <= max_speed
        for res, m in zip(results, matchers):
            t0 = time.perf_counter()
            d, v0 = compute_disparity(res.cfg, m, gl, gr)
            res.times_ms.append((time.perf_counter() - t0) * 1000.0)
            # 左端の探索幅ぶんは原理的に視差が出ないので、割合の分母から外す
            res.valid_frac.append(float(np.mean(d[:, res.cfg.num_disp:] > 0)))
            if save is not None:
                save(i, res.cfg, gl, d)
            if fr.scan is None or not still:
                continue
            pts = geom.rect.points_from_disparity(d, v0=v0, scale=res.cfg.scale)
            bc = compare_with_lidar(geom, pts, fr.scan, band_m=band_m)
            res.rel_err.append((bc.r_stereo - bc.r_lidar) / bc.r_lidar)
            res.r_lidar.append(bc.r_lidar)
    return results


def wall_check(frames: list[StereoFrame], geom: Geometry, configs: list[MatcherConfig],
               wall_m: float, *, half_width_deg: int = 2, requality: int | None = None,
               band_m: float = 0.02) -> dict:
    """物差しの確認: 正面 ±`half_width_deg` の LiDAR とステレオの距離（中央値）を壁の距離と比べる。

    `wall_m` は **LiDAR の中心から壁まで**の距離（正面）。返り値は `{"lidar": [...], 方式名: [...]}`（組ごとの値）。
    """
    degs = np.arange(-half_width_deg, half_width_deg + 1) % 360
    out: dict[str, list[float]] = {"lidar": []}
    for c in configs:
        out[c.name] = []
    matchers = [_matcher(c) for c in configs]
    for fr in frames:
        if fr.scan is None:
            continue
        s = fr.scan[degs]
        out["lidar"].append(float(np.median(s[s > 0])) if (s > 0).any() else math.nan)
        gl, gr = rectify_gray(geom, fr, requality)
        for c, m in zip(configs, matchers):
            d, v0 = compute_disparity(c, m, gl, gr)
            deg, r, _n = stereo_ranges(geom, geom.rect.points_from_disparity(d, v0=v0, scale=c.scale),
                                       band_m=band_m)
            sel = np.isin(deg, degs)
            out[c.name].append(float(np.median(r[sel])) if sel.any() else math.nan)
    out["wall_m"] = [wall_m]
    return out


def with_rows(cfg: MatcherConfig, rows: tuple[int, int] | None) -> MatcherConfig:
    return replace(cfg, rows=rows) if rows is not None else cfg
