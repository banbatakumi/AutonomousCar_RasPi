"""LiDAR の点群からノイズの点を捨てる — **走行モードに依らない**前処理（`planning_node` が `mask_body` の後に掛ける）。

2026-11-08 の試走（toyota の会場）で、ショートカットの後のヘアピンで毎回点群にノイズが乗り、slam2d の
「障害物を避ける」を入れるとそこで毎回止まった。窓からの日光が疑わしいが、記録に強度が無く断定できていない。
原因が何であっても効くよう、**地図を使わず LiDAR とオドメトリだけ**で判定する（slam2d 以外のモードにも効く）。

## 捨てる点

点は次の 3 つの性質で見る:

- **孤立**: 両隣の点（1°ずつ）と距離が大きく食い違う（`speckle_m` + 距離 × `speckle_rel`）。
  本物の物は隣り合う点が連なる
- **再現しない**: 前の周の点を車速とヨーレートで今の車体の位置へずらしたとき、同じ向きの近く（±`persist_bins`°）に
  対応する点が無い（`persist_m` + 距離 × `persist_rel`）。日光のノイズは距離がばらばらで続けて同じ所に出ない
- **強度が低い**（強度付きの形式 `LIDAR_FORMAT=1` のときだけ）: 下の自動の下限より弱い

捨てるのは「孤立 **かつ** 再現しない」点と、強度があれば「強度が低い **かつ**（孤立 **または** 再現しない）」点。
1 つの性質だけでは捨てない: 遠い壁を掠める点は孤立に見え、動く他車の点は再現しないように見え、黒い幕の壁は強度が低い。

## 強度の下限は自動で決める

孤立せず再現もした「確かな点」の強度の分布の下の方（`intensity_q` 分位）× `intensity_scale` を、周ごとに
指数平均で追う。床・壁の材質や明るさが会場で変わっても合わせられる（地図は使わない）。`intensity_min` は手で決める下限
（自動の値がこれより下がらない）。

## 消した点は「測れなかった」（`dist = 0`）

「空き」にはしない（`raspi/nav/deskew.py` の「3種類の点を区別する」）。生の `/scan` は記録に残るので、
捨て方は後から記録で確かめ直せる（`ScanFilter` に記録の点群を順に通せばよい）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, fields

import msgspec
import numpy as np

from ..msgs.types import Scan, VehicleState

__all__ = ["ScanFilterConfig", "ScanFilter", "config_from_items"]


@dataclass(frozen=True)
class ScanFilterConfig:
    enabled: bool = True
    #: これより遠い点は判定しない（残す）[m]。ノイズで困るのは車の近くの障害物の判定で、遠い点は SLAM の位置合わせが
    #: Huber で弱める。遠くの点は 1° の間隔が広く、壁を掠めると孤立に見えるので、捨てると位置合わせの点が減るだけ
    #: （2026-11-08 の記録で、絞らないと捨てた点の 7 割が 5m より遠かった）
    max_range_m: float = 3.0
    #: 孤立の判定: 両隣との距離の差がこれ + 距離 × `speckle_rel` を超える [m]
    speckle_m: float = 0.15
    speckle_rel: float = 0.05
    #: 再現の判定: 前の周の点との距離の差がこれ + 距離 × `persist_rel` 以内なら再現した [m]
    persist_m: float = 0.08
    persist_rel: float = 0.03
    #: 再現の判定で見る、向きの幅（±この度数）。ヨーレートの推定のずれ・1°の量子化を飲む
    persist_bins: int = 2
    #: 前の周がこれより古ければ再現の判定をしない（点群が途切れた後）[s]
    persist_max_dt: float = 0.25
    #: 強度の自動の下限 = 確かな点の強度のこの分位 × `intensity_scale`
    intensity_q: float = 0.02
    intensity_scale: float = 0.6
    #: 手で決める強度の下限（自動の値がこれより下がらない）。0 で自動だけ
    intensity_min: float = 0.0
    #: 自動の下限の指数平均の重み（1 周あたり）
    intensity_alpha: float = 0.05


def config_from_items(items) -> ScanFilterConfig:
    """`config/vehicle.toml` の `[sensors.lidar.filter]`（`Vehicle.lidar_filter`）から作る。知らないキーは無視する。"""
    known = {f.name: f.type for f in fields(ScanFilterConfig)}
    kw = {}
    for k, v in dict(items or ()).items():
        if k not in known:
            continue
        if k == "enabled":
            kw[k] = bool(v)
        elif k == "persist_bins":
            kw[k] = int(v)
        else:
            kw[k] = float(v)
    return ScanFilterConfig(**kw)


_ANG = np.radians(np.arange(360))
_COS, _SIN = np.cos(_ANG), np.sin(_ANG)


class ScanFilter:
    """周ごとに `apply()` を呼ぶ（前の周を覚えているので、点群 1 本の流れに 1 つ）。"""

    def __init__(self, config: ScanFilterConfig = ScanFilterConfig(), *,
                 lidar_x: float = 0.0, lidar_y: float = 0.0) -> None:
        self.config = config
        self._lx, self._ly = float(lidar_x), float(lidar_y)
        self._prev: np.ndarray | None = None     #: 前の周の距離（360、0 = 無効）
        self._prev_t = 0
        #: 強度の自動の下限（まだ決まっていなければ 0）
        self.threshold = 0.0
        #: 直近の周に捨てた点の数・捨てた点のうち強度で捨てた数（診断）
        self.dropped = 0
        self.dropped_intensity = 0
        self.scans = 0

    def reset(self) -> None:
        self._prev = None
        self._prev_t = 0

    def apply(self, scan: Scan, vs: VehicleState | None) -> Scan:
        """ノイズの点を `dist = 0` にした `scan` を返す（捨てる点が無ければ `scan` そのもの）。"""
        cfg = self.config
        d = np.asarray(scan.dist, dtype=np.float64)
        if not cfg.enabled or d.shape != (360,):
            self.dropped = self.dropped_intensity = 0
            return scan
        self.scans += 1
        valid = d > 0.0
        iso = self._isolated(d, valid)
        persist = self._persistent(d, valid, scan.t_capture, vs)
        valid = valid & (d <= cfg.max_range_m)
        drop = valid & iso & ~persist
        inten = scan.intensity
        low = np.zeros(360, dtype=bool)
        if inten is not None and len(inten) == 360:
            iv = np.asarray(inten, dtype=np.float64)
            sure = valid & ~iso & persist
            if sure.sum() >= 30:
                q = float(np.quantile(iv[sure], cfg.intensity_q)) * cfg.intensity_scale
                a = cfg.intensity_alpha
                self.threshold = q if self.threshold <= 0.0 else self.threshold + a * (q - self.threshold)
            thr = max(self.threshold, cfg.intensity_min)
            if thr > 0.0:
                low = valid & (iv < thr) & (iso | ~persist)
        all_drop = drop | low
        self.dropped = int(all_drop.sum())
        self.dropped_intensity = int((low & ~drop).sum())
        # 再現の判定に使う前の周は、捨てる前の点（捨てた点も次の周の照合相手にはなる。続けて出たら本物）
        self._prev = d.copy()
        self._prev_t = int(scan.t_capture)
        if not all_drop.any():
            return scan
        out = d.copy()
        out[all_drop] = 0.0
        sat = scan.saturated
        if sat is not None:
            sat = [bool(s) and not k for s, k in zip(sat, all_drop)]
        if inten is not None and len(inten) == 360:
            inten = [0 if k else int(v) for v, k in zip(inten, all_drop)]
        return msgspec.structs.replace(scan, dist=out.tolist(), saturated=sat, intensity=inten)

    def _isolated(self, d: np.ndarray, valid: np.ndarray) -> np.ndarray:
        cfg = self.config
        tol = cfg.speckle_m + cfg.speckle_rel * d
        out = np.ones(360, dtype=bool)
        for k in (-1, 1):
            nb = np.roll(d, k)
            nv = np.roll(valid, k)
            out &= ~(nv & (np.abs(nb - d) <= tol))
        return out & valid

    def _persistent(self, d: np.ndarray, valid: np.ndarray, t_ns: int,
                    vs: VehicleState | None) -> np.ndarray:
        """前の周を今の車体の位置へずらして、同じ向き（±`persist_bins`°）の近くに点があるか。
        判定できない（前の周が無い・古い）ときは全部「再現した」（＝この性質では捨てない）。"""
        cfg = self.config
        prev = self._prev
        dt = (int(t_ns) - self._prev_t) / 1e9 if self._prev_t and t_ns else -1.0
        if prev is None or not (0.0 < dt <= cfg.persist_max_dt):
            return np.ones(360, dtype=bool)
        v = float(vs.speed) if vs is not None and math.isfinite(vs.speed) else 0.0
        w = float(vs.yaw_rate) if vs is not None and math.isfinite(vs.yaw_rate) else 0.0
        dth = w * dt
        # 前の周の車体から今の車体への動き（円弧を弦で近似）
        dx = v * dt * math.cos(0.5 * dth)
        dy = v * dt * math.sin(0.5 * dth)
        pv = prev > 0.0
        px = self._lx + prev[pv] * _COS[pv] - dx
        py = self._ly + prev[pv] * _SIN[pv] - dy
        c, s = math.cos(-dth), math.sin(-dth)
        qx = c * px - s * py - self._lx
        qy = s * px + c * py - self._ly
        r = np.hypot(qx, qy)
        b = np.round(np.degrees(np.arctan2(qy, qx))).astype(np.int64) % 360
        # 向きごとの前の周の距離（ずらすと同じ向きに 2 点入ることがあるので、近い方と遠い方の 2 つを持つ）
        near = np.full(360, np.inf)
        far = np.full(360, -np.inf)
        np.minimum.at(near, b, r)
        np.maximum.at(far, b, r)
        tol = cfg.persist_m + cfg.persist_rel * d
        ok = np.zeros(360, dtype=bool)
        for k in range(-cfg.persist_bins, cfg.persist_bins + 1):
            n_k = np.roll(near, -k)
            f_k = np.roll(far, -k)
            ok |= (np.abs(n_k - d) <= tol) | (np.abs(f_k - d) <= tol)
        return ok | ~valid
