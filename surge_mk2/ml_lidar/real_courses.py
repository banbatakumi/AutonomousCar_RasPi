"""実機で作った地図（`saved_maps/magp_*.npz`、`raspi.auto.mapstore`形式）を学習・観戦用の`Course`にする。

- 壁 = 占有 **と未知**（trinary != 1）。LiDARの届かなかった所は走れない所として扱う。
- 中心線 = 地図の`centerline_xy`と走行軌跡`routes_json["explore_traj"]`のうち長い方
  （a/a3の`centerline_xy`は断片、a2・bは空）。走行軌跡は閉じきらず数m空くことがあるが、
  進捗の射影は閉じ線分を含めて扱える。
- スタート姿勢 = 中心線の先頭と、その少し先への向き。
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from sim.course import Course

REPO_ROOT = Path(__file__).resolve().parents[1]
MAPS_DIR = REPO_ROOT / "saved_maps"
PREFIX = "magp_"

#: 中心線の点間隔[m]（env の進捗窓幅はこの平均間隔から決まる）
_SPACING_M = 0.1


def list_real_course_names() -> list[str]:
    return sorted(p.stem for p in MAPS_DIR.glob(f"{PREFIX}*.npz")) if MAPS_DIR.is_dir() else []


def _resample_closed(xy: np.ndarray, spacing: float) -> np.ndarray:
    pts = np.vstack([xy, xy[:1]])
    seg = np.hypot(*np.diff(pts, axis=0).T)
    arc = np.concatenate([[0.0], np.cumsum(seg)])
    n = max(8, int(arc[-1] / spacing))
    s = np.linspace(0.0, arc[-1], n, endpoint=False)
    return np.column_stack([np.interp(s, arc, pts[:, 0]), np.interp(s, arc, pts[:, 1])])


def load_real_course(name_or_path: str | Path) -> Course:
    p = Path(name_or_path)
    if not p.suffix:
        p = MAPS_DIR / f"{name_or_path}.npz"
    with np.load(p, allow_pickle=False) as z:
        trinary = np.asarray(z["trinary"])
        resolution = float(z["resolution"])
        origin = (float(z["origin_x"]), float(z["origin_y"]))
        centerline_xy = np.asarray(z["centerline_xy"], dtype=np.float64).reshape(-1, 2)
        routes = json.loads(str(z["routes_json"])) if "routes_json" in z.files else {}
    traj = np.asarray(routes.get("explore_traj", []), dtype=np.float64).reshape(-1, 2)

    def _length(q: np.ndarray) -> float:
        return float(np.hypot(*np.diff(q, axis=0).T).sum()) if len(q) > 1 else 0.0

    # magp_a/a3 の centerline_xy は2.7mほどの断片だった。長い方（実質の周回）を使う
    xy = centerline_xy if _length(centerline_xy) >= _length(traj) else traj
    if len(xy) < 8:
        raise ValueError(f"{p.name}: 中心線にできる点列（centerline_xy / explore_traj）が無い")
    xy = _resample_closed(xy, _SPACING_M)
    nxt = np.roll(xy, -1, axis=0)
    yaw = np.arctan2(nxt[:, 1] - xy[:, 1], nxt[:, 0] - xy[:, 0])
    centerline = np.column_stack([xy, yaw])
    k = min(5, len(xy) - 1)
    start_yaw = math.atan2(xy[k, 1] - xy[0, 1], xy[k, 0] - xy[0, 0])
    # 行0 = y最小（mapstore・Course共通）。未知(0)も壁として塞ぐ
    grid = np.ascontiguousarray(trinary != 1)
    return Course(name=p.stem, path=p, resolution=resolution, origin=origin,
                  start=(float(xy[0, 0]), float(xy[0, 1]), start_yaw),
                  grid=grid, centerline=centerline, width=None, role="eval")
