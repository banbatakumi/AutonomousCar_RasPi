"""記録（`.mcap`）からステレオの組・点群・車速を読む。"""

from __future__ import annotations

import base64
import json
from bisect import bisect_left
from dataclasses import dataclass
from pathlib import Path

import numpy as np

__all__ = ["StereoFrame", "load_frames", "pair_times", "load_run_info", "load_stereo_info"]

IMAGE_RIGHT = "/viz/image/front"
IMAGE_LEFT = "/viz/image/left"
#: 1周 100ms。`Scan` の `log_time` は最初のセクタの時刻なので、真ん中は +50ms
SCAN_HALF_NS = 50_000_000


@dataclass
class StereoFrame:
    t_ns: int                       #: 右の撮影時刻（記録の時刻＝epoch ns）
    dt_ns: int                      #: 左 − 右 の撮影時刻の差
    right_jpg: bytes
    left_jpg: bytes
    #: 1° ごとの LiDAR の距離（車両角。0 = 無効）。近い時刻の1周が無ければ None
    scan: np.ndarray | None
    #: 組と点群の時刻の差 [ns]（点群の真ん中との差）
    scan_dt_ns: int
    #: 車速 [m/s]。記録に無ければ None
    speed: float | None

    def decode(self) -> tuple[np.ndarray, np.ndarray]:
        """`(右, 左)` の BGR 画像。"""
        import cv2

        r = cv2.imdecode(np.frombuffer(self.right_jpg, np.uint8), cv2.IMREAD_COLOR)
        l_ = cv2.imdecode(np.frombuffer(self.left_jpg, np.uint8), cv2.IMREAD_COLOR)
        return r, l_


def pair_times(tr: list[int], tl: list[int], tol_ns: int) -> list[tuple[int, int]]:
    """右と左の時刻の並び（昇順）から、差が `tol_ns` 以内の組の添字 `(i, j)`。1枚は1組にしか入らない。"""
    out = []
    j0 = 0
    used = set()
    for i, t in enumerate(tr):
        j = bisect_left(tl, t - tol_ns, lo=j0)
        best, best_dt = None, tol_ns + 1
        while j < len(tl) and tl[j] <= t + tol_ns:
            if j not in used and abs(tl[j] - t) < best_dt:
                best, best_dt = j, abs(tl[j] - t)
            j += 1
        if best is not None:
            used.add(best)
            out.append((i, best))
            j0 = best + 1
    return out


def _nearest(ts: list[int], t: int) -> int | None:
    if not ts:
        return None
    k = bisect_left(ts, t)
    cand = [c for c in (k - 1, k) if 0 <= c < len(ts)]
    return min(cand, key=lambda c: abs(ts[c] - t))


def load_frames(path: str | Path, *, pair_tol_ns: int = 500_000, max_scan_dt_ns: int = 100_000_000,
                stride: int = 1, max_frames: int | None = None) -> list[StereoFrame]:
    """記録からステレオの組を読む（JPEG のまま。画像は `StereoFrame.decode` で開く）。

    :param stride: 組を何組おきに使うか（止まっている場面は似た組が続く）
    """
    from mcap.reader import make_reader

    right, left, scans, speeds = [], [], [], []
    with open(path, "rb") as f:
        for _s, ch, m in make_reader(f).iter_messages(
                topics=[IMAGE_RIGHT, IMAGE_LEFT, "/scan", "/vehicle_state"]):
            if ch.topic in (IMAGE_RIGHT, IMAGE_LEFT):
                d = json.loads(m.data)
                (right if ch.topic == IMAGE_RIGHT else left).append(
                    (m.log_time, base64.b64decode(d["data"])))
            elif ch.topic == "/scan":
                scans.append((m.log_time + SCAN_HALF_NS, json.loads(m.data)["dist"]))
            else:
                speeds.append((m.log_time, float(json.loads(m.data).get("speed", 0.0))))
    for lst in (right, left, scans, speeds):
        lst.sort(key=lambda e: e[0])
    tr = [t for t, _ in right]
    tl = [t for t, _ in left]
    ts = [t for t, _ in scans]
    tv = [t for t, _ in speeds]
    pairs = pair_times(tr, tl, pair_tol_ns)[::max(1, stride)]
    if max_frames is not None:
        pairs = pairs[:max_frames]
    out = []
    for i, j in pairs:
        t = tr[i]
        k = _nearest(ts, t)
        scan, sdt = None, 0
        if k is not None and abs(ts[k] - t) <= max_scan_dt_ns:
            scan, sdt = np.asarray(scans[k][1], np.float64), ts[k] - t
        kv = _nearest(tv, t)
        out.append(StereoFrame(t, tl[j] - t, right[i][1], left[j][1], scan, sdt,
                               speeds[kv][1] if kv is not None else None))
    return out


def load_run_info(path: str | Path) -> dict[str, dict[str, str]]:
    """記録のメタデータ（`surge` = 記録元・引数、`provenance` = 校正値 `vehicle_toml`・コードの版 `git` `src_sha`）。

    `vehicle_toml` を `tomllib.loads` に渡せば、撮った時の校正値で照合をやり直せる（`raspi/rec/provenance.py`）。
    古い記録（`provenance` 無し）は `surge` だけ。
    """
    from mcap.reader import make_reader

    out: dict[str, dict[str, str]] = {}
    with open(path, "rb") as f:
        for rec in make_reader(f).iter_metadata():
            out[rec.name] = dict(rec.metadata)
    return out


def load_stereo_info(path: str | Path) -> list[dict]:
    """`stereo_node` が照合ごとに出した診断（`StereoInfo`）。`log_time`（撮影時刻、epoch ns）を足して時刻順に返す。

    左右の撮影時刻の差 `dt_lr_ns`、遅れの内訳 `cam_ms` `wait_ms` `read_ms` `proc_ms` `latency_ms`、点数 `n_points`、
    視差の出た割合 `valid_frac`、照合・抽出の設定（`algo` `scale` `num_disp` `rows` `z_min` ...）、取付 `cam_pose`。
    記録に `/stereo/info` が無い（古い記録）なら空。
    """
    from mcap.reader import make_reader

    out = []
    with open(path, "rb") as f:
        for _s, _ch, m in make_reader(f).iter_messages(topics=["/stereo/info"]):
            d = json.loads(m.data)
            d["log_time"] = m.log_time
            out.append(d)
    out.sort(key=lambda d: d["log_time"])
    return out
