"""ml_e2e/records.py — 記録（`.mcap`）を、ラベル付けに使う時系列の配列へ読む。

読むトピックと使い道:

| トピック | 使い道 |
|---|---|
| `/viz/image/front` | 入力の画像（JPEG）。`log_time` は撮影時刻 |
| `/scan` | 入力の点群。`log_time` は1周の最初のセクタの時刻 |
| `/vehicle_state` | 入力の車速・実舵角・ヨーレート |
| `/odom` | 推測航法。**記録の中の2時刻の間の動き**を出すのに使う（姿勢の時刻合わせ・通った軌跡） |
| `/cmd` | 誰が運転していたか（`mode`・`source`）と、比べる相手の舵・速度の指令（手本にはしない） |
| `/auto/state` | 地図上の姿勢（`pose_*`）・走行段・一致度・障害物の有無 |
| `/auto/map` | 最適ライン（`raceline`）とその計画速度（`raceline_v`） |

## 姿勢の時刻

`AutoState.pose_*` は**点群の基準時刻**の姿勢で、`AutoState` の時刻（publish した時刻）より古い。
`scan_age_ms` は publish 時点での点群の先頭セクタからの経過なので、姿勢の時刻は
`t_state − scan_age + τ`（τ は点群の基準時刻が先頭セクタからどれだけ後か）と置き、τ は記録ごとに
`estimate_pose_lag()` で推測航法の回頭と突き合わせて求める。
"""

from __future__ import annotations

import base64
import json
import zlib
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from mcap.reader import make_reader

__all__ = ["Recording", "MapLine", "load", "estimate_pose_lag", "decode_cells",
           "iter_image_pairs"]

_TOPICS = ("/scan", "/vehicle_state", "/odom", "/cmd", "/auto/state", "/auto/map")
NS = 1_000_000_000


@dataclass
class MapLine:
    """ある時刻から有効な最適ライン（map フレーム）。"""

    t: int                      #: この版が流れた時刻 [ns]
    xy: np.ndarray              #: (N, 2)
    v: np.ndarray               #: (N,) 計画速度 [m/s]。無ければ長さ 0
    map_name: str = ""
    resolution: float = 0.0
    origin: tuple[float, float] = (0.0, 0.0)
    width: int = 0
    height: int = 0
    cells: bytes = b""          #: 圧縮済みの 3 値地図（`decode_cells`）


@dataclass
class Recording:
    path: Path
    # 車両状態（100Hz）
    vs_t: np.ndarray = field(default_factory=lambda: np.zeros(0, np.int64))
    vs_speed: np.ndarray = field(default_factory=lambda: np.zeros(0))
    vs_yaw_rate: np.ndarray = field(default_factory=lambda: np.zeros(0))
    vs_steer: np.ndarray = field(default_factory=lambda: np.zeros(0))
    # 推測航法（100Hz）
    od_t: np.ndarray = field(default_factory=lambda: np.zeros(0, np.int64))
    od_x: np.ndarray = field(default_factory=lambda: np.zeros(0))
    od_y: np.ndarray = field(default_factory=lambda: np.zeros(0))
    od_yaw: np.ndarray = field(default_factory=lambda: np.zeros(0))     #: 巻き戻さない方位
    od_dist: np.ndarray = field(default_factory=lambda: np.zeros(0))    #: 走行距離の累積
    # 指令（50Hz）
    cmd_t: np.ndarray = field(default_factory=lambda: np.zeros(0, np.int64))
    cmd_mode: np.ndarray = field(default_factory=lambda: np.zeros(0, np.int8))
    cmd_arm: np.ndarray = field(default_factory=lambda: np.zeros(0, bool))
    cmd_steer: np.ndarray = field(default_factory=lambda: np.zeros(0))
    cmd_speed: np.ndarray = field(default_factory=lambda: np.zeros(0))
    cmd_brake: np.ndarray = field(default_factory=lambda: np.zeros(0, bool))
    cmd_torque_mode: np.ndarray = field(default_factory=lambda: np.zeros(0, bool))
    cmd_source: list[str] = field(default_factory=list)
    # 点群（10Hz）
    scan_t: np.ndarray = field(default_factory=lambda: np.zeros(0, np.int64))
    scan_dist: np.ndarray = field(default_factory=lambda: np.zeros((0, 360), np.float32))
    scan_seen: np.ndarray = field(default_factory=lambda: np.zeros((0, 12), bool))
    scan_sat: np.ndarray = field(default_factory=lambda: np.zeros((0, 360), bool))
    # 自動運転の状態（10Hz）
    st_t: np.ndarray = field(default_factory=lambda: np.zeros(0, np.int64))
    st_pose: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    st_score: np.ndarray = field(default_factory=lambda: np.zeros(0))
    st_cross: np.ndarray = field(default_factory=lambda: np.zeros(0))
    st_scan_age: np.ndarray = field(default_factory=lambda: np.zeros(0))   #: [s]
    st_obstacles: np.ndarray = field(default_factory=lambda: np.zeros(0, np.int16))
    st_engaged: np.ndarray = field(default_factory=lambda: np.zeros(0, bool))
    st_phase: list[str] = field(default_factory=list)
    st_mode: list[str] = field(default_factory=list)
    # 最適ライン（版ごと）
    lines: list[MapLine] = field(default_factory=list)

    # ── 時刻で引く ──

    def odom_at(self, t) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """時刻 `t`（スカラーでも配列でも）の推測航法 `(x, y, yaw, dist)`。線形補間。"""
        tt = np.asarray(t, dtype=np.float64)
        ot = self.od_t.astype(np.float64)
        return (np.interp(tt, ot, self.od_x), np.interp(tt, ot, self.od_y),
                np.interp(tt, ot, self.od_yaw), np.interp(tt, ot, self.od_dist))

    def state_at(self, t: int) -> tuple[float, float, float]:
        """時刻 `t` の `(車速, 実舵角, ヨーレート)`。"""
        tt = float(t)
        vt = self.vs_t.astype(np.float64)
        return (float(np.interp(tt, vt, self.vs_speed)), float(np.interp(tt, vt, self.vs_steer)),
                float(np.interp(tt, vt, self.vs_yaw_rate)))

    def line_at(self, t: int) -> MapLine | None:
        """時刻 `t` に有効だった最適ライン。まだ流れていなければ `None`。"""
        best = None
        for ln in self.lines:
            if ln.t <= t:
                best = ln
            else:
                break
        return best


def _arr(vals, dtype=np.float64) -> np.ndarray:
    return np.asarray(vals, dtype=dtype)


def load(path: Path) -> Recording:
    """記録を読む。画像は読まない（`iter_image_pairs`）。"""
    vs, od, cmd, scan, st = [], [], [], [], []
    rec = Recording(path=Path(path))
    with open(path, "rb") as f:
        for _schema, ch, m in make_reader(f).iter_messages(topics=list(_TOPICS)):
            d = json.loads(m.data)
            t = m.log_time
            if ch.topic == "/vehicle_state":
                vs.append((t, d.get("speed", 0.0), d.get("yaw_rate", 0.0), d.get("steer_actual", 0.0)))
            elif ch.topic == "/odom":
                od.append((t, d["x"], d["y"], d.get("yaw_unwrapped", d["yaw"]), d.get("dist", 0.0)))
            elif ch.topic == "/cmd":
                cmd.append((t, int(d.get("mode", 0)), bool(d.get("arm")), d.get("target_steer", 0.0),
                            d.get("target_speed", 0.0), bool(d.get("brake")),
                            bool(d.get("torque_mode")), str(d.get("source", ""))))
            elif ch.topic == "/scan":
                sat = d.get("saturated")
                scan.append((t, d["dist"], d["sector_seen"], sat if sat else [False] * 360))
            elif ch.topic == "/auto/state":
                st.append((t, d.get("pose_x", 0.0), d.get("pose_y", 0.0), d.get("pose_yaw", 0.0),
                           d.get("match_score", 0.0), d.get("cross_track", 0.0),
                           d.get("scan_age_ms", 0.0) / 1e3, len(d.get("obstacles") or []) // 3,
                           bool(d.get("engaged")), str(d.get("phase", "")), str(d.get("mode", ""))))
            elif ch.topic == "/auto/map":
                xy = _arr(d.get("raceline") or []).reshape(-1, 2)
                if len(xy) < 3:
                    continue
                v = _arr(d.get("raceline_v") or [])
                cells = d.get("cells") or b""
                if isinstance(cells, str):
                    cells = base64.b64decode(cells)
                ln = MapLine(t=t, xy=xy, v=v if len(v) == len(xy) else np.zeros(0),
                             map_name=str(d.get("map_name", "")),
                             resolution=float(d.get("resolution", 0.0)),
                             origin=(float(d.get("origin_x", 0.0)), float(d.get("origin_y", 0.0))),
                             width=int(d.get("width", 0)), height=int(d.get("height", 0)), cells=cells)
                last = rec.lines[-1] if rec.lines else None
                if last is None or last.xy.shape != xy.shape or not np.array_equal(last.xy, xy) \
                        or not np.array_equal(last.v, ln.v):
                    rec.lines.append(ln)

    def col(rows, i, dtype=np.float64):
        return _arr([r[i] for r in rows], dtype)

    vs.sort(key=lambda r: r[0]); od.sort(key=lambda r: r[0]); cmd.sort(key=lambda r: r[0])
    scan.sort(key=lambda r: r[0]); st.sort(key=lambda r: r[0]); rec.lines.sort(key=lambda ln: ln.t)
    if vs:
        rec.vs_t, rec.vs_speed = col(vs, 0, np.int64), col(vs, 1)
        rec.vs_yaw_rate, rec.vs_steer = col(vs, 2), col(vs, 3)
    if od:
        rec.od_t, rec.od_x, rec.od_y = col(od, 0, np.int64), col(od, 1), col(od, 2)
        rec.od_yaw, rec.od_dist = col(od, 3), col(od, 4)
    if cmd:
        rec.cmd_t, rec.cmd_mode, rec.cmd_arm = col(cmd, 0, np.int64), col(cmd, 1, np.int8), col(cmd, 2, bool)
        rec.cmd_steer, rec.cmd_speed = col(cmd, 3), col(cmd, 4)
        rec.cmd_brake, rec.cmd_torque_mode = col(cmd, 5, bool), col(cmd, 6, bool)
        rec.cmd_source = [r[7] for r in cmd]
    if scan:
        rec.scan_t = col(scan, 0, np.int64)
        # 測距不能は None で入ることがある（JSON に NaN が無い）
        rec.scan_dist = np.array([[x if x else 0.0 for x in r[1]] for r in scan], dtype=np.float32)
        rec.scan_seen = np.array([r[2] for r in scan], dtype=bool)
        rec.scan_sat = np.array([r[3] for r in scan], dtype=bool)
    if st:
        rec.st_t = col(st, 0, np.int64)
        rec.st_pose = np.array([r[1:4] for r in st], dtype=np.float64)
        rec.st_score, rec.st_cross, rec.st_scan_age = col(st, 4), col(st, 5), col(st, 6)
        rec.st_obstacles, rec.st_engaged = col(st, 7, np.int16), col(st, 8, bool)
        rec.st_phase = [r[9] for r in st]
        rec.st_mode = [r[10] for r in st]
    return rec


def iter_image_pairs(path: Path, rejects=None):
    """`(右の撮影時刻 [ns], 右の JPEG, 左の JPEG)` を記録の順に返す（前方ステレオの左右の組）。

    組の作り方（撮影時刻の差が `STEREO_PAIR_TOL_NS` 以内）は `ml_cam_e2e/extract_pairs.iter_images` と同じものを使う。
    組にならなかった右のコマは返さず、`rejects`（`Counter`）に「左のコマが無い」で数える。
    """
    import sys

    root = Path(__file__).resolve().parents[1]
    for p in (str(root), str(root / "ml_cam_e2e")):
        if p not in sys.path:
            sys.path.insert(0, p)
    from extract_pairs import (
        iter_images,  # ml_cam_e2e のスクリプト形式のモジュール
    )

    for _cam, t, data, left in iter_images(path, rejects=rejects):
        yield (t, base64.b64decode(json.loads(data)["data"]),
               base64.b64decode(json.loads(left)["data"]))


def decode_cells(ln: MapLine) -> np.ndarray | None:
    """`AutoMap.cells`（2bit×4 を 1 バイトに詰めて zlib）を `(H, W)` の 3 値（0=未知 1=空き 2=占有）へ。"""
    if not ln.cells or ln.width <= 0 or ln.height <= 0:
        return None
    raw = np.frombuffer(zlib.decompress(ln.cells), dtype=np.uint8)
    n = ln.width * ln.height
    out = np.empty((len(raw), 4), dtype=np.uint8)
    for k in range(4):
        out[:, k] = (raw >> (2 * k)) & 3
    flat = out.reshape(-1)
    if len(flat) < n:
        return None
    return flat[:n].reshape(ln.height, ln.width)


def estimate_pose_lag(rec: Recording, *, lo_s: float = -0.15, hi_s: float = 0.25,
                      step_s: float = 0.005) -> tuple[float, float]:
    """姿勢の時刻のずれ τ [s] と、そのときの回頭の食い違いの RMS [rad] を返す。

    `t_pose = t_state − scan_age + τ` と置き、隣り合う `AutoState` の間の SLAM の回頭と、
    同じ区間の推測航法の回頭の差が最小になる τ を選ぶ。回頭の速い区間が無い記録（直進だけ）は
    決まらないので `(nan, nan)`。
    """
    ok = np.array([p == "RACE" for p in rec.st_phase], dtype=bool) & (rec.st_score > 0.3)
    idx = np.flatnonzero(ok[1:] & ok[:-1]) + 1
    if len(idx) < 50 or len(rec.od_t) < 100:
        return float("nan"), float("nan")
    t1 = rec.st_t[idx] - (rec.st_scan_age[idx] * NS).astype(np.int64)
    t0 = rec.st_t[idx - 1] - (rec.st_scan_age[idx - 1] * NS).astype(np.int64)
    keep = (t1 - t0 > 0.05 * NS) & (t1 - t0 < 0.25 * NS)
    idx, t0, t1 = idx[keep], t0[keep], t1[keep]
    dyaw = rec.st_pose[idx, 2] - rec.st_pose[idx - 1, 2]
    dyaw = (dyaw + np.pi) % (2 * np.pi) - np.pi
    if len(idx) < 50 or np.std(dyaw) < 0.01:
        return float("nan"), float("nan")
    ot = rec.od_t.astype(np.float64)
    best = (float("nan"), float("inf"))
    for tau in np.arange(lo_s, hi_s + 1e-9, step_s):
        s = tau * NS
        d = np.interp(t1 + s, ot, rec.od_yaw) - np.interp(t0 + s, ot, rec.od_yaw)
        rms = float(np.sqrt(np.mean((d - dyaw) ** 2)))
        if rms < best[1]:
            best = (float(tau), rms)
    return best
