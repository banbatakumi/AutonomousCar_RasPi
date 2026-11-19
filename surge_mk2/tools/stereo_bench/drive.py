"""走行中の `scan/stereo` を LiDAR と比べる（`--drive`）。画面を持たない純ロジック＋記録の読み込み。

    .venv/bin/python -m tools.stereo_bench REC.mcap --drive              # 記録に残った scan/stereo を採点
    .venv/bin/python -m tools.stereo_bench REC.mcap --drive --no-comp    # 補正なし（比べる用）

段階3 の物差し（`bench.run`）は止まった組だけを見る。走っていると、LiDAR の1周（100ms）とステレオの撮影時刻が
ずれ、2m/s・1rad/s では 20cm・6° ずれた所を比べてしまう。ここでは **LiDAR の点を1点ずつ、その点の時刻の
自車の姿勢（`/odom`）から、ステレオの撮影時刻の base_link へ移してから** 比べる。

## 採点の約束

- 物差しは LiDAR（走査面は路面から `[sensors.lidar] z`）。ステレオは高さ `z_min`〜`z_max`（既定 3〜20cm）の
  かたまりを出すので、**LiDAR の面より低い物（〜9.5cm）はステレオにだけ写る**。「LiDAR に無い所の点」は
  偽検出の候補であって、全部が偽とは限らない（`--save-fp` の絵で中身を見る）
- 方位は base_link の車両角 1°（`scan/stereo` と同じ）。LiDAR は ±`nbr_deg` の方位の最も近い点と比べる
  （ステレオの物の縁は照合の窓のぶん太るので、隣の方位の LiDAR に当たっていれば合っているとみなす）
- ステレオの点 1 つ（方位 1 つ）の判定:
  - `match`: |ステレオ − LiDAR| ≤ max(`tol_abs`, `tol_rel`·LiDAR)
  - `closer`（偽検出の候補）: ステレオが LiDAR より手前（上の許容を超えて）、または LiDAR がその方位に `r_max`+`free_margin` まで何も無い
  - `farther`: ステレオが LiDAR より奥（LiDAR の面より上の物の手前の縁を拾った等。危険側）
- 見逃し（`recall`）: ステレオの視野の中で LiDAR が `recall_r` 以内に点を持つ方位のうち、ステレオがその方位（±`nbr_deg`）に
  何かを出した割合。無地の壁はステレオに写らないので 1 にはならない
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field

import numpy as np

__all__ = ["PoseTrack", "LidarScan", "DriveLog", "DriveParams", "lidar_points_base", "lidar_bearing_ranges",
           "score_frame", "drive_eval", "DriveResult", "load_drive", "vehicle_from_log", "lidar_ranges_at"]


# ---------------------------------------------------------------- 自車の姿勢

@dataclass
class PoseTrack:
    """`/odom` の時系列（単調時刻 ns）。間は線形に補間する（yaw は巻き戻さない値で）。"""

    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray

    def at(self, t_ns) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        t = np.asarray(t_ns, np.float64)
        return (np.interp(t, self.t, self.x), np.interp(t, self.t, self.y), np.interp(t, self.t, self.yaw))

    def covers(self, t_ns: int) -> bool:
        return self.t.size > 1 and self.t[0] <= t_ns <= self.t[-1]


# ---------------------------------------------------------------- LiDAR

@dataclass
class LidarScan:
    t: int                         #: `t_capture`（単調時刻 ns。最初のセクタ）
    dist: np.ndarray               #: (360,) 車両角ごとの距離 [m]。0 = 無効
    sector_t_ns: np.ndarray        #: (12,)
    sector_dur_us: np.ndarray      #: (12,)

    def point_times(self) -> np.ndarray:
        """車両角ごとの点の時刻 [ns]（`Scan` の docstring の式）。セクタが欠けた点は 0。"""
        deg = np.arange(360)
        sensor = (360 - deg) % 360
        s, i = 11 - sensor // 30, sensor % 30
        t0 = self.sector_t_ns[s].astype(np.float64)
        t = t0 + self.sector_dur_us[s].astype(np.float64) * 1000.0 * i / 29.0
        t[t0 == 0] = 0.0
        return t

    @property
    def t_mid(self) -> float:
        ts = self.sector_t_ns[self.sector_t_ns > 0]
        return float(np.mean(ts)) if ts.size else float(self.t)


def lidar_points_base(scan: LidarScan, t_ref: int, poses: PoseTrack | None,
                      lidar_pose: tuple[float, float, float, float]) -> tuple[np.ndarray, np.ndarray]:
    """LiDAR の点 → 時刻 `t_ref` の base_link の (x, y)。`(xy (N,2), 点の時刻 (N,))`。

    `poses` が None なら補正しない（全点を `t_ref` の姿勢で撮ったとみなす）。
    """
    lx, ly, _lz, lyaw = lidar_pose
    deg = np.nonzero(scan.dist > 0)[0]
    r = scan.dist[deg]
    tp = scan.point_times()[deg]
    ok = tp > 0
    deg, r, tp = deg[ok], r[ok], tp[ok]
    a = np.radians(deg) + lyaw
    px, py = lx + r * np.cos(a), ly + r * np.sin(a)        # 点の時刻の base_link
    if poses is None:
        return np.stack([px, py], 1), tp
    x1, y1, th1 = poses.at(tp)                              # 点の時刻の自車 → 世界
    wx = x1 + px * np.cos(th1) - py * np.sin(th1)
    wy = y1 + px * np.sin(th1) + py * np.cos(th1)
    x0, y0, th0 = poses.at(t_ref)                           # 世界 → t_ref の自車
    dx, dy = wx - x0, wy - y0
    c, s = math.cos(float(th0)), math.sin(float(th0))
    return np.stack([c * dx + s * dy, -s * dx + c * dy], 1), tp


def lidar_bearing_ranges(xy: np.ndarray) -> np.ndarray:
    """base_link の点 → 原点から見た 1° の方位ごとの最も近い距離 (360,)。点が無い方位は 0。"""
    out = np.zeros(360)
    if xy.size == 0:
        return out
    r = np.hypot(xy[:, 0], xy[:, 1])
    deg = np.round(np.degrees(np.arctan2(xy[:, 1], xy[:, 0]))).astype(int) % 360
    order = np.argsort(-r)                                  # 遠い順に書けば近い点が残る
    out[deg[order]] = r[order]
    return out


# ---------------------------------------------------------------- 採点

@dataclass(frozen=True)
class DriveParams:
    nbr_deg: int = 1               #: 隣の方位をどこまで見るか
    tol_abs: float = 0.10          #: [m]
    tol_rel: float = 0.10
    r_max: float = 2.0             #: ステレオの距離の上限（`ObstacleParams.r_max`）
    free_margin: float = 0.3       #: LiDAR がこれより遠くまで何も無ければ「LiDAR に無い」[m]
    recall_r: float = 1.5          #: 見逃しを数える LiDAR の距離の上限 [m]
    recall_r_min: float = 0.4      #: 同・下限（近すぎる所は照合できる最も近い距離 0.21m・車体の陰）
    max_dt_ms: float = 60.0        #: 点の時刻とステレオの撮影時刻の差の上限（これより離れた点は使わない）


def _nbr_min(rng: np.ndarray, k: int) -> np.ndarray:
    """方位 ±k の最も近い（0 は無視）。"""
    v = np.where(rng > 0, rng, np.inf)
    out = v.copy()
    for s in range(1, k + 1):
        out = np.minimum(out, np.minimum(np.roll(v, s), np.roll(v, -s)))
    return np.where(np.isfinite(out), out, 0.0)


@dataclass
class FrameScore:
    deg: np.ndarray                #: ステレオが点を出した方位
    s: np.ndarray                  #: その距離
    l: np.ndarray                  #: LiDAR（±nbr の最も近い。0 = 何も無い）
    cls: np.ndarray                #: 0 match / 1 closer / 2 farther
    n_recall: int                  #: 見逃しを数える方位の数
    n_recalled: int


def score_frame(stereo: np.ndarray, lidar_rng: np.ndarray, fov_degs, p: DriveParams = DriveParams()
                ) -> FrameScore:
    stereo = np.asarray(stereo, np.float64)
    L = _nbr_min(lidar_rng, p.nbr_deg)
    deg = np.nonzero(stereo > 0)[0]
    s, l_ = stereo[deg], L[deg]
    tol = np.maximum(p.tol_abs, p.tol_rel * l_)
    none = (l_ <= 0) | (l_ > p.r_max + p.free_margin)
    cls = np.where(np.abs(s - l_) <= tol, 0, np.where(s < l_, 1, 2))
    cls[none] = 1
    fov = np.zeros(360, bool)
    fov[np.asarray(list(fov_degs), int) % 360] = True
    want = fov & (lidar_rng >= p.recall_r_min) & (lidar_rng <= p.recall_r)
    S = _nbr_min(stereo, p.nbr_deg)
    got = want & (S > 0)
    return FrameScore(deg, s, l_, cls, int(want.sum()), int(got.sum()))


@dataclass
class DriveResult:
    rows: list[dict] = field(default_factory=list)   #: 方位ごと（ステレオが点を出した所）
    frames: list[dict] = field(default_factory=list)  #: 組ごと

    def table(self):
        import numpy as _np
        keys = ("speed", "yaw_rate", "deg", "s", "l", "cls", "frame")
        return {k: _np.array([r[k] for r in self.rows], float) for k in keys}

    def summary(self, speed_bins=((0, 0.3), (0.3, 1.5), (1.5, 2.5), (2.5, 99)),
                yaw_bins=((0, 0.3), (0.3, 1.0), (1.0, 99))) -> dict:
        T = self.table()
        F = {k: np.array([f[k] for f in self.frames], float) for k in ("speed", "yaw_rate", "n_recall", "n_recalled",
                                                                       "n_closer", "n_hits")} if self.frames else {}

        def agg(m, fm):
            n = int(m.sum())
            if n == 0:
                return None
            c = T["cls"][m]
            mm = m & (T["cls"] == 0) & (T["l"] > 0)
            rel = (T["s"][mm] - T["l"][mm]) / T["l"][mm]
            nr = F["n_recall"][fm].sum() if F else 0
            return {"n": n, "frames": int(fm.sum()), "match": float(np.mean(c == 0)), "closer": float(np.mean(c == 1)),
                    "farther": float(np.mean(c == 2)),
                    "closer_per_frame": float(F["n_closer"][fm].mean()) if F and fm.any() else math.nan,
                    "bias": float(np.median(rel)) if rel.size else math.nan,
                    "abs_med": float(np.median(np.abs(rel))) if rel.size else math.nan,
                    "recall": float(F["n_recalled"][fm].sum() / nr) if F and nr else math.nan}
        out = {"全部": agg(np.ones(len(T["cls"]), bool), np.ones(len(self.frames), bool))}
        for lo, hi in speed_bins:
            out[f"車速 {lo}-{hi}"] = agg((T["speed"] >= lo) & (T["speed"] < hi),
                                          (F["speed"] >= lo) & (F["speed"] < hi))
        mv = T["speed"] >= 1.0
        fmv = F["speed"] >= 1.0
        for lo, hi in yaw_bins:
            out[f"走行中 |ヨー| {lo}-{hi}"] = agg(mv & (np.abs(T["yaw_rate"]) >= lo) & (np.abs(T["yaw_rate"]) < hi),
                                                 fmv & (np.abs(F["yaw_rate"]) >= lo) & (np.abs(F["yaw_rate"]) < hi))
        return out


@dataclass
class DriveLog:
    """採点に要る記録の中身（単調時刻 ns にそろえたもの）。"""

    stereo: list[tuple[int, np.ndarray]]               #: (撮影時刻, dist (360,))。記録に残った `scan/stereo`
    scans: list[LidarScan]
    poses: PoseTrack
    state_t: np.ndarray                                #: `/vehicle_state` の時刻
    speed: np.ndarray
    yaw_rate: np.ndarray
    mono_to_epoch_ns: int                              #: 単調時刻 + これ = 記録の log_time
    vehicle_toml: str | None = None


def _nearest_scans(scans: list[LidarScan], mids: np.ndarray, t: int) -> list[LidarScan]:
    k = int(np.searchsorted(mids, t))
    return [scans[j] for j in (k - 1, k) if 0 <= j < len(scans)]


def lidar_ranges_at(log: DriveLog, t: int, lidar_pose, *, compensate: bool = True,
                    max_dt_ms: float = 60.0, mids: np.ndarray | None = None) -> np.ndarray:
    """時刻 `t` の base_link から見た LiDAR の 1° の方位ごとの距離。前後2周の点のうち、時刻が `max_dt_ms` 以内のもの。"""
    if mids is None:
        mids = np.array([s.t_mid for s in log.scans])
    poses = log.poses if compensate else None
    xys = []
    for sc in _nearest_scans(log.scans, mids, t):
        xy, tp = lidar_points_base(sc, t, poses, lidar_pose)
        keep = np.abs(tp - t) <= max_dt_ms * 1e6
        xys.append(xy[keep])
    return lidar_bearing_ranges(np.concatenate(xys) if xys else np.zeros((0, 2)))


def drive_eval(log: DriveLog, fov_degs, lidar_pose, *, stereo=None, compensate: bool = True,
               p: DriveParams = DriveParams(), t_shift_ns: int = 0) -> DriveResult:
    """`stereo`（`[(撮影時刻, dist)]`。None なら記録の `scan/stereo`）を LiDAR で採点する。

    `t_shift_ns`: ステレオの撮影時刻に足してから比べる（時刻のずれを探す `--find-dt` 用）。
    """
    mids = np.array([s.t_mid for s in log.scans])
    res = DriveResult()
    for fi, (t, dist) in enumerate(log.stereo if stereo is None else stereo):
        t = int(t) + int(t_shift_ns)
        if not log.poses.covers(t) or not log.scans:
            continue
        k = min(int(np.searchsorted(log.state_t, t)), len(log.state_t) - 1)
        v, w = float(log.speed[k]), float(log.yaw_rate[k])
        L = lidar_ranges_at(log, t, lidar_pose, compensate=compensate, max_dt_ms=p.max_dt_ms, mids=mids)
        sc = score_frame(np.asarray(dist), L, fov_degs, p)
        for d, s, l_, c in zip(sc.deg.tolist(), sc.s.tolist(), sc.l.tolist(), sc.cls.tolist()):
            res.rows.append({"speed": v, "yaw_rate": w, "deg": d, "s": s, "l": l_, "cls": c, "frame": fi})
        res.frames.append({"t": t, "speed": v, "yaw_rate": w, "n_recall": sc.n_recall, "n_recalled": sc.n_recalled,
                           "n_closer": int(np.sum(sc.cls == 1)), "n_hits": int(sc.deg.size)})
    return res


# ---------------------------------------------------------------- 記録の読み込み

def load_drive(path) -> DriveLog:
    from mcap.reader import make_reader

    stereo, scans, odom, st = [], [], [], []
    meta = {}
    with open(path, "rb") as f:
        rd = make_reader(f)
        for rec in rd.iter_metadata():
            meta[rec.name] = dict(rec.metadata)
    offset = None
    with open(path, "rb") as f:
        for _s, ch, m in make_reader(f).iter_messages(topics=["/scan/stereo", "/scan", "/odom", "/vehicle_state"]):
            d = json.loads(m.data)
            tc = int(d.get("t_capture", 0))
            if ch.topic == "/odom":
                if offset is None and tc:
                    offset = m.log_time - tc
                odom.append((tc, d["x"], d["y"], d.get("yaw_unwrapped", d["yaw"])))
            elif ch.topic == "/vehicle_state":
                st.append((tc, float(d.get("speed", 0.0)), float(d.get("yaw_rate", 0.0))))
            elif ch.topic == "/scan":
                if not tc:
                    continue
                scans.append(LidarScan(tc, np.asarray(d["dist"], np.float64),
                                       np.asarray(d["sector_t_ns"], np.int64), np.asarray(d["sector_dur_us"], np.int64)))
            else:
                if tc:
                    stereo.append((tc, np.asarray(d["dist"], np.float64)))
    if offset is None:
        s = meta.get("surge", {})
        offset = int(s.get("t0_unix_ns", 0)) - int(s.get("t0_mono_ns", 0))
    for lst in (stereo, odom, st):
        lst.sort(key=lambda e: e[0])
    scans.sort(key=lambda s: s.t_mid)
    O = np.array(odom, np.float64).reshape(-1, 4)
    S = np.array(st, np.float64).reshape(-1, 3)
    return DriveLog(stereo, scans, PoseTrack(O[:, 0], O[:, 1], O[:, 2], O[:, 3]), S[:, 0], S[:, 1], S[:, 2],
                    int(offset), meta.get("provenance", {}).get("vehicle_toml"))


def vehicle_from_log(log: DriveLog):
    """撮った時の vehicle.toml（記録の `provenance`）の `Vehicle`。無ければ None。"""
    if not log.vehicle_toml:
        return None
    from raspi.core.vehicle import Vehicle

    return Vehicle.from_toml_text(log.vehicle_toml, "<provenance>")


# ---------------------------------------------------------------- 照合のやり直しと CLI

def replay_stereo(path, log: DriveLog, vehicle, *, stride: int = 1) -> tuple[list[tuple[int, np.ndarray]], list[int]]:
    """記録の画像の組（`/viz/image/front`・`/viz/image/left`）を今のコードの照合（`StereoProcessor`）でやり直す。

    `(scan/stereo 相当の [(撮影時刻, dist)], 視野の方位)`。画像は JPEG（`--stereo-jpeg-quality` 既定 90）なので
    実機の照合とわずかに違う（会場の記録で、やり直しと実機の採点はほぼ同じ値だった）。
    """
    from raspi.nodes.stereo_node import StereoProcessor

    from .data import load_frames

    proc = StereoProcessor(vehicle)
    out = []
    for fr in load_frames(path, stride=stride):
        r, l_ = fr.decode()
        st, _d = proc.process(r, l_, "BGR888")
        out.append((fr.t_ns - log.mono_to_epoch_ns, np.asarray(st.dist, np.float64)))
    return out, proc.fov_degs


def fit_yaw(log: DriveLog, fov_degs, lidar_pose, stereo=None, *, degs=range(-8, 9),
            min_speed: float = 1.0) -> list[tuple[int, float, float]]:
    """ステレオの方位を k 度回したときの `(k, match の割合, |相対誤差| の中央値)`（車速 `min_speed` 以上の組）。

    右カメラの取付の yaw を k 度にしたのとほぼ同じ（1〜2m 先では、カメラ位置と base_link 原点の差は効かない）。
    向きがずれていると、走行中に左右の壁を斜めに見る所で、片側は手前・反対側は奥に出る。
    """
    src = log.stereo if stereo is None else stereo
    out = []
    for k in degs:
        T = drive_eval(log, fov_degs, lidar_pose, stereo=[(t, np.roll(d, k)) for t, d in src]).table()
        mv = T["speed"] >= min_speed
        ok = mv & (T["l"] > 0.3)
        rel = (T["s"][ok] - T["l"][ok]) / T["l"][ok]
        out.append((int(k), float(np.mean(T["cls"][mv] == 0)) if mv.any() else math.nan,
                    float(np.median(np.abs(rel))) if rel.size else math.nan))
    return out


def _fmt_row(name: str, s: dict | None) -> str:
    if not s:
        return f"  {name:22s} —"
    return (f"  {name:22s} {s['n']:6d}点 {s['frames']:5d}組  一致 {s['match'] * 100:4.0f}%  "
            f"LiDARに無い {s['closer'] * 100:4.0f}%（{s['closer_per_frame']:4.1f}/組）  奥 {s['farther'] * 100:4.0f}%  "
            f"偏り {s['bias'] * 100:+5.1f}%  |誤差| {s['abs_med'] * 100:4.1f}%  見えた {s['recall'] * 100:4.0f}%")


def main_drive(args) -> int:
    import sys
    from dataclasses import replace
    from pathlib import Path

    from raspi.core.stereo_match import fov_degrees

    from .bench import Geometry

    for p in args.mcap:
        log = load_drive(p)
        v = vehicle_from_log(log)
        if v is None:
            from raspi.core.vehicle import Vehicle
            v = Vehicle.load(args.toml) if args.toml else Vehicle.load()
            print("# 記録に vehicle.toml が無い（古い記録）。今の vehicle.toml で採点する", file=sys.stderr)
        if args.cam_yaw_deg is not None:
            v = replace(v, cam_front_yaw=math.radians(args.cam_yaw_deg))
        try:
            geom = Geometry.from_vehicle(v)
        except ValueError as e:
            print(f"!! {e}", file=sys.stderr)
            return 1
        lidar_pose = geom.lidar_pose
        stereo, fov = None, fov_degrees(geom.rect, geom.cam_pose)
        if args.replay:
            stereo, fov = replay_stereo(p, log, v, stride=args.stride)
        src = log.stereo if stereo is None else stereo
        print(f"\n=== {Path(p).name}: ステレオ {len(src)} 組・LiDAR {len(log.scans)} 周"
              f"（{'照合をやり直した' if args.replay else '記録の scan/stereo'}・"
              f"{'動きの補正なし' if args.no_comp else '動きを補正'}"
              + (f"・右カメラ yaw {args.cam_yaw_deg:+.1f}°" if args.cam_yaw_deg is not None else "") + "）")
        if not src or not log.scans:
            print("!! scan/stereo か /scan が無い", file=sys.stderr)
            continue
        if args.fit_yaw:
            rows = fit_yaw(log, fov, lidar_pose, stereo)
            best = max(rows, key=lambda r: (r[1], -r[2]))
            for k, m, e in rows:
                print(f"  方位を {k:+3d}° 回す: 一致 {m * 100:4.1f}%  |誤差| {e * 100:4.1f}%" + ("  ←" if k == best[0] else ""))
            yaw0 = math.degrees(v.cam_front_yaw)
            print(f"  → 右カメラの yaw の見積もり {yaw0 + best[0]:+.0f}°（今 {yaw0:+.1f}°）。"
                  "走行中の組だけで見る。止めて正面以外にも物がある場面で確かめてから vehicle.toml に書く")
            continue
        res = drive_eval(log, fov, lidar_pose, stereo=stereo, compensate=not args.no_comp)
        for name, s in res.summary().items():
            print(_fmt_row(name, s))
    print("\n（一致: LiDAR と ±max(10cm, 10%)。LiDARに無い: ステレオが LiDAR より手前・または LiDAR に何も無い"
          "＝偽検出の候補（LiDAR の面 9.5cm より低い物も入る）。奥: ステレオが LiDAR より奥。"
          "見えた: LiDAR が 0.4〜1.5m に点を持つ方位のうちステレオも何か出した割合）")
    return 0
