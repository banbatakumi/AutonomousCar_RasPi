"""記録を slam2d に通し直して、SLAM の自己位置と推測航法のずれを MCAP に足す。

    python3 -m raspi.tools.slam_replay logs/surge_20260928_120000.sfl
    python3 -m raspi.tools.slam_replay ~/Downloads/sysid_corner_20260927_150903.mcap
    python3 -m raspi.tools.slam_replay run.mcap -o /tmp/run_slam.mcap --no-loop-closure

出力は `<入力>_slam.mcap`。**元の中身は全部入ったまま**（`.mcap` 入力は解釈せずに写し、
`.sfl` 入力は `sfl2mcap` と同じ変換を通す）で、下の表のトピックが増える。

## 何が見えるか

| トピック | 中身 |
|---|---|
| `/slam2d` | 1周ごとの SLAM の姿勢・一致度・見失い・キーフレーム数（独自 JSON） |
| `/pose_compare` | **推測航法と SLAM の差**（位置・方位・走行距離に対する割合） |
| `/viz/slam2d/pose` `/viz/slam2d/path` | SLAM の姿勢と軌跡（frame=`slam2d`） |
| `/viz/slam2d/scan` | 点群を **SLAM の姿勢で**置いたもの（`/viz/scan` は推測航法の姿勢で置かれる） |
| `/viz/slam2d/map` | SLAM が作った占有格子 |
| `/viz/slam2d/path_optimized` | ループ閉じで最適化した後のキーフレーム軌跡（最後に1回） |
| `/tf` | `odom`→`slam2d`（始点合わせ）。`odom`→`base_link` は `rec/viz.py` が書く |

Foxglove の 3D パネルで表示フレームを `odom` にすると、**推測航法の軌跡と SLAM の軌跡が
同じ始点から描かれ、離れていく様子がそのままドリフトとして見える**。
Plot パネルで `/pose_compare.err_pos` を出すと、ずれが溜まっていく速さが分かる。

## 始点合わせ（`odom`→`slam2d`）

SLAM の原点は最初に取り込んだ点群の基準時刻の車両位置、推測航法の原点は記録の
開始位置で、両者は一致しない。**最初の点群の時刻で2つの姿勢が重なる**ように
`odom`→`slam2d` を1つ決めて固定する（以後は動かさない。動かすとずれが見えなくなる）。

実機では `map` と `odom` をつながない（`rec/viz.py`）が、ここでつなぐのは
**オフラインなので始点を後から揃えられる**から。ずれは「推測航法が SLAM から離れた量」で、
SLAM の方が桁違いに正確（シムで 2〜4cm、`sim.slam_bench`）なので、ほぼ推測航法の誤差と読める。

## `.mcap` 入力は `t_pub` の順に流し直す

MCAP のメッセージは `log_time`（＝`t_capture`、センサが測った時刻）順に並ぶ。
点群の `t_capture` は1周の**最初の**セクタの時刻なので、そのまま流すと点群が
「その1周の間の車速・ヨーレート」より先に届き、点ごとの脱スキュー
（`slam2d/core/deskew.py`）が効かない。実機の planning_node が受け取った順
（`t_pub`）に並べ直してから渡す（`REORDER_WINDOW_S` だけ溜めて並べ替える）。
"""

from __future__ import annotations

import argparse
import bisect
import heapq
import itertools
import math
import sys
import time
from collections import deque
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import msgspec  # noqa: E402

from raspi.msgs.types import (  # noqa: E402
    TOPIC_AUTO_MAP,
    TOPIC_AUTO_STATE,
    TOPIC_SCAN,
    TOPIC_VEHICLE_STATE,
    AutoMap,
    AutoState,
    Scan,
    VehicleState,
)
from raspi.nav.se2 import between, compose  # noqa: E402
from raspi.rec.mcap_log import McapLog  # noqa: E402
from raspi.rec.odom import DeadReckoner  # noqa: E402
from raspi.rec.viz import (  # noqa: E402
    FOXGLOVE_FRAME_TRANSFORM_SCHEMA,
    FOXGLOVE_GRID_SCHEMA,
    FOXGLOVE_POSE_IN_FRAME_SCHEMA,
    FOXGLOVE_POSES_IN_FRAME_SCHEMA,
    ODOM_FRAME_ID,
    VizRecorder,
    _Path,
    _pose2d,
    frame_transform,
    grid_from_trinary,
    pose_in_frame,
    poses_in_frame,
)

__all__ = ["SlamReplay", "replay", "default_out_path", "format_summary", "SLAM_FRAME_ID"]

NS = 1_000_000_000

SLAM_FRAME_ID = "slam2d"

#: 書き込みの最小間隔 [ns]
PATH_PERIOD_NS = NS
ALIGN_PERIOD_NS = NS                       # 始点合わせの tf。シークしても消えないよう繰り返す
MAP_PERIOD_NS = 5 * NS                     # 地図は大きい（切り詰めても数百KB）

#: `.mcap` 入力で並べ替えのために溜める幅 [s]。点群の `t_pub - t_capture` は約0.1〜0.2s
REORDER_WINDOW_S = 0.5

#: 推測航法の履歴を持つ長さ [s]（SLAM の基準時刻の姿勢を引くため）
ODOM_HISTORY_S = 3.0

#: 出力で作り直すトピック。**`.mcap` 入力に同名があれば写さない**（二重になる）
REGENERATED_PREFIXES = ("/odom", "/tf", "/viz/odom/", "/viz/slam/", "/viz/map",
                        "/viz/park", "/slam2d", "/pose_compare", "/viz/slam2d/")


class SlamStep(msgspec.Struct):
    """1周ぶんの SLAM の結果（`/slam2d`）。姿勢は `slam2d` フレーム。"""

    t_ref: int = 0                         #: 姿勢の基準時刻 [ns]（単調時刻）
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    score: float = 0.0                     #: 位置合わせの当たり率 0〜1
    lost: bool = False                     #: 合わなかった（推測航法の予測を採った）
    matched: bool = False                  #: 位置合わせを試みたか（地図が空なら False）
    keyframes: int = 0
    loop_closures: int = 0
    update_ms: float = 0.0                 #: この1周の処理時間（**この PC での値**）


class PoseCompare(msgspec.Struct):
    """推測航法と SLAM の差（`/pose_compare`）。**どちらも `odom` フレーム**に揃えてある。"""

    t_ref: int = 0
    slam_x: float = 0.0
    slam_y: float = 0.0
    slam_yaw: float = 0.0
    odom_x: float = 0.0
    odom_y: float = 0.0
    odom_yaw: float = 0.0
    #: 推測航法 − SLAM を **SLAM の車体座標**で見たもの（前後・左右に分けて読める）
    err_along: float = 0.0
    err_cross: float = 0.0
    err_pos: float = 0.0                   #: 位置のずれ [m]
    err_yaw: float = 0.0                   #: 方位のずれ [rad]
    odom_dist: float = 0.0                 #: 始点合わせからの走行距離 [m]
    drift_pct: float = 0.0                 #: `err_pos / odom_dist` [%]（走り出す前は 0）
    lost: bool = False


def _r(v: float, n: int = 4) -> float:
    return round(float(v), n)


class SlamReplay:
    """`vehicle_state` と `scan` を受けて slam2d を回し、MCAP に書く。

    :param log: 書き込み先
    :param loop_closure: ループ閉じを使うか。**実機の `slam2d_raceline` と同じく、
        走行中は拘束を溜めるだけで最適化は `close()` の1回だけ**
    """

    def __init__(self, log: McapLog, *, loop_closure: bool = True) -> None:
        from raspi.auto._slam2d_nav import Slam2dNav
        from raspi.auto.slam2d_raceline import MAP_MAX_RANGE, MAP_RES, MAP_SIZE_M
        from raspi.core.vehicle import Vehicle

        veh = Vehicle.load()
        self.log = log
        self.nav = Slam2dNav(resolution=MAP_RES, size_m=MAP_SIZE_M,
                             lidar_x=veh.lidar_x, lidar_y=veh.lidar_y,
                             max_range=MAP_MAX_RANGE, loop_closure=loop_closure)
        #: `rec/viz.py` の `/odom` と**同じ入力を同じ順に**積むので同じ値になる
        self.dr = DeadReckoner()
        self._odom_t: deque[int] = deque()
        self._odom_p: deque[tuple[float, float, float, float]] = deque()
        self._vs: VehicleState | None = None
        #: 推測航法がまだその時刻まで進んでいない比較 `(pose, t_ref, lost)`（`_compare` 参照）
        self._pending: deque = deque()
        self._scan_t_prev = 0
        self._align = None                 # odom フレームでの slam2d の原点
        self._align_dist = 0.0
        self._path = _Path()
        self._next: dict[str, int] = {}

        # ── 集計（`summary()`） ──
        self.scans = 0
        self.skipped = 0                   # 車両状態がまだ来ていない等で捨てた点群
        self.lost = 0
        self.score_sum = 0.0
        self.update_s = 0.0
        self.last_cmp: PoseCompare | None = None
        self.compare_missed = 0
        self.max_err = 0.0
        self.final_optimized_err: float | None = None

    # ── 入力 ──

    def on_bus(self, topic: str, msg) -> None:
        if topic == TOPIC_VEHICLE_STATE:
            self.on_vehicle_state(msg)
        elif topic == TOPIC_SCAN:
            self.on_scan(msg)

    def on_vehicle_state(self, vs: VehicleState) -> None:
        self._vs = vs
        self.nav.on_vehicle_state(vs)
        p = self.dr.update(vs)
        t = p.t_capture
        if self._odom_t and t <= self._odom_t[-1]:
            return
        self._odom_t.append(t)
        self._odom_p.append((self.dr.x, self.dr.y, self.dr.yaw, self.dr.dist))
        while self._odom_t and self._odom_t[0] < t - ODOM_HISTORY_S * NS:
            self._odom_t.popleft()
            self._odom_p.popleft()
        while self._pending and self._pending[0][1] <= t:
            self._compare(*self._pending.popleft())

    def _odom_at(self, t: int):
        """時刻 `t` の推測航法の姿勢（線形補間）。履歴の外なら None。"""
        ts = self._odom_t
        if not ts or t < ts[0] or t > ts[-1]:
            return None
        i = bisect.bisect_left(ts, t)
        if ts[i] == t or i == 0:
            return self._odom_p[i]
        t0, t1 = ts[i - 1], ts[i]
        a = (t - t0) / (t1 - t0)
        p0, p1 = self._odom_p[i - 1], self._odom_p[i]
        return tuple(p0[k] + a * (p1[k] - p0[k]) for k in range(4))

    def on_scan(self, scan: Scan) -> None:
        vs = self._vs
        if vs is None:
            self.skipped += 1
            return
        t_cap = scan.t_capture or scan.t_pub
        dt = (t_cap - self._scan_t_prev) / NS if self._scan_t_prev else 0.1
        self._scan_t_prev = t_cap
        t0 = time.perf_counter()
        u = self.nav.update(scan, dt, yaw_rate=vs.yaw_rate, speed=vs.speed)
        el = time.perf_counter() - t0
        self.update_s += el
        self.scans += 1
        self.lost += int(u.lost)
        self.score_sum += u.score
        t_ref = int(getattr(self.nav._fe, "_t_ref", 0)) or t_cap
        pose = (u.pose.x, u.pose.y, u.pose.yaw)
        uref = self.log.to_unix_ns(t_ref)

        self.log.write("/slam2d", SlamStep(
            t_ref=t_ref, x=_r(pose[0]), y=_r(pose[1]), yaw=_r(pose[2], 5),
            score=_r(u.score, 3), lost=bool(u.lost), matched=bool(u.matched),
            keyframes=len(self.nav._fe.keyframes), loop_closures=self.nav.loop_closures,
            update_ms=_r(el * 1000.0, 2)), t_mono_ns=t_ref)
        self._path.add(*pose)
        self.log.write_dict("/viz/slam2d/pose",
                            pose_in_frame(SLAM_FRAME_ID, *pose, uref),
                            "foxglove.PoseInFrame", FOXGLOVE_POSE_IN_FRAME_SCHEMA, t_ref)
        self.log.write_viz_scan(scan, t_mono_ns=t_ref, topic="/viz/slam2d/scan",
                                frame_id=SLAM_FRAME_ID, pose=_pose2d(*pose))
        if self._due("path", t_ref, PATH_PERIOD_NS):
            self.log.write_dict("/viz/slam2d/path",
                                poses_in_frame(SLAM_FRAME_ID, self._path.pts, uref),
                                "foxglove.PosesInFrame", FOXGLOVE_POSES_IN_FRAME_SCHEMA, t_ref)
        if self._due("map", t_ref, MAP_PERIOD_NS):
            self._write_map(t_ref)
        # **比較はすぐにはできないことが多い。** `t_ref` は1周の終わり寄りの時刻で、
        # その時刻の車両状態は点群より後に届く（実機の記録で比較が1件も出なかった）。
        # 推測航法がそこまで進んだら `on_vehicle_state` が解決する
        self._pending.append((pose, t_ref, bool(u.lost)))
        if self._odom_t and t_ref <= self._odom_t[-1]:
            self._compare(*self._pending.popleft())

    def _compare(self, pose, t_ref: int, lost: bool) -> None:
        od = self._odom_at(t_ref)
        if od is None:                     # 履歴より古い（車両状態が欠けていた等）
            self.compare_missed += 1
            return
        odom_pose = (od[0], od[1], od[2])
        if self._align is None:
            # **最初の点群の時刻で2つの姿勢を重ねる**（モジュール docstring）
            self._align = compose(odom_pose, between(pose, (0.0, 0.0, 0.0)))
            self._align_dist = od[3]
        if self._due("align", t_ref, ALIGN_PERIOD_NS):
            ax, ay, ayaw = self._align
            self.log.write_dict("/tf", frame_transform(ODOM_FRAME_ID, SLAM_FRAME_ID, ax, ay,
                                                       ayaw, self.log.to_unix_ns(t_ref)),
                                "foxglove.FrameTransform", FOXGLOVE_FRAME_TRANSFORM_SCHEMA,
                                t_ref)
        s = compose(self._align, pose)                    # SLAM を odom フレームへ
        e = between(s, odom_pose)                         # SLAM の車体座標で見たずれ
        # 方位は丸めない累積値で差を取る（±π を跨いでも飛ばない）
        err_yaw = math.remainder(od[2] - s[2], 2.0 * math.pi)
        dist = od[3] - self._align_dist
        err = math.hypot(e[0], e[1])
        c = PoseCompare(
            t_ref=t_ref, slam_x=_r(s[0]), slam_y=_r(s[1]), slam_yaw=_r(s[2], 5),
            odom_x=_r(od[0]), odom_y=_r(od[1]), odom_yaw=_r(math.remainder(od[2], 2 * math.pi), 5),
            err_along=_r(e[0]), err_cross=_r(e[1]), err_pos=_r(err), err_yaw=_r(err_yaw, 5),
            odom_dist=_r(dist), drift_pct=_r(100.0 * err / dist, 3) if dist > 0.5 else 0.0,
            lost=lost)
        self.last_cmp = c
        self.max_err = max(self.max_err, err)
        self.log.write("/pose_compare", c, t_mono_ns=t_ref)

    def _due(self, key: str, t: int, period: int) -> bool:
        nxt = self._next.get(key)
        if nxt is not None and nxt - 2 * period <= t < nxt:
            return False
        self._next[key] = t + period
        return True

    def _write_map(self, t: int, topic: str = "/viz/slam2d/map") -> None:
        g = self.nav.grid
        self.log.write_dict(topic, grid_from_trinary(
            g.trinary(), resolution=g.resolution, origin_x=g.origin[0], origin_y=g.origin[1],
            unix_ns=self.log.to_unix_ns(t), frame_id=SLAM_FRAME_ID),
            "foxglove.Grid", FOXGLOVE_GRID_SCHEMA, t)

    # ── 後始末 ──

    def close(self) -> None:
        """ループ閉じを反映し、最適化後の軌跡と最後の地図を書く。**`McapLog.close()` より先に。**"""
        if self._scan_t_prev == 0:
            return
        t = self._scan_t_prev
        sysm = self.nav._system
        if sysm is not None:
            sysm.flush()
            kfs = self.nav._fe.keyframes
            if kfs:
                self.log.write_dict("/viz/slam2d/path_optimized", poses_in_frame(
                    SLAM_FRAME_ID, [(k.pose.x, k.pose.y, k.pose.yaw) for k in kfs],
                    self.log.to_unix_ns(t)),
                    "foxglove.PosesInFrame", FOXGLOVE_POSES_IN_FRAME_SCHEMA, t)
                # 最後のキーフレームで推測航法と比べ直す（最適化で SLAM 側がどれだけ動いたか）
                k = kfs[-1]
                od = self._odom_at(k.stamp_ns)
                if od is not None and self._align is not None:
                    s = compose(self._align, (k.pose.x, k.pose.y, k.pose.yaw))
                    self.final_optimized_err = math.hypot(od[0] - s[0], od[1] - s[1])
        self._write_map(t)

    def summary(self) -> dict[str, Any]:
        n = max(self.scans, 1)
        c = self.last_cmp
        return {
            "scans": self.scans,
            "skipped": self.skipped,
            "lost_pct": 100.0 * self.lost / n,
            "score_mean": self.score_sum / n,
            "update_ms_mean": 1000.0 * self.update_s / n,
            "keyframes": len(self.nav._fe.keyframes),
            "loop_closures": self.nav.loop_closures,
            "dist": c.odom_dist if c else 0.0,
            "err_final": c.err_pos if c else 0.0,
            "err_yaw_final_deg": math.degrees(c.err_yaw) if c else 0.0,
            "err_max": self.max_err,
            "err_final_optimized": self.final_optimized_err,
            "compare_missed": self.compare_missed + len(self._pending),
        }


# ── 入力ごとの流し方 ─────────────────────────────────────────────────────

def _replay_sfl(src: Path, out: Path, *, loop_closure: bool, compression: str,
                start_s: float, end_s: float | None) -> tuple[McapLog, SlamReplay]:
    from raspi.tools.sfl2mcap import export

    holder: dict[str, SlamReplay] = {}

    def make(log: McapLog):
        holder["slam"] = SlamReplay(log, loop_closure=loop_closure)
        return holder["slam"].on_bus

    log = export(src, out, compression=compression, start_s=start_s, end_s=end_s,
                 quiet=True, on_bus=make, on_close=lambda: holder["slam"].close())
    return log, holder["slam"]


_DECODE = {TOPIC_VEHICLE_STATE: VehicleState, TOPIC_SCAN: Scan,
           TOPIC_AUTO_STATE: AutoState, TOPIC_AUTO_MAP: AutoMap}


def _replay_mcap(src: Path, out: Path, *, loop_closure: bool, compression: str,
                 start_s: float, end_s: float | None,
                 progress=None) -> tuple[McapLog, SlamReplay]:
    from mcap.reader import make_reader

    with open(src, "rb") as f:
        reader = make_reader(f)
        meta = {m.name: m.metadata for m in reader.iter_metadata()}.get("surge", {})
        # 進捗は記録の時刻で数える（索引の統計から全体の長さが分かる。尻切れで無ければ None にならない）
        stats = reader.get_summary().statistics if reader.get_summary() else None
        span = ((stats.message_start_time, stats.message_end_time)
                if stats and stats.message_end_time > stats.message_start_time else None)
        if "t0_mono_ns" not in meta:
            raise ValueError("surge のメタデータが無い（logger_node / sfl2mcap 以外で作った MCAP）")
        t0_mono, t0_unix = int(meta["t0_mono_ns"]), int(meta["t0_unix_ns"])
        log = McapLog(out, t0_mono_ns=t0_mono, t0_unix_ns=t0_unix, compression=compression,
                      metadata={**meta, "source": src.name, "converter": "slam_replay",
                                "loop_closure": loop_closure})
        viz = VizRecorder(log)
        slam = SlamReplay(log, loop_closure=loop_closure)
        heap: list = []
        tie = itertools.count()
        window = int(REORDER_WINDOW_S * NS)

        def feed(bus_topic: str, msg) -> None:
            if bus_topic == TOPIC_VEHICLE_STATE:
                viz.on_vehicle_state(msg)
            elif bus_topic == TOPIC_AUTO_STATE:
                viz.on_auto_state(msg)
            elif bus_topic == TOPIC_AUTO_MAP:
                viz.on_auto_map(msg)
            slam.on_bus(bus_topic, msg)

        t_first = None
        n = 0
        try:
            for schema, channel, message in reader.iter_messages(log_time_order=True):
                if t_first is None:
                    t_first = message.log_time
                n += 1
                if progress is not None and span is not None and n % 500 == 0:
                    progress((message.log_time - span[0]) / (span[1] - span[0]))
                rel = (message.log_time - t_first) / NS
                if rel < start_s or (end_s is not None and rel > end_s):
                    continue
                topic = channel.topic
                if topic.startswith(REGENERATED_PREFIXES):
                    continue
                log.copy_message(schema, channel, message)
                bus_topic = topic[1:]
                cls = _DECODE.get(bus_topic)
                if cls is None:
                    continue
                msg = msgspec.json.decode(message.data, type=cls)
                heapq.heappush(heap, (msg.t_pub or msg.t_capture, next(tie), bus_topic, msg))
                now_mono = message.log_time - t0_unix + t0_mono
                while heap and heap[0][0] <= now_mono - window:
                    _, _, bt, m = heapq.heappop(heap)
                    feed(bt, m)
            while heap:
                _, _, bt, m = heapq.heappop(heap)
                feed(bt, m)
        finally:
            viz.close()
            slam.close()
            log.close()
    return log, slam


def replay(src: str | Path, out: str | Path, *, loop_closure: bool = True,
           compression: str = "zstd", start_s: float = 0.0,
           end_s: float | None = None, progress=None) -> tuple[McapLog, SlamReplay]:
    """`.sfl` か `.mcap` を1本流し直す。閉じた `McapLog` と `SlamReplay`（集計用）を返す。

    :param progress: `(0〜1) -> None`。`.mcap` 入力のときだけ呼ばれる（GUI の進捗表示用。
        `.sfl` は全体の長さを先に知る安い方法が無いので呼ばない）
    """
    src, out = Path(src), Path(out)
    if src.suffix == ".sfl":
        return _replay_sfl(src, out, loop_closure=loop_closure, compression=compression,
                           start_s=start_s, end_s=end_s)
    return _replay_mcap(src, out, loop_closure=loop_closure, compression=compression,
                        start_s=start_s, end_s=end_s, progress=progress)


def default_out_path(src: Path) -> Path:
    """`<入力>_slam.mcap`（入力と同じフォルダ）。"""
    return src.with_name(src.stem + "_slam.mcap")


def format_summary(src: Path, out: Path, log: McapLog, s: dict, wall_s: float) -> str:
    """集計を人間向けの数行にする（CLI と GUI で共用）。"""
    lines = [
        f"{src.name} → {out}  {log.size_text}  ({wall_s:.1f}s で処理)",
        f"  点群 {s['scans']}周（捨てた {s['skipped']}）  見失い {s['lost_pct']:.1f}%  "
        f"一致度 平均{s['score_mean']:.2f}  処理 平均{s['update_ms_mean']:.1f}ms/周",
        f"  キーフレーム {s['keyframes']}  ループ閉じ {s['loop_closures']}本"
        + (f"  比較できなかった周 {s['compare_missed']}" if s["compare_missed"] else ""),
    ]
    line = (f"  推測航法と SLAM の差: 走行 {s['dist']:.1f}m で最後 {s['err_final']:.3f}m / "
            f"{s['err_yaw_final_deg']:+.1f}°  最大 {s['err_max']:.3f}m")
    if s["dist"] > 0.5:
        line += f"（走行距離の {100 * s['err_final'] / s['dist']:.1f}%）"
    lines.append(line)
    if s["err_final_optimized"] is not None:
        lines.append(f"  ループ閉じの最適化後: 最後のキーフレームで {s['err_final_optimized']:.3f}m")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", type=Path, nargs="+", help=".sfl または .mcap")
    ap.add_argument("-o", "--out", type=Path, default=None,
                    help="出力先（既定は <入力>_slam.mcap）")
    ap.add_argument("--no-loop-closure", action="store_true",
                    help="ループ閉じを使わない（Frontend 単体。実機のキルスイッチと同じ）")
    ap.add_argument("--start", type=float, default=0.0, help="開始位置 [s]")
    ap.add_argument("--end", type=float, default=None, help="終了位置 [s]")
    ap.add_argument("--compression", default="zstd", choices=["zstd", "lz4", "none"])
    args = ap.parse_args()

    if args.out is not None and len(args.paths) > 1:
        print("-o は入力1本のときだけ使えます", file=sys.stderr)
        return 2
    rc = 0
    for p in args.paths:
        if not p.exists():
            print(f"ファイルがありません: {p}", file=sys.stderr)
            rc = 2
            continue
        out = args.out or default_out_path(p)
        if out.resolve() == p.resolve():
            print(f"入力と出力が同じです: {p}", file=sys.stderr)
            return 2
        t0 = time.perf_counter()
        try:
            log, slam = replay(p, out, loop_closure=not args.no_loop_closure,
                               compression=args.compression, start_s=args.start,
                               end_s=args.end)
        except RuntimeError as e:             # mcap が無い
            print(f"!! {e}", file=sys.stderr)
            return 2
        except ValueError as e:               # 壊れている / surge の記録ではない
            print(f"!! {p}: {e}", file=sys.stderr)
            rc = 1
            continue
        print(format_summary(p, out, log, slam.summary(), time.perf_counter() - t0))
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
