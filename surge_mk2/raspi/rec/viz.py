"""自己位置・地図を Foxglove の 3D パネルで見るためのトピック（`rec/mcap_log.py` の続き）。

`logger_node` と `tools/sfl2mcap.py` の両方が `VizRecorder` を1つ持ち、
バスのメッセージを渡すと MCAP に以下を書く。

| トピック | スキーマ | 中身 |
|---|---|---|
| `/odom` | `OdomPose`（独自） | 推測航法の数値（`rec/odom.py`）。Plot パネル用 |
| `/tf` | `foxglove.FrameTransform` | `odom`→`base_link`。**これで `/viz/scan` が世界座標に並ぶ** |
| `/viz/odom/pose` | `foxglove.PoseInFrame` | 推測航法の現在姿勢 |
| `/viz/odom/path` | `foxglove.PosesInFrame` | 推測航法の軌跡 |
| `/viz/slam/pose` `/viz/slam/path` | 同上（frame=`map`） | 地図を使う planner の姿勢（`AutoState.pose_*`） |
| `/viz/map` | `foxglove.Grid` | planner の占有格子（`AutoMap.cells`） |
| `/viz/map/lines` | `foxglove.SceneUpdate` | 中心線・レーシングライン |
| `/viz/park` | `foxglove.SceneUpdate` | 駐車の計画経路と目標（base_link に固定） |

## `map` と `odom` は tf でつながない

推測航法（odom）と planner の SLAM（map）は**独立した2つの推定**で、原点も
向きも一致する保証が無い。`map→odom` を適当に書くと、存在しない関係を
あるかのように描くことになる。3D パネルの表示フレームを切り替えて見る:

- `odom` … 推測航法の軌跡＋点群（点群の減衰時間を伸ばすと簡易的な地図になる）
- `map` … SLAM の軌跡＋地図＋レーシングライン

## 軌跡は「全部入り」のメッセージを間引いて書く

Foxglove はシークすると各トピックの最新1件から描き直すので、軌跡は毎回
先頭からの全点を持たせる必要がある（差分だけ送るとシークで消える）。
そのかわり**点の間隔を空け、点数が上限を超えたら間隔を倍にして間引き直す**。
メッセージの大きさに上限を設けるため。

## 地図の書き込みは間引く

`auto/map` は「変わったときだけ」流れるが、地図を作っている間は頻繁に変わる。
RGBA 格子は 400×400 で 640KB あるので、`MAP_MIN_PERIOD_S` より短い間隔では
書かず、保留した最新の1枚を次の機会（または `close()`）に書く。
"""

from __future__ import annotations

import base64
import math
from typing import Any

from .mcap_log import _POSE, _QUAT, _TIME, _VEC3, McapLog, _stamp
from .odom import DeadReckoner, OdomPose

__all__ = [
    "VizRecorder", "ODOM_FRAME_ID", "MAP_FRAME_ID",
    "frame_transform", "pose_in_frame", "poses_in_frame", "grid_from_trinary",
]

ODOM_FRAME_ID = "odom"
MAP_FRAME_ID = "map"
BASE_FRAME_ID = "base_link"

NS = 1_000_000_000

#: 書き込みの最小間隔 [ns]
TF_PERIOD_NS = NS // 50                    # 50Hz。100Hz 全部だと JSON が倍になるだけ
POSE_PERIOD_NS = NS // 10
PATH_PERIOD_NS = NS                        # 1Hz。先端は /viz/*/pose が 10Hz で埋める
MAP_MIN_PERIOD_S = 2.0

#: 軌跡の点の最小間隔 [m] と点数の上限（超えたら間隔を倍にして間引き直す）
PATH_MIN_STEP_M = 0.05
PATH_MAX_POINTS = 1500

#: `foxglove.NumericType` の UINT8
_UINT8 = 1

#: 占有格子の3値 → RGBA。0=未知（透明）1=空き（薄い灰）2=占有（黒）
_TRINARY_RGBA = ((0, 0, 0, 0), (230, 230, 230, 120), (20, 20, 20, 255))

_COLOR = {"type": "object", "properties": {k: {"type": "number"} for k in "rgba"}}

FOXGLOVE_FRAME_TRANSFORM_SCHEMA: dict[str, Any] = {
    "type": "object",
    "properties": {
        "timestamp": _TIME,
        "parent_frame_id": {"type": "string"},
        "child_frame_id": {"type": "string"},
        "translation": _VEC3,
        "rotation": _QUAT,
    },
}

FOXGLOVE_POSE_IN_FRAME_SCHEMA: dict[str, Any] = {
    "type": "object",
    "properties": {"timestamp": _TIME, "frame_id": {"type": "string"}, "pose": _POSE},
}

FOXGLOVE_POSES_IN_FRAME_SCHEMA: dict[str, Any] = {
    "type": "object",
    "properties": {"timestamp": _TIME, "frame_id": {"type": "string"},
                   "poses": {"type": "array", "items": _POSE}},
}

FOXGLOVE_GRID_SCHEMA: dict[str, Any] = {
    "type": "object",
    "properties": {
        "timestamp": _TIME,
        "frame_id": {"type": "string"},
        "pose": _POSE,
        "column_count": {"type": "integer"},
        "cell_size": {"type": "object",
                      "properties": {"x": {"type": "number"}, "y": {"type": "number"}}},
        "row_stride": {"type": "integer"},
        "cell_stride": {"type": "integer"},
        "fields": {"type": "array", "items": {
            "type": "object",
            "properties": {"name": {"type": "string"}, "offset": {"type": "integer"},
                           "type": {"type": "integer"}}}},
        "data": {"type": "string", "contentEncoding": "base64"},
    },
}

_LINE = {
    "type": "object",
    "properties": {
        "type": {"type": "integer"},
        "pose": _POSE,
        "thickness": {"type": "number"},
        "scale_invariant": {"type": "boolean"},
        "points": {"type": "array", "items": _VEC3},
        "color": _COLOR,
    },
}
_SPHERE = {
    "type": "object",
    "properties": {"pose": _POSE, "size": _VEC3, "color": _COLOR},
}
_ARROW = {
    "type": "object",
    "properties": {"pose": _POSE, "shaft_length": {"type": "number"},
                   "shaft_diameter": {"type": "number"},
                   "head_length": {"type": "number"},
                   "head_diameter": {"type": "number"}, "color": _COLOR},
}

FOXGLOVE_SCENE_UPDATE_SCHEMA: dict[str, Any] = {
    "type": "object",
    "properties": {
        "deletions": {"type": "array", "items": {
            "type": "object",
            "properties": {"timestamp": _TIME, "type": {"type": "integer"},
                           "id": {"type": "string"}}}},
        "entities": {"type": "array", "items": {
            "type": "object",
            "properties": {
                "timestamp": _TIME,
                "frame_id": {"type": "string"},
                "id": {"type": "string"},
                "lifetime": _TIME,
                "frame_locked": {"type": "boolean"},
                "lines": {"type": "array", "items": _LINE},
                "spheres": {"type": "array", "items": _SPHERE},
                "arrows": {"type": "array", "items": _ARROW},
            }}},
    },
}

#: `foxglove.LineType` の LINE_STRIP と `foxglove.SceneEntityDeletionType` の ALL
_LINE_STRIP = 0
_DELETE_ALL = 1

_IDENTITY_POSE = {"position": {"x": 0.0, "y": 0.0, "z": 0.0},
                  "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}


# ── 変換関数 ────────────────────────────────────────────────────────────

#: 書き出す桁数。**位置 0.1mm・四元数 1e-6 で十分**。全桁の JSON は数倍に膨らむ
_POS_DIGITS = 4
_QUAT_DIGITS = 6


def _quat_yaw(yaw: float) -> dict[str, float]:
    h = 0.5 * yaw
    return {"x": 0.0, "y": 0.0, "z": round(math.sin(h), _QUAT_DIGITS),
            "w": round(math.cos(h), _QUAT_DIGITS)}


def _xyz(x: float, y: float) -> dict[str, float]:
    return {"x": round(x, _POS_DIGITS), "y": round(y, _POS_DIGITS), "z": 0.0}


def _pose2d(x: float, y: float, yaw: float) -> dict:
    return {"position": _xyz(x, y), "orientation": _quat_yaw(yaw)}


def frame_transform(parent: str, child: str, x: float, y: float, yaw: float,
                    unix_ns: int) -> dict:
    """2D 姿勢 → `foxglove.FrameTransform`。"""
    return {"timestamp": _stamp(unix_ns), "parent_frame_id": parent,
            "child_frame_id": child, "translation": _xyz(x, y),
            "rotation": _quat_yaw(yaw)}


def pose_in_frame(frame_id: str, x: float, y: float, yaw: float, unix_ns: int) -> dict:
    return {"timestamp": _stamp(unix_ns), "frame_id": frame_id, "pose": _pose2d(x, y, yaw)}


def poses_in_frame(frame_id: str, poses: list[tuple[float, float, float]],
                   unix_ns: int) -> dict:
    return {"timestamp": _stamp(unix_ns), "frame_id": frame_id,
            "poses": [_pose2d(x, y, yaw) for x, y, yaw in poses]}


def grid_from_trinary(trinary, *, resolution: float, origin_x: float, origin_y: float,
                      unix_ns: int, frame_id: str = MAP_FRAME_ID, crop: bool = True) -> dict:
    """3値の占有格子（行0 が y 最小）→ RGBA の `foxglove.Grid`。

    `foxglove.Grid` も「行0 が pose の原点側、列が +x、行が +y」なので並びはそのまま。
    **RGBA の4フィールドを持たせる**と、3D パネルの「RGBA (separate fields)」で
    色の設定をしなくても地図らしく描ける。
    """
    import numpy as np
    trinary = np.asarray(trinary)
    if crop:
        # **未知しか無い外周を切り落とす**。slam2d の地図は必要な方向に伸びるので
        # 大半が未知のことがあり、RGBA にすると 1 枚で数MB になる
        known = np.nonzero(trinary)
        if known[0].size:
            r0, r1 = int(known[0].min()), int(known[0].max()) + 1
            c0, c1 = int(known[1].min()), int(known[1].max()) + 1
            trinary = trinary[r0:r1, c0:c1]
            origin_x += c0 * resolution
            origin_y += r0 * resolution
    height, width = trinary.shape
    # 1セル4バイトへの展開は numpy の表引きで済ませる（400×400 を Python で回さない）。
    # 3値の外の値は透明にする
    table = np.zeros((256, 4), dtype=np.uint8)
    table[:len(_TRINARY_RGBA)] = _TRINARY_RGBA
    rgba = table[np.ascontiguousarray(trinary, dtype=np.uint8)]
    return {
        "timestamp": _stamp(unix_ns),
        "frame_id": frame_id,
        "pose": {"position": {"x": origin_x, "y": origin_y, "z": 0.0},
                 "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
        "column_count": int(width),
        "cell_size": {"x": resolution, "y": resolution},
        "row_stride": int(width) * 4,
        "cell_stride": 4,
        "fields": [{"name": n, "offset": i, "type": _UINT8}
                   for i, n in enumerate(("red", "green", "blue", "alpha"))],
        "data": base64.b64encode(rgba.tobytes()).decode("ascii"),
    }


def _line(xy: list[float], color: tuple[float, float, float, float],
          thickness: float = 0.03) -> dict:
    """`[x0, y0, x1, y1, ...]` → LinePrimitive（LINE_STRIP）。"""
    return {"type": _LINE_STRIP, "pose": _IDENTITY_POSE, "thickness": thickness,
            "scale_invariant": False,
            "points": [_xyz(xy[i], xy[i + 1])
                       for i in range(0, len(xy) - 1, 2)],
            "color": dict(zip("rgba", color))}


def _entity(eid: str, frame_id: str, unix_ns: int, *, frame_locked: bool = False,
            lines=(), spheres=(), arrows=()) -> dict:
    return {"timestamp": _stamp(unix_ns), "frame_id": frame_id, "id": eid,
            "lifetime": {"sec": 0, "nsec": 0}, "frame_locked": frame_locked,
            "lines": list(lines), "spheres": list(spheres), "arrows": list(arrows)}


class _Path:
    """軌跡の点列。**点数に上限を設け、超えたら間隔を倍にして間引き直す**。"""

    def __init__(self, step: float = PATH_MIN_STEP_M, cap: int = PATH_MAX_POINTS) -> None:
        self.step = step
        self.cap = cap
        self.pts: list[tuple[float, float, float]] = []

    def add(self, x: float, y: float, yaw: float) -> None:
        if self.pts:
            px, py, _ = self.pts[-1]
            if math.hypot(x - px, y - py) < self.step:
                return
        self.pts.append((x, y, yaw))
        if len(self.pts) > self.cap:
            self.step *= 2.0
            thinned = [self.pts[0]]
            for p in self.pts[1:]:
                q = thinned[-1]
                if math.hypot(p[0] - q[0], p[1] - q[1]) >= self.step:
                    thinned.append(p)
            self.pts = thinned


# ── 本体 ────────────────────────────────────────────────────────────────

class VizRecorder:
    """バスのメッセージを受けて、自己位置と地図の可視化トピックを MCAP に書く。

    :param log: 書き込み先
    :param odom: 推測航法（`/odom` `/tf` `/viz/odom/*`）を書くか
    """

    def __init__(self, log: McapLog, *, odom: bool = True) -> None:
        self.log = log
        self.odom_enabled = odom
        self.dr = DeadReckoner()
        self.last_odom: OdomPose | None = None
        self._odom_path = _Path()
        self._slam_path = _Path()
        self._next: dict[str, int] = {}
        #: `slam` の軌跡を始めた planner の id。**planner が変われば地図の原点が変わる**ので軌跡を切る
        self._slam_mode: str | None = None
        self._map_pending = None
        self._map_last_s: float | None = None
        self._park_drawn = False

    def _due(self, key: str, t_ns: int, period_ns: int) -> bool:
        """`key` の書き込み時刻が来たか。**記録の時刻で数える**（sfl2mcap は実時間より速い）。"""
        nxt = self._next.get(key)
        if nxt is not None and nxt - 2 * period_ns <= t_ns < nxt:
            return False
        self._next[key] = t_ns + period_ns
        return True

    # ── 推測航法 ──

    def on_vehicle_state(self, st) -> None:
        if not self.odom_enabled:
            return
        p = self.dr.update(st)
        self.last_odom = p
        t = p.t_capture
        u = self.log.to_unix_ns(t)
        self.log.write("/odom", p, t_mono_ns=t)
        self._odom_path.add(p.x, p.y, p.yaw)
        if self._due("tf", t, TF_PERIOD_NS):
            self.log.write_dict("/tf", frame_transform(ODOM_FRAME_ID, BASE_FRAME_ID,
                                                       p.x, p.y, p.yaw, u),
                                "foxglove.FrameTransform",
                                FOXGLOVE_FRAME_TRANSFORM_SCHEMA, t)
        if self._due("odom_pose", t, POSE_PERIOD_NS):
            self.log.write_dict("/viz/odom/pose",
                                pose_in_frame(ODOM_FRAME_ID, p.x, p.y, p.yaw, u),
                                "foxglove.PoseInFrame", FOXGLOVE_POSE_IN_FRAME_SCHEMA, t)
        if self._due("odom_path", t, PATH_PERIOD_NS):
            self.log.write_dict("/viz/odom/path",
                                poses_in_frame(ODOM_FRAME_ID, self._odom_path.pts, u),
                                "foxglove.PosesInFrame", FOXGLOVE_POSES_IN_FRAME_SCHEMA, t)

    # ── planner の SLAM 姿勢・駐車 ──

    def on_auto_state(self, s) -> None:
        t = s.t_capture or s.t_pub
        u = self.log.to_unix_ns(t)
        if s.phase:
            if s.mode != self._slam_mode:
                self._slam_mode = s.mode
                self._slam_path = _Path()
            self._slam_path.add(s.pose_x, s.pose_y, s.pose_yaw)
            if self._due("slam_pose", t, POSE_PERIOD_NS):
                self.log.write_dict("/viz/slam/pose",
                                    pose_in_frame(MAP_FRAME_ID, s.pose_x, s.pose_y,
                                                  s.pose_yaw, u),
                                    "foxglove.PoseInFrame",
                                    FOXGLOVE_POSE_IN_FRAME_SCHEMA, t)
            if self._due("slam_path", t, PATH_PERIOD_NS):
                self.log.write_dict("/viz/slam/path",
                                    poses_in_frame(MAP_FRAME_ID, self._slam_path.pts, u),
                                    "foxglove.PosesInFrame",
                                    FOXGLOVE_POSES_IN_FRAME_SCHEMA, t)
        self._park(s, t, u)
        self._flush_map(t)

    def _park(self, s, t: int, u: int) -> None:
        """駐車の計画経路と目標。**base_link 基準の座標**なので `frame_locked` で車に固定する。"""
        if not s.park_active:
            if self._park_drawn:
                self._park_drawn = False
                self.log.write_dict("/viz/park", {"deletions": [
                    {"timestamp": _stamp(u), "type": _DELETE_ALL, "id": ""}], "entities": []},
                    "foxglove.SceneUpdate", FOXGLOVE_SCENE_UPDATE_SCHEMA, t)
            return
        if not self._due("park", t, POSE_PERIOD_NS):
            return
        self._park_drawn = True
        lines = []
        xs, ys = s.park_path_x, s.park_path_y
        n = min(len(xs), len(ys))
        if n >= 2:
            k = s.park_path_reverse_from if 0 <= s.park_path_reverse_from < n else n
            fwd = [v for i in range(min(k + 1, n)) for v in (xs[i], ys[i])]
            rev = [v for i in range(k, n) for v in (xs[i], ys[i])]
            if len(fwd) >= 4:
                lines.append(_line(fwd, (0.1, 0.6, 1.0, 1.0)))
            if len(rev) >= 4:
                lines.append(_line(rev, (1.0, 0.5, 0.1, 1.0)))
        arrow = {"pose": _pose2d(s.park_target_x, s.park_target_y, s.park_target_yaw),
                 "shaft_length": 0.25, "shaft_diameter": 0.03,
                 "head_length": 0.08, "head_diameter": 0.08,
                 "color": {"r": 0.2, "g": 0.9, "b": 0.3, "a": 1.0}}
        ent = _entity("park", BASE_FRAME_ID, u, frame_locked=True,
                      lines=lines, arrows=[arrow])
        self.log.write_dict("/viz/park", {"deletions": [], "entities": [ent]},
                            "foxglove.SceneUpdate", FOXGLOVE_SCENE_UPDATE_SCHEMA, t)

    # ── 地図 ──

    def on_auto_map(self, m) -> None:
        self._map_pending = m
        self._flush_map(m.t_capture or m.t_pub)

    def _flush_map(self, t_now: int, force: bool = False) -> None:
        m = self._map_pending
        if m is None:
            return
        now_s = t_now / NS
        if not force and self._map_last_s is not None \
                and 0.0 <= now_s - self._map_last_s < MAP_MIN_PERIOD_S:
            return
        self._map_pending = None
        self._map_last_s = now_s
        t = m.t_capture or m.t_pub or t_now
        u = self.log.to_unix_ns(t)
        if m.width > 0 and m.height > 0 and m.cells:
            try:
                from ..nav.grid import unpack_trinary
                tri = unpack_trinary(m.cells, m.width, m.height)
            except (ValueError, ImportError):
                tri = None
            if tri is not None:
                self.log.write_dict("/viz/map", grid_from_trinary(
                    tri, resolution=m.resolution, origin_x=m.origin_x,
                    origin_y=m.origin_y, unix_ns=u),
                    "foxglove.Grid", FOXGLOVE_GRID_SCHEMA, t)
        lines = []
        if len(m.centerline) >= 4:
            lines.append(_line(m.centerline, (0.6, 0.6, 0.6, 0.8), 0.02))
        if len(m.raceline) >= 4:
            lines.append(_line(m.raceline, (1.0, 0.2, 0.3, 1.0), 0.04))
        if lines:
            self.log.write_dict("/viz/map/lines", {"deletions": [], "entities": [
                _entity("map_lines", MAP_FRAME_ID, u, lines=lines)]},
                "foxglove.SceneUpdate", FOXGLOVE_SCENE_UPDATE_SCHEMA, t)

    # ── 後始末 ──

    def close(self) -> None:
        """保留中の地図と最後の軌跡を書く。**`McapLog.close()` より先に呼ぶこと。**"""
        if self._map_pending is not None:
            m = self._map_pending
            self._flush_map(m.t_capture or m.t_pub, force=True)
        if self.last_odom is not None and self._odom_path.pts:
            t = self.last_odom.t_capture
            self.log.write_dict("/viz/odom/path",
                                poses_in_frame(ODOM_FRAME_ID, self._odom_path.pts,
                                               self.log.to_unix_ns(t)),
                                "foxglove.PosesInFrame", FOXGLOVE_POSES_IN_FRAME_SCHEMA, t)
