"""実機/SIM共通テレメトリ・スキーマ（データ契約）。

UNIFIED_UI_DESIGN.md の確定事項に基づく、UI⇔バックエンド間の共通フォーマット。
このモジュールは「直列化／復元」だけを担い、通信(WebSocket)や制御には依存しない。

確定事項:
  ① 封筒(envelope)はJSON、LiDAR・占有格子は配列なのでbinaryフレーム
  ② 占有格子は全体をbinaryで低レート、姿勢/LiDARは高レートで別送
  ⑤ ワイヤはSI統一: 位置[m]/速度[m/s]/角度[deg], heading 0=East反時計回り
     LiDARのみ uint16 mm（LD06ネイティブ, 0=範囲外, リトルエンディアン）

バイナリ仕様:
  LiDAR : uint16 little-endian × n。各値は距離[mm]。0=範囲外/無効。
  占有格子: int8 × (h*w)。行優先(row-major)。0=free, 1=occupied, -1=unknown。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from core.interfaces import LidarScan, OccupancyGrid

SCHEMA_VERSION = 1
LIDAR_DTYPE = np.dtype("<u2")   # uint16 little-endian [mm]
GRID_DTYPE = np.dtype("i1")     # int8

# ---------------------------------------------------------------------------
# テレメトリ・フレーム（ダウンリンク: バックエンド → UI）
# ---------------------------------------------------------------------------
@dataclass
class TelemetryFrame:
    """1フレーム分のテレメトリ（JSON封筒部分）。

    LiDAR距離・占有格子の実データは別binaryフレームで送るため、ここには
    メタ情報(lidar_meta/map_meta)のみ持つ。
    """

    t: float                       # [s] タイムスタンプ
    source: str                    # "sim" | "real"
    drive_mode: str                # "manual" | "auto" | "reactive"
    vehicle: dict                  # {"speed","accel","steer"}  SI
    pose_est: dict                 # {"x","y","heading","conf","src"}
    command: dict                  # {"target_speed","target_steer"}
    planner: dict                  # {"target_point":[x,y]|None, "has_path":bool}
    health: dict                   # {"comm_ok":bool, "estop":bool}
    lidar_meta: dict | None = None    # {"n","max_range_mm"}（実データは別binary）
    pose_truth: dict | None = None    # SIM専用デバッグ {"x","y","heading"}
    sim_ctrl: dict | None = None      # SIM専用 {"paused","speed_mult"}
    map_meta: dict | None = None      # {"res","ox","oy","w","h"}（実データは別binary）
    hw: dict | None = None            # 実機HWテレメトリ（任意）
    slam: dict | None = None          # {"mapping":bool, "has_rl":bool}

    # --- JSON封筒へ ---------------------------------------------------
    def to_envelope(self) -> dict:
        env = {
            "type": "telemetry",
            "v": SCHEMA_VERSION,
            "t": round(self.t, 4),
            "source": self.source,
            "drive_mode": self.drive_mode,
            "vehicle": _round_dict(self.vehicle, 3),
            "pose_est": _round_dict(self.pose_est, 4),
            "command": _round_dict(self.command, 3),
            "planner": self.planner,
            "health": self.health,
        }
        if self.lidar_meta is not None:
            env["lidar"] = self.lidar_meta
        if self.pose_truth is not None:
            env["pose_truth"] = _round_dict(self.pose_truth, 4)
        if self.sim_ctrl is not None:
            env["sim_ctrl"] = self.sim_ctrl
        if self.map_meta is not None:
            env["map"] = self.map_meta
        if self.hw is not None:
            env["hw"] = self.hw
        if self.slam is not None:
            env["slam"] = self.slam
        return env

    # --- JSON封筒から ------------------------------------------------
    @classmethod
    def from_envelope(cls, d: dict) -> "TelemetryFrame":
        return cls(
            t=d["t"],
            source=d["source"],
            drive_mode=d["drive_mode"],
            vehicle=d["vehicle"],
            pose_est=d["pose_est"],
            command=d["command"],
            planner=d["planner"],
            health=d["health"],
            lidar_meta=d.get("lidar"),
            pose_truth=d.get("pose_truth"),
            sim_ctrl=d.get("sim_ctrl"),
            map_meta=d.get("map"),
            hw=d.get("hw"),
        )


# ---------------------------------------------------------------------------
# ControllerSnapshot からテレメトリを組み立てる
# ---------------------------------------------------------------------------
def build_telemetry(snapshot, source: str = "sim",
                    pose_truth: dict | None = None,
                    comm_ok: bool = True, estop: bool = False,
                    hw: dict | None = None) -> TelemetryFrame:
    """ControllerSnapshot（duck-typed）から TelemetryFrame を作る。

    snapshot に期待する属性: state, scan, command, localization, drive_mode,
    paused, speed_multiplier, sim_time, target_point, path
    """
    st = snapshot.state
    loc = snapshot.localization
    cmd = snapshot.command
    scan = snapshot.scan

    vehicle = {"speed": float(st.speed), "accel": float(st.acceleration),
               "steer": float(st.steer_angle)}

    if loc is not None:
        pose_est = {"x": float(loc.x), "y": float(loc.y),
                    "heading": float(loc.heading),
                    "conf": float(loc.confidence), "src": loc.source}
    else:
        # 推定がまだ無い場合は車両状態を暫定表示（src=none）
        pose_est = {"x": float(st.x), "y": float(st.y),
                    "heading": float(st.heading), "conf": 0.0, "src": "none"}

    command = {"target_speed": float(cmd.target_speed),
               "target_steer": float(cmd.target_steer)}

    tp = snapshot.target_point
    planner = {"target_point": [round(float(tp[0]), 4), round(float(tp[1]), 4)]
               if tp is not None else None,
               "has_path": snapshot.path is not None}

    lidar_meta = None
    if scan is not None:
        lidar_meta = {"n": int(scan.distances.size), "max_range_mm": 12000}

    sim_ctrl = None
    if source == "sim":
        sim_ctrl = {"paused": bool(snapshot.paused),
                    "speed_mult": float(snapshot.speed_multiplier)}

    slam = {"mapping": bool(getattr(snapshot, "mapping", False)),
            "has_rl": bool(getattr(snapshot, "has_racing_line", False))}

    return TelemetryFrame(
        t=float(snapshot.sim_time),
        source=source,
        drive_mode=snapshot.drive_mode,
        vehicle=vehicle,
        pose_est=pose_est,
        command=command,
        planner=planner,
        health={"comm_ok": comm_ok, "estop": estop},
        lidar_meta=lidar_meta,
        pose_truth=pose_truth,
        sim_ctrl=sim_ctrl,
        hw=hw,
        slam=slam,
    )


# ---------------------------------------------------------------------------
# UIビュー（UIが描画に使う統合ビューモデル）
# ---------------------------------------------------------------------------
@dataclass
class UIView:
    """UI(pygame/Web)が1フレーム描画するのに必要な全データ。

    in-process(SIMローカル)でもネットワーク(実機・遠隔)でも、UIはこの形だけを読む。
    network経由の場合は受信した envelope/binary を復元してこれを組み立てる。
    """

    frame: TelemetryFrame
    lidar: LidarScan | None
    scene: SceneFrame
    grid: OccupancyGrid | None = None


def build_view(snapshot, scene: "SceneFrame", source: str = "sim",
               grid: OccupancyGrid | None = None,
               include_truth: bool = True) -> UIView:
    """ControllerSnapshot から UIView を組み立てる。

    include_truth=True（pygameデバッグビューア）の時のみ真値(pose_truth)を同梱する。
    Web等の運用UIへ配信する場合は include_truth=False とし、真値を載せない
    （実機には真値が存在しないため、SIMでも運用UIには出さない）。
    """
    pose_truth = None
    if include_truth and source == "sim":
        st = snapshot.state
        pose_truth = {"x": float(st.x), "y": float(st.y),
                      "heading": float(st.heading)}
    frame = build_telemetry(snapshot, source=source, pose_truth=pose_truth)
    return UIView(frame=frame, lidar=snapshot.scan, scene=scene, grid=grid)


# ---------------------------------------------------------------------------
# LiDAR バイナリ（uint16 LE mm）
# ---------------------------------------------------------------------------
def encode_lidar(scan: LidarScan) -> bytes:
    """LidarScan(距離[m]) → uint16 LE mm のバイナリ。0=範囲外/無効。"""
    d_m = np.asarray(scan.distances, dtype=np.float64)
    d_mm = np.clip(np.round(d_m * 1000.0), 0, 65535).astype(LIDAR_DTYPE)
    return d_mm.tobytes()


def decode_lidar(buf: bytes, max_range_mm: int = 12000) -> LidarScan:
    """uint16 LE mm のバイナリ → LidarScan(距離[m], 角度[deg]=index)。"""
    d_mm = np.frombuffer(buf, dtype=LIDAR_DTYPE).astype(np.float64)
    distances = d_mm / 1000.0
    # 0(範囲外/無効)は最大レンジへ
    distances[d_mm == 0] = max_range_mm / 1000.0
    n = distances.size
    angles = np.arange(n, dtype=np.float64) * (360.0 / n if n else 1.0)
    return LidarScan(distances=distances, angles=angles, timestamp=0.0)


# ---------------------------------------------------------------------------
# 占有格子 バイナリ（int8 row-major）
# ---------------------------------------------------------------------------
def grid_meta(grid: OccupancyGrid) -> dict:
    h, w = grid.grid.shape
    return {"res": float(grid.resolution), "ox": float(grid.origin_x),
            "oy": float(grid.origin_y), "w": int(w), "h": int(h)}


def encode_grid(grid: OccupancyGrid) -> bytes:
    """OccupancyGrid → int8 row-major バイナリ。"""
    return np.asarray(grid.grid, dtype=GRID_DTYPE).tobytes()


def decode_grid(buf: bytes, meta: dict, timestamp: float = 0.0) -> OccupancyGrid:
    """int8 バイナリ + メタ → OccupancyGrid。"""
    h, w = int(meta["h"]), int(meta["w"])
    arr = np.frombuffer(buf, dtype=GRID_DTYPE).astype(np.int8).reshape(h, w)
    return OccupancyGrid(grid=arr, resolution=float(meta["res"]),
                         origin_x=float(meta["ox"]), origin_y=float(meta["oy"]),
                         timestamp=timestamp)


# ---------------------------------------------------------------------------
# コマンド・フレーム（アップリンク: UI → バックエンド）
# ---------------------------------------------------------------------------
@dataclass
class CommandFrame:
    """UI からの操作指令。name で種別を判別する。"""

    name: str          # manual_input|set_mode|estop|reset|pause|speed_mult|set_course
    payload: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {"type": "cmd", "v": SCHEMA_VERSION, "name": self.name, **self.payload}

    @classmethod
    def from_dict(cls, d: dict) -> "CommandFrame":
        payload = {k: v for k, v in d.items() if k not in ("type", "v", "name")}
        return cls(name=d["name"], payload=payload)


def build_command(name: str, **payload) -> CommandFrame:
    return CommandFrame(name=name, payload=payload)


def apply_command(controller, cmd: CommandFrame, course_resolver=None) -> None:
    """CommandFrame を Controller のメソッド呼び出しへ写像する。

    course_resolver: set_course 用の name->course(dict) 解決関数（任意）。
    """
    name = cmd.name
    p = cmd.payload
    if name == "manual_input":
        controller.set_manual_targets(float(p.get("speed", 0.0)),
                                      float(p.get("steer", 0.0)))
    elif name == "set_mode":
        controller.set_mode(p.get("mode", "manual"))
    elif name == "estop":
        # 緊急停止: 手動へ落として速度0
        controller.set_mode("manual")
        controller.set_manual_targets(0.0, 0.0)
    elif name == "reset":
        controller.reset()
    elif name == "pause":
        controller.set_paused(bool(p.get("value", True)))
    elif name == "speed_mult":
        controller.set_speed_multiplier(float(p.get("value", 1.0)))
    elif name == "toggle_mapping":
        controller.toggle_mapping()
    elif name == "start_mapping":
        controller.start_mapping()
    elif name == "stop_mapping":
        controller.stop_mapping()
    elif name == "build_racing_line":
        controller.build_racing_line()
    elif name == "set_course":
        if course_resolver is not None:
            course = course_resolver(p.get("course"))
            if course is not None:
                controller.set_course(course)
    # 未知のnameは無視（前方互換）


# ---------------------------------------------------------------------------
# シーン・フレーム（接続時/コース変更時に1回）
# ---------------------------------------------------------------------------
@dataclass
class SceneFrame:
    """コース静的情報。実機(壁の真値なし)では walls=None。

    racing_line / slam_center は SLAM＋最適化で生成され次第セットされ、
    生成時にサーバが再ブロードキャストする。
    """

    source: str
    walls: list | None = None          # [[[x1,y1],[x2,y2]], ...] SIM真値
    center_line: list | None = None    # [[x,y], ...] カンニング中心線
    racing_line: list | None = None    # [[x,y], ...] 最適化レーシングライン
    slam_center: list | None = None    # [[x,y], ...] SLAM抽出中心線

    def to_dict(self) -> dict:
        return {"type": "scene", "v": SCHEMA_VERSION, "source": self.source,
                "walls": self.walls, "center_line": self.center_line,
                "racing_line": self.racing_line, "slam_center": self.slam_center}

    @classmethod
    def from_dict(cls, d: dict) -> "SceneFrame":
        return cls(source=d.get("source", "sim"), walls=d.get("walls"),
                   center_line=d.get("center_line"),
                   racing_line=d.get("racing_line"), slam_center=d.get("slam_center"))


def _pts_to_list(pts):
    if pts is None:
        return None
    return [[float(x), float(y)] for x, y in np.asarray(pts).tolist()]


def build_scene(source: str, walls=None, center_line=None,
                racing_line=None, slam_center=None) -> SceneFrame:
    """壁(線分タプル列)・各経路(numpy可) を JSON-safe な SceneFrame にする。"""
    w = None
    if walls is not None:
        w = [[[float(a[0]), float(a[1])], [float(b[0]), float(b[1])]]
             for (a, b) in walls]
    return SceneFrame(source=source, walls=w,
                      center_line=_pts_to_list(center_line),
                      racing_line=_pts_to_list(racing_line),
                      slam_center=_pts_to_list(slam_center))


# ---------------------------------------------------------------------------
def _round_dict(d: dict, ndigits: int) -> dict:
    out = {}
    for k, v in d.items():
        out[k] = round(v, ndigits) if isinstance(v, float) else v
    return out
