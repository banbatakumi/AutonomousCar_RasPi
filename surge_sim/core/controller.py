"""制御ループ（50Hz）。

UIループ（pygame描画）から独立したスレッドで制御周期を回す。
LocalizationResult.source に応じて使用する Localizer を自動切替する。
手動操作モードと自律走行モードをフラグで切り替える。

スレッド間共有データはロックで保護し、UI側は get_snapshot() で安全に読む。
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass

import numpy as np

from core.course_analyzer import CourseAnalyzer, build_course_map
from core.interfaces import (
    BackendBase,
    ControlCommand,
    CourseMap,
    LidarScan,
    LocalizationResult,
    OccupancyGrid,
    VehicleState,
)
from core.localization import CheatLocalizer, SLAMLocalizer
from core.occupancy import OccupancyGridMapper
from core.path_utils import extract_loop
from core.planner import PurePursuitPlanner
from core.racing_line import RacingLineOptimizer
from core.reactive_planner import ReactivePlanner


@dataclass
class ControllerSnapshot:
    """UI描画用の状態スナップショット。"""

    state: VehicleState
    scan: LidarScan | None
    localization: LocalizationResult | None
    command: ControlCommand
    sim_time: float
    paused: bool
    autonomous: bool
    speed_multiplier: float
    drive_mode: str = "manual"   # "manual" | "auto" | "reactive"
    path: np.ndarray | None = None
    target_point: np.ndarray | None = None
    mapping: bool = False
    has_racing_line: bool = False


class Controller:
    """50Hz制御スレッド。"""

    def __init__(self, backend: BackendBase, config: dict,
                 control_hz: float = 50.0):
        self.backend = backend
        self.config = config
        self.dt = 1.0 / control_hz
        self.control_hz = control_hz

        # Localizer群（source文字列で切替）
        loc_mode = config.get("localization", {}).get("mode", "cheat")
        self._cheat = CheatLocalizer()
        self._slam: SLAMLocalizer | None = None  # 必要時(loc_mode=="slam")に生成
        self.loc_mode = loc_mode

        # 手動指令値
        self._manual_speed = 0.0
        self._manual_steer = 0.0
        # 走行モード: "manual" | "auto"(地図追従) | "reactive"(LiDAR直接)
        self.drive_mode = "manual"

        # 実行制御
        self.paused = False
        self.speed_multiplier = 1.0
        self._running = False
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._sim_time = 0.0

        # 最新スナップショット要素
        self._last_scan: LidarScan | None = None
        self._last_loc: LocalizationResult | None = None
        self._last_cmd = ControlCommand()

        # 車両パラメータ（手動操作の刻み・上限）
        v = config["vehicle"]
        self.max_speed = float(v["max_speed"])
        self.max_steer = float(v["max_steer_angle"])

        # 経路追従（Phase2）
        pp = config.get("planner", {})
        self.planner = PurePursuitPlanner(
            wheelbase=float(v["wheelbase"]),
            max_steer=self.max_steer,
            lookahead_distance=float(pp.get("lookahead_distance", 0.5)),
            lookahead_gain=float(pp.get("lookahead_gain", 0.3)),
            min_lookahead=float(pp.get("min_lookahead", 0.3)),
            max_lookahead=float(pp.get("max_lookahead", 1.5)),
            cruise_speed=float(pp.get("cruise_speed", 1.5)),
            max_speed=self.max_speed,
            curvature_slowdown=float(pp.get("curvature_slowdown", 0.6)),
        )
        self.course_map: CourseMap | None = None
        self.path: np.ndarray | None = None
        self._last_target: np.ndarray | None = None

        # リアクティブ制御（地図不要・LiDAR直接）
        self.reactive = ReactivePlanner(
            max_steer=self.max_steer,
            cruise_speed=float(pp.get("cruise_speed", 1.5)),
            fov_deg=float(pp.get("reactive_fov_deg", 90.0)),
            bubble_m=float(pp.get("reactive_bubble_m", 0.30)),
        )

        # SLAMマッピング＆レーシングライン生成（Phase3-4）
        self.map_resolution = float(config.get("slam", {}).get("resolution", 0.05))
        self.mapper: OccupancyGridMapper | None = None
        self._map_bounds: tuple | None = None
        self._start_xy = (0.0, 0.0)
        self.mapping = False
        self._traj: list = []
        self._map_decim = 0
        self.occupancy: OccupancyGrid | None = None
        self.racing_line: np.ndarray | None = None
        self.slam_center: np.ndarray | None = None
        self.speed_profile: np.ndarray | None = None

    # ==================================================================
    # スレッド制御
    # ==================================================================
    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    # ==================================================================
    # 制御ループ本体
    # ==================================================================
    def _loop(self) -> None:
        next_t = time.perf_counter()
        while self._running:
            now = time.perf_counter()
            if now < next_t:
                time.sleep(min(next_t - now, self.dt))
                continue
            next_t += self.dt

            if not self.paused:
                self._tick()

    def _tick(self) -> None:
        """1制御周期分の処理。"""
        sim_dt = self.dt * self.speed_multiplier

        # 1. 指令生成
        cmd = self._compute_command()

        # 2. 指令送信
        self.backend.send_command(cmd)

        # 3. 物理を進める
        self.backend.step(sim_dt)

        # 4. センサ取得
        scan = self.backend.get_lidar_scan()
        state = self.backend.get_vehicle_state()

        # 5. 自己位置推定（sourceに応じて切替）
        loc = self._localize(state, scan)

        # 6. SLAMマッピング（有効時、推定姿勢で占有格子へ統合）
        if self.mapping and self.mapper is not None:
            self._map_decim += 1
            if self._map_decim >= 4:   # ~制御周期/4 でマッピング
                self._map_decim = 0
                self.mapper.integrate_scan(scan, loc)
                self._traj.append((loc.x, loc.y))
                # ライブ地図を配信用に随時更新
                self.occupancy = self.mapper.to_occupancy_grid(self._sim_time)

        with self._lock:
            self._sim_time += sim_dt
            self._last_scan = scan
            self._last_loc = loc
            self._last_cmd = cmd

    # ------------------------------------------------------------------
    def _compute_command(self) -> ControlCommand:
        """走行モードに応じた制御指令を生成する。"""
        ts = time.time()

        # 地図追従（Pure Pursuit）: 既知の中心線＋自己位置
        if self.drive_mode == "auto" and self.path is not None and len(self.path) >= 2:
            state = self.backend.get_vehicle_state()
            loc = self._localize(state, None)
            cmd = self.planner.compute_command(loc, self.path,
                                               current_speed=state.speed)
            self._last_target = self.planner.last_target
            return cmd

        # リアクティブ（Follow the Gap）: LiDARスキャンのみで制御。地図・位置不要
        if self.drive_mode == "reactive":
            scan = self.backend.get_lidar_scan()
            cmd = self.reactive.compute_command(scan)
            # 可視化用に選択方向をワールド点へ変換（真値poseは描画専用、制御には未使用）
            self._last_target = self._reactive_target_world()
            return cmd

        # 手動
        with self._lock:
            return ControlCommand(target_speed=self._manual_speed,
                                  target_steer=self._manual_steer,
                                  timestamp=ts)

    def _reactive_target_world(self) -> np.ndarray:
        """リアクティブの選択方向を描画用にワールド座標へ変換する。"""
        st = self.backend.get_vehicle_state()
        h = math.radians(st.heading + self.reactive.best_angle)
        dist = max(self.reactive.best_dist, 0.3)
        return np.array([st.x + dist * math.cos(h), st.y + dist * math.sin(h)])

    def _localize(self, state: VehicleState,
                  scan: LidarScan) -> LocalizationResult:
        """loc_mode に応じて Localizer を切り替える。"""
        if self.loc_mode == "slam" and scan is not None and self._map_bounds is not None:
            if self._slam is None:
                self._slam = SLAMLocalizer(
                    bounds=self._map_bounds,
                    start_pose=(self._start_xy[0], self._start_xy[1], state.heading),
                    resolution=self.map_resolution)
            return self._slam.update(scan)
        return self._cheat.update(state)

    # ==================================================================
    # UIスレッドから呼ぶAPI
    # ==================================================================
    def set_manual_targets(self, speed: float, steer: float) -> None:
        with self._lock:
            self._manual_speed = max(-self.max_speed, min(self.max_speed, speed))
            self._manual_steer = max(-self.max_steer, min(self.max_steer, steer))

    def adjust_speed(self, delta: float) -> None:
        with self._lock:
            self._manual_speed = max(-self.max_speed,
                                     min(self.max_speed, self._manual_speed + delta))

    def adjust_steer(self, delta: float) -> None:
        with self._lock:
            self._manual_steer = max(-self.max_steer,
                                     min(self.max_steer, self._manual_steer + delta))

    def set_steer(self, value: float) -> None:
        with self._lock:
            self._manual_steer = max(-self.max_steer, min(self.max_steer, value))

    def toggle_pause(self) -> None:
        self.paused = not self.paused

    def set_paused(self, value: bool) -> None:
        self.paused = value

    def set_speed_multiplier(self, mult: float) -> None:
        self.speed_multiplier = float(mult)

    @property
    def autonomous(self) -> bool:
        """手動以外（自動／リアクティブ）かどうか。"""
        return self.drive_mode != "manual"

    def set_mode(self, mode: str) -> None:
        """走行モードを設定する。auto は経路が必要。"""
        if mode == "auto" and self.path is None:
            return
        if mode not in ("manual", "auto", "reactive"):
            return
        self.drive_mode = mode
        self._last_target = None

    def toggle_mode(self, mode: str) -> None:
        """指定モードと手動をトグルする（A/Fキー用）。"""
        self.set_mode("manual" if self.drive_mode == mode else mode)

    # 後方互換
    def set_autonomous(self, value: bool) -> None:
        self.set_mode("auto" if value else "manual")

    def toggle_autonomous(self) -> None:
        self.toggle_mode("auto")

    def set_course(self, course: dict) -> None:
        """コース定義から CourseMap・追従経路を構築する（Phase2カンニング）。"""
        cmap = build_course_map(course)
        with self._lock:
            self.course_map = cmap
            self.path = cmap.center_line if len(cmap.center_line) else None
            self._last_target = None
            if self.path is None and self.drive_mode == "auto":
                self.drive_mode = "manual"

        # SLAMマッピング用の地図範囲・スタート位置（配列サイズのみ。中身はLiDAR由来）
        self._start_xy = tuple(course["start_pose"][:2])
        walls = course.get("walls", [])
        if walls:
            xs = [p for seg in walls for p in (seg[0][0], seg[1][0])]
            ys = [p for seg in walls for p in (seg[0][1], seg[1][1])]
            self._map_bounds = (min(xs) - 0.5, min(ys) - 0.5,
                                max(xs) + 0.5, max(ys) + 0.5)
        # 旧地図・走行ラインは破棄
        self.mapper = None
        self.mapping = False
        self.occupancy = None
        self.racing_line = None
        self.slam_center = None
        self.speed_profile = None
        self._traj = []

    # ==================================================================
    # SLAMマッピング／レーシングライン生成（Phase3-4）
    # ==================================================================
    def start_mapping(self) -> None:
        """占有格子マッピングを開始する（走りながら地図を作る）。"""
        if self._map_bounds is None:
            return
        self.mapper = OccupancyGridMapper(*self._map_bounds,
                                          resolution=self.map_resolution)
        self._traj = []
        self.mapping = True

    def stop_mapping(self) -> None:
        self.mapping = False
        if self.mapper is not None:
            self.occupancy = self.mapper.to_occupancy_grid(self._sim_time)

    def toggle_mapping(self) -> None:
        if self.mapping:
            self.stop_mapping()
        else:
            self.start_mapping()

    def build_racing_line(self) -> bool:
        """構築済み地図＋探索軌跡から中心線・レーシングライン・速度を生成する。"""
        if self.mapper is None or len(self._traj) < 30:
            return False
        grid = self.mapper.to_occupancy_grid(self._sim_time)
        lap = extract_loop(self._traj, start_xy=self._start_xy)
        if len(lap) < 10:
            return False
        try:
            analyzer = CourseAnalyzer(spacing=0.08, smooth_window=13, recenter_iters=4)
            cmap = analyzer.analyze(grid, seed_path=lap)
            opt = RacingLineOptimizer(safety_margin=0.15)
            racing = opt.optimize(cmap)
            speed = opt.compute_speed_profile(racing, self.max_speed)
        except Exception:
            return False
        with self._lock:
            self.occupancy = grid
            self.course_map = cmap
            self.slam_center = cmap.center_line
            self.racing_line = racing
            self.speed_profile = speed
            self.path = racing            # AUTO はレーシングラインを追従
            self._last_target = None
        return True

    def get_occupancy(self) -> OccupancyGrid | None:
        return self.occupancy

    def reset(self) -> None:
        with self._lock:
            self.backend.reset()
            self._manual_speed = 0.0
            self._manual_steer = 0.0
            self._sim_time = 0.0
            self._last_scan = None
            self._last_loc = None
            self._last_cmd = ControlCommand()

    # ------------------------------------------------------------------
    def get_snapshot(self) -> ControllerSnapshot:
        """UI描画用の最新スナップショットを取得する。"""
        with self._lock:
            return ControllerSnapshot(
                state=self.backend.get_vehicle_state(),
                scan=self._last_scan,
                localization=self._last_loc,
                command=self._last_cmd,
                sim_time=self._sim_time,
                paused=self.paused,
                autonomous=self.autonomous,
                speed_multiplier=self.speed_multiplier,
                drive_mode=self.drive_mode,
                path=self.path,
                target_point=self._last_target,
                mapping=self.mapping,
                has_racing_line=self.racing_line is not None,
            )
