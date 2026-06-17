"""制御ループ（50Hz）。

制御ループを UI ループ・WebSocket と分離した専用スレッドで回す。
DriveMode に応じて動作を切り替える：
  MANUAL       : SharedState の ControlCommand をそのままバックエンドへ送信
  MAP_BUILDING : MANUAL と同じ操作（Phase3 で SLAM エンジン呼び出しを追加）
  AUTONOMOUS   : Planner が生成した ControlCommand を送信（Phase2 で実装）

緊急停止: SharedState の緊急停止フラグで、どのスレッドからも発動できる。
ウォッチドッグ: 最後に ControlCommand を受信してから 500ms で速度0・ステア0を送信。
"""
from __future__ import annotations

import threading
import time

from pathlib import Path

import numpy as np

from .course_analyzer import CourseAnalyzer
from .interfaces import BackendBase, ControlCommand, DriveMode, LocalizationResult
from .localization import CheatLocalizer
from .logger import Logger
from .obstacle import ObstacleAvoider
from .path_utils import extract_loop
from .planner import PurePursuitPlanner
from .racing_line import RacingLineOptimizer
from .reactive_planner import ReactivePlanner
from .shared_state import SharedState
from .slam import HectorSLAM

CONTROL_HZ = 50.0
CONTROL_DT = 1.0 / CONTROL_HZ


class Controller:
    def __init__(self, backend: BackendBase, shared_state: SharedState,
                 logger: Logger | None = None,
                 command_timeout_ms: float = 500.0,
                 is_sim: bool = True,
                 vehicle_cfg: dict | None = None,
                 planner_cfg: dict | None = None,
                 slam_cfg: dict | None = None,
                 racing_cfg: dict | None = None,
                 obstacle_cfg: dict | None = None,
                 reactive_cfg: dict | None = None,
                 localization_mode: str = "slam",
                 saved_maps_dir: str = "saved_maps") -> None:
        self.backend = backend
        self.shared = shared_state
        self.logger = logger
        self.command_timeout = command_timeout_ms / 1000.0
        self.is_sim = is_sim

        # 自己位置推定モード: "slam"（実機相当・LiDARのみ）/ "cheat"（真値・デバッグ用）
        self.loc_mode = localization_mode
        self.localizer = CheatLocalizer()

        # Phase2: 経路追従
        vcfg = vehicle_cfg or {}
        pcfg = dict(planner_cfg or {})
        pcfg.setdefault("max_steer_angle", vcfg.get("max_steer_angle", 40.0))
        pcfg.setdefault("max_speed", vcfg.get("max_speed", 3.0))
        self.analyzer = CourseAnalyzer(
            course_width=float(pcfg.get("course_width", 1.0)),
            waypoint_spacing=float(pcfg.get("waypoint_spacing", 0.1)),
        )
        self.planner = PurePursuitPlanner(
            wheelbase=float(vcfg.get("wheelbase", 0.230)),
            config=pcfg,
        )
        self.path: np.ndarray | None = None
        self.auto_target_speed: float | None = None

        # Phase4: レーシングライン最適化
        rcfg = dict(racing_cfg or {})
        rcfg.setdefault("max_speed", vcfg.get("max_speed", 3.0))
        # 車体半幅 ≈ トレッド/2 + ボディ余裕（壁との実クリアランス確保に使う）
        rcfg.setdefault("vehicle_half_width", vcfg.get("tread", 0.155) / 2.0 + 0.03)
        self.racing_enabled = bool(rcfg.get("enabled", True))
        self.racing_optimizer = RacingLineOptimizer(config=rcfg)
        self.speed_profile: np.ndarray | None = None

        # 動的障害物回避
        ocfg = obstacle_cfg or {}
        self.avoid_enabled = bool(ocfg.get("enabled", True))
        self.avoider = ObstacleAvoider(ocfg, max_steer=float(pcfg.get("max_steer_angle", 40.0)))
        self.obstacle_info: dict = {"detected": False, "estop": False, "distance": None}

        # リアクティブ走行（LiDARのみ・自動探索）
        self.reactive = ReactivePlanner(
            reactive_cfg or {},
            max_steer=float(pcfg.get("max_steer_angle", 40.0)),
            max_speed=float(vcfg.get("max_speed", 3.0)),
        )

        # Phase3 / 実機相当 SLAM
        scfg = slam_cfg or {}
        self.slam = HectorSLAM(resolution=float(scfg.get("resolution", 0.05)))
        self._map_rate_divider = int(scfg.get("map_rate_divider", 4))
        # SLAM 自己位置推定の実行間隔（制御ループ何回に1回）。2 = 25Hz。
        self._slam_loc_divider = int(scfg.get("loc_rate_divider", 2))
        self._map_tick = 0
        self._slam_loc_tick = 0
        self._last_loc: LocalizationResult | None = None
        self.trajectory: list[tuple[float, float]] = []
        self._start_xy: tuple[float, float] = (0.0, 0.0)
        self.has_slam_map = False
        self.saved_maps_dir = Path(saved_maps_dir)

        self._thread: threading.Thread | None = None
        self._running = False
        self._last_cmd_at = time.time()

    # ---- ライフサイクル ---------------------------------------------------
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

    # ---- 外部からの操作（スレッドセーフ：SharedState 経由） ---------------
    def set_command(self, cmd: ControlCommand) -> None:
        self._last_cmd_at = time.time()
        self.shared.set_command(cmd)
        self.shared.set_emergency_stop(False)

    def emergency_stop(self) -> None:
        self.shared.set_emergency_stop(True)
        self.shared.set_command(ControlCommand(0.0, 0.0, time.time()))
        self.shared.set_autonomous_running(False)

    def set_mode(self, mode: DriveMode) -> None:
        self.shared.set_mode(mode)

    def set_course(self, course: dict) -> None:
        """コース定義から追従用 CourseMap（カンニング中心線）を構築する。

        Phase2: center_line をそのまま追従経路に使う。SharedState にも公開して
        WebUI/pygame が経路を表示できるようにする。
        """
        # SLAM マッピング範囲を壁から算出（実機は walls 無し→デフォルト維持）
        start_pose = tuple(course.get("start_pose", (0.0, 0.0, 0.0)))
        self._start_xy = tuple(start_pose[:2])
        walls = course.get("walls")
        if walls:
            pts = np.array(walls, dtype=float).reshape(-1, 2)
            margin = 1.0
            bounds = (pts[:, 0].min() - margin, pts[:, 1].min() - margin,
                      pts[:, 0].max() + margin, pts[:, 1].max() + margin)
            self.slam.set_bounds(bounds)
        self.slam.set_start_pose(start_pose)   # 既知スタート地点（実機でも置く位置は既知）
        self.reset_map()

        # cheat モードは CENTER_LINE から即経路を作る。
        # slam モードは「未知環境」想定なので、走って地図を作るまで経路は持たない
        # （探索後に build_course_from_map で経路を生成する）。
        if self.loc_mode == "cheat":
            try:
                course_map = self.analyzer.build_course_map(course)
            except ValueError:
                self.path = None
                return
            self._apply_course_map(course_map)

    def set_autonomous_target(self, speed: float | None) -> None:
        self.auto_target_speed = speed

    def _apply_course_map(self, course_map) -> None:
        """CourseMap を受け取り、Phase4 のレーシングライン＋速度プロファイルを
        生成して追従経路に採用する。最適化が無効なら中心線を追従する。"""
        if self.racing_enabled and len(course_map.center_line) >= 5:
            try:
                racing = self.racing_optimizer.optimize(course_map)
                course_map.racing_line = racing
                self.speed_profile = self.racing_optimizer.compute_speed_profile(racing)
                self.path = racing
            except Exception as exc:  # noqa: BLE001  最適化失敗時は中心線にフォールバック
                print(f"[controller] racing line optimize failed: {exc}")
                self.path = course_map.center_line
                self.speed_profile = None
        else:
            self.path = course_map.center_line
            self.speed_profile = None
        self.shared.update_course_map(course_map)

    # ---- Phase3: SLAM マッピング操作 -------------------------------------
    def reset_map(self) -> None:
        self.slam.reset()
        self.slam.localization_only = False
        self.trajectory = []
        self.has_slam_map = False
        self._map_tick = 0
        self._slam_loc_tick = 0
        self._last_loc = None
        self.shared.update_slam_map(None)  # type: ignore[arg-type]

    def save_map(self, name: str) -> str:
        """占有格子＋探索軌跡(1周)を saved_maps/<name>.npz に保存する。"""
        self.saved_maps_dir.mkdir(parents=True, exist_ok=True)
        path = str(self.saved_maps_dir / f"{name}.npz")
        m = self.slam.mapper
        loop = extract_loop(np.array(self.trajectory), self._start_xy) \
            if self.trajectory else np.empty((0, 2))
        np.savez_compressed(
            path, log_odds=m.log_odds, resolution=m.resolution,
            origin_x=m.origin_x, origin_y=m.origin_y, trajectory=loop,
        )
        return path

    def load_map(self, name: str) -> bool:
        """保存済み地図を読み込み、SLAM 占有格子・経路を再構築する。"""
        path = self.saved_maps_dir / (name if name.endswith(".npz") else f"{name}.npz")
        if not path.exists():
            return False
        data = np.load(str(path))
        m = self.slam.mapper
        m.log_odds = data["log_odds"].astype(np.float32)
        m.resolution = float(data["resolution"])
        m.origin_x = float(data["origin_x"])
        m.origin_y = float(data["origin_y"])
        m.height, m.width = m.log_odds.shape
        self.trajectory = [tuple(p) for p in data["trajectory"]] \
            if "trajectory" in data else []
        self.has_slam_map = True
        self.shared.update_slam_map(self.slam.get_map())
        self.build_course_from_map()
        return True

    def build_course_from_map(self, seed_path: np.ndarray | None = None) -> bool:
        """SLAM 占有格子＋探索軌跡から CourseMap を抽出し追従経路に採用する。"""
        grid = self.slam.get_map()
        if seed_path is None:
            if not self.trajectory:
                return False
            seed_path = extract_loop(np.array(self.trajectory), self._start_xy)
        try:
            course_map = self.analyzer.analyze(grid, seed_path=seed_path)
        except (ValueError, IndexError):
            return False
        # Phase4: SLAM 由来コースにもレーシングライン最適化を適用
        self._apply_course_map(course_map)
        return True

    def prepare_autonomous(self) -> None:
        """自律走行開始前に、SLAM 地図があれば SLAM 由来経路に切り替える。

        地図が無ければ Phase2 のカンニング経路（set_course 済み）をそのまま使う。
        """
        if self.has_slam_map and self.trajectory:
            self.build_course_from_map()

    # ---- 制御ループ -------------------------------------------------------
    def _loop(self) -> None:
        next_t = time.perf_counter()
        while self._running:
            now = time.perf_counter()
            if now < next_t:
                time.sleep(min(next_t - now, CONTROL_DT))
                continue
            next_t += CONTROL_DT

            try:
                self._tick(CONTROL_DT)
            except Exception as exc:  # noqa: BLE001  制御ループは止めない
                print(f"[controller] tick error: {exc}")

    def _tick(self, dt: float) -> None:
        # 一時停止中はステップを進めない（シミュ専用）
        paused = self.shared.is_paused()

        mode = self.shared.get_mode()
        cmd = self._resolve_command(mode)

        # 緊急停止が最優先
        if self.shared.is_emergency_stop():
            cmd = ControlCommand(0.0, 0.0, time.time())

        # ウォッチドッグ：テレオペ（手動操作）時のみ。最後の指令受信から
        # 500ms 経過で停止。自律走行は planner が内部で指令を生成するため対象外。
        is_teleop = mode in (DriveMode.MANUAL, DriveMode.MAP_BUILDING)
        if is_teleop and time.time() - self._last_cmd_at > self.command_timeout:
            cmd = ControlCommand(0.0, 0.0, time.time())

        self.backend.send_command(cmd)

        if self.is_sim:
            if not paused:
                mult = self.shared.get_speed_multiplier()
                self.backend.step(dt * mult)
        else:
            self.backend.step(dt)

        # 自己位置推定
        vehicle = self.backend.get_vehicle_state()
        if self.loc_mode == "slam":
            self._localize_slam(mode, paused, vehicle)
        else:
            # cheat: 真値を使用。MapBuilding 中のみ既知姿勢でマッピング。
            loc = self.localizer.update(vehicle)
            self.shared.update_localization(loc)
            if mode in (DriveMode.MAP_BUILDING, DriveMode.REACTIVE) and not paused:
                self._map_tick += 1
                if self._map_tick % self._map_rate_divider == 0:
                    grid = self.slam.update(self.backend.get_lidar_scan(), loc)
                    self.shared.update_slam_map(grid)
                    self.has_slam_map = True
                    self.trajectory.append((loc.x, loc.y))

        # 接続状態の更新
        self.shared.update_connection(self.backend.get_connection_status())

        # ログ記録
        if self.logger is not None and self.logger.is_recording():
            self.logger.record_frame(self.shared.get_system_state())

    def _localize_slam(self, mode: DriveMode, paused: bool, vehicle) -> None:
        """実機相当 SLAM：LiDAR のみで自己位置推定し、常時マッピングする。

        真値 vehicle は使わない（pygame 比較表示・誤差評価のためにのみ存在）。
        """
        if paused:
            return
        # 地図生成前は SLAM を動かさない＝Webに地図を出さない。
        # MapBuilding / Reactive（地図生成・自動探索）に入るか、既に地図がある時だけ動く。
        # （実機同様、地図が無い状態では何も表示しない／自己位置も出さない）
        if mode not in (DriveMode.MAP_BUILDING, DriveMode.REACTIVE) and not self.has_slam_map:
            return
        self._slam_loc_tick += 1
        if self._slam_loc_tick % self._slam_loc_divider != 0:
            return  # 25Hz 程度で実行。間は直前の推定を保持。

        scan = self.backend.get_lidar_scan()
        imu = self.backend.get_imu_reading()
        imu_heading = imu.heading if imu is not None else None
        loc = self.slam.process(scan, imu_heading=imu_heading)  # 自己位置推定＋地図更新
        self._last_loc = loc
        self.shared.update_localization(loc)

        # 地図の配信（低レート）
        self._map_tick += 1
        if self._map_tick % self._map_rate_divider == 0:
            self.shared.update_slam_map(self.slam.get_map())
            self.has_slam_map = True

        # 探索（手動 or リアクティブ）中は推定軌跡を記録 → コース構築の種にする
        if mode in (DriveMode.MANUAL, DriveMode.MAP_BUILDING, DriveMode.REACTIVE):
            self.trajectory.append((loc.x, loc.y))

    def _resolve_command(self, mode: DriveMode) -> ControlCommand:
        cmd = self.shared.get_command()
        if cmd is None:
            cmd = ControlCommand(0.0, 0.0, time.time())

        if mode in (DriveMode.MANUAL, DriveMode.MAP_BUILDING):
            # 手動操作の指令をそのまま使う
            return cmd

        if mode == DriveMode.REACTIVE:
            # LiDARのみのリアクティブ走行（地図/自己位置不要、自動探索）
            return self.reactive.compute_command(self.backend.get_lidar_scan())

        if mode == DriveMode.AUTONOMOUS:
            if not self.shared.get_autonomous_running():
                return ControlCommand(0.0, 0.0, time.time())
            if self.path is None or len(self.path) < 2:
                return ControlCommand(0.0, 0.0, time.time())
            # Pure Pursuit で経路追従。Phase4 では racing_line＋速度プロファイル。
            loc = self.shared.get_localization()
            cmd = self.planner.compute_command(
                loc, self.path, speed_cap=self.auto_target_speed,
                speed_profile=self.speed_profile)
            # 動的障害物回避：SLAM占有地図で「地図free上の点＝障害物」を検知して回避
            if self.avoid_enabled and self.has_slam_map:
                cmd, self.obstacle_info = self.avoider.adjust(
                    cmd, loc, self.backend.get_lidar_scan(), self.slam.mapper)
            return cmd

        return cmd
