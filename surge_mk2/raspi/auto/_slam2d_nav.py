"""`slam2d.core.Frontend`(車体非依存の汎用SLAM)を`raspi/auto/`のplannerから使う
ための薄いブリッジ。

`slam2d/`は`raspi.*`を一切importしない独立ライブラリとして設計してある
（`/Users/banbatakumi/.claude/plans/wiggly-doodling-moth.md`参照）。逆方向
（`raspi/`側が`slam2d`を使う）はこのファイルに閉じ込め、以下の車体固有の
変換・付加機能だけをここに置く:

- `Scan`（raspi固有のLD06セクタ形式）→`RawScan`（slam2d、点ごとの時刻を持つ
  汎用形式）の変換
- `VehicleState`（ジャイロ+速度）→`Twist2D`の橋渡し
- `raspi/nav/slam.py`の`Slam`が持っていた`lap_progress()`（累積回頭÷360°）
  のような、slam2d本体には無い車体固有の付加機能

## ループ閉じ（`slam2d/backend/`・`slam2d/pipeline.SlamSystem`）を使う

以前は「単一〜少数のループ拘束では`Frontend`単体より精度が悪化することが
ある」（`slam2d/backend/loop_detection.py`旧docstring「既知の限界」）という
短距離ovalシムでの実測に基づき、`Frontend`単体のみを使いbackendを一切
importしていなかった（Pi実機側の依存を増やさないための判断でもあった）。

長距離ドリフト対策（`~/.claude/plans/slam2d-slam-slam2d-imu-slam-slam2d-imu-nifty-aurora.md`）
で`core/confidence.py`の情報行列推定を点対応ベースのFisher情報に作り直した
結果、oval基準ベンチマークでループ閉じが初めてFrontend単体を上回るように
なった（詳細は`loop_detection.py`モジュールdocstring「追記」参照）ため、
方針を転換して有効化する。`g2opy`はPi 5(aarch64)向けビルド済みwheelがあり、
実機（Python 3.13.5）で動作確認済み（`slam2d/tools/spike_g2o.py`、
`raspi/requirements.txt`参照）。

**ループクロージャの最適化・地図再構築は、EXPLORE→BUILD遷移の瞬間にのみ
行う**（`SlamSystem.flush()`）。`raspi/auto/slam2d_raceline.py`の`_explore()`
はこの瞬間`ready=False`で車両を止める設計になっており、数百ms〜1秒の一時
停止コストを安全に払える。RACE段は`Frontend`を直接叩き、追跡専用に保つ
（走行中に最適化・地図再構築の重い処理を挟まない）。

`ENABLE_LOOP_CLOSURE`はキルスイッチ——現場で問題が出た場合に1行で
`Frontend`単体運用へ戻せる。
"""

from __future__ import annotations

import math

import numpy as np

from slam2d.backend.loop_detection import LoopDetectorConfig
from slam2d.core.frontend import Frontend, FrontendConfig, FrontendUpdate
from slam2d.core.grid import OccGrid
from slam2d.core.motion import ExternalTwistModel, GyroBiasEstimator
from slam2d.core.types import Pose2D, RawScan, Twist2D, wrap_angle
from slam2d.pipeline import PipelineConfig, SlamSystem

from ..msgs.types import Scan, VehicleState
from ..nav.deskew import point_times_ns

__all__ = ["Slam2dNav", "occgrid_from_trinary", "scan_to_raw"]

#: キルスイッチ。Falseにすると`Frontend`単体運用（ループ閉じ無し）に戻る
ENABLE_LOOP_CLOSURE = True


def occgrid_from_trinary(trinary: np.ndarray, *, resolution: float,
                         origin: tuple[float, float], seq: int,
                         min_hits: int = 3, min_seen: int = 3) -> OccGrid:
    """保存済みの3値地図（`raspi/auto/mapstore.py`）から、走行時に読まれる判定
    （`wall_mask`/`known_free_mask`/`score_map`/`raycast`）を完全に再現する
    凍結済み`OccGrid`を作る。

    **生のhits/missesは要らない。** 凍結後の地図はこの4関数からしか読まれず、
    どれも3値の判定結果だけの関数だから（`raspi/auto/mapstore.py`のモジュール
    docstring参照）。`hits`/`misses`は「判定結果がそうなる」最小の値
    （占有は`hits=min_hits, misses=0`、空きは`hits=0, misses=min_seen`）で埋める。
    """
    h, w = trinary.shape
    if h != w:
        raise ValueError(f"trinary must be square (got {h}x{w}); OccGrid only supports square grids")
    grid = OccGrid(resolution=resolution, size_m=w * resolution, origin=origin,
                   min_hits=min_hits, min_seen=min_seen)
    if grid.width != w or grid.height != h:
        raise ValueError(f"reconstructed grid size mismatch: {grid.width}x{grid.height} != {w}x{h}")

    occ = trinary == 2
    free = trinary == 1
    grid.hits[occ] = min_hits
    grid.hits[free] = 0
    grid.misses[occ] = 0
    grid.misses[free] = min_seen
    grid.frozen = True
    grid.seq = seq
    return grid


def scan_to_raw(scan: Scan) -> RawScan:
    """`Scan`（raspi固有のLD06セクタ形式）を`RawScan`（slam2dの汎用形式）へ変換する。

    有効判定（`sector_seen`かつ`dist>0`）・飽和判定は`raspi/nav/deskew.py`の
    `deskew()`と同じ規約に合わせてある——ここだけ違う読み方をすると、この
    plannerだけ「空きとして彫ってはいけない場所」を彫ってしまう事故になる
    （`raspi/nav/deskew.py`冒頭docstringの「3種類の点を区別する」参照）。
    """
    deg = np.arange(360)
    dist = np.asarray(scan.dist, dtype=np.float64)
    seen = np.asarray(scan.sector_seen, dtype=bool)
    sector = ((deg - 1) % 360) // 30
    valid = seen[sector] & (dist > 0.0)
    saturated = (np.asarray(scan.saturated, dtype=bool) if scan.saturated is not None
                else np.zeros(360, dtype=bool))
    angles = np.radians(deg.astype(np.float64))
    t_point = point_times_ns(scan)
    return RawScan(angles, dist, valid, saturated, t_point)


class Slam2dNav:
    """`raspi/nav/slam.py`の`Slam`と部分的に互換なインターフェースで
    `slam2d.core.Frontend`を包む。

    `raspi/auto/raceline.py`が期待する最小限の面（`update`/`pose`/`grid`/
    `trajectory`/`trajectory_array`/`lap_progress`/`distance`/`speed`/
    `yaw_rate`/`freeze`/`reset`）だけを実装する。`close_loop()`は実装しない
    （上のモジュールdocstring参照。呼び出し側でループ閉じ相当の処理を省く）。
    """

    def __init__(self, *, resolution: float, size_m: float,
                lidar_x: float = 0.0, lidar_y: float = 0.0,
                max_range: float = 12.0) -> None:
        self._resolution = resolution
        self._size_m = size_m
        self._lidar_x = lidar_x
        self._lidar_y = lidar_y
        self._max_range = max_range
        self._raw_yaw_rate = 0.0
        self._raw_speed = 0.0
        self.reset()

    def reset(self) -> None:
        grid = OccGrid(resolution=self._resolution, size_m=self._size_m,
                       min_hits=3, min_seen=3)
        grid.seq = self.next_seq()
        bias = GyroBiasEstimator()
        motion = ExternalTwistModel(self._current_twist, bias_estimator=bias)
        config = FrontendConfig(mount_x=self._lidar_x, mount_y=self._lidar_y,
                                max_range=self._max_range)
        if ENABLE_LOOP_CLOSURE:
            self._system = SlamSystem(grid, motion, PipelineConfig(
                frontend=config, loop_detector=LoopDetectorConfig(),
                # ★ 既定(10)のままだと、長いコースのEXPLORE走行中に自動で
                # バッチ最適化・地図再構築が発火しうる（`sim.bench --course
                # circuit`の実測でplan()が最大1.2秒スパイクすることを確認）。
                # `planning_node.py`はplan()を10Hzで呼ぶ設計なので、走行中の
                # 秒単位の停止は許容できない。最適化・地図再構築は`freeze()`
                # （EXPLORE→BUILD遷移、車両停止済み）での明示的な`flush()`
                # だけに限定するため、自動発火の閾値を実質無効な大きさにする
                loop_batch_size=1_000_000))
            self._fe = self._system.frontend
        else:
            self._system = None
            self._fe = Frontend(grid, motion, config)
        #: EXPLORE/BUILD中はTrue（`SlamSystem`経由でループ検出する）。
        #: RACE中はFalse（`Frontend`直結、追跡専用）に切り替える
        self._backend_active = ENABLE_LOOP_CLOSURE
        self.heading_total = 0.0

    @property
    def loop_closures(self) -> int:
        """検出・反映されたループ拘束の本数（GUI表示・診断用）。"""
        return self._system.loop_closures if self._system is not None else 0

    def next_seq(self) -> int:
        """次に使う版番号。**直前の地図の`seq`より必ず大きい値を返す**
        （`raspi/nav/slam.py`の`_new_grid()`と同じパターン）。

        `OccGrid.integrate()`は取り込むたびに`seq`を進めるので、resetの
        たびに0へ戻すと**育った地図より小さい版番号**になる。GUI側の
        「古い版で新しい版を上書きしない」ガード（`gui/src/ws/map.ts`）に
        新しい（空の）地図が弾かれ、削除前の地図が画面に残り続ける原因になる。
        保存済み地図の読み込み時（`occgrid_from_trinary`）にも同じ理由で使う。
        """
        prev = getattr(self, "_fe", None)
        return (prev.grid.seq + 1) if prev is not None else 0

    def _current_twist(self) -> Twist2D:
        return Twist2D(self._raw_speed, 0.0, self._raw_yaw_rate)

    def update(self, scan: Scan, dt: float, *, yaw_rate: float | None,
               speed: float | None) -> FrontendUpdate:
        """1周ぶんの点群を取り込んで姿勢を更新する。

        `raspi/nav/slam.py`の`Slam.update()`と同じ引数（`yaw_rate`/`speed`が
        `None`なら`ExternalTwistModel`に0を渡す——`slam2d`側に等速度モデルへの
        自動フォールバックは無いため、`raspi/auto/raceline.py`の`lidar_only`
        オプションはこのブリッジでは未対応）。
        """
        self._raw_yaw_rate = 0.0 if yaw_rate is None else float(yaw_rate)
        self._raw_speed = 0.0 if speed is None else float(speed)
        raw = scan_to_raw(scan)
        prev_yaw = self._fe.pose.yaw
        if self._backend_active and self._system is not None:
            u = self._system.update(raw, dt)
        else:
            u = self._fe.update(raw, dt)
        self.heading_total += wrap_angle(u.pose.yaw - prev_yaw)
        return u

    @property
    def pose(self) -> Pose2D:
        return self._fe.pose

    @property
    def grid(self) -> OccGrid:
        return self._fe.grid

    @property
    def trajectory(self) -> list[Pose2D]:
        return self._fe.trajectory

    def trajectory_array(self) -> np.ndarray:
        """軌跡を`(N, 3)`の配列で。空なら`(0, 3)`（`raspi/nav/slam.py`と同じ約束）。"""
        if not self._fe.trajectory:
            return np.zeros((0, 3), dtype=np.float64)
        return np.asarray(self._fe.trajectory, dtype=np.float64)

    @property
    def distance(self) -> float:
        """累積走行距離[m]（軌跡の点間距離の総和で近似）。"""
        traj = self._fe.trajectory
        if len(traj) < 2:
            return 0.0
        arr = np.asarray(traj, dtype=np.float64)
        d = np.diff(arr[:, :2], axis=0)
        return float(np.hypot(d[:, 0], d[:, 1]).sum())

    def lap_progress(self) -> float:
        """周回の進み具合[周]。累積回頭を360°で割ったもの（`Slam`と同じ定義）。"""
        return abs(self.heading_total) / (2.0 * math.pi)

    @property
    def speed(self) -> float:
        return self._fe.motion.current_twist().vx

    @property
    def yaw_rate(self) -> float:
        return self._fe.motion.current_twist().yaw_rate

    @property
    def max_range(self) -> float:
        return self._max_range

    @property
    def lidar_x(self) -> float:
        return self._lidar_x

    @property
    def lidar_y(self) -> float:
        return self._lidar_y

    def freeze(self) -> None:
        """EXPLORE→BUILD遷移で地図構築を終える。

        保留中のループ拘束があれば`flush()`で一括反映してから凍結する
        （モジュールdocstring参照。`slam2d_raceline.py`の`_explore()`はこの
        瞬間`ready=False`で車両を止めているので、最適化・地図再構築の一時的な
        処理コストを安全に払える）。以後RACEまでは`Frontend`直結の追跡専用に
        切り替える——凍結後の地図にループ拘束を追加で足す理由が無いため
        """
        if self._backend_active and self._system is not None:
            self._system.flush()
        self._fe.grid.freeze()
        self._backend_active = False

    def replace_grid_for_race(self, grid: OccGrid) -> None:
        """保存済み地図をRACEの土台として差し込む（`Frontend.load_map()`参照）。"""
        self._fe.load_map(grid)
        self._backend_active = False

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        """グローバルローカリゼーション（`slam2d.core.localize.GlobalLocalizer`）
        が決めた姿勢を採用する。以後は通常の追跡型スキャンマッチに引き継ぐ。
        """
        self._fe.pose = Pose2D(x, y, yaw)

    def deskew_scan(self, scan: Scan):
        """`scan`を現在保持しているtwistで脱スキューする（障害物検出用）。

        `raspi/auto/raceline.py`の`_update_obstacles()`が使う。`Frontend.update()`
        の内部でも同じ脱スキューを1回行っているが、そちらは非公開の中間状態
        なのでここで独立に呼び直す（`raspi/nav/slam.py`の`Slam`が`speed`/
        `yaw_rate`を露出していたのと同じ立て付け）。
        """
        from slam2d.core.deskew import deskew as slam2d_deskew

        raw = scan_to_raw(scan)
        return slam2d_deskew(raw, self._current_twist(),
                             mount_x=self._lidar_x, mount_y=self._lidar_y,
                             max_range=self._max_range)
