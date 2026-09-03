"""1周ぶんの処理パイプライン — 脱スキュー→推測航法→(scan2scan)→scanmatch→地図更新。

`raspi/nav/slam.py`のオーケストレーション層を車体非依存の形で移植したもの。
**ループ閉じ（弧長比例配分）はここに無い。** キーフレーム列（`keyframes`）と
累積走行量を外部に公開するだけの責務にし、ループ検出・グラフ最適化は
`backend/`に任せる（単一責任の分割）。

## 姿勢の基準時刻は「点群の時刻」であって「今」ではない

`raspi/nav/slam.py`から引き継いだ設計判断: `update()`が呼ばれる時刻は、
点群の最後の点が測られた時刻より遅れる（センサの伝送遅延+受信処理）。
オドメトリを「今」まで進めてから「点群の時刻」の地図と照合すると、
毎周期その遅れぶんだけ後ろへ引き戻される誤差が蓄積する。なので姿勢は
常に`ScanPoints.t_ref_ns`時点のものとして持ち、推測航法を進める区間も
前回の`t_ref`から今回の`t_ref`までにする（呼び出し側から渡される`dt`は、
点群の時刻が取れないときの予備でしかない）。

## 見失ったら黙って走らない

マッチの得点が`min_score`を下回ったら`lost=True`を返し、地図を更新しない。
見失ったままにもしない——`reloc_after`周期続けて見失ったら、探索範囲を
大きく広げて（事前分布も切って）1回だけ探し直す。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import NamedTuple

import numpy as np

from .confidence import ConfidenceConfig, estimate_information, fuse_gaussians
from .deskew import deskew, truncate
from .grid import OccGrid
from .motion import MotionModel
from .scan2scan import IcpConfig, match_scans
from .scanmatch import MatcherConfig, Stage, match
from .types import Cov3, Pose2D, RawScan, ScanPoints, between, compose, integrate_twist, wrap_angle

__all__ = ["FrontendConfig", "FrontendUpdate", "Frontend", "RELOC_STAGES"]

NS = 1_000_000_000

#: 見失ったときに1回だけ使う広い探索。通常の探索（`MatcherConfig`の既定`stages`）
#: では、いったん大きくずれると二度と戻れない
RELOC_STAGES: tuple[Stage, ...] = (
    Stage(0.40, 0.10, math.radians(20.0), math.radians(5.0)),
    Stage(0.10, 0.03, math.radians(5.0), math.radians(1.0)),
    Stage(0.02, 0.005, math.radians(1.0), math.radians(0.2)),
)


@dataclass(frozen=True, slots=True)
class FrontendConfig:
    matcher: MatcherConfig = MatcherConfig()
    icp: IcpConfig = IcpConfig()
    #: **毎周期のライブブレンド（`fuse_gaussians`で`self.pose`を決める部分）専用**。
    #: `core/confidence.py`の既定（`analytic=True`、点対応ベースのFisher情報）
    #: をそのまま毎周期のブレンドに使うと、`info_pred`（運動モデルの固定・
    #: 小さい予測共分散）に対して`info_obs`が桁違いに大きくなり、毎周期
    #: ほぼ生の`obs_pose`（離散化された探索格子上の解）へ張り付くようになって
    #: 平滑化が効かなくなる——実測（oval 3周）でヨー誤差が0.85〜1.4°から
    #: 14〜65°へ悪化することを確認したため、**ここだけ明示的に旧・曲率ベース
    #: （`analytic=False`）に固定してある**。ループ拘束・キーフレームの
    #: エッジ重みには`keyframe_confidence`（既定`analytic=True`）を別途使う
    confidence: ConfidenceConfig = ConfidenceConfig(analytic=False)
    #: **キーフレームに添える情報行列（`backend/`のオドメトリ・ループ拘束の
    #: エッジ重みになる）専用**。既定（`analytic=True`）の点対応ベースFisher
    #: 情報を使う——ループ拘束が本来の到達精度どおりに信頼されるようにする
    #: のが目的で、こちらは毎周期のライブブレンドには使わない（上の`confidence`
    #: と役割を分けている理由はそちらのdocstring参照）
    keyframe_confidence: ConfidenceConfig = ConfidenceConfig()
    #: 前の周の点群と直接合わせて推測航法を磨くか（`core/scan2scan.py`）
    use_scan_to_scan: bool = False
    #: True なら`core/confidence.py`の固有値ベース動的異方性ブレンドを使う。
    #: False なら固定利得の等方的ブレンド（`match_gain`）にフォールバックする。
    #: **oval等での回転ドリフト対策として新規設計した部分であり効果は未検証**——
    #: 退行時にすぐ戻せるよう、切替可能にしてある
    anisotropic: bool = True
    #: 地図に焼く点の距離の上限[m]。照合には全点を使う
    map_range: float = 3.0
    #: 前回焼いた場所からこれだけ動いたら焼く
    kf_dist: float = 0.03
    kf_yaw: float = math.radians(2.0)
    #: マッチの得点がこれ未満なら見失ったとみなす
    min_score: float = 0.35
    #: これだけ連続で見失ったら探索範囲を広げて探し直す
    reloc_after: int = 15
    reloc_stages: tuple[Stage, ...] = RELOC_STAGES
    #: `anisotropic=False`のときのブレンド利得（0〜1）。地図凍結前後で分ける
    #: 理由は`raspi/nav/slam.py`と同じ——構築中に地図を汚す心配があるうちは
    #: 弱く、凍結後は強く寄せてよい
    match_gain: float = 0.1
    match_gain_frozen: float = 0.5
    #: `anisotropic=False`のときにキーフレームへ添える情報行列の大きさ。
    #: `backend/`のオドメトリエッジがこれを使う（`anisotropic=True`では
    #: `core/confidence.py`の実測値を使うのでこの値は参照されない）
    isotropic_keyframe_info: float = 100.0
    mount_x: float = 0.0
    mount_y: float = 0.0
    max_range: float = 12.0


class FrontendUpdate(NamedTuple):
    pose: Pose2D
    score: float                           #: マッチの得点 0〜1
    lost: bool                             #: 見失った（地図を更新していない）
    matched: bool                          #: 実際に探索したか


@dataclass
class Frontend:
    """1周ぶんの点群を取り込んで姿勢を更新する。ループ閉じは持たない。"""

    grid: OccGrid
    motion: MotionModel
    config: FrontendConfig = field(default_factory=FrontendConfig)

    def __post_init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.pose = Pose2D(0.0, 0.0, 0.0)
        #: 通った跡。ループ検出・グラフ構築は`backend/`がこれを見て行う
        self.trajectory: list[Pose2D] = []
        #: 焼いた点群と、そのときの観測情報行列。`backend/`がループ閉じ・
        #: グラフ最適化で使う（情報行列はオドメトリエッジの信頼度になる）
        self.keyframes: list[tuple[Pose2D, ScanPoints, Cov3]] = []
        self._kf: Pose2D | None = None
        self._prev_pts: ScanPoints | None = None
        self._t_ref = 0
        self.lost_streak = 0
        self.updates = 0
        #: 探し直した回数。増え続けるなら地図か点群を疑う
        self.relocs = 0

    def update(self, raw: RawScan, dt: float) -> FrontendUpdate:
        """1周ぶんの点群を取り込んで姿勢を更新する。

        :param dt: 前回の`update()`からの経過[s]。点群の時刻が取れないときの
            予備でしかない（上のモジュールdocstring参照）
        """
        twist = self.motion.current_twist()
        pts = deskew(raw, twist, mount_x=self.config.mount_x, mount_y=self.config.mount_y,
                    max_range=self.config.max_range)

        # **点群の時刻どうしの差**で進める。呼び出し間隔ではない
        span = dt
        if self._t_ref and pts.t_ref_ns:
            span = (pts.t_ref_ns - self._t_ref) / NS
            if not (0.0 < span < 1.0):     # 時刻が跳んだ。呼び出し間隔で代用する
                span = dt
        if pts.t_ref_ns:
            self._t_ref = pts.t_ref_ns
        span = max(1e-3, span)

        # 推測航法の1周期ぶん（前の周の車体座標での相対移動）
        delta_pred = integrate_twist(twist, span)
        cov_pred = self.motion.prediction_covariance(span)

        # ── scan2scan。観測できる方向だけ磨かれる ──
        if self.config.use_scan_to_scan and self._prev_pts is not None:
            d = match_scans(self._prev_pts, pts, delta_pred, config=self.config.icp)
            if d.ok:
                delta_pred = d.pose
        self._prev_pts = pts

        guess = compose(self.pose, delta_pred)

        # ── スキャンマッチ ──
        m = match(self.grid, pts, guess, config=self.config.matcher)

        # ── 見失ったままにしない ──
        if (self.lost_streak >= self.config.reloc_after
                and self.lost_streak % self.config.reloc_after == 0
                and self.grid.wall_mask().any()):
            wide_config = MatcherConfig(stages=self.config.reloc_stages, prior_w=0.0)
            wide = match(self.grid, pts, guess, config=wide_config)
            if wide.searched and wide.score > max(m.score, self.config.min_score):
                m = wide
                self.lost_streak = 0
                self.relocs += 1

        lost = m.searched and m.score < self.config.min_score
        # 照合できなかった（点が既知セルにほとんど落ちない）のも、地図が
        # 育ったあとなら見失ったのと同じ。地図が空の最初だけは通す
        if not m.searched and self.grid.wall_mask().any():
            lost = True

        info_obs: Cov3 = np.eye(3) * self.config.isotropic_keyframe_info
        #: キーフレームに添える（＝`backend/`のオドメトリ・ループ拘束のエッジ
        #: 重みになる）情報行列。`info_obs`（ライブブレンド専用、上の
        #: `FrontendConfig.confidence`docstring参照）とは別に、
        #: `keyframe_confidence`（既定`analytic=True`）で算出し直す
        keyframe_info: Cov3 = info_obs
        if lost:
            self.lost_streak += 1
            new_pose = guess
        else:
            self.lost_streak = 0
            obs_pose = Pose2D(m.x, m.y, m.yaw)
            if self.config.anisotropic:
                info_obs = estimate_information(self.grid, pts, obs_pose,
                                                 config=self.config.confidence)
                keyframe_info = estimate_information(self.grid, pts, obs_pose,
                                                     config=self.config.keyframe_confidence)
                info_pred = np.linalg.inv(cov_pred + np.eye(3) * 1e-12)
                new_pose = fuse_gaussians(guess, info_pred, obs_pose, info_obs)
            else:
                gain = (self.config.match_gain_frozen if self.grid.frozen
                       else self.config.match_gain)
                new_pose = _isotropic_blend(guess, obs_pose, gain)

        # motionの内部状態（速度推定・バイアス等）を、実際に採用した相対姿勢で更新
        measured_delta = between(self.pose, new_pose)
        self.motion.update(measured_delta, span)

        self.pose = new_pose

        # ── 地図更新。3つの条件を全部満たしたときだけ ──
        if not lost and self._moved_enough():
            small = truncate(pts, self.config.map_range)
            self.grid.integrate(small, self.pose)
            self.trajectory.append(self.pose)
            self._kf = self.pose
            self.keyframes.append((self.pose, small, keyframe_info))

        self.updates += 1
        return FrontendUpdate(self.pose, m.score, lost, m.searched)

    def load_map(self, grid: OccGrid) -> None:
        """事前に構築済みの地図（凍結済み想定）に差し替える。

        `reset()`と違い**姿勢・軌跡・キーフレームは動かさない**——呼び出し側が
        直後にグローバルローカリゼーション（`core/localize.py`）で`pose`を
        外部から設定する運用を想定している。捨てるのは**直近のスキャン間
        追跡状態だけ**（`_prev_pts`のscan-to-scan初期値・`_kf`の最後に焼いた
        場所・`lost_streak`）。古い地図との整合を前提にしたこれらの値を
        新しい地図に持ち越すと、初回の`update()`が誤った基準で動く。
        """
        self.grid = grid
        self._prev_pts = None
        self._kf = None
        self.lost_streak = 0

    def rebuild(self, *, grid: OccGrid, keyframes: list[tuple[Pose2D, ScanPoints, Cov3]],
               trajectory: list[Pose2D], pose: Pose2D) -> None:
        """ループ閉じ後、`backend/`が補正した地図・キーフレーム・軌跡・姿勢で置き換える。

        `pipeline.SlamSystem`がポーズグラフ最適化の結果を反映するために呼ぶ。
        `Frontend`自身はグラフ最適化のロジックを持たない（単一責任の分割）。
        """
        self.grid = grid
        self.keyframes = keyframes
        self.trajectory = trajectory
        self.pose = pose
        self._kf = pose

    def _moved_enough(self) -> bool:
        """前回焼いた場所から十分動いたか。止まっている間は焼かない。"""
        if self._kf is None:
            return True
        dx = self.pose.x - self._kf.x
        dy = self.pose.y - self._kf.y
        if math.hypot(dx, dy) >= self.config.kf_dist:
            return True
        return abs(wrap_angle(self.pose.yaw - self._kf.yaw)) >= self.config.kf_yaw


def _isotropic_blend(pred: Pose2D, obs: Pose2D, gain: float) -> Pose2D:
    """予測`pred`をマッチ結果`obs`へ等方的に`gain`だけ寄せる（`anisotropic=False`用）。

    `raspi/nav/slam.py`の`match_gain`（進行方向・横方向を区別しない旧来の
    ブレンド）と同じ考え方。固有値ベースの異方性ブレンドが効かない・退行した
    場合の比較基準として残してある。
    """
    return Pose2D(
        pred.x + gain * (obs.x - pred.x),
        pred.y + gain * (obs.y - pred.y),
        pred.yaw + gain * wrap_angle(obs.yaw - pred.yaw),
    )
