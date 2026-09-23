"""1周ぶんの処理 — 脱スキュー → 推測航法の予測 → 距離場への位置合わせ → 地図更新。

**ループ閉じはここに無い。** キーフレーム列（`keyframes`）を外部に公開し、
ループ検出・ポーズグラフ最適化は`backend/`・`pipeline.SlamSystem`に任せる。
最適化の結果は`apply_correction()`で受け取り、地図と局所地図を作り直す。

## 2つの動作モード

| | 地図作成中（`grid.frozen == False`） | 凍結地図での走行（`True`） |
|---|---|---|
| 合わせる相手 | **直近のキーフレームの点群**（`core/localmap.py`） | 凍結した地図全体の距離場 |
| 地図の更新 | キーフレームごとに`grid`へ焼く | しない |
| ドリフト | ゆっくり溜まる（ループ閉じで直す） | 地図に対して溜まらない |

地図作成中に全体の地図へ合わせない理由は`core/localmap.py`のdocstring参照
（周回して戻ったとき古い壁と新しい壁が混ざる・走り出した直後は壁が立たない）。

## 位置合わせ（`core/register.py`）

推測航法の予測を事前分布にした Gauss-Newton。観測できる方向は点群が決め、
観測できない方向（通路の進行方向）は推測航法が残る。**旧実装の相関探索＋
情報フィルタのブレンドは、推測航法の誤差をほとんど正せていなかった**
（`sim/slam_bench.py`: 車速の倍率誤差2%だけで ATE が6cm、MPU6050相当の誤差で
複雑コースは見失ったまま戻らない）。

## 見失いの扱い

- 位置合わせの**当たり率**（照合できた点のうち壁から5cm以内の割合）が
  `min_inlier`を切るか、照合できた点が`min_used`未満なら、その周期は
  「合っていない」とみなして推測航法の予測を採る（地図は焼かない）
- `reloc_after`周期続いたら、予測の周りを総当たり（`core/register.search`）で
  探し直す
- 地図作成中に`restart_after`周期続いたら、**局所地図を今の点群から作り直す**
  （ずれを受け入れて走り続ける。旧実装は「見失い中は地図を更新しない」
  ために一度見失うと永久に戻れなかった）。途切れた区間のキーフレーム間の
  拘束は推測航法だけの弱い重みになり、ループ閉じで直る余地が残る

## 姿勢の基準時刻は「点群の時刻」であって「今」ではない

`update()`が呼ばれる時刻は、点群の最後の点が測られた時刻より遅れる。姿勢は
常に`ScanPoints.t_ref_ns`時点のものとして持ち、推測航法を進める区間も
前回の`t_ref`から今回の`t_ref`までにする（呼び出し側から渡される`dt`は、
点群の時刻が取れないときの予備でしかない）。**最初の周は予測しない**
（原点＝最初の点群の時刻の姿勢）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import NamedTuple

import numpy as np

from .deskew import TwistBuffer, deskew, deskew_traj, truncate
from .surfmap import SurfaceMap
from .grid import OccGrid
from .localmap import LocalMap, LocalMapConfig
from .motion import MotionModel
from .register import RegisterConfig, RegisterResult, register, search
from .types import (Cov3, Pose2D, RawScan, ScanPoints, Twist2D, between, compose,
                    integrate_twist, wrap_angle)

__all__ = ["FrontendConfig", "FrontendUpdate", "Frontend", "Keyframe", "rotate_info",
           "inflate_info"]

NS = 1_000_000_000


@dataclass(frozen=True, slots=True)
class FrontendConfig:
    register: RegisterConfig = RegisterConfig()
    local_map: LocalMapConfig = LocalMapConfig()
    mount_x: float = 0.0
    mount_y: float = 0.0
    max_range: float = 12.0
    #: 位置合わせに使う点の距離の上限[m]
    match_range: float = 8.0
    #: 地図に焼く点の距離の上限[m]。遠い点は測距誤差が大きく、壁が太る
    map_range: float = 3.0
    #: 前回のキーフレームからこれだけ動いたら新しいキーフレームにする
    kf_dist: float = 0.10
    kf_yaw: float = math.radians(5.0)
    #: 「合っている」の判定。当たり率だけでなく**残差の大きさ**も見る——
    #: 当たり率が半分くらいでも残差が大きいまま採用すると、誤った姿勢に
    #: 貼り付いたまま「合っている」と言い続ける（実測: course3 のヘアピンで
    #: 当たり率0.5・残差15cmのまま推定がその場に固定され、5m以上ずれた）。
    #:
    #: ★ 残差の判定は**絶対値ではなく、直近の残差に対する比**が主。地図が
    #: 歪んでいる・測距ノイズが大きい環境では「正常でも残差が大きい」のが
    #: 普通で、絶対値で切ると延々と見失い扱いになって推測航法だけで走ることに
    #: なる（実測: harsh 条件で絶対8cmの閾値を入れたら見失いが2.9%→87%に増え、
    #: 自己位置の誤差が5m→11mへ悪化した）
    min_used: int = 30
    min_inlier: float = 0.4
    max_rms: float = 0.15
    #: 直近の残差（移動平均）の何倍までを「合っている」とみなすか
    rms_factor: float = 3.0
    #: 残差の移動平均の追従の速さ（1周期あたり）
    rms_alpha: float = 0.05
    #: 見失いからの復帰
    reloc_after: int = 2
    reloc_trans: float = 0.4
    reloc_rot: float = math.radians(15.0)
    reloc_trans_step: float = 0.04
    reloc_rot_step: float = math.radians(1.5)
    #: 探し直しの結果を採る条件: 離れた候補の得点がこの割合未満であること
    reloc_ambiguity: float = 0.85
    #: 地図作成中にこれだけ続けて合わなければ局所地図を作り直す
    restart_after: int = 15
    #: 凍結地図で対応を付ける距離の上限[m]
    frozen_max_dist: float = 0.5
    #: キーフレーム間の拘束に足す「モデル化していない誤差」（`inflate_info`）
    edge_sigma_xy: float = 0.02
    edge_sigma_yaw: float = math.radians(0.3)


class FrontendUpdate(NamedTuple):
    pose: Pose2D
    score: float                           #: 位置合わせの当たり率 0〜1
    lost: bool                             #: 合っていない（予測を採った）
    matched: bool                          #: 位置合わせを試みたか（地図が空なら False）


class Keyframe(NamedTuple):
    pose: Pose2D
    #: 地図に焼く点（`map_range`で切り詰め済み、車体座標）
    points: ScanPoints
    #: 位置合わせ・ループ閉じに使う壁の点（`match_range`以内、車体座標）
    hx: np.ndarray
    hy: np.ndarray
    #: 直前のキーフレームとの相対姿勢の情報行列（**直前のキーフレームの座標系**）。
    #: 最初のキーフレームはゼロ行列
    info: Cov3
    #: 累積走行距離 [m]
    path: float
    stamp_ns: int


def rotate_info(info_world: Cov3, yaw: float) -> Cov3:
    """世界座標の情報行列を、向き`yaw`の座標系の情報行列へ直す。"""
    c, s = math.cos(yaw), math.sin(yaw)
    t = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return t.T @ info_world @ t


def inflate_info(info: Cov3, *, sigma_xy: float, sigma_yaw: float) -> Cov3:
    """情報行列に「モデル化していない誤差」を足して過信を落とす。

    点群の位置合わせのヘッセ行列は「その点群が地図に対してどれだけ鋭く
    決まるか」しか表さない。実際には**地図そのものの誤差**（焼くときの姿勢の
    ずれ・格子の量子化・測距の系統誤差で1〜2cm）が乗るので、そのぶんを共分散に
    足してから情報行列へ戻す。

    これを省くと、ポーズグラフのエッジが桁違いに過信になり、
    **ロバストカーネル（DCS）が正しいループ拘束まで外れ値として無効化する**
    （実測: toyota でループ拘束78本を入れても最適化後の姿勢が0.1cmしか動かなかった）。
    """
    cov = np.linalg.pinv(np.asarray(info, dtype=np.float64))
    cov = cov + np.diag([sigma_xy ** 2, sigma_xy ** 2, sigma_yaw ** 2])
    return np.linalg.inv(cov)


def _world_cov(cov_body: Cov3, yaw: float) -> Cov3:
    c, s = math.cos(yaw), math.sin(yaw)
    t = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return t @ cov_body @ t.T


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
        #: キーフレームの姿勢の列（`keyframes[i].pose`と同じ）
        self.trajectory: list[Pose2D] = []
        self.keyframes: list[Keyframe] = []
        self.local = LocalMap(self.config.local_map)
        #: 時刻つきの生の twist（ジャイロ+車速）。あれば点ごとの脱スキューと
        #: 推測航法の予測の両方をこの履歴から作る（`core/deskew.py`）
        self.twists = TwistBuffer()
        self._t_ref = 0
        self._path = 0.0
        #: 直前のキーフレームから積んだ推測航法の共分散（世界座標）
        self._cov_since_kf = np.zeros((3, 3))
        #: 直前のキーフレーム以降に「合っていない」周期があったか
        self._gap_since_kf = False
        self._frozen_field: SurfaceMap | None = None
        self._frozen_seq = -1
        self.lost_streak = 0
        self.updates = 0
        #: 探し直しで復帰した回数。増え続けるなら地図か点群を疑う
        self.relocs = 0
        #: 局所地図を作り直した回数（地図作成中に見失い続けた回数）
        self.restarts = 0
        #: 直近の位置合わせの診断
        self.last: RegisterResult | None = None
        #: 受け入れた位置合わせの残差の移動平均[m]（見失い判定の基準線）
        self.rms_ref = self.config.register.sigma

    # ── 1周期 ──

    def add_twist(self, t_ns: int, twist: Twist2D) -> None:
        """ジャイロ+車速のサンプルを**届くたびに**入れる（実機は50Hz）。

        入れておくと`update()`が点ごとの脱スキュー（`deskew_traj`）と、
        サンプル列を積んだ推測航法の予測を使う。入れなければ従来どおり
        「1周期のあいだ twist は一定」の近似になる。
        """
        self.twists.add(t_ns, twist)

    def update(self, raw: RawScan, dt: float) -> FrontendUpdate:
        """1周ぶんの点群を取り込んで姿勢を更新する。

        :param dt: 前回の`update()`からの経過[s]。点群の時刻が取れないときの予備
        """
        cfg = self.config
        twist = self.motion.current_twist()
        buf = self._corrected_twists()
        t_lo = int(raw.t_point_ns.min()) if raw.t_point_ns.size else 0
        t_hi = int(raw.t_point_ns.max()) if raw.t_point_ns.size else 0
        use_traj = (buf is not None and t_lo > 0
                    and buf.covers(min(t_lo, self._t_ref or t_lo), t_hi))
        if use_traj:
            pts = deskew_traj(raw, buf, mount_x=cfg.mount_x, mount_y=cfg.mount_y,
                              max_range=cfg.max_range)
        else:
            pts = deskew(raw, twist, mount_x=cfg.mount_x, mount_y=cfg.mount_y,
                         max_range=cfg.max_range)

        first = self.updates == 0
        span = dt
        if self._t_ref and pts.t_ref_ns:
            span = (pts.t_ref_ns - self._t_ref) / NS
            if not (0.0 < span < 1.0):     # 時刻が跳んだ。呼び出し間隔で代用する
                span = dt
        prev_t_ref = self._t_ref
        if pts.t_ref_ns:
            self._t_ref = pts.t_ref_ns
        span = max(1e-3, span)

        raw_delta: Pose2D | None = None
        if first:
            delta_pred = Pose2D(0.0, 0.0, 0.0)
            cov_body = np.diag([1e-6, 1e-6, 1e-8])
        else:
            if use_traj and prev_t_ref > 0:
                delta_pred = buf.delta(prev_t_ref, pts.t_ref_ns)
                raw_delta = self.twists.delta(prev_t_ref, pts.t_ref_ns)
            else:
                delta_pred = integrate_twist(twist, span)
            cov_body = self.motion.prediction_covariance(span)
        pred = compose(self.pose, delta_pred)
        cov_world = _world_cov(cov_body, self.pose.yaw)
        info_pred = np.linalg.inv(cov_world + np.eye(3) * 1e-12)

        d = np.hypot(pts.x, pts.y)
        sel = pts.hit & (d <= cfg.match_range)
        hx, hy = pts.x[sel], pts.y[sel]

        field_ = self._field(pred)
        matched = field_ is not None and not field_.empty and hx.size > 0
        lost = False
        score = 0.0
        new_pose = pred
        res: RegisterResult | None = None
        if matched:
            res = register(field_, hx, hy, pred, prior=pred, prior_info=info_pred,
                           config=cfg.register)
            good = self._good(res)
            if not good and self.lost_streak + 1 >= cfg.reloc_after:
                alt = self._relocalize(field_, hx, hy, pred)
                if alt is not None and self._good(alt):
                    res, good = alt, True
                    self.relocs += 1
            score = res.inlier
            if good:
                new_pose = res.pose
                a = cfg.rms_alpha
                self.rms_ref += a * (min(res.rms, cfg.max_rms) - self.rms_ref)
            else:
                lost = True
        self.last = res

        if not lost and res is not None and not first:
            info_body = rotate_info(res.info / max(cfg.register.info_scale, 1e-12), new_pose.yaw)
            self.motion.update(between(self.pose, new_pose), span, info_body, raw_delta,
                               absolute=self.grid.frozen)
        elif not lost and not first:
            self.motion.update(between(self.pose, new_pose), span, None, raw_delta,
                               absolute=self.grid.frozen)

        self._path += math.hypot(new_pose.x - self.pose.x, new_pose.y - self.pose.y)
        self.pose = new_pose
        self._cov_since_kf = self._cov_since_kf + cov_world
        restarted = False
        if lost:
            self.lost_streak += 1
            self._gap_since_kf = True
            if (not self.grid.frozen and self.lost_streak >= cfg.restart_after):
                # 見失ったまま走り続けない。今の点群から局所地図を作り直す
                self.local.clear()
                self.restarts += 1
                self.lost_streak = 0
                lost = False                # 下で新しいキーフレームとして採る
                restarted = True
        else:
            self.lost_streak = 0

        if not self.grid.frozen and not lost and (restarted or self._moved_enough()):
            self._add_keyframe(pts, hx, hy, res)

        self.updates += 1
        return FrontendUpdate(self.pose, score, lost, matched)

    def _corrected_twists(self) -> TwistBuffer | None:
        """生のサンプルに、学習済みのバイアス・倍率を適用した履歴を作る。"""
        if len(self.twists) < 2:
            return None
        correct = getattr(self.motion, "correct", None)
        out = TwistBuffer()
        for t, vx, vy, w in zip(self.twists.t, self.twists.vx, self.twists.vy, self.twists.w):
            tw = Twist2D(vx, vy, w)
            out.add(t, correct(tw) if correct is not None else tw)
        return out

    def _good(self, r: RegisterResult) -> bool:
        cfg = self.config
        limit = max(cfg.max_rms, self.rms_ref * cfg.rms_factor)
        return (r.used >= cfg.min_used and r.inlier >= cfg.min_inlier
                and r.rms <= limit)

    def _relocalize(self, field_: SurfaceMap, hx, hy, center: Pose2D) -> RegisterResult | None:
        cfg = self.config
        sr = search(field_, hx, hy, center, trans=cfg.reloc_trans, rot=cfg.reloc_rot,
                    trans_step=cfg.reloc_trans_step, rot_step=cfg.reloc_rot_step)
        if sr.score <= 0.0:
            return None
        # ★ 離れた候補が肉薄しているなら飛び移らない。似た形が並ぶコース
        #   （平行な車線・繰り返しの通路）で、見失った拍子に隣の通路へ
        #   移ってしまうのを防ぐ。曖昧なときは推測航法のまま進む方が安全
        if sr.second >= cfg.reloc_ambiguity * sr.score:
            return None
        return register(field_, hx, hy, sr.pose, config=cfg.register)

    def _field(self, around: Pose2D) -> SurfaceMap | None:
        if self.grid.frozen:
            if self._frozen_field is None or self._frozen_seq != self.grid.seq:
                g = self.grid
                mean = g.hit_mean()
                self._frozen_field = SurfaceMap(
                    g.wall_mask(), resolution=g.resolution, origin=g.origin,
                    mean_x=None if mean is None else mean[0],
                    mean_y=None if mean is None else mean[1],
                    known=g.seen > 0, max_dist=self.config.frozen_max_dist)
                self._frozen_seq = g.seq
            return self._frozen_field
        return self.local.field(around)

    def _moved_enough(self) -> bool:
        if not self.keyframes:
            return True
        kf = self.keyframes[-1].pose
        if math.hypot(self.pose.x - kf.x, self.pose.y - kf.y) >= self.config.kf_dist:
            return True
        return abs(wrap_angle(self.pose.yaw - kf.yaw)) >= self.config.kf_yaw

    def _add_keyframe(self, pts: ScanPoints, hx, hy, res: RegisterResult | None) -> None:
        cfg = self.config
        if self.keyframes:
            prev = self.keyframes[-1].pose
            # 観測できない方向の拘束は推測航法（キーフレーム間で積んだ共分散）、
            # 観測できる方向は位置合わせの情報行列が決める
            info_w = np.linalg.inv(self._cov_since_kf + np.eye(3) * 1e-12)
            if res is not None and not self._gap_since_kf:
                info_w = info_w + res.info
            info_w = inflate_info(info_w, sigma_xy=cfg.edge_sigma_xy,
                                  sigma_yaw=cfg.edge_sigma_yaw)
            info = rotate_info(info_w, prev.yaw)
        else:
            info = np.zeros((3, 3))
        small = truncate(pts, cfg.map_range)
        self.grid.integrate(small, self.pose)
        self.local.add(self.pose, hx, hy)
        self.keyframes.append(Keyframe(self.pose, small, hx.astype(np.float32),
                                       hy.astype(np.float32), info, self._path, self._t_ref))
        self.trajectory.append(self.pose)
        self._cov_since_kf = np.zeros((3, 3))
        self._gap_since_kf = False

    # ── 外からの差し替え ──

    def apply_correction(self, poses: list[Pose2D]) -> None:
        """ポーズグラフ最適化の結果（キーフレームごとの姿勢）を反映する。

        地図を**新しい版の格子に焼き直し**（古い版に上書きすると、同じコースを
        角度違いで重ね描きした形に崩れる）、局所地図を作り直し、今の姿勢を
        最後のキーフレームとの相対関係を保ったまま移す。
        """
        if len(poses) != len(self.keyframes) or not poses:
            raise ValueError("poses must match keyframes")
        old_last = self.keyframes[-1].pose
        rel = between(old_last, self.pose)
        g = self.grid
        new = OccGrid(resolution=g.resolution, size_m=1.0, origin=g.origin,
                      min_hits=g.min_hits, min_seen=g.min_seen, grow=g.grow,
                      grow_step_m=g.grow_step_m, max_size_m=g.max_size_m)
        new.hits = np.zeros_like(g.hits)
        new.misses = np.zeros_like(g.misses)
        new.hit_sx = np.zeros_like(g.hit_sx)
        new.hit_sy = np.zeros_like(g.hit_sy)
        new.hit_n = np.zeros_like(g.hit_n)
        new.height, new.width = g.hits.shape
        new.seq = g.seq + 1
        kfs = []
        for p, kf in zip(poses, self.keyframes):
            new.integrate(kf.points, p)
            kfs.append(kf._replace(pose=p))
        self.grid = new
        self.keyframes = kfs
        self.trajectory = [k.pose for k in kfs]
        self.local.reset_to([(k.pose, k.hx, k.hy) for k in kfs[-self.config.local_map.max_keyframes:]])
        self.pose = compose(poses[-1], rel)

    def load_map(self, grid: OccGrid) -> None:
        """事前に構築済みの地図（凍結済み想定）に差し替える。

        `reset()`と違い**姿勢は動かさない**——呼び出し側が直後にグローバル
        ローカリゼーション（`core/localize.py`）で`pose`を外部から設定する運用を
        想定している。捨てるのは直近の追跡状態だけ。
        """
        self.grid = grid
        self._frozen_field = None
        self._frozen_seq = -1
        self.lost_streak = 0

    def refine(self, pts: ScanPoints, guess: Pose2D, *, trans: float | None = None,
               rot: float | None = None) -> RegisterResult | None:
        """凍結地図の上で`guess`の周りを探し直し、Gauss-Newton で仕上げた結果。

        グローバルローカリゼーションの粗い解を仕上げる用途。**姿勢は書き換えない。**
        """
        cfg = self.config
        f = self._field(guess)
        if f is None or f.empty:
            return None
        d = np.hypot(pts.x, pts.y)
        sel = pts.hit & (d <= cfg.match_range)
        hx, hy = pts.x[sel], pts.y[sel]
        if hx.size == 0:
            return None
        sr = search(f, hx, hy, guess, trans=cfg.reloc_trans if trans is None else trans,
                    rot=cfg.reloc_rot if rot is None else rot,
                    trans_step=cfg.reloc_trans_step, rot_step=cfg.reloc_rot_step)
        return register(f, hx, hy, sr.pose, config=cfg.register)
