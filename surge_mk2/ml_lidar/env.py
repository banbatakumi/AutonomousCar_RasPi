"""ml_lidar/env.py — LiDAR-only E2E強化学習のGymnasium環境。

`ML_LIDAR_V2_PROMPT.md`（旧`ml_lidar`削除後にCopilot CLIの独立レビューを経て
確定した設計方針）の仕様をそのまま実装する。要点:

## 報酬（v1・最小構成 + v16: ステア実効ペナルティ）

`progress`（センターラインへの弧長射影の単調増加分、`dt`で正規化）＋
`collision_penalty`（衝突で終端）＋`steer_effort_weight`（生アクション`a[0]`
＝舵角速度指令の**絶対値そのもの**に対する罰則、既定0）。TAL項・報酬正規化は
入れない。`progress`はラップアラウンド処理（半周を超える飛びを`±total_length`
で補正）を最初から実装する——これが無いと周回完走判定自体が壊れる。

**v8〜v10で試して手詰まりと確定した`steer_rate_weight`（生アクションの隣接
ステップ差分＝jerkへのL1罰則）は2026-09-11に撤去した**。「隣接差分の総和は
テレスコープ和で始点と終点の差に等しくなるため、方向転換には効くが
『一度に大きく切って据え置く』挙動には何も損しない」という構造的な問題が
あり、重みを0.03〜0.2と振っても単調に悪化した（PROGRESS.md 2026-09-09節）。

**v16の`steer_effort_weight`はこれとは別物**——v13(2026-09-11)の行動空間
再パラメータ化により`a[0]`自体が舵角速度になったので、**その絶対値に直接
罰則を掛けると「意味もなく切り続ける・微調整し続ける」ことそのものを
罰し、いま向いている角度を保持する(`a[0]≈0`)限り罰則を受けない**——
差分ではなく絶対値を見るので上記のテレスコープ和の穴に該当しない。
バンビの「数値上は良いがライン取りが汚い（レーシングラインのような
アウトインアウトに近い走行をしたい）」という指摘を受けて導入（PROGRESS.md
2026-09-11「6回目の続き」節）。L1（絶対値）を選んだのは、L2（二乗）だと
0付近の勾配がほぼ平坦で微小な補正を抑止する力が弱いため——「一定角度を
正確に保持する」ことを積極的に奨励したいのでL1のスパース性が狙いに合う
（CAPSの「罰則が強すぎると鈍い方策に倒れる」副作用は既知なので、単一変数
として導入し`eval_stats.py`のステア滑らかさ指標と`collision_rate`/
`mean_speed`を同時に見ながら調整すること）。

## 観測・行動契約（`raspi/auto/e2e_lidar.py`と完全一致させる）

観測は`_to_obs()`、行動→物理量の変換は`_to_physical()`にまとめてある。
`raspi/auto/e2e_lidar.py`の`plan()`はこの2つの関数と対になる後処理を実車側で行う
（同ファイルのコメント参照）。**観測にステア角を含める場合は`self._steer_target`
（方策が舵角速度を積分して直接制御している内部状態、`steer_tau`フィルタ前）を
使う**——`self._steer`（`steer_tau`フィルタ後）でも`VehicleModel`の`steer_actual`
（むだ時間・1次遅れ込みの物理的な実舵角）でもない。方策は自分が直接動かしている
状態を観測できて初めて「現在の積分値＋今回の舵角速度」で次の目標を一貫して
決められる——`steer_tau`以降の物理的な平滑化・遅れは（`VehicleModel`の応答と
同じく）方策から見えない下流の実装詳細として扱う。実車の`E2ELidar.plan()`も
自分の`self._steer_target`（前回まで積分した指令値）を観測に使っており、
ここを揃えないと訓練と実行で観測の意味がずれる。

## LiDARシム経路（中間案）

`sim.lidar.VirtualLidar`でセクタパケットを生成し、`raspi.msgs.ScanAssembler`で
実機と同じ鏡像反転・欠測フラグ組み立てまで通すが、`sim.link`のUART/STM32バイト
フレーミング層は経由しない（`sim_support.stm_us()`は時刻同期なしの単純なns→us変換）。

## 行動→物理量の変換（v13: 舵角→舵角速度への再パラメータ化）

**`a[0]`は舵角そのものではなく舵角速度**（`[-1,1]`を`steer_rate_max_rad_s`
[rad/s]倍したもの）。`self._steer_target += a[0]*steer_rate_max_rad_s*dt`で
積分し`±max_steer`にクランプしたものを「プランナの目標舵角」として扱う
（v1〜v12は`steer = a[0]*max_steer`という位置そのものの写像だった）。

**経緯（2026-09-10〜11決定）**: v1〜v12で報酬側の罰則（`steer_rate_weight`の
L1、weight 0.03〜0.2で単調悪化）・探索ノイズの構造変更（gSDE、tuning後も
改善は誤差程度）の2つの間接的アプローチを尽くしたが、決定論的方策の
生アクションの振動（隣接ステップ符号反転率8〜11%、探索ノイズ込みで21〜36%）
は解消しなかった。文献調査（["Is Bang-Bang Control All You
Need?"](https://arxiv.org/pdf/2111.02552)）で、ガウス方策+tanh
squashingの連続制御は報酬・探索ノイズの設計に関わらず両極端に振れやすい
構造的傾向があると確認——**間接的に「滑らかになることを期待する」のではなく、
1ステップで動ける量そのものに構造的な上限をかける**方針に転換した。
レーシングRL文献（[arXiv:2406.14934](https://arxiv.org/pdf/2406.14934)等）
でも1step当たりの舵角変化を物理的な上限でクリップする手法が使われており、
同じ機構。**行動空間の意味が変わるため既存モデル(v1〜v12)とは非互換
（ゼロから再学習が必要）**。

`steer_tau`一次遅れフィルタは、上記の目標舵角（`self._steer_target`、方策が
直接制御する内部状態）に対して従来通りそのまま適用し`self._steer`
（サーボ応答を模した状態）を作ってから`VehicleModel`に渡す——`steer_rate_max_rad_s`
の上限は「プランナが1stepで動かせる量」を、`steer_tau`は「サーボの物理応答の遅れ」
を表しており役割が異なるため両方残す。`VehicleModel`自体もさらに`dead_time_s`/
`tau_steer_s`で実機の物理応答を模擬するので、都合三段のフィルタがかかる。
`speed = (a[1]+1)/2 * max_speed`（後退は無し、`e2e_lidar.py`と同じ0〜max_speed）。

**`max_steer`は`EnvConfig`に持たせず、`config/vehicle.toml`（`VehicleSpec.max_steer`）
をそのまま使う。** 車体の物理的な最大舵角は実測確定済みの1つの値であり、学習側で
別の値を持つと「モデルの行動空間の上限」と「車両が物理的に切れる上限」がずれる
（`VehicleModel.step()`はどのみち`sp.max_steer`でクランプするので、行動空間側を
それより狭くすると実際に使える舵角が無駄に削られ、広くすると学習が範囲外の
行動を出してクランプで潰れる無駄撃ちを増やす）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from raspi.auto.base import scan_window
from raspi.msgs.convert import ScanAssembler
from raspi.msgs.types import Scan
from sim.course import Course
from sim.lidar import VirtualLidar
from sim.params import SimParams
from sim.vehicle import DriveInput, VehicleModel, VehicleSpec

from .course_gen import random_walk_loop_course
from .sim_support import pump_lidar_until_scan, stm_us

__all__ = ["EnvConfig", "LidarE2EEnv"]

NS = 1_000_000_000
#: 物理積分の刻み。LiDARセクタ周期（120Hz≒8.3ms）より細かく取り、
#: セクタの取りこぼしが無いようにする
_MAX_RESET_PUMP_S = 0.25
#: `reset()`でスタート姿勢が壁に埋まっていた場合の作り直し上限回数
_MAX_RESET_RETRY = 5


@dataclass
class EnvConfig:
    dt: float = 0.05                  # RLステップの制御周期 [s]（20Hz）
    physics_substep: float = 0.005    # 物理積分の刻み [s]
    # 前方270°（左右135°ずつ）。360°にしないのは、Conv1dが観測配列の両端を
    # 「隣接した円環」として扱えないため——除外した真後ろ90°ぶんが「継ぎ目」を
    # 引き受ける形にすることで、配列の両端が現実にも本当に離れた方向になる
    # （継ぎ目問題自体は起きなくなる）。180°では継ぎ目は避けられるが、ヘアピンの
    # 内側の壁など側方〜後方寄りの情報が視野外になりやすいため270°まで広げてある
    fov_deg: float = 270.0
    max_range: float = 10.0           # 実機LIDAR_SECTOR(無圧縮mm)の実用レンジ。`e2e_lidar.py`既定と揃える
    max_speed: float = 2.0
    #: `a[0]`（[-1,1]）に掛けて舵角速度[rad/s]にするスケール（v13で導入）。
    #: 2.0rad/sは、v1〜v12で実測した振動レート(0.6〜1.6rad/s)を明確に下回りつつ、
    #: 全舵角域(1.05rad)を約0.5秒で切れる値として選定（2026-09-11決定）
    steer_rate_max_rad_s: float = 2.0
    steer_tau: float = 0.10           # `e2e_lidar.py`の`steer_tau`既定と同じ
    #: v3検証(2026-09-07)で-10.0だと学習後半(400k~1Mstep)にかけて速度が単調に上がる一方で
    #: 汎化コースでの衝突率も0%→10%前後まで悪化する傾向が実測された——`progress`が速度に
    #: 比例するため、探索ノイズ(std)が下がり方策が先鋭化するほど「衝突するまでに稼げる
    #: 累積progress」が増え、固定の衝突ペナルティでは抑止力が相対的に薄まっていくと考えられる。
    #: v4ではこの抑止力を強めて同じ傾向が緩和されるか切り分ける（他は変更しない）
    collision_penalty: float = -30.0
    lap_bonus: float = 10.0
    #: 生アクション`a[0]`（-1..1、フィルタ前。v13以降は舵角速度指令）の絶対値に
    #: 掛ける罰則の重み。既定0（無効）。v16でCLIの`--steer-effort-weight`で
    #: 指定する（`EnvConfig`docstring参照、旧`steer_rate_weight`との違いに注意）
    steer_effort_weight: float = 0.0
    randomize_lidar: bool = True
    randomize_dynamics: bool = True
    dynamics_jitter_frac: float = 0.2


class LidarE2EEnv(gym.Env):
    """:param course: 指定すると`reset()`のたびにこの固定コースを使う（eval用）。
        `None`（既定）なら`course_gen.random_walk_loop_course()`で毎回作り直す（train用）。
    """

    metadata: dict = {"render_modes": []}

    def __init__(self, config: EnvConfig | None = None, *, course: Course | None = None,
                seed: int | None = None) -> None:
        """:param seed: 構築時点の乱数シード。`VecEnv`の各ワーカーに別々の値を渡すと、
            `reset(seed=...)`を呼ばずとも1本ごとに異なるコース・ドメインランダム化列になる
            （未指定ならOSエントロピー由来。`reset(seed=...)`で明示的に上書きもできる）
        """
        super().__init__()
        self.cfg = config or EnvConfig()
        self._fixed_course = course
        self._rng = np.random.default_rng(seed)
        #: 車体の物理的な最大舵角。`config/vehicle.toml`のものをそのまま使う
        #: （学習側で別の値を持たせない。`EnvConfig`docstring参照）
        self.max_steer = VehicleSpec.load().max_steer

        self.action_space = spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)
        n_window = int(round(self.cfg.fov_deg)) + 1
        low = np.concatenate([np.zeros(n_window + 1, dtype=np.float32), [-1.0]]).astype(np.float32)
        high = np.concatenate([np.ones(n_window + 1, dtype=np.float32), [1.0]]).astype(np.float32)
        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)

        self.course: Course | None = None
        self.vehicle: VehicleModel | None = None
        self.lidar: VirtualLidar | None = None
        self.assembler = ScanAssembler()
        self._scan: Scan | None = None
        self._body: np.ndarray | None = None
        self._t_ns = 0
        self._steer = 0.0
        self._steer_target = 0.0
        self._s_prev = 0.0
        self._lap_progress_m = 0.0
        self._centerline_xy: np.ndarray | None = None
        self._centerline_arc: np.ndarray | None = None
        self._total_length = 0.0

    # ── Gym API ──

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        if seed is not None:
            self._rng = np.random.default_rng(seed)

        spec = self._sample_vehicle_spec()
        for _ in range(_MAX_RESET_RETRY):
            course = self._fixed_course or random_walk_loop_course(self._rng)
            body = course.body_samples(spec.footprint)
            if not course.collides(*course.start, body):
                break
            if self._fixed_course is not None:
                break  # 固定コースは作り直せない。壁埋まりはコース側の不備として諦める
        self.course = course
        self._body = body
        self._prepare_centerline(course)

        params = self._sample_sim_params()
        self.vehicle = VehicleModel(spec, course.start)
        lidar_seed = int(self._rng.integers(0, 2**31 - 1))
        self.lidar = VirtualLidar(course, spec, params, seed=lidar_seed)
        self.assembler = ScanAssembler()
        self._scan = None
        self._t_ns = 0
        self._steer = 0.0
        self._steer_target = 0.0
        self._s_prev = self._project_arc_length(self.vehicle.x, self.vehicle.y)
        self._lap_progress_m = 0.0

        self._pump_lidar_until_scan()
        return self._to_obs(), {}

    def step(self, action: np.ndarray):
        steer_cmd, speed_cmd = self._to_physical(action)
        raw_steer = float(np.clip(np.asarray(action, dtype=np.float64)[0], -1.0, 1.0))
        steer_effort_penalty = self.cfg.steer_effort_weight * abs(raw_steer)

        alpha = 1.0 if self.cfg.steer_tau <= 1e-3 else \
            1.0 - math.exp(-self.cfg.dt / self.cfg.steer_tau)
        self._steer += (steer_cmd - self._steer) * alpha

        cmd = DriveInput(armed=True, target_speed=speed_cmd, target_steer=self._steer)

        n_sub = max(1, int(round(self.cfg.dt / self.cfg.physics_substep)))
        sub_dt = self.cfg.dt / n_sub
        collided = False
        for _ in range(n_sub):
            self.vehicle.apply(cmd)
            self.vehicle.step(sub_dt)
            self._t_ns += int(round(sub_dt * NS))
            hit = self.course.collides(self.vehicle.x, self.vehicle.y, self.vehicle.yaw, self._body)
            self.vehicle.note_collision(hit)
            self._poll_lidar()
            if hit:
                collided = True
                break

        s_now = self._project_arc_length(self.vehicle.x, self.vehicle.y)
        delta = self._progress_delta(s_now)
        self._s_prev = s_now
        self._lap_progress_m += delta
        progress = max(0.0, delta)

        reward = progress / self.cfg.dt - steer_effort_penalty
        terminated = False
        if collided:
            reward += self.cfg.collision_penalty
            terminated = True
        elif self._lap_progress_m >= self._total_length:
            reward += self.cfg.lap_bonus
            terminated = True

        obs = self._to_obs()
        info = {
            "progress_m": progress,
            "lap_progress_m": self._lap_progress_m,
            "collided": collided,
            "speed": self.vehicle.speed,
        }
        return obs, reward, terminated, False, info

    # ── 観測・行動の変換（`raspi/auto/e2e_lidar.py`と対になる契約） ──

    def _to_obs(self) -> np.ndarray:
        w = scan_window(self._scan, self.cfg.fov_deg, self.cfg.max_range)
        scan_n = np.asarray(w.dist, dtype=np.float32) / self.cfg.max_range
        speed_norm = float(np.clip(self.vehicle.speed / self.cfg.max_speed, 0.0, 1.0))
        steer_norm = float(np.clip(self._steer_target / self.max_steer, -1.0, 1.0)) \
            if self.max_steer > 1e-6 else 0.0
        return np.concatenate([scan_n, [speed_norm, steer_norm]]).astype(np.float32)

    def _to_physical(self, action: np.ndarray) -> tuple[float, float]:
        """モデル出力（tanh、-1..1）→ 物理量。`e2e_lidar.py`の後処理と対になる。

        `a[0]`は舵角速度（`EnvConfig`docstring参照）。`self._steer_target`
        （方策が積分で直接制御する目標舵角、`steer_tau`フィルタ前）を更新して返す
        ——`step()`側はこれを従来通り`steer_tau`フィルタに通す。
        """
        a = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        self._steer_target = float(np.clip(
            self._steer_target + float(a[0]) * self.cfg.steer_rate_max_rad_s * self.cfg.dt,
            -self.max_steer, self.max_steer))
        speed = (float(a[1]) + 1.0) * 0.5 * self.cfg.max_speed
        return self._steer_target, speed

    # ── LiDAR ──

    def _poll_lidar(self) -> None:
        for gen_ns, pkt in self.lidar.poll(self._t_ns, self.vehicle, stm_us):
            scan = self.assembler.feed(pkt, gen_ns)
            if scan is not None:
                self._scan = scan

    def _pump_lidar_until_scan(self) -> None:
        """`reset()`直後、車体は動かさずスキャンが1周ぶん組み上がるまで時計だけ進める。"""
        sub_ns = int(round(self.cfg.physics_substep * NS))

        def _step():
            self._t_ns += sub_ns
            self._poll_lidar()
            return self._scan

        pump_lidar_until_scan(_step, self.cfg.physics_substep, _MAX_RESET_PUMP_S)

    # ── センターライン弧長進捗（ラップアラウンド対応） ──

    def _prepare_centerline(self, course: Course) -> None:
        if course.centerline is None:
            raise ValueError("centerlineを持たないコースは進捗報酬を計算できない")
        xy = course.centerline[:, :2]
        seg = np.hypot(*np.diff(xy, axis=0).T)
        arc = np.concatenate([[0.0], np.cumsum(seg)])
        closing = math.hypot(xy[-1, 0] - xy[0, 0], xy[-1, 1] - xy[0, 1])
        self._centerline_xy = xy
        self._centerline_arc = arc
        self._total_length = float(arc[-1] + closing)

    def _project_arc_length(self, x: float, y: float) -> float:
        d2 = (self._centerline_xy[:, 0] - x) ** 2 + (self._centerline_xy[:, 1] - y) ** 2
        idx = int(np.argmin(d2))
        return float(self._centerline_arc[idx])

    def _progress_delta(self, s_now: float) -> float:
        """`s_prev → s_now`の弧長差分。**半周を超える飛びは周回とみなして補正する**——
        しないと、ゴール直前（弧長が`total_length`に近い）からスタート付近
        （弧長が`0`に近い）へ一周した瞬間に、`progress`が`total_length`ぶん
        負に振れて「一周した」判定もprogress自体も壊れる。
        """
        d = s_now - self._s_prev
        half = self._total_length / 2.0
        if d > half:
            d -= self._total_length
        elif d < -half:
            d += self._total_length
        return d

    # ── ドメインランダム化 ──

    def _sample_sim_params(self) -> SimParams:
        if self.cfg.randomize_lidar:
            return SimParams()
        p = SimParams()
        p.lidar_noise_sigma_m = 0.0
        p.lidar_noise_rel = 0.0
        p.lidar_drop_rate = 0.0
        p.lidar_sector_drop_rate = 0.0
        p.lidar_delay_ms = 0.0
        p.lidar_delay_jitter_ms = 0.0
        return p

    def _sample_vehicle_spec(self) -> VehicleSpec:
        base = VehicleSpec.load()
        if not self.cfg.randomize_dynamics:
            return base
        frac = self.cfg.dynamics_jitter_frac

        def jitter(v: float) -> float:
            return float(v * (1.0 + self._rng.uniform(-frac, frac)))

        base.tau_steer_s = jitter(base.tau_steer_s)
        base.dead_time_s = jitter(base.dead_time_s)
        base.tau_speed_s = jitter(base.tau_speed_s)
        base.mu = jitter(base.mu)
        return base
