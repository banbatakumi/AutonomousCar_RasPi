"""ml_lidar/env.py — LiDAR-only E2E強化学習のGymnasium環境。

`ML_LIDAR_V2_PROMPT.md`（旧`ml_lidar`削除後にCopilot CLIの独立レビューを経て
確定した設計方針）の仕様をそのまま実装する。要点:

## 報酬（v21・最小構成へ回帰）

`progress`（センターラインへの弧長射影の単調増加分、`dt`で正規化）＋
`collision_penalty`（衝突で終端）＋`lap_bonus`（完走で終端）のみ。報酬正規化は
入れない。`progress`はラップアラウンド処理（半周を超える飛びを`±total_length`で
補正）を最初から実装する——これが無いと周回完走判定自体が壊れる。

## v21: v16〜v20で足した罰則3種をすべて撤去した（2026-09-17）

撤去の根拠は、この日に初めて測った「ラップタイムの分解」と文献の突き合わせ:

- **失っているのはラインではなく速度だった**。v17の実ラップはMCL理想ラップの
  1.24倍だが、走行距離はMCL長の0.98〜1.04倍——欠損はほぼ全部が平均速度
  （理想の0.80〜0.83倍）。`straight`コースでは理想速度プロファイルが99%全開
  なのに方策の全開率は9%。`raceline_weight`(v18/v19)も`steer_angle_weight`(v20)も、
  **タイムを失っている場所を狙っていなかった**
- **速度が出ないのは報酬設計の産物ではなく舵の精度の帰結**。v17のスロットル
  指令を+10%持ち上げるだけで衝突率が6%→16%になる（ランダム100本）。方策は
  自分の舵で安全に走れる上限で既に走っている——
  [arXiv:2401.17732](https://arxiv.org/pdf/2401.17732)の「E2Eは軌跡が滑らかで
  ないため保守的な速度しか選べない」という記述と一致する
- **`steer_angle_weight`(v20)の失敗は文献が実証済みだった**。Evans et al.
  [arXiv:2103.10098](https://arxiv.org/pdf/2103.10098)はステア絶対値罰`-β|δ|`を
  racing報酬中で最遅（10.2s、CTH(MCL基準)は8.8s）と報告し、「コーナーを非常に
  下手に曲がる」「トラック中央に留まり、コーナーで長い経路を取る」と明記して
  いる——v20の「あらゆるコースで壁に張り付く」はこの既知の失敗そのもの。
  `steer_effort_weight`(v16)も同系統の罰則なので併せて撤去する

**撤去済みの変数（再導入するなら理由と共に）**:

| 変数 | 版 | 罰した量 | 撤去理由 |
|---|---|---|---|
| `steer_rate_weight` | v8〜v10 | 生アクションの隣接差分（jerk）のL1 | テレスコープ和になるため「一度に大きく切って据え置く」を罰せない。重み0.03〜0.2で単調悪化 |
| `steer_effort_weight` | v16〜v20 | 生アクション`a[0]`（舵角速度指令）の絶対値 | 上記の通りステア罰は文献で最遅。単独の効果を確認できないまま既定0.02で5版ぶん入りっぱなしだった |
| `raceline_weight` | v18・v19 | MCL（最小曲率線）からの横偏差 | 重み0.5→1.0で2連敗。そもそもv17の走行距離は既にMCL長の±4%以内で、伸びしろが無い量を狙っていた |
| `steer_angle_weight` | v20 | 目標舵角`_steer_target`の絶対値 | Evans 2021の通り。実測でもラップ比1.24→1.42倍と全版中最悪 |

**今後、舵の滑らかさは報酬項ではなく損失項で攻める**——CAPSのspatial smoothness
（[arXiv:2012.06644](https://arxiv.org/pdf/2012.06644)、観測に実測ノイズ相当のσを
乗せた`‖π(s)-π(s̄)‖`を損失に加える）やLCPの勾配罰則
（[arXiv:2410.11825](https://arxiv.org/html/2410.11825v1)、λ=0.002）。**決定論的
方策で振動が残る＝探索ノイズではなく方策写像そのものが粗い**ということなので、
サンプルされた行動への報酬罰（v8〜v20で試した全部）では原理的に届かない。事後
ローパスも不可——v17の生アクションに後付けするとτ=0.1sで衝突率44%、τ=0.2sで
100%になり、閉ループが高周波補正に依存していることが確認できている。

理想ラインを方策に与えること自体を諦めたわけではない。2025年の実車SOTA
（[On-Board RL, RLC 2025](https://rlj.cs.umass.edu/2025/papers/RLJ_RLC_2025_90.pdf)・
[TC-Driver](https://arxiv.org/pdf/2205.09370)・[RLPP](https://arxiv.org/pdf/2501.17311)）は
いずれも**報酬は進捗＋衝突の2項に保ったまま、参照ラインと左右境界を観測に
入れる**構成をとる。報酬へ戻すのではなく観測へ移すのが次の検討先（その場合
実車側は`raspi/auto/e2e_lidar.py`が単一スキャンから局所中心線を復元する必要が
あり、[arXiv:2401.17732](https://arxiv.org/pdf/2401.17732)の局所地図抽出が該当する）。

## v21b: 制御周期をLiDARの回転周期(10Hz)へ合わせた（2026-09-17）

v1〜v21aは`dt=0.05`（20Hz）だったが、**LD06は10Hz回転なのでスキャンは100msに1度しか
更新されない**。実測すると`dt=0.05`では**54%のステップで観測のスキャン部が前ステップと
完全に同一**で、方策は半分のステップを新しい外界情報ゼロで判断していた。

決定的だったのは、振動がどちらのステップで起きているかの内訳（v17、ノイズ無効、n≈8800）:

| | `|Δa[0]|` | 隣接ステップ符号反転率 |
|---|---|---|
| スキャン据え置きのステップ | 0.056 | **0.2%** |
| スキャン更新のステップ | 0.139 | **13.7%** |

**振動はスキャンが更新された瞬間にほぼ全部集中していた**——据え置きの間は滑らかに
惰行し、新しいスキャンが来るたびに大きく（しばしば符号反転を伴って）舵を振り直す、
という周期2のパターン。`dt=0.10`にすると据え置き率は54%→10%（残りは遅延ジッタぶん）。

**さらにこれは訓練と実車の食い違いでもあった**。`raspi/nodes/planning_node.py:_replan()`
は`scan.seq`が前回と同じなら計算せず戻るので、**実車の`E2ELidar.plan()`は10Hzでしか
呼ばれない**。つまり実車は上の表の「スキャン更新のステップ」だけを、しかも学習時の2倍の
`dt`（＝1決定あたり`steer_rate_max*dt`が2倍＝0.1rad→0.2rad）で実行していた。
`steer_tau`の1次遅れ係数`1-exp(-dt/tau)`も0.393対0.632でずれていた。
**実車で「舵が発散し壁に衝突」（PROGRESS.md 2026-09-02のv10診断）が出ていた理由の
有力な説明**になっている。

**観測ノイズの増幅は主因ではない**（先にこちらを疑ったが実測で否定した）。LiDARノイズを
完全に切っても`|Δa[0]|`は0.116→0.102としか下がらず（v17、n=60エピソード）、方策の観測
感度も増幅率4.3倍で一定＝実機ノイズ相当(σ≒1.5cm)が生む舵指令の振れは0.006
（観測された0.116の5%）。**CAPS spatial/LCP勾配罰則が狙うのはこの5%の方**なので、
先に`dt`を合わせてから残りを見る。

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
from .obstacles import add_obstacles
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
    #: RLステップの制御周期 [s]。**LD06の回転周期(10Hz、`sim.lidar.SCAN_HZ`)と実車の
    #: 計画周期に一致させてある**——`raspi/nodes/planning_node.py:_replan()`は
    #: `scan.seq`が変わったときだけ`plan()`を呼ぶので、実車の方策は10Hzでしか
    #: 動かない。v1〜v21aの0.05(20Hz)は**訓練と実行で決定レートが2倍ずれていた**
    #: （モジュールdocstring「v21b」参照）
    dt: float = 0.10
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
    #:
    #: **v23(2026-09-22)で2.0→1.0に下げた。** v13が2.0を選んだのは`dt=0.05`の頃で、
    #: 意図は「1決定あたり0.1rad（`max_steer`の19%）しか動かせないようにする」ことだった。
    #: v21で`dt`を0.10へ変更した際にこの値を据え置いたため、**1決定あたり0.2rad＝当初の
    #: 意図の2倍の権限**になっていた（見落とし）。1.0にすると`1.0*0.10=0.1rad`で
    #: v13の意図どおりに戻る。
    #:
    #: 妥当性は理想ライン側から確認した——MCLを速度プロファイル通りに追従するのに
    #: 必要な舵角速度は中央0.129・p90 0.954 rad/sで、**1.0 rad/sならp90まで賄える**
    #: （残る約9%はヘアピン等のタイトコーナーで、ここは曲がりきれなくなるリスクがある
    #: ——`collision_rate`と`lap_ratio`を必ず併せて見ること）。一方v22の実測は
    #: 平均0.626 rad/sで**理想の中央値の約4.9倍**も舵を動かしていた。
    #:
    #: 報酬でステアを罰するアプローチ（`steer_rate_weight`/`steer_effort_weight`/
    #: `steer_angle_weight`）は3つとも失敗し撤去済み（モジュールdocstring「v21」参照）。
    #: **行動空間そのものに構造的な上限をかけるこの方法だけが、これまで唯一効いた**
    #: （v13）ので、報酬を複雑化せずに滑らかさを追う手段としてここを絞る
    #:
    #: **v25(2026-09-24)で1.0→1.5に上げた。** 静的障害物を入れたv24（1.0）の失敗は
    #: 衝突・停止とも障害物の直前に集中し、衝突は減速せず1.4m/sのまま避けきれずに起きていた
    #: ——1.0では1.4m/sで舵を0→最大まで切るのに0.73m走る計算で、回避の舵が間に合わない、
    #: という読み。2.0（v22）へ戻すと滑らかさを失うので中間を取った（PROGRESS.md 2026-09-24節）
    steer_rate_max_rad_s: float = 1.5
    steer_tau: float = 0.10           # `e2e_lidar.py`の`steer_tau`既定と同じ
    #: v3検証(2026-09-07)で-10.0だと学習後半(400k~1Mstep)にかけて速度が単調に上がる一方で
    #: 汎化コースでの衝突率も0%→10%前後まで悪化する傾向が実測された——`progress`が速度に
    #: 比例するため、探索ノイズ(std)が下がり方策が先鋭化するほど「衝突するまでに稼げる
    #: 累積progress」が増え、固定の衝突ペナルティでは抑止力が相対的に薄まっていくと考えられる。
    #: v4ではこの抑止力を強めて同じ傾向が緩和されるか切り分ける（他は変更しない）
    collision_penalty: float = -30.0
    lap_bonus: float = 10.0
    randomize_lidar: bool = True
    randomize_dynamics: bool = True
    dynamics_jitter_frac: float = 0.2
    #: 静的な円柱障害物を置くコースの割合（2026-09-24）。**0なら乱数を1つも消費しない**
    #: ——v23以前のrun・評価と同じシードで同じコース列になる。置くコースでは
    #: 1〜`max_obstacles`個を一様に引き、`ml_lidar/obstacles.py`が1個ずつ通過可能性を
    #: 確かめてから刻む（通れない配置は学習させない。同モジュールdocstring参照）。
    #: 固定コース（`course=`指定、evalの使い回し）には置かない
    obstacle_prob: float = 0.0
    max_obstacles: int = 4


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
        # 障害物はスタート地点の前後を避けて置く（`obstacles._START_CLEAR_*`）ので、
        # 上のスタート姿勢の壁埋まり判定をやり直す必要はない
        if (self._fixed_course is None and self.cfg.obstacle_prob > 0.0
                and self._rng.random() < self.cfg.obstacle_prob):
            n = int(self._rng.integers(1, self.cfg.max_obstacles + 1))
            add_obstacles(course, self._rng, n, spec)
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

        reward = progress / self.cfg.dt
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
        """車体位置をセンターラインへ最近傍射影したときの弧長 [m]。"""
        d2 = (self._centerline_xy[:, 0] - x) ** 2 + (self._centerline_xy[:, 1] - y) ** 2
        return float(self._centerline_arc[int(np.argmin(d2))])

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
