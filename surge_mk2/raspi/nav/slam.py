"""SLAM — 脱スキュー・地図・スキャンマッチを束ねて「今どこに居るか」を出す。

**SLAM エンジンを差し替えるならここだけを書き換える。** `Slam` の外に見えているのは
`update()` / `freeze()` / `pose` / `grid` / `trajectory` の5つで、planner
（`raspi/auto/raceline.py`）はその5つしか知らない。

## ★ 車輪オドメトリは使わない。ジャイロは使う

**この2つを一緒くたにしてはいけない。** 弱点がまったく違う。

| 車輪オドメトリの弱点 | ジャイロに当てはまるか |
|---|---|
| エンコーダが**操舵輪である前輪**に付いている（§2） | ✗ 無関係 |
| `encoder_ticks_per_rev` が未実測（STM32 側） | ✗ 無関係 |
| 加減速でタイヤが滑ると距離が狂う | ✗ **滑っても車体の回頭は正しく測る** |
| 低速で `wheel_speed[]` が符号ごとばたつく（`PROGRESS.md`） | ✗ 無関係 |

ジャイロの弱点は**バイアスドリフトだけ**で、時定数は分オーダー。ところがここで
欲しいのは **1 周期（100ms）ぶんの回頭**なので、バイアス 0.02°/s でも誤差は
**0.002°**。スキャンマッチ1回のばらつき（±0.15°）より 2 桁良い。

    回頭   → ジャイロ（短期に強い）。長期のバイアスは地図が直す
    前進   → STM32 の `speed`（下記）。無ければ等速度モデルで代用

**地図そのものは LiDAR だけで作る。** ジャイロが担うのは「次のスキャンがどこを
向いているか」の予測だけで、壁の位置を決める材料にはならない。

## ★ 前進速度だけは車輪の値を「弱い拠り所」として使う

**進行方向の位置は、平行な壁に挟まれた区間では点群から決まらない**（corridor
problem）。しかも分離帯があると車線が 0.42m の管になり、その先が見えないので
コーナーによる回収も効かない。実測では**5m の直線を走っただけで縦に 4.5m 流れた**
（向きの誤差は 1° のまま）。観測源が1つも無いと、ここは原理的に閉じない。

そこで `VehicleState.speed`（STM32 が車体中心線方向に射影し、ローパスを掛けた値）
を 1 周期ぶんの前進量に使う。**「絶対距離の物差しとして信じる」のとは別物**である
ことに注意:

- 使うのは 1 周期（100ms）ぶんの増分だけ。倍率誤差が 2% あっても 0.45m/s で **0.2mm**
- 累積した誤差は地図とのマッチが直す
- 滑れば増分ごと間違えるが、それも次のコーナーで回収される

**`wheel_speed[]` は使わない**（低速で符号ごとばたつく。`PROGRESS.md` の実測）。
`speed` は同じ測定できれいだった方。

`speed` が無いときは自分の推定姿勢の履歴から等速度モデルで代用する
（10Hz なら 2 m/s² の加速でも 1 周期の予測誤差は 2cm）。

## ジャイロのバイアスは「回頭を2通りで測って引き算」する

    gyro_bias += β × ( ジャイロが言う回頭速度 − 点群が言う回頭速度 )

点群が言う回頭速度は、**スキャン対スキャンの結果**（あればそれ。無ければ
地図とのマッチが向きをどちらへ直したか）から取る。どちらもジャイロとは
独立に回頭を測っているので、差の平均がバイアスそのものになる。

`ジャイロの読み − 実際に採用した回頭` で作ってはいけない。採用値は予測
（＝バイアス込み）にほぼ一致してしまい、差が消えて**いつまでも収束しない**
（実測で 120 周期かけて 6% しか寄らなかった）。

β は遅くする（時定数 10 秒相当）。速くすると**点群側のばらつきがバイアスに
化けて、それがまた予測を狂わせる**という循環に入る。

## 1周期でやること

    ① 脱スキュー          走りながら測った1周を、1周の終端時刻の点群に直す
    ② 推測航法            ジャイロ（回頭）と `speed`（前進）で1周期ぶんの動きを作る
    ③ スキャン対スキャン  前の周の点群と直接合わせて、②を**格子に縛られずに**磨く
    ④ スキャン対地図      地図に対していちばん合う姿勢を探す（長期のドリフトを直す）
    ⑤ 地図更新            その姿勢で点群を焼く（凍結後はしない）

③ と ④ は役割が違う。**③ は相対移動（1周期ぶん）、④ は絶対位置**。
③ は地図の 2.5cm 量子化に縛られないので相対移動が精密になり、④ はそれが
積み上がったドリフトを地図基準で直す。片方だけでは足りない。

## ★ 姿勢の基準時刻は「点群の時刻」であって「今」ではない

`plan()` が呼ばれる時刻は、点群の最後の点が測られた時刻より 15〜20ms 遅い
（LiDAR の伝送遅延 + 受信処理）。**オドメトリを「今」まで進めてから、
「点群の時刻」の地図と照合すると、毎周期その 15〜20ms ぶんだけ後ろへ
引き戻される。** 1周期あたり 1cm・0.1° の小さな量だが、10Hz で 500 周期
走れば回頭で数十度になる。実測でこれが誤差のほぼ全部だった。

なので**姿勢は常に `Points.t_ref_ns`（1周の終端）時点のもの**として持ち、
オドメトリを進める区間も前回の `t_ref` から今回の `t_ref` までにする。
呼び出し側から渡される `dt` は使わない（あれは「plan の呼び出し間隔」で、
点群の間隔とは 1〜2 割ずれる）。

その結果 `pose` は「今」より少し過去のものになるが、Pure Pursuit には
もともと遅延補償（`nav/purepursuit.py`）が入っているので不都合は無い。

## 姿勢を決めるのはスキャンマッチだけ

等速度モデルは**予測**であって観測ではない。それ単独ではどこまでも流れていく
（積分すれば必ずそうなる）ので、絶対位置を決めるのは常にマッチの答え。
`match_gain` は「予測から答えへどれだけ寄せるか」で、**1.0 が素直な置き換え**。

1 未満にすると 1 回ぶんのマッチのばらつき（5cm 格子・実機相当ノイズで
±0.4cm / ±0.15°）が均されるが、**下げすぎると予測の流れを直しきれない**。
車輪オドメトリのような独立した情報源があるわけではないので、
**下げてよい幅は狭い**（既定 0.7）。

## ★ 地図に焼く条件を3つに絞る（2026-08-13 見直し）

実機の GUI で地図が「同じコースを角度違いで重ね描きした」形に崩れたのを受けて、
**毎周期・全点を焼くのをやめた**。崩れる経路はいつも同じで、
`少し汚れた地図 → 照合がずれる → もっと汚れる` の一方通行。入口を塞ぐ。

1. **近い点だけ焼く**（`map_range`、既定 3m）。8m の点は測距 σ が格子 2 セル、
   点間隔が 5.6 セルで、**点線状の太い滲み**にしかならない。照合には全点を使う
2. **動いたときだけ焼く**（`kf_dist` / `kf_yaw`）。止まっている 10 秒で 100 枚
   焼くと、測距ノイズのぶんだけ壁が太る。**同じ場所を何度も焼いても情報は増えない**
3. **見失っていないときだけ焼く**（`min_score`）。

   ★ ここを「走ってよい閾値より厳しく」しようとして失敗した。**育ち始めの地図は
   本質的に得点が低い**（既知セルが少ないので重なりようがない）ので、絶対値の
   門を置くと「地図が薄い → 得点が低い → 焼かない → いつまでも薄い」で詰む。
   実測で壁が 73 セルしか立たなかった。**閾値は1つだけにする。**

## ★ 1周して戻ってきたら、地図を作り直す（ループ閉じ）

スキャンマッチだけでは 1 周ぶんの誤差が必ず残る。実測（circuit・24m）で
**60 秒時点の地図は真の壁と 100% 一致していたのに、1 周して戻ってきた時点で
46% まで落ちた**。2 周目の壁が 1 周目とずれた場所に焼かれるからで、
GUI では「同じコースを角度違いで重ね描きした」形に見える。

そこで**焼いた点群（キーフレーム）を全部覚えておき**、周回を検出した時点で

1. **今の点群を、地図に対して広く探し直して**「本当はどこに居るか」を求める
2. 今の推定との差（＝1周ぶんの累積誤差）を**軌跡全体へ弧長に比例して配る**
   （1箇所で吸収させると折れ目ができる）
3. 直した姿勢で**地図を最初から焼き直す**

をやる。ポーズグラフ最適化の、拘束が1本だけの場合に相当する。焼き直しは
500 キーフレームで 0.3 秒ほど掛かるが、**BUILD 段は止まっているので問題にならない**。

★ **「始点にぴったり戻った」と仮定してはいけない。** 周回の判定は始点から
0.8m・35° 以内なら成立するので、最後の姿勢を最初の姿勢に貼り付けると
**最大 0.8m の誤差をこちらから注入する**ことになる。実測で地図の正確さが
99% → 71% に落ちた。残差は必ず**点群と地図の照合**で測る。

## 見失ったら黙って走らない

マッチの得点が `min_score` を下回ったら `lost=True` を返し、**地図を更新しない**。
特徴の乏しい直線や、点群が大きく欠けた周でこれが起きる。ここで地図を焼くと、
ずれた姿勢の壁が地図に混ざって**次の周期はもっと合わなくなる**（一度崩れると戻らない）。

姿勢自体は推測航法の初期値をそのまま採用して進める。数周期なら持つし、
走ってよいかどうかの判断は planner が `lost` を見て決める。

**見失ったままにしない。** `reloc_after` 周期続けて見失ったら、探索範囲を大きく
広げて（事前分布も切って）1回だけ探し直す。通常の探索は ±8cm しか見ないので、
一度大きくずれると**二度と戻れない**（実機で一致度 0.06 のまま走り続けた）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import NamedTuple

import numpy as np

from ..msgs.types import Scan
from .deskew import Points, deskew, integrate_pose, truncate
from .grid import OccGrid
from .scan2scan import match_scans
from .scanmatch import DEFAULT_STAGES, PRIOR_W, PRIOR_XY, PRIOR_YAW, Stage, match

__all__ = ["Slam", "SlamConfig", "SlamUpdate"]

NS = 1_000_000_000
#: 速度推定の1次遅れ係数。**マッチのばらつきが次の予測を揺らすのを防ぐ**
VEL_ALPHA = 0.5
#: ジャイロのバイアス推定の1次遅れ係数。10Hz で時定数 10 秒相当。
#: **速くするとマッチのばらつきがバイアスに化ける**
BIAS_ALPHA = 0.01
#: 推定するバイアスの上限 [rad/s]（≒ 5.7°/s）。これを超えるならセンサ側の異常
BIAS_LIMIT = 0.1
#: 覚えておくキーフレームの上限。1つ約 5KB なので 1500 枚で 7MB 程度
MAX_FRAMES = 1500
#: 見失ったときに1回だけ使う広い探索。**通常の ±8cm では二度と戻れない**
#: **1周期の予算を食い潰さない範囲で**広げる（±40cm・±20°）。実測で
#: ±60cm・5° 刻みにしたら 1 周期 196ms まで伸び、指令の途絶判定に触れた
RELOC_STAGES: tuple[Stage, ...] = (
    Stage(0.40, 0.10, math.radians(20.0), math.radians(5.0)),
    Stage(0.10, 0.03, math.radians(5.0), math.radians(1.0)),
    Stage(0.02, 0.005, math.radians(1.0), math.radians(0.2)),
)

#: `close_loop()` 専用の探索範囲。**`RELOC_STAGES` より広く取ってよい**——
#: 見失い復帰は毎周期の予算（10Hz）を食うが、ループ閉じは1周に1回しか
#: 呼ばれないので余裕がある。周回ごとに `close_loop()` を呼ぶようになった
#: （`raspi/auto/raceline.py`）ことで1回ぶんのドリフトは 20〜30cm 程度に
#: 収まる想定だが、実車のジャイロ・LiDAR ノイズは未確認のため余裕を持たせる
LOOP_CLOSE_STAGES: tuple[Stage, ...] = (
    Stage(0.60, 0.10, math.radians(20.0), math.radians(5.0)),
    Stage(0.10, 0.03, math.radians(5.0), math.radians(1.0)),
    Stage(0.02, 0.005, math.radians(1.0), math.radians(0.2)),
)


@dataclass(frozen=True, slots=True)
class SlamConfig:
    resolution: float = 0.05               #: 格子の刻み [m]
    size_m: float = 24.0                   #: 地図の一辺 [m]。**足りないと外に出る**
    #: 壁として確定するのに必要なヒット回数。**動く物を壁にしないための条件**
    min_hits: int = 3
    min_seen: int = 3                      #: 「空きと確信できる」のに必要な観測回数
    max_range: float = 8.0                 #: これより遠い点は距離を切り詰める [m]
    #: マッチの得点がこれ未満なら見失ったとみなす。0〜1
    min_score: float = 0.35
    #: 地図に焼く点の距離の上限 [m]。照合には `max_range` まで使う
    map_range: float = 3.0
    #: 前回焼いた場所からこれだけ動いたら焼く [m]
    kf_dist: float = 0.03
    #: 同 向き [rad]
    kf_yaw: float = math.radians(2.0)
    #: これだけ連続で見失ったら探索範囲を広げて探し直す
    reloc_after: int = 15
    #: 前の周の点群と直接合わせて推測航法を磨くか（`nav/scan2scan.py`）。
    #:
    #: ★ **既定は切ってある。** 「地図の格子に量子化されない相対移動が得られる
    #: から効くはず」と考えて実装したが、実測では**どのコースでも位置精度を
    #: 悪化させた**（60秒・位置誤差の平均）:
    #:
    #:     oval     12.2cm → 59.5cm
    #:     circuit  11.3cm → 13.4cm
    #:     split    42.0cm → 71.9cm
    #:
    #: 理由は、ジャイロと `speed` による推測航法が既に 1 周期あたり数 mm・
    #: 0.002° と非常に正確で、**ICP がそれを上回る余地が無い**こと。
    #: 上回れないのに毎周期ふらつくぶんだけ悪くなる。地図の正確さは
    #: 上がる場合がある（split で 49% → 86%）ので、**推測航法が悪い車体では
    #: 逆転しうる**。実車で `speed` が信用できないと分かったら入れ直すこと
    scan_to_scan: bool = False
    stages: tuple[Stage, ...] = DEFAULT_STAGES
    #: オドメトリ事前分布（`nav/scanmatch.py` の「通路で必ず壊れる」節）。
    #: **`prior_w = 0` にすると得点だけで選ぶ**ので、直線の多いコースで滑る
    prior_xy: float = PRIOR_XY
    prior_yaw: float = PRIOR_YAW
    prior_w: float = PRIOR_W
    #: 推測航法の予測からマッチの答えへ寄せる割合（0〜1）。**1.0 が素直な置き換え**。
    #:
    #: 推測航法（ジャイロ＋`speed`）が 100ms で数 mm の精度を持つので、マッチは
    #: **長期のドリフトだけを直す弱い補正**でよい。上げるとマッチ 1 回のばらつき
    #: （2.5cm 格子・実機相当ノイズで ±0.4cm/±0.15°）が乱歩として溜まる。
    #:
    #: ★ **評価はジャイロのドリフトを実機相当（0.3°/s）にして行うこと。**
    #: ドリフトが小さい条件だと推測航法が完璧に近く、「マッチを弱くするほど
    #: 良い」＝「SLAM を切るほど良い」という無意味な結論しか出ない。
    #:
    #: シム実測（bias 0.3°/s・80秒、位置誤差の平均。circuit / split）:
    #:   推測航法のみ 64 / 61cm  |  **0.1 → 16 / 30cm**  |  0.3 → 28 / 68cm
    match_gain: float = 0.1
    #: **地図を凍結したあと**の寄せ方。走る段（RACE）で使う。
    #:
    #: 地図を作っている間は、マッチのばらつきをそのまま採用すると
    #: **その姿勢で地図を焼いてしまう**（汚れた地図がまた照合を狂わせる）ので
    #: 弱くしていた。**凍結後はその心配が無い ＝ 強く寄せてよい。**
    #: 弱いままだと走行中に自己位置がじわじわ流れ、動的障害物の判定
    #: （壁から 15cm 以内は無視）が誤検出だらけになる
    match_gain_frozen: float = 0.5
    #: `match_gain`（横方向）に対する**進行方向**の寄せ方の比率。0〜1。
    #:
    #: ★ **等方的に寄せると oval のような角の無いループで位置推定が回転する。**
    #: 進行方向の位置は直線区間では点群から決まらない（corridor problem、上の
    #: docstring）。それでもマッチは候補ごとの得点差（ノイズで ±0.001 程度）で
    #: 最大値を選ぶので、その「意味の無い差」を横方向と同じ強さで採用すると、
    #: 毎周期ランダムな向きへ滑り、角が無いコースでは1方向へ蓄積して全体が
    #: コース中心まわりに回転する（実測: 前後誤差 ÷ 向き誤差[rad] がコースの
    #: 実効半径と一致。`docs/progress_archive.md`「oval が渦巻きになる」節）。
    #: そこでマッチの答えを**進行方向・横方向に分解**し、信用できる横方向は
    #: そのまま、信用できない進行方向はこの比率だけ弱めて混ぜる。
    #:
    #: 0 にしないのは、コーナーでは進行方向にも壁からの情報が実在するため
    #: （四方を囲む壁の曲率が付けば、そこだけ進行方向も観測できる）。
    #:
    #: ★★ **未検証。** 上のシム実測はすべて旧（等方的）実装のもの。
    #: この値を含む異方性の効果は `sim.bench` で再確認してから信じること
    match_gain_fwd_ratio: float = 0.3
    lidar_x: float = 0.0                   #: LiDAR の取付位置 [m]（base_link 基準）
    lidar_y: float = 0.0


class SlamUpdate(NamedTuple):
    x: float
    y: float
    yaw: float
    score: float                           #: マッチの得点 0〜1
    lost: bool                             #: 見失った（地図を更新していない）
    matched: bool                          #: 実際に探索したか（最初の数周は False）


@dataclass
class Slam:
    cfg: SlamConfig = field(default_factory=SlamConfig)

    def __post_init__(self) -> None:
        self.grid = self._new_grid()
        self.reset()

    def _new_grid(self) -> OccGrid:
        """空の格子を作る。**版番号は引き継いで必ず増やす。**

        `OccGrid` を作り直すのは「地図を削除」とループ閉じの焼き直しの2回。
        どちらも版が 0 に戻ると、GUI 側の「古い版で新しい版を上書きしない」
        ガード（`gui/src/ws/map.ts`）に**新しい地図の方が弾かれる**。
        実機で「地図を削除しても消えない」「焼き直した地図が反映されない」に
        なっていたのはこれ。**版番号は単調に増やす。**
        """
        g = OccGrid(resolution=self.cfg.resolution, size_m=self.cfg.size_m,
                    min_hits=self.cfg.min_hits, min_seen=self.cfg.min_seen)
        old = getattr(self, "grid", None)
        g.seq = (old.seq + 1) if old is not None else 0
        return g

    def reset(self) -> None:
        """地図も軌跡も捨てて最初から。**`Planner.reset()` から呼ばれる。**"""
        self.grid = self._new_grid()
        self.x = self.y = self.yaw = 0.0
        #: 通った跡 (x, y, yaw)。**中心線の初期値になる**
        self.trajectory: list[tuple[float, float, float]] = []
        #: 累積走行距離 [m]。オドメトリの差分の絶対値を積んだもの
        self.distance = 0.0
        #: 累積回頭 [rad]。**±180 に畳まない。** 周回の判定はこれが主役
        self.heading_total = 0.0
        #: 推定した車速 [m/s]（車体前方）とヨーレート [rad/s]。**等速度モデルの中身**
        self.speed = 0.0
        self.yaw_rate = 0.0
        #: 推定したジャイロのバイアス [rad/s]。**走りながら地図が教えてくれる**
        self.gyro_bias = 0.0
        #: ジャイロを実際に使えた周期の数。0 のままなら配線を疑う
        self.gyro_updates = 0
        self.updates = 0
        self.lost_streak = 0
        #: 焼いた点群そのもの。**ループ閉じで焼き直すために覚えておく**
        self._frames: list[tuple[tuple[float, float, float], Points]] = []
        self._kf: tuple[float, float, float] | None = None
        #: 探し直した回数。**増え続けるなら地図か点群を疑う**
        self.relocs = 0
        #: スキャン対スキャンが採用された回数と、直近の対応率
        self.s2s_used = 0
        self.s2s_inlier = 0.0
        self._prev_pts: Points | None = None
        #: 直近の脱スキュー済み点群。**ループ閉じが残差を測るのに使う**
        self.last_points: Points | None = None
        #: 前回使った点群の基準時刻 [ns]。**姿勢はこの時刻のもの**
        self._t_ref = 0
        self._last_seq = -1

    @property
    def pose(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.yaw)

    @property
    def keyframes(self) -> int:
        return len(self._frames)

    def close_loop(self, pts: Points | None = None, *,
                   max_residual: float = 1.0,
                   stages: tuple[Stage, ...] = LOOP_CLOSE_STAGES) -> float:
        """1周ぶんの誤差を軌跡へ配り、**地図を焼き直す**（上の docstring）。

        :param pts: 今の点群。**これを地図に照合して残差を測る。** 省略すると
            測りようがないので何もしない
        :param max_residual: これを超える残差は配らない [m]。**大きすぎる残差は
            「1周した」の判定自体を疑うべき**で、無理に閉じると地図を壊す
        :returns: 配った残差の大きさ [m]

        呼ぶのは周回を検出した瞬間（`freeze()` の直前）。
        """
        n = len(self._frames)
        if n < 20 or pts is None:
            return 0.0

        # ★ 今どこに居るかを**地図に聞く**。広く探すのは、1周ぶんのドリフトが
        #   通常の探索範囲（±8cm）より大きいことが普通だから
        fix = match(self.grid, pts, self.pose, stages=stages, prior_w=0.0)
        if not fix.searched or fix.score < self.cfg.min_score:
            return 0.0
        ex, ey = fix.x - self.x, fix.y - self.y
        eyaw = _wrap(fix.yaw - self.yaw)
        residual = math.hypot(ex, ey)
        if residual > max_residual:
            return 0.0

        (x0, y0, _yaw0), _ = self._frames[0]

        # 弧長に比例して配る。**均等割りだと、止まっていた区間にも誤差を配ってしまう**
        seg = [0.0]
        for i in range(1, n):
            a = self._frames[i - 1][0]
            b = self._frames[i][0]
            seg.append(seg[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        total = seg[-1] or 1.0

        self.grid = self._new_grid()
        self.trajectory = []
        fixed = []
        for i, ((px, py, pyaw), pts) in enumerate(self._frames):
            w = seg[i] / total
            # 向きの補正は**その点より前の回頭にだけ効く**ので、位置は回転も込みで直す
            dth = eyaw * w
            c, s = math.cos(dth), math.sin(dth)
            rx, ry = px - x0, py - y0
            nx = x0 + rx * c - ry * s + ex * w
            ny = y0 + rx * s + ry * c + ey * w
            pose = (nx, ny, _wrap(pyaw + dth))
            self.grid.integrate(pts, pose)
            self.trajectory.append(pose)
            fixed.append((pose, pts))
        self._frames = fixed
        self.x, self.y, self.yaw = self.trajectory[-1]
        self._kf = self.pose
        return residual

    def freeze(self) -> None:
        self.grid.freeze()

    # ── 本体 ──

    def update(self, scan: Scan, dt: float, *,
               yaw_rate: float | None = None,
               speed: float | None = None) -> SlamUpdate:
        """1周ぶんの点群を取り込んで姿勢を更新する。

        :param dt: 前回の `update()` からの経過 [s]。**点群の時刻が取れないときの
            予備**でしかない（上の「姿勢の基準時刻」節）
        :param yaw_rate: ジャイロの生の読み [rad/s]。`None` なら等速度モデルで代用
        :param speed: 車体中心線方向の速度 [m/s]（`VehicleState.speed`）。
            `None` なら等速度モデルで代用

        **数値で受け取る**のは、`wheel_speed[]` のような使ってはいけない値を
        うっかり渡せなくするため（`nav/deskew.py` と同じ方針）。
        `yaw_rate` に渡してよいのは**ジャイロだけ**で、車輪から計算した回頭速度を
        渡すと滑った瞬間に地図が壊れる。
        """
        # 使うのは「バイアスを引いたジャイロ」。無ければ自分の推定値
        rate = self.yaw_rate
        if yaw_rate is not None:
            rate = float(yaw_rate) - self.gyro_bias
            self.gyro_updates += 1
        fwd = self.speed if speed is None else float(speed)

        # ① 脱スキューにも同じ値を使う
        pts = deskew(scan, fwd, rate,
                     mount_x=self.cfg.lidar_x, mount_y=self.cfg.lidar_y,
                     max_range=self.cfg.max_range)

        # ── ② 初期値（等速度モデル） ──
        # **点群の時刻どうしの差**で進める。plan() の呼び出し間隔ではない
        span = dt
        if self._t_ref and pts.t_ref_ns:
            span = (pts.t_ref_ns - self._t_ref) / NS
            if not (0.0 < span < 1.0):     # 時刻が跳んだ。呼び出し間隔で代用する
                span = dt
        if pts.t_ref_ns:
            self._t_ref = pts.t_ref_ns
        span = max(1e-3, span)

        # 推測航法の1周期ぶん（前の周の車体座標での相対移動）
        delta = integrate_pose(0.0, 0.0, 0.0, fwd * span, rate * span)

        # ── ③ スキャン対スキャン。**観測できる方向だけ磨かれる** ──
        measured_rate: float | None = None
        if self.cfg.scan_to_scan and self._prev_pts is not None:
            d = match_scans(self._prev_pts, pts, delta)
            self.s2s_inlier = d.inlier
            if d.ok:
                delta = d.pose
                self.s2s_used += 1
                measured_rate = d.dyaw / span
        self._prev_pts = pts
        self.last_points = pts

        c0, s0 = math.cos(self.yaw), math.sin(self.yaw)
        gx = self.x + delta[0] * c0 - delta[1] * s0
        gy = self.y + delta[0] * s0 + delta[1] * c0
        gyaw = self.yaw + delta[2]

        # ── ③ スキャンマッチ ──
        m = match(self.grid, pts, (gx, gy, gyaw), stages=self.cfg.stages,
                  prior_xy=self.cfg.prior_xy, prior_yaw=self.cfg.prior_yaw,
                  prior_w=self.cfg.prior_w)

        # ── 見失ったままにしない ──
        # **毎周期は試さない。** 広い探索は通常の 10 倍近く掛かるので、
        # 見失っている間ずっと回すと 1 周期の予算を食い潰す（実測 196ms）
        if (self.lost_streak >= self.cfg.reloc_after
                and self.lost_streak % self.cfg.reloc_after == 0
                and self.grid.wall_mask().any()):
            wide = match(self.grid, pts, (gx, gy, gyaw), stages=RELOC_STAGES,
                         prior_w=0.0)
            if wide.searched and wide.score > max(m.score, self.cfg.min_score):
                m = wide
                self.lost_streak = 0
                self.relocs += 1

        lost = m.searched and m.score < self.cfg.min_score
        # **照合できなかった**（点が既知セルにほとんど落ちない）のも、地図が
        # 育ったあとなら「見失った」と同じ。地図が空の最初だけは通す
        if not m.searched and self.grid.wall_mask().any():
            lost = True
        if lost:
            # 探索はしたが合わなかった。姿勢はオドメトリのまま進め、地図は汚さない
            self.lost_streak += 1
            nx, ny, nyaw = gx, gy, gyaw
        else:
            self.lost_streak = 0
            # **置き換えずに寄せる**（docstring の相補フィルタ）。
            # 凍結後は地図を汚す心配が無いので強く寄せる
            g = (self.cfg.match_gain_frozen if self.grid.frozen
                 else self.cfg.match_gain)
            nx, ny = _blend_xy(gx, gy, gyaw, m.x, m.y,
                               g, g * self.cfg.match_gain_fwd_ratio)
            nyaw = gyaw + g * _wrap(m.yaw - gyaw)

        # ── 速度の更新。**次の周期の初期値がこれで決まる** ──
        # 生の差分をそのまま入れるとマッチのばらつきが速度に乗り、それが次の予測を
        # 揺らして発散する。1次遅れで均す（`VEL_ALPHA`）
        dx, dy = nx - self.x, ny - self.y
        fwd = dx * math.cos(self.yaw) + dy * math.sin(self.yaw)
        turned = _wrap(nyaw - self.yaw) / span
        self.speed += VEL_ALPHA * (fwd / span - self.speed)
        self.yaw_rate += VEL_ALPHA * (turned - self.yaw_rate)

        # ジャイロのバイアス。**点群が独立に測った回頭**と引き算する（docstring）。
        # スキャン対スキャンがあればそれを、無ければ地図マッチの補正量を使う
        if yaw_rate is not None and not lost:
            if measured_rate is not None:
                err = rate - measured_rate
            elif m.searched:
                err = -_wrap(m.yaw - gyaw) / span
            else:
                err = None
            if err is not None:
                self.gyro_bias = max(-BIAS_LIMIT,
                                     min(BIAS_LIMIT,
                                         self.gyro_bias + BIAS_ALPHA * err))

        self.heading_total += _wrap(nyaw - self.yaw)
        self.distance += math.hypot(dx, dy)
        # **±180 に畳む。** 畳まないと GUI に −661° と出るし、
        # 何周も回ったあとに float の桁が効いてくる
        self.x, self.y, self.yaw = nx, ny, _wrap(nyaw)

        # ── ④ 地図更新。**3つの条件を全部満たしたときだけ**（docstring 参照） ──
        #
        if not lost and self._moved_enough():
            small = truncate(pts, self.cfg.map_range)
            self.grid.integrate(small, self.pose)
            self.trajectory.append(self.pose)
            self._kf = self.pose
            if len(self._frames) < MAX_FRAMES:
                self._frames.append((self.pose, small))

        self.updates += 1
        self._last_seq = scan.seq
        return SlamUpdate(nx, ny, nyaw, m.score, lost, m.searched)

    def _moved_enough(self) -> bool:
        """前回焼いた場所から十分動いたか。**止まっている間は焼かない。**

        同じ場所を 10 秒（100 枚）焼いても情報は増えず、測距ノイズのぶんだけ
        壁が太るだけ。**「動いたら焼く」が地図をいちばん鋭くする。**
        """
        if self._kf is None:
            return True
        dx = self.x - self._kf[0]
        dy = self.y - self._kf[1]
        if math.hypot(dx, dy) >= self.cfg.kf_dist:
            return True
        return abs(_wrap(self.yaw - self._kf[2])) >= self.cfg.kf_yaw

    # ── 周回の判定に使う量 ──

    def lap_progress(self) -> float:
        """周回の進み具合 [周]。**累積回頭を 360° で割ったもの。**

        位置が始点に戻ったかどうかより先にこれを見る。往復コースや、
        スタート地点をかすめただけの経路を「1周した」と誤認しないため。
        """
        return abs(self.heading_total) / (2.0 * math.pi)

    def trajectory_array(self) -> np.ndarray:
        """軌跡を `(N, 3)` の配列で。空なら `(0, 3)`。"""
        if not self.trajectory:
            return np.zeros((0, 3), dtype=np.float64)
        return np.asarray(self.trajectory, dtype=np.float64)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _blend_xy(gx: float, gy: float, gyaw: float, mx: float, my: float,
              gain_lat: float, gain_fwd: float) -> tuple[float, float]:
    """マッチの答え `(mx, my)` を予測 `(gx, gy)` へ**異方的に**混ぜる。

    残差を進行方向（`gyaw` 向き）と横方向に分解し、別々の利得で寄せてから
    世界座標へ戻す。**横方向（壁との距離）は観測できるので `gain_lat` で
    素直に寄せ、進行方向（corridor problem で観測できない）は `gain_fwd`
    で弱める。** 等方的な利得だと、この「決まらない方向」にまで補正が
    漏れて蓄積し、角の無いループ（oval）で位置推定全体が回転する
    （`SlamConfig.match_gain_fwd_ratio` の docstring 参照）。
    """
    c, s = math.cos(gyaw), math.sin(gyaw)
    ex, ey = mx - gx, my - gy
    e_fwd = ex * c + ey * s
    e_lat = -ex * s + ey * c
    corr_fwd = gain_fwd * e_fwd
    corr_lat = gain_lat * e_lat
    nx = gx + corr_fwd * c - corr_lat * s
    ny = gy + corr_fwd * s + corr_lat * c
    return nx, ny
