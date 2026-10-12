"""横グリップのリアルタイム最適化 — 速度プロファイルが使う横加速度の上限を、走行中の車の
反応から場所ごとに決める（`slam2d_raceline` / `slam2d_route`）。

車に固有の値はシステム同定、**路面で変わる値は走行中に決める**。`config/vehicle.toml` の同定値は
初期値として読むだけで、書き戻さない。横（`GripLearner`）と、加速・減速（`LongLearner`、下の節）を学習する。

## 持ち方（`GripMap`）

地図座標の `CELL_M` 格子。各セルに「通った横加速度の最大 `ok`」「前兆が出た横加速度の最小 `bad`」
[m/s²] と、前兆の量の普段の値 `base`。経路上の位置では持たない（線の引き直し・経路の切替で
対応が崩れるため）。全体で共有する値は `level`（まだ前兆が出ていない場所の限界の見積もり）・
`ok`（どこかで通った最大）・`bad`（複数の場所で出た前兆の値）。

速度プロファイルへ渡すのは `levels()` の `(求める横加速度, 限界の見積もり)`。プロファイルは前者を
横の上限に、後者を摩擦円の半径に使う。

### 「使う割合」は無い

人が決める「限界の何割を使うか」（以前の設定 `lat_use`）は無くした。planner は `lat_use` = 1 を渡し、
学習した限界をそのまま求める（余裕は、前兆の値の `bad_margin` 下で止めるぶんだけ）。`GripLearner` の
`lat_use`（収束後だけ掛ける割合）は残してあるが、使っていない。

## 3つの層

1. **即時に下げる**（100Hz、`observe`）: 前兆が出たその周期に `cap`（目標速度の上限）を出し、
   その場所の `bad` を書く。周単位の更新だけでは、超えた周のうちに外へ出る。大きく超えているとき
   （路面が急に変わった）は、ならした値ではなく今出ている横加速度を採り、加速・減速も同じ割合で下げる
2. **コーナー1つ＝1回の試行**: 実際の横加速度が上限の `trial_use` 以上まで出た区間を抜けたら
   評価する。前兆なしで抜けたら「通った」。`v_max` や加減速で決まっている区間は数えない
3. **共有と挟み込み**: 路面はおおむね一様と見て、通ったら `level` を `step_up` ずつ上げる
   （全体に効く）。前兆が出た場所はそのセルだけ下げ、離れた2か所で近い値の前兆が出たら全体の
   `bad` にして `ok` との間を詰める。幅が `tol` を切ったら上げるのをやめる（下げる方は働き続ける）

### 1か所で前兆が出た後（`_prov`）

1か所の前兆だけでは、そこだけ滑りやすいのか全体の限界なのか分からない。求める横加速度をその
前兆の値 `b` の `1 + 2·tol` 倍までで止め、ほかの場所で確かめる: `b` の `1 + tol` 倍より上が出たら
「そこだけ」（止めを外して上げ続ける）、それだけ求めても出ない試行が `prov_n` 回、またはどちらとも
言えない試行が `prov_max` 回続いたら「全体の限界」（全体の `bad` にする）。確かめている間は、壁の近くで
上げ幅を小さくする扱いをしない（求める値が足りず、どちらとも言えないままになる）。


### 頭打ち（前兆として扱う）

短いコーナーでは舵が動き続け、曲率の不足が判定できないまま抜けることがある。そこで、同じ場所を
**前より高い上限で通ったのに、出た横加速度が増えていない**区間も前兆として記録する（減速は
しない）。これが無いと、検出できないコーナーで限界を数 % 超えたまま走り続け、検出できる
コーナーとの間で全体の値が上下して収束しなかった（sim.bench の実測、真値 0.42）。

## 前兆の量

`dev = 車速² ×（舵角から出る曲率 − 実曲率）`＝舵が要求した横加速度のうち出ていないぶん [m/s²]
（実曲率 = ヨーレート ÷ 車速）。100Hz で来て自己位置に依らない。舵角から出る曲率には同定した
ヨーの2次遅れを通す。**その場所の普段の値 `base` との差**が、しきい値を `dev_n` 回続けて超えたら前兆
（実機では実曲率÷舵角の曲率が横加速度の帯で 1.10〜1.00 と普段から動く）。

### しきい値は雑音より下げない（★実機で崩れた所）

しきい値は「その場所の上限の `dev_thr` 倍」と「**測った雑音の幅（`noise_quiet` を超えたぶん）の
`noise_k` 倍**」の大きい方。雑音の幅 `GripMap.noise` は、曲率の差（舵角から出る曲率 − 実曲率）から
ゆっくり動くぶんを引いた残りのばらつき [1/m] で、走っている間ずっと（学習を止めている間も直線でも）
測り、地図と一緒に保存する。実機で 0.035〜0.05、雑音を足していないシムで 0.009。横加速度に直すと
幅 × 車速²（実機の雑音はジャイロの揺れ ∝ 車速・舵の揺れ → 曲率で、車速² に比例する）。
雑音が `dev_thr` を上回るぶん、続けて超える回数も `dev_n_max` まで延ばす。

- 2026-10-12、しきい値を上限の 5%・40ms で固定していた版を実機に入れたら、**滑っていない車で
  前兆が 46 回**出て、横の上限が 3.9 → 1.0m/s²（下限）、加速・減速も半分になり、1 周 5.7s → 13.1s に
  なった。実機の走行中の雑音はシムの既定の 15〜25 倍（ジャイロの高周波 σ 4.5〜8°/s・舵のばたつき
  σ 0.016rad、約 11Hz）で、不足の 200ms 平均でも横加速度の 2 割近くまで振れる
- しきい値を上限の割合だけで決めると、**上限が下がるほどしきい値の絶対値も下がり、誤検出が増えて
  戻れなくなる**（上限 1.0 で 0.05m/s²＝雑音の中）。雑音の項は上限に依らない
- 雑音の小さいシムでは `dev_thr`・`dev_n` のまま（感度は変わらない）。実機の雑音では、即時の層が
  見るのは遅いコーナーで上限の 1 割、速いコーナーで数割の不足になる。数 % の超過は下の 2 つで拾う
- 確かめ方: `raspi/tools/grip_replay.py`（実機の記録を流し直す。滑っていない記録で前兆 0 が合格）と
  `sim.bench --real-noise`（実機で測った揺れをシムのテレメトリに足す）

### コーナー 1 つぶんで見る前兆（抜けた後に記録する。減速はしない）

- **平均の不足**: 試行の間（舵が落ち着いている所）の不足の平均が、上限の `mean_thr` 倍と、平均の
  雑音（幅 ÷ √独立な標本の数）の `mean_k` 倍の両方を超えた
- **横偏差の増え**: 試行の間の横偏差が、前兆なしで抜けた今までの同じ場所の値を `cross_gain` 以上
  上回った。速いコーナーで加速しながら外へ膨らむ形は曲率の不足にほとんど出ない
- 雑音が `noise_quiet` を超えているときは、1 回の上げ幅を `noisy_step` 倍にし、出ている横加速度を
  長めにならす（試行の「出た最大」が雑音の山を拾うと、前兆の値を消して上限が跳ね上がった）

### 崩れないための歯止め

- 横の上限は、出発点の `floor` 倍より下へは下げない
- 雑音が大きい（`noise_quiet` を超える）とき、「想定外」の前兆で加速・減速まで下げるのは、大きな
  不足（`cap_hard`）か横偏差の前兆だけ
- 誤って出た前兆の値を上へ見直す仕組みは**入れていない**（前兆なしで通り続けたら 3% ずつ上げる形を
  試したが、雑音なしのシムで前兆と超過が増えた——真値 4.12 に対して 3 回→6 回・横偏差 4→17cm）。
  誤った値が残ったら `.grip.json` を消す

- 比（実曲率÷舵角の曲率）そのものでは見ない。舵の切り返しで分母が 0 を通り、限界内でも ±1 まで
  振れた（sim.bench の実測。18 回の誤検出）。不足量なら曲率が小さい所で自然に小さくなる
- 舵が動いている間（要求の横加速度の変化が `steady` を超える）は判定しない。遅れのモデルの誤差が
  そのまま不足量に見える。ただし、はっきり大きい不足（上限の `gross` 以上）は舵が動いていても見る
- 補助: 経路からの横偏差（10Hz、自己位置に依る）が `cross_thr` を超えたら同じ扱い
- 舵が上限に張り付いている所・曲がり切れない区間・通常の周回以外は planner が `gate` で止める
- 加減速しながら出た前兆は、前後に使っていた力を合成した半径で記録する（横だけの限界と取り違えない）

## 加速・減速（`LongLearner`）— 出ない加速度を求めて、出た値を測る

横と違って場所ごとには持たず、コース全体で 1 つずつ（`GripMap.accel`・`decel`）。

**測り方**: プロファイルが加速（減速）を求め続けている間の車速を 0.15〜0.4 秒の窓に溜め、最小二乗の
傾き＝出た加速度を、求めた加速度と比べる。

- 求めたぶん出ている（97% 以上）→ 限界はそれ以上
- 出ていなくて、**車速が計画から遅れていき、傾きが一定**（窓の前半と後半で同じ）→ 路面で頭打ち。
  **出た値がそのまま限界**（上にも下にも、その 1 回で採る）
- どちらでもない（立ち上がりの途中・計画より先に居て追いつかれているだけ・区間の終わりで先に
  細った）→ 何もしない

「頭打ちで一定になった所だけを測る」のが要点。以前の形（区間の始めと終わりの速度²の変化を計画と
比べ、85%／95% のしきい値で「出た／出ない」を決める）は精度が 5〜10% で、1 割の不足に気づけなかった
（減速の見積もりが真値を 6〜11% 上回ったまま残った）。

- 見るのは、横加速度が横の限界の `lat_max`（0.5）未満の間だけ。摩擦円のぶん（`sqrt(1 − 割合²)`）で
  割って直線での値に直す
- 区間の初め `settle_s`・求めるのをやめる手前 `tail_s` は入れない。計画の速度が跳んだ窓は捨てる
- **車速に雑音があるとき**（窓の中の 2 階の差分から出した傾きの誤差が、求めた加速度の `se_free` 以上）は
  1 つの窓で決めない。減速は「今の値より強く求めた窓の、出た値」を誤差の重みで溜め、その平均から
  誤差の `z` 倍を引いた下の端まで上げる。加速は 出た÷求めた の下の端が `pass_lo` 以上で 1 段。
  「限界」と言うのは、出た値が誤差の `z` 倍を足しても足りないときだけ。実機の車速の雑音（σ 0.03〜
  0.05m/s）では 0.4 秒の窓の傾きが ±0.1〜0.3m/s² ずれ、1 つの窓で決めると、たまたま高く出た窓の
  たびに減速が 15% ずつ上がって真値を 17% 上回った（sim.bench `--real-noise`）。
  ★そのぶん上がるのは遅い（真値 2.56 に対して 170 秒で 1.91 → 2.2）
- 短い窓（0.2〜0.3 秒）の傾きをプロファイルの加速度と比べて「出ていない」を決める形は採らなかった
  （走り出し・追いつく途中・作り直した直後に取り違え、減速が下がり続けた）。今の形との違いは、
  「出ていない」と言うのに**遅れが増えていること・傾きが一定であること**を求める点と、比べるのでは
  なく**出た値そのものを採る**点

- **加速**: 求めたぶん出たら `step_up`（10%）上げる。頭打ちになったら、出た値の `acc_keep`（0.97）倍に
  して止める。そこで出続けていれば、限界を 3% ずつ上へ見直す（1 回の測り違いで低く決めたままにしない）
- 計画よりずっと遅れている間（前兆で減速した後・迂回の後の「追いつく途中」）は測らない。planner は
  計画の加速度ではなく、今の車速から追いつく速度を指令している
- **減速**: 見積もりが高すぎると、止まれない速度でコーナーに入る。だから**制動を始める位置は確かめた値
  のまま、制動区間の初めの 0.35 秒だけ `boost`（1.15）倍の減速を求める**（`raceline.probe_brake`。
  出なくても元のプロファイルより速くは入らない）。出たら出た値（の 0.97 倍）まで上げ、頭打ちになったら出た値を採る。
  近い値が 2 回そろったら「決まった」（`dec_done`）として、強く求めるのをやめる（時間がたっても
  測り直さない——確かめた値が真値に近いと、強く求めた区間で車が目標から遅れ、コーナーへ速く入る）
  - ふだんの制動で頭打ちになったら（路面が滑りやすくなった）、その区間のうちに下げて、また測る
  - 速度プロファイルには確かめた値の 0.95 倍を使う（planner の `LEARN_BRAKE_USE`）
- **路面が変わったとき**: 横の学習が「想定外」の前兆を見たら、加速・減速も同じ割合で先に下げて測り直す
  （`rescale`）。落ち着いていた加速がはっきり落ちたら、減速も同じ割合で下げる。**前後の低下から横を
  下げることはしない**（試したが、1 回の測り違いで横まで大きく下げてしまった）

## シムと実機の違い

シムの車は限界で曲率がいきなり頭打ちになり、既定では前兆の量の雑音も小さい（`sim.bench --real-noise`
で実機の揺れを足せる）。実機のタイヤは徐々に逃げる。**実機で本当に限界を超えた記録はまだ無い**ので、
「本物の前兆を見逃さない」ことは雑音を足したシムでしか確かめていない（`docs/development.md` §4.3）。
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field

import numpy as np

__all__ = ["CELL_M", "GripConfig", "GripMap", "GripLearner", "LaunchFit", "LongConfig", "LongLearner"]

#: 格子の刻み [m]
CELL_M = 0.25
#: 保存ファイルの形式の版（違う版は読まない）
FORMAT = 1
#: 読み込んだ値を丸める範囲 [m/s²]（壊れた値・古い形式で速度が跳ね上がらないように）
A_FLOOR = 1.0
A_CEIL = 9.0
#: 読み込むセルの数の上限
MAX_CELLS = 20000
#: 加速・減速の上限の下限 [m/s²]
X_FLOOR = 0.5
#: 雑音の幅（`GripMap.noise`）の上限 [1/m]（壊れた値・滑り続けた記録で判定が効かなくならないように）
NOISE_CEIL = 0.15


@dataclass
class GripConfig:
    #: 1回の試行で上げる割合（planner の設定 `grip_step` が上書きする）。シムでは、即時に下げる層を
    #: 入れた状態で初期値を真値より 4〜45% 高くしても衝突 0・横偏差 最大 11cm だった
    #: （`PROGRESS.md` 2026-10-11「横グリップのリアルタイム最適化」）。前兆のしきい値 `dev_thr` より
    #: 大きくしておく（小さいと、超えていても前兆に出ないまま走り続ける）
    step_up: float = 0.06
    #: 前兆が出た値・保存した値から再開するときに掛ける割合
    back_off: float = 0.92
    #: 前兆が出た値に対して残す余裕
    bad_margin: float = 0.02
    #: 挟み込みをやめる幅（`(bad − ok) / bad`）
    tol: float = 0.03
    #: 実際の横加速度が上限のこの割合以上まで出たら「試した」と数える
    trial_use: float = 0.88
    #: この割合を切ったら区間の終わり
    exit_use: float = 0.6
    exit_s: float = 0.15
    #: 前兆とみなす `dev − base`（その場所の上限に対する割合）と、続く回数（100Hz）。
    #: シムの実測: 限界内の p99.9 が 0.024〜0.046、限界を超えている間の中央値が 0.06〜0.076
    dev_thr: float = 0.05
    dev_n: int = 4
    #: 普段の値を持たない場所で使うしきい値の倍率
    dev_unknown: float = 1.5
    #: **しきい値は雑音より下げない**: 前兆とみなす不足は、測った雑音の幅（曲率の不足のばらつき
    #: `GripMap.noise` のうち `noise_quiet` を超えたぶん × 車速²）の `noise_k` 倍以上。雑音が
    #: `dev_thr` を上回るぶん、続く回数も `dev_n_max` まで延ばす（実機の雑音は約 11Hz で揺れる。
    #: 1 周期ぶん続けて超えるのは、平均がずれたときだけ）。雑音の小さいシムでは `dev_thr`・`dev_n`
    #: のまま。`noise_k` は実機の記録（`raspi/tests/data/grip_real_*.npz`、滑っていない）で、上限を
    #: 1.8〜5.5m/s² のどこに置いても前兆が 0 になる値（10 だと 1 回出る）
    noise_k: float = 12.0
    dev_n_max: int = 15
    #: 雑音の幅の出発点 [1/m] と、それを何標本ぶんの重みで持つか／ならす標本数（100Hz）。
    #: 出発点は実機の値（0.03〜0.05）の下の端: 測るまでの数秒を、雑音の無いシムの感度でも
    #: 実機の雑音でも通れるように
    noise_prior: float = 0.03
    noise_prior_n: int = 30
    noise_n: int = 500
    #: 雑音を測るとき、これよりゆっくり動くぶんは雑音に数えない [s]
    noise_slow_s: float = 0.3
    #: **雑音の幅のうち、これ以下のぶんは雑音に数えない** [1/m]（しきい値に効くのは超えたぶんだけ）。
    #: 雑音を足していないシムでも幅は 0.009 になる（舵を切る・戻すときの遅れのモデルのずれ。
    #: 揺れではないので、続けて超える回数の条件で落ちる）。これをそのまま `noise_k` 倍すると、
    #: 速いコーナーのしきい値が上限の 1〜2 割になり、路面が 2 割下がった直後のコーナーで前兆が
    #: 遅れた（sim.bench、線から 30cm。数えなければ 6〜8cm）。
    #: 雑音の幅がこれを超えていたら、1 回の上げ幅も `noisy_step` 倍にする。雑音があると
    #: 数 % の超過は 1 つのコーナーでは見分けられず、気づいた時の超過は「見分けられる幅＋上げ幅」
    #: になる（sim.bench `--real-noise`、上げ幅 6% で真値を 9% 超えた所で線から 16cm）
    noise_quiet: float = 0.015
    noisy_step: float = 0.5
    #: 出ている横加速度をならす時定数 [s]（ふだん／雑音が `noise_quiet` を超えているときに試行へ書く値）
    alat_tau: float = 0.05
    alat_tau_noisy: float = 0.15
    #: **コーナー 1 つぶんの平均で見る前兆**: 試行の間（舵が落ち着いている所）の不足の平均が、
    #: 上限の `mean_thr` 倍と、平均の雑音（幅 ÷ √独立な標本の数）の `mean_k` 倍の両方を超えたら、
    #: 抜けた後に前兆として記録する（減速はしない）。即時のしきい値は雑音の `noise_k` 倍まで
    #: 粗くなる（実機の速いコーナーでは上限の数割）ので、数 % の超過はここで拾う。
    #: `noise_corr_s` は雑音が独立とみなせる間隔 [s]（約 11Hz の揺れの 1 周期）、`mean_min_s` は
    #: 平均に要る時間 [s]
    mean_thr: float = 0.03
    mean_k: float = 3.0
    mean_min_s: float = 0.15
    noise_corr_s: float = 0.1
    #: **その場所の普段より横偏差が増えた**: 試行の間の横偏差の大きさ（セルごとの最大）が、前兆なしで
    #: 抜けた今までの試行の値を `cross_gain` [m] 以上上回ったら、抜けた後に前兆として記録する。
    #: 速いコーナーで加速しながら外へ膨らむ形は、曲率の不足にほとんど出ない（雑音のしきい値が
    #: 車速² に比例して上がる）うえ、`cross_thr`（12cm）に届く前に何周も 5〜9cm 膨らみ続けた
    #: （sim.bench `--real-noise`）。実機の記録では、同じ場所の周ごとの差は最大 2.6cm
    cross_gain: float = 0.04
    cross_gain_n: int = 2
    #: 舵が要求する横加速度の変化がこれを超える間は判定しない（上限に対する割合 / 秒）
    steady: float = 1.5
    #: 舵が動いている間に前兆とみなす不足（上限に対する割合）と、舵の速さ（上限に対する割合 / 秒）
    #: 1 あたりに足すぶん [s]
    gross: float = 0.25
    gross_rate: float = 0.04
    #: 経路からの横偏差がこれを超えたら前兆として扱う [m]（上限の `cross_use` 以上を使っている間）
    cross_thr: float = 0.12
    cross_use: float = 0.8
    #: 横偏差の前兆で記録する値の上限（求めていた横加速度に対する割合）
    cross_keep: float = 0.95
    #: 前兆の値が上限よりこれ以上低ければ「想定外」（路面が変わったと見て、全体を `back_off` 倍まで
    #: すぐ下げる）。探っている途中の前兆は、検出の遅れのぶん上限より 4〜8% 低い値で出る
    surprise: float = 0.10
    surprise_combo: float = 0.25
    #: 別の場所とみなす距離 [m] と、同じ値とみなす差
    place_m: float = 1.0
    consistent: float = 0.08
    #: 前兆が出たとき、車の周りのこの半径のセルへ書く [m]
    patch_m: float = 0.4
    #: 前兆の値を上限にした速度へ落とすときの余裕と、目標車速を下げる速さ（限界に対する割合）
    cap_margin: float = 0.03
    cap_decel: float = 0.35
    cap_max_s: float = 1.5
    #: 出ている横加速度が要求のこの割合を切っていたら、落とす速さを抑えない（大きく超えている。
    #: 路面が急に変わったときなど。緩く落としていると線から大きく外れる）
    cap_hard: float = 0.85
    #: 前後の加速度がこの割合を超えていたら「加減速しながらの前兆」
    combo: float = 0.2
    #: 壁との余裕がこれ未満の所は、上げ幅を余裕に比例して小さくする [m]（下限 `wall_min`）
    wall_full_m: float = 0.10
    wall_min: float = 0.3
    #: 上限が動かない試行がこれだけ続いたら収束とみなす
    stall_n: int = 8
    #: 1か所の前兆の値より上を求めても出ない試行がこれだけ続いたら、全体の限界とみなす
    prov_n: int = 2
    #: 同じく、どちらとも言えない試行（求める値が足りない）がこれだけ続いたら、全体の限界とみなす
    prov_max: int = 6
    #: これまでに出た最大より上を求めても出ない試行がこれだけ続いたら、その最大を全体の限界とみなす
    #: （前兆として検出できないまま限界を数 % 超えて走り続けない）
    short_n: int = 3
    #: 上限のこの割合以上まで出た試行は、上限そのものを「通った」と数える
    full_use: float = 0.97
    #: 頭打ち: 上限を `step_up` のこの割合以上上げて通ったのに、出た横加速度の伸びが上げ幅の
    #: `plateau_gain` 倍に届かないセルが `plateau_cells` 個以上（かつ伸びたセルより多い）
    plateau_ask: float = 0.5
    plateau_gain: float = 0.3
    plateau_cells: int = 2
    #: 横の上限を下げてよいのは、出発点のこの割合まで（測り違いが重なって下がり続けないための
    #: 歯止め。加速・減速の `LongConfig.floor` と同じ）
    floor: float = 0.5
    #: 発進の加速度から初期値を出すときの安全率
    launch_safety: float = 0.92
    #: True なら `step_up` を planner の設定で上書きしない（`sim.bench --grip-set step_up=…`）
    fixed_step: bool = False
    #: True なら周の境目でだけ上げる（収束の速さを比べる基準版。`sim.bench --grip-per-lap`）
    per_lap: bool = False


class _Cell:
    __slots__ = ("ok", "bad", "base", "n", "ap", "al", "cx", "cxn")

    def __init__(self, ok: float = 0.0, bad: float = 0.0, base: float = 0.0, n: int = 0) -> None:
        self.ok = ok            #: 通った横加速度の最大 [m/s²]（0 = 無い）
        self.bad = bad          #: 前兆が出た横加速度の最小 [m/s²]（0 = 無い）
        self.base = base        #: 前兆の量の普段の値
        self.n = n              #: `base` に入れた回数
        #: 直前に試行で通ったときの上限と、出た横加速度 [m/s²]（頭打ちの判定用。保存しない）
        self.ap = 0.0
        self.al = 0.0
        #: 前兆なしで抜けた試行での、経路からの横偏差の大きさの普段の値 [m] と、入れた回数（保存しない）
        self.cx = 0.0
        self.cxn = 0


def _cell(x: float, y: float) -> tuple[int, int]:
    return int(math.floor(x / CELL_M)), int(math.floor(y / CELL_M))


def _num(v, lo: float, hi: float) -> float:
    """有限な数なら `[lo, hi]` へ丸める。そうでなければ 0（= 無い）。"""
    try:
        f = float(v)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(f) or f <= 0.0:
        return 0.0
    return max(lo, min(hi, f))


@dataclass
class GripMap:
    """学習した路面の情報（モジュール docstring）。地図と一緒に保存する（`mapstore.save_grip`）。"""

    cells: dict[tuple[int, int], _Cell] = field(default_factory=dict)
    level: float = 0.0          #: 前兆が出ていない場所の限界の見積もり [m/s²]（0 = 未設定）
    ok: float = 0.0             #: どこかで通った横加速度の最大 [m/s²]
    bad: float = 0.0            #: 複数の場所で出た前兆の値 [m/s²]（0 = 無い）
    accel: float = 0.0          #: 加速の上限 [m/s²]（`LongLearner`。0 = 未設定）
    decel: float = 0.0          #: 減速の上限 [m/s²]（確かめた値。0 = 未設定）
    #: 曲率の不足（舵角から出る曲率 − 実曲率）の普段のばらつき [1/m]（標準偏差の見積もり。0 = 未測定）
    #: と、それに入れた標本の数。前兆のしきい値の下限になる（`GripConfig.noise_k`）
    noise: float = 0.0
    noise_n: int = 0

    def to_json(self) -> str:
        cells = [[ix, iy, round(c.ok, 3), round(c.bad, 3), round(c.base, 4), min(c.n, 50)]
                 for (ix, iy), c in sorted(self.cells.items()) if c.ok or c.bad or c.n]
        return json.dumps({"format": FORMAT, "cell_m": CELL_M, "level": round(self.level, 3),
                           "ok": round(self.ok, 3), "bad": round(self.bad, 3),
                           "accel": round(self.accel, 3), "decel": round(self.decel, 3),
                           "noise": round(self.noise, 5), "noise_n": min(self.noise_n, 100000),

                           "cells": cells})

    @classmethod
    def from_json(cls, text: str) -> "GripMap":
        """検査して読む。形式の違い・壊れた中身は `ValueError`。値は `[A_FLOOR, A_CEIL]` へ丸める。"""
        raw = json.loads(text)
        if not isinstance(raw, dict) or raw.get("format") != FORMAT:
            raise ValueError("grip: 形式の版が違う")
        if abs(float(raw.get("cell_m", 0.0)) - CELL_M) > 1e-9:
            raise ValueError("grip: 格子の刻みが違う")
        rows = raw.get("cells", [])
        if not isinstance(rows, list) or len(rows) > MAX_CELLS:
            raise ValueError("grip: セルの数が不正")
        m = cls(level=_num(raw.get("level"), A_FLOOR, A_CEIL),
                ok=_num(raw.get("ok"), A_FLOOR, A_CEIL), bad=_num(raw.get("bad"), A_FLOOR, A_CEIL),
                accel=_num(raw.get("accel"), X_FLOOR, A_CEIL),
                decel=_num(raw.get("decel"), X_FLOOR, A_CEIL),
                noise=_num(raw.get("noise"), 0.0, NOISE_CEIL))
        try:
            m.noise_n = max(0, min(100000, int(raw.get("noise_n", 0)))) if m.noise > 0.0 else 0
        except (TypeError, ValueError):
            m.noise_n = 0
        for r in rows:
            if not isinstance(r, list) or len(r) < 4:
                raise ValueError("grip: セルの形が不正")
            ix, iy = int(r[0]), int(r[1])
            if abs(ix) > 100000 or abs(iy) > 100000:
                raise ValueError("grip: セルの位置が不正")
            base = float(r[4]) if len(r) > 4 else 0.0
            n = int(r[5]) if len(r) > 5 else 0
            if not math.isfinite(base):
                base, n = 0.0, 0
            c = _Cell(_num(r[2], A_FLOOR, A_CEIL), _num(r[3], A_FLOOR, A_CEIL),
                      max(-0.5, min(0.5, base)), max(0, min(50, n)))
            if c.bad and c.ok >= c.bad:
                c.ok = 0.0                      # 前兆の値より上の「通った」は矛盾。捨てる
            m.cells[(ix, iy)] = c
        if m.bad and m.ok >= m.bad:
            m.ok = 0.0
        return m


class LaunchFit:
    """発進で TC が介入している間の加速度 [m/s²]（前後方向の限界）。

    車速を1点ずつ微分しない（前輪エンコーダは粗い）。舵がほぼ真っすぐで TC が介入し続けた区間
    ごとに直線を当てはめ（2 点しか無い区間は両端の差）、区間の長さで重みを付けて合計 `MIN_S` 秒ぶん
    集まったら値を出す。TC は断続的に介入するので、途切れをまたいで1本の直線にはしない（途切れの
    間は加速が落ちていて、傾きを低く見積もる）。シムでは介入が 0.01〜0.2 秒の細切れで、続いた
    0.3 秒を待つと最初のコーナーに間に合わなかった。傾きに転がり抵抗 `offset` を足す（タイヤが路面に
    伝えている力ぶんの加速度。傾きのままだと、横の限界との比が路面で変わってしまう）。
    """

    MIN_S = 0.3
    #: TC の旗がこれより長く途切れたら区間の終わり [s]
    GAP_S = 0.04
    #: 集めかけの区間を捨てるまでの時間 [s]（別の加速と混ぜない）
    STALE_S = 1.5
    STEER_MAX = 0.12

    def __init__(self, offset: float = 0.0) -> None:
        self.offset = offset                #: 転がり抵抗 [m/s²]（同定値）
        self.values: list[float] = []
        self._t: list[float] = []
        self._v: list[float] = []
        self._sum = 0.0                     #: Σ 傾き×長さ
        self._w = 0.0                       #: Σ 長さ [s]
        self._last = 0.0

    def feed(self, t: float, v: float, tc_active: bool, steer: float) -> float | None:
        """新しい値が出たらその値、そうでなければ `None`。"""
        on = tc_active and abs(steer) < self.STEER_MAX and v > 0.05
        if self._t and (not on or t - self._t[-1] > self.GAP_S):
            self._close()
        if self._w and t - self._last > self.STALE_S:
            self._sum = self._w = 0.0
        if on:
            self._t.append(t)
            self._v.append(v)
            self._last = t
            if self._w + (t - self._t[0]) >= self.MIN_S:
                self._close()
        if self._w < self.MIN_S:
            return None
        a = self._sum / self._w + self.offset
        self._sum = self._w = 0.0
        if not 0.3 < a < A_CEIL:
            return None
        self.values.append(a)
        return a

    def _close(self) -> None:
        """今の区間に直線を当てはめて、傾きを長さの重みで足す。"""
        ts, vs = self._t, self._v
        self._t, self._v = [], []
        d = ts[-1] - ts[0] if ts else 0.0
        if len(ts) < 2 or d <= 0.0:
            return
        if len(ts) >= 3:
            self._sum += float(np.polyfit(np.asarray(ts) - ts[0], np.asarray(vs), 1)[0]) * d
        else:
            self._sum += vs[-1] - vs[0]
        self._w += d

    @property
    def value(self) -> float | None:
        return float(np.median(self.values)) if self.values else None


@dataclass
class LongConfig:
    #: 加速: 求めたぶん出たら 1 回で上げる割合。高すぎても車が付いて来ないだけで、その区間で
    #: 出た値が測れる（モジュール docstring）
    step_up: float = 0.10
    #: 減速: 制動区間の初めに求める減速度（確かめた値に対する倍率。上限は `dec_max`）。**出ない値を
    #: 求めて、出た値を測る**ので、1 割ずつ試すより大きくしてある
    #: ★ 値が決まったら（`dec_done`）強く求めるのをやめ、時間がたっても測り直さない。確かめた値が
    #: 真値に近いと、強く求めた区間で車が「試す形」から 0.2m/s ほど遅れ、コーナーへ速く入って横の前兆が
    #: 出ることがある（sim.bench、60 秒ごとに測り直していた版で線から 18cm・横の値を 1 割低く誤った）。
    #: 測り直すのは、ふだんの制動で出なかったとき・横の学習が「路面が変わった」と見たとき・保存した値から
    #: 再開したときだけ。路面が良くなったことには気づかない（上げ直すには `.grip.json` を消す）
    boost: float = 1.15
    back_off: float = 0.92      #: 保存した値から再開するときに掛ける割合
    #: 求めた加速度に対して出た割合がこれ以上なら「出た」（＝限界はそれ以上）
    pass_ratio: float = 0.97
    #: プロファイルが上限のこの割合以上を求めている間を 1 つの区間とする
    zone: float = 0.5
    #: 窓を通して平均で上限のこの割合以上を求めていた窓だけ評価する
    demand: float = 0.85
    #: 横加速度が横の限界のこの割合未満の間だけ測る。摩擦円で前後に使える力が減るぶん
    #: （`sqrt(1 − 割合²)`）で割って直線での値に直すが、この補正はモデルに依る（シムの車は
    #: `1/sqrt(1 + 割合²)`）。0.5 なら補正は最大 13%・モデルの違いは最大 3%（窓の平均ではその半分ほど）。
    #: 0.35 にすると、コーナーの続くコース（toyota2）では測れる窓が 1 周に 1 つも無くなる
    lat_max: float = 0.5
    #: 減速を強く求める（`raceline.probe_brake`）のは、横がこの割合未満の所・落とす速度²のこの
    #: 割合まで・この時間 [s] 以上続けられる区間だけ
    probe_lat: float = 0.5
    probe_part: float = 0.6
    probe_min_s: float = 0.25
    #: 同じく、強く求める時間の上限 [s]（測るのに要るぶんだけ）
    probe_max_s: float = 0.35
    #: 同じく、強く求めた後に元のプロファイルへ戻す距離の時定数 [m]
    probe_ease_m: float = 1.2
    #: 窓の長さ [s]: これだけ溜まったら評価して次の窓へ／区間の終わりで評価する下限
    win_s: float = 0.4
    win_min_s: float = 0.15
    #: 区間の初めのこの時間は窓に入れない [s]（指令の遅れ・STM32 の傾きのローパス・車速のローパス）
    settle_s: float = 0.06
    #: 区間の終わりのこの時間も評価に入れない [s]。指令は少し先の場所の速度なので（`speedctl.ref_command`
    #: の前倒し）、区間が終わる手前で車の加速が先に細る——それを「出ていない」と取り違えた（sim.bench）
    tail_s: float = 0.08
    #: 窓の前半と後半の傾きの差がこの割合を超えたら捨てる（立ち上がりの途中。頭打ちになっていない）
    plateau: float = 0.15
    #: 「求めたのに出ていない」と言うのに要る、窓の間に増えた車速の遅れ [m/s]
    lag_min: float = 0.03
    #: 「計画に乗っている」とみなす車速の差 [m/s]。区間の最初の窓がこれより遅れて始まったら、
    #: 「出ていない」とは言わない
    lag_on: float = 0.3
    #: 加速: 測った限界に掛けて上限にする割合
    acc_keep: float = 0.97
    #: 減速: 求めたぶん出たとき、出た値に掛けて上限にする割合
    dec_keep: float = 0.97
    #: 加速: 測った限界のすぐ下でこれだけ続けて「出た」ら、限界を `acc_creep` だけ上へ見直す。1 回の
    #: 測り違いで低く決めたままにしない（走り出しの 1 回で真値 3.0 を 2.2 と測り、そのまま 150 秒
    #: 走った、sim.bench）。10% ずつ試し直す形にはしない（コーナーの出口で求めすぎになる）
    acc_creep_n: int = 6
    acc_creep: float = 0.03
    #: 減速: 続けて 2 回、この幅に収まる測定が出たら「決まった」（`dec_done`。強く求めるのをやめる。
    #: ラインの引き直しもこれを待つ）
    settle_tol: float = 0.04
    #: 上限を下げてよいのは、出発点のこの割合まで（測り違いが重なって下がり続けないための歯止め）
    floor: float = 0.5
    #: 学習値が変わってからこの時間は見ない [s]（速度プロファイルが作り直され、求める加速度が
    #: 窓の途中で変わる）
    quiet_s: float = 0.4
    #: 出た加速度が求めた値のこの割合に届かない窓は捨てる（路面の限界ではなく、追っていないだけ）
    sane_ratio: float = 0.4
    #: これより小さい変化では値を書き換えない（速度プロファイルを作り直し続けない）
    min_change: float = 0.01
    #: 計画の速度が 1 つの状態の間にこれ以上動いたら、窓を捨てる [m/s]（ふだんは 0.06 まで）
    jump: float = 0.15
    #: 落ち着いていた加速の上限がこの割合を切ったら「路面が変わった」（減速も同じ割合で先に下げる）
    road_drop: float = 0.8
    #: **車速の雑音があるとき**（窓の直線当てはめの残りから出した傾きの誤差が、求めた加速度の
    #: `se_free` 以上）は、1 つの窓で決めない。出た÷求めた を窓ごとに誤差の重みで溜め、その
    #: 平均から誤差の `z` 倍を引いた下の端で判断する:
    #: - 減速を上げる: 今の値より強く求めた窓の「出た値」を溜め、下の端（× `dec_keep`）が今の値を
    #:   `gain_min` 以上上回ったら、そこまで。求めた値までは上げない
    #: - 加速を上げる: 出た÷求めた の下の端が `pass_lo` 以上で 1 段
    #: - 「限界」と言うのは、その窓の出た値が誤差の `z` 倍を足しても足りないときだけ
    #: 実機の車速の雑音（σ 0.03〜0.05m/s、約 11Hz）では 0.4 秒の窓の傾きが ±0.2m/s²（約 8%）ずれる。
    #: 1 つの窓で決めると、たまたま高く出た窓のたびに減速が 15% ずつ上がり、真値を 17% 上回った
    #: （sim.bench `--real-noise`、真値 2.56 に対して 3.00。コーナーへ止まれない速度で入る）
    se_free: float = 0.015
    z: float = 1.5
    gain_min: float = 0.03
    pass_lo: float = 0.95
    #: 車速の雑音が独立とみなせる間隔 [s]（実機の車速は 2〜3 標本先で相関がほぼ消える）
    noise_corr_s: float = 0.03
    #: 雑音があるとき、TC が窓のこの割合以上で介入していたら、加速は路面で頭打ちとみなす
    tc_part: float = 0.5
    #: 区間の途中では、横の割合が `lat_max` のこの倍率まで続ける
    lat_hold: float = 1.2


class LongLearner:
    """加速・減速の上限を走行中に決める（モジュール docstring）。値は `GripMap.accel`・`decel`。"""

    def __init__(self, cfg: LongConfig | None = None) -> None:
        self.cfg = cfg or LongConfig()
        self.reset(GripMap())

    def reset(self, m: GripMap) -> None:
        self.map = m
        self.acc_max = self.dec_max = A_CEIL
        self.dec_use = 1.0
        self.down_only = False
        self.acc_done = self.dec_done = False
        self.ver = 0                        #: 値が変わるたびに進む
        self.ver_down = 0
        self.events: list[dict] = []        #: 値を下げた測定の記録（表示・ベンチ用）
        self.windows = 0                    #: 評価した窓の数
        self.skips: dict[str, int] = {}     #: 評価しなかった窓の数（理由ごと。診断用）
        self.acc_floor = self.dec_floor = X_FLOOR
        self._dec_init = 0.0
        self._dec_seen = False              #: 減速の窓を 1 つでも評価したか
        #: 窓ごとの (時刻, 向き, 求めた, 出た, 判定)（診断用。直近 2000）。判定は "出た"・"限界"・""
        self.log: list[tuple] = []
        self.acc_ceil = 0.0                 #: 測った加速の限界 [m/s²]（0 = まだ測っていない）
        self._acc_pass = 0                  #: 測った限界のすぐ下で続けて「出た」数
        self._dec_hist: list[float] = []    #: 直近の減速の測定 [m/s²]
        self._dec_done_t = 0.0              #: 減速が「決まった」時刻
        #: 今の窓 [向き, 時刻…, 車速…, 計画の速度…, 求めた加速度…, 摩擦円の割合…]
        self._win: list | None = None
        self._zone_sign = 0
        self._zone_t = 0.0                  #: 今の区間が始まった時刻
        self._zone_first = True             #: 今の区間で、まだ「計画に乗っている」窓を見ていない
        self._quiet_t = -1e9                #: 最後に学習値が変わった時刻
        self._now = 0.0
        #: 雑音があるときに溜める、向きごとの [Σ重み, Σ重み×値, Σ重み×求めた]（`LongConfig.se_free`。
        #: 値は、加速が 出た÷求めた、減速が 出た値 [m/s²]）
        self._ev: dict[int, list[float]] = {1: [0.0, 0.0, 0.0], -1: [0.0, 0.0, 0.0]}
        self.noisy = 0                      #: 雑音ありとして扱った窓の数（診断用）

    def configure(self, *, acc_init: float, dec_init: float, acc_max: float, dec_max: float,
                  down_only: bool, dec_use: float = 1.0) -> None:
        """:param dec_use: 速度プロファイルが減速に使う割合（確かめた値に対して。planner の
            `LEARN_BRAKE_USE`）。制動区間の初めに強く求めるのは、それより強く求められる間だけ"""
        m = self.map
        self.dec_use = dec_use
        self.acc_max = max(X_FLOOR, acc_max)
        self.dec_max = max(X_FLOOR, dec_max)
        self.down_only = down_only
        self.acc_floor = max(X_FLOOR, self.cfg.floor * acc_init)
        self.dec_floor = max(X_FLOOR, self.cfg.floor * dec_init)
        self._dec_init = dec_init
        if m.accel <= 0.0:
            m.accel = max(X_FLOOR, min(self.acc_max, acc_init))
            self._bump(True)
        if m.decel <= 0.0:
            m.decel = max(X_FLOOR, min(self.dec_max, dec_init))
            self._bump(True)
        if m.accel > self.acc_max or m.decel > self.dec_max:
            m.accel, m.decel = min(m.accel, self.acc_max), min(m.decel, self.dec_max)
            self._bump(True)

    def load(self) -> None:
        """保存した値から再開する（`reset` の後）。そのまま信じず `back_off` 倍から。"""
        m = self.map
        if m.accel > 0.0:
            m.accel = max(X_FLOOR, m.accel * self.cfg.back_off)
        if m.decel > 0.0:
            m.decel = max(X_FLOOR, m.decel * self.cfg.back_off)

    @property
    def probe(self) -> float:
        """制動区間の初めに求める減速度 [m/s²]（`raceline.probe_brake`）。"""
        return min(self.dec_max, self.map.decel * self.cfg.boost)

    @property
    def probing(self) -> bool:
        """確かめた値より強い減速を求めて測るか（「下げるだけ」のときは求めない）。"""
        return (not self.down_only and not self.dec_done
                and self.probe > self.dec_use * self.map.decel * 1.02)

    def rescale(self, r: float) -> None:
        """路面が変わった（横の限界が `r` 倍になった。横の学習が呼ぶ）。前後も同じ割合で下げ、測り直す。"""
        m = self.map
        self._ev = {1: [0.0, 0.0, 0.0], -1: [0.0, 0.0, 0.0]}
        if r < 1.0:
            m.accel = max(self.acc_floor, m.accel * r)
            m.decel = max(self.dec_floor, m.decel * r)
        self.acc_ceil = 0.0
        self._dec_hist = []
        self.dec_done = False
        self._bump(True)

    def recheck(self) -> None:
        """減速を測り直す（路面が変わったらしい。横の学習が呼ぶ）。"""
        if self.dec_done:
            self.dec_done = False
            self._dec_hist = self._dec_hist[-1:]
            self._bump(False)

    def observe(self, t: float, v: float, a_plan: float, lat_frac: float, gate: str,
                v_plan: float | None = None, tc: bool = False) -> None:
        """車両状態 1 つぶん。

        :param a_plan: 今の場所でプロファイルが求めている前後の加速度 [m/s²]（減速は負）
        :param lat_frac: 今の横加速度 ÷ その場所の横の限界
        :param v_plan: 今の場所のプロファイルの速度 [m/s]
        :param tc: STM32 の TC が介入しているか（`VehicleState.tc_active`）
        """
        cfg, m = self.cfg, self.map
        self._now = t
        if v_plan is None or t - self._quiet_t < cfg.quiet_s:
            gate = gate or "見ない"
        sign, f = 0, 1.0
        # 区間の途中では、横の割合が境目を少し超えても続ける（境目の近くで行き来して窓が細切れに
        # なると、どの窓も「短い」で評価されない。sim.bench `--real-noise`、真値 0.30 で加速の窓 0）
        lat_ok = lat_frac < cfg.lat_max * (cfg.lat_hold if self._zone_sign else 1.0)
        if not gate and v > 0.5:
            f = math.sqrt(1.0 - min(lat_frac, cfg.lat_max * cfg.lat_hold) ** 2)
            if a_plan >= cfg.zone * m.accel * f:
                sign = 1
            elif -a_plan >= cfg.zone * m.decel * f:
                sign = -1
        demand = sign                           # 横を見ない「求めている向き」
        if not lat_ok:
            sign = 0
        w = self._win
        if sign != self._zone_sign:
            # 区間が変わった（止められた・横を使い始めた・求める向きが変わった）。溜めた窓を評価する。
            # 求めるのをやめた所で終わったなら、終わりの `tail_s` を除く（横を使い始めて終わった
            # だけなら、求め続けているので除かない）
            self._zone_sign, self._zone_t, self._win = sign, t, None
            if w is not None and not gate:
                self._eval(t, w if demand == w[0] else self._cut(w, t - cfg.tail_s)[0])
            self._zone_first = True
            w = None
        if not sign or t - self._zone_t < cfg.settle_s:
            return
        if w is None:
            w = self._win = [sign, [], [], [], [], [], []]
        if w[3] and abs(v_plan - w[3][-1]) > cfg.jump:
            # 計画の速度が跳んだ（プロファイルが作り直された・自己位置が動いた）。窓を取り直す
            self._skip("計画が跳んだ")
            self._win = None
            self._zone_sign = 0
            return
        w[1].append(t)
        w[2].append(v)
        w[3].append(v_plan)
        w[4].append(abs(a_plan) / f)
        w[5].append(f)
        w[6].append(1.0 if tc else 0.0)
        if t - w[1][0] >= cfg.win_s + cfg.tail_s:
            # 窓が溜まった。終わりの `tail_s` は次の窓へ回す（そこで区間が終わるかもしれない）
            head, self._win = self._cut(w, t - cfg.tail_s)
            self._eval(t, head)

    @staticmethod
    def _cut(w: list, t_end: float) -> tuple[list, list]:
        """窓を時刻 `t_end` で前後に分ける。"""
        k = len(w[1])
        while k > 0 and w[1][k - 1] > t_end:
            k -= 1
        return [w[0]] + [x[:k] for x in w[1:]], [w[0]] + [x[k:] for x in w[1:]]

    def _eval(self, t: float, w: list, half: bool = False) -> None:
        """窓 1 つを評価する: 車速の傾き（最小二乗）＝出た加速度を、求めた加速度と比べる。

        - 求めたぶん出ている → 限界はそれ以上
        - 出ていなくて、車速が計画から離れていき、傾きが一定 → **路面で頭打ち。出た値が限界**
        - どちらでもない（立ち上がりの途中・追っていないだけ）→ 何もしない
        """
        cfg, m = self.cfg, self.map
        sign, ts, vs, vps, asks, fs, tcs = w
        n = len(ts)
        if n < 8 or ts[-1] - ts[0] < cfg.win_min_s:
            self._skip("短い")
            return
        tt = np.asarray(ts) - ts[0]
        vv = np.asarray(vs)
        slope = float(np.polyfit(tt, vv, 1)[0])
        got = sign * slope / (sum(fs) / n)      # 直線での値に直す
        ask = sum(asks) / n
        level = m.accel if sign > 0 else m.decel
        if ask < cfg.demand * level or got < cfg.sane_ratio * ask:
            self._skip("求めていない" if ask < cfg.demand * level else "追っていない")
            return
        # 傾きの誤差。車速の雑音の幅は 2 階の差分の中央値から出す（傾きが途中で変わった窓でも
        # 雑音と取り違えない。直線からの残りで測ると、頭打ちに入った折れ目を雑音に数えてしまう）。
        # 2 階の差分の幅 → 雑音の幅の換算 0.65 は実機の車速の自己相関（隣 0.4〜0.6）での値
        span = float(tt[-1])
        d2 = vv[2:] - 2.0 * vv[1:-1] + vv[:-2]
        sv = 0.65 * 1.4826 * float(np.median(np.abs(d2)))
        se = sv * math.sqrt(12.0 / max(2.0, span / cfg.noise_corr_s)) / max(span, 1e-6) / (sum(fs) / n)
        noisy = se >= cfg.se_free * ask
        h = n // 2
        s1 = float(np.polyfit(tt[:h], vv[:h], 1)[0])
        s2 = float(np.polyfit(tt[h:], vv[h:], 1)[0])
        # 半分ずつの傾きは、全体の傾きより誤差が約 2.8 倍大きい（長さ半分・標本半分）
        flat = abs(s1 - s2) <= cfg.plateau * abs(slope) + (cfg.z * 2.8 * math.sqrt(2.0) * se if noisy else 0.0)
        lag0, lag1 = sign * (vps[0] - vs[0]), sign * (vps[-1] - vs[-1])
        verdict = ""
        if (not flat and not half and got < cfg.pass_ratio * ask
                and ts[-1] - ts[h] >= cfg.win_min_s and abs(s2) < abs(s1)):
            # 窓の途中で傾きが落ちた（路面が変わった・頭打ちに入った）。後半だけで評価し直す
            self._eval(t, [sign] + [x[h:] for x in w[1:]], half=True)
            return
        bounded = False
        lag_min = cfg.lag_min
        if noisy:
            # 遅れは窓の両端の 1 点ずつではなく、当てはめた直線の両端で見る（1 点の車速は雑音で動く）
            lf = np.polyfit(tt, sign * (np.asarray(vps) - vv), 1)
            lag0, lag1 = float(lf[1]), float(lf[1] + lf[0] * span)
            lag_min += cfg.z * se * span        # 傾きの誤差のぶん、遅れの増えを多めに求める
        behind = (flat and lag1 >= lag_min and lag1 - lag0 >= lag_min
                  and (abs(lag0) <= cfg.lag_on or not self._zone_first))
        if noisy:
            self.noisy += 1
            ev = self._ev[sign]
            if sign > 0:
                # 加速: 出た÷求めた を溜める（求めたぶん出ているか）
                wgt = (ask / se) ** 2
                ev[0] += wgt
                ev[1] += wgt * got / ask
            elif ask >= m.decel * (1.0 + cfg.gain_min):
                # 減速: 今の値より強く求めた窓の、出た値を溜める（どこまで出るか）。ふだんの制動の窓
                # （求めた値が今の値以下）を混ぜると、平均が今の値に寄って上がらなくなる
                wgt = 1.0 / (se * se)
                ev[0] += wgt
                ev[1] += wgt * got
                ev[2] += wgt * ask
            lo = ev[1] / ev[0] - cfg.z / math.sqrt(ev[0]) if ev[0] > 0.0 else 0.0
            # 加速は、TC が窓の半分以上で介入していれば路面で頭打ち（車速の傾きの形に頼らない。
            # 雑音があると「遅れが増えていて傾きが一定」が言えず、高すぎる加速が下がらなかった）
            tc_lim = sign > 0 and sum(tcs) >= cfg.tc_part * n and got < cfg.pass_ratio * ask
            if tc_lim or (got + cfg.z * se < cfg.pass_ratio * ask and behind):
                verdict = "限界"
            elif sign > 0 and lo >= cfg.pass_lo:
                verdict = "出た"
            elif sign < 0 and lo * cfg.dec_keep > m.decel * (1.0 + cfg.gain_min):
                verdict = "出た"
                ask, got = ev[2] / ev[0], lo
                bounded = True                          # 上げるのは、下の端が示す値まで
            if verdict:
                self._ev[sign] = [0.0, 0.0, 0.0]
        elif got >= cfg.pass_ratio * ask:
            verdict = "出た"
        elif behind:
            # 車速が計画より遅れていて（減速なら速くて）、その遅れが増えている。計画より先に居る車が
            # 計画に追いつかれるだけの窓（遅れが負から 0 へ）は数えない——路面ではなく車速PIが抑えている。
            # 区間の初めから計画より大きく遅れている車（前兆で減速した後・迂回の後の「追いつく途中」）も
            # 数えない——planner は計画の加速度ではなく、今の車速から追いつく速度を指令している
            verdict = "限界"
        if verdict == "限界" or lag1 < cfg.lag_on:
            self._zone_first = False            # 計画に乗っていた区間の続きの窓は、遅れていてよい
        self.windows += 1
        self.last = (lag0, lag1, s1, s2)        # 診断用
        self.log.append((t, sign, ask, got, verdict))
        del self.log[:-2000]
        if verdict and sign > 0:
            self._accel(t, ask, got, verdict == "限界")
        elif verdict:
            self._decel(t, ask, got, verdict == "限界", bounded)

    def _skip(self, why: str) -> None:
        self.skips[why] = self.skips.get(why, 0) + 1

    def _accel(self, t: float, ask: float, got: float, limit: bool) -> None:
        cfg, m = self.cfg, self.map
        if limit:
            # 路面で頭打ち。出た値が限界
            new = max(self.acc_floor, min(self.acc_max, got * cfg.acc_keep))
            d = m.decel
            if not self._dec_seen:
                # 加速が路面で頭打ちなら、減速も同じ後輪で同じ路面。まだ一度も制動を見て
                # いないなら、同じ割合まで先に下げておく（最初のコーナーへ速すぎる速度で
                # 入らない。下げすぎなら、制動区間の初めの測定で戻る）
                d = max(self.dec_floor, self._dec_init * got / ask)
            elif self.acc_done and new < cfg.road_drop * m.accel:
                # 落ち着いていた加速の上限が、はっきり落ちた（路面が変わった）。減速も同じ割合で
                # 先に下げて、測り直す——次の制動で気づくのでは、そのコーナーに間に合わない
                d = max(self.dec_floor, m.decel * new / m.accel)
            if d < m.decel:
                m.decel = d
                self._dec_hist = []
                self.dec_done = False
                self._bump(True)
            self.acc_ceil = got
            self._acc_pass = 0
            self.acc_done = True
            if new < m.accel * (1.0 - cfg.min_change) or (
                    new > m.accel * (1.0 + cfg.min_change) and not self.down_only):
                if new < m.accel:
                    self.events.append({"t": t, "kind": "加速", "ask": ask, "got": got})
                down = new < m.accel
                m.accel = new
                self._bump(down)
            return
        if ask < 0.9 * m.accel or self.down_only:
            return
        top = self.acc_max
        if self.acc_ceil:
            top = min(top, self.acc_ceil * cfg.acc_keep)
        if m.accel >= top * (1.0 - cfg.min_change):
            # 測った限界（か設定の上限）のすぐ下で出ている
            self.acc_done = True
            if self.acc_ceil:
                self._acc_pass += 1
                if self._acc_pass >= cfg.acc_creep_n:
                    self._acc_pass = 0
                    self.acc_ceil *= 1.0 + cfg.acc_creep
            return
        m.accel = min(top, m.accel * (1.0 + cfg.step_up))
        self._bump(False)

    def _decel(self, t: float, ask: float, got: float, limit: bool, bounded: bool = False) -> None:
        """:param bounded: 雑音があるときの「出た」。求めた値までは上げず、`got`（溜めた窓の下の端）まで"""
        cfg, m = self.cfg, self.map
        self._dec_seen = True
        was = self.dec_done
        if not limit:
            # 求めたぶん出た。出た値（の少し下）までは出ると分かったので、そこまで上げる。求めた値では
            # なく出た値で上げる——窓には強く求めていない所も入るので、求めた値の平均で上げると 1 周に
            # 4% ずつしか上がらなかった（sim.bench、真値 3.0 に届くまで 150 秒）
            new = min(self.dec_max, got * cfg.dec_keep if bounded else max(ask, got * cfg.dec_keep))
            if new > m.decel * (1.0 + cfg.min_change) and not self.down_only:
                m.decel = new
                self._dec_hist = []
                self._bump(False)
            if m.decel >= self.dec_max * (1.0 - cfg.min_change) and ask >= 0.97 * self.dec_max:
                self._done(t, True)             # 設定の上限まで出ている
            return
        # 路面で頭打ち。出た値が限界。はっきり低い値はすぐ採る（止まれない速度で入り続けない）。
        # 近い値は直近 3 つの中央値にする（1 回の測り違いで動かさない）
        hist = self._dec_hist
        if got < m.decel * (1.0 - 0.03):
            hist[:] = [got]
            new = got
        else:
            hist.append(got)
            del hist[:-3]
            new = float(np.median(hist)) if len(hist) >= 2 else min(got, max(m.decel, got * 0.97))
        new = max(self.dec_floor, min(self.dec_max, new))
        if new < m.decel * (1.0 - cfg.min_change):
            self.events.append({"t": t, "kind": "減速", "ask": ask, "got": got})
            m.decel = new
            self._bump(True)
        elif new > m.decel * (1.0 + cfg.min_change) and not self.down_only:
            m.decel = new
            self._bump(False)
        self._done(t, len(hist) >= 2 and max(hist) - min(hist) <= cfg.settle_tol * new)
        if was and not self.dec_done:
            self._bump(False)                   # また強く求めて測る（プロファイルの形が変わる）

    def _done(self, t: float, done: bool) -> None:
        if done and not self.dec_done:
            self.dec_done = True
            self._dec_done_t = t
            self._bump(False)                   # 強く求めるのをやめる（プロファイルの形が変わる）
        elif not done:
            self.dec_done = False

    def _bump(self, down: bool) -> None:
        self.ver += 1
        if down:
            self.ver_down += 1
        self.note_change()

    def note_change(self) -> None:
        """学習値（横も含む）が変わった。今の窓を捨て、しばらく見ない。"""
        self._quiet_t = self._now
        self._win = None
        self._zone_sign = 0


class GripLearner:
    """走行中に `GripMap` を更新する（モジュール docstring）。planner が 100Hz で `observe` を呼ぶ。"""

    def __init__(self, yaw_wn: float = 0.0, yaw_zeta: float = 1.0,
                 cfg: GripConfig | None = None, rolling_resistance: float = 0.0) -> None:
        self.cfg = cfg or GripConfig()
        self.rolling_resistance = rolling_resistance
        #: 舵角から出る曲率に掛けるヨーの2次遅れ（同定値。固有振動数 [rad/s] と減衰比）。0 なら掛けない
        self.yaw_wn = max(0.0, yaw_wn)
        self.yaw_zeta = max(0.1, yaw_zeta)
        self.map = GripMap()
        self.long = LongLearner()
        self.reset()

    def reset(self, keep_map: bool = False) -> None:
        if not keep_map:
            self.map = GripMap()
        self.long.reset(self.map)
        #: 値が変わるたびに進む（速度プロファイルの作り直しの鍵）。`ver_down` は下げたときだけ
        self.ver = 0
        self.ver_down = 0
        self.max_level = A_CEIL
        self.floor = A_FLOOR                #: 横の上限の下限 [m/s²]（`configure`）
        self.down_only = False
        self.lat_use = 1.0
        #: 直近に通った `level`（壁に近い所はここから `level` へ向けて一部だけ上げる）
        self.conf = self.map.level
        self.converged = False
        self.launch = LaunchFit(self.rolling_resistance)
        self.launch_ratio = 0.0             #: 横の限界 ÷ 発進の加速度（車固有。0 = まだ無い）
        self._launch_used = False
        self.events: list[dict] = []        #: 前兆の記録（`reason`・ベンチ用）
        self.trials = 0                     #: 評価した試行の数
        self.passes = 0
        self.t_converged: float | None = None
        self._stall = 0
        #: 1か所で出た前兆 [値, x, y, 「それより上が出ない」試行の数, どちらとも言えない試行の数]
        #: （全体か局所かを確かめている間）
        self._prov: list | None = None
        #: 試行で出た横加速度の最大 [m/s²] と、それより上を求めても出なかった試行の数
        self._best = 0.0
        self._short = 0
        self._lap_pass = False
        self._lap_event = False
        # 100Hz の状態
        self._t = 0.0
        self._kexp = 0.0                    #: 舵角から出る曲率（遅れを通した後）
        self._kexp_prev = 0.0
        self._k2 = (0.0, 0.0)               #: ヨーの2次遅れの状態（値・傾き）
        self._kmeas = 0.0
        self._alat = 0.0
        self._alat_s = 0.0                  #: 同じく、長めにならしたもの（`GripConfig.alat_tau_noisy`）
        self._cross = 0.0                   #: 直近の横偏差 [m]（planner が 10Hz で渡す）
        self._dev_run = 0
        self._gross_run = 0
        self._nz_mean = 0.0                 #: 曲率の差のゆっくり動くぶん（`_noise_add`）
        self._base_all = 0.0
        self._base_n = 0
        self._seg: dict | None = None
        self._seg_low = 0.0
        self.cap: float | None = None       #: 前兆で抑えている目標速度 [m/s]（`None` = 抑えていない）
        self.cap_hard = False               #: 落とす速さを抑えない（`GripConfig.cap_hard`）
        self._cap_t = 0.0
        self._cap_low = 0.0
        #: 直近の `dev − base`（前兆の判定に使った量。しきい値の余裕を測るための診断）
        self.last_excess = 0.0
        self.last_thr = 0.0                 #: 同じく、そのときのしきい値
        #: 直近の試行の (平均の不足, そのしきい値)（上限に対する割合。診断用）
        self.last_mean = (0.0, 0.0)
        self.last_cross_gain = 0.0          #: 直近の試行の、その場所の普段の横偏差からの増え [m]

    # ── 設定と初期値 ──

    def configure(self, *, init_level: float, max_level: float, down_only: bool,
                  lat_use: float) -> None:
        """planner が毎周期渡す設定。`level` が未設定なら `init_level` から始める。"""
        self.max_level = max(A_FLOOR, min(A_CEIL, max_level))
        self.down_only = down_only
        self.lat_use = max(0.1, lat_use)
        self.floor = max(A_FLOOR, min(self.max_level, self.cfg.floor * init_level))
        if 0.0 < self.map.level < self.floor:
            self.map.level = self.floor     # 保存した値が歯止めより低い（以前の測り違い）
            self.conf = min(self.conf, self.map.level)
            self._bump(down=False)
        if self.map.level <= 0.0:
            self.map.level = self.conf = max(A_FLOOR, min(self.max_level, init_level))
            self._bump(down=True)
        elif self.map.level > self.max_level:
            self.map.level = self.max_level
            self.conf = min(self.conf, self.max_level)
            self._bump(down=True)

    def load(self, m: GripMap) -> None:
        """保存した値から再開する。**そのまま信じない**: 全体とセルの「通った値」を `back_off` 倍に
        してから始め、通れば挟み込みで元へ戻る（日が変わると路面は変わる）。前兆の値は残す。"""
        b = self.cfg.back_off
        for c in m.cells.values():
            c.ok *= b
        m.ok *= b
        if m.level > 0.0:
            m.level = max(A_FLOOR, m.level * b)
        self.map = m
        self.long.reset(m)
        self.long.load()
        self.conf = m.level
        self.converged = False
        self._stall = 0
        self._bump(down=True)

    def set_launch(self, a: float) -> None:
        """発進の加速度から横の初期値を出す（比が分かっているとき。まだ前兆も試行も無い間だけ）。"""
        if self.launch_ratio <= 0.0 or self._launch_used or self.events or self.passes:
            return
        self._launch_used = True
        g = max(A_FLOOR, min(self.max_level, a * self.launch_ratio * self.cfg.launch_safety))
        if g < self.map.level or not self.down_only:
            down = g < self.map.level
            self.map.level = self.conf = g
            self._bump(down=down)

    # ── 速度プロファイルへ渡す値 ──

    def _cell_level(self, c: _Cell | None) -> tuple[float, bool]:
        """セルの限界の見積もり [m/s²] と、そのセル自身の前兆で決まっているか。"""
        g = self.map.level
        if c is None or not c.bad:
            return g, False
        cfg = self.cfg
        lo = c.ok if 0.0 < c.ok < c.bad else c.bad * cfg.back_off
        tgt = 0.5 * (lo + c.bad) if (c.bad - lo) / c.bad > cfg.tol else lo
        own = min(c.bad * (1.0 - cfg.bad_margin), tgt)
        return (own, True) if own < g else (g, False)

    @property
    def use(self) -> float:
        """限界の見積もりの何割を求めるか: 探している間は 1（モジュール docstring）、それ以外は `lat_use`。"""
        return self.lat_use if self.converged or self.down_only else 1.0

    def level_at(self, x: float, y: float) -> float:
        return max(self.floor, self._cell_level(self.map.cells.get(_cell(x, y)))[0])

    def levels(self, xy: np.ndarray, wall: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray]:
        """経路の各点の `(求める横加速度, 限界の見積もり)` [m/s²]。

        :param wall: 各点での車体と壁の余裕 [m]。小さい所は、通った値 `conf` から `level` へ
            上げる幅を余裕に比例して小さくする（同じ超過でも耐えられる横偏差が場所で違う）
        """
        cfg = self.cfg
        cells = self.map.cells
        g, conf = self.map.level, min(self.conf, self.map.level)
        ix = np.floor(xy[:, 0] / CELL_M).astype(np.int64).tolist()
        iy = np.floor(xy[:, 1] / CELL_M).astype(np.int64).tolist()
        w = None
        if wall is not None and g > conf and self._prov is None:
            w = np.clip(np.asarray(wall) / cfg.wall_full_m, cfg.wall_min, 1.0).tolist()
        out = []
        for i in range(len(ix)):
            lv, own = self._cell_level(cells.get((ix[i], iy[i])))
            if not own and w is not None:
                lv = conf + w[i] * (g - conf)
            out.append(lv)
        lim = np.maximum(np.asarray(out, dtype=np.float64), self.floor)
        return self.use * lim, lim

    # ── 100Hz の観測 ──

    def observe(self, t: float, x: float, y: float, v: float, yaw_rate: float, kappa_steer: float,
                *, a_x: float = 0.0, gate: str = "", tc_active: bool = False,
                v_plan: float | None = None, long_gate: str = "") -> None:
        """車両状態 1 つぶん。

        :param kappa_steer: 今の実舵角から出る曲率 [1/m]（同定した舵の効きを通したもの）
        :param a_x: 今の場所で計画している前後の加速度 [m/s²]（速度プロファイルの傾き）
        :param v_plan: 今の場所のプロファイルの速度 [m/s]（加速・減速の学習が使う）
        :param gate: 空でなければ学習を止める理由（planner が決める）
        :param long_gate: 空でなければ、加速・減速の学習だけを止める理由（横は見る）
        """
        cfg = self.cfg
        dt = t - self._t
        if 0.0 <= dt < 1e-4:
            return                              # 同じ時刻の状態が続けて届いた（まとめて受信）。捨てる
        self._t = t
        if not 0.0 < dt < 0.1:
            # 間が空いた。遅れの状態を今の値から始める（0 から始めると、追いつくまでが不足に見える）
            self._kexp = self._kexp_prev = kappa_steer
            self._k2 = (kappa_steer, 0.0)
            self._kmeas = yaw_rate / v if v > 0.3 else 0.0
            self._dev_run = 0
            self._seg = None
            return
        # 舵角から出る曲率 → ヨーの2次遅れ → 実曲率と同じローパス
        k2, k2d = self._k2
        if self.yaw_wn > 1.0:
            n = max(1, int(math.ceil(dt * self.yaw_wn / 0.2)))
            h = dt / n
            for _ in range(n):
                k2d += (self.yaw_wn ** 2 * (kappa_steer - k2) - 2.0 * self.yaw_zeta * self.yaw_wn * k2d) * h
                k2 += k2d * h
        else:
            k2 = kappa_steer
        self._k2 = (k2, k2d)
        lp = 1.0 - math.exp(-dt / 0.03)
        self._kexp_prev = self._kexp
        self._kexp += (k2 - self._kexp) * lp
        kmeas = yaw_rate / v if v > 0.3 else 0.0
        self._kmeas += (kmeas - self._kmeas) * lp
        self._alat += (abs(v * yaw_rate) - self._alat) * (1.0 - math.exp(-dt / cfg.alat_tau))
        a = self._alat
        # 試行に「出た値」として書く横加速度。雑音が大きいときは長めにならしたものを使う: 最大を
        # 取ると雑音の山を拾う（ならさないと、実機相当の雑音・車速 3.9m/s で上限より 2 割高い値が
        # 「通った」に入り、前兆の値を消して上限を 19% 跳ね上げた。sim.bench `--real-noise`）。
        # 「上限いっぱいまで出たか」は速い方で見る（遅い方は短いコーナーで山に届かず、上限まで
        # 出ているのに「出ていない」となって、真値の 2 割下で上げるのをやめた）
        self._alat_s += (abs(v * yaw_rate) - self._alat_s) * (1.0 - math.exp(-dt / cfg.alat_tau_noisy))
        a_rec = self._alat_s if self.noise_sigma > cfg.noise_quiet else a

        if self.cap is not None and (t - self._cap_t > cfg.cap_max_s or self._cap_low > 0.3):
            self.cap = None
        key = _cell(x, y)
        cell = self.map.cells.get(key)
        applied = self.use * max(self.floor, self._cell_level(cell)[0])
        v2 = v * v
        rate = abs(self._kexp - self._kexp_prev) / dt * v2 / max(applied, 1e-6)
        if v > 0.8 and rate < cfg.steady and self.cap is None:
            # 雑音の幅は、学習を止めている間も直線でも測る（センサと車の性質で、走り方に依らない）
            self._noise_add(self._kexp - self._kmeas, dt)
        self.long.observe(t, v, a_x, a / max(self.level_at(x, y), 1e-6),
                          gate or long_gate or ("横の前兆" if self.cap is not None else ""), v_plan,
                          tc_active)
        if gate:
            self._dev_run = 0
            self._seg = None
            if self.cap is not None:
                self._cap_low += dt
            return

        use = a / max(applied, 1e-6)
        if self.cap is not None:
            self._cap_low = self._cap_low + dt if use < cfg.exit_use else 0.0

        # 舵が要求した横加速度のうち出ていないぶん（上限に対する割合）。舵が動いている間は見ない
        turning = v > 0.8 and self._kexp * self._kmeas > 0.0
        valid = turning and rate < cfg.steady
        dev = v2 * (abs(self._kexp) - abs(self._kmeas)) / max(applied, 1e-6) if turning else 0.0
        # 雑音で決まるしきい値の下限（上限に対する割合）と、続けて超える回数。雑音の幅は曲率で
        # 持つ（実機の雑音は車速² に比例する: ジャイロの揺れ ∝ 車速、舵の揺れ → 曲率）。上限が
        # 下がっても絶対値は変わらない——割合だけで決めると、下がるほど誤検出が増えて戻れなくなる
        nthr = cfg.noise_k * max(0.0, self.noise_sigma - cfg.noise_quiet) * v2 / max(applied, 1e-6)
        need = max(cfg.dev_n, min(cfg.dev_n_max, int(math.ceil(cfg.dev_n * nthr / cfg.dev_thr))))
        self.last_excess = 0.0
        ex: float | None = None             # 不足の、その場所の普段の値との差（見ていない周期は None）
        if turning and not valid and use > 0.5:
            # 舵が動いている間も、はっきり大きい不足だけは見る（切り込みながら限界を大きく超えている。
            # 路面が急に変わった後、定常になるのを待つと 0.5 秒遅れた）。遅れのモデルの誤差は舵の
            # 速さに比例するので、そのぶんしきい値を上げる
            if dev > max(cfg.gross, 2.0 * nthr) + cfg.gross_rate * rate:
                self._gross_run += 1
                if self._gross_run >= need:
                    self._gross_run = 0
                    ratio = abs(self._kmeas) / max(abs(self._kexp), 1e-6)
                    self._precursor(t, x, y, v, a, a_x, ratio, applied, tc_active, dev)
            else:
                self._gross_run = 0
            dev = 0.0
        else:
            self._gross_run = 0
        if valid and use > 0.5:
            known = cell is not None and cell.n > 0
            base = cell.base if known else self._base_all
            thr = max(cfg.dev_thr * (1.0 if known else cfg.dev_unknown), nthr)
            self.last_excess = (dev - base) / (1.0 if known else cfg.dev_unknown)
            self.last_thr = thr
            ex = dev - base
            if dev - base > thr:
                self._dev_run += 1
            else:
                self._dev_run = 0
                if dev - base < 0.5 * thr:
                    # 普段の値。上限を上げるにつれて少しずつ動くぶんには付いていく（実機の
                    # アンダーステアは横加速度とともに増える）が、しきい値に近い値は入れない
                    if cell is None:
                        cell = self.map.cells[key] = _Cell()
                    cell.n += 1
                    cell.base += (dev - cell.base) / min(cell.n, 30)
                    self._base_n += 1
                    self._base_all += (dev - self._base_all) / min(self._base_n, 300)
            if self._dev_run >= need:
                ratio = abs(self._kmeas) / max(abs(self._kexp), 1e-6)
                self._precursor(t, x, y, v, a, a_x, ratio, applied, tc_active, dev - base)
        else:
            self._dev_run = 0
        if abs(self._cross) > cfg.cross_thr and use > cfg.cross_use and v > 0.8:
            # 補助: 横偏差（自己位置に依る・10Hz）。曲率の不足に出ないまま外へ出ている
            self._precursor(t, x, y, v, a, a_x, 0.95, applied, tc_active, 0.0, kind="横偏差")

        # コーナー1つ＝1回の試行
        seg = self._seg
        if use >= cfg.trial_use:
            if seg is None:
                seg = self._seg = {"cells": {}, "fail": False, "peak": 0.0, "top": 0.0, "lv": 0.0,
                                   "ex": 0.0, "ex_t": 0.0, "ex_v2": 0.0, "ex_ap": 0.0, "cx": {}}
            self._seg_low = 0.0
            seg["cx"][key] = max(seg["cx"].get(key, 0.0), abs(self._cross))
            if ex is not None:
                # コーナー 1 つぶんの平均（`_end_trial`）: 不足 [m/s²]・時間・車速²・上限
                seg["ex"] += ex * applied * dt
                seg["ex_t"] += dt
                seg["ex_v2"] += v2 * dt
                seg["ex_ap"] += applied * dt
            lv, own = self._cell_level(cell)
            if a_rec > seg["cells"].get(key, (0.0, 0.0))[0]:
                seg["cells"][key] = (a_rec, applied, own)
                if a_rec > seg["peak"]:
                    seg["at"] = (x, y)
            seg["peak"] = max(seg["peak"], a_rec)
            seg["top"] = max(seg["top"], a)
            if not own:
                seg["lv"] = max(seg["lv"], lv)
        elif seg is not None:
            if use < cfg.exit_use:
                self._seg_low += dt
            if self._seg_low >= cfg.exit_s:
                self._seg = None
                self._end_trial(t, seg)

    @property
    def step(self) -> float:
        """1 回の試行で上げる割合（雑音が大きいときは `noisy_step` 倍）。"""
        cfg = self.cfg
        return cfg.step_up * (cfg.noisy_step if self.noise_sigma > cfg.noise_quiet else 1.0)

    @property
    def noise_sigma(self) -> float:
        """曲率の不足の普段のばらつき [1/m]（まだ測っていなければ出発点 `noise_prior`）。"""
        m = self.map
        return m.noise if m.noise_n > 0 else self.cfg.noise_prior

    def _noise_add(self, d: float, dt: float) -> None:
        """雑音の幅を更新する。

        :param d: 舵角から出る曲率 − 実曲率 [1/m]（符号つき）

        ゆっくり動くぶん（`noise_slow_s` でならした値。舵の効きのずれ・アンダーステア）を引いた
        残りの大きさをならす。大きく外れた値は今の幅の 3 倍までで頭を打たせる（本当に滑って
        いる間の値で広げすぎない。前兆で減速している間は呼ばれない）。
        """
        cfg, m = self.cfg, self.map
        if m.noise_n <= 0:
            m.noise, m.noise_n = cfg.noise_prior, cfg.noise_prior_n
        self._nz_mean += (d - self._nz_mean) * (1.0 - math.exp(-dt / cfg.noise_slow_s))
        m.noise_n += 1
        # 平均絶対偏差 × 1.2533 = 正規分布なら標準偏差
        r = min(abs(d - self._nz_mean), 3.0 * m.noise + 1e-4)
        m.noise += (1.2533 * r - m.noise) / min(m.noise_n, cfg.noise_n)
        m.noise = min(m.noise, NOISE_CEIL)

    def observe_cross(self, cross: float) -> None:
        """経路からの横偏差 [m]（planner が計画の周期で渡す。補助の前兆）。"""
        self._cross = cross

    def _precursor(self, t: float, x: float, y: float, v: float, a: float, a_x: float,
                   ratio: float, applied: float, tc_active: bool, excess: float,
                   kind: str = "") -> None:
        """前兆が出た。目標速度をすぐ落とし、その場所の値を下げる。

        :param ratio: 実曲率 ÷ 舵が要求した曲率（出ている横加速度の割合）
        """
        cfg = self.cfg
        # 曲率の不足ぶんだけ横加速度が出ていない。出ている横加速度で回れる速度まで落とす
        hard = ratio < cfg.cap_hard
        ratio = max(0.5, min(1.0, ratio))
        cap = v * math.sqrt(ratio) * (1.0 - cfg.cap_margin)
        self._cap_t = t
        self._cap_low = 0.0
        if self.cap is not None:
            self.cap = min(self.cap, cap)
            self.cap_hard = self.cap_hard or hard
            return                              # 同じ前兆の続き。値は最初の1回だけ書く
        self.cap = cap
        self.cap_hard = hard
        if hard:
            # 大きく超えている（路面が急に変わった）。ならした横加速度 `a` は変わる前の値を引きずって
            # いて、そのまま採ると新しい限界を 3 割高く見積もった（sim.bench、4 割の低下。次のコーナーへ
            # 止まれない速度で入り、線から 0.6〜1m 外れた）。今出ている横加速度（車速² × 実曲率）を採る
            a = min(a, v * v * abs(self._kmeas))
        # 加減速しながら出た前兆は、前後に使っていた力を合成した半径で記録する
        combo = abs(a_x) > cfg.combo * max(a, 1e-6)
        b = max(self.floor, math.hypot(a, a_x) if combo else a)
        if kind == "横偏差":
            # 線から `cross_thr` 外れた＝求めていた横加速度は出ていない。雑音で高く出た横加速度を
            # そのまま採ると値がほとんど下がらず、同じ外れ方を繰り返した（sim.bench `--real-noise`、
            # 真値より 18% 高い出発点から 14・21・17cm と 3 回）。求めていた値の `cross_keep` 倍までとし、
            # 想定外として扱う（さらに `back_off` 倍・加速と減速も下げる）
            b = max(self.floor, min(b, cfg.cross_keep * applied))
        # 「想定外」で加速・減速まで下げてよいか: 雑音が小さければ前兆そのものを信じる。雑音が大きい
        # ときは、大きな不足（`cap_hard`）か横偏差の前兆に限る
        corroborated = hard or kind == "横偏差" or self.noise_sigma <= cfg.noise_quiet
        kind = kind or (("加速＋横" if a_x > 0 else "減速＋横") if combo else "横")
        # 「想定外」は横だけで出た前兆に限る（加減速しながらの前兆は、前後の見積もりのずれでも出る）
        # ……が、合成した半径でも大きく下回っていれば（`surprise_combo`）、路面が変わったと見る
        surprise = (kind == "横偏差"
                    or b < (1.0 - (cfg.surprise_combo if combo else cfg.surprise)) * applied)
        m = self.map
        r = int(math.ceil(cfg.patch_m / CELL_M))
        cx, cy = _cell(x, y)
        for ix in range(cx - r, cx + r + 1):
            for iy in range(cy - r, cy + r + 1):
                if math.hypot((ix - cx) * CELL_M, (iy - cy) * CELL_M) > cfg.patch_m:
                    continue
                c = m.cells.get((ix, iy))
                if c is None:
                    c = m.cells[(ix, iy)] = _Cell()
                c.bad = b if not c.bad else min(c.bad, b)
                if c.ok >= c.bad:
                    c.ok = 0.0
        if self._seg is not None:
            self._seg["fail"] = True
        old_level = m.level
        self._lap_event = True
        self._share(x, y, b)
        # 全体の値を、出ていた横加速度まで下げる（それより上を別の場所で試すのは、そこで通ってから）。
        # 想定よりずっと低い所で出たなら、路面が変わったと見てさらに `back_off` 倍まで下げる。
        # 局所だったなら、ほかの場所を通るたびに `step_up` ずつ戻る
        m.level = min(m.level, max(self.floor, b * (cfg.back_off if surprise else 1.0)))
        self.conf = min(self.conf, m.level)
        self._best = min(self._best, b)
        self._short = 0
        if surprise or not m.bad:
            self.converged = False              # 路面が変わった／上限いっぱいで止まっていた。探し直す
            self._stall = 0
        if surprise:
            self._best = 0.0
            if corroborated:
                # 路面が変わった。前後の限界も同じ割合で先に下げて、測り直す（次の制動が間に合う
                # ように）。雑音が大きいときに小さな不足 1 回で前後まで下げると、測り違いのたびに
                # 3 つとも下がっていく（実機、2026-10-12: 滑っていない車で加速 2.66→1.33・
                # 減速 1.91→0.95）。雑音が小さいときまで絞ると、路面が 2 割下がった直後の制動が
                # 間に合わない（sim.bench: 次のコーナーへ 7 割速く入り、線から 23cm）
                self.long.rescale(m.level / max(old_level, 1e-6))
            else:
                self.long.recheck()
        self._relevel(t)
        self.events.append({"t": t, "x": x, "y": y, "a": a, "a_x": a_x, "b": b, "kind": kind,
                            "applied": applied, "surprise": surprise, "tc": tc_active,
                            "excess": excess})
        self._bump(down=True)

    def _end_trial(self, t: float, seg: dict) -> None:
        self.trials += 1
        if seg["fail"]:
            return
        self.passes += 1
        m = self.map
        cfg = self.cfg
        flat: list[_Cell] = []
        grew = 0
        # 速度プロファイルに効く値が変わったか（変わらなければ版を進めない——進めると planner が
        # プロファイルを作り直し、加速・減速の測定も毎コーナー止まる）
        sig = (m.level, self.conf, m.bad, self._prov is None, self.converged)
        cells_changed = False
        flat_own = True
        for key, (a, ap, own) in seg["cells"].items():
            c = m.cells.get(key)
            if c is None:
                c = m.cells[key] = _Cell()
            if c.bad and a > c.bad * (1.0 + cfg.tol):
                c.bad = 0.0                     # 前兆の値より上で通った。前兆の値が古い
                cells_changed = True
            if a > c.ok and (not c.bad or a < c.bad):
                c.ok = a
                cells_changed = cells_changed or bool(c.bad)
            # 頭打ち: 前より高い上限で通ったのに、出た横加速度が増えていない
            if c.ap > 0.0 and ap >= c.ap * (1.0 + cfg.plateau_ask * self.step):
                if a - c.al < cfg.plateau_gain * (ap / c.ap - 1.0) * c.al:
                    flat.append(c)
                    flat_own = flat_own and own
                else:
                    grew += 1
            c.ap, c.al = ap, a
        # 前兆の値は周り `patch_m` のセルにも書いてある（`_precursor`）が、コーナーの入口などは
        # 上限いっぱいを使わないので試行に入らず、自分では見直されない。通ったセルの隣の、試行に
        # 入らなかったセルは、隣の通ったセル（前兆の値がいちばん低いもの）の値に合わせる
        r = int(math.ceil(cfg.patch_m / CELL_M))
        near: dict[tuple[int, int], tuple[float, float]] = {}
        for (cx, cy) in seg["cells"]:
            c = m.cells[(cx, cy)]
            b0 = (c.bad or math.inf, c.ok)
            for ix in range(-r, r + 1):
                for iy in range(-r, r + 1):
                    k2 = (cx + ix, cy + iy)
                    if k2 in seg["cells"] or math.hypot(ix, iy) * CELL_M > cfg.patch_m:
                        continue
                    c2 = m.cells.get(k2)
                    if c2 is not None and c2.bad:
                        near[k2] = min(near.get(k2, (math.inf, 0.0)), b0)
        for k2, (b0, ok0) in near.items():
            c2 = m.cells[k2]
            b0 = 0.0 if math.isinf(b0) else b0
            ok0 = ok0 if 0.0 < ok0 < b0 else 0.0
            if abs(b0 - c2.bad) > 1e-6 * c2.bad or abs(ok0 - c2.ok) > 1e-6:
                c2.bad, c2.ok = b0, ok0
                cells_changed = True
        peak = seg["peak"]
        if len(flat) >= max(cfg.plateau_cells, grew + 1):
            self._plateau(t, seg, flat, local=flat_own)
            return
        # その場所の普段より横偏差が増えた（`GripConfig.cross_gain`）
        gain = max((cx - m.cells[k].cx for k, cx in seg["cx"].items()
                    if m.cells[k].cxn >= cfg.cross_gain_n), default=0.0)
        self.last_cross_gain = gain
        if gain > cfg.cross_gain:
            ap = max(ap for _, ap, _ in seg["cells"].values())
            self._plateau(t, seg, self._patch(seg["cells"]), b=min(peak, ap / (1.0 + self.step)),
                          kind="横偏差の増え", excess=gain, local=seg["lv"] <= 0.0)
            return
        for k, cx in seg["cx"].items():
            c = m.cells[k]
            c.cxn += 1
            c.cx += (cx - c.cx) / min(c.cxn, 5)
        w = seg["ex_t"]
        if w >= cfg.mean_min_s:
            # コーナー 1 つぶんの平均の不足（`GripConfig.mean_thr`）
            mean, ap = seg["ex"] / w, seg["ex_ap"] / w
            nz = (max(0.0, self.noise_sigma - cfg.noise_quiet) * seg["ex_v2"] / w
                  / math.sqrt(max(1.0, w / cfg.noise_corr_s)))
            self.last_mean = (mean / max(ap, 1e-6), max(cfg.mean_thr, cfg.mean_k * nz / max(ap, 1e-6)))
            if mean > max(cfg.mean_thr * ap, cfg.mean_k * nz):
                self._plateau(t, seg, self._patch(seg["cells"]), b=min(peak, ap - mean), kind="平均の不足",
                              excess=mean / max(ap, 1e-6), local=seg["lv"] <= 0.0)
                return
        if m.bad and peak > m.bad * (1.0 + cfg.tol):
            m.bad = 0.0
        if peak > m.ok and (not m.bad or peak < m.bad):
            m.ok = peak
        pv = self._prov
        if pv is not None and not m.bad:
            x, y = seg.get("at", (pv[1], pv[2]))
            if math.hypot(x - pv[1], y - pv[2]) > cfg.place_m and seg["lv"] > 0.0:
                if peak >= pv[0] * (1.0 + cfg.tol):
                    self._prov = None           # 別の場所ではもっと出た。あの前兆は局所
                else:
                    if self.use * seg["lv"] >= pv[0] * (1.0 + 1.3 * cfg.tol):
                        pv[3] += 1              # 求めたのに出ない
                    else:
                        pv[4] += 1              # 求める値が足りず、どちらとも言えない
                    if pv[3] >= cfg.prov_n or pv[4] >= cfg.prov_max:
                        m.bad = pv[0]
                        self._prov = None
        if seg["lv"] > 0.0 and not m.bad:
            ask = self.use * min(seg["lv"], m.level)
            if peak > self._best * 1.005:
                self._best, self._short = peak, 0
            elif ask >= self._best * (1.0 + cfg.tol):      # 追従の誤差（2% 前後）とは区別できる差
                self._short += 1
                if self._short >= cfg.short_n:
                    m.bad = self._best
                    m.ok = m.bad * (1.0 - 0.9 * cfg.tol)    # 幅は詰まっている（すぐ下で止める）
                    self._prov = None
        if seg["lv"] > 0.0:
            # 通ったと数えるのは、実際に出た横加速度まで（上限だけ上がって車が付いて来ていない
            # ＝気づかないまま限界を超えているとき、そこから先へ上げない）
            lv = min(seg["lv"], m.level)
            if seg["top"] < cfg.full_use * self.use * lv:
                lv = seg["top"] / self.use
            self.conf = max(self.conf, lv)
            self._lap_pass = True
        before = (m.level, self.ver)
        if not self.cfg.per_lap:
            self._relevel(t, raise_ok=seg["lv"] > 0.0)
        if not cfg.per_lap:
            self._stall = 0 if abs(m.level - before[0]) > 0.005 * before[0] else self._stall + 1
            if not self.converged and self._stall >= cfg.stall_n and not self.down_only:
                # 上限を上げても出る横加速度が増えない。実際に出た値を限界として止める
                if not m.bad and self.conf < m.level:
                    m.level = self.conf
                self._set_converged(t)
        now = (m.level, self.conf, m.bad, self._prov is None, self.converged)
        if now != sig or cells_changed:
            self._bump(down=m.level < before[0] or now[1] < sig[1] or cells_changed)

    def _patch(self, keys) -> list[_Cell]:
        """`keys` のセルと、その周り `patch_m` のセル（即時の前兆と同じ範囲。コーナーの入口のセルは
        まだ上限いっぱいを使っておらず、試行のセルに入っていない）。無ければ作る。"""
        r = int(math.ceil(self.cfg.patch_m / CELL_M))
        out = {(cx + ix, cy + iy) for cx, cy in keys
               for ix in range(-r, r + 1) for iy in range(-r, r + 1)
               if math.hypot(ix, iy) * CELL_M <= self.cfg.patch_m}
        return [self.map.cells.setdefault(k, _Cell()) for k in out]

    def _plateau(self, t: float, seg: dict, flat: list[_Cell], *, b: float | None = None,
                 kind: str = "頭打ち", excess: float = 0.0, local: bool = False) -> None:
        """頭打ちの区間（モジュール docstring）。前兆として値を書くが、通り終えているので減速はしない。

        :param b: 記録する値 [m/s²]（既定は試行で出た横加速度の最大）
        :param local: その場所だけの値（セルの前兆の値）で走っていた区間。全体の値は動かさない
            （全体まで下げると、ほかのコーナーをその低い値から探し直すことになる）
        """
        m, cfg = self.map, self.cfg
        b = max(self.floor, seg["peak"] if b is None else b)
        for c in flat:
            c.bad = b if not c.bad else min(c.bad, b)
            if c.ok >= c.bad:
                c.ok = 0.0
        x, y = seg.get("at", (0.0, 0.0))
        self._lap_event = True
        applied = self.use * seg["lv"]
        if local:
            applied = max((ap for _, ap, _ in seg["cells"].values()), default=0.0)
        else:
            self._share(x, y, b)
            m.level = min(m.level, b)
            self.conf = min(self.conf, m.level)
            self._relevel(t)
        self.events.append({"t": t, "x": x, "y": y, "a": b, "a_x": 0.0, "b": b, "kind": kind,
                            "applied": applied, "surprise": False, "tc": False,
                            "excess": excess})
        self._bump(down=True)

    def _share(self, x: float, y: float, b: float) -> None:
        """離れた別の場所でも近い値で前兆が出ていたら、全体の値にする。古くなった全体の値は捨てる。"""
        m, cfg = self.map, self.cfg
        if m.ok >= b:
            m.ok = 0.0                          # 通ったはずの値で前兆が出た。古い（路面が変わった）
        if m.bad and b < m.bad * (1.0 - cfg.consistent):
            m.bad = 0.0                         # 全体の値よりずっと低い。全体の値は古い
        for e in reversed(self.events):
            if (math.hypot(e["x"] - x, e["y"] - y) > cfg.place_m
                    and abs(e["b"] - b) <= cfg.consistent * b):
                nb = min(e["b"], b)
                if not m.bad or nb < m.bad:
                    m.bad = nb
                break
        self._prov = None if m.bad else [b, x, y, 0, 0]

    def lap(self, t: float) -> None:
        """1周した（基準版 `per_lap` はここでだけ上げる）。"""
        if self.cfg.per_lap and self._lap_pass and not self._lap_event:
            lv = self.map.level
            self._relevel(t, raise_ok=True)
            self._bump(down=self.map.level < lv)
        self._lap_pass = self._lap_event = False

    def _relevel(self, t: float, raise_ok: bool = False) -> None:
        """全体の `level` を決め直す: 前兆の値が無ければ `step_up` ずつ上げ、あれば `ok` との間を詰める。"""
        m, cfg = self.map, self.cfg
        if m.bad:
            lo = m.ok if 0.0 < m.ok < m.bad else m.bad * cfg.back_off
            gap = (m.bad - lo) / m.bad
            tgt = 0.5 * (lo + m.bad) if gap > cfg.tol else lo
            g = min(m.bad * (1.0 - cfg.bad_margin), tgt, self.max_level)
            if self.down_only:
                g = min(g, m.level)
            m.level = max(self.floor, g)
            self.conf = min(self.conf, m.level)
            if gap <= cfg.tol:
                self._set_converged(t)
            else:
                self.converged = False
            return
        if raise_ok and not self.down_only and not self.converged:
            g = min(self.max_level, max(m.level, self.conf * (1.0 + self.step)))
            if self._prov is not None:
                g = min(g, max(m.level, self._prov[0] * (1.0 + 2.0 * cfg.tol)))
            m.level = g
            if self.conf >= self.max_level - 1e-6:
                self._set_converged(t)          # 設定の上限まで通った

    def _set_converged(self, t: float) -> None:
        if not self.converged:
            self.converged = True
            self.t_converged = t
            if self.lat_use < 1.0:
                self._bump(down=True)           # ここから `lat_use` 倍で走る（求める値が下がる）
            else:
                self.ver += 1                   # 値は変わらない（状態の保存だけ進める）
            a = self.launch.value
            if a:
                self.launch_ratio = (self.map.bad or self.map.level) / a

    def _bump(self, down: bool = False) -> None:
        """速度プロファイルに効く値が変わった。上げただけの変更は、車の足元のプロファイルを
        変えない（planner の `_grip_hold`）ので、加速・減速の測定は止めない。"""
        self.ver += 1
        if down:
            self.ver_down += 1
            self.long.note_change()

    # ── 表示 ──

    def state(self, gate: str = "") -> str:
        if gate:
            return f"停止: {gate}"
        if self.cap is not None:
            return "★前兆で減速"
        if self.down_only:
            return "下げるだけ"
        if self.converged:
            return "収束"
        return "挟み込み" if self.map.bad else "探索"
