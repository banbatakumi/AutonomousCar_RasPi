"""ml_lidar/course_gen.py — 手続きコース生成。チェックポイント＋旋回率制限ウォーク方式。

`sim.track` の低レベル関数（`build_from_points()`）は再利用するが、生成ポリシー
自体はここに閉じる。旧`sim/random_course.py`がSLAM評価（`sim/courses/make_random_courses.py`
経由）からも依存されて削除時の依存関係の洗い出しに手間取った反省を踏まえ、`sim/`直下の
共有モジュールからは独立させてある（`ML_LIDAR_V2_PROMPT.md`）。

`sim/courses/`の固定資産（自作コース・SLAM評価用`circuit_chicane_*`）とは無関係。

## 旧方式（極座標 r(θ) 補間）を置き換えた理由

旧実装は、中心Oからの極座標 `(θ, r)` を局所Catmull-Romで周期補間していた。この方式は
「θに対しrが1価」という構成により自己交差なし・閉ループを数学的に保証できる利点があったが、
**中心1点を囲む「星型」の形状しか原理的に作れない**という構造的な限界があった（trainの
straight/corner/medium/hairpin/chicaneというアーキタイプ名は、この単一の生成器の
パラメータ違いに過ぎず、見た目は常に「歪んだドーナツ」の枠から出られなかった——2026-09、
ユーザーが可視化画面で5本とも酷似していることを指摘して発覚）。

## チェックポイント＋旋回率制限ウォーク方式

OpenAI Gym CarRacing・F1TENTH gym（`random_trackgen.py`）が使う手法を採用した。
中心Oを囲む円周上にランダムなチェックポイント群 `(θ, r)` を置くところまでは旧方式と
同じだが、**チェックポイント間を滑らかな解析式で補間するのではなく、車両が実際に
チェックポイントを順番に追いかけるように「歩幅一定・1歩あたりの旋回角を上限でクランプ」
しながら歩いて中心線を作る**。r(θ)という1価関数の形に縛られないため、本物のヘアピン
（最小旋回半径ぎりぎりの弧を長く描く鋭い折り返し）や連続シケイン（S字に蛇行する連続した
方向転換）、長い直線区間が自然に出現する。

## 旋回率の上限は構成時点で曲率安全マージンを保証する

旧方式は「生成してから厳密曲率を計算し、車両の最小旋回半径を割り込んでいたら作り直す」
という事後リジェクトサンプリングに頼っていた（Catmull-Romは制御点群から曲率が非線形に
決まり、事前の解析的制約を組みにくいため）。ウォーク方式では、1歩の歩幅`step_m`に対する
最大旋回角を`step_m / min_radius_m`（弧長 = 半径 × 弧度の逆算）にクランプするだけで、
**生成される中心線の曲率は構成上つねに`1/min_radius_m`以下になる**。事後の曲率チェックが
不要になった（`_walk_checkpoints()`のdocstring参照）。

## 自己交差は構成的に排除できないので明示チェックが必要

旧方式の「r>0なら自己交差は起こりえない」という数学的保証は、r(θ)という1価関数の形を
やめた時点で失われる。チェックポイントの配置次第では、ウォークが自分自身の経路に幅
`width_m`未満まで接近する（＝壁が重なる）ことがありうるため、`_self_intersects()`で
明示的にチェックし、違反時はリトライする。

## 初期条件の過渡応答を捨てるため2周分歩き、2周目だけを中心線として使う

歩行の初期姿勢（最初のチェックポイントでの向き）は「2番目のチェックポイントへ向かう方向」
という恣意的な選び方をしているため、そのまま1周しただけでは終点の向きが始点の向きと
大きくズレる（実測: 平均55°前後）。CarRacingの実装が`laps`を数えて複数周ぶん歩かせるのと
同じ理由で、1周目を「操舵の定常状態への収束」に使い捨て、2周目（＝1周目の終わりから
連続して繋がっている区間）だけを中心線として採用する。チェックポイント配置は周ごとに
変わらないため、2周目は1周目の終端と同じ操舵状態から始まり、同じ操舵状態へ収束して
戻ってくる——始点・終点の向きが自然に一致する（`_walk_checkpoints()`のdocstring参照）。

## chicane_len・chicane_offset_fracはstraight/corner背景から独立したパラメータ

`_CHICANE_OFFSET_SEVERITY`はチェックポイント振幅を`base_radius_m`に対する道幅比
（`width_m / base_radius_m`）で絞るための調整で、これが無いと「半径が小さくコース幅が
相対的に太い」組み合わせでシケインの振幅が自己交差チェックに頻発してひっかかる
（scratchpad実測、N=1500でchicane_len=4・振幅固定なら10/1500=0.67%全滅——`chicane_len`を
3に抑え、この道幅比補正を入れるとN=2000で0/2000まで下がる）。
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import NamedTuple

import numpy as np

from sim import track
from sim.course import Course
from sim.vehicle import VehicleSpec

__all__ = ["RandomWalkLoopParams", "walk_loop_course", "random_walk_loop_course",
          "sample_random_walk_loop_params", "vehicle_min_turn_radius_m"]

_PROCEDURAL_PATH = Path("<procedural>")

#: 車両の理論最小旋回半径に対する安全マージン。1.0未満にはしない——車両が物理的に
#: 曲がりきれないコーナーを作ると、衝突が方策の巧拙と無関係な避けようのない失敗になり、
#: 報酬信号にノイズ/バイアスを持ち込む。旧方式からそのまま引き継いだ値（旧
#: `course_gen.py`の`_RADIUS_MARGIN`docstring参照、`sim/courses/toyota.json`実測の
#: 最厳しいコーナー0.44mに対する較正済みの値）
_RADIUS_MARGIN = 1.1

#: 生成→検証（閉合・自己交差）→だめなら作り直す、のリトライ上限
_MAX_COURSE_GEN_RETRY = 20

#: チェックポイント追跡を打ち切るまでの最大ステップ数を見積もる係数（周長の見積もりに
#: 対する倍率）。旋回率制限がきつい・チェックポイントが遠い場合でも2周分歩き切れる
#: だけの余裕を持たせる（scratchpad実測、この値でN=2000中「ステップ上限に達して打ち切り」
#: は一度も発生しなかった）
_MAX_STEPS_PERIMETER_FACTOR = 9.0

#: `random_walk_loop_course()`がパラメータごと引き直す上限回数
_MAX_PARAM_RESAMPLE = 10

#: 手続き生成コースの道幅レンジ [m]（`sample_random_walk_loop_params`）。
#: **v22(2026-09-17)で上限を1.4→2.5mへ広げた。** 旧レンジ0.8〜1.4mでは近い側の壁が
#: 必ず0.70m以内にあり、方策は「広いところ」を一度も見たことがない状態だった
#: （ランダム20本で、近い側の壁が1m以上離れた点は0.0%）。自作コースをこのレンジに
#: 入る割合で並べるとシムでの成否と一致する——normal 100%(完走6/6)・course3 94.7%
#: (衝突6/6)・toyota 2.2%(シム完走だが実機で不安定)・course1 0.0%(衝突6/6)。
#: 決定的だったのはcourse3で、道幅が1.4mを超えるのは進行度63.8〜66.3%と67.2〜69.9%の
#: 2区間だけ(最大2.45m)なのに、8本中7本の衝突が進行度66.9〜67.8%＝その2区間の間に
#: 集中していた（PROGRESS.md 2026-09-17「6回目」節）。
#: 上限2.5mは実測した自作コースの最大(course1の中央1.95m・p95 4.2m)を余裕を持って
#: 覆う値。生成器はこのレンジのまま自己交差チェックを通る（3.0mでも30/30で成功を確認）
_WIDTH_RANGE_M = (0.8, 2.5)

#: 道幅の上限を「コースの最小フィーチャサイズ」`base_radius_m * (1 - radius_jitter_frac)`
#: の何倍までに抑えるか。**v22の学習を1本落としてから入れた**（2026-09-17）。
#: レンジを2.5mまで広げた直後は上限を素で引いており、`base_radius`が小さく
#: `radius_jitter`が大きい組（ウォークが中心付近まで食い込む）と広い道幅が同時に出ると、
#: `_self_intersects()`を満たす中心線が引けず`_generate_walk_centerline`が
#: `RuntimeError`で落ちる。発生率は**4000本中3本(0.07%)**で、60本の事前確認では
#: 検出できなかった——が、8env×3Mstepでは約25,000エピソード＝19回程度起きる計算で、
#: `SubprocVecEnv`のワーカーごと落ちて学習が停止した。
#: 実測した失敗3本はいずれもこの比が**3.28以上**（成功例は中央1.07・p95 2.33）。
#: 2.5に制限すると成功例の95%以上を保ったまま失敗域を外せる。
#: なお比だけに頼らず`random_walk_loop_course()`側にも取りこぼし用のリトライを入れてある
_WIDTH_FEASIBLE_RATIO = 2.5

#: シケインクラスタの半径振幅を`width_m / base_radius_m`（道幅がコース半径に対して
#: 相対的にどれだけ太いか）で絞るための調整幅。severity=0(道幅が半径よりずっと細い)なら
#: 振幅そのまま、severity=1(道幅が半径と同オーダー)なら半分まで絞る
_CHICANE_OFFSET_SEVERITY_SCALE = 0.5


def vehicle_min_turn_radius_m(spec: VehicleSpec | None = None) -> float:
    """車両が**舵角の限界で**切れる最小旋回半径 [m]（`wheelbase / tan(max_steer)`）。

    `config/vehicle.toml`の値を動的に読む（ハードコードしない）——実測値が
    更新されても生成コースの最小半径が自動で追従する。

    ## ★これは低速でしか使えない半径（速度が上がるとグリップが律速する）

    実際に曲がれる半径は`max(この値, v^2/(mu*g))`。`sim/vehicle.py`の`step()`が
    `max_curvature = a_lat_max / speed^2`で同じ制限をかけている。実測値では
    `mu*g = 4.46 m/s^2`・舵角限界`0.398 m`なので、**この半径を使い切れるのは
    `sqrt(mu*g*R) = 1.33 m/s`まで**——`max_speed`既定の2.0 m/sでは最小半径は
    `0.897 m`と倍以上になる。

    つまり生成コースの最小半径0.438m相当のコーナーは、**全開では曲がれず
    1.4 m/s程度まで減速して初めて通れる**。これは意図した設計（レーシングとして
    自然なブレーキング）だが、**通過可能性を判定するときにこの値を速度非依存の
    定数として使うと誤る**（2026-09-22に実際に踏んだ。`sim/courses/course3.json`の
    ヘアピンを「2.0 m/sでも通過可能」と誤判定した。速度依存で測り直すと1.33 m/sが上限）。
    """
    spec = spec or VehicleSpec.load()
    return spec.wheelbase / math.tan(spec.max_steer)


def _sample_checkpoints(rng: np.random.Generator, n_checkpoints: int, base_radius_m: float,
                        angle_jitter_frac: float, radius_jitter_frac: float,
                        chicane_len: int = 0, chicane_offset_frac: float = 0.0
                        ) -> tuple[np.ndarray, np.ndarray]:
    """θ単調増加のチェックポイント群`(θ, r)`を`n_checkpoints`個サンプリングする。

    旧方式の`_sample_control_points()`と配置ロジックは同じ（円周上に均等角度＋
    ジッター）。違いはこの後段——ここで置いたチェックポイントを補間で結ぶのではなく
    `_walk_checkpoints()`が旋回率制限ウォークで辿る点にある。

    :param chicane_len: 0より大きい場合、ランダムに選んだ開始位置から連続する
        `chicane_len`個のチェックポイントの半径を`±chicane_offset_frac`で交互に
        上書きし、「外→内→外」の連続した折り返しを強制する
    """
    assert 0.0 <= angle_jitter_frac < 1.0, "angle_jitter_fracは1.0未満でなければならない"
    assert 0 <= chicane_len <= n_checkpoints - 2, "chicane_lenはn_checkpoints-2以下でなければならない"
    assert chicane_len == 0 or chicane_offset_frac > 0.0, \
        "chicane_len>0ならchicane_offset_fracも>0でなければならない"
    step = 2 * math.pi / n_checkpoints
    base_theta = np.arange(n_checkpoints) * step
    theta = base_theta + rng.uniform(-0.5, 0.5, n_checkpoints) * angle_jitter_frac * step
    theta = np.sort(theta)
    radius = base_radius_m * (1.0 + rng.uniform(-radius_jitter_frac, radius_jitter_frac, n_checkpoints))
    if chicane_len > 0:
        start = int(rng.integers(0, n_checkpoints))
        sign = 1.0
        for i in range(chicane_len):
            radius[(start + i) % n_checkpoints] = base_radius_m * (1.0 + sign * chicane_offset_frac)
            sign = -sign
    return theta, radius


def _walk_checkpoints(theta: np.ndarray, radius: np.ndarray, step_m: float,
                      max_turn_per_step: float, max_total_steps: int) -> np.ndarray | None:
    """チェックポイント群を旋回率制限ウォークで辿り、中心線点列`(N,3)`を作る。

    毎ステップ、現在向いている方向`yaw`から次のチェックポイントへの角度差を
    `±max_turn_per_step`にクランプしてから`yaw`を更新し、歩幅`step_m`だけ前進する。
    **この1操作だけで、生成される中心線の曲率は構成上つねに`max_turn_per_step/step_m`
    以下になる**（弧長`step_m`に対する旋回角の上限を直接指定しているので、事後の
    曲率チェックが原理的に不要——モジュールdocstring参照）。

    次のチェックポイントへの切り替えは「距離が近づいたら」ではなく「原点周りの
    極角(累積・アンラップ済み)が次チェックポイントのθを追い越したら」で判定する
    （OpenAI Gym CarRacingと同じ方式）。距離ベースの切り替えだと、旋回率がきつく
    チェックポイントに幾何学的に到達できない場合にその場を無限に周回して詰まる
    （scratchpad実測で発覚）。極角ベースなら、たとえ狙った位置に正確に到達できなくても
    「原点周りを一周した」という進捗自体は必ず前進するため、詰まらない。

    2周分歩き、**2周目だけを返す**——1周目は初期姿勢（最初のチェックポイントに
    向かうときの恣意的な初期向き）の過渡応答を吸収するための使い捨てで、2周目は
    1周目の終端の操舵状態から連続して始まり、同じ操舵状態へ収束して戻ってくるため、
    始点・終点の位置・向きが自然に一致する（モジュールdocstring参照）。

    チェックポイントに旋回率制限内でどうしても追いつけず、2周分歩き切れなかった場合は
    `None`を返す（呼び出し側でリトライする）。
    """
    n = len(theta)
    xy = np.column_stack((radius * np.cos(theta), radius * np.sin(theta)))
    pos = xy[0].copy()
    yaw = math.atan2(xy[1, 1] - xy[0, 1], xy[1, 0] - xy[0, 0])
    alpha_prev = math.atan2(pos[1], pos[0])
    alpha_swept = 0.0
    dest_i = 1
    pts = [(pos[0], pos[1], yaw)]
    lap_boundaries: list[int] = []
    next_lap_mark = 2 * math.pi
    n_laps = 2

    for _ in range(max_total_steps):
        if alpha_swept >= 2 * math.pi * n_laps:
            break
        target = xy[dest_i % n]
        dx, dy = target[0] - pos[0], target[1] - pos[1]
        desired_yaw = math.atan2(dy, dx)
        dyaw = (desired_yaw - yaw + math.pi) % (2 * math.pi) - math.pi
        dyaw = max(-max_turn_per_step, min(max_turn_per_step, dyaw))
        yaw += dyaw
        pos = (pos[0] + step_m * math.cos(yaw), pos[1] + step_m * math.sin(yaw))
        pts.append((pos[0], pos[1], yaw))

        alpha = math.atan2(pos[1], pos[0])
        dalpha = (alpha - alpha_prev + math.pi) % (2 * math.pi) - math.pi
        alpha_swept += dalpha
        alpha_prev = alpha
        dest_theta = theta[dest_i % n] + 2 * math.pi * (dest_i // n)
        if alpha_swept + theta[0] >= dest_theta:
            dest_i += 1
        if alpha_swept >= next_lap_mark:
            lap_boundaries.append(len(pts) - 1)
            next_lap_mark += 2 * math.pi
    else:
        return None  # max_total_steps以内に2周分歩き切れなかった

    if len(lap_boundaries) < n_laps:
        return None
    pts_arr = np.asarray(pts, dtype=np.float64)
    start_idx, end_idx = lap_boundaries[-2], lap_boundaries[-1]
    return pts_arr[start_idx:end_idx + 1]


def _self_intersects(pts: np.ndarray, width_m: float, step_m: float) -> bool:
    """中心線が自分自身に道幅`width_m`未満まで接近していないかを判定する。

    旋回率制限ウォークは（旧方式のr(θ)が1価であることによる構成的な保証と違い）
    自己交差を構造的に排除できない。弧長方向のインデックス差で十分離れた点同士
    （直近走行区間・ループ端の隣接は除外）のユークリッド距離が`width_m`を下回れば、
    コースの壁が自分自身と重なる/接触するとみなす。
    """
    xy = pts[:, :2]
    n = len(xy)
    exclude = max(2, int(math.ceil(2.0 * width_m / step_m)))
    d = np.hypot(xy[:, 0:1] - xy[None, :, 0], xy[:, 1:2] - xy[None, :, 1])
    idx = np.arange(n)
    diff = np.abs(idx[:, None] - idx[None, :])
    far = np.minimum(diff, n - diff) > exclude  # ループ端の隣接も考慮した「離れている」判定
    return bool(np.any(d[far] < width_m))


def _generate_walk_centerline(rng: np.random.Generator, n_checkpoints: int, base_radius_m: float,
                              angle_jitter_frac: float, radius_jitter_frac: float,
                              resolution: float, min_radius_m: float, width_m: float, name: str,
                              chicane_len: int = 0, chicane_offset_frac: float = 0.0) -> np.ndarray:
    """閉合・自己交差なしの中心線が引けるまで作り直す。曲率は`_walk_checkpoints()`が
    構成時点で保証するため、ここでの検証対象は閉合と自己交差の2つだけでよい。
    """
    step_m = resolution
    max_turn_per_step = step_m / min_radius_m
    max_total_steps = int(_MAX_STEPS_PERIMETER_FACTOR * math.pi
                          * base_radius_m * (1.0 + radius_jitter_frac) / step_m)
    for _ in range(_MAX_COURSE_GEN_RETRY):
        theta, radius = _sample_checkpoints(rng, n_checkpoints, base_radius_m,
                                            angle_jitter_frac, radius_jitter_frac,
                                            chicane_len=chicane_len,
                                            chicane_offset_frac=chicane_offset_frac)
        pts = _walk_checkpoints(theta, radius, step_m, max_turn_per_step, max_total_steps)
        if pts is None:
            continue  # 旋回率制限内でチェックポイントを2周ぶん辿り切れなかった
        gap = math.hypot(pts[-1, 0] - pts[0, 0], pts[-1, 1] - pts[0, 1])
        dth = abs((pts[-1, 2] - pts[0, 2] + math.pi) % (2 * math.pi) - math.pi)
        if gap > width_m * 0.25 or dth > math.radians(10):
            continue  # 閉じていない
        if _self_intersects(pts, width_m, step_m):
            continue
        return pts
    raise RuntimeError(
        f"{name}: {_MAX_COURSE_GEN_RETRY}回リトライしても閉合・自己交差なしの"
        "コースが生成できなかった。パラメータの上限が強すぎる可能性")


class RandomWalkLoopParams(NamedTuple):
    n_checkpoints: int
    base_radius_m: float
    angle_jitter_frac: float
    radius_jitter_frac: float
    width_m: float
    clockwise: bool
    chicane_len: int = 0
    chicane_offset_frac: float = 0.0


def sample_random_walk_loop_params(rng: np.random.Generator) -> RandomWalkLoopParams:
    """`random_walk_loop_course()`のパラメータ抽出だけを切り出したもの。

    `walk_loop_course()`（コース構築、重い）を経由せずに抽出結果を直接検証できるように
    分離してある。

    旧方式と異なり、曲率安全マージンは`_walk_checkpoints()`が構成時点で保証するため
    （モジュールdocstring参照）、`n_checkpoints`別のジッター上限較正テーブルは不要になった
    ——ジッターが強くても「作り直しが増える」のではなく「ウォークがより急な弧を描く」
    だけで済む。リトライが必要になるのは閉合・自己交差の失敗のみで、これは
    `chicane_offset_frac`（下記）以外は素の角度・半径ジッターの範囲では実質発生しない
    （scratchpad実測、N=1500で全滅0件）。
    """
    n_checkpoints = int(rng.integers(6, 11))                # 6〜10点
    base_radius_m = float(rng.uniform(1.5, 3.5))
    angle_jitter_frac = float(rng.uniform(0.15, 0.50))
    radius_jitter_frac = float(rng.uniform(0.10, 0.65))
    # 道幅は`base_radius`/`radius_jitter`と独立には引けない（`_WIDTH_FEASIBLE_RATIO`参照）。
    # 小さく・食い込みの大きいループに広い道幅を combine すると自己交差が避けられない
    width_lo, width_hi = _WIDTH_RANGE_M
    feasible_hi = _WIDTH_FEASIBLE_RATIO * base_radius_m * (1.0 - radius_jitter_frac)
    width_m = float(rng.uniform(width_lo, max(width_lo, min(width_hi, feasible_hi))))
    clockwise = bool(rng.random() < 0.5)
    # `sim/courses/toyota.json`（観戦専用）は短い直線を挟んで急角ターンが連続するが、
    # 独立ジッターだけでは「1点だけ鋭く尖る」孤立ヘアピンしか実質出現しない。一定確率で
    # `_sample_checkpoints`の`chicane_len`クラスタ強制を有効にし、連続タイトターンへの
    # 露出を学習分布に追加する。chicane_lenは3で頭打ち——4だと`chicane_offset_frac`を
    # 自己交差なしで満たせる余地が急に狭まる（scratchpad実測、モジュールdocstring参照）。
    # 振幅は`width_m/base_radius_m`（道幅がコース半径に対してどれだけ相対的に太いか）で
    # 絞る——これが無いと「半径が小さく道幅が相対的に太い」組み合わせで自己交差リトライが
    # 全滅しうる
    _CHICANE_PROB = 0.35
    max_chicane = min(n_checkpoints - 2, 3)
    if max_chicane >= 3 and rng.random() < _CHICANE_PROB:
        chicane_len = max_chicane
        severity = min(1.0, width_m / base_radius_m)
        chicane_offset_frac = (float(rng.uniform(0.16, 0.26))
                               * (1.0 - _CHICANE_OFFSET_SEVERITY_SCALE * severity))
    else:
        chicane_len = 0
        chicane_offset_frac = 0.0
    return RandomWalkLoopParams(n_checkpoints, base_radius_m, angle_jitter_frac,
                                radius_jitter_frac, width_m, clockwise,
                                chicane_len, chicane_offset_frac)


def walk_loop_course(n_checkpoints: int, base_radius_m: float, angle_jitter_frac: float,
                     radius_jitter_frac: float, width_m: float, *, seed: int,
                     resolution: float = 0.03, role: str = "train",
                     name: str = "walk_loop", clockwise: bool = False,
                     chicane_len: int = 0, chicane_offset_frac: float = 0.0) -> Course:
    """パラメータ＋乱数シードを固定で渡して閉ループコースを1本作る。

    形状そのものが乱数列（チェックポイントのθ・rジッター）に依存するため、
    **再現性にはシードそのものを引数で受け取る**（幾何パラメータの「範囲」だけでは
    形状が一意に決まらない）。`train_rl.py`のeval固定コースはこの関数に固定シードを
    渡すことで決定論的に再現する。

    :param clockwise: `True`で時計回りにする。チェックポイントをθ増加方向に辿る
        ウォークは構成上必ず反時計回りになるので、時計回りにするには生成後に
        点列を反転させ、向きを180°回転させる
    """
    spec = VehicleSpec.load()
    min_radius_m = vehicle_min_turn_radius_m(spec) * _RADIUS_MARGIN
    rng = np.random.default_rng(seed)

    pts = _generate_walk_centerline(rng, n_checkpoints, base_radius_m, angle_jitter_frac,
                                    radius_jitter_frac, resolution, min_radius_m, width_m, name,
                                    chicane_len=chicane_len, chicane_offset_frac=chicane_offset_frac)
    if clockwise:
        pts = pts[::-1].copy()
        pts[:, 2] = pts[:, 2] + math.pi

    meta = {
        "name": name,
        "width": width_m,
        "resolution": resolution,
        "loop": True,
        "role": role,
    }
    built = track.build_from_points(pts, meta)
    return Course(
        name=name,
        path=_PROCEDURAL_PATH,
        resolution=built["resolution"],
        origin=built["origin"],
        start=built["start"],
        grid=np.ascontiguousarray(built["grid"]),
        centerline=built["centerline"],
        width=built["width"],
        obstacles=built.get("obstacles"),
        role=role,
    )


def random_walk_loop_course(rng: np.random.Generator, *, resolution: float = 0.03) -> Course:
    """チェックポイント＋旋回率制限ウォーク方式で閉ループコースを1本ランダムに作る（学習用）。

    制御点数・ベース半径・角度/半径ジッター・道幅・旋回方向をランダムに選ぶ。
    旋回方向を固定すると学習が右コーナーをほぼ経験しないまま進むため、50/50で
    ランダム化する（観測・行動が左右対称な設計なので、方策が左右対称に汎化する
    理由が無い——旧実装からの既知の教訓）。
    """
    # ★パラメータを引き直して再挑戦する。`_generate_walk_centerline()`の内側リトライは
    # **チェックポイント配置だけ**を引き直すので、パラメータの組自体が実現不能なとき
    # （道幅に対してループが小さすぎる等、`_WIDTH_FEASIBLE_RATIO`参照）は20回とも失敗して
    # `RuntimeError`になる。学習中はこれが`SubprocVecEnv`のワーカーごと落として
    # **数時間の学習を停止させる**（2026-09-17に実際にv22の学習が1本落ちた）。
    # `_WIDTH_FEASIBLE_RATIO`で発生率自体は下げてあるが、取りこぼしはここで吸収する
    for attempt in range(_MAX_PARAM_RESAMPLE):
        p = sample_random_walk_loop_params(rng)
        seed = int(rng.integers(0, 2 ** 31 - 1))
        try:
            return walk_loop_course(p.n_checkpoints, p.base_radius_m, p.angle_jitter_frac,
                                    p.radius_jitter_frac, p.width_m, seed=seed,
                                    resolution=resolution, role="train",
                                    name="random_walk_loop", clockwise=p.clockwise,
                                    chicane_len=p.chicane_len,
                                    chicane_offset_frac=p.chicane_offset_frac)
        except RuntimeError as e:
            # 握り潰すと「静かに分布が偏る」ので、起きたことは必ず見えるようにする
            print(f"!! コース生成に失敗したのでパラメータを引き直す "
                  f"({attempt + 1}/{_MAX_PARAM_RESAMPLE}): {e}", file=sys.stderr, flush=True)
    raise RuntimeError(
        f"random_walk_loop: パラメータを{_MAX_PARAM_RESAMPLE}組引き直しても生成できなかった。"
        "サンプリングレンジ（_WIDTH_RANGE_M・_WIDTH_FEASIBLE_RATIO等）を見直すこと")
