"""理想レーシングライン — 中心線+道幅から、最小曲率に寄せたオフセットと
曲率考慮の目標速度プロファイルをオフラインで計算する。

`sim/gym_env.py` の `SimE2EEnv` が「壁に衝突しないが、コーナーでアペックスを
突く・進入前に減速するといったレーシングラインらしい走り方をしない」問題への
対策として使う（`docs/progress` 2026-09-01「ml_lidar v8→v9」参照）。方策自体は
生LiDAR+速度のE2Eのまま変えず、**学習時の報酬にだけ**理想ラインへの追従度を
組み込む（Trajectory-Aided Learning、Bosello et al. arXiv:2306.07003 に倣う）。

## 2026-09-16: 曲率二乗「和」の直接最小化から、centerline回帰つき二次計画へ全面置換

旧実装（`torch.optim.LBFGS`で厳密曲率`atan2`ベースの`Σκ²`を直接最小化）は、
バンビが`ml_lidar/watch.py`の観戦画面で「カーブでアウトに非常に膨らんだ
ラインになっている」と指摘した症状を、`hairpin`/`corner`固定コースで再現・
定量検証した結果、**実装のバグではなく目的関数設計の欠陥**と判明した：
中心線に対して引き戻す項が無いまま`Σκ²`だけを最小化すると、「ループ全体を
一律に外側へシフトする」解（多数を占める緩やかな区間の曲率が広範囲で下がる
一方、少数のヘアピン区間だけ悪化する——広範囲の改善の総和が上回るため
数学的に真に最適）に収束しうる。実測（`hairpin`固定コース）でロス36.0まで
収束済み（反復を8→30回に増やしても同じ）、逆に曲率符号に沿ってアペックスを
切る初期値から再最適化してもロス421.6（センターラインそのものの148.4より
悪化）にしかならず、単純な収束不足でも初期値依存の局所解でもないことを
確認した（PROGRESS.md 2026-09-16節に検証手順の詳細）。

代わりに、実車ナビゲーションスタックの`raspi/nav/raceline.py`（SLAM実走で
妥当なアウトインアウトを生成できていると確認済み）と同じ定式化に置き換える。
これはHeilmeier et al.のQP最小曲率法（TUM autonomous racing、要点は
[arXiv:2511.00946](https://arxiv.org/pdf/2511.00946)等の追試論文にも整理されている）
と同じ系統——中心線からのオフセット`α_i`を変数に、位置の巡回2階差分
`D`（= 離散ラプラシアン、曲率の2次近似）のノルム二乗に、**中心線へ引き戻す
正則化項`λ‖α‖²`を足した二次形式**を最小化する:

    J(α) = ‖D p_x‖² + ‖D p_y‖² + λ‖α‖²,   p = c + α·n

`λ`項が無いと（旧実装が正にそうだった）目的関数は「どれだけループ全体を
一律にシフトしても構わない」degenerate方向を持ちうる——`λ‖α‖²`はこの自由度に
ペナルティを与え、必要最小限のオフセットだけを許す。二次形式なので**疎行列の
直接解**（アクティブセット法で箱制約を扱う）で解け、`torch`・非線形最適化は
不要になった。詳細は`_min_curvature_alpha()`のdocstring（`raspi/nav/raceline.py`の
同名関数から移植、コメントもそちらの実測知見をそのまま引き継ぐ）を参照。

## LiDARシム経路（中間案）

`sim.lidar.VirtualLidar`でセクタパケットを生成し、`raspi.msgs.ScanAssembler`で
実機と同じ鏡像反転・欠測フラグ組み立てまで通すが、`sim.link`のUART/STM32バイト
フレーミング層は経由しない（`sim_support.stm_us()`は時刻同期なしの単純なns→us変換）。
"""

from __future__ import annotations

import math

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla

from .course import Course
from .vehicle import GRAVITY_MPS2

__all__ = ["compute_raceline_offsets", "compute_speed_profile", "compute_raceline_xy"]

#: `width=None`のコース（道幅がスカラー/配列で語れないコース全般。
#: `sim/random_course.py`には現状該当アーキタイプは無いが、将来の手作業
#: コースエディタが作るコース等を想定したインフラとして残してある）で、
#: `Course.raycast_batch`により参照パス（中心線）の法線方向へ直接壁までの
#: 距離を測るときの探索上限 [m]。狭く見積もると「壁に当たらなかった」
#: （`raycast`系の0.0センチネル）が発生し、`_lateral_bounds`がそれを補う
#: 救済措置（下記）に頼りきりになって道幅推定の精度が落ちる
#: （2026-09-05追加。経緯は次の定数のdocstring参照）
_WALL_RAYCAST_MAX_RANGE_M = 20.0

#: `width=None`のコースでraycast実測した`(lo, hi)`に掛ける上限 [m]。
#: ★2026-09-05実測で発覚: 壁境界を1個の多角形として直接生成し、参照パスは
#: それを小さい`inset_m`(0.15〜0.30m)だけ内側へ寄せた副産物とする
#: 「自由多角形境界のアリーナ」というコース生成方式（手続き生成の`wallseg`
#: アーキタイプの試作版の1つ、後に不採用となり削除済み）で発生した:
#: 壁に近い側は数cm〜数十cmで収まる一方、アリーナ内部側は開けた空間を
#: そのまま突っ切ってしまい**実測3.6m〜12mという桁違いの非対称`box`制約**
#: になっていた。この巨大な箱の中で曲率エネルギーを最小化しようとして
#: 参照パスから大きく・不連続に振れる理想ラインを作ってしまう——`width`方式
#: （道幅0.7〜1.6m程度）が暗黙に仮定
#: していた「箱は車体規模のオーダー」が壊れることが根本原因。実測した
#: 壁までの距離をそのまま使うのではなく、現実的な「その場でのふらつきの
#: 許容量」程度に上限を掛けて、探索範囲を車体規模に保つ
_RAYCAST_LATERAL_OFFSET_CAP_M = 0.5

#: `drive_accel_m_s2`が未実測(0.0)のときに使う、加速側のフォールバック値 [m/s²]。
#: `sim/vehicle.py`の`_next_speed()`が未実測時にハードウェア仕様値へフォールバック
#: するのと同じ思想（ここでは目標速度プロファイルの見積もり用の仮値）
DEFAULT_DRIVE_ACCEL_M_S2 = 1.5

#: 前進(加速)・後退(減速)パスを何周ぶん繰り返すか。閉ループなので1周だけだと
#: 「開始点から見て終盤」の制約が「序盤」まで伝播しきらない。2周で十分収束する
#: （プロファイルは速度ボーナスの目安であり、真の最適解である必要はない）
_SPEED_PASS_LAPS = 2

#: 曲率エネルギー`‖Dp‖²`とセンターライン回帰`λ‖α‖²`の重み付け。大きいほど
#: センターライン寄り、小さいほど曲率最小化（＝アウトインアウトの振れ幅）を
#: 優先する。`raspi/nav/raceline.py`（実車SLAMナビで実測・妥当な挙動を
#: 確認済み）と同じ値・同じ`lam*step**4`正規化式をそのまま踏襲する——
#: 点間隔`step`を変えても（`course_gen`の`resolution`設定に依らず）同じ
#: 「強さ」の正則化になる（`_min_curvature_alpha`docstring参照）
_LAM_DEFAULT = 0.1

#: アクティブセット法（箱制約）の反復上限。境界に貼り付いた点をKKT条件で
#: 解放しながら再解を繰り返す。`raspi/nav/raceline.py`の実績（実測で通常
#: 3〜8回で収まる）を踏まえた余裕を持たせた値
_ACTIVE_SET_MAX_ITER = 30

#: 障害物回避が箱制約[lo,hi]を狭める弧長方向の窓を、除外半径(`r_excl`。障害物半径+
#: 車体半幅+安全マージン)からさらに広げる余裕 [m]。窓を`r_excl`ちょうどにすると
#: 障害物の真横の数点しか回避に参加できず、急な進路変更（局所的に曲率が跳ね上がる
#: 解）しか選べなくなる。前後に余裕を持たせ、なだらかな回避カーブを選べるようにする
#: （2026-09-03追加、`ml_lidar`のobstacleアーキタイプ衝突率100%の診断を受けて。
#: `docs/`ではなくコミットログ・PROGRESS.md参照）
_OBSTACLE_AVOID_MARGIN_M = 0.4


def _segment_lengths(xy: np.ndarray) -> np.ndarray:
    """`seg[i]` = 点`i`から点`i+1`（最後は点0に周回）までの距離 [m]。
    `sim/random_course.py`の`_min_turn_radius_m`と同じ周回差分の作り方。"""
    loop = np.vstack([xy, xy[:1]])
    return np.hypot(*np.diff(loop, axis=0).T)


def _discrete_curvature(xy: np.ndarray) -> np.ndarray:
    """`sim/random_course.py`の`_min_turn_radius_m`と同じ離散曲率
    （隣接セグメントのyaw差 / セグメント長）。閉ループの点`i`ごとの符号付き曲率 [1/m]。
    `_min_curvature_alpha`が解くのはこれの二次近似（`‖Dp‖²`）だが、最終的な
    「どちらの解が本当に曲率が小さいか」の判定（`compute_raceline_offsets`の
    ★安全弁）や`compute_speed_profile`の速度プロファイルには、近似を使わず
    このまま厳密曲率を使う。"""
    loop = np.vstack([xy, xy[:1]])
    seg = np.hypot(*np.diff(loop, axis=0).T)
    yaw = np.arctan2(np.diff(loop[:, 1]), np.diff(loop[:, 0]))
    dyaw = np.diff(np.unwrap(np.concatenate([yaw, yaw[:1]])))
    return dyaw / np.maximum(seg, 1e-6)


def _cyclic_d2(n: int) -> "sp.csr_matrix":
    """巡回2階差分`D`（n×n、疎行列）。閉ループの境界条件はこれだけで完結する
    （`raspi/nav/raceline.py`の`_cyclic_d2`と同じ定義。あちらは密行列だが、
    ここでは`course_gen`が生成する密な中心線（数百〜数千点）でも直接解ける
    よう疎行列のまま扱う——`D`はどのみち3重対角、`D^T D`も5重対角の疎行列で、
    密行列に展開する意味が無い）。"""
    i = np.arange(n)
    rows = np.concatenate([i, i, i])
    cols = np.concatenate([i, (i - 1) % n, (i + 1) % n])
    data = np.concatenate([np.full(n, -2.0), np.full(n, 1.0), np.full(n, 1.0)])
    return sp.csr_matrix((data, (rows, cols)), shape=(n, n))


def _min_curvature_alpha(c: np.ndarray, nrm: np.ndarray, lo: np.ndarray, hi: np.ndarray, *,
                         lam: float, step_m: float, max_iter: int) -> np.ndarray:
    """箱制約つき最小曲率問題を解いて`α`（中心線`c`からの法線`nrm`方向オフセット）を返す。
    `raspi/nav/raceline.py`の`min_curvature_alpha()`の移植（実車SLAMナビで実測・
    妥当な挙動を確認済みのアルゴリズムをそのまま流用する）。

    :param lo: 各点の下限（右側の余裕。**負の値**）
    :param hi: 各点の上限（左側の余裕）
    :param lam: 中心線へ引き戻す重み（モジュール定数`_LAM_DEFAULT`docstring参照）
    :param step_m: 点の平均間隔 [m]。`lam`の正規化に使う（下記）

    ## `lam`は`step_m⁴`で正規化する

    曲率エネルギー項`‖Dp‖²`は点の間隔`ds`に対して`(κ·ds²)²`で効くので、
    刻みを変えると同じ`lam`の意味が4乗で変わる。**内部で`lam·step_m⁴`を
    使う**ことで、`course_gen`の`resolution`設定を変えても同じ「強さ」の
    正則化になる。

    ## 一度境界に固定した点は、勾配が内側を向いたら**解放する**

    固定しっぱなしにすると、最初の1回の解が大きく振れたときに全点が境界に
    貼り付いて、そのまま answer になる——**中心線より曲がった経路が「最適解」
    として出てくる**（`raspi/nav/raceline.py`で実測済みの失敗パターン）。
    KKT条件（下限にいる点の勾配は0以上、上限にいる点は0以下）を見て、
    破っている点を自由集合へ戻す。
    """
    n = len(c)
    d2 = _cyclic_d2(n)
    dtd = (d2.T @ d2).tocsr()
    nx, ny = nrm[:, 0], nrm[:, 1]
    nx_d, ny_d = sp.diags(nx), sp.diags(ny)
    lam_eff = lam * step_m ** 4
    m = (nx_d @ dtd @ nx_d + ny_d @ dtd @ ny_d + lam_eff * sp.identity(n)).tocsr()
    b = nx * (dtd @ c[:, 0]) + ny * (dtd @ c[:, 1])
    tol = 1e-12 * max(1.0, float(np.abs(b).max()))

    alpha = np.zeros(n)
    at_lo = np.zeros(n, dtype=bool)
    at_hi = np.zeros(n, dtype=bool)
    idx_all = np.arange(n)
    for _ in range(max(1, max_iter)):
        fixed = at_lo | at_hi
        free = ~fixed
        if free.any():
            free_idx = idx_all[free]
            fixed_idx = idx_all[fixed]
            sub = m[free_idx, :][:, free_idx]
            contribution = (m[free_idx, :][:, fixed_idx] @ alpha[fixed_idx]
                           if fixed_idx.size else np.zeros(free_idx.size))
            rhs = -(b[free_idx] + contribution)
            try:
                alpha[free_idx] = spla.spsolve(sub.tocsc(), rhs)
            except Exception:                                          # noqa: BLE001
                # 退化した（同じ点が並んでいる等）。**例外で計算を止めない**
                alpha[free_idx] = np.linalg.lstsq(sub.toarray(), rhs, rcond=None)[0]

        below = alpha < lo - 1e-12
        above = alpha > hi + 1e-12
        if below.any() or above.any():
            alpha = np.clip(alpha, lo, hi)
            at_lo |= below
            at_hi |= above
            continue

        # 実行可能。境界に貼り付いている点を解放できるかをKKTで見る
        g = 2.0 * (m @ alpha + b)
        release = (at_lo & (g < -tol)) | (at_hi & (g > tol))
        if not release.any():
            break
        at_lo[release] = False
        at_hi[release] = False
    return np.clip(alpha, lo, hi)


def _lateral_bounds(xy: np.ndarray, yaw: np.ndarray, width: float | np.ndarray | None,
                    course: "Course | None", *, vehicle_half_width_m: float,
                    safety_margin_m: float) -> tuple[np.ndarray, np.ndarray]:
    """中心線点列における、法線方向オフセットの許容範囲`(lo, hi)`（符号付き、
    常に`lo <= 0 <= hi`）。

    `width`が渡されれば従来通りそこから対称に求める。`width`が`None`なら
    `course`必須——`course.raycast_batch()`で中心線の法線方向（左右）に実測
    した壁までの距離から非対称な境界を求める。壁が非対称に配置され
    `Course.width`（スカラー/centerlineと同じ長さの配列）では表現しきれない
    コース向け（2026-09-05追加）。
    """
    n_pts = len(xy)
    if width is not None:
        if isinstance(width, np.ndarray):
            half_w = width / 2.0
        else:
            half_w = np.full(n_pts, width / 2.0)
        max_offset = np.maximum(0.0, half_w - vehicle_half_width_m - safety_margin_m)
        return -max_offset, max_offset

    if course is None:
        raise ValueError("width が None のときは course が必須です（壁までの実測に使うため）")

    margin = vehicle_half_width_m + safety_margin_m
    nx, ny = -np.sin(yaw), np.cos(yaw)                    # 法線（左が正）
    left = course.raycast_batch(xy, np.arctan2(ny, nx), _WALL_RAYCAST_MAX_RANGE_M)
    right = course.raycast_batch(xy, np.arctan2(-ny, -nx), _WALL_RAYCAST_MAX_RANGE_M)
    # raycastの0.0センチネル（`_WALL_RAYCAST_MAX_RANGE_M`以内に壁が無かった）
    # への備え。将来のwidth=Noneコースが陥りうる安全弁として残す。発生時は
    # 「その方向には`_WALL_RAYCAST_MAX_RANGE_M`より壁が無い＝実質開けている」
    # とみなし、探索上限をそのまま距離として採用する——例外で学習を止めるより
    # 安全側（道幅を広めに見積もるだけで、誤って理想ラインを壁の外へ張り付か
    # せる方向には倒れない）
    left = np.where(left <= 0.0, _WALL_RAYCAST_MAX_RANGE_M, left)
    right = np.where(right <= 0.0, _WALL_RAYCAST_MAX_RANGE_M, right)
    # 実測した壁までの距離を`_RAYCAST_LATERAL_OFFSET_CAP_M`で頭打ちにする
    # （定数のdocstring参照）——width=Noneのコースで片側だけ壁が非常に遠い
    # 場合、その距離をそのまま箱制約に使うと車体規模を大きく超えて振れる
    # 理想ラインを作ってしまうため
    hi = np.minimum(np.maximum(0.0, left - margin), _RAYCAST_LATERAL_OFFSET_CAP_M)
    lo = -np.minimum(np.maximum(0.0, right - margin), _RAYCAST_LATERAL_OFFSET_CAP_M)
    return lo, hi


def _narrow_bounds_for_obstacles(xy: np.ndarray, yaw: np.ndarray, lo: np.ndarray, hi: np.ndarray,
                                 obstacles: np.ndarray, *, vehicle_half_width_m: float,
                                 safety_margin_m: float) -> tuple[np.ndarray, np.ndarray]:
    """障害物ごとに、その周辺の箱制約`[lo, hi]`を「障害物が無い側」へ狭める
    （ハード除外）。旧実装（L-BFGS＋`relu`ソフトペナルティ）から、二次計画＋
    アクティブセット法への置き換えに合わせて、箱制約の追加として実装し直した
    ——ソフトペナルティは目的関数を非二次にしてしまい、疎行列の直接解が使えなく
    なるため。

    ★現状どの呼び出し元（`ml_lidar/env.py`・`watch.py`・`live_watch.py`）も
    `obstacles`を渡していない（`course_gen.py`の現行アーキタイプは障害物を
    生成しない）——将来のための未使用インフラであり、この関数自体は実運用で
    検証されていない。使う際は改めて動作確認すること。
    """
    lo = lo.copy()
    hi = hi.copy()
    nrm = np.column_stack((-np.sin(yaw), np.cos(yaw)))
    seg = _segment_lengths(xy)
    s = np.concatenate([[0.0], np.cumsum(seg)])[:-1]
    total = float(s[-1] + seg[-1])

    for ox, oy, r_obs in obstacles:
        d2 = (xy[:, 0] - ox) ** 2 + (xy[:, 1] - oy) ** 2
        s_obs = s[int(np.argmin(d2))]
        r_excl = float(r_obs) + vehicle_half_width_m + safety_margin_m
        window = r_excl + _OBSTACLE_AVOID_MARGIN_M
        dz = np.abs(s - s_obs)
        dz = np.minimum(dz, total - dz)
        mask = dz <= window
        if not np.any(mask):
            continue

        d_lat = ((ox - xy[mask, 0]) * nrm[mask, 0] + (oy - xy[mask, 1]) * nrm[mask, 1])
        left = d_lat >= 0.0
        idx = np.where(mask)[0]
        hi[idx[left]] = np.maximum(lo[idx[left]], np.minimum(hi[idx[left]], d_lat[left] - r_excl))
        lo[idx[~left]] = np.minimum(hi[idx[~left]], np.maximum(lo[idx[~left]], d_lat[~left] + r_excl))
    return lo, hi


def compute_raceline_offsets(centerline: np.ndarray, width: float | np.ndarray | None, *,
                             vehicle_half_width_m: float, safety_margin_m: float = 0.03,
                             obstacles: np.ndarray | None = None,
                             course: "Course | None" = None,
                             lam: float = _LAM_DEFAULT,
                             max_iter: int = _ACTIVE_SET_MAX_ITER) -> np.ndarray:
    """中心線`(N,3)`(x,y,yaw)からの法線方向オフセット`offset[i]`（符号付き、
    法線`(-sin(yaw), cos(yaw))`の正方向）を返す。`centerline + offset*normal`が
    理想ライン（中心線回帰つき曲率最小化で求めた、アウトインアウトのレーシング
    ライン）の座標になる。アルゴリズムの詳細は`_min_curvature_alpha()`・
    モジュールdocstring参照。

    :param width: `Course.width`と同じ（スカラーまたは`centerline`と同じ長さの配列）。
        `None`の場合は`course`が必須——`course.raycast_batch()`で中心線の法線
        方向に実測した壁までの距離から、非対称な（左右で異なりうる）道幅制約を
        求める（`_lateral_bounds`参照）。`width`が渡された場合は`course`の
        有無に関わらず従来通りの対称計算を使う
    :param vehicle_half_width_m: 車体全幅の半分 [m]。壁との安全マージンぶん、
        道幅の半分より内側にしかオフセットできないようにする
    :param safety_margin_m: `vehicle_half_width_m`に加えて残す余裕 [m]
    :param course: `width is None`のときに壁までの実測レイキャストに使う`Course`。
        `width`が渡されているときは無視される
    :param obstacles: `Course.obstacles`と同じ（(K,3)=x,y,半径[m]、世界座標）。
        `None`（既定）なら既存の挙動と完全に同一。指定すると障害物周辺の箱制約
        `[lo,hi]`を障害物が無い側へ狭める（`_narrow_bounds_for_obstacles`参照。
        ★未使用インフラ、docstring参照）
    :param lam: `_min_curvature_alpha`にそのまま渡す正則化重み
    :param max_iter: `_min_curvature_alpha`にそのまま渡すアクティブセット法の反復上限
    """
    xy = centerline[:, :2]
    yaw = centerline[:, 2]
    lo, hi = _lateral_bounds(xy, yaw, width, course,
                             vehicle_half_width_m=vehicle_half_width_m,
                             safety_margin_m=safety_margin_m)
    if obstacles is not None and len(obstacles) > 0:
        lo, hi = _narrow_bounds_for_obstacles(xy, yaw, lo, hi, obstacles,
                                              vehicle_half_width_m=vehicle_half_width_m,
                                              safety_margin_m=safety_margin_m)

    nrm = np.column_stack((-np.sin(yaw), np.cos(yaw)))
    step_m = float(np.mean(_segment_lengths(xy)))
    alpha = _min_curvature_alpha(xy, nrm, lo, hi, lam=lam, step_m=step_m, max_iter=max_iter)

    # ★悪くなった解は返さない（`raspi/nav/raceline.py`の`optimize()`と同じ安全弁）。
    # 正則化`λ‖α‖²`があれば通常起こらないはずだが、`lam`を極端に小さくした場合や
    # 数値的な退化への保険として、厳密曲率（近似ではない`_discrete_curvature`）の
    # 二乗和で中心線(α=0)と比較し、改善していなければ中心線をそのまま返す
    energy_center = float(np.sum(_discrete_curvature(xy) ** 2))
    energy_alpha = float(np.sum(_discrete_curvature(xy + alpha[:, None] * nrm) ** 2))
    if energy_alpha >= energy_center:
        return np.zeros_like(alpha)
    return alpha


def compute_raceline_xy(centerline: np.ndarray, width: float | np.ndarray | None, *,
                        vehicle_half_width_m: float, safety_margin_m: float = 0.03,
                        obstacles: np.ndarray | None = None, course: "Course | None" = None,
                        lam: float = _LAM_DEFAULT,
                        max_iter: int = _ACTIVE_SET_MAX_ITER) -> np.ndarray:
    """`compute_raceline_offsets()`を呼び、理想ラインの世界座標`(N,2)`
    （`centerline + offset*normal`）に変換したものを返す。引数は`compute_raceline_offsets`と
    同じ——呼び出し側（`ml_lidar/env.py`の報酬計算、`ml_lidar/watch.py`・`live_watch.py`の
    観戦描画）で同じ`offset→normal→xy`変換を重複させないための薄いラッパー。
    """
    offsets = compute_raceline_offsets(centerline, width, vehicle_half_width_m=vehicle_half_width_m,
                                       safety_margin_m=safety_margin_m, obstacles=obstacles,
                                       course=course, lam=lam, max_iter=max_iter)
    yaw = centerline[:, 2]
    normal = np.column_stack((-np.sin(yaw), np.cos(yaw)))
    return centerline[:, :2] + offsets[:, None] * normal


def compute_speed_profile(centerline: np.ndarray, offsets: np.ndarray, *, mu: float,
                          max_speed: float, drive_accel_m_s2: float, brake_decel_m_s2: float,
                          default_drive_accel_m_s2: float = DEFAULT_DRIVE_ACCEL_M_S2) -> np.ndarray:
    """理想ライン上の点ごとの目標速度 [m/s]。曲率ベースのグリップ限界速度を、
    加速度上限(前進パス)・減速度上限(後退パス)で挟んで滑らかにする——実車の
    レーシングラインの基本則（タイトコーナーの手前で早めに減速し、立ち上がりは
    グリップ限界いっぱいまで踏んで加速する）を近似する古典的な3段アルゴリズム。

    :param mu: このエピソードの摩擦係数（`episode_spec.mu`、ドメインランダム化後の
        値を渡すこと——固定`spec.mu`を使うと`randomize_dynamics=True`時に
        今エピソードのグリップと目標速度がズレる）
    :param drive_accel_m_s2: 実測の最大加速度 [m/s²]。未実測(0.0)なら
        `default_drive_accel_m_s2`にフォールバック（`sim/vehicle.py:_next_speed()`
        と同じフォールバックの思想）
    :param brake_decel_m_s2: 実測の最大減速度 [m/s²]。未実測(0.0)なら
        グリップ限界`mu*g`そのものを制動側の床として使う
    """
    yaw = centerline[:, 2]
    normal = np.column_stack((-np.sin(yaw), np.cos(yaw)))
    xy = centerline[:, :2] + offsets[:, None] * normal

    a_lat_max = max(mu, 1e-6) * GRAVITY_MPS2
    curvature = _discrete_curvature(xy)
    with np.errstate(divide="ignore"):
        v_curve = np.sqrt(a_lat_max / np.maximum(np.abs(curvature), 1e-6))
    v = np.minimum(v_curve, max_speed)

    drive_a = drive_accel_m_s2 if drive_accel_m_s2 > 1e-6 else default_drive_accel_m_s2
    brake_a = brake_decel_m_s2 if brake_decel_m_s2 > 1e-6 else a_lat_max

    seg = _segment_lengths(xy)
    n = len(v)
    for _ in range(_SPEED_PASS_LAPS):
        for i in range(n):
            j = i - 1              # 直前の点（負インデックスで自動的に周回する）
            v_reach = math.sqrt(v[j] ** 2 + 2.0 * drive_a * seg[j])
            if v[i] > v_reach:
                v[i] = v_reach
        for i in range(n - 1, -1, -1):
            j = (i + 1) % n         # 直後の点
            v_reach = math.sqrt(v[j] ** 2 + 2.0 * brake_a * seg[i])
            if v[i] > v_reach:
                v[i] = v_reach
    return v
