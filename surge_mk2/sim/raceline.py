"""理想レーシングライン — 中心線+道幅から、最小曲率に寄せたオフセットと
曲率考慮の目標速度プロファイルをオフラインで計算する。

`sim/gym_env.py` の `SimE2EEnv` が「壁に衝突しないが、コーナーでアペックスを
突く・進入前に減速するといったレーシングラインらしい走り方をしない」問題への
対策として使う（`docs/progress` 2026-09-01「ml_lidar v8→v9」参照）。方策自体は
生LiDAR+速度のE2Eのまま変えず、**学習時の報酬にだけ**理想ラインへの追従度を
組み込む（Trajectory-Aided Learning、Bosello et al. arXiv:2306.07003 に倣う）。

## 曲率最小化に反復平滑化(Laplacian的な手法)を使わなかった理由

最初は`sim/random_course.py`の`_min_turn_radius_m`と同じ「`np.roll`による
周回差分」の流儀で、隣接点の中点に寄せる反復平滑化（離散ラプラシアン平滑化）を
試した。だがこれは**曲率最小化とは逆方向に働く**——閉ループ全体に一様に
適用すると「曲線短縮フロー」そのものになり、円が一様に収縮して半径が
小さくなる（＝曲率が増える）方向に収束することが数値実験で確認できた
（実測: 生成コースで`sum(curvature^2)`が最大10倍近く悪化した）。線形近似
`κ_path ≈ κ_ref + n''`（オフセット`n`が曲率半径に対して十分小さい前提）で
解いても、実際のコースの道幅に対する余裕（オフセット/半径比が0.3〜0.5程度）
ではこの前提が崩れ、非線形項`κ_ref²`の寄与が支配的になり同様に悪化した。

代わりに**厳密な離散曲率（`atan2`ベース、線形近似なし）をそのまま目的関数にし、
`torch`の自動微分で正確な勾配を取ってbox制約付き勾配降下（Adam+射影）で
最小化する**方式にした。手で導いた線形近似・反復平滑化はいずれも符号や
非線形項の扱いを誤りやすく実測で悪化が確認されたため、正確性を優先した。
`torch`は`stable_baselines3`が要求する既存の依存で、学習プロセスには
常に読み込まれているため新規依存の追加ではない。1コース(200〜400点)・
200ステップの最適化で実測20〜40ms程度（コース生成1回=エピソード1回につき
1回だけ計算すればよく、ステップ毎の計算コストは不要）。
"""

from __future__ import annotations

import math

import numpy as np
import torch

from .vehicle import GRAVITY_MPS2

__all__ = ["compute_raceline_offsets", "compute_speed_profile"]

#: `drive_accel_m_s2`が未実測(0.0)のときに使う、加速側のフォールバック値 [m/s²]。
#: `sim/vehicle.py`の`_next_speed()`が未実測時にハードウェア仕様値へフォールバック
#: するのと同じ思想（ここでは目標速度プロファイルの見積もり用の仮値）
DEFAULT_DRIVE_ACCEL_M_S2 = 1.5

#: 前進(加速)・後退(減速)パスを何周ぶん繰り返すか。閉ループなので1周だけだと
#: 「開始点から見て終盤」の制約が「序盤」まで伝播しきらない。2周で十分収束する
#: （プロファイルは速度ボーナスの目安であり、真の最適解である必要はない）
_SPEED_PASS_LAPS = 2

#: `compute_raceline_offsets`のL-BFGS最適化1回あたりの反復回数(`max_iter`)・
#: 初期ステップ幅(`lr`。`line_search_fn="strong_wolfe"`が実際の刻み幅を決めるので
#: ほぼ効かない)。**`env.reset()`のたびに呼ばれるので速度が重要。**
#:
#: `opt.step(closure)`を1回だけ呼ぶ実装にしたところ(2026-09-01)、`max_iter`を
#: 5〜200のどれにしても**同じ(悪い)ロス値で頭打ちになり、理想ラインがほぼ
#: 中心線のまま動かない**という新しい不具合を生んだ——実測すると、
#: `torch.optim.LBFGS`は`strong_wolfe`の内部で「これ以上その回の直線探索では
#: 改善できない」と判断すると`max_iter`を使い切る前に`step()`から抜けてしまい、
#: **`step()`を続けて何度も呼び直す（quasi-Newton履歴を引き継いだまま再探索
#: させる）ことで初めて先へ進む**という、`max_iter`を増やすだけでは代替できない
#: 挙動があることが分かった（実測: circuitのロスが1回目`step()`で50.97のまま
#: 頭打ちに見えたが、`step()`をさらに9回呼び直すと22.8まで下がり、そこで初めて
#: `max|offset|`が0.01→0.38まで育った＝アペックスを突くラインになった）。
#: そのため`compute_raceline_offsets`は`step()`を`_OPT_MAX_CALLS`回まで
#: 呼び直し、ロスの改善が`_OPT_REL_TOL`を下回ったら早期終了する
_OPT_ITERATIONS = 20
_OPT_LR = 1.0
_OPT_MAX_CALLS = 8
_OPT_REL_TOL = 0.02

#: 最適化にかける点の間引き間隔 [m]。`sim/random_course.py`の`final_step`(全アーキ
#: タイプ共通0.1m)と同じ値——コースの曲率半径(数十cm〜)に対して十分細かく、
#: 間引きによる形状の劣化は無視できる。`sim/track.py`の`path`指定コース
#: (circuit/fuji)は解像度そのまま(2cm間隔)の中心線を持ち間引かれていないため、
#: 自由度(977点)が多すぎてL-BFGSの収束が遅い(実測: 5000反復・24秒)。ここで
#: 弧長ベースに間引いてから最適化し、結果を周期線形補間で密な点列に戻す
_OPT_STEP_M = 0.1


def _segment_lengths(xy: np.ndarray) -> np.ndarray:
    """`seg[i]` = 点`i`から点`i+1`（最後は点0に周回）までの距離 [m]。
    `sim/random_course.py`の`_min_turn_radius_m`と同じ周回差分の作り方。"""
    loop = np.vstack([xy, xy[:1]])
    return np.hypot(*np.diff(loop, axis=0).T)


def _discrete_curvature(xy: np.ndarray) -> np.ndarray:
    """`sim/random_course.py`の`_min_turn_radius_m`と同じ離散曲率
    （隣接セグメントのyaw差 / セグメント長）。閉ループの点`i`ごとの符号付き曲率 [1/m]。"""
    loop = np.vstack([xy, xy[:1]])
    seg = np.hypot(*np.diff(loop, axis=0).T)
    yaw = np.arctan2(np.diff(loop[:, 1]), np.diff(loop[:, 0]))
    dyaw = np.diff(np.unwrap(np.concatenate([yaw, yaw[:1]])))
    return dyaw / np.maximum(seg, 1e-6)


def _coarse_indices(xy: np.ndarray, step: float) -> np.ndarray:
    """弧長 `step` [m] おきに最も近い既存点を選んだインデックス列（昇順・重複無し）。
    新しい点を補間で作るのではなく既存インデックスを選ぶので、`yaw`（周期量で
    単純補間できない）も`xy[idx]`でそのまま引ける。"""
    seg = _segment_lengths(xy)
    s = np.concatenate([[0.0], np.cumsum(seg)])[:-1]
    total = float(s[-1] + seg[-1])
    n = max(3, int(round(total / step)))
    if n >= len(xy):
        return np.arange(len(xy))
    targets = np.linspace(0.0, total, n, endpoint=False)
    idx = np.clip(np.searchsorted(s, targets, side="left"), 0, len(xy) - 1)
    return np.unique(idx)


def _curvature_sq_sum_torch(offset: "torch.Tensor", centerline_xy: "torch.Tensor",
                            normal: "torch.Tensor") -> "torch.Tensor":
    """`_discrete_curvature`と同じ式をtorchで書き直したもの（自動微分用）。`np.unwrap`は
    微分できないので、隣接差分を`[-pi,pi]`に丸め込む等価な式（`remainder`）で代用する
    ——`np.unwrap`は元々「累積角がこの範囲を跨いだときの2πジャンプを取り除く」ためのもので、
    隣接点間隔が細かいコースでは差分自体が`[-pi,pi]`に収まるので等価。"""
    p = centerline_xy + offset.unsqueeze(1) * normal
    loop = torch.cat([p, p[:1]], dim=0)
    diff = loop[1:] - loop[:-1]
    seg = torch.hypot(diff[:, 0], diff[:, 1]).clamp_min(1e-6)
    yaw = torch.atan2(diff[:, 1], diff[:, 0])
    dyaw = torch.cat([yaw[1:], yaw[:1]]) - yaw
    dyaw = torch.remainder(dyaw + math.pi, 2 * math.pi) - math.pi
    kappa = dyaw / seg
    return torch.sum(kappa * kappa)


def compute_raceline_offsets(centerline: np.ndarray, width: float | np.ndarray | None, *,
                             vehicle_half_width_m: float, safety_margin_m: float = 0.03,
                             iterations: int = _OPT_ITERATIONS, lr: float = _OPT_LR) -> np.ndarray:
    """中心線 `(N,3)`(x,y,yaw) からの法線方向オフセット `offset[i]`（符号付き、
    法線 `(-sin(yaw), cos(yaw))` の正方向）を返す。`centerline + offset*normal`が
    理想ライン（曲率二乗和を最小化した、最小曲率に寄せたレーシングライン）の
    座標になる。

    ## Adam(射影勾配法)からL-BFGSへ変更した経緯

    当初はbox制約(道幅の範囲内)をAdam勾配降下→毎ステップ`clamp_`で解いていたが、
    実測（`circuit.json`）で隣接点の60%以上がオフセットの符号を反転させる高周波の
    ギザギザ解に陥り、しかも反復回数を200→5000に増やすと悪化する（境界張り付き点が
    13→468に増加）ことが判明した（`watch.py`で理想ラインが「毛羽立って」見えた実例、
    2026-09-01）。原因は、この目的関数（曲率二乗和）が`offset`の2階差分を含む
    梁のたわみ的な（4階微分相当の）性質を持つのに対し、Adamは各点を独立に
    ほぼ一定幅で動かすため、隣接点が逆方向に押し合うチェッカーボード状のノイズを
    減衰できないこと。`torch.optim.LBFGS`（fullbatch・2階情報を近似するquasi-Newton）
    に変えたところ、同程度の反復回数で目的関数値がAdamの1/3〜1/6まで下がり、
    境界張り付きも解消した。

    box制約は`clamp_`（射影）ではなく`offset = max_offset * tanh(z)`という
    滑らかな再パラメータ化で埋め込む——L-BFGSは内部で1回の`step()`につき
    `iterations`回まで独自に反復するため、Adamのように毎ステップ外側からclampを
    挟めない（挟むとL-BFGSの2階情報の履歴と矛盾し収束が乱れる）。`tanh`なら
    無制約最適化のまま自動的に`[-max_offset, max_offset]`に収まる。

    :param width: `Course.width`と同じ（スカラーまたは`centerline`と同じ長さの配列）
    :param vehicle_half_width_m: 車体全幅の半分 [m]。壁との安全マージンぶん、
        道幅の半分より内側にしかオフセットできないようにする
    :param safety_margin_m: `vehicle_half_width_m`に加えて残す余裕 [m]
    """
    xy = centerline[:, :2]
    yaw = centerline[:, 2]
    n_pts = len(xy)
    if isinstance(width, np.ndarray):
        half_w = width / 2.0
    else:
        half_w = np.full(n_pts, (width if width is not None else 1.0) / 2.0)
    max_offset = np.maximum(0.0, half_w - vehicle_half_width_m - safety_margin_m)

    # 間引いた点だけをL-BFGSにかける（`_OPT_STEP_M`のdocstring参照）
    idx = _coarse_indices(xy, _OPT_STEP_M)
    seg = _segment_lengths(xy)
    s_full = np.concatenate([[0.0], np.cumsum(seg)])[:-1]
    total = float(s_full[-1] + seg[-1])

    xy_c, yaw_c, max_offset_c = xy[idx], yaw[idx], max_offset[idx]
    centerline_t = torch.from_numpy(xy_c).to(torch.float64)
    normal_t = torch.from_numpy(np.column_stack((-np.sin(yaw_c), np.cos(yaw_c)))).to(torch.float64)
    max_offset_t = torch.from_numpy(max_offset_c).to(torch.float64)
    z_t = torch.zeros(len(idx), dtype=torch.float64, requires_grad=True)

    opt = torch.optim.LBFGS([z_t], lr=lr, max_iter=iterations, line_search_fn="strong_wolfe")

    def closure() -> "torch.Tensor":
        opt.zero_grad()
        offset = max_offset_t * torch.tanh(z_t)
        loss = _curvature_sq_sum_torch(offset, centerline_t, normal_t)
        loss.backward()
        return loss

    # `_OPT_ITERATIONS`のdocstring参照——1回の`step()`では内部の直線探索が
    # 早期に頭打ちするため、ロスの改善が鈍るまで呼び直す
    prev_loss = None
    for _ in range(_OPT_MAX_CALLS):
        loss = opt.step(closure).item()
        if prev_loss is not None and abs(prev_loss - loss) < _OPT_REL_TOL * max(prev_loss, 1e-9):
            break
        prev_loss = loss

    offset_c = (max_offset_t * torch.tanh(z_t)).detach().numpy()
    # 間引いた点の弧長位置を基準に、密な点列へ周期線形補間で戻す
    offset = np.interp(s_full, s_full[idx], offset_c, period=total)
    # 補間の丸め込みで、道幅が急に変わる区間だけ僅かに制約を超えうるので
    # 密な点列側の`max_offset`で最終的にクランプしておく
    return np.clip(offset, -max_offset, max_offset)


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
