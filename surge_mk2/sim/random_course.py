"""ランダムな閉ループコースの生成 — RL 訓練のコース多様性確保用。

`sim/editor.py` のような「直線/円弧を手でつないで閉じる」方式は自動生成に向かない
（総回頭が360の倍数になるよう組み合わせを選ぶ必要があり、機械的に閉じるのが難しい）。
代わりに**円周上のランダム半径点列**を多角形として作り、各頂点を**半径が保証された
真円のフィレット**（CADの角丸めと同じ）で置き換える。円周上の関数として多角形を
作っているので、始点と終点は常にぴったり閉じる（`close_gap()`のような補正が
要らない）——この「閉じる」性質は保ったまま、角の丸め方だけを書き直した。

## ★ 車両が物理的に曲がりきれないコーナーを作らない（2026-08-29、書き直し）

最初の実装は`smooth_loop()`（移動平均によるぼかし）で角を丸めていた。これは
「どれだけ均すか」という**ぼかしの強さ**は指定できても、**結果の実際の半径が
いくつになるか**を制御できない——実測したところ、生成コースは平均で全長の約1割・
最大18%が車両の幾何学的な最小旋回半径（ホイールベースと最大舵角から
`R_min = L / tan(max_steer)`、現行値で約0.40m）を下回っており、**車両がどう
操舵しても壁に接触せずには曲がれない区間**を作っていた（バンビの指摘で発覚）。
衝突は方策の巧拙と無関係な「避けようのない失敗」として`collision_penalty`に
乗るため、報酬信号にノイズ/バイアスを持ち込んでいた。

書き直した現在の方式は、多角形の各頂点の方位変化`δ`から接線長
`t = R * tan(|δ| / 2)`（CADの角丸めの標準公式）を計算し、`straight`/`arc`の
セグメント列を組み立てて`sim/track.py`の`centerline()`（`fuji`/`circuit`など
手作りコースと全く同じ経路）にそのまま渡す。半径`R`を明示的に指定するので、
「ぼかしの強さ」ではなく「結果の半径」を直接保証できる。

接線長は隣接する辺の長さの`_MAX_EDGE_FRAC`（既定0.4）までしか食い込まない
よう頭打ちにしてある（両側の頂点が同じ辺を分け合っても`2*0.4=0.8<1`なので、
辺の一部は必ず直線として残る——直線区間が消える2026-08-28の再発防止）。
この頭打ちにより、頂点の方位変化が急峻すぎる場合は`min_turn_radius_m`を
達成できないことがあるため、**コース全体の最小旋回半径を検査し、割り込んで
いたら多角形を引き直す**（`_MAX_GEN_ATTEMPTS`回まで）。実測では平均1.1回・
最大3回で成功する——旧方式で同じ基準で50回試しても1回も成功しなかったのとは
対照的。フィレット半径自体は`min_turn_radius_m`〜`2.5`倍でコーナーごとに
ランダムに振る（タイトなコーナーと緩いコーナーが混在する。`fuji`が
0.5m/1.0m/2.0mの異なる半径を使い分けているのに近づける狙い）。

`_add_chicanes()`の局所的なS字蛇行（`fuji`のシケイン区間に近い性格）も同じ理由で
最小旋回半径を割り込みうるため、正弦変位の曲率上限（`半径 ≈ 窓幅^2 / (振幅 * π^2)`）
から安全な窓幅を逆算するようにした。蛇行は窓の両端で変位0になるように作るので、
ループの閉じ方には一切影響しない（法線方向の局所的な足し算で済む）。
"""

from __future__ import annotations

import math
from functools import lru_cache
from pathlib import Path

import numpy as np

from raspi.nav.centerline import resample_loop

from .course import Course
from .track import centerline as _track_centerline
from .track import rasterize
from .vehicle import VehicleSpec

__all__ = ["generate_random_course", "generate_random_course_dr"]

#: 車両の幾何学的な最小旋回半径に掛ける安全マージン。数値誤差やLiDARノイズの
#: 余地をゼロにしないため、ぴったり`R_min`は狙わない
_RADIUS_MARGIN = 1.3
#: コーナーごとのフィレット半径をランダムに振る上限（`min_turn_radius_m`の倍率）。
#: タイトなコーナーと緩いコーナーを混在させる（`fuji`が複数の半径を使い分けるのに近づける）
_RADIUS_MAX_MULT = 2.5
#: 1頂点のフィレットが片側の辺から食い込んでよい長さの割合。両側の頂点が同じ辺を
#: 食い合っても`2*_MAX_EDGE_FRAC < 1`なので、辺の一部は必ず直線として残る
_MAX_EDGE_FRAC = 0.4
#: 頂点の方位変化がこれ未満なら「実質直線」としてフィレットを作らない
_MIN_TURN_RAD = math.radians(1.0)
#: 車両が曲がりきれるコースが得られるまでの多角形の引き直し回数の上限。
#: 実測では平均1.1回・最大3回で成功するため、20でも5000本中2本(0.04%)しか
#: 割り込みが残らなかったが、1本あたり1ms程度と軽いので更に余裕を持たせてある
_MAX_GEN_ATTEMPTS = 50
#: フィレットの真円区間を`sim/track.py`の`centerline()`で刻む間隔 [m]
_ARC_STEP = 0.05


@lru_cache(maxsize=1)
def _vehicle_min_turn_radius_m() -> float:
    """車両の幾何学的な最小旋回半径 [m]（自転車モデル、`R = L / tan(max_steer)`）。

    `generate_random_course()`は`course_fn: Callable[[rng], Course]`という1引数の
    契約（`SimE2EEnv`が`course_fn(self.rng)`で呼ぶ、`sim/gym_env.py`参照）で毎
    エピソード呼ばれる（学習全体で数万〜数十万回オーダー）ため、そのたびに
    `config/vehicle.toml`を読み直さないよう1プロセス内でキャッシュする
    （`vehicle.toml`はプロセス起動後に変わらない前提）。
    """
    spec = VehicleSpec.load()
    return spec.wheelbase / math.tan(spec.max_steer)


def _min_turn_radius_m(xy: np.ndarray) -> float:
    """閉ループの点列から、実現されている旋回半径の最小値 [m] を測る。"""
    loop = np.vstack([xy, xy[:1]])
    seg = np.hypot(*np.diff(loop, axis=0).T)
    yaw = np.arctan2(np.diff(loop[:, 1]), np.diff(loop[:, 0]))
    dyaw = np.diff(np.unwrap(np.concatenate([yaw, yaw[:1]])))
    curv = np.abs(dyaw) / np.maximum(seg, 1e-6)
    return float(1.0 / max(float(curv.max()), 1e-9))


def _filleted_polygon_xy(rng: np.random.Generator, min_turn_radius_m: float) -> np.ndarray:
    """半径が保証されたフィレット付きの多角形の点列（x, y のみ）を1本作る。"""
    n_ctrl = int(rng.integers(4, 9))                    # 角の数。少ないほど直線が際立つ
    base_r = float(rng.uniform(1.8, 4.5))
    jaggedness = float(rng.uniform(0.15, 0.5))          # 半径のばらつき（コースごとに変える）
    radius_range = (base_r * (1 - jaggedness), base_r * (1 + jaggedness))

    angles = np.linspace(0.0, 2 * math.pi, n_ctrl, endpoint=False)
    radii = rng.uniform(radius_range[0], radius_range[1], n_ctrl)
    verts = np.column_stack((radii * np.cos(angles), radii * np.sin(angles)))

    nxt = np.roll(verts, -1, axis=0)
    edge = nxt - verts
    elen = np.hypot(edge[:, 0], edge[:, 1])
    edir = edge / elen[:, None]
    prev_dir = np.roll(edir, 1, axis=0)                 # 各頂点への入射方向
    cross = prev_dir[:, 0] * edir[:, 1] - prev_dir[:, 1] * edir[:, 0]
    dot = prev_dir[:, 0] * edir[:, 0] + prev_dir[:, 1] * edir[:, 1]
    delta = np.arctan2(cross, dot)                      # 各頂点の符号付き方位変化（反時計回り正）

    # ★接線長 t = R*tan(|δ|/2)（CADの角丸めの標準公式）。辺の長さの`_MAX_EDGE_FRAC`
    # までで頭打ちにし、直線区間を必ず残す
    t = np.zeros(n_ctrl)
    fillet_r = np.zeros(n_ctrl)
    for i in range(n_ctrl):
        if abs(delta[i]) < _MIN_TURN_RAD:
            continue
        r_i = float(rng.uniform(min_turn_radius_m, min_turn_radius_m * _RADIUS_MAX_MULT))
        t_want = r_i * math.tan(abs(delta[i]) / 2)
        t_cap = _MAX_EDGE_FRAC * min(elen[i - 1], elen[i])
        t[i] = min(t_want, t_cap)
        fillet_r[i] = t[i] / math.tan(abs(delta[i]) / 2) if t[i] < t_want else r_i

    start = verts[0] + t[0] * edir[0]
    start_yaw = math.atan2(edir[0, 1], edir[0, 0])

    path: list[tuple] = []
    for i in range(n_ctrl):
        j = (i + 1) % n_ctrl
        straight_len = elen[i] - t[i] - t[j]
        if straight_len > 1e-6:
            path.append(("straight", float(straight_len)))
        if abs(delta[j]) >= _MIN_TURN_RAD:
            path.append(("arc", float(fillet_r[j]), math.degrees(delta[j])))

    pts = _track_centerline(path, float(start[0]), float(start[1]), start_yaw, step=_ARC_STEP)
    return pts[:, :2]


def generate_random_course(rng: np.random.Generator, *, name: str = "rand",
                           width: float = 1.0, resolution: float = 0.02,
                           final_step: float = 0.1,
                           min_turn_radius_m: float | None = None) -> Course:
    """ランダムな閉ループの中心線から `Course` を組み立てて返す。

    :param min_turn_radius_m: 生成するコーナーの半径の下限 [m]。省略時は
        `config/vehicle.toml`から計算した車両の幾何学的な最小旋回半径に
        安全マージン（`_RADIUS_MARGIN`倍）を掛けた値——詳細はモジュール
        docstring参照

    最小旋回半径の**厳密な**保証はしない（頂点の方位変化が辺の長さに対して
    急峻すぎる病的なケースでは、`_MAX_GEN_ATTEMPTS`回引き直しても割り込みが
    残ることがある）。ただし実測では平均1.1回・最大3回で成功しており、
    旧方式（保証なし、平均で全長の1割が違反）とは実害の水準が異なる。
    """
    r_min = (min_turn_radius_m if min_turn_radius_m is not None
            else _vehicle_min_turn_radius_m() * _RADIUS_MARGIN)

    xy = np.zeros((0, 2))
    for attempt in range(_MAX_GEN_ATTEMPTS):
        base_xy = _filleted_polygon_xy(rng, r_min)
        chi_xy = _add_chicanes(rng, base_xy, n=int(rng.integers(0, 3)), min_radius_m=r_min)
        xy = resample_loop(chi_xy, final_step)
        if _min_turn_radius_m(xy) >= r_min or attempt == _MAX_GEN_ATTEMPTS - 1:
            break

    # ★周回方向のランダム化（2026-08-29追加、バンビの指摘で発覚）。頂点の角度を
    # 常に単調増加（`_filleted_polygon_xy`の`np.linspace`）で辿るため、周回方向は
    # 常に反時計回り（数学の慣例、ROS規約の正方向）に固定されていた——生成コースが
    # 全て同じ回転方向で、`fuji`（時計回り）とは逆だった。点列の順序を丸ごと逆転
    # させれば、同じ壁の形状のまま走行方向だけが逆になる（`rasterize()`は点の順序に
    # 依存しない円盤の重ね塗りなので、逆転しても壁の形は変わらない）
    if rng.random() < 0.5:
        xy = xy[::-1].copy()

    nxt = np.roll(xy, -1, axis=0)
    yaw = np.arctan2(nxt[:, 1] - xy[:, 1], nxt[:, 0] - xy[:, 0])
    centerline = np.column_stack((xy, yaw))

    grid, origin = rasterize(centerline, width, resolution)
    start = (float(centerline[0, 0]), float(centerline[0, 1]), float(centerline[0, 2]))

    return Course(name=name, path=Path(f"<random:{name}>"), resolution=resolution,
                 origin=origin, start=start, grid=np.ascontiguousarray(grid),
                 centerline=centerline, width=width)


def generate_random_course_dr(rng: np.random.Generator, *, name: str = "rand",
                              width_range: tuple[float, float] = (0.7, 1.3),
                              **kwargs) -> Course:
    """`generate_random_course()`に**道幅のドメインランダム化**を足した薄いラッパー。

    `circuit`/`fuji`（評価用コース）も既存の学習コース生成もこれまで幅1.0m固定
    だったため、E2E方策が「幅1.0mでの安全マージン・速度感覚」に過学習し、
    大会コースの狭い/広い区間に汎化できないおそれがあった（2026-08-28、
    バンビの指摘）。`GymSurgeEnv`の`course_fn`は`Callable[[rng], Course]`の形しか
    受け取らないので、幅を乱数で決めてから`generate_random_course()`に渡すだけの
    ラッパーとして分離してある（形状生成そのものは変えない）。

    :param width_range: 幅 [m] の一様分布の範囲。既定 0.7〜1.3m は現状の固定値
        1.0m を挟む暫定レンジ（大会コースの実測値ではない）。車体トレッド0.155m・
        全幅0.18mに対しては下限0.7mでも十分な余裕がある。
    """
    width = float(rng.uniform(*width_range))
    return generate_random_course(rng, name=name, width=width, **kwargs)


def _add_chicanes(rng: np.random.Generator, xy: np.ndarray, *, n: int,
                  min_radius_m: float, amplitude_range: tuple[float, float] = (0.25, 0.6)
                  ) -> np.ndarray:
    """局所的なS字の蛇行を `n` 箇所ランダムに挿入する。**窓の両端で変位0**にして
    あるので、挿入してもループの閉じ方（始点=終点）は崩れない。

    :param min_radius_m: 蛇行が作る最もタイトな曲率でもこの半径を下回らないよう、
        窓幅を振幅から逆算する（正弦変位 `y=A sin(pi s/W)` の最大曲率は
        `A(pi/W)^2` なので、半径 `R ≈ W^2/(A*pi^2)`。安全係数1.15込みで
        `W >= 1.15 * pi * sqrt(R_min * A_max)`）。フィレットと同じ理由
        （2026-08-29、モジュールdocstring参照）
    """
    if n <= 0 or len(xy) < 12:
        return xy
    window_m = 1.15 * math.pi * math.sqrt(min_radius_m * amplitude_range[1])
    out = xy.copy()
    N = len(xy)
    seg = np.hypot(*np.diff(np.vstack([xy, xy[:1]]), axis=0).T)
    avg_step = float(seg.mean()) if seg.mean() > 1e-9 else 0.05
    half_w = max(3, int(round(window_m / avg_step / 2)))

    for _ in range(n):
        # ★法線は挿入前の座標(base)から計算する。out を直接参照すると、
        # 直前に変位させた隣接点を使って次の点の法線を出すことになり、
        # 変位が変位を呼んでねじれる（実際にこれで自己交差する蛇行が出た）
        base = out.copy()
        center = int(rng.integers(0, N))
        amp = float(rng.uniform(*amplitude_range)) * float(rng.choice([-1.0, 1.0]))
        for k in range(-half_w, half_w + 1):
            idx = (center + k) % N
            t = (k + half_w) / (2 * half_w)
            bump = amp * math.sin(math.pi * t)          # 窓の両端(t=0,1)で0
            a, b = base[(idx - 1) % N], base[(idx + 1) % N]
            tangent = b - a
            norm = math.hypot(*tangent)
            if norm < 1e-9:
                continue
            nx, ny = -tangent[1] / norm, tangent[0] / norm
            out[idx, 0] = base[idx, 0] + bump * nx
            out[idx, 1] = base[idx, 1] + bump * ny
    return out
