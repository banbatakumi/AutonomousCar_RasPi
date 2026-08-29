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

## ★ ヘアピン対応 + S字の真のS化・タイト化（2026-08-29、バンビの指摘で判明した
   汎化不足への対応）

v3(3M step目標のうち1.9Mで打ち切り・net_arch未拡張)がヘアピン/S字のあるコース
（`fuji`）でだけ衝突するのを診断したところ、**学習コース生成器がヘアピンを
原理的に一度も作っていなかった**ことが最大の要因と判明した（学習不足・
ネットワーク容量の小ささは別途の要因）。

実測で確認した事実: 「1つの中心から見た半径の関数」として多角形を作る現行方式
（`_filleted_polygon_xy`）は、1頂点の方位変化`δ`をどれだけ深い切り込み（半径を
ほぼ0に）にしても**δが約60°で頭打ちになる**（角度が単調増加する極座標表現の
構造的な限界。半径をどう振っても、隣接頂点との角度間隔で決まる上限を超えられ
ない）。fuji.jsonの実際のヘアピンは`arc,1.0,-90`のような90°アークを**直線を
挟まず2つ連続**させて作っている——1頂点で180°近くを一気に折ろうとすると
接線長`t=R*tan(|δ|/2)`が発散し現実的な辺長では実現できないため、90°ずつ2つに
分けるのが鍵だった。

`_hairpin_polygon_xy()`はこれを踏襲し、ヘアピン部分だけは**2アークの断片として
直接組み立て**（半径は他の頂点と同じ`min_turn_radius_m`〜`2.5`倍からランダム、
`_MAX_EDGE_FRAC`による頭打ちの影響を受けない——辺を共有しないため）、残りの
頂点は「方位変化(turn)を先に決め、それを閉じるための辺長を最小二乗で解く」
方式にする。`_filleted_polygon_xy`のような「位置を先に決めて方位変化を逆算する」
方式では大きなturnを直接指定できないため、逆に「turnを先に指定し位置を後から
解く」方式に切り替えている。turnの合計をちょうど360°に固定すれば方位の閉じは
保証され、位置の閉じ（始点=終点）は「各辺の目標長からの補正量のノルムを
最小化しつつ位置を閉じる」制約付き最小二乗（`δ=A^T(AA^T)^-1(-frag-A@L0)`）で
保証する——`_filleted_polygon_xy`の「極座標なので自動的に閉じる」保証の代わりに
「最小二乗を解いて閉じる」保証に置き換えた形。最初は「2辺だけを解く」単純な
連立方程式で実装したが、残り全辺のランダム長を2方向だけで打ち消す必要があり
非現実的な辺長になりやすく単発成功率0.7%止まりだった——補正を全辺に分散する
最小二乗に変えて1.5%まで改善した（実測、モジュールの性質上まだ低いため
`_HAIRPIN_MAX_ATTEMPTS`を300に確保し98%まで引き上げている）。自己交差はしない
保証がないため`_polygon_is_simple()`で検査し、閉じられない/自己交差する組み合わせは
`_HAIRPIN_MAX_ATTEMPTS`回まで引き直し、それでも失敗したら`None`を返して
呼び出し側が`_filleted_polygon_xy`にフォールバックする。

S字（`_add_chicanes`）は、以前は`sin(pi*t)`（片側だけ膨らむC字）で、docstringは
「S字」と書いていたが実際は単方向の膨らみで、fuji.jsonの実際のS字シケイン
（半径0.5m、正負両方向に折れる）ほどタイトでもなければ真のSでもなかった。
`sin(2*pi*t)`（両側に膨らむ真のS字、原点を1回横切る）に変更し、コーナーごとに
目標半径`min_radius_m*[1.05, 2.0]`倍をランダムに振って窓幅を逆算するようにした
（fujiのシケイン半径0.5m ≈ R_min*1.25倍に匹敵するタイトさまで許容）。

**★実装中に踏んだ罠（曲率の合成）**: 窓幅の安全マージン公式は「まっすぐな
基準線に蛇行を乗せる」前提で導出したもので、孤立した直線上でテストすると
実測でも常に安全側（目標より15%緩い）に倒れることを確認した。ところが実際の
`_filleted_polygon_xy`の出力（既にフィレットで丸めた頂点を含む）に適用すると、
**蛇行の挿入位置がたまたま頂点フィレットの近傍だと、base曲線側の既存曲率と
蛇行の曲率が加算されて**合成半径が目標の半分以下（実測で`min_radius_m`の
6%まで）に落ち込み、生成コースの9割以上が最小旋回半径違反になっていた
（複数のシケインの窓同士が重なる場合も同じ理由で加算される）。`_add_chicanes`
は挿入前にbase曲線側の既存曲率を調べ、**実質直線とみなせる区間
（半径`3*min_radius_m`超）にだけ**蛇行を置くよう修正済み——窓同士の重複も
同じ理由で避けている。この教訓はヘアピン側の`_hairpin_polygon_xy`には
適用不要（辺を共有しない独立断片のため合成が起きない）。
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

#: ヘアピン1本あたりの各アーク折れ角の範囲 [deg]。2つ連続で同符号に折るのが鍵
#: （1つの頂点で180°近くを一気に折ろうとすると接線長`t=R*tan(|δ|/2)`が発散し
#: 現実的な辺長では実現できない。fuji.jsonの`arc,1.0,-90`×2連続と同じ発想）
_HAIRPIN_LEG_DEG = (55.0, 88.0)
#: ヘアピンを構成する2つのアークの間に挟む直線の長さ [m]（fujiの「直線を挟まず
#: 連続するarc」に近づけるため、車両全長よりずっと短い値にする）
_HAIRPIN_GAP_M = (0.05, 0.20)
#: ヘアピンのアーク半径を`min_turn_radius_m`の何倍にするか。通常の頂点と同じ
#: `_RADIUS_MAX_MULT`(2.5倍)まで許すと、コース全体のスケール（辺長1.5〜3m）に
#: 対して半径が大きくなりすぎ、「大きく優雅に曲がる」だけの見た目になって
#: ヘアピンらしい鋭さが視覚的に消える（実測で確認——ズームすれば正しく鋭い
#: ターンだが、コース全体を見ると普通の角と区別がつかなかった）。他の頂点より
#: タイトな範囲に絞ってヘアピンらしい見た目を確保する
_HAIRPIN_RADIUS_MULT = (1.0, 1.3)
#: ヘアピン以外の辺の目標(補正前)長さ [m]。`_hairpin_polygon_xy`の最小二乗補正が
#: ここからの乖離を最小化する
_HAIRPIN_EDGE_NOMINAL_M = (1.5, 3.0)
#: 最小二乗補正後の辺長として許容する範囲 [m]。これを外れたら不採用
_HAIRPIN_EDGE_BOUNDS_M = (0.3, 8.0)
#: 生成コースの外接ボックスの一辺として許容する最大値 [m]。`_filleted_polygon_xy`
#: が作る通常コースの実測（200本、平均6.4m・p90 8.6m・最大10.9m）に合わせた上限。
#: 最小二乗解は個々の辺長を範囲内に収めても稀に全体として巨大な多角形を返す
#: ことがある（実測でp90超えの11.6m×26.1mが出た）ため、別途サイズで弾く
_HAIRPIN_MAX_EXTENT_M = 11.0
#: `_hairpin_polygon_xy`（閉じる/自己交差しない配置を探す）の内部リトライ回数上限。
#: 1回あたり0.02〜0.03ms程度と軽い（実測）ため、実測成功率(単発1.5%程度)から
#: 300回で98%・500回で100%成功するよう多めに確保してある
_HAIRPIN_MAX_ATTEMPTS = 300


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


def _fillet_path(rng: np.random.Generator, elen: np.ndarray, delta: np.ndarray,
                 min_turn_radius_m: float, *, hairpin_vertex: int | None = None,
                 hairpin_frag: list[tuple] | None = None) -> tuple[list[tuple], np.ndarray]:
    """辺長`elen`・各頂点の符号付き方位変化`delta`から、フィレット済みの
    straight/arcセグメント列を組み立てる（`_filleted_polygon_xy`と
    `_hairpin_polygon_xy`で共有）。

    `hairpin_vertex`を指定すると、その頂点だけは通常のフィレット計算
    （`t=R*tan(|δ|/2)`）をせず、`hairpin_frag`（2アークのヘアピン断片path）を
    そのまま差し込む——ヘアピンは辺を共有しない独立した断片として組み立てて
    あるため、`_MAX_EDGE_FRAC`による頭打ちを受けない（モジュールdocstring参照）。
    """
    n = len(elen)
    t = np.zeros(n)
    fillet_r = np.zeros(n)
    for i in range(n):
        if i == hairpin_vertex or abs(delta[i]) < _MIN_TURN_RAD:
            continue
        r_i = float(rng.uniform(min_turn_radius_m, min_turn_radius_m * _RADIUS_MAX_MULT))
        t_want = r_i * math.tan(abs(delta[i]) / 2)
        t_cap = _MAX_EDGE_FRAC * min(elen[i - 1], elen[i])
        t[i] = min(t_want, t_cap)
        fillet_r[i] = t[i] / math.tan(abs(delta[i]) / 2) if t[i] < t_want else r_i

    path: list[tuple] = []
    for i in range(n):
        j = (i + 1) % n
        straight_len = elen[i] - t[i] - t[j]
        if straight_len > 1e-6:
            path.append(("straight", float(straight_len)))
        if j == hairpin_vertex:
            path.extend(hairpin_frag)
        elif abs(delta[j]) >= _MIN_TURN_RAD:
            path.append(("arc", float(fillet_r[j]), math.degrees(delta[j])))
    return path, t


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

    path, t = _fillet_path(rng, elen, delta, min_turn_radius_m)
    start = verts[0] + t[0] * edir[0]
    start_yaw = math.atan2(edir[0, 1], edir[0, 0])

    pts = _track_centerline(path, float(start[0]), float(start[1]), start_yaw, step=_ARC_STEP)
    return pts[:, :2]


def _segment_intersect(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, p4: np.ndarray) -> bool:
    """線分p1-p2とp3-p4が(端点での接触ではなく)実際に交差しているか。"""
    def orient(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    d1, d2 = orient(p3, p4, p1), orient(p3, p4, p2)
    d3, d4 = orient(p1, p2, p3), orient(p1, p2, p4)
    return (d1 * d2 < 0) and (d3 * d4 < 0)


def _polygon_is_simple(verts: np.ndarray) -> bool:
    """多角形の頂点列（閉ループ）が自己交差していないかを調べる（隣接辺は除く、
    O(n^2)だが`_hairpin_polygon_xy`が扱う頂点数は数個〜10個程度なので無視できるコスト）。
    """
    n = len(verts)
    loop = np.vstack([verts, verts[:1]])
    for i in range(n):
        for j in range(i + 1, n):
            if (j + 1) % n == i or (i + 1) % n == j:
                continue                                # 隣接辺(頂点を共有)はスキップ
            if _segment_intersect(loop[i], loop[i + 1], loop[j], loop[j + 1]):
                return False
    return True


def _hairpin_polygon_xy(rng: np.random.Generator, min_turn_radius_m: float) -> np.ndarray | None:
    """ヘアピン(180°近い急な切り返し)を1箇所含む閉ループ多角形を作る。

    詳細な設計意図はモジュールdocstring参照。失敗（閉じる/自己交差しない配置が
    `_HAIRPIN_MAX_ATTEMPTS`回で見つからない）した場合は`None`を返し、呼び出し側
    （`generate_random_course`）が`_filleted_polygon_xy`にフォールバックする。
    """
    for _ in range(_HAIRPIN_MAX_ATTEMPTS):
        n_edges = int(rng.integers(4, 7))
        sign = 1.0 if rng.random() < 0.5 else -1.0
        leg1 = float(rng.uniform(*_HAIRPIN_LEG_DEG)) * sign
        leg2 = float(rng.uniform(*_HAIRPIN_LEG_DEG)) * sign
        hairpin_turn_deg = leg1 + leg2
        remaining = 360.0 - hairpin_turn_deg
        if remaining <= (n_edges - 1) * 10.0:
            continue                                    # ヘアピンが急すぎて残りが配分できない

        w = rng.uniform(15.0, 90.0, n_edges - 1)
        normal_turns_deg = w * (remaining / w.sum())
        hairpin_vertex = int(rng.integers(0, n_edges))
        turns_deg = np.insert(normal_turns_deg, hairpin_vertex, hairpin_turn_deg)
        delta = np.radians(turns_deg)                   # 合計はちょうど2π（方位の閉じを保証）

        d = np.concatenate([[0.0], np.cumsum(delta[1:])])
        edir = np.column_stack((np.cos(d), np.sin(d)))

        r_h = float(rng.uniform(min_turn_radius_m * _HAIRPIN_RADIUS_MULT[0],
                                min_turn_radius_m * _HAIRPIN_RADIUS_MULT[1]))
        gap = float(rng.uniform(*_HAIRPIN_GAP_M))
        hairpin_frag = [("arc", r_h, leg1), ("straight", gap), ("arc", r_h, leg2)]
        frag_end = _track_centerline(hairpin_frag, 0.0, 0.0, 0.0, step=_ARC_STEP)[-1]
        entry_dir = d[hairpin_vertex - 1]                # 断片に入る直前の方位
        c, s = math.cos(entry_dir), math.sin(entry_dir)
        frag_global = np.array([[c, -s], [s, c]]) @ frag_end[:2]

        # ★位置の閉じは最小二乗で解く（2辺だけを解く方式だと、残り全辺の
        # ランダム長の合計ベクトルをたった2方向だけで打ち消す必要があり、非現実的な
        # 辺長になりやすい実測結果が出た——単発成功率0.7%。目標長`L0`（全`n_edges`辺）
        # からの補正`δ`のノルムを最小化しつつ`Σ(L0+δ)*edir=-frag_global`を満たす
        # （制約付き最小二乗、`δ=A^T(AA^T)^-1(-frag_global-A@L0)`）ことで補正が
        # 全辺に分散され、単発成功率が1.5%まで改善した（実測）
        L0 = rng.uniform(*_HAIRPIN_EDGE_NOMINAL_M, n_edges)
        A = edir.T                                      # 2 x n_edges
        AAt = A @ A.T
        if abs(np.linalg.det(AAt)) < 1e-9:
            continue
        lam = np.linalg.solve(AAt, -frag_global - A @ L0)
        elen = L0 + A.T @ lam
        if not np.all((elen > _HAIRPIN_EDGE_BOUNDS_M[0]) & (elen < _HAIRPIN_EDGE_BOUNDS_M[1])):
            continue                                    # 解けても辺長が非現実的なら不採用

        verts = np.zeros((n_edges, 2))
        for i in range(1, n_edges):
            verts[i] = verts[i - 1] + elen[i - 1] * edir[i - 1]
        extent = verts.max(axis=0) - verts.min(axis=0)
        if float(extent.max()) > _HAIRPIN_MAX_EXTENT_M:
            continue                                    # 最小二乗解が稀に巨大な多角形を返す対策
        if not _polygon_is_simple(verts):
            continue

        path, t = _fillet_path(rng, elen, delta, min_turn_radius_m,
                               hairpin_vertex=hairpin_vertex, hairpin_frag=hairpin_frag)
        start = verts[0] + t[0] * edir[0]
        pts = _track_centerline(path, float(start[0]), float(start[1]), d[0], step=_ARC_STEP)
        if _min_turn_radius_m(pts[:, :2]) >= min_turn_radius_m:
            return pts[:, :2]
    return None


def generate_random_course(rng: np.random.Generator, *, name: str = "rand",
                           width: float = 1.0, resolution: float = 0.02,
                           final_step: float = 0.1,
                           min_turn_radius_m: float | None = None,
                           hairpin_prob: float = 0.35) -> Course:
    """ランダムな閉ループの中心線から `Course` を組み立てて返す。

    :param min_turn_radius_m: 生成するコーナーの半径の下限 [m]。省略時は
        `config/vehicle.toml`から計算した車両の幾何学的な最小旋回半径に
        安全マージン（`_RADIUS_MARGIN`倍）を掛けた値——詳細はモジュール
        docstring参照
    :param hairpin_prob: このコースにヘアピン(180°近い急な切り返し)を1箇所
        含めようと試みる確率。`_hairpin_polygon_xy`が失敗した場合は
        （閉じる/自己交差しない配置が見つからなかった場合）通常のフィレット
        多角形にフォールバックするため、実際にヘアピンが入る確率はこれより
        わずかに低い

    最小旋回半径の**厳密な**保証はしない（頂点の方位変化が辺の長さに対して
    急峻すぎる病的なケースでは、`_MAX_GEN_ATTEMPTS`回引き直しても割り込みが
    残ることがある）。ただし実測では平均1.1回・最大3回で成功しており、
    旧方式（保証なし、平均で全長の1割が違反）とは実害の水準が異なる。
    """
    r_min = (min_turn_radius_m if min_turn_radius_m is not None
            else _vehicle_min_turn_radius_m() * _RADIUS_MARGIN)

    hairpin_xy = _hairpin_polygon_xy(rng, r_min) if rng.random() < hairpin_prob else None

    xy = np.zeros((0, 2))
    for attempt in range(_MAX_GEN_ATTEMPTS):
        base_xy = hairpin_xy if hairpin_xy is not None else _filleted_polygon_xy(rng, r_min)
        chi_xy = _add_chicanes(rng, base_xy, n=int(rng.integers(1, 4)), min_radius_m=r_min)
        xy = resample_loop(chi_xy, final_step)
        if _min_turn_radius_m(xy) >= r_min or attempt == _MAX_GEN_ATTEMPTS - 1:
            break
        hairpin_xy = None                                # シケイン込みで条件を満たせず→通常方式へ

    # ★周回方向のランダム化（2026-08-29追加、バンビの指摘で発覚)。頂点の角度を
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
                  min_radius_m: float, amplitude_range: tuple[float, float] = (0.12, 0.35),
                  radius_mult_range: tuple[float, float] = (1.05, 2.0)) -> np.ndarray:
    """局所的な**真のS字**（曲率の符号が反転する蛇行）を `n` 箇所ランダムに挿入する。
    **窓の両端で変位0**にしてあるので、挿入してもループの閉じ方（始点=終点）は崩れない。

    ★2026-08-29書き直し: 以前は`sin(pi*t)`（片側だけ膨らむC字、片側にしか
    曲がらない）で、docstringは「S字」と書いていたが実際は単方向の膨らみ
    だった上、`fuji.json`の実際のS字シケイン（半径0.5m、正負両方向に折れる）
    ほどタイトでもなかった（バンビの指摘で判明、モジュールdocstring参照）。
    `sin(2*pi*t)`（窓の中央で符号が反転する真のS字、原点を1回横切る）に変更し、
    コーナーごとに目標半径を`min_radius_m*radius_mult_range`からランダムに
    振って窓幅を逆算するようにした（下限1.05倍は`fuji`のシケイン半径0.5m
    ≈ R_min*1.25倍に匹敵するタイトさ）。

    :param min_radius_m: 蛇行が作る最もタイトな曲率でもこの半径を下回らないよう、
        窓幅を目標半径`R`と振幅`A`から逆算する（正弦変位`y=A sin(2*pi*s/W)`の
        最大曲率は`A(2*pi/W)^2`なので、半径`R ≈ W^2/(4*pi^2*A)`。安全係数1.15込みで
        `W = 2*pi*sqrt(1.15*R*A)`）。フィレットと同じ理由（モジュールdocstring参照）
    """
    if n <= 0 or len(xy) < 12:
        return xy
    out = xy.copy()
    N = len(xy)
    loop = np.vstack([xy, xy[:1]])
    seg = np.hypot(*np.diff(loop, axis=0).T)
    avg_step = float(seg.mean()) if seg.mean() > 1e-9 else 0.05

    # ★安全マージンの窓幅公式（`真っ直ぐな基準線に乗せる`前提）は、実際には
    # 既にカーブしている`_filleted_polygon_xy`の頂点フィレット付近に蛇行を
    # 差し込むと**曲率が加算されて**想定より大幅にタイトになることが実測で
    # 判明した（合成半径が目標の半分以下に落ち込むケースがあり、生成コースの
    # 9割以上が最小旋回半径違反になっていた）。base曲線側の既存曲率が小さい
    # （＝実質直線に近い）区間だけを候補にすることで、この加算を避ける
    yaw = np.arctan2(np.diff(loop[:, 1]), np.diff(loop[:, 0]))
    dyaw = np.abs(np.diff(np.unwrap(np.concatenate([yaw, yaw[:1]]))))
    base_curv = dyaw / np.maximum(seg, 1e-6)             # index i: 頂点iの曲率 [1/m]
    straight_enough = base_curv < (1.0 / (3.0 * min_radius_m))

    # ★複数のシケインの窓が重なった場合も同じ理由で曲率が加算されうる（実測で
    # 確認）。既に置いた窓の弧長区間を記録し、新しい窓と重ならない位置が
    # 見つかるまで`center`を引き直す（見つからなければそのシケインは諦める）
    occupied: list[tuple[int, int]] = []

    def overlaps(lo: int, hi: int) -> bool:
        return any(not (hi < olo or ohi < lo) for olo, ohi in occupied)

    for _ in range(n):
        amp = float(rng.uniform(*amplitude_range))
        target_r = min_radius_m * float(rng.uniform(*radius_mult_range))
        window_m = 2 * math.pi * math.sqrt(1.15 * target_r * amp)
        half_w = max(4, int(round(window_m / avg_step / 2)))
        half_w = min(half_w, N // 2 - 1)                # 窓がコース全長を超えないように
        if half_w < 2:
            continue

        center = None
        for _try in range(40):
            cand = int(rng.integers(0, N))
            lo, hi = cand - half_w, cand + half_w
            window_idx = [(cand + k) % N for k in range(-half_w, half_w + 1)]
            if not np.all(straight_enough[window_idx]):
                continue                                # base曲線が既にカーブしている区間は避ける
            if not overlaps(lo, hi) and not overlaps(lo + N, hi + N) and not overlaps(lo - N, hi - N):
                center = cand
                break
        if center is None:
            continue                                    # 空いている場所が見つからない→このシケインは諦める
        occupied.append((center - half_w, center + half_w))

        # ★法線は挿入前の座標(base)から計算する。out を直接参照すると、
        # 直前に変位させた隣接点を使って次の点の法線を出すことになり、
        # 変位が変位を呼んでねじれる（実際にこれで自己交差する蛇行が出た）
        base = out.copy()
        for k in range(-half_w, half_w + 1):
            idx = (center + k) % N
            t = (k + half_w) / (2 * half_w)
            bump = amp * math.sin(2 * math.pi * t)      # 窓の両端(t=0,1)で0、中央(t=0.5)で符号反転
            a, b = base[(idx - 1) % N], base[(idx + 1) % N]
            tangent = b - a
            norm = math.hypot(*tangent)
            if norm < 1e-9:
                continue
            nx, ny = -tangent[1] / norm, tangent[0] / norm
            out[idx, 0] = base[idx, 0] + bump * nx
            out[idx, 1] = base[idx, 1] + bump * ny
    return out
