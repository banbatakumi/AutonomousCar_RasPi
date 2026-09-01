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

## アーキタイプの多様化（2026-08-31、バンビの指摘）

v6まで学習を進めた段階で、`watch.py`で眺めると生成コースが「似た形ばかり」に
見えるという指摘を受けた。原因は単純で、生成器が終始1種類
（`_filleted_polygon_xy`のランダム多角形＋ヘアピン＋シケイン）しかなく、
パラメータの範囲内でしか多様性が出ないため。`generate_random_course`の本体
（多角形生成→ヘアピン差し込み→シケイン→旋回半径チェック→周回方向ランダム化）を
`_build_loop()`に切り出し、`_filleted_polygon_xy`の`n_ctrl`/`base_r`/`jaggedness`
範囲と`_add_chicanes`の個数をプリセットとして差し替えるだけで
**circuit**（頂点数・ヘアピン/シケイン頻度を上げた複雑なコース）・**corridor**
（頂点数を4つ=矩形相当に固定し半径を大きく取った、直線区間が支配的なコース）
の2アーキタイプを追加した——新しい幾何アルゴリズムは要らなかった。

**narrow**（急な道幅変化）と**obstacle**（障害物）は`track.py`の`add_offset_discs()`
（`rasterize()`の`divider`——「掘ってから立てる」——を、中心線上に限らず任意の
横オフセット位置に一般化したもの）を共通の土台にする。narrowは中心線の
**両側**対称に壁を追加して局所的に道幅を狭め、その狭まった実際の道幅を
`Course.width`の配列（centerlineと同じ長さ）として持たせる——`sim/gym_env.py`の
`_CenterlineProgress`がコース全体でスカラー1個の道幅から横偏差の余白`margin`を
計算する設計だったため、道幅が場所によって変わるコースをそのまま入れると
「狭い区間で罰が甘く、広い区間で罰が辛い」不整合が起きる。これを避けるため
`_CenterlineProgress`側も最近傍点のインデックスを使って位置ごとの道幅を
参照するよう改修した（`sim/gym_env.py`参照）。

obstacleは中心線から**片側だけ**オフセットした孤立円盤（左右どちらの壁にも
接触しない、浮いた障害物）を置く。衝突判定・LiDARは素の`grid`だけを見るので
障害物への追加コードは不要——`add_offset_discs`で壁を足すだけで両方に効く。
反対側には常に車両が通れる余裕（`_vehicle_half_width_m()`＋余裕）を残すよう
オフセットの上限を計算する。**初回実装では「回避のための横偏差ペナルティ免除」は
入れていない**——効果が不透明な割に複雑さが増すため、まずは物理障害物のみで
学習挙動を見てから要否を判断する方針（Plan agentのレビューを踏まえた判断）。

### ★ corridorの作り直し（同日、v7学習前のバンビのレビューで判明）

`watch.py`で実際に眺めたバンビから「直線のコースが毎回同じ菱形にしか見えない」
との指摘。原因は`_filleted_polygon_xy`の頂点配置が`np.linspace(0, 2π, n_ctrl,
endpoint=False)`で**常に同じ角度**（4頂点なら0/90/180/270°）に固定されている
ことで、半径をどれだけランダムに振っても「向き・アスペクト比が変わらない
菱形」から抜け出せなかった（corridorはjaggedness≈0で使っていたのでなおさら
症状が顕著だった）。

`generate_corridor_course`を`_build_loop`経由から、`straight`/`arc`セグメントを
直接組み立てる専用構築（`_stadium_path`＝直線2本+半円2つのオーバル、
`_rectangle_path`＝直線4本+直角4つの角丸長方形、半々でランダムに選ぶ）に
作り直した。対辺の長さ・半径をそれぞれ揃えるだけで対称性から自動的に閉じるため、
`_build_loop`の「引き直して確認」ループが不要になった。開始向き
（`start_yaw`）もランダム化し、同じアスペクト比でも見た目の向きが揃わない
ようにした。「ラジコンの練習コースのような、板で仕切っただけの直線区間」という
バンビの意図に合わせ、旋回半径は直線長に対してタイトな範囲
（`_CORRIDOR_RADIUS_MULT`）に絞ってある。

あわせて`ml_lidar/watch.py`にobstacle/narrowの可視化ハイライトを追加した
（`Course.obstacles`を赤丸、`Course.width`配列の道幅が狭い区間の中心線点を
オレンジ丸で上書き描画）。壁と同系色のグレーで焼き込まれるだけだと、
サムネイル解像度では肉眼でほぼ判別できなかったため（`_draw_panel`参照）。
"""

from __future__ import annotations

import math
from functools import lru_cache
from pathlib import Path

import numpy as np

from raspi.nav.centerline import resample_loop

from .course import Course
from .track import add_offset_discs
from .track import centerline as _track_centerline
from .track import rasterize
from .vehicle import VehicleSpec

__all__ = [
    "generate_random_course", "generate_random_course_dr",
    "generate_circuit_course", "generate_corridor_course",
    "generate_narrow_course", "generate_obstacle_course",
    "generate_diverse_course", "CurriculumCourseFn",
]

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


@lru_cache(maxsize=1)
def _vehicle_half_width_m() -> float:
    """車体の全幅の半分 [m]（footprintのy座標の絶対値の最大）。`obstacle`
    アーキタイプが浮遊障害物の反対側に残す余裕の計算に使う。
    `_vehicle_min_turn_radius_m()`と同じ理由でキャッシュする。
    """
    fp = VehicleSpec.load().footprint
    return max(abs(p[1]) for p in fp)


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


def _filleted_polygon_xy(rng: np.random.Generator, min_turn_radius_m: float, *,
                         n_ctrl_range: tuple[int, int] = (4, 9),
                         base_r_range: tuple[float, float] = (1.8, 4.5),
                         jaggedness_range: tuple[float, float] = (0.15, 0.5)) -> np.ndarray:
    """半径が保証されたフィレット付きの多角形の点列（x, y のみ）を1本作る。

    :param n_ctrl_range: 角の数の範囲（`rng.integers`、少ないほど直線が際立つ）
    :param base_r_range: 基準半径の範囲 [m]
    :param jaggedness_range: 半径のばらつきの範囲（コースごとに変える）。
        `circuit`/`corridor`アーキタイプ（モジュールdocstring参照）はここを
        差し替えるだけで実現している
    """
    n_ctrl = int(rng.integers(*n_ctrl_range))            # 角の数。少ないほど直線が際立つ
    base_r = float(rng.uniform(*base_r_range))
    jaggedness = float(rng.uniform(*jaggedness_range))   # 半径のばらつき（コースごとに変える）
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


def _build_loop(rng: np.random.Generator, *, min_turn_radius_m: float | None = None,
                hairpin_prob: float = 0.35, final_step: float = 0.1,
                n_ctrl_range: tuple[int, int] = (4, 9),
                base_r_range: tuple[float, float] = (1.8, 4.5),
                jaggedness_range: tuple[float, float] = (0.15, 0.5),
                chicane_n_range: tuple[int, int] = (1, 4)) -> tuple[np.ndarray, float]:
    """閉ループ中心線 `(N,3)`(x, y, yaw) を組み立てる本体。`generate_random_course`
    から切り出した（2026-08-31、コース多様化）——形状パラメータのプリセットを
    差し替えるだけで`circuit`/`corridor`アーキタイプを作れるようにするため
    （モジュールdocstring「アーキタイプの多様化」参照）。挙動は元の
    `generate_random_course`と完全に同一（既定値は元のハードコード値のまま）。

    :param min_turn_radius_m: 生成するコーナーの半径の下限 [m]。省略時は
        `config/vehicle.toml`から計算した車両の幾何学的な最小旋回半径に
        安全マージン（`_RADIUS_MARGIN`倍）を掛けた値——詳細はモジュール
        docstring参照
    :param hairpin_prob: このコースにヘアピン(180°近い急な切り返し)を1箇所
        含めようと試みる確率。`_hairpin_polygon_xy`が失敗した場合は
        （閉じる/自己交差しない配置が見つからなかった場合）通常のフィレット
        多角形にフォールバックするため、実際にヘアピンが入る確率はこれより
        わずかに低い
    :returns: `(centerline, 実際に使った最小旋回半径 [m])`

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
        base_xy = hairpin_xy if hairpin_xy is not None else _filleted_polygon_xy(
            rng, r_min, n_ctrl_range=n_ctrl_range, base_r_range=base_r_range,
            jaggedness_range=jaggedness_range)
        chi_xy = _add_chicanes(rng, base_xy, n=int(rng.integers(*chicane_n_range)),
                               min_radius_m=r_min)
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
    return np.column_stack((xy, yaw)), r_min


def generate_random_course(rng: np.random.Generator, *, name: str = "rand",
                           width: float = 1.0, resolution: float = 0.02,
                           final_step: float = 0.1,
                           min_turn_radius_m: float | None = None,
                           hairpin_prob: float = 0.35) -> Course:
    """ランダムな閉ループの中心線から `Course` を組み立てて返す（`organic`
    アーキタイプ。引数の意味は`_build_loop`のdocstring参照）。
    """
    centerline, _ = _build_loop(rng, min_turn_radius_m=min_turn_radius_m,
                                hairpin_prob=hairpin_prob, final_step=final_step)
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


# ══════════════════════════════════════════════════════════════════════════
# アーキタイプの多様化（モジュールdocstring「アーキタイプの多様化」参照）
# ══════════════════════════════════════════════════════════════════════════

#: circuit アーキタイプ: 頂点数・ヘアピン/シケイン頻度を上げ、複雑に曲がりくねった
#: コースにする
_CIRCUIT_N_CTRL_RANGE = (8, 15)
_CIRCUIT_BASE_R_RANGE = (3.0, 6.0)
_CIRCUIT_JAGGEDNESS_RANGE = (0.25, 0.55)
_CIRCUIT_CHICANE_N_RANGE = (2, 5)
_CIRCUIT_HAIRPIN_PROB = 0.6

#: corridor アーキタイプ: 直線区間の長さ [m]（ラジコンの練習コースのような、
#: 板で仕切っただけの直線区間が主体のコースにする狙い）
_CORRIDOR_STRAIGHT_RANGE_M = (2.5, 8.0)
#: corridor: 曲がる場所（直線の両端だけ）の半径を`min_turn_radius_m`の何倍にするか。
#: 直線に対してタイトな範囲に絞ることで、「大きく緩やかに曲がる」ではなく
#: 「直線の両端で鋭く折り返す」見た目にする
_CORRIDOR_RADIUS_MULT = (1.0, 1.8)
#: corridor: 旋回半径は対向直線の中心線間隔の半分そのものなので、道幅の半分
#: より小さいと内側の「島」が対向車線と合体してしまう（`_corridor_turn_radius`
#: docstring参照）。半径の下限を`道幅の半分 + この値`まで引き上げて、島の壁の
#: 厚みを最低でもこの2倍は確保する
_CORRIDOR_LANE_GAP_M = 0.15

#: narrow アーキタイプ: 1本あたりの狭め区間の弧長 [m]
_NARROW_LEN_RANGE = (0.5, 1.2)
#: narrow: 狭める先の道幅を元の道幅の何倍にするか
_NARROW_FRAC_RANGE = (0.35, 0.6)
#: narrow: 狭めた後の道幅がここを下回らないようにする下限 [m]。
#: `nominal_width * frac`だけで決めると（幅0.7m×frac0.35≈0.245m）車体全幅
#: (`2*_vehicle_half_width_m()`≈0.18m)に対して余裕が無さすぎ、方策の巧拙と無関係な
#: 「そもそも通り抜けようがない」区間ができてしまう（`min_turn_radius_m`と同じ理由で
#: 報酬信号にノイズを持ち込む。モジュールdocstring参照）
_NARROW_MIN_CLEARANCE_M = 0.20

#: obstacle アーキタイプ: 円盤障害物の半径 [m]
_OBSTACLE_RADIUS_RANGE = (0.08, 0.15)
#: obstacle: 障害物の反対側に必ず残す、車両が通れる余裕 [m]
#: （`_vehicle_half_width_m()`≈0.09mに対する余裕）
_OBSTACLE_CLEARANCE_M = 0.16

#: `generate_diverse_course`が各アーキタイプを選ぶ重み（比率だけが意味を持つ、
#: 正規化して使う）。学習分布のバランスを変えたいときはここを調整する
#: （`hairpin_prob`等と同じ、モジュール定数によるチューニングの流儀）
_ARCHETYPE_WEIGHTS = {
    "organic": 0.35,
    "circuit": 0.20,
    "corridor": 0.15,
    "narrow": 0.15,
    "obstacle": 0.15,
}


def generate_circuit_course(rng: np.random.Generator, *, name: str = "rand-circuit",
                            width: float = 1.0, resolution: float = 0.02,
                            final_step: float = 0.1,
                            min_turn_radius_m: float | None = None) -> Course:
    """頂点数・ヘアピン/シケイン頻度を上げた、サーキット風の複雑なコース。
    幾何アルゴリズム自体は`_build_loop`（`_filleted_polygon_xy`等）と同じで、
    パラメータ範囲だけを変えたプリセット（モジュールdocstring参照）。
    """
    centerline, _ = _build_loop(rng, min_turn_radius_m=min_turn_radius_m,
                                hairpin_prob=_CIRCUIT_HAIRPIN_PROB,
                                n_ctrl_range=_CIRCUIT_N_CTRL_RANGE,
                                base_r_range=_CIRCUIT_BASE_R_RANGE,
                                jaggedness_range=_CIRCUIT_JAGGEDNESS_RANGE,
                                chicane_n_range=_CIRCUIT_CHICANE_N_RANGE,
                                final_step=final_step)
    grid, origin = rasterize(centerline, width, resolution)
    start = (float(centerline[0, 0]), float(centerline[0, 1]), float(centerline[0, 2]))
    return Course(name=name, path=Path(f"<random:{name}>"), resolution=resolution,
                 origin=origin, start=start, grid=np.ascontiguousarray(grid),
                 centerline=centerline, width=width)


def _corridor_turn_radius(rng: np.random.Generator, r_min: float, half_width: float) -> float:
    """corridorの旋回半径を、`r_min`基準の範囲と「対向直線と合体しない」下限
    （`half_width + _CORRIDOR_LANE_GAP_M`）の両方を満たすように決める。

    ★2026-08-31発覚（バンビの「同じようなバグがないか」という指摘で調査して
    判明）: 旋回半径は対向する直線同士の**中心線間隔の半分**そのもの
    （stadiumなら直線間隔ちょうど`2r`、rectangleの角でも同じ）。ここを
    `r_min`だけを基準に決め、`width`（道幅）と無関係にしていたため、
    道幅が広い・半径が小さい組み合わせ（実測で半径0.52〜0.94mに対し道幅は
    最大1.3m＝半分0.65m）だと**半径が道幅の半分を下回り、内側の「島」ごと
    掘り取られて対向車線と合体する**（stadiumで実測約5.7%の頻度）。
    「板で仕切ったコース」のはずが、部分的に壁の無い広場になってしまっていた。
    """
    lo = max(r_min * _CORRIDOR_RADIUS_MULT[0], half_width + _CORRIDOR_LANE_GAP_M)
    hi = max(r_min * _CORRIDOR_RADIUS_MULT[1], lo * 1.3)
    return float(rng.uniform(lo, hi))


def _stadium_path(rng: np.random.Generator, r_min: float, half_width: float) -> list[tuple]:
    """直線2本＋半円2つ（180°ずつ）で作る、陸上トラック型（オーバル）の経路。
    対辺の直線・半円がそれぞれ同じ長さ/半径なので、対称性だけで必ず閉じる
    （`_filleted_polygon_xy`のような「引き直して確かめる」検査は不要）。
    """
    r = _corridor_turn_radius(rng, r_min, half_width)
    length = float(rng.uniform(*_CORRIDOR_STRAIGHT_RANGE_M))
    return [("straight", length), ("arc", r, 180.0),
            ("straight", length), ("arc", r, 180.0)]


def _rectangle_path(rng: np.random.Generator, r_min: float, half_width: float) -> list[tuple]:
    """直線4本＋直角4つ（90°ずつ）で作る、角丸長方形の経路。対辺の長さが
    等しく半径も共通なので、標準的なレーストラック矩形の構成として必ず閉じる。
    """
    r = _corridor_turn_radius(rng, r_min, half_width)
    lw = float(rng.uniform(*_CORRIDOR_STRAIGHT_RANGE_M))
    lh = float(rng.uniform(*_CORRIDOR_STRAIGHT_RANGE_M))
    return [("straight", lw), ("arc", r, 90.0), ("straight", lh), ("arc", r, 90.0),
            ("straight", lw), ("arc", r, 90.0), ("straight", lh), ("arc", r, 90.0)]


def generate_corridor_course(rng: np.random.Generator, *, name: str = "rand-corridor",
                             width: float = 1.0, resolution: float = 0.02,
                             final_step: float = 0.1,
                             min_turn_radius_m: float | None = None) -> Course:
    """直線区間が支配的な、板で仕切っただけのラジコン練習コースのようなコース
    （オーバル型/角丸長方形型を半々でランダムに選ぶ）。

    ★2026-08-31書き直し: 旧実装は`_build_loop`（`_filleted_polygon_xy`、頂点数4・
    jaggedness≈0のプリセット）を流用していたが、`_filleted_polygon_xy`は頂点を
    `np.linspace(0, 2π, n_ctrl, endpoint=False)`で**常に同じ角度**（4頂点なら
    0/90/180/270°）に置く実装のため、半径をどう振っても**毎回同じ「菱形」に
    しか見えない**——バンビの指摘で判明。曲がる場所を直線の両端だけに限定した
    専用の経路（`_stadium_path`/`_rectangle_path`、`straight`/`arc`セグメントを
    直接組み立てる）に作り直し、対称性だけで閉じることを保証した（`_build_loop`
    の「引き直して確認」ループを経由しない）。

    ★2026-08-31追記: 旋回半径を`width`と無関係に決めていたため、道幅が広く
    半径が小さい組み合わせで内側の「島」が対向車線と合体するバグが別途あった
    （`_corridor_turn_radius`docstring参照）。半径を`width`連動の下限付きで
    選ぶよう修正済み。
    """
    r_min = (min_turn_radius_m if min_turn_radius_m is not None
            else _vehicle_min_turn_radius_m() * _RADIUS_MARGIN)
    half_width = width / 2.0
    path_fn = _stadium_path if rng.random() < 0.5 else _rectangle_path
    path = path_fn(rng, r_min, half_width)

    start_yaw = float(rng.uniform(0.0, 2 * math.pi))     # 見た目の向きもランダム化する
    pts = _track_centerline(path, 0.0, 0.0, start_yaw, step=_ARC_STEP)
    xy = resample_loop(pts[:, :2], final_step)
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


def generate_narrow_course(rng: np.random.Generator, *, name: str = "rand-narrow",
                           width_range: tuple[float, float] = (0.9, 1.3),
                           resolution: float = 0.02, final_step: float = 0.1,
                           min_turn_radius_m: float | None = None,
                           n_narrow_range: tuple[int, int] = (1, 2)) -> Course:
    """道幅が局所的に急に狭くなる区間を1〜2箇所持つコース。

    物理的な壁は`add_offset_discs()`（`sim/track.py`）で中心線の両側**対称**に
    追加する。報酬側の余白計算（`sim/gym_env.py`の`_CenterlineProgress`）に使う
    `Course.width`は、区間ごとの実際の道幅を反映した配列にする——スカラーの
    ままだと広い区間で過剰に罰し、狭い区間で罰が足りなくなるため
    （モジュールdocstring「アーキタイプの多様化」参照）。

    ★2026-08-31修正1: 初回実装は`r_disc`/`offset`の式が誤っており、**意図した
    `local_width`より実際に掘る幅の方が広くなる**バグがあった（オフセットを
    `half_nom - r_disc`（壁側基準）で決めていたため、余白ぶんの`+2*resolution`が
    そのまま中心線側への食い込みに上乗せされていた）。極端な場合は左右の円盤が
    中心線を越えて重なり、**道が完全に塞がる**ことがバンビの質問で判明。
    `offset`を`half_local + r_disc`（中心線側の到達点を先に固定する）に直し、
    実際に掘られる境界が常に`half_local`ちょうど（それより内側に食い込まない）に
    なるようにした。あわせて`_NARROW_MIN_CLEARANCE_M`で「車体が物理的に
    通れない」水準まで狭まることも防いでいる。

    ★2026-08-31修正2: 修正1の後も、実際にgridをレイキャストして計測すると
    記録した`local_width`より数cm狭いことがあった。原因は`_add_chicanes`が
    docstringで書いている「曲率の合成」と同じ罠——`add_offset_discs`は中心線に
    沿って**一定のオフセット**で円盤を置くだけなので、ベースの中心線自体が
    （`_build_loop`のヘアピン/フィレットで）曲がっている区間に置くと、円盤が
    描く弧の内側／外側で実際の余白が均等にならない。`_add_chicanes`と同じ
    `straight_enough`判定（曲率が`1/(2*r_min)`未満の区間だけを候補にする）を
    移植し、ベースが実質直線とみなせる区間だけにnarrowを置くようにした。
    """
    nominal_width = float(rng.uniform(*width_range))
    min_local_width = 2 * _vehicle_half_width_m() + _NARROW_MIN_CLEARANCE_M
    centerline, r_min = _build_loop(rng, min_turn_radius_m=min_turn_radius_m, hairpin_prob=0.35,
                                    final_step=final_step)
    grid, origin = rasterize(centerline, nominal_width, resolution)

    xy = centerline[:, :2]
    N = len(xy)
    loop = np.vstack([xy, xy[:1]])
    seg = np.hypot(*np.diff(loop, axis=0).T)
    arc = np.concatenate([[0.0], np.cumsum(seg)])[:-1]
    yaw = np.arctan2(np.diff(loop[:, 1]), np.diff(loop[:, 0]))
    dyaw = np.abs(np.diff(np.unwrap(np.concatenate([yaw, yaw[:1]]))))
    base_curv = dyaw / np.maximum(seg, 1e-6)
    # `_add_chicanes`の`3*min_radius_m`よりは緩め——narrowは曲率が単純加算される
    # わけではなく「均等でなくなる」だけなので、そこまで厳しくしなくても安全
    straight_enough = base_curv < (1.0 / (2.0 * r_min))

    width_profile = np.full(N, nominal_width)
    spans: list[tuple[float, float, float, float]] = []
    for _ in range(int(rng.integers(*n_narrow_range))):
        frac = float(rng.uniform(*_NARROW_FRAC_RANGE))
        local_width = max(nominal_width * frac, min(min_local_width, nominal_width))
        length = float(rng.uniform(*_NARROW_LEN_RANGE))
        half_win = max(2, int(round(length / 2.0 / max(final_step, 1e-6))))
        half_win = min(half_win, N // 2 - 1)
        if half_win < 2:
            continue

        # `add_offset_discs`（`sim/track.py`）は`s0 <= s1`の単純な区間マスクで
        # ループの継ぎ目をまたげない前提なので、候補窓は配列の両端をまたがない
        # 範囲だけに限定する（narrowを置ける場所が少し減るだけで実害は無い）
        s0 = s1 = None
        for _try in range(40):
            cand = int(rng.integers(half_win, N - half_win))
            window_idx = range(cand - half_win, cand + half_win + 1)
            if np.all(straight_enough[list(window_idx)]):
                s0, s1 = float(arc[cand - half_win]), float(arc[cand + half_win])
                break
        if s0 is None:
            continue                                    # 直線区間が見つからなければこの狭め箇所は諦める

        width_profile[(arc >= s0) & (arc <= s1)] = local_width

        half_local = local_width / 2.0
        depth = nominal_width / 2.0 - half_local
        # `r_disc`の下限を余白ぶん確保しつつ、`offset`は「中心線側の到達点が
        # ちょうど`half_local`になる」方を基準にする——`r_disc`が余白で
        # 大きくなっても中心線側への食い込みが増えない（食い込みが増えるのは
        # 壁側基準で決めていた旧実装のバグ）
        r_disc = max(depth / 2.0 + resolution, resolution * 3)
        offset = half_local + r_disc
        spans.append((s0, s1, offset, r_disc))
        spans.append((s0, s1, -offset, r_disc))

    if spans:
        add_offset_discs(grid, origin, resolution, centerline, spans)

    start = (float(centerline[0, 0]), float(centerline[0, 1]), float(centerline[0, 2]))
    return Course(name=name, path=Path(f"<random:{name}>"), resolution=resolution,
                 origin=origin, start=start, grid=np.ascontiguousarray(grid),
                 centerline=centerline, width=width_profile)


def generate_obstacle_course(rng: np.random.Generator, *, name: str = "rand-obstacle",
                             width_range: tuple[float, float] = (0.9, 1.3),
                             resolution: float = 0.02, final_step: float = 0.1,
                             min_turn_radius_m: float | None = None,
                             n_obstacles_range: tuple[int, int] = (2, 5)) -> Course:
    """孤立した円盤障害物を数個浮かべたコース。左右どちらの壁にも接触させず、
    反対側に車両が通れる余裕（`_OBSTACLE_CLEARANCE_M`）を必ず残す
    （モジュールdocstring「アーキタイプの多様化」参照）。

    衝突判定・LiDARは素の`grid`だけを見るので、障害物を追加しても
    `sim/course.py`・`sim/lidar.py`側は無改修でそのまま機能する。
    `Course.obstacles`（(K,3): x, y, 半径）に障害物の中心を残し、
    `sim/gym_env.py`の`reset()`がスタート地点抽選時にこれへめり込まないよう
    避けるのに使う。
    """
    width = float(rng.uniform(*width_range))
    centerline, _ = _build_loop(rng, min_turn_radius_m=min_turn_radius_m, hairpin_prob=0.2,
                                final_step=final_step)
    grid, origin = rasterize(centerline, width, resolution)

    seg = np.hypot(*np.diff(centerline[:, :2], axis=0).T)
    arc = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(arc[-1])
    half_w = width / 2.0
    veh_half_w = _vehicle_half_width_m()

    xs, ys, yaws = centerline[:, 0], centerline[:, 1], centerline[:, 2]
    spans: list[tuple[float, float, float, float]] = []
    centers: list[tuple[float, float, float]] = []
    for _ in range(int(rng.integers(*n_obstacles_range))):
        r_obs = float(rng.uniform(*_OBSTACLE_RADIUS_RANGE))
        # 反対側に veh_half_w + クリアランスぶんの余裕を残す下限、壁に接触しない上限
        lo = veh_half_w + _OBSTACLE_CLEARANCE_M + r_obs - half_w
        hi = half_w - r_obs - 0.03
        if lo >= hi:
            continue                                     # このコース幅には狭すぎる→諦める
        d = float(rng.uniform(lo, hi)) * (1.0 if rng.random() < 0.5 else -1.0)
        s = float(rng.uniform(0.0, total))
        spans.append((max(0.0, s - r_obs), min(total, s + r_obs), d, r_obs))
        i = int(np.argmin(np.abs(arc - s)))
        nx, ny = -math.sin(yaws[i]), math.cos(yaws[i])
        centers.append((xs[i] + nx * d, ys[i] + ny * d, r_obs))

    if spans:
        add_offset_discs(grid, origin, resolution, centerline, spans)

    start = (float(centerline[0, 0]), float(centerline[0, 1]), float(centerline[0, 2]))
    obstacles = np.asarray(centers, dtype=np.float64) if centers else None
    return Course(name=name, path=Path(f"<random:{name}>"), resolution=resolution,
                 origin=origin, start=start, grid=np.ascontiguousarray(grid),
                 centerline=centerline, width=width, obstacles=obstacles)


def generate_diverse_course(rng: np.random.Generator, *, name: str = "rand",
                            width_range: tuple[float, float] = (0.7, 1.3),
                            weights: dict[str, float] | None = None,
                            resolution: float = 0.02, final_step: float = 0.1,
                            min_turn_radius_m: float | None = None) -> Course:
    """学習コースのアーキタイプ（organic/circuit/corridor/narrow/obstacle）を
    `weights`（省略時`_ARCHETYPE_WEIGHTS`）の重みでエピソードごとにランダムに
    選ぶ、`course_fn`用のエントリポイント（`ml_lidar/train_rl.py`が
    `course_fn=generate_diverse_course`で使う）。単一の生成器だけだと似た形の
    コースばかりになる、というバンビの指摘（2026-08-31）を受けて追加した
    （モジュールdocstring参照）。

    `course_fn: Callable[[rng], Course]`という1引数の契約
    （`sim/gym_env.py`の`SimE2EEnv.reset()`参照）はそのまま守っているので、
    `ml_lidar/train_rl.py`側はこの関数へ差し替えるだけで対応できる。

    :param weights: `_ARCHETYPE_WEIGHTS`と同じキー集合の重み（比率だけが意味を
        持つ、正規化して使う）。カリキュラム学習（`CurriculumCourseFn`）が
        学習序盤だけ難しいアーキタイプの重みを下げるために渡す
    """
    weights = weights if weights is not None else _ARCHETYPE_WEIGHTS
    kinds = list(weights.keys())
    p = np.array([weights[k] for k in kinds])
    kind = kinds[int(rng.choice(len(kinds), p=p / p.sum()))]

    common = dict(name=name, resolution=resolution, final_step=final_step,
                 min_turn_radius_m=min_turn_radius_m)
    if kind == "organic":
        return generate_random_course(rng, width=float(rng.uniform(*width_range)), **common)
    if kind == "circuit":
        return generate_circuit_course(rng, width=float(rng.uniform(*width_range)), **common)
    if kind == "corridor":
        return generate_corridor_course(rng, width=float(rng.uniform(*width_range)), **common)
    if kind == "narrow":
        return generate_narrow_course(rng, width_range=width_range, **common)
    return generate_obstacle_course(rng, width_range=width_range, **common)


class CurriculumCourseFn:
    """`generate_diverse_course`をラップし、`set_progress()`で難度を段階的に
    引き上げる`course_fn`（`Callable[[rng], Course]`と同じ1引数契約）。

    v9の実測（`docs/progress`参照）で、評価rewardの標準偏差が一部の評価回だけ
    100超まで跳ねる＝一部のコース条件でだけ大きく崩れる傾向が見えたことを受け、
    学習序盤は易しいアーキタイプ・広めの道幅だけを経験させ、`progress`が
    1.0に近づくにつれ`generate_diverse_course`本来の分布（`_ARCHETYPE_WEIGHTS`・
    `width_range`）へ線形に近づける。narrow/obstacle（最も衝突しやすい
    アーキタイプ）は`progress=0`で重み0（一切出さない）にしてある。

    `set_progress()`は`ml_lidar/train_rl.py`の`CurriculumCallback`が
    `VecEnv.env_method()`経由で各ワーカープロセスの`SimE2EEnv.set_curriculum_progress()`
    →`GymSurgeEnv`を通じて呼ぶ（`sim/gym_env.py`参照）。インスタンスは
    `course_fn`と同じくワーカープロセスごとに独立して持つ（`ml_lidar/train_rl.py`
    の`_make()`内で生成する想定）ので、他ワーカーの進捗と混ざる心配はない。
    """

    #: `progress=0`（学習序盤）で使う重み。narrow/obstacleを完全に除外し、
    #: 残りは易しい順（organic>circuit>corridor）に厚めに配分する
    _EASY_WEIGHTS = {"organic": 0.5, "circuit": 0.3, "corridor": 0.2,
                     "narrow": 0.0, "obstacle": 0.0}

    def __init__(self, *, full_width_range: tuple[float, float] = (0.7, 1.3),
                easy_width_low: float = 1.0, **course_kwargs) -> None:
        """:param full_width_range: `progress=1.0`で使う道幅レンジ（既存の
            `generate_diverse_course`既定と同じ値にすること）
        :param easy_width_low: `progress=0`での道幅レンジ下限。上限は
            `full_width_range[1]`で固定（広い道幅はそもそも易しいので序盤から
            許容し、狭い側だけを段階的に解放する）
        :param course_kwargs: `generate_diverse_course`へそのまま渡す追加引数
            （`resolution`・`final_step`・`min_turn_radius_m`等）
        """
        self.progress = 0.0
        self._full_width_range = full_width_range
        self._easy_width_low = easy_width_low
        self._course_kwargs = course_kwargs

    def set_progress(self, progress: float) -> None:
        self.progress = max(0.0, min(1.0, progress))

    def __call__(self, rng: np.random.Generator) -> Course:
        t = self.progress
        width_low = self._easy_width_low + t * (self._full_width_range[0] - self._easy_width_low)
        weights = {k: self._EASY_WEIGHTS[k] + t * (_ARCHETYPE_WEIGHTS[k] - self._EASY_WEIGHTS[k])
                  for k in _ARCHETYPE_WEIGHTS}
        return generate_diverse_course(rng, weights=weights,
                                       width_range=(width_low, self._full_width_range[1]),
                                       **self._course_kwargs)
