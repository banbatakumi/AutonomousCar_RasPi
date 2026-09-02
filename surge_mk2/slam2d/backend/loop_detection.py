"""汎用ループ検出 — 過去のキーフレームを再訪したかを判定する。

`raspi/nav/slam.py`の「始点から0.8m・35°以内なら1周」という判定は、単一の
周回コース専用のヒューリスティック（累積回頭360°で「1周した」を検出し、
残差を軌跡全体へ弧長比例で配る、拘束1本だけの特殊ケース）だった。ここでは
任意個数・任意位置の再訪（複数回の周回、8の字コースの交差点での再訪等）に
対応できるよう一般化する。

## 手順

1. 新しいキーフレームの点群を、現在の地図（占有格子）に対して**広域探索**
   （`LOOP_CLOSE_STAGES`、事前分布なし）で照合し、「本当はどこにいるか」を
   1回だけ求める——地図全体に対するグローバルな一致度を使うことで、
   「どの過去キーフレームと対応するか」を先に決め打ちしない
2. その結果（`fixed_pose`）が、時間的に離れた（`min_index_gap`件より前の）
   過去キーフレームの近く（`radius`・`yaw_tolerance`以内）にあれば、
   その過去キーフレームとの相対姿勢をループ拘束の候補とする
3. スコア閾値（`min_score`）を超えたものだけ採用する

`match()`を候補ごとに何度も呼ばず、新しいキーフレーム1つにつき1回の広域
探索で済ませているのは計算コストを抑えるため（候補フィルタリングは単純な
距離計算なのでO(N)でも軽い）。

## 既知の限界（oval周回シムでの実測、Phase 5）

**第1ラウンド（拘束が見つかるたびに即座に最適化する逐次処理）**: ループ
拘束を1本追加してすぐポーズグラフ最適化すると、`Frontend`単体（ループ
閉じ無し）よりも最終的な自己位置の回転誤差が**悪化する**ケースを実測で
確認した（3周のoval周回、測距ノイズ1cm、seed=7: ループ閉じ有りで3周目
約3.3°、無しで約1.4°）。原因を切り分けたところ:

- ループ拘束の情報行列に上限を掛ける、`LOOP_CLOSE_STAGES`の最終段を通常
  マッチと同精度まで絞る、をそれぞれ単独で試すと3.3°→2.0°程度まで
  部分的に改善したが、両方を組み合わせて実装に反映した状態では改善効果
  が打ち消され、単独実験の改善が再現しなかった（相互作用の原因は未特定）
- 反復回数を増やしても変化なし（最適化自体は収束済みで、反復不足が原因
  ではない）
- 検出された対応（`src`/`dst`）自体は物理的に正しい再訪だった（真の位置を
  突き合わせて確認済み）ので、誤対応が原因でもない
- ロバストカーネル（`g2opy`の`RobustKernelHuber`）を単発の拘束に適用しても
  変化なし——外れ値を弱める仕組みは「他の拘束との矛盾」があって初めて
  効くので、拘束が1本しかない状況では機能しない

**第2ラウンド（`pipeline.SlamSystem`の`loop_batch_size`本ぶん溜めてから
一括最適化するバッチ処理、`pipeline.py`参照）**: `loop_batch_size=3`の
逐次的なバッチ処理で3.3°→1.9°、走行中は最適化を保留し走行終了時の
`flush()`で全ループ拘束を1回でまとめて最適化する運用では3.3°→1.3°まで
改善した。複数の観測を同時にグラフへ入れることで、単一拘束のノイズがそのまま
姿勢に反映されるのではなく、他の拘束（オドメトリ鎖・他のループ拘束）との
整合性込みで解が決まるようになったため——1つの観測を鵜呑みにしなくなった、
と理解できる。ただし**`Frontend`単体の精度（`flush()`後で約0.85°）には
まだ届いておらず、改善も一貫していない**（8周・4シードでの追試では
2/4のみ改善。詳細な数値は`pipeline.py`のモジュールdocstring参照）。

このシナリオでは、オドメトリ側の累積誤差が既に小さい（`core/confidence.py`
のおかげで1〜2周でも1°未満）のに対し、広域探索1回あたりのノイズが
相対的に大きく、少数の拘束では統計的に十分均せない、というのが現時点の
理解。

**第3ラウンド（外れ値除去の検証）**: 「拘束の中にノイズの大きい外れ値が
混じっているのでは」という仮説のもと、`g2opy`の`RobustKernelHuber`と
`RobustKernelDCS`（Dynamic Covariance Scaling、SLAMのループクロージャ外れ値
除去専用に設計された手法）をバッチ処理後の複数拘束に適用して4シードで
検証したが、**カーネルの閾値をいくら変えても結果が寸分違わず一致し、
全く効果が無かった**。実際のループ拘束6本の内容を出力して調べたところ、
原因が判明した——1本だけ他よりずっと大きい値（delta=(-0.27m, -6.5°)、
他5本は数mm・1°未満）を持つ拘束があり、一見これが外れ値に見えたが、
**真の軌道と突き合わせるとフロントエンド推定・観測delta・真値の3つが
すべて誤差0.3cm・0.4°程度で一致しており、外れ値ではなく単に「1周弱
走った時点の正確な相対姿勢」だった**。ロバストカーネルは「他の拘束との
矛盾」があって初めて重みを弱める仕組みなので、そもそも矛盾（真の外れ値）
が存在しない状況では原理的に効果が出ない。

つまり**「外れ値除去」は今回の症状には的外れな処方だった**。個々の拘束は
すべて正確なのに、それらを組み合わせて最適化すると精度が悪化することが
あるという事実は変わらず残っており、原因はより構造的（各エッジの情報行列
の異方性と、実際の蓄積誤差の分配パターンが噛み合っていない可能性）と
考えられる。これを追うには`core/confidence.py`の情報行列推定自体をやり直す
規模の作業になり、Phase 5の検証としてはここで区切ることにした。次に
着手するならここが最有力候補——局所曲率ベースの推定を、複数の独立した
観測から求めた統計的な共分散（例えば同じ区間を複数回通った際のばらつき
の実測）に置き換える等が考えられる。8の字コースのように複数回の再訪が
ある状況では拘束が複数本あるぶんこの問題は緩和される可能性があるが
未検証——`tests/test_pipeline_oval.py`の`TestFigureEightDoesNotBreak`で
確認できているのは「見失わず動く」ことまで。

**現状の結論**: このモジュール（`backend/`一式、`pipeline.SlamSystem`の
バッチ最適化込み）は「複数の拘束を扱える枠組み」として動作し、逐次処理の
最悪ケースは大幅に緩和したが、oval1周のような拘束の少ないシナリオでは
精度面で`Frontend`単体を上回れていない。使うかどうかは用途で判断すべき。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import NamedTuple

from ..core.confidence import ConfidenceConfig, estimate_information
from ..core.grid import OccGrid
from ..core.scanmatch import MatcherConfig, Stage, match
from ..core.types import Cov3, Pose2D, ScanPoints, between, wrap_angle

__all__ = ["LoopDetectorConfig", "LoopCandidate", "LOOP_CLOSE_STAGES", "find_loop_closure"]

#: ループ閉じ専用の広い探索範囲。通常の`MatcherConfig`の既定`stages`より広く
#: 取ってよい——見失い復帰は毎周期の予算を食うが、ループ検出は1キーフレーム
#: に1回しか呼ばれないので余裕がある。
#:
#: ★ 最終段は`core/scanmatch.py`の`DEFAULT_STAGES`最終段と同じ精度まで絞る。
#: oval周回シムでの実測で、粗い最終段（旧: 半径2cm・刻み0.5cm）のままだと
#: 通常のオドメトリマッチより粗いノイズをループ拘束が持ち込んでしまい、
#: ループ閉じ無しより自己位置の回転誤差が悪化するケースを確認した
LOOP_CLOSE_STAGES: tuple[Stage, ...] = (
    Stage(0.60, 0.10, math.radians(20.0), math.radians(5.0)),
    Stage(0.10, 0.03, math.radians(5.0), math.radians(1.0)),
    Stage(0.02, 0.005, math.radians(1.0), math.radians(0.2)),
    Stage(0.008, 0.004, math.radians(0.25), math.radians(0.05)),
)


@dataclass(frozen=True, slots=True)
class LoopDetectorConfig:
    #: 広域探索の結果が、過去キーフレームからこれ以内なら再訪とみなす[m]
    radius: float = 1.0
    #: 同 向き差[rad]
    yaw_tolerance: float = math.radians(45.0)
    #: これより新しい（直近`min_index_gap`件の）キーフレームは候補にしない。
    #: 直近のキーフレームは当然近くにあるので、それをループ閉じと誤認しない
    min_index_gap: int = 20
    #: 広域探索の得点がこれ未満なら採用しない
    min_score: float = 0.5
    stages: tuple[Stage, ...] = LOOP_CLOSE_STAGES
    #: ループ拘束の信頼度推定に使う`core/confidence.py`の設定。
    #:
    #: ★ `max_eigenvalue`をcoreの既定(1e6、事実上無制限)より大幅に絞ってある。
    #: oval周回シムの実測で、局所曲率がオドメトリの累積誤差より過信気味な
    #: 値になり、1本のループ拘束がグラフ全体を悪い方向へ引っ張るケースを
    #: 確認した。絞ってもなお改善しきらない場合があることも実測しており
    #: （下記モジュールdocstring「既知の限界」参照）、これは対症療法でしかない
    confidence: ConfidenceConfig = ConfidenceConfig(max_eigenvalue=10.0)


class LoopCandidate(NamedTuple):
    src: int                               #: 過去のキーフレームindex
    dst: int                               #: 現在のキーフレームindex
    delta: Pose2D                          #: srcから見たdstの相対姿勢
    score: float
    #: `delta`の信頼度（`core/confidence.py`で広域探索の最良解周辺の局所曲率
    #: から推定したもの）。通常のオドメトリ観測の情報行列と違い、ループ拘束は
    #: 探索範囲が広いぶん誤対応のリスクも高いので、呼び出し側の情報行列を
    #: 使い回さずここで専用に計算する
    information: Cov3


def find_loop_closure(keyframes: list[tuple[Pose2D, ScanPoints, Cov3]],
                      grid: OccGrid, new_index: int, *,
                      config: LoopDetectorConfig = LoopDetectorConfig()) -> LoopCandidate | None:
    """`keyframes[new_index]`が過去のキーフレームの近くに戻ってきたかを調べる。

    見つかれば、その過去キーフレーム(`src`)から見た現在(`dst=new_index`)の
    相対姿勢を`LoopCandidate`として返す。見つからなければ`None`。
    """
    if new_index - config.min_index_gap < 0:
        return None
    new_pose, new_pts, _ = keyframes[new_index]

    m = match(grid, new_pts, new_pose,
             config=MatcherConfig(stages=config.stages, prior_w=0.0))
    if not m.searched or m.score < config.min_score:
        return None
    fixed_pose = Pose2D(m.x, m.y, m.yaw)

    best_index: int | None = None
    best_dist = math.inf
    for i in range(0, new_index - config.min_index_gap + 1):
        old_pose, _, _ = keyframes[i]
        dist = math.hypot(fixed_pose.x - old_pose.x, fixed_pose.y - old_pose.y)
        if dist > config.radius:
            continue
        if abs(wrap_angle(fixed_pose.yaw - old_pose.yaw)) > config.yaw_tolerance:
            continue
        if dist < best_dist:
            best_dist = dist
            best_index = i

    if best_index is None:
        return None
    old_pose, _, _ = keyframes[best_index]
    delta = between(old_pose, fixed_pose)
    info = estimate_information(grid, new_pts, fixed_pose, config=config.confidence)
    return LoopCandidate(best_index, new_index, delta, m.score, info)
