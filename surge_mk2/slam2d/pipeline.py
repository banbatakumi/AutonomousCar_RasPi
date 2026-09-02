"""SlamSystem — `Frontend`(姿勢推定・地図構築)と`backend/`(ループ検出・ポーズ
グラフ最適化)を束ねる公開API。

`raspi/nav/slam.py`の`close_loop()`（周回検出のたびに、拘束1本を軌跡全体へ
弧長比例で配る特殊ケース）を、任意個数・任意位置の拘束（複数回の周回、
8の字コースでの再訪等）に対応できるポーズグラフ最適化に一般化したもの
（`/Users/banbatakumi/.claude/plans/wiggly-doodling-moth.md`参照）。

## ★ ループ拘束は見つかるたびにではなく、**まとめて**最適化する

Phase 5の最初の実装は「ループ拘束が1本見つかるたびに即座に最適化・地図
再構築する」逐次処理だった。oval周回シムで実測したところ、`Frontend`単体
（ループ閉じ無し）より自己位置の回転誤差が**悪化する**ケースがあった
（3周・測距ノイズ1cm・seed=7で、無し1.4°に対し逐次処理3.3°）。

原因を切り分けると、オドメトリ側の累積誤差が既に小さい（`core/confidence.py`
の動的異方性ブレンドのおかげで1〜2周でも1°未満）のに対し、ループ検出の
広域探索1回分の観測ノイズの方が相対的に大きく、**「蓄積誤差を正すはずの
拘束が、単独では逆に新しいノイズ源として働いてしまう**」状況だった。

`loop_batch_size`本のループ拘束が集まってから一括で最適化するよう変更した
ところ、`loop_batch_size=3`で3.3°→1.9°まで改善した。**既定値を大きく
（`loop_batch_size=10`）して走行中は最適化を保留し、走行終了時に`flush()`
で全ループ拘束をまとめて1回だけ最適化する運用（`tests/test_pipeline_oval.py`
参照）では3.3°→1.3°まで改善した**。複数の観測を同時にグラフへ入れる
ことで、1本の拘束のノイズがそのまま姿勢に反映されず、他の拘束・末尾まで
含めたオドメトリ鎖全体との整合性込みで解が決まるようになるため——バッチ
処理の途中で最適化を挟むと、その時点でまだキーフレームが揃っていない
（走行の終わりまで到達していない）状態の姿勢を「最終結果」として評価する
ことになり不正確になる、という罠を実験中に踏んだ（走行終わりまで待たず
早めに最適化すると良い値が出たように見えたが、実際には走り切る前の途中
状態を見ていただけだった）。この教訓から、**バッチ処理は「都度」より
「走行終了時に一括」の方が素直で、精度も安定する**という結論に至った。

それでも`Frontend`単体（`flush()`後で約0.85°）にはまだ届いておらず、
コースや乱数シードによっては届かないことの方が多い（オドメトリの蓄積誤差
が既に小さく、広域探索1回のノイズの方が相対的に大きいため）。詳細と
次の改善候補は`backend/loop_detection.py`のモジュールdocstring
「既知の限界」参照。
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .backend.loop_detection import LoopCandidate, LoopDetectorConfig, find_loop_closure
from .backend.posegraph import PoseGraph
from .core.frontend import Frontend, FrontendConfig, FrontendUpdate
from .core.grid import OccGrid
from .core.motion import MotionModel
from .core.types import RawScan, between

__all__ = ["PipelineConfig", "SlamSystem"]


@dataclass(frozen=True, slots=True)
class PipelineConfig:
    frontend: FrontendConfig = field(default_factory=FrontendConfig)
    loop_detector: LoopDetectorConfig = field(default_factory=LoopDetectorConfig)
    #: 何本のループ拘束が新規に見つかったら最適化・地図再構築するか。
    #: 1にすると見つかるたびに即座に最適化する旧来の逐次処理に戻る
    #: （上のモジュールdocstring参照。実測でバッチサイズが大きいほど安定して
    #: 改善する傾向があったため、既定は大きめに取ってある）。
    #:
    #: ★ **短い走行では既定値に達する前に一度も最適化されない可能性がある。
    #: 走行の終わりに必ず`flush()`を呼ぶこと。** この値はコースの長さ・
    #: `min_index_gap`次第で「1回の走行で何本くらいループ拘束が出るか」が
    #: 変わるため、絶対的な最適値ではない——`flush()`頼みの運用を前提にした
    #: 保険的な大きさとして選んである
    loop_batch_size: int = 10


class SlamSystem:
    """`Frontend`と`backend/`を束ねた、SLAM全体の公開インターフェース。"""

    def __init__(self, grid: OccGrid, motion: MotionModel,
                config: PipelineConfig = PipelineConfig()) -> None:
        self.config = config
        self.frontend = Frontend(grid, motion, config.frontend)
        self._graph = PoseGraph()
        #: `_node_ids[i]`が`frontend.keyframes[i]`に対応するg2oノードID
        self._node_ids: list[int] = []
        #: グラフへ既に取り込み済みのキーフレーム件数
        self._synced = 0
        self.loop_closures = 0
        #: 直近でループ閉じを検出したキーフレームindex（クールダウン用）
        self._last_loop_index: int | None = None
        #: まだグラフに追加していない（バッチが溜まるのを待っている）候補
        self._pending: list[LoopCandidate] = []

    def update(self, raw: RawScan, dt: float) -> FrontendUpdate:
        u = self.frontend.update(raw, dt)
        self._sync_graph()
        return u

    def flush(self) -> None:
        """保留中のループ拘束を、`loop_batch_size`本に満たなくても反映する。

        走行・記録を終える際に呼ぶと、中途半端に溜まった拘束を捨てずに
        最後の地図へ反映できる。
        """
        if self._pending:
            self._apply_pending_and_rebuild()

    def _sync_graph(self) -> None:
        """`Frontend`に新しく積まれたキーフレームをグラフへ反映し、ループ検出する。"""
        kfs = self.frontend.keyframes
        while self._synced < len(kfs):
            i = self._synced
            pose, _pts, info = kfs[i]
            node_id = self._graph.add_node(pose)
            self._node_ids.append(node_id)
            if i == 0:
                self._graph.fix(node_id)    # ゲージ拘束
            else:
                prev_pose, _, _ = kfs[i - 1]
                delta = between(prev_pose, pose)
                self._graph.add_odometry_edge(self._node_ids[i - 1], node_id, delta, info)

            # ★ クールダウン: 周回コースの直線区間では「過去の同じ直線区間の
            # キーフレーム」が毎ステップ空間的に近く、`find_loop_closure`が
            # 際限なく再ヒットする（実測: ovalシムで94ステップ連続発火した）。
            # 直前の検出から`min_index_gap`ぶん経つまで次を試みない
            cooldown_ok = (self._last_loop_index is None
                          or i - self._last_loop_index >= self.config.loop_detector.min_index_gap)
            if cooldown_ok:
                cand = find_loop_closure(kfs, self.frontend.grid, i,
                                         config=self.config.loop_detector)
                if cand is not None:
                    self._pending.append(cand)
                    self._last_loop_index = i
                    if len(self._pending) >= self.config.loop_batch_size:
                        self._apply_pending_and_rebuild()
                        kfs = self.frontend.keyframes    # rebuild後は新しいリストを見る

            self._synced += 1

    def _apply_pending_and_rebuild(self) -> None:
        """保留中のループ拘束をまとめてグラフに追加し、最適化・地図再構築する。"""
        for cand in self._pending:
            self._graph.add_loop_edge(self._node_ids[cand.src], self._node_ids[cand.dst],
                                      cand.delta, cand.information)
            self.loop_closures += 1
        self._pending = []
        self._optimize_and_rebuild()

    def _optimize_and_rebuild(self) -> None:
        """グラフを最適化し、補正された姿勢で地図・キーフレーム・軌跡を作り直す。

        `raspi/nav/slam.py`の`close_loop()`と同じ考え方——版番号を単調に
        増やした新しい`OccGrid`に、補正済みの姿勢で全キーフレームを焼き直す
        （古い版に上書きすると、地図が「同じコースを角度違いで重ね描きした」
        形に崩れる。既存実装が実測で踏んだ罠）。
        """
        self._graph.optimize()
        fixed_poses = self._graph.all_poses()

        old_grid = self.frontend.grid
        size_m = old_grid.width * old_grid.resolution
        new_grid = OccGrid(resolution=old_grid.resolution, size_m=size_m,
                          origin=old_grid.origin, min_hits=old_grid.min_hits,
                          min_seen=old_grid.min_seen)
        new_grid.seq = old_grid.seq + 1

        kfs = self.frontend.keyframes
        new_keyframes = []
        new_trajectory = []
        for pose, (_, pts, info) in zip(fixed_poses, kfs):
            new_grid.integrate(pts, pose)
            new_trajectory.append(pose)
            new_keyframes.append((pose, pts, info))

        self.frontend.rebuild(grid=new_grid, keyframes=new_keyframes,
                              trajectory=new_trajectory, pose=fixed_poses[-1])
