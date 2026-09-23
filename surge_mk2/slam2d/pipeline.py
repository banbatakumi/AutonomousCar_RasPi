"""SlamSystem — `Frontend`（位置合わせ・地図構築）と`backend/`（ループ検出・ポーズ
グラフ最適化）を束ねる公開API。

## 流れ

- `update()`: `Frontend.update()` → 新しいキーフレームをポーズグラフへ
  （ノード＋直前との相対姿勢エッジ）→ 走行距離`loop_every_m`ごとにループ検出。
  見つかった拘束はグラフに足すだけで、**最適化は`optimize()`/`flush()`まで
  保留する**（走行中に秒単位で止まらないように）
- `optimize()`: グラフを最適化し、`Frontend.apply_correction()`で地図・局所地図・
  今の姿勢を補正後の値で作り直す

`optimize_every`（拘束の本数）を正にすると、その本数たまるたびに走行中でも
自動で最適化する。地図の焼き直しはキーフレーム数に比例するので、Pi の
10Hz ループの中で回すなら0（`flush()`でだけ）にしておくこと。

## ループ拘束の外れ値

ループ検出は幾何の検査を何段も通したものだけを返すが、それでも対称な形の
取り違えは原理的に残る。拘束は Huber を掛けて入れ（影響が線形に頭打ちになる
だけで、効きはする）、**最適化してみて残差が桁違いに大きいものを外して
やり直す**（`loop_reject_chi2`）。DCS を使わない理由は
`backend/posegraph.PoseGraph.add_loop_edge`のdocstring参照。
"""

from __future__ import annotations

import math
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
    #: 走行距離でこれだけ進むごとにループ検出を試す[m]
    loop_every_m: float = 0.4
    #: ループ拘束がこれだけ溜まったら走行中でも最適化する。0なら`flush()`でだけ
    optimize_every: int = 0
    #: ループ拘束に掛ける Huber のしきい値（残差ノルム＝√カイ二乗）。
    #: `core/frontend.inflate_info`で共分散に下限（並進2cm・回頭0.3°）を
    #: 入れてあるので、3 は「3σ を超えたら影響を頭打ちにする」の意味
    loop_huber: float = 3.0
    #: 最適化後の残差（カイ二乗）がこれを超えるループ拘束は**外して**やり直す。
    #: 3自由度なので、全方向で5σずれていればカイ二乗75。誤検出（対称形状の
    #: 取り違え等）はこの桁を軽く超える
    loop_reject_chi2: float = 75.0
    #: 外れ値を外してやり直す回数の上限
    reject_passes: int = 2
    iterations: int = 30


class SlamSystem:
    """`Frontend`と`backend/`を束ねた、SLAM全体の公開インターフェース。"""

    def __init__(self, grid: OccGrid, motion: MotionModel,
                 config: PipelineConfig = PipelineConfig()) -> None:
        self.config = config
        self.frontend = Frontend(grid, motion, config.frontend)
        self._graph = PoseGraph()
        #: グラフへ既に取り込み済みのキーフレーム件数（ノードID = キーフレームindex）
        self._synced = 0
        self._last_try_path = -1e9
        #: グラフに入れたループ拘束（診断・テスト用）
        self.loops: list[LoopCandidate] = []
        #: 最適化後の残差が大きすぎて外した拘束（診断用）
        self.rejected: list[LoopCandidate] = []
        self._since_opt = 0
        self.optimizations = 0

    @property
    def loop_closures(self) -> int:
        return len(self.loops)

    def update(self, raw: RawScan, dt: float) -> FrontendUpdate:
        u = self.frontend.update(raw, dt)
        self._sync_graph()
        return u

    def flush(self) -> None:
        """保留中のループ拘束があれば最適化して地図を作り直す。走行の終わりに呼ぶ。"""
        if self._since_opt > 0:
            self.optimize()

    def optimize(self) -> None:
        """グラフを最適化し、外れ値のループ拘束を外してやり直し、地図へ反映する。"""
        if self._synced < 2:
            return
        self._graph.optimize(self.config.iterations)
        for _ in range(max(0, self.config.reject_passes)):
            chi2 = self._graph.loop_chi2()
            bad = [i for i, c in enumerate(chi2)
                   if self._graph.loop_edges[i] is not None and c > self.config.loop_reject_chi2]
            bad = [i for i in bad if math.isfinite(chi2[i])]
            if not bad:
                break
            for i in bad:
                self._graph.drop_loop(i)
                self.rejected.append(self.loops[i])
            self._graph.optimize(self.config.iterations)
        poses = self._graph.all_poses()
        self.frontend.apply_correction(poses)
        self._since_opt = 0
        self.optimizations += 1

    def _sync_graph(self) -> None:
        kfs = self.frontend.keyframes
        while self._synced < len(kfs):
            i = self._synced
            kf = kfs[i]
            nid = self._graph.add_node(kf.pose)
            if i == 0:
                self._graph.fix(nid)       # ゲージ拘束
            else:
                self._graph.add_odometry_edge(i - 1, i, between(kfs[i - 1].pose, kf.pose),
                                              kf.info)
            self._synced += 1

            if kf.path - self._last_try_path >= self.config.loop_every_m:
                self._last_try_path = kf.path
                cand = find_loop_closure(kfs, i, config=self.config.loop_detector)
                if cand is not None:
                    self._graph.add_loop_edge(cand.src, cand.dst, cand.delta, cand.information,
                                              huber=self.config.loop_huber or None)
                    self.loops.append(cand)
                    self._since_opt += 1
                    if 0 < self.config.optimize_every <= self._since_opt:
                        self.optimize()
                        kfs = self.frontend.keyframes
