"""ポーズグラフ最適化バックエンド — `g2opy`への薄いラッパー。

自作するのはフロントエンド（スキャンマッチ・占有格子・センサ融合・ループ検出）
までで、疎行列の非線形最適化そのものは既に成熟した領域であり自作しても
`g2o`の品質・堅牢性を超えるのは難しいため、ここだけ既存ライブラリに委ねる
（`/Users/banbatakumi/.claude/plans/wiggly-doodling-moth.md`参照）。

`g2opy`（PyPI、メンテナはg2o本家作者のRainerKuemmerle）を採用した。
`g2o-python`（サードパーティ製）はEigen3等のC++依存が無いとソースビルドが
失敗することを実機で確認して不採用にした一方、`g2opy`はmacOS/Linux
aarch64向けのビルド済みwheelがあり`pip install`だけで動く
（`slam2d/tools/spike_g2o.py`で3ノードのトイポーズグラフが厳密解に
収束することを検証済み）。

`raspi/nav/slam.py`の「弧長比例配分によるループクロージャ」（拘束1本だけの
特殊ケース）を、任意個数・任意位置の拘束を扱えるポーズグラフ最適化に
一般化したもの。ノード=キーフレーム姿勢、エッジ=(隣接ノード間の)オドメトリ
拘束と(非隣接ノード間の)ループ閉じ拘束。
"""

from __future__ import annotations

import numpy as np

try:
    import g2opy as g2o
except ImportError as exc:  # pragma: no cover - 環境依存のエラーメッセージ
    raise ImportError(
        "backend/posegraph.py には g2opy が必要です。"
        "`.venv/bin/pip install -r slam2d/requirements.txt` を実行してください。"
    ) from exc

from ..core.types import Cov3, Pose2D

__all__ = ["PoseGraph"]


class PoseGraph:
    """SE2ポーズグラフの`g2opy`ラッパー。

    ノードIDは`add_node()`が呼ばれた順に0から自動採番する（呼び出し側が
    IDを管理しなくてよいようにするための設計）。
    """

    def __init__(self) -> None:
        solver = g2o.BlockSolverSE2(g2o.LinearSolverEigenSE2())
        algo = g2o.OptimizationAlgorithmLevenberg(solver)
        self._opt = g2o.SparseOptimizer()
        self._opt.set_algorithm(algo)
        self._next_id = 0

    @property
    def size(self) -> int:
        return self._next_id

    def add_node(self, pose: Pose2D) -> int:
        """`pose`を初期推定としてノードを追加し、そのIDを返す。"""
        v = g2o.VertexSE2()
        node_id = self._next_id
        self._next_id += 1
        v.set_id(node_id)
        v.set_estimate(g2o.SE2(pose.x, pose.y, pose.yaw))
        self._opt.add_vertex(v)
        return node_id

    def fix(self, node_id: int) -> None:
        """`node_id`を固定する（ゲージ拘束。通常は最初のノードに使う）。"""
        self._opt.vertex(node_id).set_fixed(True)

    def add_odometry_edge(self, src: int, dst: int, delta: Pose2D,
                          information: Cov3) -> None:
        """隣接キーフレーム間の相対姿勢拘束を追加する。"""
        self._add_edge(src, dst, delta, information)

    def add_loop_edge(self, src: int, dst: int, delta: Pose2D,
                      information: Cov3) -> None:
        """非隣接キーフレーム間（ループ閉じ）の相対姿勢拘束を追加する。

        API上はオドメトリエッジと同じ（`EdgeSE2`は拘束の種類を区別しない）。
        呼び出し側の意図を名前で示すために分けてある。
        """
        self._add_edge(src, dst, delta, information)

    def _add_edge(self, src: int, dst: int, delta: Pose2D, information: Cov3) -> None:
        e = g2o.EdgeSE2()
        e.set_vertex(0, self._opt.vertex(src))
        e.set_vertex(1, self._opt.vertex(dst))
        e.set_measurement(g2o.SE2(delta.x, delta.y, delta.yaw))
        e.set_information(np.asarray(information, dtype=np.float64))
        self._opt.add_edge(e)

    def optimize(self, iterations: int = 20) -> None:
        self._opt.initialize_optimization()
        self._opt.optimize(iterations)

    def pose(self, node_id: int) -> Pose2D:
        est = self._opt.vertex(node_id).estimate()
        return Pose2D(float(est[0]), float(est[1]), float(est[2]))

    def all_poses(self) -> list[Pose2D]:
        return [self.pose(i) for i in range(self._next_id)]
