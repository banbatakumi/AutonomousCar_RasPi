"""Phase 0 スパイク — g2opy が2Dポーズグラフ最適化として実際に使えるか確認する。

3ノード(0→1→2 の直線)を、ノイズ入りの初期値から、正しいオドメトリ拘束2本
（隣接ノード間）だけを与えて最適化する。既知の厳密解（各ステップ (1, 0, 0)
だけ前進）に収束すれば、`backend/posegraph.py` の設計（`VertexSE2`/`EdgeSE2`/
`SparseOptimizer`）がそのまま使えると判断できる。

`g2o-python`（PyPI, miquelmassot作）ではなく `g2opy`（PyPI, メンテナはg2o本家
作者のRainerKuemmerle）を先に試す。理由は Pi 5(aarch64) 向け wheel の有無
（`slam2d/requirements.txt` 参照）。

実行: `.venv/bin/python slam2d/tools/spike_g2o.py`
"""

from __future__ import annotations

import numpy as np

import g2opy as g2o


def _optimizer() -> g2o.SparseOptimizer:
    solver = g2o.BlockSolverSE2(g2o.LinearSolverEigenSE2())
    algo = g2o.OptimizationAlgorithmLevenberg(solver)
    opt = g2o.SparseOptimizer()
    opt.set_algorithm(algo)
    return opt


def main() -> None:
    opt = _optimizer()

    # 真の軌跡: (0,0,0) -> (1,0,0) -> (2,0,0)。初期値にはノイズを乗せる。
    true_poses = [g2o.SE2(0.0, 0.0, 0.0), g2o.SE2(1.0, 0.0, 0.0), g2o.SE2(2.0, 0.0, 0.0)]
    noisy_init = [g2o.SE2(0.0, 0.0, 0.0), g2o.SE2(1.3, 0.2, 0.05), g2o.SE2(1.9, -0.3, -0.05)]

    for i, pose in enumerate(noisy_init):
        v = g2o.VertexSE2()
        v.set_id(i)
        v.set_estimate(pose)
        if i == 0:
            v.set_fixed(True)  # ゲージ拘束: 最初のノードを固定
        opt.add_vertex(v)

    info = np.identity(3) * 100.0  # (dx, dy, dyaw) を強く信頼する設定
    for i in range(2):
        e = g2o.EdgeSE2()
        e.set_vertex(0, opt.vertex(i))
        e.set_vertex(1, opt.vertex(i + 1))
        e.set_measurement(g2o.SE2(1.0, 0.0, 0.0))  # 正しいオドメトリ拘束
        e.set_information(info)
        opt.add_edge(e)

    opt.initialize_optimization()
    opt.optimize(20)

    print("最適化後の姿勢 vs 真の姿勢:")
    max_err = 0.0
    for i in range(3):
        est = opt.vertex(i).estimate()
        est_vec = np.array([est[0], est[1], est[2]])
        true_vec = np.array([true_poses[i][0], true_poses[i][1], true_poses[i][2]])
        err = np.linalg.norm(est_vec - true_vec)
        max_err = max(max_err, err)
        print(f"  node {i}: est=({est[0]:+.4f}, {est[1]:+.4f}, {est[2]:+.4f}) "
              f"true=({true_vec[0]:+.4f}, {true_vec[1]:+.4f}, {true_vec[2]:+.4f}) "
              f"err={err:.6f}")

    assert max_err < 1e-4, f"収束していない（最大誤差 {max_err}）"
    print(f"OK: 収束確認（最大誤差 {max_err:.2e} < 1e-4）")


if __name__ == "__main__":
    main()
