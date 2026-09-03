"""走行可能領域の中心線 — カメラ由来の占有格子から、前方複数距離の回廊中心を作る。

`raspi/nav/centerline.py` の `measure()`（SLAM地図の中心線を左右 raycast で測る）と
同じトリックを、閉ループではない・その場限りの1フレーム分だけに使う。

## 車両原点1点からの角度スキャンではなく、前方距離ごとに独立して幅を測る

`cam_perception_node.py` の既存経路（`project_mask_to_grid()` → `OccGrid.raycast()`
→ 擬似 `Scan`）は、車両原点1点から見た角度ごとの距離という1次元表現に潰してから
`raspi/auto/follow_the_gap.py` へ渡す。ここでは逆に、前方距離 `x_k` ごとに
`(x_k, 0)` から左右へ独立にレイを撃ち、その場所での回廊の幅と中心を直接測る——
セグメンテーションマスクが持つ「走行可能領域の形」をより直接使うための経路。

## 左右2N本を1回のベクトル化呼び出しでまとめて撃つ

点ごとに `raycast()` を呼ぶと呼び出しのたびに配列を作り直して遅くなる
（`nav/centerline.py` の `measure()` と同じ理由）。
"""

from __future__ import annotations

import numpy as np

from .grid import OccGrid

__all__ = ["extract_centerline"]


def extract_centerline(grid: OccGrid, blocked: np.ndarray, *,
                        x_min: float, x_max: float,
                        x_step: float) -> tuple[list[float], list[float], list[float]]:
    """`blocked`（True＝壁として扱うセル）から前方の中心線点列を作る。

    `x_min` から `x_max` まで `x_step` 刻みで前方距離を走査し、各点で車両の
    前後軸上 `(x_k, 0)` から左右へ独立にレイを撃って、その場所の走行可能な
    回廊の中心 `y_k` と左右合計の空き幅 `width_k` を測る。返り値は
    `(xs, ys, widths)`（**`xs` は近→遠の昇順、全点を返す**）。

    ★ **どこから先を「通れない」とみなすかはここでは決めない。** 閾値
    （車幅＋余裕）は呼び出し側（`raspi/auto/cam_centerline.py` の GUI 調整可能な
    `min_width_m`）が `widths` に適用する。`Scan.dist` が生の距離配列のまま
    `FollowTheGap` に渡り `gap_min` がそこで初めて効くのと同じ役割分担——
    ここで打ち切ってしまうと、閾値を変えるたびにこのノードを作り直す羽目になる。

    `blocked` はカメラの視野外・地平線より上も True にしておくこと
    （`raspi.nav.ipm.project_seen_to_grid()` 参照）。そうしないと、まだ見えて
    いないだけの場所を「幅がある」と誤認して中心線がそちらへ逃げる。
    """
    n = max(0, int(round((x_max - x_min) / x_step)) + 1)
    if n == 0:
        return [], [], []

    xs_all = x_min + np.arange(n) * x_step
    zeros = np.zeros(n)
    # 左右2N本を1回のraycastで撃つ（`nav/centerline.py` の `measure()` と同じ形）。
    # 側方の探索上限は「前方どこまで見るか」（x_max）とは無関係な、グリッドの
    # 実際の広がりで決める——グリッド外は `raycast` 側で既に壁扱いになるので、
    # ここは大きめに取っても無駄球が増えるだけで安全側にしか転ばない
    max_lateral = grid.width * grid.resolution
    ox = np.concatenate([xs_all, xs_all])
    oy = np.concatenate([zeros, zeros])
    angles = np.concatenate([np.full(n, np.pi / 2.0), np.full(n, -np.pi / 2.0)])
    dist = grid.raycast(ox, oy, angles, max_lateral, mask=blocked)
    left, right = dist[:n], dist[n:]

    ys = (left - right) / 2.0
    widths = left + right
    return xs_all.tolist(), ys.tolist(), widths.tolist()
