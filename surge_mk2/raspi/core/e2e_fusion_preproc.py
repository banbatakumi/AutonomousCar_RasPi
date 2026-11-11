"""e2e_fusion（LiDAR＋カメラ＋車両状態 → 前方の経路点）の入出力の形 — **学習と推論が同じ関数を通るための唯一の定義。**

Pi の推論（`raspi/nodes/e2e_fusion_node.py`）と Mac の学習・評価（`ml_e2e/`）の両方がここを import する
（`cam_e2e_preproc.py` と同じ理由。画像の前処理はそちらの関数をそのまま使う）。numpy だけで書く。

## 入力

| 名前 | 形 | 中身 |
|---|---|---|
| `image` | `(1, 3, h, w)` | 前方カメラ。`cam_e2e_preproc.preprocess`（左右の切り落とし・面積平均・0〜1） |
| `lidar` | `(1, H+1, n)` | 前方 `fov_deg` の距離（`max_range` で割った 0〜1）を新しい順に `H` 周、最後の1行は最新の周の「実測点か」 |
| `state` | `(1, 3)` | 車速・実舵角・ヨーレート（`speed_ref`・`max_steer`・`yaw_rate_ref` で割る） |
| `avail` | `(1, 3)` | 画像・点群・車両状態が使えるか（1/0）。0 の入力は中身を見ない（学習でも一定割合で伏せている） |

距離は `raspi.auto.base.scan_window` を通したもの（欠測セクタ＝0＝壁、測距不能・飽和＝`max_range`）。
点群の読み方の契約をここに書き写さないため、`lidar_rows` は `ScanWindow` の `dist`・`measured` を受け取る。

## 出力（1本のベクトル、長さ `4K`）

`[x_1..x_K, y_1..y_K, v_1..v_K, b_1..b_K]`。`v` は 0〜1（×`speed_ref`）、
`x`・`y` は `xy_scale` [m] を掛けて車体座標（x=前・y=左）の経路点、`b` は各点の位置の不確かさ [m]
（学習したラプラス分布の幅。大きいほど自信が無い）。`k` 番目の点は経路に沿って `(k+1)·ds` 先。

舵・速度の直接出力は持たない（版 1 にはあった。走行は経路点を追うので 2026-11-11 に外した）。
"""

from __future__ import annotations

from typing import NamedTuple

import numpy as np

__all__ = ["PREPROC_VERSION", "INPUT_NAMES", "FusionOutput", "lidar_rows", "lidar_input",
           "state_input", "output_size", "decode_output"]

#: ここの処理（行の並び・正規化・出力の並び）を変えたら上げる。モデル同梱の JSON と照合する
PREPROC_VERSION = 2
#: ONNX の入力名（この順）
INPUT_NAMES = ("image", "lidar", "state", "avail")


class FusionOutput(NamedTuple):
    path_xy: np.ndarray     #: (K, 2) 車体座標 [m]
    path_v: np.ndarray      #: (K,) [m/s]
    path_b: np.ndarray      #: (K,) 位置の不確かさ [m]


def lidar_rows(dist, measured, max_range: float) -> np.ndarray:
    """`ScanWindow` の `dist`・`measured` → `(2, n)` float32（0〜1 の距離、実測点か）。"""
    d = np.asarray(dist, dtype=np.float32) / np.float32(max_range)
    return np.stack([np.clip(d, 0.0, 1.0), np.asarray(measured, dtype=np.float32)])


def lidar_input(rows: list[np.ndarray | None], history: int) -> np.ndarray:
    """新しい順の `lidar_rows`（`rows[0]` が最新）→ `(1, history+1, n)`。

    古い周が無い（`None`・足りない）所は、それより新しい周で埋める（起動直後・点群が1周抜けたとき）。
    """
    if not rows or rows[0] is None:
        raise ValueError("最新の点群が無い")
    out = []
    last = rows[0]
    for h in range(history):
        r = rows[h] if h < len(rows) and rows[h] is not None else last
        out.append(r[0])
        last = r
    out.append(rows[0][1])
    return np.ascontiguousarray(np.stack(out)[None, ...], dtype=np.float32)


def state_input(speed: float, steer: float, yaw_rate: float, *, speed_ref: float, max_steer: float,
                yaw_rate_ref: float) -> np.ndarray:
    """車速・実舵角・ヨーレート → `(1, 3)` float32。"""
    return np.array([[speed / speed_ref, steer / max_steer, yaw_rate / yaw_rate_ref]], dtype=np.float32)


def output_size(k: int) -> int:
    return 4 * k


def decode_output(vec: np.ndarray, *, k: int, speed_ref: float, xy_scale: float) -> FusionOutput:
    """モデルの出力ベクトル → 物理量。長さが合わない・有限でない値があれば例外。"""
    v = np.asarray(vec, dtype=np.float64).reshape(-1)
    if v.size != output_size(k):
        raise ValueError(f"出力の長さ {v.size} が契約の {output_size(k)} と合わない")
    if not np.isfinite(v).all():
        raise ValueError("出力に NaN/Inf がある")
    xy = np.stack([v[:k], v[k:2 * k]], axis=1) * xy_scale
    return FusionOutput(path_xy=xy, path_v=np.clip(v[2 * k:3 * k], 0.0, 1.0) * speed_ref,
                        path_b=v[3 * k:4 * k])
