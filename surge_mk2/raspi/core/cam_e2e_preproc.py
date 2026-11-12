"""cam_e2e（カメラE2E・模倣学習）の前処理 — **学習と推論が同じ関数を通るための唯一の定義。**

Pi の推論（`raspi/nodes/cam_e2e_node.py`）と Mac の学習・評価
（`ml_cam_e2e/dataset.py`・`eval.py`）の両方がここを import する。
学習と推論で処理（リサイズ方式・色順）が食い違うと、学習したとおりの絵が実車のモデルに入らない。
**同じ処理を2箇所に書かない**ことでこの種の事故を構造的に防ぐ。

numpy だけで書く——Pi には opencv も Pillow も入れていない
（`raspi/requirements.lock.txt`）。

`to_rgb()`・`resize_area()` は `raspi/nodes/cam_perception_node.py`
（セグメンテーション）と `ml_cam/dataset.py` も使う。同じ関数に揃えてある。

`PREPROC_VERSION` はモデル同梱の `<name>.json` に書かれ、ノードが読み込み時に
照合する。ここの処理（色順・リサイズ方式・正規化）を変えたら必ず上げること。
古い前処理で学習したモデルを新しい前処理で走らせる事故を、読み込み拒否で止める。

## 左右の切り落とし（`crop_x`、cam_e2e だけ）

cam_e2e は学習で「車の向きをずらした絵」を作る（`ml_cam_e2e/dataset.py`）。絵を横にずらすと
端に元画像の無い帯ができ、それを埋めた画素はラベルの補正と完全に連動する——モデルが帯を
手がかりに覚えてしまう。そこで**学習でも推論でも左右を `crop_x`（画像幅に対する割合）ずつ
切り落とした中央の窓を使い**、ずらすのは窓の位置だけにする（端に作り物の画素が出ない）。

共有の `PREPROC_VERSION` を上げると切り落としと無関係な `ml_cam`（セグメンテーション）の
モデルまで読めなくなるので、cam_e2e の契約だけ `PREPROC_VERSION_CROP` を書く。
切り落としを知らない古いノードは版の照合でそのモデルを拒否する（黙って全幅で推論しない）。

## 左右2台の入力（`PREPROC_VERSION_STEREO`、cam_e2e・e2e_fusion）

右（主、`image/front`）と左（`image/left`）の同じ瞬間の組を入力にする
（`docs/stereo_plan.md` §1）。2枚をそれぞれ `preprocess` と同じ処理（切り落とし・面積平均・正規化）に
通し、**チャネル方向に `[右 RGB, 左 RGB]` の順で重ねた `(1, 6, h, w)`** をモデルに入れる
（`preprocess_stereo`）。モデルは前半3チャネルと後半3チャネルを同じ重みの骨格に別々に通すので、
画素の位置が左右で合っている必要は無い。版2（切り落とし）を含む。

**cam_e2e・e2e_fusion のノードは版3だけを読む。** 版1・2（右1台）は前後配置のころのモデルの版で、
視点が変わって使えないので読み込みで拒否する（`PREPROC_VERSION` は `ml_cam` が今も使う）。
"""

from __future__ import annotations

from functools import lru_cache

import numpy as np

__all__ = ["PREPROC_VERSION", "PREPROC_VERSION_CROP", "PREPROC_VERSION_STEREO", "STEREO_CAMS",
           "STEREO_PAIR_TOL_NS", "to_rgb", "crop_margin_px", "crop_window", "resize_area",
           "normalize", "preprocess", "preprocess_stereo"]

PREPROC_VERSION = 1
#: 版1に「左右の切り落とし `crop_x`」を足した版（cam_e2e の契約だけが使う。モジュールdocstring参照）
PREPROC_VERSION_CROP = 2
#: 版2を左右2台の入力（`[右, 左]` を 6 チャネルに重ねる）にした版（モジュールdocstring参照）
PREPROC_VERSION_STEREO = 3
#: 左右2台の入力のカメラの並び（契約 JSON の `cams`）。`image/front` が右（主）、`image/left` が左
STEREO_CAMS = ("front", "left")
#: 左右を同じ瞬間の組とみなす撮影時刻の差の上限。`camera_node.STEREO_SYNC_TOL_NS`（`synced` を立てる条件）と同じ
STEREO_PAIR_TOL_NS = 500_000


def to_rgb(frame: np.ndarray, fmt: str) -> np.ndarray:
    """共有メモリの生フレーム → RGB の (H, W, 3)。

    `fmt` は `ImageRef.fmt`（**メモリ上の実際のバイト順**、`camera_node.py` の
    `_MEMORY_ORDER` 参照）。判定は `raspi/core/jpeg.py` と同じ「`BGR` で始まるか」。
    記録の JPEG は jpeg.py が同じ規則で正しい色にしているので、ここが揃っていれば
    学習（JPEG をデコードした RGB）と推論の色が一致する。
    """
    if fmt.startswith("BGR"):
        return frame[..., 2::-1]
    return frame[..., :3]


@lru_cache(maxsize=8)
def _area_bins(src: int, out: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """面積平均の区画 `(先頭, 画素数, 重み)`。出力 i は入力 `[先頭, 先頭+画素数)` を覆う。

    重みは float32 の `1/画素数`（を float64 で持つ。積を丸めずに求めるため）。
    拡大方向（out > src）では区画が1画素になる（結果は最近傍と同じ）。
    """
    start = (np.arange(out, dtype=np.int64) * src) // out
    counts = np.maximum(np.diff(np.append(start, src)), 1)
    weight = (1.0 / counts).astype(np.float32).astype(np.float64)
    return start, counts, weight


def _area_axis0(x: np.ndarray, out: int) -> np.ndarray:
    """先頭の軸を面積平均で `out` 個にする（float32）。

    区画の先頭から順に「重み×画素」を足し、1項ごとに float32 へ丸める
    （FMA で積和する BLAS と同じ丸め。下の `resize_area` 参照）。積と和は
    float64 で求めるので、丸めは float32 へ落とす1回だけになる。
    """
    start, counts, weight = _area_bins(x.shape[0], out)
    w = weight.reshape((-1,) + (1,) * (x.ndim - 1))
    acc = (w * x[start]).astype(np.float32)
    for k in range(1, int(counts.max())):
        rows = np.flatnonzero(counts > k)
        acc[rows] = (acc[rows] + w[rows] * x[start[rows] + k]).astype(np.float32)
    return acc


def resize_area(rgb: np.ndarray, width: int, height: int) -> np.ndarray:
    """面積平均で (height, width, C) の uint8 に縮小する。

    640x360 → 224x128 は約2.9倍の縮小。最近傍だと画素を拾い飛ばして
    エイリアシングが出るうえ、JPEG（学習データ）と生フレーム（推論）の差が
    そのまま残る。区画内を平均すればブロックノイズも均されて差が小さくなる。

    縦→横の順に区画内を重み付きで足す。**BLAS を使わない。** 重み行列との
    積（`np.tensordot`）にすると、Pi の numpy（Debian の `python3-numpy`）は
    参照実装の BLAS につながっていて、640x360→224x128 に約 180ms かかる
    （Mac は約 1ms）。ここは区画の画素だけを足すので Pi で数 ms。
    丸めは行列積の結果と同じになるように足している（`_area_axis0`。学習済みモデルの絵と揃えるため。
    `raspi/tests/test_cam_e2e_preproc.py` が行列積版と画素単位で突き合わせる）。
    """
    src_h, src_w = rgb.shape[:2]
    if (src_w, src_h) == (width, height):
        return np.ascontiguousarray(rgb, dtype=np.uint8)
    x = _area_axis0(rgb, height)                                       # (h, W, C)
    # 横の段は連続配列に直してから（飛び飛びの読み出しのままより Pi で約 1ms 速い）
    x = _area_axis0(np.ascontiguousarray(x.swapaxes(0, 1)), width).swapaxes(0, 1)  # (h, w, C)
    return np.floor(x + np.float32(0.5)).astype(np.uint8)


def normalize(resized: np.ndarray, mean: float = 0.0, std: float = 255.0) -> np.ndarray:
    """縮小済みの (h, w, 3) → モデル入力 (1, 3, h, w) float32。

    学習側（`ml_cam_e2e/dataset.py`）は縮小と正規化の間に明るさの揺らぎを
    挟むので、2段に分けてある。推論は `preprocess()` で一息に通す。
    """
    x = (resized.astype(np.float32) - mean) / std
    return np.ascontiguousarray(np.transpose(x, (2, 0, 1))[None, ...])


def crop_margin_px(width: int, crop_x: float) -> int:
    """幅 `width` の画像で左右それぞれ切り落とす画素数。"""
    return round(width * crop_x)


def crop_window(rgb: np.ndarray, crop_x: float, shift_px: int = 0) -> np.ndarray:
    """左右を `crop_x` ずつ切り落とした中央の窓。`shift_px` だけ窓を右へずらす（学習の拡張用）。

    窓の幅はずらしても変わらない。`|shift_px|` が切り落とし幅を超えると画像の外に出るので例外。
    """
    if crop_x <= 0:
        if shift_px:
            raise ValueError("切り落とし無しでは窓をずらせない")
        return rgb
    w = rgb.shape[1]
    m = crop_margin_px(w, crop_x)
    if abs(shift_px) > m:
        raise ValueError(f"ずらし {shift_px}px が切り落とし幅 {m}px を超える")
    return rgb[:, m + shift_px: w - m + shift_px]


def preprocess(rgb: np.ndarray, size: tuple[int, int], mean: float = 0.0,
               std: float = 255.0, crop_x: float = 0.0) -> np.ndarray:
    """RGB の (H, W, 3) uint8 → モデル入力 (1, 3, h, w) float32。`size` は (width, height)。"""
    w, h = size
    return normalize(resize_area(crop_window(rgb, crop_x), w, h), mean, std)


def preprocess_stereo(rgb_right: np.ndarray, rgb_left: np.ndarray, size: tuple[int, int],
                      mean: float = 0.0, std: float = 255.0, crop_x: float = 0.0) -> np.ndarray:
    """右・左の RGB `(H, W, 3)` uint8 → モデル入力 `(1, 6, h, w)` float32（`[右, 左]` の順）。"""
    return np.concatenate([preprocess(rgb_right, size, mean, std, crop_x),
                           preprocess(rgb_left, size, mean, std, crop_x)], axis=1)
