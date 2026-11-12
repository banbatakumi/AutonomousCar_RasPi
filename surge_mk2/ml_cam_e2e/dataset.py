"""ml_cam_e2e/dataset.py — (左右の画像, 正規化した操舵・速度) のペアを読む PyTorch Dataset。

どの行を学習に使うか・ラベルをどう作るかは `samples.py`（torch を知らない
純粋関数）が決める。ここは画像を読んでテンソルにするだけ。

**前処理は `raspi/core/cam_e2e_preproc.py` を通す。** 実車の推論
（`raspi/nodes/cam_e2e_node.py`）と同じ関数なので、学習した絵と実車で
モデルに入る絵が食い違わない。

向きをずらす拡張（`yaw_max_deg > 0`）は `yaw_aug.py`、ブレ・ノイズ・色味・JPEG の外乱
（`photo_strength > 0`）は `photo_aug.py` 参照。拡張しない絵（検証・評価）は推論と同じ
中央の窓（`crop_window`）になる。

## 左右2台

右（manifest の行）と左（`samples.left_path`）を読み、`[右, 左]` の 6 チャネルにする（推論の
`cam_e2e_preproc.preprocess_stereo` と同じ並び）。外乱・明るさの揺らぎは2枚に同じものを掛ける（露光は左が右に
追従するので、実機でも2枚の写り方は揃っている）。向きの拡張は同じ ψ をカメラごとの fx・車体マスクで掛ける。

**反転は「左右を入れ替えてそれぞれ鏡像」**にする。取付が左右対称（y = ±0.0375m）なので、鏡像の左の絵は
「鏡に映した世界での右カメラの絵」になる。1枚ずつその場で鏡像にすると、右下にだけ床が写る右カメラでは
実機で起こらない見え方になる（`docs/stereo_plan.md` 段階5）。
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # repo root

import numpy as np  # noqa: E402
import torch  # noqa: E402
from torch.utils.data import Dataset  # noqa: E402

from raspi.core.cam_e2e_preproc import crop_margin_px, crop_window, normalize, resize_area  # noqa: E402
import photo_aug  # noqa: E402
from samples import Sample, left_path, load_rgb, speed_norm, steer_norm  # noqa: E402
from yaw_aug import sample_psi, yawed_window  # noqa: E402

__all__ = ["load_rgb", "DriveDataset"]

#: 明るさ（加算）とコントラスト（乗算）の揺らぎ幅。照明や露出の違いに
#: 引きずられないようにする程度の弱い値
_BRIGHTNESS = 25.0
_CONTRAST = 0.2


class DriveDataset(Dataset):
    """`__getitem__` は `(image[3,H,W] float32, target[2] float32)`（左右2台は `image[6,H,W]`）。

    `target` は `(steer_norm -1..1, speed_norm 0..1)`。正規化の基準
    （`max_steer`・`speed_ref`）は `export_onnx.py` がモデル同梱の JSON に書き、
    実車がその値で物理量へ戻す——ここだけ違う基準で正規化すると学習と推論でズレる。
    """

    def __init__(self, samples: list[Sample], *, size: tuple[int, int] = (224, 128),
                 max_steer: float, speed_ref: float, augment: bool = False,
                 flip: bool = True, mean: float = 0.0, std: float = 255.0,
                 crop_x: float = 0.0, yaw_max_deg: float = 0.0, yaw_k: float = 0.0,
                 fx_px: float = 0.0, body_mask: np.ndarray | None = None,
                 photo_strength: float = 0.0, fx_px_left: float = 0.0,
                 body_mask_left: np.ndarray | None = None) -> None:
        self.samples = samples
        self.size = size            # (width, height)
        self.max_steer = max_steer
        self.speed_ref = speed_ref
        self.augment = augment
        self.flip = flip
        self.mean = mean
        self.std = std
        #: 左右の切り落とし（`cam_e2e_preproc.crop_window`）。推論と同じ値
        self.crop_x = crop_x
        #: 向きをずらす拡張の最大角 [deg]。0 で拡張しない（`yaw_aug.py`）
        self.yaw_max_deg = yaw_max_deg
        #: 戻す舵の係数: δ' = δ − k·ψ
        self.yaw_k = yaw_k
        self.fx_px = fx_px
        self.body_mask = body_mask
        #: ブレ・ノイズ・色味・JPEG の外乱の強さの倍率。0 で無効（`photo_aug.py`）
        self.photo_strength = photo_strength
        #: 左カメラの fx と車体マスク（モジュールdocstring「左右2台」）
        self.fx_px_left = fx_px_left
        self.body_mask_left = body_mask_left
        if yaw_max_deg > 0 and (crop_x <= 0 or fx_px <= 0 or fx_px_left <= 0):
            raise ValueError("向きの拡張には crop_x と fx_px・fx_px_left が要る")

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, idx: int) -> tuple[torch.Tensor, torch.Tensor]:
        sample = self.samples[idx]
        w, h = self.size
        rgbs = [load_rgb(sample.path), load_rgb(left_path(sample.path))]
        steer = steer_norm(sample, self.max_steer)
        speed = speed_norm(sample, self.speed_ref)
        if self.augment and self.photo_strength > 0:
            # 元の解像度で掛ける（縮小の後だとノイズが強すぎ、ブレが細かすぎる）。
            # 乱数は np.random から種を取り、学習の seed で再現できるようにする。左右は同じ種（同じ外乱）
            seed = np.random.randint(0, 2**31)
            rgbs = [photo_aug.apply(r, np.random.default_rng(seed), self.photo_strength) for r in rgbs]
        if self.augment and self.yaw_max_deg > 0:
            psi = sample_psi(self.yaw_max_deg)
            fxs = (self.fx_px, self.fx_px_left)
            bodies = (self.body_mask, self.body_mask_left)
            rgbs = [yawed_window(r, crop_margin_px(r.shape[1], self.crop_x), psi, fxs[i], bodies[i])
                    for i, r in enumerate(rgbs)]
            # 車が左（ψ>0）を向いていたら右へ戻す。正規化値で補正して出力範囲に収める
            steer = max(-1.0, min(1.0, steer - self.yaw_k * psi / self.max_steer))
        else:
            rgbs = [crop_window(r, self.crop_x) for r in rgbs]
        smalls = [resize_area(r, w, h) for r in rgbs]

        if self.augment:
            if self.flip and np.random.rand() < 0.5:
                # **左右反転と同時に操舵の符号を反転する。** 画像だけ反転すると
                # 「右カーブの絵に左へ切れという教師信号」になり学習が矛盾する
                # （DAVE-2/DonkeyCar系の定番手法）。速度は左右で変わらない。
                # 左右は入れ替えてから鏡像にする（モジュールdocstring「左右2台」）
                smalls = [x[:, ::-1, :] for x in smalls[::-1]]
                steer = -steer
            gain = 1.0 + np.random.uniform(-_CONTRAST, _CONTRAST)
            bias = np.random.uniform(-_BRIGHTNESS, _BRIGHTNESS)
            smalls = [np.clip(x.astype(np.float32) * gain + bias, 0.0, 255.0) for x in smalls]

        img = np.concatenate([normalize(x, self.mean, self.std)[0] for x in smalls], axis=0)
        return torch.from_numpy(img), torch.tensor([steer, speed], dtype=torch.float32)
