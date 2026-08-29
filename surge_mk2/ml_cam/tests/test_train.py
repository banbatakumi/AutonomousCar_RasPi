"""`ml_cam/train.py` のテスト。

`iou_score`/`split_pairs` は純粋関数として厳密に検証し、学習ループ本体は
「合成データ2枚で1エポック回して壊れないこと」までのスモークテストに留める
（実際の学習効果は本物のデータが要る領域なので確認しない）。
"""

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # ml_cam/

import torch  # noqa: E402
from PIL import Image  # noqa: E402
from torch.utils.data import DataLoader  # noqa: E402

from dataset import DrivableDataset  # noqa: E402
from model import DrivableSegModel  # noqa: E402
from train import evaluate, iou_score, split_pairs, train_one_epoch  # noqa: E402


class TestIouScore(unittest.TestCase):
    def test_perfect_match_is_one(self):
        """`eps` を分母に足しているぶん厳密には 1.0 未満だが、無視できる程度。"""
        t = torch.tensor([[[[1.0, 1.0], [0.0, 0.0]]]])
        self.assertAlmostEqual(iou_score(t, t), 1.0, delta=1e-4)

    def test_no_overlap_is_zero(self):
        pred = torch.tensor([[[[1.0, 0.0], [0.0, 0.0]]]])
        target = torch.tensor([[[[0.0, 1.0], [0.0, 0.0]]]])
        self.assertAlmostEqual(iou_score(pred, target), 0.0)

    def test_both_empty_is_one_not_a_division_error(self):
        """走行可能な画素が両方とも0個の画像で、0除算を「不一致」と誤判定しない。"""
        z = torch.zeros(1, 1, 4, 4)
        self.assertAlmostEqual(iou_score(z, z), 1.0)

    def test_threshold_is_applied_to_prediction(self):
        pred = torch.tensor([[[[0.9, 0.4]]]])          # 0.4 は閾値未満で背景扱い
        target = torch.tensor([[[[1.0, 1.0]]]])
        self.assertAlmostEqual(iou_score(pred, target, threshold=0.5), 0.5, places=3)


class TestSplitPairs(unittest.TestCase):
    def test_no_overlap_between_train_and_val(self):
        pairs = list(range(20))
        train, val = split_pairs(pairs, val_ratio=0.2, seed=1)
        self.assertEqual(set(train) | set(val), set(pairs))
        self.assertEqual(set(train) & set(val), set())
        self.assertEqual(len(val), 4)

    def test_deterministic_given_seed(self):
        pairs = list(range(10))
        a = split_pairs(pairs, seed=42)
        b = split_pairs(pairs, seed=42)
        self.assertEqual(a, b)

    def test_single_pair_has_no_validation_split(self):
        """1枚しか無いときに val が空を食って学習側まで空にしない。"""
        train, val = split_pairs([("x", "y")])
        self.assertEqual(len(train), 1)
        self.assertEqual(len(val), 0)


def _make_pair(d: Path, name: str) -> tuple[Path, Path]:
    from PIL import Image
    import numpy as np

    img = Path(d) / f"{name}.jpg"
    msk = Path(d) / f"{name}_mask.png"
    Image.fromarray(np.random.randint(0, 255, (24, 32, 3), dtype="uint8")).save(img)
    Image.fromarray((np.random.rand(24, 32) > 0.5).astype("uint8") * 255,
                    mode="L").save(msk)
    return img, msk


class TestTrainingSmoke(unittest.TestCase):
    """**合成データで1エポック回して壊れないことだけを見る。** 実際の学習効果は
    実データが要る領域（計画の「ユーザー側の作業」）なので、ここでは確認しない。
    """

    def test_one_epoch_runs_without_error_and_loss_is_finite(self):
        with tempfile.TemporaryDirectory() as d:
            pairs = [_make_pair(Path(d), f"f{i}") for i in range(4)]
            ds = DrivableDataset(pairs, size=(32, 24), augment=True)
            loader = DataLoader(ds, batch_size=2, shuffle=True)

            model = DrivableSegModel(pretrained=False)
            optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
            device = torch.device("cpu")

            loss = train_one_epoch(model, loader, optimizer, device)
            self.assertTrue(loss == loss)              # NaN でないこと
            self.assertGreater(loss, 0.0)

            iou = evaluate(model, loader, device)
            self.assertGreaterEqual(iou, 0.0)
            self.assertLessEqual(iou, 1.0)


if __name__ == "__main__":
    unittest.main()
