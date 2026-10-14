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
from torch.utils.data import DataLoader  # noqa: E402

from dataset import DrivableDataset  # noqa: E402
from model import DrivableSegModel  # noqa: E402
from train import evaluate, iou_score, split_pairs, split_pairs_blocks, train_one_epoch  # noqa: E402


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

    def test_batch_average_is_macro_not_micro(self):
        """★ E3: バッチ内で走行可能領域の広さが違う画像が混ざっても、
        画像単位のIoUを平均する（macro-average）こと。

        1枚目は大きい画像で完全一致(IoU=1)、2枚目は小さい画像で完全不一致
        (IoU=0)。macro-averageなら単純平均の0.5。micro-averageだと
        大きい画像（1枚目）の画素数が支配的になり0.5からずれる。
        """
        pred_batch = torch.zeros(2, 1, 8, 8)
        target_batch = torch.zeros(2, 1, 8, 8)
        pred_batch[0], target_batch[0] = 1.0, 1.0        # 画像1: 完全一致
        pred_batch[1, 0, 0, 0] = 1.0
        target_batch[1, 0, 0, 1] = 1.0                   # 画像2: 完全不一致
        self.assertAlmostEqual(iou_score(pred_batch, target_batch), 0.5, delta=1e-3)


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


class TestSplitPairsBlocks(unittest.TestCase):
    @staticmethod
    def _pairs():
        """10fps・60秒ぶんの連続フレーム（`extract_frames.py`の命名）。"""
        return [(Path(f"run1_front_{int(i * 0.1 * 1e9)}.jpg"), Path("m.png")) for i in range(600)]

    def test_neighbouring_frames_do_not_straddle_train_and_val(self):
        train, val = split_pairs_blocks(self._pairs(), val_ratio=0.2, seed=1, block_s=5.0)
        self.assertEqual(len(train) + len(val), 600)
        t_val = {int(p[0].stem.rsplit("_", 1)[1]) // 5_000_000_000 for p in val}
        t_train = {int(p[0].stem.rsplit("_", 1)[1]) // 5_000_000_000 for p in train}
        self.assertEqual(t_val & t_train, set())          # 同じ5秒ブロックは片側だけ
        self.assertGreaterEqual(len(val), 120)

    def test_deterministic_given_seed(self):
        self.assertEqual(split_pairs_blocks(self._pairs(), seed=3),
                         split_pairs_blocks(self._pairs(), seed=3))

    def test_unparsable_names_fall_back_to_frame_split(self):
        pairs = [(Path(f"frame{i}.jpg"), Path("m.png")) for i in range(20)]
        train, val = split_pairs_blocks(pairs, val_ratio=0.2, seed=1)
        self.assertEqual((len(train), len(val)), (16, 4))

    def test_single_block_falls_back_and_keeps_both_sides(self):
        pairs = [(Path(f"a_front_{i}.jpg"), Path("m.png")) for i in range(20)]
        train, val = split_pairs_blocks(pairs, val_ratio=0.2, seed=1)
        self.assertTrue(train and val)

    def test_train_keeps_at_least_one_block(self):
        pairs = [(Path("a_front_0.jpg"), Path("m.png")), (Path("a_front_9000000000.jpg"), Path("m.png"))]
        train, val = split_pairs_blocks(pairs, val_ratio=0.9, seed=0, block_s=5.0)
        self.assertEqual((len(train), len(val)), (1, 1))


def _make_pair(d: Path, name: str) -> tuple[Path, Path]:
    from PIL import Image
    import numpy as np

    img = Path(d) / f"{name}.jpg"
    msk = Path(d) / f"{name}_mask.png"
    Image.fromarray(np.random.randint(0, 255, (24, 32, 3), dtype="uint8")).save(img)
    Image.fromarray((np.random.rand(24, 32) > 0.5).astype("uint8") * 255,
                    mode="L").save(msk)
    return img, msk


class TestSeedReproducibility(unittest.TestCase):
    """★ E2: `train.py`の`--seed`が`torch.manual_seed()`で重み初期化・
    DataLoaderのシャッフル順の再現性を担保すること（`main()`内の呼び出し
    自体は統合テストが重いので、その根拠となる`torch.manual_seed`の
    効果自体をここで確認する）。"""

    def test_same_seed_gives_the_same_initial_weights(self):
        torch.manual_seed(123)
        m1 = DrivableSegModel(pretrained=False)
        torch.manual_seed(123)
        m2 = DrivableSegModel(pretrained=False)
        for p1, p2 in zip(m1.parameters(), m2.parameters()):
            self.assertTrue(torch.equal(p1, p2))

    def test_different_seed_gives_different_initial_weights(self):
        torch.manual_seed(1)
        m1 = DrivableSegModel(pretrained=False)
        torch.manual_seed(2)
        m2 = DrivableSegModel(pretrained=False)
        differs = any(not torch.equal(p1, p2) for p1, p2 in zip(m1.parameters(), m2.parameters()))
        self.assertTrue(differs)


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
