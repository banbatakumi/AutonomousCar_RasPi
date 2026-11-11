"""`ml_e2e/dataset.py`・`model.py`・`train.py` の損失・`export_onnx.py` のテスト（合成した記録で通す）。"""

import io
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

import numpy as np  # noqa: E402
import torch  # noqa: E402
from PIL import Image  # noqa: E402

from ml_e2e import extract as X  # noqa: E402
from ml_e2e.dataset import FusionDataset, InputSpec, load_records, sample_weights, split_records  # noqa: E402
from ml_e2e.export_onnx import build_model, export, verify_parity  # noqa: E402
from ml_e2e.model import B_MAX, B_MIN, FusionModel  # noqa: E402
from ml_e2e.tests.test_extract import _write  # noqa: E402
from ml_e2e.train import _avail, losses, split_output  # noqa: E402
from raspi.core.cam_e2e_preproc import PREPROC_VERSION_CROP  # noqa: E402
from raspi.core.e2e_fusion_preproc import PREPROC_VERSION, decode_output, output_size  # noqa: E402

K = 8
SPEC = InputSpec(size=(64, 32), crop_x=0.1, fov_deg=90, max_range=8.0, k=K, max_steer=0.4)


def _jpeg() -> bytes:
    """左半分が明るい絵（左右反転が分かる）。"""
    a = np.zeros((36, 80, 3), dtype=np.uint8)
    a[:, :40] = 220
    buf = io.BytesIO()
    Image.fromarray(a).save(buf, format="JPEG", quality=95)
    return buf.getvalue()


class _Data(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.dir = Path(cls.tmp.name) / "data"
        cls.dir.mkdir()
        cfg = X.ExtractConfig(k=K, ds=0.25)
        meta = {"config": cfg.to_json(), "records": {}}
        for name in ("a", "b", "c"):
            _write(Path(cls.tmp.name) / f"{name}.mcap", duration_s=8.0, jpeg=_jpeg())
            meta["records"][name] = X.extract_recording(Path(cls.tmp.name) / f"{name}.mcap", cls.dir, cfg)
        (cls.dir / "dataset.json").write_text(json.dumps(meta))
        cls.records, _ = load_records(cls.dir, SPEC, margin_deg=6)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()


class TestDataset(_Data):
    def test_split_holds_out_whole_records(self):
        train, val = split_records(self.records)
        self.assertEqual([r.name for r in val], ["c"])
        self.assertEqual([r.name for r in train], ["a", "b"])
        train, val = split_records(self.records, ["a"])
        self.assertEqual([r.name for r in val], ["a"])

    def test_item_shapes_and_values(self):
        ds = FusionDataset(self.records, self.dir, SPEC, margin_deg=6)
        it = ds[10]
        self.assertEqual(tuple(it["image"].shape), (3, 32, 64))
        self.assertEqual(tuple(it["lidar"].shape), (4, 91))
        self.assertEqual(tuple(it["path"].shape), (K, 2))
        np.testing.assert_allclose(it["lidar"][:3].numpy(), 3.0 / 8.0, atol=1e-6)
        np.testing.assert_allclose(it["lidar"][3].numpy(), 1.0)
        self.assertAlmostEqual(float(it["state"][0]), 1.0 / SPEC.speed_ref, places=5)
        self.assertNotIn("ctrl", it)                                  # 舵・速度の手本は持たない
        self.assertAlmostEqual(ds.xy_scale, K * 0.25)
        self.assertGreater(float(it["image"][:, :, :20].mean()), float(it["image"][:, :, -20:].mean()))

    def test_flip_mirrors_every_input_and_label_together(self):
        plain = FusionDataset(self.records, self.dir, SPEC, margin_deg=6)[10]
        ds = FusionDataset(self.records, self.dir, SPEC, margin_deg=6, augment=True, flip=True)
        np.random.seed(0)
        for _ in range(20):
            it = ds[10]
            if float(it["state"][1]) * float(plain["state"][1]) < 0:
                break
        else:
            self.fail("反転した標本が出なかった")
        self.assertAlmostEqual(float(it["state"][1]), -float(plain["state"][1]), places=5)
        self.assertAlmostEqual(float(it["state"][2]), -float(plain["state"][2]), places=5)
        np.testing.assert_allclose(it["path"][:, 1].numpy(), -plain["path"][:, 1].numpy(), atol=1e-6)
        np.testing.assert_allclose(it["path"][:, 0].numpy(), plain["path"][:, 0].numpy(), atol=1e-6)
        self.assertLess(float(it["image"][:, :, :20].mean()), float(it["image"][:, :, -20:].mean()))

    def test_yaw_augmentation_rotates_scan_and_path_by_the_same_angle(self):
        """点群に1本だけ近い点を入れ、車を左へ向けると、その点も経路も同じ角度だけ右へ回る。"""
        rec = self.records[0]
        si = int(rec.data["scan_idx"][10][0])
        saved = rec.rows.copy()
        try:
            rec.rows[si, 0, :] = 1.0
            rec.rows[si, 0, 6 + 45 + 10] = 0.1                        # 元の +10° に近い点（余白 6・視野の左端 -45）
            ds = FusionDataset(self.records, self.dir, SPEC, margin_deg=6, augment=True, flip=False,
                               yaw_max_deg=6.0, fx_px=38.0)
            plain = FusionDataset(self.records, self.dir, SPEC, margin_deg=6)[10]
            np.random.seed(3)
            seen = set()
            for _ in range(30):
                it = ds[10]
                near = int(np.argmin(it["lidar"][0].numpy())) - 45    # 近い点の今の角度 [deg]
                psi = 10 - near                                       # 車が左へ向いた角度
                seen.add(psi)
                a0 = math.atan2(float(plain["path"][-1, 1]), float(plain["path"][-1, 0]))
                a1 = math.atan2(float(it["path"][-1, 1]), float(it["path"][-1, 0]))
                self.assertAlmostEqual(math.degrees(a0 - a1), psi, places=3)
            self.assertGreater(len(seen), 4)
            self.assertLessEqual(max(abs(p) for p in seen), 6)
        finally:
            rec.rows[:] = saved

    def test_sample_weights_favor_rare_steer_bins(self):
        w = sample_weights(self.records, 0.4)
        self.assertEqual(len(w), sum(len(r) for r in self.records))
        self.assertTrue((w > 0).all())


class TestModelAndLoss(unittest.TestCase):
    def setUp(self):
        torch.manual_seed(0)
        self.model = FusionModel(lidar_ch=4, lidar_n=91, k=K, xy_scale=2.0, pretrained=False).eval()
        self.inp = (torch.rand(2, 3, 32, 64), torch.rand(2, 4, 91), torch.rand(2, 3))

    def test_output_layout_matches_the_runtime_decoder(self):
        with torch.no_grad():
            out = self.model(*self.inp, torch.ones(2, 3))
        self.assertEqual(tuple(out.shape), (2, output_size(K)))
        o = decode_output(out[0].numpy(), k=K, speed_ref=5.0, xy_scale=2.0)
        xy, v, b = split_output(out, K)
        np.testing.assert_allclose(o.path_xy, xy[0].numpy() * 2.0, atol=1e-6)
        np.testing.assert_allclose(o.path_v, v[0].numpy() * 5.0, atol=1e-5)
        self.assertTrue(((b >= B_MIN) & (b <= B_MAX)).all())
        self.assertLess(float(b.mean()), 1.0)                         # 初期値は約 0.3m

    def test_hidden_inputs_are_ignored(self):
        """伏せた入力は、中身を変えても出力が変わらない。"""
        other = (torch.rand(2, 3, 32, 64), torch.rand(2, 4, 91), torch.rand(2, 3))
        with torch.no_grad():
            for i in range(3):
                avail = torch.ones(2, 3)
                avail[:, i] = 0.0
                a = self.model(*self.inp, avail)
                swapped = list(self.inp)
                swapped[i] = other[i]
                np.testing.assert_allclose(a.numpy(), self.model(*swapped, avail).numpy(), atol=1e-6)
                self.assertGreater(float((a - self.model(*self.inp, torch.ones(2, 3))).abs().max()), 1e-6)

    def test_avail_never_hides_image_and_lidar_together(self):
        a = _avail(4000, torch.device("cpu"), drop_image=0.3, drop_lidar=0.3, drop_state=0.2)
        self.assertEqual(int(((a[:, 0] == 0) & (a[:, 1] == 0)).sum()), 0)
        self.assertAlmostEqual(float((a[:, 0] == 0).float().mean()), 0.3, delta=0.04)
        self.assertAlmostEqual(float((a[:, 1] == 0).float().mean()), 0.3, delta=0.04)

    def test_masked_labels_do_not_contribute(self):
        out = self.model(*self.inp, torch.ones(2, 3))
        batch = {"path": torch.zeros(2, K, 2), "path_v": torch.zeros(2, K), "pt_ok": torch.ones(2, K),
                 "v_ok": torch.zeros(2)}
        ls = losses(out, batch, K, 2.0)
        self.assertEqual(set(ls), {"path", "v"})
        self.assertEqual(float(ls["v"]), 0.0)
        batch["pt_ok"] = torch.zeros(2, K)
        self.assertEqual(float(losses(out, batch, K, 2.0)["path"]), 0.0)


class TestExport(unittest.TestCase):
    def test_export_writes_a_contract_the_node_accepts(self):
        from raspi.nodes.e2e_fusion_node import load_model

        with tempfile.TemporaryDirectory() as d:
            run = Path(d) / "run"
            run.mkdir()
            cfg = {"preproc_version": PREPROC_VERSION, "image_preproc_version": PREPROC_VERSION_CROP,
                   "input_size": [64, 32], "crop_x": 0.05, "fov_deg": 90, "max_range": 8.0, "history": 3,
                   "k": K, "ds": 0.25, "xy_scale": 2.0, "max_steer": 0.4, "speed_ref": 5.0, "yaw_rate_ref": 6.0,
                   "steer_link": [1.0, 0.0], "maps": ["m"], "train_records": ["a"]}
            (run / "train_config.json").write_text(json.dumps(cfg))
            torch.manual_seed(0)
            model = build_model(cfg)
            torch.save(model.state_dict(), run / "best.pt")
            out = Path(d) / "models" / "t.onnx"
            contract = export(run, out)
            self.assertLess(verify_parity(out, model, cfg), 1e-3)
            self.assertEqual(contract["train_records"], ["a"])
            m = load_model(out)
            o = m.infer(np.zeros((1, 3, 32, 64), np.float32), None, None)
            self.assertEqual(o.path_xy.shape, (K, 2))

            bad = dict(cfg, preproc_version=PREPROC_VERSION + 1)
            (run / "train_config.json").write_text(json.dumps(bad))
            with self.assertRaises(ValueError):
                export(run, out)

    def test_weights_with_the_old_direct_head_export_to_the_same_path_outputs(self):
        """舵・速度の直接出力があった頃（版 1）の重みは、そのヘッドだけ捨てて今の版で書き出せる。"""
        from raspi.nodes.e2e_fusion_node import load_model

        with tempfile.TemporaryDirectory() as d:
            run = Path(d) / "run"
            run.mkdir()
            cfg = {"preproc_version": 1, "image_preproc_version": PREPROC_VERSION_CROP,
                   "input_size": [64, 32], "crop_x": 0.05, "fov_deg": 90, "max_range": 8.0, "history": 3,
                   "k": K, "ds": 0.25, "xy_scale": 2.0, "max_steer": 0.4, "speed_ref": 5.0, "yaw_rate_ref": 6.0,
                   "steer_link": [1.0, 0.0], "label_shift_ms": 100, "maps": ["m"], "train_records": ["a"]}
            (run / "train_config.json").write_text(json.dumps(cfg))
            torch.manual_seed(0)
            model = build_model(cfg)
            old = dict(model.state_dict())
            old["ctrl_head.weight"] = torch.zeros(2, 256)
            old["ctrl_head.bias"] = torch.zeros(2)
            torch.save(old, run / "best.pt")
            out = Path(d) / "models" / "t.onnx"
            contract = export(run, out)
            self.assertEqual(contract["preproc_version"], PREPROC_VERSION)
            self.assertNotIn("label_shift_ms", contract)
            self.assertLess(verify_parity(out, model, contract), 1e-3)
            self.assertEqual(load_model(out).k, K)


if __name__ == "__main__":
    unittest.main()
