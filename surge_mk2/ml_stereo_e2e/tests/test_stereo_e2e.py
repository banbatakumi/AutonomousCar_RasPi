"""`ml_stereo_e2e` の抽出・Dataset・モデル・書き出し → Pi の `stereo_e2e_node` → planner までを合成した記録で通す。

照合そのもの（`StereoProcessor`）は `raspi/tests/test_stereo_node.py` が見ているので、ここでは差し替える
（合成した記録の画像は小さく、校正した 640×480 の平行化が使えない）。
"""

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

import numpy as np  # noqa: E402
import torch  # noqa: E402

from ml_e2e.tests.test_extract import _write  # noqa: E402
from ml_e2e.tests.test_model_dataset import _jpeg  # noqa: E402
from ml_e2e.train import _avail, losses  # noqa: E402
from ml_stereo_e2e import extract as S  # noqa: E402
from ml_stereo_e2e.dataset import StereoDataset, StereoSpec, load_records, split_records  # noqa: E402
from ml_stereo_e2e.export_onnx import build_model, export, verify_parity  # noqa: E402
from ml_stereo_e2e.model import StereoE2EModel  # noqa: E402
from raspi.core.cam_e2e_preproc import PREPROC_VERSION_STEREO  # noqa: E402
from raspi.core.stereo_e2e_preproc import (DISP_VERSION, PREPROC_VERSION, band_geometry, crop_disp,  # noqa: E402
                                           disp_margin_cols, disp_to_bytes, yaw_shift_disp)
from raspi.core.stereo_match import MatchParams  # noqa: E402
from raspi.msgs import StereoDisp, VehicleState  # noqa: E402

K = 8
HW = (6, 20)
SETTINGS = S.stereo_settings(MatchParams(num_disp=16), (150, 174), HW)


class _FakeProc:
    """`StereoProcessor` の代わり。左の列ほど近い（視差が大きい）帯を返す。"""

    match = MatchParams(num_disp=16)
    rect = SimpleNamespace(f=268.5, cx=320.0)
    last_valid_frac = 0.9

    def process(self, img_r, img_l, fmt):
        d = np.tile(np.linspace(15.0, 1.0, 16 + 160, dtype=np.float32), (24, 1))
        d[:, :16] = -1.0
        return None, d


class _Data(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.dir = Path(cls.tmp.name) / "data"
        cls.dir.mkdir()
        cfg = S.E.ExtractConfig(k=K, ds=0.25, self_sources=S.SELF_SOURCES)
        meta = {"config": cfg.to_json(), "stereo": SETTINGS, "records": {}}
        with mock.patch.object(S, "make_processor", lambda v, s: _FakeProc()):
            for name in ("a", "b", "c"):
                _write(Path(cls.tmp.name) / f"{name}.mcap", duration_s=8.0, jpeg=_jpeg())
                meta["records"][name] = S.extract_recording(Path(cls.tmp.name) / f"{name}.mcap", cls.dir, cfg,
                                                            SETTINGS)
        (cls.dir / "dataset.json").write_text(json.dumps(meta))
        cls.meta = meta
        cls.records, _ = load_records(cls.dir)
        cls.spec = StereoSpec(size=(64, 32), crop_x=0.1, k=K, max_steer=0.4, disp_hw=HW)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()


class TestExtractAndDataset(_Data):
    def test_extract_adds_disparity_band(self):
        info = self.meta["records"]["a"]
        self.assertGreater(info["samples"], 10)
        self.assertEqual(info["disp_ok"], info["samples"])
        cv = info["calib_values"]                    # 記録の校正から求めた学習の値（レビュー M10）
        self.assertAlmostEqual(cv["band_f"], 268.5 / ((640 - 16) // HW[1]))   # Pi は 640 幅で照合する
        self.assertEqual((cv["band_f"], cv["band_cx"]), band_geometry(268.5, 320.0, 1.0, 16, HW[1]))
        for key in ("fx_px", "fx_px_left", "max_steer", "steer_link", "image_w"):
            self.assertIn(key, cv)
        d = self.records[0].data
        self.assertEqual(d["disp"].shape[1:], (2, *HW))
        self.assertEqual(d["disp"].dtype, np.float16)
        # 左の列ほど視差が大きい・全部のブロックで照合が出ている
        self.assertGreater(float(d["disp"][0, 0, :, 0].mean()), float(d["disp"][0, 0, :, -1].mean()))
        np.testing.assert_allclose(d["disp"][0, 1].astype(np.float32), 1.0)

    def test_self_runs_are_not_used_as_teacher(self):
        cfg = S.E.ExtractConfig(k=K, ds=0.25, self_sources=S.SELF_SOURCES)
        self.assertEqual(cfg.to_json()["self_sources"], list(S.SELF_SOURCES))
        # 既定（E2E 融合）は `dataset.json` に項目を書かない（前に抽出した標本と比べられる）
        self.assertNotIn("self_sources", S.E.ExtractConfig(k=K).to_json())

    def test_items_and_unmatched_samples(self):
        ds = StereoDataset(self.records, self.dir, self.spec)
        it = ds[3]
        self.assertEqual(tuple(it["image"].shape), (6, 32, 64))
        self.assertEqual(tuple(it["disp"].shape), (2, *HW))
        self.assertEqual(tuple(it["path"].shape), (K, 2))
        n = len(ds)
        self.records[0].data["disp_ok"][:5] = False             # 照合できなかった標本は使わない
        try:
            self.assertEqual(len(StereoDataset(self.records, self.dir, self.spec)), n - 5)
        finally:
            self.records[0].data["disp_ok"][:5] = True

    def test_split_and_yaw_aug_moves_disp_and_path_together(self):
        train, val = split_records(self.records)
        self.assertEqual([r.name for r in val], ["c"])
        geom = {name: (c["band_f"], c["band_cx"])
                for name, c in ((n, self.meta["records"][n]["calib_values"]) for n in ("a", "b", "c"))}
        m = max(disp_margin_cols(np.radians(6), f, cx, HW[1]) for f, cx in geom.values())
        ds = StereoDataset(self.records, self.dir, self.spec, band_geom=geom, crop_cols=m, augment=True,
                           yaw_max_deg=6, fx_px=100.0, fx_px_left=100.0, load_images=False)
        plain = StereoDataset(self.records, self.dir, self.spec, band_geom=geom, crop_cols=m, load_images=False)[3]
        self.assertEqual(tuple(plain["disp"].shape), (2, HW[0], HW[1] - 2 * m))
        for psi in (6, -6):
            with mock.patch.object(ds, "_draw_aug", return_value=(psi, False)):
                it = ds[3]
            # ★ ずらしで空いた列（照合できない＝0）は窓に入らない（実車に無い帯を手がかりにさせない。レビュー M9）
            self.assertTrue((it["disp"][1] > 0.99).all(), psi)
        with mock.patch.object(ds, "_draw_aug", return_value=(6, False)):
            it = ds[3]
        # 車が左へ向く → 物は右へ。帯は左の列ほど視差が大きいので、同じ列に元より大きい視差が来る
        self.assertGreater(float(it["disp"][0, :, 5].mean()), float(plain["disp"][0, :, 5].mean()))
        # 経路は −ψ 回る（左へ向いたので経路は右へ）
        self.assertLess(float(it["path"][-1, 1]), float(plain["path"][-1, 1]))


class TestShift(unittest.TestCase):
    def test_shift_follows_the_perspective(self):
        """列の向き atan((c − cx)/f) が ψ だけ回る（端の列ほど大きく動く）。はみ出しは 0。"""
        f, cx, w = 10.0, 9.5, 20
        ramp = np.tile(np.arange(w, dtype=np.float32) + 100.0, (2, 1, 1))   # 値 = 元の列 + 100
        psi = np.radians(10)
        out = yaw_shift_disp(ramp, psi, f, cx)
        c = np.arange(w)
        src = cx + f * np.tan(np.arctan((c - cx) / f) - psi)          # 新しい列 c に来る元の列
        inside = (src >= 0) & (src <= w - 1)
        np.testing.assert_allclose(out[0, 0, inside], src[inside] + 100.0, rtol=1e-5)
        self.assertTrue((src[inside] < c[inside]).all())               # 左へ向く → 物は右へ動く
        shift = c - src
        self.assertGreater(shift[inside][-1], shift[inside][len(shift[inside]) // 2])   # 端ほど大きく動く
        self.assertEqual(float(yaw_shift_disp(ramp, np.radians(60), f, cx)[0, 0, 0]), 0.0)
        np.testing.assert_array_equal(yaw_shift_disp(ramp, 0.0, f, cx), ramp)

    def test_margin_keeps_the_window_filled(self):
        f, cx, w = 8.66, 9.2, 20
        m = disp_margin_cols(np.radians(6), f, cx, w)
        ones = np.ones((2, 1, w), np.float32)
        for deg in (-6, -3, 3, 6):
            self.assertTrue((crop_disp(yaw_shift_disp(ones, np.radians(deg), f, cx), m) > 0.999).all(), deg)
        with self.assertRaises(ValueError):
            disp_margin_cols(np.radians(80), f, cx, w)


class TestModel(unittest.TestCase):
    def test_forward_loss_and_masked_inputs(self):
        torch.manual_seed(0)
        m = StereoE2EModel(disp_hw=HW, k=K, xy_scale=2.0, pretrained=False)
        n = 3
        image, disp, state = torch.rand(n, 6, 32, 64), torch.rand(n, 2, *HW), torch.rand(n, 3)
        out = m(image, disp, state, _avail(n, "cpu", drop_image=0.3, drop_lidar=0.3))
        self.assertEqual(tuple(out.shape), (n, 4 * K))
        batch = {"path": torch.zeros(n, K, 2), "pt_ok": torch.ones(n, K), "path_v": torch.zeros(n, K),
                 "v_ok": torch.ones(n)}
        ls = losses(out, batch, K, 2.0)
        self.assertTrue(torch.isfinite(ls["path"]) and torch.isfinite(ls["v"]))
        # 視差を伏せたら視差の中身は効かない
        a = torch.tensor([[1.0, 0.0, 1.0]]).expand(n, 3)
        m.eval()
        torch.testing.assert_close(m(image, disp, state, a), m(image, torch.rand(n, 2, *HW), state, a))


def _cfg() -> dict:
    return {"preproc_version": PREPROC_VERSION, "image_preproc_version": PREPROC_VERSION_STEREO,
            "input_size": [64, 32], "crop_x": 0.05, "disp_hw": list(HW), "disp_num_disp": 16,
            "disp_rows": [150, 174], "disp_scale": 1.0, "disp_version": DISP_VERSION, "disp_algo": "sgbm",
            "disp_block": 5, "disp_uniqueness": 25, "disp_crop_cols": 2, "disp_col_f": 8.66, "disp_col_cx": 9.2, "k": K, "ds": 0.25, "xy_scale": 2.0, "max_steer": 0.4,
            "speed_ref": 5.0, "yaw_rate_ref": 6.0, "steer_link": [1.0, 0.0], "maps": ["m"], "train_records": ["a"]}


def _disp(t: int, **over) -> StereoDisp:
    arr = np.full((2, *HW), 0.3, np.float32)
    kw = dict(data=disp_to_bytes(arr), h=HW[0], w=HW[1], num_disp=16, scale=1.0, rows=[150, 174],
              preproc_version=DISP_VERSION, algo="sgbm", block=5, uniqueness=25, t_capture=t, seq=1)
    kw.update(over)
    return StereoDisp(**kw)


class TestExportToNodeAndPlanner(unittest.TestCase):
    def test_export_node_and_planner(self):
        from raspi.auto.registry import merged_params
        from raspi.auto.stereo_e2e import StereoE2E
        from raspi.nodes.stereo_e2e_node import StereoE2ENode, load_model

        with tempfile.TemporaryDirectory() as d:
            run = Path(d) / "run"
            run.mkdir()
            cfg = _cfg()
            (run / "train_config.json").write_text(json.dumps(cfg))
            torch.manual_seed(0)
            model = build_model(cfg)
            with torch.no_grad():                    # 経路をまっすぐ前・自信ありにする
                model.path_head.weight.zero_()
                model.path_head.bias.zero_()
                model.path_head.bias[:K] = torch.linspace(0.125, 1.0, K)
                model.path_head.bias[2 * K:3 * K] = 0.0
                model.path_head.bias[3 * K:] = -6.0
            torch.save(model.state_dict(), run / "best.pt")
            out = Path(d) / "models" / "t.onnx"
            contract = export(run, out)
            self.assertLess(verify_parity(out, model, cfg), 1e-3)
            self.assertEqual(contract["cams"], ["front", "left"])

            node = StereoE2ENode(model=load_model(out), model_name="t")
            t0 = 10 * 10**9
            img = np.zeros((48, 96, 3), np.uint8)
            vs = VehicleState(t_capture=t0 + 50_000_000, speed=1.0)
            plan = node.infer_once(disp=_disp(t0), rgb=img, rgb_left=img, vs=vs, now_ns=t0 + 50_000_000)
            self.assertTrue(plan.ready, plan.reason)
            self.assertTrue(plan.has_image and plan.has_lidar)              # has_lidar = 視差を使った
            self.assertEqual(plan.t_capture, t0)
            self.assertAlmostEqual(plan.path_x[-1], 2.0, places=3)
            # 片目なら画像を伏せて視差だけで推論する
            self.assertFalse(node.infer_once(disp=_disp(t0), rgb=img, rgb_left=None, vs=vs,
                                             now_ns=t0 + 50_000_000).has_image)
            # 視差の設定が契約と違う・古い・無いときは推論しない
            for bad, now in ((_disp(t0, num_disp=32), t0), (_disp(t0, rows=[140, 174]), t0),
                             (_disp(t0, algo="bm"), t0), (_disp(t0, uniqueness=10), t0),
                             (_disp(t0, algo="", block=0, uniqueness=0), t0),
                             (_disp(t0, h=4, w=30), t0), (_disp(t0), t0 + 10**9), (None, t0)):
                p = node.infer_once(disp=bad, rgb=img, rgb_left=img, vs=vs, now_ns=now)
                self.assertFalse(p.ready)

            # planner（既定は MPC）が経路を追う
            planner = StereoE2E()
            for i in range(30):
                planner.on_vehicle_state(VehicleState(t_capture=t0 - 200_000_000 + i * 10_000_000, speed=1.0))
            st = planner.plan(plan, vs, merged_params(StereoE2E.id, {}), 0.1)
            self.assertTrue(st.ready, st.reason)
            self.assertIn("視差", st.reason)
            self.assertLess(abs(st.target_steer), math.radians(3))
