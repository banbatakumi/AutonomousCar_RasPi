"""`ml_cam/export_onnx.py` のテスト。

**実データ・実学習は要らない。** ランダム初期化のモデルをそのままエクスポート
し、PyTorch と ONNXRuntime の出力が一致すること（＝配管が正しいこと）だけを
確認する。推論の「正しさ」（意味のあるセグメンテーションかどうか）は
実データが学習された後でしか確認できない。
"""

import json
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # ml_cam/

import onnxruntime as ort  # noqa: E402
import torch  # noqa: E402

from export_onnx import MEAN, STD, THRESHOLD, export, verify_parity  # noqa: E402
from model import DrivableSegModel  # noqa: E402
from raspi.core.cam_e2e_preproc import PREPROC_VERSION  # noqa: E402


class TestExportOnnx(unittest.TestCase):
    def test_export_writes_onnx_and_config(self):
        with tempfile.TemporaryDirectory() as d:
            ckpt_path = Path(d) / "model.pt"
            model = DrivableSegModel(pretrained=False)
            torch.save(model.state_dict(), ckpt_path)

            out_path = Path(d) / "model.onnx"
            export(ckpt_path, out_path, (64, 48))

            self.assertTrue(out_path.exists())
            cfg_path = out_path.with_suffix(".json")
            self.assertTrue(cfg_path.exists())
            cfg = json.loads(cfg_path.read_text())
            self.assertEqual(cfg["input_size"], [64, 48])
            self.assertEqual(cfg["mean"], MEAN)
            self.assertEqual(cfg["std"], STD)
            self.assertEqual(cfg["threshold"], THRESHOLD)
            self.assertEqual(cfg["note"], "")   # 省略時は空文字
            self.assertEqual(cfg["resize"], "area")
            self.assertEqual(cfg["preproc_version"], PREPROC_VERSION)

    def _ckpt_with_train_config(self, d, **cfg):
        ckpt_path = Path(d) / "best.pt"
        torch.save(DrivableSegModel(pretrained=False).state_dict(), ckpt_path)
        (Path(d) / "train_config.json").write_text(json.dumps(cfg))
        return ckpt_path

    def test_size_defaults_to_the_trained_size(self):
        with tempfile.TemporaryDirectory() as d:
            ckpt = self._ckpt_with_train_config(d, input_size=[64, 48], preproc_version=PREPROC_VERSION)
            out = Path(d) / "m.onnx"
            self.assertEqual(export(ckpt, out), (64, 48))
            self.assertEqual(json.loads(out.with_suffix(".json").read_text())["input_size"], [64, 48])

    def test_size_differing_from_the_trained_size_is_rejected(self):
        """学習 64x48 を 128x96 で書き出すと、変換の突き合わせでは見つからず精度だけが落ちる。"""
        with tempfile.TemporaryDirectory() as d:
            ckpt = self._ckpt_with_train_config(d, input_size=[64, 48])
            with self.assertRaises(ValueError):
                export(ckpt, Path(d) / "m.onnx", (128, 96))

    def test_missing_size_everywhere_is_an_error(self):
        with tempfile.TemporaryDirectory() as d:
            ckpt = self._ckpt_with_train_config(d)
            with self.assertRaises(ValueError):
                export(ckpt, Path(d) / "m.onnx")

    def test_stale_preproc_version_is_rejected(self):
        """旧い前処理で学習したモデルに今の版を書くと、ノードの照合をすり抜ける。"""
        with tempfile.TemporaryDirectory() as d:
            ckpt = self._ckpt_with_train_config(d, input_size=[64, 48], preproc_version=PREPROC_VERSION - 1)
            with self.assertRaises(ValueError):
                export(ckpt, Path(d) / "m.onnx")

    def test_note_is_embedded_in_the_exported_json(self):
        """`ml_cam/app.py`の備考欄→`note.txt`→エクスポート、という経路で
        `<名前>.json`に書き込まれる自由記述の備考（`ml_lidar/export_onnx_rl.py`
        の同名テストと対称）。GUIのモデル選択（`AutoPanel.tsx`）
        が表示するので、`note`キーの往復を確認する。"""
        with tempfile.TemporaryDirectory() as d:
            ckpt_path = Path(d) / "model.pt"
            model = DrivableSegModel(pretrained=False)
            torch.save(model.state_dict(), ckpt_path)

            out_path = Path(d) / "model.onnx"
            export(ckpt_path, out_path, (64, 48), note="夜間走行用、露出補正あり")

            cfg = json.loads(out_path.with_suffix(".json").read_text())
            self.assertEqual(cfg["note"], "夜間走行用、露出補正あり")

    def test_onnxruntime_output_matches_pytorch(self):
        with tempfile.TemporaryDirectory() as d:
            ckpt_path = Path(d) / "model.pt"
            model = DrivableSegModel(pretrained=False)
            torch.save(model.state_dict(), ckpt_path)

            out_path = Path(d) / "model.onnx"
            export(ckpt_path, out_path, (64, 48))

            reloaded = DrivableSegModel(pretrained=False)
            reloaded.load_state_dict(torch.load(ckpt_path, map_location="cpu"))
            err = verify_parity(out_path, reloaded, (64, 48))
            self.assertLess(err, 1e-3)

    def test_onnx_file_loads_alone_without_its_export_directory(self):
        """`models/`への配置運用は`.onnx`単体をコピーする。**もとのエクスポート先
        ディレクトリに`.data`（外部重みファイル）が残ったままだと、その隣にある間は
        気づかず動いてしまう**——`.onnx`だけを別ディレクトリへコピーしてロードすることで、
        `torch.onnx.export`の既定`external_data=True`が割った重みを参照できずに
        失敗する回帰を検出する（`ml_lidar/export_onnx_rl.py`が実際に踏んだ罠と同じ）。
        """
        with tempfile.TemporaryDirectory() as d:
            ckpt_path = Path(d) / "model.pt"
            model = DrivableSegModel(pretrained=False)
            torch.save(model.state_dict(), ckpt_path)

            out_path = Path(d) / "model.onnx"
            export(ckpt_path, out_path, (64, 48))

            with tempfile.TemporaryDirectory() as lone_dir:
                lone_path = Path(lone_dir) / "renamed.onnx"
                shutil.copy(out_path, lone_path)          # .json も .data も持っていかない
                sess = ort.InferenceSession(str(lone_path), providers=["CPUExecutionProvider"])
                sess.run(None, {"input": torch.zeros(1, 3, 48, 64).numpy()})

    def test_mismatched_model_raises(self):
        """**エクスポートできただけでは何も保証されない**——別の重みで検証すると
        検出できることを確認する。"""
        with tempfile.TemporaryDirectory() as d:
            ckpt_path = Path(d) / "model.pt"
            model = DrivableSegModel(pretrained=False)
            torch.save(model.state_dict(), ckpt_path)

            out_path = Path(d) / "model.onnx"
            export(ckpt_path, out_path, (64, 48))

            different_model = DrivableSegModel(pretrained=False)   # 別の乱数初期化
            with self.assertRaises(ValueError):
                verify_parity(out_path, different_model, (64, 48), atol=1e-6)


if __name__ == "__main__":
    unittest.main()
