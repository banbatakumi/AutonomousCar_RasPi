"""記録 → 抽出 → 学習 → エクスポート → 実車側の読み込み → 評価 を1本で通す結合テスト。

各スクリプトを**実際のコマンドとして**起動する（操作パネルが呼ぶのと同じ形）。
段ごとの単体テストは通っていても、段と段の間の約束（manifest の列、
`train_config.json` のキー、モデル同梱 JSON の契約）がずれると全体は動かない
——ここはそのずれを捕まえるためのテスト。

精度は問わない（1エポック・事前学習なし・極小解像度）。実データでの精度は
操作パネルの「評価」タブで見る。
"""

import csv
import io
import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
ML_DIR = REPO_ROOT / "ml_cam_e2e"
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(ML_DIR))

import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

import samples as S  # noqa: E402
from eval_model import EVAL_COLUMNS, SPLIT_UNUSED, read_eval, summarize  # noqa: E402
from extract_pairs import CMD_MCAP_TOPIC, STATE_MCAP_TOPIC  # noqa: E402
from raspi.msgs.types import DriveCmd, VehicleState  # noqa: E402
from raspi.nodes.cam_e2e_node import load_model  # noqa: E402
from raspi.rec.mcap_log import McapLog  # noqa: E402

MS = 1_000_000
NS = 1_000_000_000


def _jpeg(left: bool) -> bytes:
    """左右どちらかが明るい絵（前カメラと同じ 16:9）。"""
    arr = np.full((90, 160, 3), 40, dtype=np.uint8)
    if left:
        arr[:, :80] = 210
    else:
        arr[:, 80:] = 210
    buf = io.BytesIO()
    Image.fromarray(arr, mode="RGB").save(buf, format="JPEG", quality=80)
    return buf.getvalue()


def _write_record(path: Path) -> dict:
    """30秒の走行記録を書く。5Hz の画像・50Hz の指令・実速度つき。

    区間ごとに中身を変えてあり、抽出・学習側のフィルタが全部働くことを確かめる:
      0〜20s  速度モードで走行（手本）。2秒ごとに左右へ切る
      20〜24s トルクモード（抽出で落ちる）
      24〜27s 止まって待機（抽出は通るが学習で落ちる）
      27〜30s 後退（同上）
    """
    jpg = {True: _jpeg(True), False: _jpeg(False)}
    n = {"drive": 0, "torque": 0, "wait": 0, "reverse": 0}
    with McapLog(path, t0_mono_ns=0, t0_unix_ns=0) as log:
        for i in range(30 * 50):
            t = i * 20 * MS
            sec = t / NS
            left = int(sec // 2) % 2 == 0
            if sec < 20:
                cmd = DriveCmd(mode=1, arm=True, target_steer=0.3 if left else -0.3,
                               target_speed=0.8 if left else 0.4)
                speed, kind = cmd.target_speed, "drive"
            elif sec < 24:
                cmd = DriveCmd(mode=1, arm=True, torque_mode=True, target_torque=0.05)
                speed, kind = 0.5, "torque"
            elif sec < 27:
                cmd = DriveCmd(mode=1, arm=True, target_speed=0.0)
                speed, kind = 0.0, "wait"
            else:
                cmd = DriveCmd(mode=1, arm=True, target_speed=-0.3)
                speed, kind = -0.3, "reverse"
            log.write(CMD_MCAP_TOPIC, cmd, t_mono_ns=t)
            log.write(STATE_MCAP_TOPIC, VehicleState(speed=speed), t_mono_ns=t)
            if i % 10 == 0:                                   # 5Hz
                log.write_viz_image(jpg[left], "front", t)
                n[kind] += 1
    return n


def _run(*argv: str) -> subprocess.CompletedProcess:
    proc = subprocess.run([sys.executable, *argv], cwd=REPO_ROOT, capture_output=True,
                          text=True, timeout=600)
    if proc.returncode != 0:
        raise AssertionError(f"{' '.join(argv)} が失敗（{proc.returncode}）\n"
                             f"{proc.stdout}\n{proc.stderr}")
    return proc


class TestPipeline(unittest.TestCase):
    def test_record_to_evaluated_model(self):
        with tempfile.TemporaryDirectory() as d:
            tmp = Path(d)
            mcap = tmp / "run.mcap"
            n = _write_record(mcap)
            run_dir = tmp / "runs" / "v1"
            frames = run_dir / "frames"
            onnx_path = tmp / "models" / "cam_e2e" / "v1.onnx"

            # ── 抽出 ──
            out = _run(str(ML_DIR / "extract_pairs.py"), str(mcap), "--out", str(frames)).stdout
            self.assertIn(f"トルクモード {n['torque']}枚", out)
            all_samples = S.load_manifest(frames)
            self.assertEqual(len(all_samples), n["drive"] + n["wait"] + n["reverse"])
            with open(frames / "manifest.csv", newline="") as f:
                self.assertEqual(next(csv.reader(f)), S.MANIFEST_COLUMNS)

            # 人が1区間（4〜6秒）を除外したことにする
            S.save_exclusions(frames, [S.Exclusion("run.mcap", "front", 4 * NS, 6 * NS - 1,
                                                   "テスト")])
            n_excluded = sum(1 for s in all_samples if 4 * NS <= s.t_ns < 6 * NS)
            n_used = n["drive"] - n_excluded

            # ── 学習 ──
            out = _run(str(ML_DIR / "train.py"), "--frames", str(frames), "--out", str(run_dir),
                       "--epochs", "1", "--batch-size", "8", "--size", "64x32",
                       "--no-pretrained", "--block-s", "1", "--val-ratio", "0.3").stdout
            self.assertIn(f"除外区間 {n_excluded}件", out)
            self.assertIn(f"静止 {n['wait']}件", out)
            self.assertIn(f"後退 {n['reverse']}件", out)
            self.assertRegex(out, r"epoch\s+1/1\s+loss=[\d.]+\s+val_mae=[\d.]+\s+val_speed_mae=")
            cfg = json.loads((run_dir / "train_config.json").read_text())
            self.assertEqual(cfg["input_size"], [64, 32])
            self.assertEqual(cfg["n_train"] + cfg["n_val"], n_used)
            self.assertGreater(cfg["n_val"], 0)
            self.assertAlmostEqual(cfg["speed_ref"], 0.8)     # 手本の最高速度
            self.assertTrue((run_dir / "best.pt").exists())

            # ── エクスポート（解像度も基準も train_config.json から） ──
            _run(str(ML_DIR / "export_onnx.py"), "--checkpoint", str(run_dir / "best.pt"),
                 "--out", str(onnx_path))
            contract = json.loads(onnx_path.with_suffix(".json").read_text())
            self.assertEqual(contract["input_size"], [64, 32])
            self.assertAlmostEqual(contract["speed_ref"], 0.8)
            self.assertAlmostEqual(contract["max_steer"], cfg["max_steer"])

            # ── 実車側の読み込み（`cam_e2e_node` が使う関数そのもの） ──
            model = load_model(onnx_path)
            frame_bgr = np.zeros((360, 640, 3), dtype=np.uint8)
            steer, speed = model.infer(frame_bgr[..., ::-1])
            self.assertTrue(math.isfinite(steer) and -1.0 <= steer <= 1.0)
            self.assertTrue(math.isfinite(speed) and 0.0 <= speed <= 1.0)

            # ── 評価 ──
            _run(str(ML_DIR / "eval_model.py"), "--frames", str(frames), "--run", str(run_dir),
                 "--model", str(onnx_path))
            with open(run_dir / "eval.csv", newline="") as f:
                self.assertEqual(next(csv.reader(f)), EVAL_COLUMNS)
            rows = read_eval(run_dir / "eval.csv")
            self.assertEqual(len(rows), len(all_samples))
            self.assertEqual(sum(1 for r in rows if r["split"] == SPLIT_UNUSED),
                             n_excluded + n["wait"] + n["reverse"])
            summary = summarize(rows)
            self.assertEqual(summary["val"]["n"], cfg["n_val"])
            self.assertEqual(summary["train"]["n"], cfg["n_train"])
            self.assertTrue(math.isfinite(summary["val"]["steer_mae_deg"]))
            for r in rows:
                self.assertLessEqual(abs(r["steer_pred"]), contract["max_steer"] + 1e-6)
                self.assertTrue(0.0 <= r["speed_pred"] <= contract["speed_ref"] + 1e-6)


class TestLearnsTheDemonstration(unittest.TestCase):
    """左右どちらが明るいかで舵と速度が決まるだけの記録を、実際に学習できること。

    配管が通る（上のテスト）だけでは、符号の向き・正規化の基準・学習時と
    推論時の食い違いは分からない。学習後の**ONNX を実車と同じ経路で通した
    出力**が手本に合うことを見る。

    過去にここで見つかった不具合: BatchNorm の移動平均が追従せず、学習の
    損失は下がるのに検証・ONNX では直進（誤差 17°）しか出さなかった
    （`model.py` の `_BN_MOMENTUM`）。
    """

    def test_exported_model_reproduces_steer_sign_and_speed(self):
        with tempfile.TemporaryDirectory() as d:
            tmp = Path(d)
            mcaps = []
            for i in range(3):
                mcaps.append(tmp / f"run{i}.mcap")
                _write_record(mcaps[-1])
            run_dir = tmp / "runs" / "v1"
            frames = run_dir / "frames"
            onnx_path = tmp / "models" / "cam_e2e" / "v1.onnx"

            _run(str(ML_DIR / "extract_pairs.py"), *map(str, mcaps), "--out", str(frames))
            # 手本の速度は左右で違う（0.8/0.4）ので、左右反転の拡張は切る。
            # 速度の重みは既定が 0（舵だけ）なので、速度の出力も見るここでは明示する
            _run(str(ML_DIR / "train.py"), "--frames", str(frames), "--out", str(run_dir),
                 "--epochs", "12", "--size", "64x32", "--no-pretrained", "--no-flip",
                 "--block-s", "1", "--val-ratio", "0.3", "--speed-weight", "0.5")
            _run(str(ML_DIR / "export_onnx.py"), "--checkpoint", str(run_dir / "best.pt"),
                 "--out", str(onnx_path))
            _run(str(ML_DIR / "eval_model.py"), "--frames", str(frames), "--run", str(run_dir),
                 "--model", str(onnx_path))

            rows = [r for r in read_eval(run_dir / "eval.csv") if r["split"] == "val"]
            summary = summarize(rows)["val"]
            # 常に直進・常に平均速度と答えるモデルは 17°・0.2m/s
            self.assertLess(summary["steer_mae_deg"], 6.0)
            self.assertLess(summary["speed_mae"], 0.1)
            left = [r["steer_pred"] for r in rows if r["steer_true"] > 0]
            right = [r["steer_pred"] for r in rows if r["steer_true"] < 0]
            self.assertGreater(sum(left) / len(left), 0.15, "左の手本に左へ切っていない")
            self.assertLess(sum(right) / len(right), -0.15, "右の手本に右へ切っていない")


if __name__ == "__main__":
    unittest.main()
