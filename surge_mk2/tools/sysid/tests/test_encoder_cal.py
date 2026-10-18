"""エンコーダ校正（`raspi/auto/sysid_encoder.py` → `tools/sysid/encoder_cal.py` → `[control]`）の確認。

    cd surge_mk2 && .venv/bin/python -m pytest tools/sysid/tests/test_encoder_cal.py -q

真値の分かっているベンチ（`bench.EncoderTruth`: 出力の範囲が全域でない・周期誤差 ±1.1°/±2.7°・左右で取付の
向きが逆）で試験を閉ループで回し、解析が真値を復元するか、復元した校正を STM32 に入れると走行距離が真の値に
なるかを見る。
"""

from __future__ import annotations

import ctypes
import math
import shutil
import subprocess
import sys
import tempfile
import tomllib
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))  # surge_mk2/

from sim.vehicle import DEFAULT_SPEC_PATH  # noqa: E402
from tools.ctrl_tune import fw  # noqa: E402
from tools.sysid import encoder_cal as ec  # noqa: E402
from tools.sysid import toml_update  # noqa: E402
from tools.sysid.bench import (TRUTH, EncoderTruth, encoder_record, run_test,  # noqa: E402
                               write_encoder_mcap)

ENC = EncoderTruth()
#: 許容誤差。段差（hi − lo）、角度に直した結果 [rad]（`_angle_err`）
SPAN_TOL = 0.001
H_TOL = math.radians(0.1)
#: 校正を入れた後の走行距離の誤差（往復の総距離に対する割合）
ODOM_TOL = 0.002


def _angle_err(a: ec.Cal, b: ec.Cal) -> float:
    """2つの校正が読みを角度に直した結果の差の最大 [rad]（一定のずれを除く）。`lo` は角度の原点を決めるだけで
    （STM32 も同じ `lo` で直す）、どこに置いても角度の差（＝回転量）は変わらないので、係数どうしでは比べない。"""
    u = np.linspace(max(a.lo, b.lo) + 1e-3, min(a.hi, b.hi) - 1e-3, 2000)
    d = ec._wrap_pi(a.angle(u) - b.angle(u))
    return float(np.max(np.abs(d - np.mean(d))))


def _run(enc: EncoderTruth, seed: int) -> dict:
    cap: dict = {}
    run_test("sysid_encoder", TRUTH, seed=seed, encoder=enc, capture=cap, max_s=120)
    return cap


class TestCalModel(unittest.TestCase):
    def test_angle_matches_firmware(self):
        """`Cal.angle` は STM32 の `EncoderCal_Angle` と同じ値を返す（ホストの gcc でヘッダを直にコンパイル）。"""
        header = fw.FW_DIR / "lib" / "encoder_cal" / "encoder_cal.h"
        gcc = shutil.which("gcc")
        if gcc is None or not header.exists():
            self.skipTest("gcc かファームのリポジトリが無い")
        with tempfile.TemporaryDirectory() as d:
            src = Path(d) / "w.c"
            src.write_text('#include "encoder_cal.h"\n'
                           "float angle(float lo, float hi, float a, float b, float c, float e, float u) {\n"
                           "  EncoderCal k = {lo, hi, a, b, c, e};\n  return EncoderCal_Angle(&k, u);\n}\n")
            lib = Path(d) / "w.so"
            subprocess.run([gcc, "-shared", "-fPIC", "-O2", f"-I{header.parent}", str(src), "-o", str(lib), "-lm"],
                           check=True)
            so = ctypes.CDLL(str(lib))
            so.angle.restype = ctypes.c_float
            so.angle.argtypes = [ctypes.c_float] * 7
            for cal in (ENC.cal[0], ENC.cal[1], ec.Cal(), ec.Cal(lo=0.3, hi=0.6)):   # 最後は壊れた範囲（恒等）
                u = np.linspace(0.0, 0.999, 997)
                py = cal.angle(u)
                c = np.array([so.angle(cal.lo, cal.hi, cal.h1s, cal.h1c, cal.h2s, cal.h2c, float(x)) for x in u])
                err = np.abs(ec._wrap_pi(py - c))
                self.assertLess(float(err.max()), 2e-5)

    def test_angle_inverts_ripple(self):
        """真の角度 θ → 読み u（モデル）→ `Cal.angle` で θ に戻る（不動点反復2回の誤差）。"""
        cal = ENC.cal[1]
        theta = np.linspace(0.0, 2 * math.pi, 500, endpoint=False)
        u = cal.lo + (cal.hi - cal.lo) * np.mod(theta + cal.ripple(theta), 2 * math.pi) / (2 * math.pi)
        self.assertLess(float(np.abs(ec._wrap_pi(cal.angle(u) - theta)).max()), math.radians(0.005))


class TestRecovery(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cap = _run(ENC, seed=0)
        cls.res = ec.analyze(encoder_record(cls.cap))

    def _check(self, res: ec.Result):
        self.assertEqual(res.warnings, [])
        for i, side in enumerate(ec.SIDES):
            c = ENC.cal[i]
            v = res.values
            self.assertAlmostEqual(v[f"enc_{side}_hi"] - v[f"enc_{side}_lo"], c.hi - c.lo, delta=SPAN_TOL)
            self.assertLess(_angle_err(ec.Cal.from_params(v, side), c), H_TOL, msg=side)

    def test_recovers_truth(self):
        self._check(self.res)

    def test_through_mcap(self):
        """実機と同じロガーで mcap に書いて読み直しても同じ結果。"""
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "enc.mcap"
            write_encoder_mcap(self.cap, ENC, path)
            rec = ec.load_record(path)
        res = ec.analyze(rec)
        self._check(res)
        for k in ec.CONTROL_KEYS:
            self.assertAlmostEqual(res.values[k], self.res.values[k], delta=1e-4)   # mcap の float32 の丸め

    def test_calibrated_firmware(self):
        """求めた校正を STM32 に入れて録り直す: 走行距離が真の値に合い、解析は同じ値を返す（生の読みから
        求めるので、前の校正に依らない）。"""
        v = self.res.values
        enc = replace(ENC, active_cal=tuple(ec.Cal.from_params(v, s) for s in ec.SIDES))
        cap = _run(enc, seed=1)
        poses = np.array(cap["poses"])
        odom = np.array(cap["vs"][-1][1]["odom_dist"])
        true = poses[-1, 4:6]
        # 往復で戻るので終点の累積は小さい。行き帰りの総距離で比べる
        dist = np.abs(np.diff(poses[:, 4:6], axis=0)).sum(axis=0)
        np.testing.assert_allclose(odom, true, atol=dist.max() * ODOM_TOL)
        res2 = ec.analyze(encoder_record(cap))
        self.assertEqual(res2.warnings, [])
        for side in ec.SIDES:
            self.assertLess(_angle_err(ec.Cal.from_params(res2.values, side), ec.Cal.from_params(v, side)), H_TOL)

    def test_calibrated_speed_has_no_ripple(self):
        """校正なしの左右の前輪の周速には1回転の周期のうねりが乗り、校正後は消える（記録の odom_dist の差分）。"""
        def ripple_rms(cap):
            t = np.array([x[0] for x in cap["vs"]])
            od = np.array([x[1]["odom_dist"][1] for x in cap["vs"]])
            idx = ec.coast_segments(encoder_record(cap))[0]
            v = np.diff(od[idx]) / np.diff(t[idx])
            # 惰行の滑らかな減速を2次式で除いた残り
            tt = t[idx][1:]
            return float(np.std(v - np.polyval(np.polyfit(tt, v, 2), tt)))

        v = self.res.values
        enc = replace(ENC, active_cal=tuple(ec.Cal.from_params(v, s) for s in ec.SIDES))
        raw, cal = ripple_rms(self.cap), ripple_rms(_run(enc, seed=0))
        self.assertLess(cal, 0.5 * raw)


class TestApplyControl(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.toml = Path(self.tmp.name) / "vehicle.toml"
        shutil.copy(DEFAULT_SPEC_PATH, self.toml)

    def tearDown(self):
        self.tmp.cleanup()

    def test_writes_control(self):
        changed = toml_update.apply_control(self.toml, {"enc_left_h1s": 0.0123, "kf_jerk_psd": 42.0})
        self.assertEqual(sorted(changed), ["enc_left_h1s", "kf_jerk_psd"])
        c = tomllib.loads(self.toml.read_text(encoding="utf-8"))["control"]
        self.assertEqual(c["enc_left_h1s"], 0.0123)
        self.assertEqual(c["kf_jerk_psd"], 42.0)

    def test_refuses_out_of_range(self):
        before = self.toml.read_bytes()
        with self.assertRaises(ValueError):
            toml_update.apply_control(self.toml, {"enc_left_h1s": 0.2})
        with self.assertRaises(ValueError):
            toml_update.apply_control(self.toml, {"speed_kp": 1.0})
        self.assertEqual(self.toml.read_bytes(), before)

    def test_ranges_match_firmware(self):
        """`CONTROL_RANGES` は STM32 の `control_params.h` の範囲と同じ。"""
        table = fw.param_table()
        for key, (lo, hi) in toml_update.CONTROL_RANGES.items():
            self.assertIn(key, table)
            self.assertAlmostEqual(table[key].lo, lo, places=9, msg=key)
            self.assertAlmostEqual(table[key].hi, hi, places=9, msg=key)


if __name__ == "__main__":
    unittest.main()
