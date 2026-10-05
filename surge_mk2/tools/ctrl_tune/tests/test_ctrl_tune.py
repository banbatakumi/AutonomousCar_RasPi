"""`tools/ctrl_tune`: ファームのホスト実行・同定（試験→解析が真値を復元する）・最適化・書き戻し。

ファームのリポジトリ（`fw.FW_DIR`）と C コンパイラが要る。無ければ全部飛ばす。
"""

import shutil
import sys
import tempfile
import tomllib
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from raspi.auto.sysid_tyre import SysIdTyre  # noqa: E402
from raspi.auto.sysid_wheel import SysIdWheel  # noqa: E402
from raspi.auto.sysid_yawmoment import MOMENT_NM, SysIdYawMoment  # noqa: E402
from raspi.core.control_params import CONTROL_PARAM_IDS, load_control_params  # noqa: E402
from tools.ctrl_tune import fit, optimize, record, tuning  # noqa: E402
from tools.ctrl_tune import scenarios as S  # noqa: E402
from tools.ctrl_tune.fw import FW_DIR, MODE_SPEED, MODE_TORQUE, OUT, firmware  # noqa: E402
from tools.ctrl_tune.plant import DEFAULT_TOML, NOMINAL, Plant  # noqa: E402

HAVE_FW = (FW_DIR / "host" / "host_sim.c").exists() and shutil.which("cc") is not None
QUIET = replace(NOMINAL, front_noise_rad_s=0.0, rear_noise_rad_s=0.0, gyro_noise_rad_s=0.0)


def scenario(key: str) -> S.Scenario:
    return {s.key: s for s in S.ALL}[key]


@unittest.skipUnless(HAVE_FW, f"ファームのリポジトリ（{FW_DIR}）か cc がありません")
class TestParamTable(unittest.TestCase):
    def test_ids_match_protocol(self):
        """`control_params.h` の表と `protocol.toml` の param_id が同じ（名前も）。"""
        table = firmware().params
        self.assertEqual({k: v.id for k, v in table.items()}, CONTROL_PARAM_IDS)

    def test_vehicle_toml_is_within_firmware_range(self):
        table = firmware().params
        for key, value in load_control_params(DEFAULT_TOML).items():
            self.assertLessEqual(table[key].lo, value, key)
            self.assertLessEqual(value, table[key].hi, key)

    def test_defaults_are_within_range(self):
        for spec in firmware().params.values():
            self.assertTrue(spec.lo <= spec.default <= spec.hi, spec.name)


@unittest.skipUnless(HAVE_FW, "ファームがありません")
class TestLogic(unittest.TestCase):
    """ファームの制御の約束（2026-10-05 の見直しで直した点が戻っていないこと）。"""

    def setUp(self):
        self.fw = firmware()

    def test_no_intervention_when_cruising(self):
        for key in ("cruise", "cruise_turn"):
            m = S.run(self.fw, NOMINAL, scenario(key)).metrics
            self.assertEqual(m["tc_active"], 0.0, key)
            self.assertEqual(m["abs_active"], 0.0, key)

    def test_tc_works_from_standstill(self):
        """発進の空転を見る（以前は 0.25m/s 未満で TC ごと無効だった）。"""
        r = S.run(self.fw, NOMINAL, scenario("launch"))
        early = r.out[:400]                              # 最初の 0.2s
        self.assertTrue(np.any(early[:, OUT["flags"]].astype(int) & 1))
        self.assertLess(r.metrics["wheel_peak"] - r.col("speed")[-1], 1.5)

    def test_lifted_wheel_does_not_run_away(self):
        m = S.run(self.fw, NOMINAL, scenario("lift")).metrics
        self.assertLess(m["wheel_peak"], 2.5)

    def test_brake_holds_near_the_limit(self):
        sc = scenario("brake")
        m = S.run(self.fw, NOMINAL, sc).metrics
        self.assertTrue(m["stopped"])
        self.assertGreater(S.oracle(self.fw, NOMINAL, sc) / m["distance"], 0.9)
        self.assertLess(m["slipping"], 0.15)

    def test_speed_command_decel_does_not_lock(self):
        """車速指令の減速（ランプ3.0m/s²は後輪だけの制動の限界を超える）でも後輪がロックしない。"""
        m = S.run(self.fw, NOMINAL, scenario("decel_pi")).metrics
        self.assertLess(m["slipping"], 0.05)

    def test_cut_wheel_torque_is_not_moved_to_the_other_wheel(self):
        """片輪を絞った分を反対の輪へ載せない（以前は総和を保つために押し増していた）。"""
        sc = S.Scenario("t", "t", (S.Seg(0.6, MODE_TORQUE, 0.10, mu_left=0.2),), 0.8, tv=False)
        r = S.run(self.fw, QUIET, sc)
        self.assertLess(np.min(r.col("cmd_left")[200:]), 0.06)        # 左は絞られている
        self.assertLessEqual(np.max(r.col("cmd_right")), 0.10 + 1e-6)

    def test_test_moment_gives_exact_torque_difference(self):
        sc = S.Scenario("t", "t", (S.Seg(0.3, MODE_SPEED, 1.0, 3.0),
                                   S.Seg(0.2, MODE_SPEED, 1.0, 3.0, test_moment=MOMENT_NM),
                                   S.Seg(0.2, MODE_SPEED, 1.0, 3.0, test_moment=0.0)), 1.0)
        r = S.run(self.fw, QUIET, sc)
        diff = r.col("cmd_right") - r.col("cmd_left")
        expect = MOMENT_NM * 2.0 * fit.WHEEL_RADIUS_M / fit.REAR_TRACK_M
        self.assertAlmostEqual(float(np.median(diff[700:950])), expect, places=4)
        self.assertGreater(float(np.mean(r.col("yaw_rate")[800:1000])), 0.02)     # 左へ回る

    def test_dead_front_encoder_does_not_runaway(self):
        """前輪エンコーダが途中で 0 を読み始めても、後輪が吹け上がらない。"""
        sc = S.Scenario("t", "t", (S.Seg(0.3, MODE_SPEED, 1.5, 3.0),
                                   S.Seg(0.7, MODE_SPEED, 1.5, 3.0, front_scale=0.0)), 1.5)
        r = S.run(self.fw, NOMINAL, sc)
        self.assertLess(r.metrics["wheel_peak"], 3.0)


@unittest.skipUnless(HAVE_FW, "ファームがありません")
class TestIdentification(unittest.TestCase):
    """真値の分かっている車で3つの試験を回し、解析が真値を復元する。"""

    TRUTH = replace(NOMINAL, wheel_inertia_kgm2=2.6e-5, wheel_friction_nm=0.0035, md_tau_s=0.006,
                    mu=0.52, load_transfer=0.22, tyre_b=14.0, tyre_c=1.45,
                    yaw_inertia_kgm2=0.03, yaw_damping=2.4)
    #: 相対の許容誤差（`None` は ABS の絶対誤差）
    REL = {"wheel_inertia_kgm2": 0.1, "wheel_friction_nm": None, "md_tau_s": 0.3, "mu": 0.08,
           "load_transfer": None, "tyre_b": 0.3, "tyre_c": None, "yaw_inertia_kgm2": 0.3,
           "yaw_damping": 0.1}
    ABS = {"wheel_friction_nm": 0.001, "load_transfer": 0.06, "tyre_c": 0.15}

    @classmethod
    def setUpClass(cls):
        fw = firmware()
        cls.recs = {"wheel": record.simulate(fw, cls.TRUTH, SysIdWheel(), lifted=True),
                    "tyre": record.simulate(fw, cls.TRUTH, SysIdTyre()),
                    "yaw": record.simulate(fw, cls.TRUTH, SysIdYawMoment())}
        cls.analysis = fit.analyze(cls.recs, NOMINAL)

    def test_planners_finish_within_the_straight(self):
        for key, rec in self.recs.items():
            self.assertEqual(rec.notes, [], key)
            self.assertLess(rec.odom.max(), 2.15, key)
            self.assertGreater(rec.odom.min(), -0.15, key)

    def test_recovers_truth(self):
        a = self.analysis
        self.assertEqual(a.errors, [])
        self.assertEqual(a.warned, {})
        for key, rel in self.REL.items():
            truth, got = getattr(self.TRUTH, key), a.results[key]
            tol = self.ABS[key] if rel is None else rel * abs(truth)
            self.assertLess(abs(got - truth), tol, f"{key}: {got:.4g}（真値 {truth:.4g}）")

    def test_warns_when_tc_was_left_on(self):
        """TC を切れていない記録（後輪が滑らない）は★になり、値は既定で適用されない。"""
        rec = self.recs["tyre"]
        capped = replace(rec, wheel_left=np.minimum(rec.wheel_left, rec.speed * 1.1),
                         wheel_right=np.minimum(rec.wheel_right, rec.speed * 1.1))
        r = fit.fit_tyre(capped, self.TRUTH)
        self.assertTrue(r.warnings)

    def test_wheel_test_warns_when_wheels_touch_the_ground(self):
        rec = record.simulate(firmware(), self.TRUTH, SysIdWheel(), lifted=False)
        r = fit.fit_wheel(rec, NOMINAL)
        self.assertTrue(r.warnings)


@unittest.skipUnless(HAVE_FW, "ファームがありません")
class TestOptimise(unittest.TestCase):
    def test_never_returns_worse_than_start(self):
        fw = firmware()
        start = {k: v.default for k, v in fw.params.items()}
        t = optimize.optimise(fw, NOMINAL, optimize.abs_problem(), start, max_iter=1, pop=2, workers=1)
        self.assertLessEqual(t.cost, t.baseline_cost + 1e-12)
        self.assertEqual(set(t.values), set(optimize.abs_problem().bounds))
        for key, (lo, hi) in optimize.abs_problem().bounds.items():
            self.assertTrue(lo - 1e-9 <= t.values[key] <= hi + 1e-9, key)

    def test_bounds_are_inside_firmware_range(self):
        table = firmware().params
        for make in optimize.PROBLEMS.values():
            for key, (lo, hi) in make().bounds.items():
                self.assertLessEqual(table[key].lo, lo, key)
                self.assertLessEqual(hi, table[key].hi, key)


@unittest.skipUnless(HAVE_FW, "ファームがありません")
class TestTomlRoundTrip(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.dir.cleanup)
        self.path = Path(self.dir.name) / "vehicle.toml"
        shutil.copy(DEFAULT_TOML, self.path)

    def test_apply_control_keeps_everything_else(self):
        before = self.path.read_text(encoding="utf-8")
        changed = tuning.apply_control(self.path, {"tc_slip_target": 0.123}, firmware())
        self.assertEqual(changed, ["tc_slip_target"])
        after = self.path.read_text(encoding="utf-8")
        self.assertEqual(len(before.splitlines()), len(after.splitlines()))
        self.assertEqual(sum(a != b for a, b in zip(before.splitlines(), after.splitlines())), 1)
        self.assertAlmostEqual(load_control_params(self.path)["tc_slip_target"], 0.123)

    def test_apply_control_rejects_out_of_range_and_derived(self):
        with self.assertRaises(ValueError):
            tuning.apply_control(self.path, {"tc_slip_target": 5.0}, firmware())
        with self.assertRaises(ValueError):
            tuning.apply_control(self.path, {"tv_steer_gain": 1.0}, firmware())

    def test_apply_plant_is_read_back(self):
        tuning.apply_plant(self.path, {"wheel_inertia_kgm2": 3.3e-5, "mu": 0.55})
        p = Plant.load(self.path)
        self.assertAlmostEqual(p.wheel_inertia_kgm2, 3.3e-5)
        self.assertAlmostEqual(p.mu, 0.55)
        with open(self.path, "rb") as f:
            self.assertIn("plant", tomllib.load(f)["control"])
        self.assertEqual(load_control_params(self.path), load_control_params(DEFAULT_TOML))


if __name__ == "__main__":
    unittest.main()
