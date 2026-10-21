"""ファームの制御の約束（`src/control` をホストでコンパイルして車のモデルで回す）。

2026-10-18 までは `tools/ctrl_tune/tests/test_ctrl_tune.py` の一部だった（同定・最適化の部分は `tools/calib` に作り直した）。
ファームのリポジトリ（`fw.FW_DIR`）と C コンパイラが要る。無ければ全部飛ばす。
"""

import shutil
import sys
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from raspi.core.control_params import CONTROL_PARAM_IDS, ENABLE_KEYS, load_control_params  # noqa: E402
from tools.fwsim import scenarios as S  # noqa: E402
from tools.fwsim.fw import (FLAG_LAUNCH, FW_DIR, MODE_BRAKE, MODE_SPEED, MODE_SPEED_LAUNCH,  # noqa: E402
                            MODE_TORQUE, OUT, firmware)
from tools.fwsim.plant import DEFAULT_TOML, NOMINAL, Plant  # noqa: E402

HAVE_FW = (FW_DIR / "host" / "host_sim.c").exists() and shutil.which("cc") is not None
#: ファームの同定用ヨーモーメント注入（`tv_test_moment_nm`）の確認に使う値 [N·m]
MOMENT_NM = 0.12
QUIET = replace(NOMINAL, front_noise_rad_s=0.0, rear_noise_rad_s=0.0, gyro_noise_rad_s=0.0)
#: 仕組み（FF が遅れを消す・ローンチが PI より速い）を見る試験の車速 PI のゲイン。vehicle.toml の値（校正 5）は
#: 車ごとに変わり、強いゲインでは FF なし・ローンチなしでも遅れが小さく、比べたい差が見えなくなる（2026-10-19、
#: kp 1.14・ki 4.26 で 3 件が落ちた）。値は 2026-10-18 までの toml の値
MECH_SPEED_PI = {"speed_kp": 0.3, "speed_ki": 0.6}


def scenario(key: str) -> S.Scenario:
    return {s.key: s for s in S.ALL}[key]

@unittest.skipUnless(HAVE_FW, "ファームがありません")
class TestParamTable(unittest.TestCase):
    def test_ids_match_protocol(self):
        """`control_params.h` の表と `protocol.toml` の param_id が同じ（名前も）。"""
        table = firmware().params
        self.assertEqual({k: v.id for k, v in table.items()}, CONTROL_PARAM_IDS)

    def test_vehicle_toml_is_within_firmware_range(self):
        table = firmware().params
        for key, value in load_control_params(DEFAULT_TOML).items():
            if key in ENABLE_KEYS:      # 機能の ON/OFF はファームの調整表に無い
                continue
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

    #: 目標を大きく超えて滑っていた時間の割合（`locking`）の上限。2026-10-19 に固定の 0.3 で見る `slipping` から
    #: 置き換えた（校正 7 の目標が真の値で約 0.3 になり、目標どおりでも数えていた）。旧ゲイン・旧目標で 0〜0.06、
    #: 2026-10-19 の校正 7 のゲインで 0.04〜0.08（標準モデルの減速だけ旧 0.000 → 0.055 と悪い）
    LOCKING_MAX = 0.1

    def test_brake_holds_near_the_limit(self):
        """制動で限界の近くに保つ: 止まる・距離が最良の9割以上・目標を大きく超えて滑らない。
        ゲインを決めた**同定した実機のタイヤ**（`Plant.load()`）と、形の違う標準モデル（`NOMINAL`）の両方で見る。"""
        sc = scenario("brake")
        for name, plant in (("標準モデル", NOMINAL), ("同定したタイヤ", Plant.load())):
            with self.subTest(plant=name):
                m = S.run(self.fw, plant, sc).metrics
                self.assertTrue(m["stopped"])
                self.assertGreater(S.oracle(self.fw, plant, sc) / m["distance"], 0.9)
                self.assertLess(m["locking"], self.LOCKING_MAX)

    def test_speed_command_decel_does_not_lock(self):
        """車速指令の減速（ランプ3.0m/s²は後輪だけの制動の限界を超える）でも後輪がロックしない。"""
        for name, plant in (("標準モデル", NOMINAL), ("同定したタイヤ", Plant.load())):
            with self.subTest(plant=name):
                m = S.run(self.fw, plant, scenario("decel_pi")).metrics
                self.assertLess(m["locking"], self.LOCKING_MAX)

    # ── 車速PIの加速度フィードフォワード（v0.21。`drive.h` の同名の節）──

    def _plant_gain(self) -> tuple[float, float]:
        """車両モデルの `(車体の利得 [(m/s²)/N·m、後輪合計], 転がり抵抗 [m/s²])`（滑らない弱いトルク2段から）。"""
        acc = []
        for nm in (0.02, 0.04):
            r = S.run(self.fw, QUIET, S.Scenario("t", "t", (S.Seg(0.6, MODE_TORQUE, nm),), 0.5), {})
            t, v = r.col("t"), r.col("speed")
            m = (t > 0.2) & (t < 0.55)
            acc.append(float(np.polyfit(t[m], v[m], 1)[0]))
        gain = (acc[1] - acc[0]) / (2.0 * (0.04 - 0.02))
        return gain, gain * 2.0 * 0.02 - acc[0]

    def test_speed_feedforward_removes_the_ramp_lag(self):
        """目標車速を一定の傾きで上げる間、車速はほぼ遅れない（無しだと偏差が溜まるまで遅れる）。
        足す値が 0 なら、足す前のファームと同じ動き。"""
        gain, rr = self._plant_gain()
        self.assertGreater(gain, 5.0)
        sc = S.Scenario("t", "t", (S.Seg(1.0, MODE_SPEED, 2.4, 2.0),), 0.0)   # 停止から 2.0m/s² で
        off = S.run(self.fw, QUIET, sc, {**MECH_SPEED_PI, "speed_ff_nm_per_m_s2": 0.0, "speed_ff_rr_nm": 0.0})
        on = S.run(self.fw, QUIET, sc, {**MECH_SPEED_PI, "speed_ff_nm_per_m_s2": 1.0 / gain,
                                        "speed_ff_rr_nm": max(rr, 0.0) / gain})

        def lag(r) -> float:
            t, v = r.col("t"), r.col("speed")
            m = (t > 0.5) & (t < 0.9)
            return float(np.mean(2.0 * t[m] - v[m]))
        # 残る遅れは、目標車速の傾きに掛けるローパス（20ms）ぶん: 0.02s × 2.0m/s² = 0.04m/s
        self.assertGreater(lag(off), 0.08)
        self.assertLess(abs(lag(on)), 0.06)
        self.assertLess(abs(lag(on)), 0.5 * lag(off))
        self.assertLess(float(on.col("speed").max()), 2.4 + 0.08)        # 行き過ぎない

    def test_speed_feedforward_is_off_while_braking(self):
        """制動に入ったら傾きの記憶を捨てる（解いた直後に古い傾きのトルクを出さない）。"""
        gain, rr = self._plant_gain()
        params = {"speed_ff_nm_per_m_s2": 1.0 / gain, "speed_ff_rr_nm": max(rr, 0.0) / gain}
        r = S.run(self.fw, QUIET, S.Scenario("t", "t", (S.Seg(0.5, MODE_SPEED, 2.0, 2.0),
                                                        S.Seg(0.6, MODE_BRAKE, 0.08)), 1.0), params)
        self.assertLess(float(r.col("speed")[-1]), 0.6)
        self.assertTrue(np.all(np.isfinite(r.col("speed"))))

    # ── ローンチコントロール（v0.20。`drive.h` の同名の節）──

    def _launching(self, r) -> np.ndarray:
        return (r.col("flags").astype(int) & FLAG_LAUNCH) != 0

    def test_launch_accelerates_like_full_torque_and_hands_over_at_target(self):
        """車速指令＋ローンチは、最初の周期から全開を要求してトルク指令の全開と同じだけ進み、
        目標に届いたら車速PIへ戻って行き過ぎない。"""
        params = {**MECH_SPEED_PI, "launch_exit_torque_nm": 0.012, "speed_ff_nm_per_m_s2": 0.0,
                  "speed_ff_rr_nm": 0.0}   # FF なしの車速PIと比べる
        lc = S.run(self.fw, QUIET, S.Scenario("t", "t", (S.Seg(1.5, MODE_SPEED_LAUNCH, 1.5, 3.0),), 0.0),
                   params)
        pi = S.run(self.fw, QUIET, S.Scenario("t", "t", (S.Seg(1.5, MODE_SPEED, 1.5, 3.0),), 0.0), params)
        full = S.run(self.fw, QUIET, S.Scenario("t", "t", (S.Seg(0.4, MODE_TORQUE, S.MAX_TORQUE_NM),), 0.0), params)
        on = self._launching(lc)
        self.assertTrue(on[0])
        self.assertAlmostEqual(float(lc.col("cmd_left")[0]), S.MAX_TORQUE_NM, places=4)
        end = int(np.nonzero(on)[0][-1])
        self.assertFalse(on[end + 1:].any())                           # 1回で終わる
        self.assertAlmostEqual(float(lc.col("distance")[799]), float(full.col("distance")[799]), delta=0.005)
        t95 = lambda r: float(r.col("t")[np.nonzero(r.col("speed") >= 0.95 * 1.5)[0][0]])   # noqa: E731
        self.assertLess(t95(lc), t95(pi) - 0.1)
        after = lc.col("speed")[end:]
        self.assertLess(float(after.max()), 1.5 + 0.08)
        self.assertGreater(float(after.min()), 1.5 - 0.08)
        self.assertGreater(float(pi.col("speed").max()), float(after.max()))   # PI の方が行き過ぎる

    def test_launch_needs_a_large_step_but_not_a_standstill(self):
        """徐行の指令（目標との差が 0.5m/s 未満）では始めない。低速で走っている途中からは始める
        （上位は経路に乗るまで低速で走り、乗ってから要求する）。"""
        slow = S.run(self.fw, QUIET, S.Scenario("t", "t", (S.Seg(0.5, MODE_SPEED_LAUNCH, 0.4, 3.0),), 0.0))
        self.assertFalse(self._launching(slow).any())
        segs = (S.Seg(0.6, MODE_SPEED, 0.5, 3.0), S.Seg(0.8, MODE_SPEED_LAUNCH, 2.0, 3.0))
        # ローンチの仕組み（いつ始まり、効くか）を見る試験なので、TC は固定の設定で見る。vehicle.toml の TC
        # （目標スリップ率 0.10）は PI の発進だけでローンチ並みの距離が出てしまい、優位が見えなくなる
        tc = {"tc_slip_target": 0.2, "tc_kp_nm_per_m_s": 0.2011, "tc_ki_nm_per_m": 5.3387,
              **MECH_SPEED_PI, "speed_ff_nm_per_m_s2": 0.0, "speed_ff_rr_nm": 0.0}
        lc = S.run(self.fw, QUIET, S.Scenario("t", "t", segs, 0.5, measure=1), tc)
        pi = S.run(self.fw, QUIET, S.Scenario(
            "t", "t", (segs[0], S.Seg(0.8, MODE_SPEED, 2.0, 3.0)), 0.5, measure=1), tc)
        on = self._launching(lc)
        self.assertFalse(on[:1200].any())
        self.assertTrue(on[1200])
        self.assertGreater(lc.metrics["distance"], pi.metrics["distance"] + 0.05)
        near = S.run(self.fw, QUIET, S.Scenario("t", "t", (S.Seg(0.5, MODE_SPEED_LAUNCH, 1.9, 3.0),), 1.5))
        self.assertFalse(self._launching(near).any())                  # 差が小さい加速は PI のまま

    def test_launch_is_rearmed_by_braking_to_a_stop(self):
        """1回の要求で1回。制動して止まれば、次の発進でまた働く。"""
        segs = (S.Seg(0.6, MODE_SPEED_LAUNCH, 1.0, 3.0), S.Seg(1.2, MODE_BRAKE, 0.15),
                S.Seg(0.6, MODE_SPEED_LAUNCH, 1.0, 3.0))
        r = S.run(self.fw, QUIET, S.Scenario("t", "t", segs, 0.0))
        on = self._launching(r)
        self.assertTrue(on[:1200].any())
        self.assertFalse(on[1200:3600].any())                          # 制動中は働かない
        self.assertTrue(on[3600:].any())

    def test_launch_needs_traction_control(self):
        """空転を抑えるのは TC だけなので、TC が無効なら始めない。"""
        sc = S.Scenario("t", "t", (S.Seg(0.5, MODE_SPEED_LAUNCH, 3.0, 3.0),), 0.0, tc=False)
        self.assertFalse(self._launching(S.run(self.fw, QUIET, sc)).any())

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
        expect = MOMENT_NM * 2.0 * 0.030 / 0.155       # 車輪半径・後輪トレッド（drive.h）
        self.assertAlmostEqual(float(np.median(diff[700:950])), expect, places=4)
        self.assertGreater(float(np.mean(r.col("yaw_rate")[800:1000])), 0.02)     # 左へ回る

    def test_tv_splits_torque_by_lateral_load(self):
        """TV（2026-10-08 に荷重比例の配分へ置き換えた）: 旋回しながらの駆動で外輪が多くなり、
        比率は ゲイン×車速×ヨーレート、内外輪の滑り率の差が TV なしより小さい。"""
        gain = self.fw.base_params()["tv_load_gain_s2_per_m"]     # vehicle.toml の値
        segs = (S.Seg(0.5, MODE_SPEED, 1.5, 3.0, steer=0.2), S.Seg(0.5, MODE_TORQUE, 0.04, steer=0.2))
        on = S.run(self.fw, QUIET, S.Scenario("t", "t", segs, 1.5, tc=False, lift=False, measure=1))
        off = S.run(self.fw, QUIET, S.Scenario("t", "t", segs, 1.5, tc=False, lift=False, tv=False, measure=1))
        w = slice(-400, None)                              # 最後の 0.2s（左旋回 = 右が外輪）
        left, right = on.col("cmd_left")[w], on.col("cmd_right")[w]
        ratio = (right - left) / (right + left)
        expect = gain * on.col("fw_speed")[w] * on.col("fw_yaw_rate")[w]
        self.assertGreater(float(np.mean(ratio)), 0.05)
        np.testing.assert_allclose(ratio, expect, atol=2e-3)
        np.testing.assert_allclose(right + left, 0.08, atol=1e-6)        # 総和は変えない
        np.testing.assert_allclose(off.col("cmd_left")[w], off.col("cmd_right")[w], atol=1e-9)
        gap = lambda r: abs(float(np.mean(r.col("kappa_left")[w] - r.col("kappa_right")[w])))
        self.assertLess(gap(on), gap(off))

    def test_tv_brakes_outer_wheel_more_when_decelerating(self):
        """車速指令の減速（総トルクが負）では外輪が多く制動する = 旋回を戻す向き。"""
        segs = (S.Seg(0.3, MODE_SPEED, 1.5, 3.0, steer=0.25), S.Seg(0.3, MODE_SPEED, 0.0, 2.0, steer=0.25))
        r = S.run(self.fw, QUIET, S.Scenario("t", "t", segs, 1.5, tc=False, lift=False, measure=1))
        w = slice(-400, -100)
        self.assertLess(float(np.mean(r.col("cmd_left")[w])), 0.0)
        self.assertLess(float(np.mean(r.col("cmd_right")[w] - r.col("cmd_left")[w])), -1e-3)

    def test_tv_does_nothing_when_straight_or_gain_zero(self):
        straight = S.run(self.fw, QUIET, S.Scenario("t", "t", (S.Seg(0.5, MODE_TORQUE, 0.05),), 1.0, tc=False))
        self.assertLess(float(np.max(np.abs(straight.col("cmd_right") - straight.col("cmd_left")))), 1e-4)
        turn = S.Scenario("t", "t", (S.Seg(0.5, MODE_TORQUE, 0.04, steer=0.2),), 1.5, tc=False, lift=False)
        zero = S.run(self.fw, QUIET, turn, {"tv_load_gain_s2_per_m": 0.0})
        np.testing.assert_allclose(zero.col("cmd_left"), zero.col("cmd_right"), atol=1e-9)

    def test_dead_front_encoder_does_not_runaway(self):
        """前輪エンコーダが途中で 0 を読み始めても、後輪が吹け上がらない。"""
        sc = S.Scenario("t", "t", (S.Seg(0.3, MODE_SPEED, 1.5, 3.0),
                                   S.Seg(0.7, MODE_SPEED, 1.5, 3.0, front_scale=0.0)), 1.5)
        r = S.run(self.fw, NOMINAL, sc)
        self.assertLess(r.metrics["wheel_peak"], 3.0)


@unittest.skipUnless(HAVE_FW, "ファームがありません")

@unittest.skipUnless(HAVE_FW, f"ファームのリポジトリ（{FW_DIR}）か cc がありません")
class TestBuildCache(unittest.TestCase):
    def test_lib_header_change_rebuilds(self):
        """`lib/`（ヘッダだけの PID・フィルタ）を直したら別のライブラリになる（古いキャッシュを使わない）。"""
        from tools.fwsim import fw as fwmod
        with tempfile.TemporaryDirectory() as d:
            tree = Path(d)
            for sub in ("src/control", *fwmod._INCLUDES[:3]):
                shutil.copytree(FW_DIR / sub, tree / sub)
            first = fwmod._build(tree)
            pid = next((tree / "lib" / "pid").glob("*.h"))
            pid.write_text(pid.read_text(encoding="utf-8") + "\n// changed\n", encoding="utf-8")
            second = fwmod._build(tree)
            self.assertNotEqual(first, second)
            self.assertTrue(second.exists())


if __name__ == "__main__":
    unittest.main()
