"""`tools/sysid/fit.py` のテスト。

本体は `tools/sysid/bench.py`：真値の分かっているシム（`sim/vehicle.py`）で
`raspi/auto/sysid_*.py` を実機と同じ経路（10Hzのスキャン・50Hzの中継・100Hzの
COMMAND・センサのノイズとバイアス）で閉ループに回し、その記録から `fit.py` が
真値を復元できるかを見る。ここでは加えて、解析の部品が `sim/vehicle.py` と同じ式に
なっていること（出力誤差法の前提）と、測れていない記録を黙って通さないことを見る。
"""

import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))  # surge_mk2/

import numpy as np  # noqa: E402

from raspi.auto._sysid_common import lateral_saturated  # noqa: E402
from sim.vehicle import SteerServo  # noqa: E402
from tools.sysid import bench, fit  # noqa: E402
from tools.sysid.bench import TRUTH, run_all, run_test  # noqa: E402


class TestBenchRecovery(unittest.TestCase):
    """閉ループで回した記録から、全パラメータが許容誤差内で復元できる。"""

    def _check(self, truth, seed):
        r = run_all(truth, seed=seed, verbose=False)
        bad = {k: v for k, v in r["values"].items() if not bench._within(k, *v)}
        self.assertEqual(bad, {}, f"許容誤差外（真値, 推定）: {bad}")
        return r

    def test_default_truth(self):
        r = self._check(TRUTH, seed=0)
        # 必要なスペース（旧手順は旋回が約2.0m四方・加減速が直線約3.2m。同じシムで計測）:
        # 直線の試験は run_length_m（2.0m）＋車体の長さ（0.37m）× 横0.7m、旋回は約1.9m四方
        car = 0.37
        sp = r["spaces"]["accel"]
        self.assertLess(sp.width, 2.0 + car + 0.05, f"accel: {sp}")
        self.assertLess(sp.height, 0.7, f"accel: {sp}")
        # ステアは±30°の段とステップで車の向きが回り、車体の前端が開始位置より後ろへ振れる分 +0.3m。
        # 階段部は後退の途中で舵を次の段へ切るので、上りの途中で車の向きが最大約25°回る（下りで
        # 戻る）。中立付近の段は0.6m/sで走る。横幅はシードによって0.6〜1.05m（2026-09-27、
        # ベンチの3シード×3つの記録の条件で最大1.04m）
        sp = r["spaces"]["steer"]
        self.assertLess(sp.width, 2.0 + car + 0.3, f"steer: {sp}")
        self.assertLess(sp.height, 1.1, f"steer: {sp}")
        # 旋回は限界の直後で止まるので、円の大きさはほぼ最小旋回半径で決まる（リンクのガタ・不感帯・
        # サーボの定常偏差で大舵角の効きが落ちるほど大きい）。速度を連続して上げる形（2026-09-27）で
        # 既定の真値は 1.74×1.92m（段で上げていた頃は限界の検出が1段遅れ、約2.0〜2.13m）
        self.assertLess(max(r["spaces"]["corner"].width, r["spaces"]["corner"].height), 2.0)
        self.assertLess(max(r["spaces"]["latency"].width, r["spaces"]["latency"].height), 0.5)
        # モデルどおりの真値では「モデルの形から外れている」（★）の警告を出さない
        for test in ("steer", "accel", "geometry"):
            self.assertFalse([n for n in r["notes"][test] if n.startswith("★")], test)

    def test_unobservable_rate_cap_no_fade_no_understeer(self):
        """物理上限が COMMAND のランプより速い（観測できない）なら 0、減衰なしなら 0 を返す。"""
        truth = replace(TRUTH, steer_rate_limit_rad_s=0.0, drive_fade_speed_m_s=0.0,
                        drive_top_speed_m_s=0.0, understeer_gradient=0.0,
                        steer_offset_rad=-0.015, mu=0.6)
        self._check(truth, seed=5)

    def test_formerly_off_model_truths(self):
        """第三者検証（2026-09-26）でモデルの外にあり、黙って誤った値を返した真値: 振動的な
        ヨー応答（舵の効きが13%ずれた）・サーボの定常偏差（存在しないヒステリシスを採った）・
        ファームのランプより高い＝観測できない加減速の上限。今は挙動が一致し、上限は 0 を返す。"""
        truth = replace(TRUTH, yaw_natural_freq_rad_s=20.0, yaw_damping=0.4, yaw_rate_filter_s=0.0,
                        yaw_relaxation_m=0.0, steer_servo_gain=0.95, steer_servo_hysteresis_rad=0.0,
                        drive_accel_m_s2=0.0, drive_fade_speed_m_s=0.0, drive_top_speed_m_s=0.0,
                        speed_decel_m_s2=0.0)
        r = self._check(truth, seed=7)
        # 上限が無い真値に、ランプ（3.0m/s²）より低い上限を読まない（0＝観測できず、または
        # ランプより上＝PIの追い上げの瞬間だけ効く値）
        for k in ("drive_accel_m_s2", "speed_decel_m_s2"):
            got = r["values"][k][1]
            self.assertTrue(got == 0.0 or got > truth.speed_ramp_max_m_s2, f"{k}: {got}")


class TestStatistics(unittest.TestCase):
    def test_n_eff_follows_the_residual_correlation(self):
        rng = np.random.default_rng(0)
        w = rng.normal(size=5000)
        self.assertAlmostEqual(fit.n_eff(w) / 5000, 1.0, delta=0.15)
        ma = np.convolve(w, np.ones(5) / 5, mode="valid")        # 以前の決め打ち（5サンプルで1自由度）
        self.assertAlmostEqual(fit.n_eff(ma) / len(ma), 0.2, delta=0.05)
        # ゆっくりした誤差（エンコーダの周期誤差など）が乗ると自由度は大きく減る
        slow = np.sin(np.arange(5000) * 2 * np.pi / 40)
        self.assertLess(fit.n_eff(slow + 0.1 * w), 5000 / 10)


def _ripple_samples(amp: float, seed: int = 0) -> list:
    """0.5m/s・0.8m/s の定速区間を持つ記録。前輪の角度の読みに1回転1周期のうねり `amp` [rad]。"""
    rng = np.random.default_rng(seed)
    r = 0.03
    out, t, d = [], 0.0, 0.0
    for v, dur in ((0.0, 1.0), (0.5, 2.0), (0.8, 2.0), (0.5, 2.0), (0.0, 1.0)):
        for _ in range(int(dur / 0.02)):
            t += 0.02
            d += v * 0.02
            th = d / r
            meas = (th + amp * np.sin(th + 0.4) + rng.normal(0, 0.003)) * r
            meas = round(meas / 1e-4) * 1e-4
            out.append(fit.Sample(t=t, target_speed=v, target_steer=0.0, brake=False, speed=v,
                                  steer_actual=0.0, steer_cmd_echo=0.0, yaw_rate=0.0, accel_x=0.0,
                                  tc_active=False, odom=meas, speed_filtered=v, odom_l=meas, odom_r=meas))
    return out, r


class TestEncoderRipple(unittest.TestCase):
    """前輪のアナログ絶対角エンコーダの周期誤差（うねり）の推定と補正（2026-09-26）。"""

    def test_ripple_is_estimated_and_removed(self):
        s, r = _ripple_samples(math.radians(4.0))
        truth = np.cumsum([0.0] + [x.target_speed * 0.02 for x in s[1:]])
        before = np.std([x.odom for x in s] - truth - np.mean([x.odom for x in s] - truth))
        notes = fit.correct_encoder_ripple(s)
        self.assertTrue(any("補正した" in n for n in notes), notes)
        after = np.std([x.odom for x in s] - truth - np.mean([x.odom for x in s] - truth))
        self.assertLess(after, 0.2 * before)

    def test_no_ripple_is_left_alone(self):
        s, _ = _ripple_samples(0.0)
        odom = [x.odom for x in s]
        notes = fit.correct_encoder_ripple(s)
        self.assertFalse(any("補正した" in n for n in notes), notes)
        self.assertEqual(odom, [x.odom for x in s])


class TestResidualWarnings(unittest.TestCase):
    def test_speed_controller_mismatch_is_flagged(self):
        """ファームの PI の定数が vehicle.toml と違う（＝シムの速度モデルの形が違う）と★で知らせる。"""
        truth = replace(TRUTH, speed_kp=0.08, speed_ki=1.5)
        log, _, _ = run_test("sysid_accel", truth, seed=2, noise=bench.REALISTIC)
        r = fit.fit_accel(log, TRUTH)
        self.assertTrue([n for n in r.notes if n.startswith("★")], r.notes)
        self.assertTrue(r.warnings, r.notes)

    def test_warned_accel_is_not_applied_by_default(self):
        """前後運動の記録がモデルから外れて★が出たら、その試験で求めた項目（車体の利得・
        転がり抵抗も）を `warned` に載せる（GUI が既定で適用しない）。★が無ければ載せない。"""
        from tools.sysid.analyze import analyze
        base = replace(TRUTH, steer_gain=1.0, steer_offset_rad=0.0)
        off = replace(TRUTH, speed_kp=0.08, speed_ki=1.5)      # 記録の車だけ別の制御器
        off_log, _, _ = run_test("sysid_accel", off, seed=1)
        a = analyze({"accel": off_log}, base)
        for k in ("speed_plant_gain", "rolling_resistance", "drive_accel_m_s2", "brake_decel_m_s2"):
            self.assertIn(k, a.warned)
        ok_log, _, _ = run_test("sysid_accel", TRUTH, seed=1)
        a = analyze({"accel": ok_log}, base)
        self.assertEqual(a.warned, {})
        self.assertAlmostEqual(a.results["speed_plant_gain"], TRUTH.speed_plant_gain,
                               delta=bench.REL_TOL["speed_plant_gain"] * TRUTH.speed_plant_gain)

    def test_warned_accel_does_not_overwrite_the_speed_log_gain(self):
        """2026-09-26 までの2つの記録（速度応答＋加減速）を合わせる使い方: 加減速に★が出たら、
        合わせて当てはめ直した利得・転がり抵抗を返さない（外れた分を吸収して 21.4→17.8 と
        壊れた、第三者検証）。ここでは速度応答の記録の代わりに、モデルどおりの車の前後運動の記録を渡す。"""
        base = replace(TRUTH, steer_gain=1.0, steer_offset_rad=0.0)
        good, _, _ = run_test("sysid_accel", TRUTH, seed=1)
        off = replace(TRUTH, speed_kp=0.08, speed_ki=1.5)
        bad, _, _ = run_test("sysid_accel", off, seed=1)
        r = fit.fit_accel(bad, base, good)
        self.assertTrue(r.warnings, r.notes)
        self.assertNotIn("speed_plant_gain", r.values)
        self.assertNotIn("rolling_resistance", r.values)


class TestBrakeFit(unittest.TestCase):
    def test_strength_slope_and_lock_from_the_accel_log(self):
        """前後運動試験の4段の制動から、傾き（強さ→減速度）と後輪のロックの頭打ちを読む
        （制動中の走行距離に、MD の境界層込みのシムの制動を当てはめる）。"""
        base = replace(TRUTH, steer_gain=1.0, steer_offset_rad=0.0)
        log, _, _ = run_test("sysid_accel", TRUTH, seed=2)
        v, notes = fit._fit_brake_runs(log, base, TRUTH.rolling_resistance)
        # 最大の強さはグリップで頭打ち＝実機では ABS が削る（★v0.15、ベンチも旗を立てる）
        self.assertTrue(any("ABS が制動を削った" in n for n in notes), notes)
        self.assertAlmostEqual(v["brake_decel_per_nm"], TRUTH.brake_decel_per_nm, delta=0.05 * TRUTH.brake_decel_per_nm)
        self.assertAlmostEqual(v["brake_decel_m_s2"], TRUTH.brake_decel_m_s2, delta=0.15)

    def test_abs_spikes_are_not_a_lock(self):
        """ABS（★v0.15）の一瞬の滑り（10〜20ms だけ後輪が前輪の6割未満）はロックではない。
        30ms 以上続けばロック。"""
        def run(n_low):
            return [fit.Sample(t=k * 0.01, target_speed=0.0, target_steer=0.0, brake=True, speed=1.5,
                               steer_actual=0.0, steer_cmd_echo=0.0, yaw_rate=0.0, accel_x=0.0,
                               tc_active=False, wheel_speed_rear=0.5 if 5 <= k < 5 + n_low else 1.45)
                    for k in range(20)]
        self.assertFalse(fit._rear_locked(run(2)))
        self.assertTrue(fit._rear_locked(run(5)))

    def test_single_strength_gives_only_the_limit(self):
        """強さが1通り（2026-09-26 までの記録）なら上限だけ（傾きは決まらない）。"""
        v, _ = fit._fit_brake({0.15: 3.9}, 0.3, TRUTH)
        self.assertEqual(v, {"brake_decel_m_s2": 3.9})

    def test_sim_brake_model_matches(self):
        """シム（`next_speed` の制動）: 傾き×強さ×tanh(車速÷境界層)＋転がり抵抗、頭打ち。境界層と
        不感帯は MD のファームの定数（`vehicle.toml`）。"""
        from sim.vehicle import DriveInput, next_speed
        sp = replace(TRUTH, brake_decel_per_nm=40.0, brake_decel_m_s2=3.9, rolling_resistance=0.3,
                     brake_boundary_speed_m_s=0.6, brake_deadband_speed_m_s=0.06)
        for v0, nm, want in ((2.0, 0.04, 40 * 0.04 * math.tanh(2.0 / 0.6) + 0.3), (2.0, 0.15, 3.9),
                             (0.3, 0.04, 40 * 0.04 * math.tanh(0.3 / 0.6) + 0.3), (0.05, 0.04, 0.3)):
            v1 = next_speed(sp, DriveInput(armed=True, brake=True, brake_torque=nm), v0, 0.001)
            self.assertAlmostEqual((v0 - v1) / 0.001, want, places=4, msg=(v0, nm))


class TestSpinDetection(unittest.TestCase):
    def test_rear_faster_than_front_while_accelerating_is_spin(self):
        """加速中に後輪が前輪より大きく速ければ空転（その段の加速度を返す）。制動中・小さな差は違う。"""
        def samples(rear_extra, brake=False):
            return [fit.Sample(t=k * 0.01, target_speed=1.5, target_steer=0.0, brake=brake,
                               speed=0.8, steer_actual=0.0, steer_cmd_echo=0.0, yaw_rate=0.0,
                               accel_x=0.0, tc_active=False, accel_limit=2.4,
                               wheel_speed_rear=0.8 + rear_extra) for k in range(20)]
        self.assertEqual(fit._spin_accel(samples(0.6)), 2.4)
        self.assertIsNone(fit._spin_accel(samples(0.3)))
        self.assertIsNone(fit._spin_accel(samples(0.6, brake=True)))
        # 加速の段の頭（まだ後退中）で後輪の値が無い記録（後輪0）は空転ではない
        back = samples(0.0)
        for x in back:
            x.speed, x.wheel_speed_rear = -0.6, 0.0
        self.assertIsNone(fit._spin_accel(back))


class TestSlipLimitedAccel(unittest.TestCase):
    @staticmethod
    def _stage(t0, level, accel, slip=0.15, n=40):
        """`level` の段を `accel` で加速しながら後輪が `slip` だけ滑っている区間（100Hz）。"""
        return [fit.Sample(t=t0 + k * 0.01, target_speed=2.0, target_steer=0.0, brake=False,
                           speed=0.5 + accel * k * 0.01, steer_actual=0.0, steer_cmd_echo=0.0,
                           yaw_rate=0.0, accel_x=0.0, tc_active=True, accel_limit=level,
                           speed_filtered=0.5 + accel * k * 0.01,
                           wheel_speed_rear=0.5 + accel * k * 0.01 + slip) for k in range(n)]

    def test_takes_the_stage_with_the_largest_accel(self):
        """TC は滑りを保つので、限界より下の段でも滑り率は0.10を超える。その段の加速度（指令どおり）
        ではなく、いちばん加速できた段を限界として返す（全開の段の `accel_limit` は 0 か GUI の上限）。"""
        def gap(t0):       # 段の間（滑っていない）
            return self._stage(t0, 0.8, 0.0, slip=0.0, n=5)
        s = self._stage(0.0, 1.6, 1.55) + gap(0.5) + self._stage(1.0, 2.4, 2.1) + gap(1.5) \
            + self._stage(2.0, 0.0, 2.2) + gap(2.5) + self._stage(3.0, 6.0, 2.2)
        a, lv = fit._slip_limited_accel(s)
        self.assertAlmostEqual(a, 2.2, delta=0.02)
        self.assertEqual(lv, 3.0)
        self.assertEqual(fit._slip_accel(s), 1.6)
        self.assertEqual(fit._top_accel_level(s), 3.0)

    def test_none_without_slip(self):
        self.assertIsNone(fit._slip_limited_accel(self._stage(0.0, 1.6, 1.55, slip=0.02)))


class TestNoFalseTopSpeed(unittest.TestCase):
    def test_uncapped_truth_gives_no_fade(self):
        """加減速の上限が無い車に、存在しない最高速（減衰）を読まない。実測の速度はオドメトリを
        ±45ms の窓で微分したもので、全開加速→制動の切り替えで区間の末尾だけ鈍る。シムに同じ
        微分を掛けずに比べていた頃は、上限なしの真値で約2.3m/sで0になる減衰を返した（2026-09-26）。"""
        base = replace(TRUTH, steer_gain=1.0, steer_offset_rad=0.0)
        for seed in (0, 3, 4):
            log, _, _ = run_test("sysid_accel", bench.TRUTH_NOCAP, seed=seed)
            v = fit.fit_accel(log, base).values
            self.assertEqual(v["drive_fade_speed_m_s"], 0.0, (seed, v))
            self.assertEqual(v["drive_top_speed_m_s"], 0.0, (seed, v))
            got = replace(bench.TRUTH_NOCAP, **{k: v[k] for k in bench._SPEED_KEYS})
            self.assertLess(bench._speed_top_err(bench.TRUTH_NOCAP, got), bench._SPEED_TOP_TOL, seed)


class TestModelConsistency(unittest.TestCase):
    """解析が `sim/vehicle.py` と同じ式を解いていること（出力誤差法の前提）。"""

    def _servo(self, ref_fn, dead, tau, cap, t_end=0.6, dt=0.001):
        sv = SteerServo(dead, tau, cap)
        ts, ys = [], []
        for k in range(int(t_end / dt)):
            ys.append(sv.step(ref_fn(k * dt), dt))
            ts.append((k + 1) * dt)
        return np.array(ts), np.array(ys)

    def test_fast_servo_response_matches_steer_servo(self):
        bp_t = np.array([0.0, 0.1, 0.175, 0.6])
        bp_v = np.array([0.0, 0.0, 0.5, 0.5])
        for cap in (0.0, 4.0):
            ts, ys = self._servo(lambda t: float(np.interp(t, bp_t, bp_v)), 0.02, 0.06, cap)
            got = fit._servo_response(bp_t, bp_v, ts, 0.0, 0.02, 0.06, cap, dt=0.001)
            self.assertLess(float(np.max(np.abs(got - ys))), 0.01, f"cap={cap}")

    def test_dead_time_is_not_quantised_toward_late_side(self):
        """むだ時間の実効値がサブステップ幅によらず指定値の±dt/2に収まる。

        入力を変えたステップを0として、新しい入力で積分し始めたステップ k の始まり
        `k*dt` が実効むだ時間（ZOHで積分される入力が切り替わる時刻）。中点で引くので
        四捨五入になる。以前の1step=1エントリ方式は `(ceil(d/dt)+1)*dt` 相当で、
        常に遅れ側へ最大2dtずれていた（dt=5msの学習環境で最大+10ms）。"""
        for dead in (0.004, 0.013, 0.027):
            for dt in (0.001, 0.005, 0.01):
                sv = SteerServo(dead, 0.0, 0.0)
                k = 0
                while sv.step(1.0, dt) == 0.0:
                    k += 1
                self.assertLessEqual(abs(k * dt - dead), dt / 2 + 1e-9, f"dead={dead} dt={dt}")


class TestEchoReconstruction(unittest.TestCase):
    def test_ramp_corners_are_recovered_between_samples(self):
        """50Hzのエコーから、ランプの始点をサンプル間隔より細かく復元する。"""
        rate, t_on, amp = 7.0, 0.1234, 0.5
        t = np.arange(0.0, 0.6, 0.02)
        echo = np.clip((t - t_on) * rate, 0.0, amp)
        bp_t, bp_v = fit._reconstruct_echo(t, np.round(echo, 4), rate)
        self.assertAlmostEqual(bp_t[1], t_on, delta=0.001)
        self.assertAlmostEqual(bp_t[2], t_on + amp / rate, delta=0.002)

    def test_instant_step_without_rate_is_centered(self):
        t = np.arange(0.0, 0.4, 0.02)
        echo = np.where(t >= 0.2, 0.3, 0.0)
        bp_t, _ = fit._reconstruct_echo(t, echo, 0.0)
        self.assertAlmostEqual(bp_t[1], 0.19, delta=1e-6)


class TestSaturation(unittest.TestCase):
    def test_understeer_alone_is_not_saturation(self):
        """アンダーステア勾配で曲率が落ちても、横加速度が伸びていれば限界ではない。"""
        spec = replace(TRUTH, understeer_gradient=0.05)
        d = 0.5
        a = [v * v * abs(spec.curvature(d, v)) for v in (1.0, 1.1)]
        self.assertFalse(lateral_saturated(1.0, a[0], 1.1, a[1]))

    def test_flat_lateral_accel_is_saturation(self):
        self.assertTrue(lateral_saturated(1.3, 4.4, 1.4, 4.45))


class TestMcapRoundTrip(unittest.TestCase):
    """実機と同じロガー（`McapLog`）で書いたmcapを `load_log` で読むと、同じ解析結果になる。

    ベンチは `Log` を直接組み立てるので、これが無いと `load_log`（トピック名・
    メッセージ内の時刻の読み方）が一度も試されない。
    """

    def test_latency_log_survives_mcap(self):
        import tempfile

        from raspi.msgs.types import DriveCmd, LinkDiag, Scan, VehicleState
        from raspi.rec.mcap_log import McapLog

        log, _, _ = run_test("sysid_latency", TRUTH, seed=2)
        t0 = 5_000_000_000
        ns = lambda t: t0 + int(round(t * 1e9))  # noqa: E731
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "x.mcap"
            with McapLog(path, t0_mono_ns=t0, t0_unix_ns=1_700_000_000_000_000_000) as w:
                for x in log.samples:
                    w.write("/vehicle_state", VehicleState(
                        t_capture=ns(x.t), speed=x.speed, yaw_rate=x.yaw_rate,
                        steer_actual=x.steer_actual, steer_cmd_echo=x.steer_cmd_echo,
                        odom_dist=[x.odom, x.odom]))
                for c in log.cmds:
                    w.write("/cmd", DriveCmd(
                        t_capture=ns(c.t), t_pub=ns(c.t), target_speed=c.target_speed,
                        target_steer=c.target_steer, brake=c.brake, accel_limit=c.accel_limit,
                        steer_rate_limit=c.steer_rate_limit, brake_torque=c.brake_torque))
                for sc in log.scans:
                    w.write("/scan", Scan(t_capture=ns(sc.t_done - 0.1), t_pub=ns(sc.t_pub),
                                          seq=sc.seq, sector_t_ns=[ns(sc.t_done)] * 12))
                # 実機の記録には io_node の診断（ファームの ID を含む）も入る
                w.write("/diag/link", LinkDiag(t_capture=ns(0.0), fw_id=0x4D463303))
            got = fit.load_log(path)
        self.assertEqual(len(got.samples), len(log.samples))
        self.assertEqual(len(got.scans), len(log.scans))
        self.assertEqual(got.fw_id, 0x4D463303)
        a = fit.fit_latency(log).values["control_latency_s"]
        r = fit.fit_latency(got)
        self.assertAlmostEqual(a, r.values["control_latency_s"], delta=1e-6)
        # TELEMETRY の t_us が送信時刻だった古いファームの記録は、遅延の値に★
        self.assertTrue(r.warnings, r.notes)
        got.fw_id = fit.FW_ID_TELEMETRY_SNAPSHOT_TIME
        self.assertEqual(fit.fit_latency(got).warnings, [])
        a = fit.fit_steer(log).values["tau_steer_s"]
        b = fit.fit_steer(got).values["tau_steer_s"]
        self.assertAlmostEqual(a, b, delta=1e-4)


class TestSpeedSimulation(unittest.TestCase):
    def test_fit_simulation_matches_vehicle_model(self):
        """`fit._simulate_speed` は `VehicleModel`（`SpeedController`）と同じ応答を返す
        （指令の切り替えが刻みの途中に来ても）。"""
        cmds = [fit.Cmd(t=0.0, target_speed=0.0, target_steer=0.0, brake=False, accel_limit=6.0),
                fit.Cmd(t=0.1013, target_speed=1.2, target_steer=0.0, brake=False, accel_limit=6.0),
                fit.Cmd(t=0.9031, target_speed=0.4, target_steer=0.0, brake=False, accel_limit=6.0)]
        t_eval = np.arange(0.02, 2.0, 0.02)
        run = fit._SpeedRun(idx=list(range(len(t_eval))), t=t_eval, v=np.zeros_like(t_eval),
                            ref0=0.0, target0=0.0, t_start=0.0, v_start=0.0)
        got = fit._simulate_speed(TRUTH, cmds, 0.0, run)
        from sim.vehicle import DriveInput, VehicleModel
        vm = VehicleModel(TRUTH, (0, 0, 0))
        dt, ref, k = 0.0001, [], 0
        for i in range(int(2.0 / dt)):
            tk = i * dt
            while k + 1 < len(cmds) and cmds[k + 1].t <= tk:
                k += 1
            c = cmds[k]
            vm.apply(DriveInput(armed=True, target_speed=c.target_speed, accel_limit=c.accel_limit))
            vm.step(dt)
            ref.append(vm.speed)
        want = np.interp(t_eval, (np.arange(len(ref)) + 1) * dt, ref)
        self.assertLess(float(np.max(np.abs(got - want))), 0.01)


class TestGeometry(unittest.TestCase):
    """舵の効き・中立ずれ・アンダーステア勾配は、観測で決まる量だけを当てはめる。"""

    def test_straight_only_fits_offset_only(self):
        log, _, _ = run_test("sysid_accel", TRUTH, seed=3)
        r = fit.fit_geometry({"accel": log}, replace(TRUTH, steer_offset_rad=0.0))
        self.assertEqual(set(r.values), {"steer_offset_rad"})
        self.assertAlmostEqual(r.values["steer_offset_rad"], TRUTH.steer_offset_rad, delta=0.003)

    def test_steer_test_alone_reads_gain_and_link(self):
        """ステア試験だけでも、舵の効き・リンクのガタ・不感帯は読める（旋回の30°に頼らない）。"""
        log, _, _ = run_test("sysid_steer", TRUTH, seed=3)
        r = fit.fit_geometry({"steer": log}, replace(TRUTH, steer_gain=1.0, steer_gain_cubic=0.0))
        self.assertAlmostEqual(r.values["steer_gain"], TRUTH.steer_gain, delta=0.03)
        self.assertAlmostEqual(r.values["steer_link_hysteresis_rad"],
                               TRUTH.steer_link_hysteresis_rad, delta=bench.ABS_TOL["steer_link_hysteresis_rad"])

    def test_straight_segments_exclude_acceleration(self):
        """中立ずれの材料は定速区間だけ（加速中は左右モータのトルク差でもヨーが出る）。"""
        log, _, _ = run_test("sysid_accel", TRUTH, seed=3)
        g = fit._geo_straight(log)
        self.assertGreaterEqual(len(g.idx), 50)
        for i in g.src:
            x = log.samples[i]
            self.assertFalse(x.brake)
            self.assertGreater(x.speed, 0.0)                 # 前進だけ
        # 加速中は含まない: どの観測点も、0.2s前から指令速度が変わっていない
        back = int(round(0.2 / fit._median_dt(np.array([x.t for x in log.samples]))))
        for i in g.src:
            self.assertEqual(log.samples[i].target_speed, log.samples[i - back].target_speed)


class TestRejectsUnmeasured(unittest.TestCase):
    def test_md_dropout_is_found_and_the_record_is_rejected(self):
        """MD が黙った記録は解析しない（止まった MD は駆動も制動もせず、車輪の値は古いまま固まる。
        2026-10-08、右後輪の MD が落ちた記録で制動の頭打ちが 2.75→1.91 と出た）。"""
        from tools.sysid import analyze
        ok, lost = [0x11, 0x11, 0x31], [0x11, 0x03, 0x31]
        msgs = [(0.01 * i, {"md_status": lost if 50 <= i < 130 else ok}) for i in range(200)]
        drops = fit.md_dropouts(msgs)
        self.assertEqual([n for _, _, n in drops], ["右後輪"])
        self.assertAlmostEqual(drops[0][0], 0.50)
        self.assertAlmostEqual(drops[0][1], 1.30)
        self.assertEqual(fit.md_dropouts([(0.0, {}), (0.1, {"md_status": [0, 0, 0]})]), [])  # 古い記録

        log, _, _ = run_test("sysid_corner", TRUTH, seed=1)
        self.assertEqual(log.md_dropouts, [])
        r = analyze.analyze({"corner": replace(log, md_dropouts=drops)}, TRUTH)
        self.assertNotIn("mu", r.results)
        self.assertRegex(r.errors[0], "右後輪のモータドライバ")

    def test_corner_without_reaching_the_limit_raises(self):
        log, _, _ = run_test("sysid_corner", TRUTH, params={"v_max": 1.0}, seed=1)
        with self.assertRaisesRegex(ValueError, "頭打ち"):
            fit.fit_corner(log, TRUTH)

    def test_steer_log_without_steps_raises(self):
        log, _, _ = run_test("sysid_accel", TRUTH, params={"cycles_per_stage": 1}, seed=1)
        with self.assertRaisesRegex(ValueError, "ステップ"):
            fit.fit_steer(log)

    def test_latency_on_non_latency_log_raises(self):
        log, _, _ = run_test("sysid_accel", TRUTH, params={"cycles_per_stage": 1}, seed=1)
        with self.assertRaises(ValueError):
            fit.fit_latency(log)


if __name__ == "__main__":
    unittest.main()
