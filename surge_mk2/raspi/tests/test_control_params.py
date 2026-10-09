"""`raspi/core/control_params.py`: `[control]` の読み込みと、STM32 へ入れて確かめる手順。"""

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.core.control_params import (CONTROL_PARAM_IDS, DEFAULT_TOML, OVERRIDABLE,  # noqa: E402
                                       STEER_LINK_X_MAX_RAD, ControlParamSync, load_control_params,
                                       steer_link_road, steer_link_x, steer_relink)
from raspi.proto.generated import packets  # noqa: E402

P = packets.Param
OK, UNKNOWN, CLAMPED = (packets.ConfigResult.OK, packets.ConfigResult.UNKNOWN_ID,
                        packets.ConfigResult.OUT_OF_RANGE)
MS = 1_000_000


class FakeStm:
    """`CONFIG_SET`/`CONFIG_GET` に実機と同じ形で答える。"""

    def __init__(self, sync_holder: list, known=None, clamp=None) -> None:
        self.values: dict[int, float] = {P.TC_ENABLE: 1.0, P.ABS_ENABLE: 1.0, P.TV_ENABLE: 1.0,
                                         P.WHEEL_LIFT_GUARD_ENABLE: 1.0}
        self.sent: list = []
        self._holder = sync_holder
        self._known = known
        self._clamp = clamp or {}
        self.answer = True

    def __call__(self, pkt) -> None:
        self.sent.append(pkt)
        if not self.answer:
            return
        pid = pkt.param_id
        if self._known is not None and pid not in self._known:
            self._holder[0].on_ack(packets.ConfigAck(param_id=pid, applied=0.0, result=UNKNOWN))
            return
        result = OK
        if isinstance(pkt, packets.ConfigSet):
            v = pkt.value
            if pid in self._clamp and v > self._clamp[pid]:
                v, result = self._clamp[pid], CLAMPED
            self.values[pid] = v
        self._holder[0].on_ack(packets.ConfigAck(param_id=pid, applied=self.values.get(pid, 0.0),
                                                 result=result))


def make(wanted, **kw):
    holder: list = [None]
    stm = FakeStm(holder, **kw)
    sync = ControlParamSync(wanted, stm)
    holder[0] = sync
    return sync, stm


def run(sync, start_ms=0, ms=3000):
    for t in range(start_ms, start_ms + ms, 10):
        sync.tick(t * MS)


class TestLoad(unittest.TestCase):
    def test_repo_toml_loads(self):
        """リポジトリの vehicle.toml が読め、[control] の値がそのまま出る（同定用の項目は出ない）。"""
        v = load_control_params(DEFAULT_TOML)
        self.assertTrue(set(v) <= set(CONTROL_PARAM_IDS))
        self.assertNotIn("tv_test_moment_nm", v)
        import tomllib
        with open(DEFAULT_TOML, "rb") as f:
            d = tomllib.load(f)
        self.assertEqual(v, {k: float(x) for k, x in d["control"].items() if not isinstance(x, dict)})
        self.assertIn("tv_load_gain_s2_per_m", v)

    def _toml(self, body: str) -> Path:
        f = tempfile.NamedTemporaryFile("w", suffix=".toml", delete=False, encoding="utf-8")
        f.write(body)
        f.close()
        self.addCleanup(Path(f.name).unlink)
        return Path(f.name)

    def test_missing_table_sends_nothing(self):
        self.assertEqual(load_control_params(self._toml("[dynamics]\nmu = 0.5\n")), {})

    def test_unknown_key_is_an_error(self):
        with self.assertRaises(ValueError):
            load_control_params(self._toml("[control]\ntc_slip_targett = 0.1\n"))

    def test_plant_subtable_is_ignored(self):
        v = load_control_params(self._toml("[control]\ntc_slip_target = 0.1\n[control.plant]\nmu = 0.6\n"))
        self.assertEqual(v, {"tc_slip_target": 0.1})


class TestSteerLink(unittest.TestCase):
    """ステアのリンクの換算（STM32 の `steering.c` と同じ式）と、別の換算のもとでの舵角の読み替え。"""

    LINK = (0.9928, -0.3917)

    def test_inverse_round_trips_over_the_range(self):
        for i in range(-20, 21):
            x = STEER_LINK_X_MAX_RAD * i / 20
            road = steer_link_road(x, *self.LINK)
            self.assertAlmostEqual(steer_link_x(road, *self.LINK), x, places=7)
        # 可動範囲の外は端で止まる
        self.assertAlmostEqual(steer_link_x(1.0, *self.LINK), STEER_LINK_X_MAX_RAD, places=9)

    def test_relink_keeps_the_motor_angle(self):
        """換算なしの頃の舵角（モータ角×0.5）は、今の換算のもとでは実際の向きの数値になる。"""
        self.assertEqual(steer_relink(0.3, self.LINK, self.LINK), 0.3)
        self.assertAlmostEqual(steer_relink(STEER_LINK_X_MAX_RAD, (1.0, 0.0), self.LINK),
                               steer_link_road(STEER_LINK_X_MAX_RAD, *self.LINK), places=9)
        back = steer_relink(steer_relink(0.25, (1.0, 0.0), self.LINK), self.LINK, (1.0, 0.0))
        self.assertAlmostEqual(back, 0.25, places=7)


class TestSync(unittest.TestCase):
    WANT = {"tc_slip_target": 0.12, "abs_kp_nm_per_m_s": 0.2, "tv_max_ratio": 0.4}

    def test_sends_everything_and_confirms(self):
        sync, stm = make(self.WANT)
        self.assertEqual(sync.status, "pending")
        run(sync, ms=200)
        self.assertEqual(sync.status, "ok")
        self.assertAlmostEqual(stm.values[P.TC_SLIP_TARGET], 0.12)
        self.assertAlmostEqual(sync.applied["abs_kp_nm_per_m_s"], 0.2)

    def test_retries_until_answered(self):
        sync, stm = make(self.WANT)
        stm.answer = False
        run(sync, ms=1000)
        self.assertEqual(sync.status, "pending")
        first = len([p for p in stm.sent if isinstance(p, packets.ConfigSet)])
        self.assertGreaterEqual(first, 6)           # 3項目を 300ms ごとに再送
        stm.answer = True
        run(sync, start_ms=1000, ms=500)
        self.assertEqual(sync.status, "ok")

    def test_nothing_to_send_is_none(self):
        sync, stm = make({})
        run(sync, ms=1000)
        self.assertEqual(sync.status, "none")
        self.assertEqual(stm.sent, [])

    def test_old_firmware_is_unsupported_and_not_spammed(self):
        sync, stm = make(self.WANT, known={P.TC_ENABLE})
        run(sync, ms=3000)
        self.assertEqual(sync.status, "unsupported")
        self.assertEqual(len(stm.sent), 3)           # 1回ずつ送って諦める
        self.assertTrue(all("対応していません" in p for p in sync.problems))

    def test_clamped_value_is_mismatch(self):
        sync, stm = make(self.WANT, clamp={P.TC_SLIP_TARGET: 0.1})
        run(sync, ms=1000)
        self.assertEqual(sync.status, "mismatch")
        self.assertIn("tc_slip_target", sync.problems[0])

    def test_stm_reboot_is_detected_by_readback_and_resent(self):
        """STM32 だけが再起動して既定値に戻っても、読み戻しで見つけて送り直す。"""
        sync, stm = make(self.WANT)
        run(sync, ms=200)
        stm.values[P.TC_SLIP_TARGET] = 0.10          # 既定値に戻った
        stm.values[P.ABS_KP_NM_PER_M_S] = 0.14
        run(sync, start_ms=200, ms=4000)
        self.assertEqual(sync.status, "ok")
        self.assertAlmostEqual(stm.values[P.TC_SLIP_TARGET], 0.12)
        self.assertAlmostEqual(stm.values[P.ABS_KP_NM_PER_M_S], 0.2)
        self.assertEqual(sync.drift_count, 2)

    def test_reset_resends_all(self):
        sync, stm = make(self.WANT)
        run(sync, ms=200)
        n = len(stm.sent)
        sync.reset()
        self.assertEqual(sync.status, "pending")
        run(sync, start_ms=200, ms=200)
        sets = [p for p in stm.sent[n:] if isinstance(p, packets.ConfigSet)]
        self.assertEqual({p.param_id for p in sets}, {CONTROL_PARAM_IDS[k] for k in self.WANT})

    def test_unknown_name_is_rejected(self):
        with self.assertRaises(ValueError):
            ControlParamSync({"no_such_param": 1.0}, lambda p: None)


class TestOverrides(unittest.TestCase):
    def test_override_applies_and_restores(self):
        """試験の間だけ TC を切り、指令が途絶えたら元へ戻す。"""
        sync, stm = make({"tc_slip_target": 0.12})
        run(sync, ms=200)
        sync.set_overrides({"tc_enable": 0.0, "tv_test_moment_nm": 0.1})
        self.assertTrue(sync.overriding)
        run(sync, start_ms=200, ms=100)
        self.assertEqual(stm.values[P.TC_ENABLE], 0.0)
        self.assertAlmostEqual(stm.values[P.TV_TEST_MOMENT_NM], 0.1)
        sync.set_overrides({})
        run(sync, start_ms=300, ms=100)
        self.assertEqual(stm.values[P.TC_ENABLE], 1.0)
        self.assertEqual(stm.values[P.TV_TEST_MOMENT_NM], 0.0)
        self.assertEqual(sync.status, "ok")

    def test_restores_the_value_before_the_override(self):
        """人が GUI で切っていた機能は、試験の後も切れたまま（勝手に入れない）。"""
        sync, stm = make({})
        stm.values[P.ABS_ENABLE] = 0.0
        sync.on_ack(packets.ConfigAck(param_id=P.ABS_ENABLE, applied=0.0, result=OK))   # 起動時の CONFIG_GET
        sync.set_overrides({"abs_enable": 1.0})
        run(sync, ms=100)
        self.assertEqual(stm.values[P.ABS_ENABLE], 1.0)
        sync.set_overrides({})
        run(sync, start_ms=100, ms=100)
        self.assertEqual(stm.values[P.ABS_ENABLE], 0.0)

    def test_override_of_a_tuned_param_goes_back_to_toml_value(self):
        sync, stm = make({"tc_slip_target": 0.12})
        run(sync, ms=200)
        sync.set_overrides({"tc_slip_target": 0.3})
        run(sync, start_ms=200, ms=100)
        self.assertAlmostEqual(stm.values[P.TC_SLIP_TARGET], 0.3)
        sync.set_overrides({})
        run(sync, start_ms=300, ms=100)
        self.assertAlmostEqual(stm.values[P.TC_SLIP_TARGET], 0.12)

    def test_garbage_overrides_are_ignored(self):
        sync, stm = make({})
        sync.set_overrides({"no_such": 1.0, "tc_enable": float("nan"), "abs_enable": True})
        self.assertFalse(sync.overriding)
        run(sync, ms=100)
        self.assertEqual(stm.sent, [])

    def test_changing_moment_is_sent_at_once(self):
        """ヨーモーメントの向きの切り替えは、再送の間隔を待たずに次の tick で送る。"""
        sync, stm = make({})
        sync.set_overrides({"tv_test_moment_nm": 0.12})
        sync.tick(0)
        sync.set_overrides({"tv_test_moment_nm": -0.12})
        sync.tick(10 * MS)
        self.assertAlmostEqual(stm.values[P.TV_TEST_MOMENT_NM], -0.12)

    def test_names_cover_enable_flags(self):
        self.assertEqual(OVERRIDABLE["tc_enable"], P.TC_ENABLE)
        self.assertEqual(OVERRIDABLE["wheel_lift_guard_enable"], P.WHEEL_LIFT_GUARD_ENABLE)


if __name__ == "__main__":
    unittest.main()
