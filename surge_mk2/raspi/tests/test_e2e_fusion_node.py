"""`raspi/nodes/e2e_fusion_node.py` の配線テスト。

**実モデルは要らない。** ダミーの ONNX（`avail` の先頭2つを舵・速度に写し、残りは定数）で、
点群の履歴 → 前処理 → 推論 → `E2EFusionPlan` 化と、契約の照合・モデルの選び方を確かめる。
"""

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

try:
    import onnx  # noqa: E402
    from onnx import TensorProto, helper, numpy_helper  # noqa: E402
except ModuleNotFoundError as e:    # ダミーモデルを書き出すだけの道具。Pi には入れない
    raise unittest.SkipTest(f"{e}（モデルを作る側の道具。Mac で回す）") from e

from raspi.core.cam_e2e_preproc import PREPROC_VERSION_CROP  # noqa: E402
from raspi.core.e2e_fusion_preproc import PREPROC_VERSION  # noqa: E402
from raspi.core.vehicle import Vehicle  # noqa: E402
from raspi.msgs import Scan, VehicleState  # noqa: E402
from raspi.nodes.e2e_fusion_node import E2EFusionNode, load_model, pick_model_name  # noqa: E402

NS = 1_000_000_000
K, W, H, FOV, HIST = 4, 32, 16, 90, 3


def _contract(**over) -> dict:
    c = {"preproc_version": PREPROC_VERSION, "image_preproc_version": PREPROC_VERSION_CROP,
         "input_size": [W, H], "crop_x": 0.05, "fov_deg": FOV, "max_range": 8.0, "history": HIST,
         "k": K, "ds": 0.5, "xy_scale": 2.0, "max_steer": 0.4, "speed_ref": 5.0, "yaw_rate_ref": 6.0}
    c.update(over)
    return c


def _write_model(path: Path, contract: dict | None = None, *, tail: np.ndarray | None = None) -> None:
    """出力 = `tail`（定数 4K 個）× `avail[1]`。点群を使ったかが出力に出る。"""
    if tail is None:
        tail = np.concatenate([np.linspace(0.25, 1.0, K), np.zeros(K), np.full(K, 0.4), np.full(K, 0.1)])
    inputs = [helper.make_tensor_value_info("image", TensorProto.FLOAT, [1, 3, H, W]),
              helper.make_tensor_value_info("lidar", TensorProto.FLOAT, [1, HIST + 1, FOV + 1]),
              helper.make_tensor_value_info("state", TensorProto.FLOAT, [1, 3]),
              helper.make_tensor_value_info("avail", TensorProto.FLOAT, [1, 3])]
    init = [numpy_helper.from_array(np.array([1], dtype=np.int64), "idx"),
            numpy_helper.from_array(tail.astype(np.float32)[None, :], "tail")]
    nodes = [helper.make_node("Gather", ["avail", "idx"], ["a_lidar"], axis=1),
             helper.make_node("Mul", ["tail", "a_lidar"], ["output"])]
    out = helper.make_tensor_value_info("output", TensorProto.FLOAT, [1, 4 * K])
    model = helper.make_model(helper.make_graph(nodes, "dummy", inputs, [out], init),
                              opset_imports=[helper.make_opsetid("", 18)])
    model.ir_version = 9
    onnx.save(model, str(path))
    path.with_suffix(".json").write_text(json.dumps(contract if contract is not None else _contract()))


def _scan(t: int, seq: int = 0, dist: float = 2.0) -> Scan:
    return Scan(t_capture=t, seq=seq, dist=[dist] * 360, sector_seen=[True] * 12)


class _Tmp(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.dir = Path(self._tmp.name)


class TestLoadModel(_Tmp):
    def test_loads_a_matching_contract(self):
        _write_model(self.dir / "m.onnx")
        m = load_model(self.dir / "m.onnx")
        self.assertEqual((m.k, m.fov_deg, m.history), (K, FOV, HIST))

    def test_rejects_missing_or_mismatched_contracts(self):
        with self.assertRaises(FileNotFoundError):
            load_model(self.dir / "none.onnx")
        for name, contract in (("ver", _contract(preproc_version=PREPROC_VERSION + 1)),
                               ("img", _contract(image_preproc_version=1)),
                               ("crop", _contract(crop_x=0.0)),
                               ("k", _contract(k=K + 1)),
                               ("ref", _contract(speed_ref=0.0))):
            _write_model(self.dir / f"{name}.onnx", contract)
            with self.assertRaises(ValueError, msg=name):
                load_model(self.dir / f"{name}.onnx")
        missing = _contract()
        del missing["xy_scale"]
        _write_model(self.dir / "miss.onnx", missing)
        with self.assertRaises(ValueError):
            load_model(self.dir / "miss.onnx")


class TestPickModel(_Tmp):
    def test_newest_model_with_a_contract_wins_unless_configured(self):
        _write_model(self.dir / "old.onnx")
        _write_model(self.dir / "new.onnx")
        os.utime(self.dir / "old.onnx", ns=(1_000 * NS, 1_000 * NS))
        os.utime(self.dir / "new.onnx", ns=(2_000 * NS, 2_000 * NS))
        (self.dir / "nocontract.onnx").write_bytes(b"x")          # 契約の無いものは選ばない
        os.utime(self.dir / "nocontract.onnx", ns=(3_000 * NS, 3_000 * NS))
        conf = self.dir / "conf.json"
        self.assertEqual(pick_model_name(self.dir, conf), "new")
        conf.write_text(json.dumps({"name": "old"}))
        self.assertEqual(pick_model_name(self.dir, conf), "old")
        conf.write_text(json.dumps({"name": "gone"}))             # 無い名前は無視
        self.assertEqual(pick_model_name(self.dir, conf), "new")
        self.assertEqual(pick_model_name(self.dir / "empty", conf), "")


class TestInferOnce(_Tmp):
    def setUp(self):
        super().setUp()
        _write_model(self.dir / "m.onnx")
        self.node = E2EFusionNode(model=load_model(self.dir / "m.onnx"), model_name="m",
                                  models_dir=self.dir, vehicle=Vehicle.load())
        self.addCleanup(self.node.close)
        self.vs = VehicleState(t_capture=10 * NS, speed=1.0)
        self.rgb = np.zeros((48, 96, 3), dtype=np.uint8)

    def test_image_and_lidar_give_a_ready_plan_at_the_image_time(self):
        self.node.push_scan(_scan(10 * NS - NS // 10))
        plan = self.node.infer_once(rgb=self.rgb, t_image_ns=10 * NS - 20_000_000, vs=self.vs, now_ns=10 * NS)
        self.assertTrue(plan.ready, plan.reason)
        self.assertTrue(plan.has_image and plan.has_lidar)
        self.assertEqual(plan.t_capture, 10 * NS - 20_000_000)
        np.testing.assert_allclose(plan.path_x, np.linspace(0.25, 1.0, K) * 2.0, atol=1e-6)
        np.testing.assert_allclose(plan.path_v, 2.0, atol=1e-6)
        np.testing.assert_allclose(plan.path_b, 0.1, atol=1e-6)
        self.assertEqual((plan.ds, plan.model), (0.5, "m"))

    def test_without_image_it_runs_on_lidar_and_dates_the_plan_from_the_scan(self):
        t_scan = 10 * NS - NS // 10
        self.node.push_scan(_scan(t_scan))
        plan = self.node.infer_once(rgb=None, t_image_ns=0, vs=self.vs, now_ns=10 * NS)
        self.assertTrue(plan.ready)
        self.assertFalse(plan.has_image)
        self.assertGreater(plan.t_capture, t_scan)
        self.assertLess(plan.t_capture - t_scan, 0.2 * NS)

    def test_stale_scan_is_not_used(self):
        self.node.push_scan(_scan(10 * NS - NS))                      # 1s 前
        plan = self.node.infer_once(rgb=self.rgb, t_image_ns=10 * NS, vs=self.vs, now_ns=10 * NS)
        self.assertTrue(plan.ready)
        self.assertFalse(plan.has_lidar)
        none = self.node.infer_once(rgb=None, t_image_ns=0, vs=self.vs, now_ns=10 * NS)
        self.assertFalse(none.ready)
        self.assertIn("届いていない", none.reason)

    def test_history_drops_scans_older_than_their_slot(self):
        """2周前の枠に入るのは、最新から 0.25s 以内の周だけ。抜けた周はそれより新しい周で埋まる。"""
        now = 10 * NS
        self.node.push_scan(_scan(now - 6 * NS // 10, dist=1.0))      # 古すぎる
        self.node.push_scan(_scan(now - 2 * NS // 10, dist=4.0))      # 1周前
        self.node.push_scan(_scan(now - 1 * NS // 10, dist=2.0))      # 最新
        x, t0 = self.node._lidar(now)
        self.assertEqual(t0, now - NS // 10)
        np.testing.assert_allclose(x[0, 0], 0.25)
        np.testing.assert_allclose(x[0, 1], 0.5)
        np.testing.assert_allclose(x[0, 2], 0.5)                      # 0.125（古すぎる周）ではない

    def test_no_model_and_bad_output_give_reasons(self):
        empty = E2EFusionNode(models_dir=self.dir / "none", vehicle=Vehicle.load())
        self.addCleanup(empty.close)
        self.assertIn("モデルが無い", empty.infer_once(rgb=self.rgb, t_image_ns=1, vs=self.vs, now_ns=1).reason)
        _write_model(self.dir / "nan.onnx", tail=np.full(4 * K, np.nan))
        self.assertTrue(self.node.reload_if_changed("nan"))
        self.node.push_scan(_scan(10 * NS - NS // 10))
        plan = self.node.infer_once(rgb=self.rgb, t_image_ns=10 * NS, vs=self.vs, now_ns=10 * NS)
        self.assertFalse(plan.ready)
        self.assertIn("推論に失敗", plan.reason)

    def test_failed_reload_keeps_the_current_model(self):
        _write_model(self.dir / "bad.onnx", _contract(preproc_version=99))
        self.assertFalse(self.node.reload_if_changed("bad"))
        self.assertEqual(self.node.model_name, "m")
        self.assertFalse(self.node.reload_if_changed("bad"))          # 同じファイルは再試行しない


class TestModelChoiceInRun(_Tmp):
    """`run()` のモデルの選び方: GUI の指定 → すぐ切り替える／「自動」→ 推論していない間だけ最新へ。"""

    def setUp(self):
        super().setUp()
        _write_model(self.dir / "old.onnx")
        _write_model(self.dir / "new.onnx")
        os.utime(self.dir / "old.onnx", ns=(1_000 * NS, 1_000 * NS))
        os.utime(self.dir / "new.onnx", ns=(2_000 * NS, 2_000 * NS))
        self.node = E2EFusionNode(models_dir=self.dir, vehicle=Vehicle.load())
        self.addCleanup(self.node.close)

    def _spin(self, latest: dict) -> list:
        from raspi.msgs.types import TOPIC_E2E_FUSION_PLAN
        from raspi.tests.fakes import FakePub, FakeSub

        pub = FakePub()
        self.node.run(sub=FakeSub(latest), pub=pub, duration_s=0.15)
        return [m for t, m in pub.sent if t == TOPIC_E2E_FUSION_PLAN]

    def test_auto_picks_the_newest_while_idle(self):
        from raspi.msgs import E2EFusionModelCtrl
        from raspi.msgs.types import TOPIC_E2E_FUSION_MODEL

        plans = self._spin({TOPIC_E2E_FUSION_MODEL: E2EFusionModelCtrl(name="")})
        self.assertEqual(self.node.model_name, "new")
        self.assertTrue(plans and not plans[-1].ready)              # モードが選ばれていないので推論しない

    def test_gui_choice_wins_over_the_newest(self):
        from raspi.msgs import E2EFusionModelCtrl
        from raspi.msgs.types import TOPIC_E2E_FUSION_MODEL

        self._spin({TOPIC_E2E_FUSION_MODEL: E2EFusionModelCtrl(name="old")})
        self.assertEqual(self.node.model_name, "old")
        # 「自動」に戻すと最新へ
        self.node._failed_key = None
        self._spin({TOPIC_E2E_FUSION_MODEL: E2EFusionModelCtrl(name="")})
        self.assertEqual(self.node.model_name, "new")

    def test_gui_choice_switches_even_while_active_but_auto_does_not(self):
        from raspi.msgs import AutoCtrl, E2EFusionModelCtrl
        from raspi.msgs.types import TOPIC_AUTO_CTRL, TOPIC_E2E_FUSION_MODEL, TOPIC_VEHICLE_STATE

        active = {TOPIC_AUTO_CTRL: AutoCtrl(mode="e2e_fusion"), TOPIC_VEHICLE_STATE: VehicleState(armed=True)}
        self._spin({**active, TOPIC_E2E_FUSION_MODEL: E2EFusionModelCtrl(name="")})
        self.assertEqual(self.node.model_name, "", "推論中に「自動」で勝手にモデルを読んだ")
        self._spin({**active, TOPIC_E2E_FUSION_MODEL: E2EFusionModelCtrl(name="old")})
        self.assertEqual(self.node.model_name, "old")

    def test_fixed_model_ignores_the_gui(self):
        from raspi.msgs import E2EFusionModelCtrl
        from raspi.msgs.types import TOPIC_E2E_FUSION_MODEL
        from raspi.tests.fakes import FakePub, FakeSub

        self.node.reload_if_changed("old")
        self.node.run(sub=FakeSub({TOPIC_E2E_FUSION_MODEL: E2EFusionModelCtrl(name="new")}), pub=FakePub(),
                      duration_s=0.1, fixed_model=True)
        self.assertEqual(self.node.model_name, "old")


if __name__ == "__main__":
    unittest.main()
