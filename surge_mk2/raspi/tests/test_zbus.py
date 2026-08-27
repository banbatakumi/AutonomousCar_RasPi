"""内部バス（ZeroMQ ラッパ）のテスト。

ハードウェアは要らない。`ipc://` をテンポラリに閉じ込めるので、
**実機のバスとも他のテストとも干渉しない**。
"""

import os
import sys
import tempfile
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.msgs import DriveCmd, VehicleState  # noqa: E402
from raspi.msgs.types import TOPIC_CMD, TOPIC_VEHICLE_STATE  # noqa: E402


def _pump(sub, want=1, timeout_s=2.0):
    """届くまで回す。PUB/SUB は接続確立まで数十ms かかる（slow joiner）。"""
    got = []
    end = time.monotonic() + timeout_s
    while len(got) < want and time.monotonic() < end:
        got += sub.poll(20)
    return got


class BusCase(unittest.TestCase):
    """テストごとに別ディレクトリの ipc を使う。"""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self._old = os.environ.get("SURGE_BUS_DIR")
        os.environ["SURGE_BUS_DIR"] = self._tmp.name
        self._close = []

    def tearDown(self):
        for o in reversed(self._close):
            o.close()
        if self._old is None:
            os.environ.pop("SURGE_BUS_DIR", None)
        else:
            os.environ["SURGE_BUS_DIR"] = self._old
        self._tmp.cleanup()

    def track(self, o):
        self._close.append(o)
        return o


class TestWireFormat(BusCase):
    def test_round_trip(self):
        from raspi.bus.zbus import decode, encode

        msg = VehicleState(speed=1.25, steer_actual=-0.1, faults=["x"])
        topic, back = decode(encode(TOPIC_VEHICLE_STATE, msg))
        self.assertEqual(topic, TOPIC_VEHICLE_STATE)
        self.assertAlmostEqual(back.speed, 1.25)
        self.assertEqual(back.faults, ["x"])

    def test_single_frame_not_multipart(self):
        """**ZMQ_CONFLATE は multipart に対応していない。**

        トピックとペイロードを1フレームに詰めてあることを固定しておく。
        multipart に戻すと CONFLATE が黙って壊れる（症状は「古い点群で走る」）。
        """
        from raspi.bus.zbus import SEP, encode

        raw = encode("scan", VehicleState())
        self.assertTrue(raw.startswith(b"scan" + SEP))
        self.assertEqual(raw.count(SEP, 0, 5), 1)

    def test_garbage_is_rejected_loudly(self):
        from raspi.bus.zbus import decode

        with self.assertRaises(ValueError):
            decode(b"no separator here")


class TestPubSub(BusCase):
    def test_publisher_stamps_seq_and_t_pub(self):
        from raspi.bus import Publisher, Subscriber

        pub = self.track(Publisher("io"))
        sub = self.track(Subscriber([TOPIC_VEHICLE_STATE]))
        time.sleep(0.2)                       # slow joiner
        for _ in range(5):
            pub.send(TOPIC_VEHICLE_STATE, VehicleState(speed=1.0))
            time.sleep(0.01)
        got = _pump(sub, 1)
        self.assertTrue(got)
        _, msg = got[-1]
        self.assertGreater(msg.seq, 0)
        self.assertGreater(msg.t_pub, 0)
        self.assertEqual(msg.t_capture, msg.t_pub)   # 未設定なら t_pub で埋める

    def test_conflate_keeps_only_the_newest(self):
        """制御系が「2秒前の点群で舵を切る」のを防ぐのが CONFLATE の目的。"""
        from raspi.bus import LATEST, Publisher, Subscriber

        pub = self.track(Publisher("io"))
        sub = self.track(Subscriber({TOPIC_VEHICLE_STATE: LATEST}))
        time.sleep(0.2)
        for i in range(50):
            pub.send(TOPIC_VEHICLE_STATE, VehicleState(speed=float(i)))
        time.sleep(0.2)
        got = sub.poll(50)
        self.assertEqual(len(got), 1)               # 溜まっていない
        self.assertGreater(got[0][1].speed, 40)     # しかも新しい方

    def test_reliable_keeps_them_all(self):
        """ロガーは1フレームも落としたくない。同じトピックを別ポリシーで受ける。"""
        from raspi.bus import RELIABLE, Publisher, Subscriber

        pub = self.track(Publisher("io"))
        sub = self.track(Subscriber({TOPIC_VEHICLE_STATE: RELIABLE}))
        time.sleep(0.2)
        for i in range(50):
            pub.send(TOPIC_VEHICLE_STATE, VehicleState(speed=float(i)))
        time.sleep(0.3)
        got = _pump(sub, 50, timeout_s=1.0)
        self.assertEqual(len(got), 50)
        self.assertEqual([m.speed for _, m in got], [float(i) for i in range(50)])

    def test_subscription_is_exact_not_a_loose_prefix(self):
        """`scan` の購読が `scan_debug` に一致してはいけない（NUL 区切りの役目）。"""
        from raspi.bus import Publisher, Subscriber
        from raspi.bus.zbus import TOPIC_OWNER

        TOPIC_OWNER["scan_debug"] = "io"
        try:
            pub = self.track(Publisher("io"))
            sub = self.track(Subscriber(["scan"]))
            time.sleep(0.2)
            for _ in range(10):
                pub.send("scan_debug", VehicleState(speed=9.0))
            time.sleep(0.2)
            self.assertEqual(sub.poll(50), [])
        finally:
            TOPIC_OWNER.pop("scan_debug")

    def test_topics_do_not_evict_each_other(self):
        """**CONFLATE はソケット単位。** 1本に相乗りさせると、LiDAR が来るたびに
        車両状態が消える。トピックごとにソケットを分けてあることを固定する。
        """
        from raspi.bus import LATEST, Publisher, Subscriber
        from raspi.msgs import Scan
        from raspi.msgs.types import TOPIC_SCAN

        pub = self.track(Publisher("io"))
        sub = self.track(Subscriber({TOPIC_VEHICLE_STATE: LATEST, TOPIC_SCAN: LATEST}))
        time.sleep(0.2)
        pub.send(TOPIC_VEHICLE_STATE, VehicleState(speed=3.0))
        time.sleep(0.05)
        for _ in range(20):
            pub.send(TOPIC_SCAN, Scan())
        time.sleep(0.2)
        topics = {t for t, _ in sub.poll(100)}
        self.assertEqual(topics, {TOPIC_VEHICLE_STATE, TOPIC_SCAN})

    def test_cmd_flows_from_control_to_io(self):
        """`cmd` の publish 元は control ノード。購読側は接続先を知らなくてよい。"""
        from raspi.bus import Publisher, Subscriber

        pub = self.track(Publisher("control"))
        sub = self.track(Subscriber([TOPIC_CMD]))
        time.sleep(0.2)
        for _ in range(5):
            pub.send(TOPIC_CMD, DriveCmd(mode=1, target_steer=0.2, source="test"))
            time.sleep(0.01)
        got = _pump(sub, 1)
        self.assertTrue(got)
        self.assertEqual(got[-1][1].source, "test")
        self.assertAlmostEqual(got[-1][1].target_steer, 0.2)

    def test_unknown_topic_cannot_be_subscribed(self):
        from raspi.bus import Subscriber

        with self.assertRaises(KeyError):
            Subscriber(["mystery"])


class TestTopicOwner(unittest.TestCase):
    """★ `TOPIC_OWNER` の登録漏れ・前方一致の事故を防ぐ。

    `planning_node.main()` は登録されている**全** planner の `input_topic` を
    起動時にまとめて購読する（`input_topics()`）。ここに登録の無いトピックが
    1つでもあると `Subscriber.__init__` が `KeyError` で落ち、**存在する
    planner を選ばなくても** planning_node そのものが起動しない。実際に
    `line_trace`（`line/cam`）を `raspi/auto/registry.py` に足したときに
    `raspi/bus/zbus.py` の `TOPIC_OWNER` を更新し忘れて、`sim.run` の起動時に
    再現した（2026-08-23）。
    """

    def test_every_registered_planner_input_topic_has_an_owner(self):
        from raspi.auto import PLANNERS
        from raspi.bus.zbus import endpoints_for_topic

        for cls in PLANNERS.values():
            with self.subTest(planner=cls.id, topic=cls.input_topic):
                endpoints_for_topic(cls.input_topic)   # 例外が出ないこと

    def test_cam_model_is_owned_by_control_and_not_swallowed_by_cam_config(self):
        """`"cam/model"`（★モデル選択トピック）が `"cam/config"` の前方一致に
        誤って拾われていないこと。両方とも `TOPIC_OWNER` の完全一致キーとして
        別々に存在し、どちらも実際の owner は `control`（telemetry_node）だが、
        **偶然同じ答えになるからといって前方一致に頼ってはいけない**——
        将来どちらかの owner だけ変えたときに、もう片方が巻き添えで
        壊れる経路を残さないため。
        """
        from raspi.bus.zbus import TOPIC_OWNER, endpoint_for_node, endpoints_for_topic

        self.assertIn("cam/model", TOPIC_OWNER, "完全一致キーとして登録されていない")
        self.assertEqual(endpoints_for_topic("cam/model"),
                         [endpoint_for_node("control")])

    def test_scan_cam_is_not_swallowed_by_the_scan_prefix(self):
        """`"scan/cam"` は `"scan"` の前方一致フォールバックに拾われてはいけない。

        両方とも辞書に**完全一致キー**として存在すべきで、`"scan/cam"` を
        引いたときに `"scan"`（io ノード）の endpoint が返ってきたら事故
        （`cam_perception_node` の配信が誰にも届かない、が例外にはならず
        `ftg_cam` が「動くのにデータが来ない」という気づきにくい壊れ方をする）。
        """
        from raspi.bus.zbus import endpoint_for_node, endpoints_for_topic

        self.assertEqual(endpoints_for_topic("scan/cam"),
                         [endpoint_for_node("cam_perception")])
        self.assertEqual(endpoints_for_topic("scan"),
                         [endpoint_for_node("io")])


if __name__ == "__main__":
    unittest.main()
