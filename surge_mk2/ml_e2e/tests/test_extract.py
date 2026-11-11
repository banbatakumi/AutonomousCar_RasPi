"""`ml_e2e/extract.py`・`records.py` のテスト。

合成した記録（半径 2m の円を 1m/s で反時計回りに走る車）を本物の書き手（`McapLog`）で `.mcap` にし、
抽出したラベルが幾何として合っているかを確かめる。真値が分かっているので、姿勢の時刻のずれ・座標変換・
点群の選び方の間違いがそのまま数字に出る。
"""

import math
import sys
import tempfile
import unittest
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

import numpy as np  # noqa: E402

from ml_e2e import extract as X  # noqa: E402
from ml_e2e import records as R  # noqa: E402
from raspi.msgs import AutoMap, AutoState, DriveCmd, Scan, VehicleState  # noqa: E402
from raspi.rec.mcap_log import McapLog  # noqa: E402
from raspi.rec.odom import OdomPose  # noqa: E402

NS = 1_000_000_000
RADIUS = 2.0
SPEED = 1.0
OMEGA = SPEED / RADIUS
#: 最適ラインは、車が走る円から中心をずらした円。**車の走る線と同じにしない**——同じだと、姿勢の時刻を
#: 間違えても経路点の形が変わらず（円の対称性）、間違いがテストに出ない
LINE_CX = 0.8
#: 合成した記録で、姿勢が `AutoState` の時刻からどれだけ古いか（点群の古さ 80ms − 基準時刻のずれ 55ms）
SCAN_AGE_S = 0.080
POSE_LAG_S = 0.055


def _map_pose(t_s: float) -> tuple[float, float, float]:
    """円の上の真の姿勢（地図の座標）。"""
    a = OMEGA * t_s
    return RADIUS * math.cos(a), RADIUS * math.sin(a), a + math.pi / 2


def _write(path: Path, *, mode: int = 2, source: str = "auto:slam2d_route", arm: bool = True,
           duration_s: float = 14.0, with_map: bool = True,
           jpeg: bytes = b"\xff\xd8jpeg\xff\xd9") -> None:
    line = np.stack([LINE_CX + RADIUS * np.cos(np.linspace(0, 2 * math.pi, 180, endpoint=False)),
                     RADIUS * np.sin(np.linspace(0, 2 * math.pi, 180, endpoint=False))], axis=1)
    with McapLog(path, t0_mono_ns=0, t0_unix_ns=0) as log:
        if with_map:
            log.write("/auto/map", AutoMap(map_seq=1, raceline=line.reshape(-1).tolist(),
                                           raceline_v=[2.0] * len(line), map_name="circle"), t_mono_ns=1)
        for i in range(int(duration_s * 100)):
            t = i * 10_000_000
            ts = t / NS
            log.write("/vehicle_state", VehicleState(speed=SPEED, yaw_rate=OMEGA, steer_actual=0.11),
                      t_mono_ns=t)
            # 推測航法は記録の開始位置が原点・開始の向きが 0
            a = OMEGA * ts
            log.write("/odom", OdomPose(t_capture=t, x=RADIUS * math.sin(a), y=RADIUS * (1 - math.cos(a)),
                                        yaw=a, yaw_unwrapped=a, dist=SPEED * ts), t_mono_ns=t)
            if i % 2 == 0:
                log.write("/cmd", DriveCmd(mode=mode, arm=arm, target_steer=0.1, target_speed=1.2,
                                           source=source), t_mono_ns=t)
            if i % 10 == 0:
                log.write("/scan", Scan(dist=[3.0] * 360, sector_seen=[True] * 12, seq=i // 10), t_mono_ns=t)
                log.write_viz_image(jpeg, "front", t_mono_ns=t + 30_000_000)
                if with_map:
                    x, y, yaw = _map_pose(ts - SCAN_AGE_S + POSE_LAG_S)
                    log.write("/auto/state", AutoState(mode="slam2d_route", phase="RACE", engaged=True,
                                                       pose_x=x, pose_y=y, pose_yaw=yaw, match_score=0.9,
                                                       scan_age_ms=SCAN_AGE_S * 1e3), t_mono_ns=t)


class TestExtractAutoRun(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        d = Path(cls.tmp.name)
        _write(d / "run.mcap")
        cls.rejects = Counter()
        cls.info = X.extract_recording(d / "run.mcap", d / "out", X.ExtractConfig(k=8, ds=0.25),
                                       rejects=cls.rejects)
        cls.data = np.load(d / "out" / "run.npz")

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_constant_turn_cannot_determine_the_pose_lag(self):
        """回頭が一定の円では、時刻をずらしても回頭の差が変わらないので決まらない（既定値を使う）。"""
        lag, _rms = R.estimate_pose_lag(R.load(Path(self.tmp.name) / "run.mcap"))
        self.assertTrue(math.isnan(lag))
        self.assertIsNone(self.info["pose_lag_ms"])

    def test_all_samples_use_the_race_line(self):
        self.assertGreater(self.info["samples"], 100)
        self.assertEqual(self.info["line"], self.info["samples"])
        self.assertTrue((self.data["path_src"] == X.SRC_LINE).all())
        self.assertTrue(self.data["v_ok"].all())
        for key in ("steer_cmd", "speed_cmd", "speed_ok"):            # 舵・速度の指令は手本にしない
            self.assertNotIn(key, self.data)

    def test_path_points_lie_on_the_line_at_the_image_time(self):
        """経路点を真の姿勢（撮影時刻）で地図へ戻すと、最適ラインの上に ds おきに並ぶ。"""
        n = len(self.data["t"])
        for i in range(0, n, max(1, n // 12)):
            x0, y0, yaw = _map_pose(int(self.data["t"][i]) / NS)
            c, s = math.cos(yaw), math.sin(yaw)
            p = self.data["path_xy"][i]
            wx, wy = x0 + c * p[:, 0] - s * p[:, 1], y0 + s * p[:, 0] + c * p[:, 1]
            np.testing.assert_allclose(np.hypot(wx - LINE_CX, wy), RADIUS, atol=0.012)
            step = np.hypot(np.diff(wx), np.diff(wy))
            np.testing.assert_allclose(step, 0.25, atol=0.01)
            # 横偏差は、線の左（反時計回りの円の内側）が正
            self.assertAlmostEqual(float(self.data["cross"][i]), RADIUS - math.hypot(x0 - LINE_CX, y0), delta=0.012)
        np.testing.assert_allclose(self.data["path_v"], 2.0, atol=1e-5)

    def test_scan_is_older_than_the_image_by_the_expected_range(self):
        age = self.data["scan_age"]
        self.assertGreaterEqual(age.min(), X.SCAN_MIN_AGE_S - 1e-6)
        self.assertLess(age.max(), X.SCAN_MIN_AGE_S + 0.1 + 1e-6)
        # 履歴は最新から1周ずつ古い
        idx = self.data["scan_idx"][10]
        self.assertEqual(idx.tolist(), [idx[0], idx[0] - 1, idx[0] - 2])

    def test_state(self):
        np.testing.assert_allclose(self.data["state"][:, 0], SPEED, atol=1e-6)
        np.testing.assert_allclose(self.data["state"][:, 1], 0.11, atol=1e-6)
        np.testing.assert_allclose(self.data["state"][:, 2], OMEGA, atol=1e-6)

    def test_images_are_written(self):
        first = Path(self.tmp.name) / "out" / str(self.data["image"][0])
        self.assertTrue(first.is_file())


class TestPoseLag(unittest.TestCase):
    def _recording(self, lag_s: float) -> R.Recording:
        """回頭が正弦波で変わる走行。`AutoState` の姿勢は `t_state − scan_age + lag_s` のもの。"""
        t = (np.arange(3000) * 10_000_000).astype(np.int64)             # 100Hz・30s

        def yaw(ts):
            return 0.8 * np.sin(2 * math.pi * 0.5 * ts)                 # 0.5Hz で左右に振る

        rec = R.Recording(path=Path("synthetic"))
        rec.od_t, rec.od_yaw = t, yaw(t / NS)
        rec.od_x = rec.od_y = rec.od_dist = np.zeros(len(t))
        st_t = t[50::10]
        rec.st_t = st_t
        rec.st_scan_age = np.full(len(st_t), SCAN_AGE_S)
        rec.st_pose = np.stack([np.zeros(len(st_t)), np.zeros(len(st_t)),
                                yaw(st_t / NS - SCAN_AGE_S + lag_s)], axis=1)
        rec.st_score = np.full(len(st_t), 0.9)
        rec.st_phase = ["RACE"] * len(st_t)
        return rec

    def test_recovers_the_lag_within_the_search_step(self):
        for lag in (0.03, 0.055, 0.1):
            est, rms = R.estimate_pose_lag(self._recording(lag))
            self.assertAlmostEqual(est, lag, delta=0.006, msg=f"lag={lag}")
            self.assertLess(rms, 0.005)


class TestExtractOtherRuns(unittest.TestCase):
    def _run(self, **kw) -> tuple[dict, Counter, Path]:
        d = Path(self._tmp.name)
        _write(d / "run.mcap", **kw)
        rejects: Counter = Counter()
        info = X.extract_recording(d / "run.mcap", d / "out", X.ExtractConfig(k=8, ds=0.25), rejects=rejects)
        return info, rejects, d / "out" / "run.npz"

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)

    def test_manual_run_uses_the_driven_path_and_masks_speed(self):
        info, _rej, npz = self._run(mode=1, source="gui:x", with_map=False)
        d = np.load(npz)
        self.assertEqual(info["future"], info["samples"])
        self.assertFalse(d["v_ok"].any())
        # 通った軌跡は左へ曲がる円弧: 原点からの弦の長さが円の弦と合う
        p = d["path_xy"][5]
        ok = d["path_valid"][5]
        self.assertTrue(ok.all())
        chord = 2 * RADIUS * np.sin(0.25 * np.arange(1, 9) / RADIUS / 2)
        np.testing.assert_allclose(np.hypot(p[:, 0], p[:, 1]), chord, atol=0.01)
        self.assertTrue((p[:, 1] > 0).all())

    def test_end_of_recording_has_invalid_future_points(self):
        _info, _rej, npz = self._run(mode=1, source="gui:x", with_map=False)
        d = np.load(npz)
        self.assertFalse(d["path_valid"][-1].all())
        self.assertTrue(d["path_valid"][-1][0] or len(d["t"]) > 0)

    def test_disarmed_and_stale_and_self_frames_are_rejected(self):
        for kw, reason in ((dict(arm=False), "DISARM中"),
                           (dict(source="auto:slam2d_route:stale"), "入力途絶中の指令(stale)"),
                           (dict(source="auto:e2e_fusion"), "このモデル自身の走行")):
            info, rej, _ = self._run(**kw)
            self.assertEqual(info["samples"], 0, reason)
            self.assertGreater(rej[reason], 0, reason)


if __name__ == "__main__":
    unittest.main()
