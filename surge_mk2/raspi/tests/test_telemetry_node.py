"""`raspi/nodes/telemetry_node.py`のうち、`self`に依存しない純粋な部分だけを狙って
テストする。`TelemetryServer`本体はソケット類を抱えて重く、軽量に構築する口が
無いので、対象を「`self`を一切使わないメソッド」に絞ってある
（`_e2e_models_list`は`TelemetryServer._e2e_models_list(None)`のように未束縛でも
呼べる）。
"""

import asyncio
import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

import raspi.nodes.telemetry_node as tn  # noqa: E402
from raspi.auto import mapstore  # noqa: E402
from raspi.msgs import AutoMap, AutoState, VehicleState  # noqa: E402
from raspi.msgs.types import TOPIC_AUTO_MAP, TOPIC_AUTO_STATE  # noqa: E402
from raspi.nav.grid import OccGrid, pack_trinary  # noqa: E402


class TestAtomicWriteBytes(unittest.TestCase):
    """★ C4: 設定JSONの書き込みが `os.replace()` でアトミックであること。"""

    def test_writes_the_content(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "sub" / "auto.json"
            tn._atomic_write_bytes(path, b'{"mode":"ftg"}')
            self.assertEqual(path.read_bytes(), b'{"mode":"ftg"}')

    def test_failure_mid_write_does_not_corrupt_existing_file(self):
        """途中で例外が起きても、既存ファイルの中身は古いまま保たれること。

        `os.fdopen(...).write()` が例外を投げても、`os.replace()` に
        到達しないので元ファイルは触られない——tmpファイルへの直書きだけが
        失敗するアトミック置換の要点。
        """
        from unittest import mock

        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "auto.json"
            path.write_bytes(b'{"mode":"old"}')

            with mock.patch("os.replace", side_effect=RuntimeError("boom")):
                with self.assertRaises(RuntimeError):
                    tn._atomic_write_bytes(path, b'{"mode":"new"}')

            self.assertEqual(path.read_bytes(), b'{"mode":"old"}')

    def test_no_leftover_tmp_file_on_success(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "auto.json"
            tn._atomic_write_bytes(path, b"{}")
            leftovers = [p for p in Path(td).iterdir() if p.name != "auto.json"]
            self.assertEqual(leftovers, [])


class TestE2eModelsListNote(unittest.TestCase):
    """`_e2e_models_list()`が`<名前>.json`の`note`（`ml_lidar/export_onnx_rl.py`が
    書く自由記述の備考。2026-08-29追加）を拾って返すことを確認する。"""

    def setUp(self) -> None:
        self._orig_dir = tn.E2E_MODELS_DIR
        self._tmp = tempfile.TemporaryDirectory()
        tn.E2E_MODELS_DIR = Path(self._tmp.name)
        self.addCleanup(self._cleanup)

    def _cleanup(self) -> None:
        tn.E2E_MODELS_DIR = self._orig_dir
        self._tmp.cleanup()

    def _by_name(self, files: list[dict]) -> dict[str, dict]:
        return {f["name"]: f for f in files}

    def test_reads_note_from_sibling_json(self) -> None:
        d = tn.E2E_MODELS_DIR
        (d / "v1.onnx").write_bytes(b"dummy")
        (d / "v1.json").write_text(json.dumps({"note": "グリップ限界+報酬改善版"}))

        result = tn.TelemetryServer._e2e_models_list(None)
        files = self._by_name(result["e2e_model_files"])
        self.assertEqual(files["v1"]["note"], "グリップ限界+報酬改善版")
        self.assertTrue(files["v1"]["has_config"])

    def test_missing_json_gives_empty_note(self) -> None:
        d = tn.E2E_MODELS_DIR
        (d / "v2.onnx").write_bytes(b"dummy")

        result = tn.TelemetryServer._e2e_models_list(None)
        files = self._by_name(result["e2e_model_files"])
        self.assertEqual(files["v2"]["note"], "")
        self.assertFalse(files["v2"]["has_config"])

    def test_corrupt_json_gives_empty_note_without_crashing(self) -> None:
        d = tn.E2E_MODELS_DIR
        (d / "v3.onnx").write_bytes(b"dummy")
        (d / "v3.json").write_text("{ not json")

        result = tn.TelemetryServer._e2e_models_list(None)
        files = self._by_name(result["e2e_model_files"])
        self.assertEqual(files["v3"]["note"], "")
        self.assertTrue(files["v3"]["has_config"])   # ファイルは在る（壊れているだけ）

    def test_json_without_note_field_gives_empty_note(self) -> None:
        d = tn.E2E_MODELS_DIR
        (d / "v4.onnx").write_bytes(b"dummy")
        (d / "v4.json").write_text(json.dumps({"max_speed": 1.5}))

        result = tn.TelemetryServer._e2e_models_list(None)
        files = self._by_name(result["e2e_model_files"])
        self.assertEqual(files["v4"]["note"], "")


class TestCamModelsListNote(unittest.TestCase):
    """`_cam_models_list()`が`<名前>.json`の`note`（`ml_cam/export_onnx.py`が
    書く自由記述の備考。`TestE2eModelsListNote`と対称、2026-08-29追加）を拾って
    返すことを確認する。"""

    def setUp(self) -> None:
        self._orig_dir = tn.MODELS_DIR
        self._tmp = tempfile.TemporaryDirectory()
        tn.MODELS_DIR = Path(self._tmp.name)
        self.addCleanup(self._cleanup)

    def _cleanup(self) -> None:
        tn.MODELS_DIR = self._orig_dir
        self._tmp.cleanup()

    def _by_name(self, files: list[dict]) -> dict[str, dict]:
        return {f["name"]: f for f in files}

    def test_reads_note_from_sibling_json(self) -> None:
        d = tn.MODELS_DIR
        (d / "v1.onnx").write_bytes(b"dummy")
        (d / "v1.json").write_text(json.dumps({"note": "夜間走行用、露出補正あり"}))

        result = tn.TelemetryServer._cam_models_list(None)
        files = self._by_name(result["cam_model_files"])
        self.assertEqual(files["v1"]["note"], "夜間走行用、露出補正あり")
        self.assertTrue(files["v1"]["has_config"])

    def test_missing_json_gives_empty_note(self) -> None:
        d = tn.MODELS_DIR
        (d / "v2.onnx").write_bytes(b"dummy")

        result = tn.TelemetryServer._cam_models_list(None)
        files = self._by_name(result["cam_model_files"])
        self.assertEqual(files["v2"]["note"], "")
        self.assertFalse(files["v2"]["has_config"])

    def test_corrupt_json_gives_empty_note_without_crashing(self) -> None:
        d = tn.MODELS_DIR
        (d / "v3.onnx").write_bytes(b"dummy")
        (d / "v3.json").write_text("{ not json")

        result = tn.TelemetryServer._cam_models_list(None)
        files = self._by_name(result["cam_model_files"])
        self.assertEqual(files["v3"]["note"], "")
        self.assertTrue(files["v3"]["has_config"])   # ファイルは在る（壊れているだけ）

    def test_json_without_note_field_gives_empty_note(self) -> None:
        d = tn.MODELS_DIR
        (d / "v4.onnx").write_bytes(b"dummy")
        (d / "v4.json").write_text(json.dumps({"input_size": [224, 224]}))

        result = tn.TelemetryServer._cam_models_list(None)
        files = self._by_name(result["cam_model_files"])
        self.assertEqual(files["v4"]["note"], "")


def _sample_auto_map() -> AutoMap:
    g = OccGrid(resolution=0.05, size_m=1.0)
    g.hits[:4, :4] = 5
    return AutoMap(map_seq=1, resolution=g.resolution, origin_x=g.origin[0], origin_y=g.origin[1],
                   width=g.width, height=g.height, cells=pack_trinary(g.trinary()),
                   centerline=[0.0, 0.0, 1.0, 1.0],
                   raceline=[0.0, 0.0, 1.0, 0.0, 1.0, 1.0],
                   raceline_v=[1.0, 1.0, 1.0])


def _fake_server(*, mode: str, phase: str | None, auto_map: AutoMap | None):
    """`TelemetryServer._maps_save`が使う属性(`_auto_mode`/`sub.latest`)だけを
    持つ軽量な代役。**実`Subscriber`はZMQソケットを抱えて重いので作らない**
    （このファイル冒頭docstringの方針どおり）。"""
    latest = {}
    if phase is not None:
        latest[TOPIC_AUTO_STATE] = AutoState(phase=phase)
    if auto_map is not None:
        latest[TOPIC_AUTO_MAP] = auto_map
    return SimpleNamespace(_auto_mode=mode, sub=SimpleNamespace(latest=latest))


class TestMapsSave(unittest.TestCase):
    """`_maps_save`（`saved_maps/`への保存。`raspi/auto/mapstore.py`委譲）。"""

    def setUp(self):
        self._td = tempfile.TemporaryDirectory()
        self._orig_dir = mapstore.MAPS_DIR
        mapstore.MAPS_DIR = Path(self._td.name) / "saved_maps"

    def tearDown(self):
        mapstore.MAPS_DIR = self._orig_dir
        self._td.cleanup()

    def test_rejects_empty_name(self):
        fake = _fake_server(mode="slam2d_raceline", phase="RACE", auto_map=_sample_auto_map())
        ok, err = tn.TelemetryServer._maps_save(fake, "")
        self.assertFalse(ok)
        self.assertTrue(err)

    def test_rejects_wrong_mode(self):
        fake = _fake_server(mode="follow_the_gap", phase="RACE", auto_map=_sample_auto_map())
        ok, err = tn.TelemetryServer._maps_save(fake, "course")
        self.assertFalse(ok)
        self.assertIn("slam2d_raceline", err)

    def test_rejects_phase_before_path_exists(self):
        fake = _fake_server(mode="slam2d_raceline", phase="EXPLORE", auto_map=_sample_auto_map())
        ok, err = tn.TelemetryServer._maps_save(fake, "course")
        self.assertFalse(ok)

    def test_accepts_done_phase(self):
        """`DONE`（BUILD完了直後、自動保存済みでレーシングライン走行を待つ段）
        でも手動保存（別名での取り直し）ができること。"""
        fake = _fake_server(mode="slam2d_raceline", phase="DONE", auto_map=_sample_auto_map())
        ok, err = tn.TelemetryServer._maps_save(fake, "course_done")
        self.assertTrue(ok, err)

    def test_rejects_missing_map(self):
        fake = _fake_server(mode="slam2d_raceline", phase="RACE", auto_map=None)
        ok, err = tn.TelemetryServer._maps_save(fake, "course")
        self.assertFalse(ok)

    def test_rejects_map_without_raceline(self):
        """`raceline`が空＝まだBUILD前。保存する意味が無い地図を弾く。"""
        m = _sample_auto_map()
        m.raceline = []
        fake = _fake_server(mode="slam2d_raceline", phase="RACE", auto_map=m)
        ok, err = tn.TelemetryServer._maps_save(fake, "course")
        self.assertFalse(ok)

    def test_saves_when_all_conditions_met(self):
        fake = _fake_server(mode="slam2d_raceline", phase="RACE", auto_map=_sample_auto_map())
        ok, err = tn.TelemetryServer._maps_save(fake, "course_x")
        self.assertTrue(ok, err)
        self.assertTrue((mapstore.MAPS_DIR / "course_x.npz").is_file())

    def test_maps_list_reflects_saved_map(self):
        fake = _fake_server(mode="slam2d_raceline", phase="RACE", auto_map=_sample_auto_map())
        tn.TelemetryServer._maps_save(fake, "course_y")

        result = tn.TelemetryServer._maps_list(None)
        self.assertEqual(result["type"], "maps")
        self.assertIn("course_y", [f["name"] for f in result["map_files"]])


class TestServeMapFile(unittest.TestCase):
    """`GET /maps/<name>`（`_serve_map_file`）。`_serve_log_file`と同じ形。"""

    def setUp(self):
        self._td = tempfile.TemporaryDirectory()
        self._orig_dir = mapstore.MAPS_DIR
        mapstore.MAPS_DIR = Path(self._td.name) / "saved_maps"

    def tearDown(self):
        mapstore.MAPS_DIR = self._orig_dir
        self._td.cleanup()

    def test_downloads_existing_map(self):
        mapstore.save_map(
            "course_z", resolution=0.05, origin_x=0.0, origin_y=0.0,
            trinary=np.zeros((4, 4), dtype=np.uint8), centerline_xy=np.zeros((0, 2)),
            raceline_xy=np.zeros((0, 2)), raceline_v=np.zeros(0))
        resp = tn.TelemetryServer._serve_map_file(None, "/maps/course_z")
        self.assertEqual(resp.status_code, 200)
        self.assertEqual(resp.body, (mapstore.MAPS_DIR / "course_z.npz").read_bytes())

    def test_missing_map_is_404(self):
        resp = tn.TelemetryServer._serve_map_file(None, "/maps/no_such_map")
        self.assertEqual(resp.status_code, 404)

    def test_path_traversal_is_404(self):
        resp = tn.TelemetryServer._serve_map_file(None, "/maps/..%2f..%2fetc%2fpasswd")
        self.assertEqual(resp.status_code, 404)


class TestServeMapPreview(unittest.TestCase):
    """`GET /maps/<name>/preview`（`_serve_map_preview`）。

    「レーシングライン走行」を押す前に地図を見せ、自己位置ヒントを
    クリックで指定できるようにするための一回きりのプレビュー取得
    （バンビの指示、2026-09-03）。`/ws/map`と同じ`AutoMap`のmsgpack形式で
    返すので、GUI側`ws/map.ts`の展開処理をそのまま再利用できる。
    """

    def setUp(self):
        self._td = tempfile.TemporaryDirectory()
        self._orig_dir = mapstore.MAPS_DIR
        mapstore.MAPS_DIR = Path(self._td.name) / "saved_maps"

    def tearDown(self):
        mapstore.MAPS_DIR = self._orig_dir
        self._td.cleanup()

    def test_returns_automap_msgpack(self):
        import msgspec

        mapstore.save_map(
            "course_p", resolution=0.05, origin_x=-1.0, origin_y=-2.0,
            trinary=np.zeros((6, 8), dtype=np.uint8), centerline_xy=np.zeros((0, 2)),
            raceline_xy=np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]]),
            raceline_v=np.array([1.0, 1.0, 1.0]))

        resp = tn.TelemetryServer._serve_map_preview(None, "/maps/course_p/preview")
        self.assertEqual(resp.status_code, 200)

        m = msgspec.msgpack.Decoder(tn.AutoMap).decode(resp.body)
        self.assertAlmostEqual(m.resolution, 0.05)
        self.assertAlmostEqual(m.origin_x, -1.0)
        self.assertAlmostEqual(m.origin_y, -2.0)
        self.assertEqual(m.width, 8)
        self.assertEqual(m.height, 6)
        self.assertEqual(len(m.raceline), 6)

    def test_missing_map_is_404(self):
        resp = tn.TelemetryServer._serve_map_preview(None, "/maps/no_such_map/preview")
        self.assertEqual(resp.status_code, 404)

    def test_name_with_slash_in_query_is_handled(self):
        """`?`以降にスラッシュが混ざっても`/preview`剥がしを誤らないこと。"""
        mapstore.save_map(
            "course_q", resolution=0.05, origin_x=0.0, origin_y=0.0,
            trinary=np.zeros((4, 4), dtype=np.uint8), centerline_xy=np.zeros((0, 2)),
            raceline_xy=np.zeros((0, 2)), raceline_v=np.zeros(0))
        resp = tn.TelemetryServer._serve_map_preview(None, "/maps/course_q/preview?t=1/2")
        self.assertEqual(resp.status_code, 200)


def _fake_cam_server(*, armed: bool | None, auto_engaged: bool = False, auto_mode: str = ""):
    """`_desired_front_fps`/`_desired_rear_enabled`/`_vehicle_armed`が使う属性だけを
    持つ軽量な代役。`_fake_server`（`TestMapsSave`）と同じ方針。`armed=None`は
    `VehicleState`がまだ届いていない起動直後を表す。

    `_desired_front_fps`/`_desired_rear_enabled`は内部で`self._vehicle_armed()`を
    呼ぶので、`SimpleNamespace`には本物の`_vehicle_armed`実装へ委譲するcallableを
    後付けする（判定ロジックをテスト側で重複させない）。
    """
    latest = {}
    if armed is not None:
        latest[tn.TOPIC_VEHICLE_STATE] = VehicleState(armed=armed)
    fake = SimpleNamespace(
        sub=SimpleNamespace(latest=latest),
        _auto_engaged=auto_engaged, _auto_mode=auto_mode,
        _cam_front_fps_armed=30.0, _cam_front_fps_disarm=10.0,
        _cam_rear_enabled_armed=True, _cam_rear_enabled_disarm=False)
    fake._vehicle_armed = lambda: tn.TelemetryServer._vehicle_armed(fake)
    return fake


class TestVehicleArmed(unittest.TestCase):
    """`_vehicle_armed`（ARM/DISARM連動カメラ節電の判定元。2026-09-03）。"""

    def test_true_when_armed(self):
        fake = _fake_cam_server(armed=True)
        self.assertTrue(tn.TelemetryServer._vehicle_armed(fake))

    def test_false_when_disarmed(self):
        fake = _fake_cam_server(armed=False)
        self.assertFalse(tn.TelemetryServer._vehicle_armed(fake))

    def test_defaults_true_before_first_vehicle_state(self):
        """起動直後、`VehicleState`がまだ届いていない一瞬は節電側に倒さない。"""
        fake = _fake_cam_server(armed=None)
        self.assertTrue(tn.TelemetryServer._vehicle_armed(fake))


class TestDesiredFrontFps(unittest.TestCase):
    """`_desired_front_fps`（ARM中/DISARM中で節電設定を切り替える。2026-09-03）。"""

    def test_armed_uses_armed_setting(self):
        fake = _fake_cam_server(armed=True)
        self.assertEqual(tn.TelemetryServer._desired_front_fps(fake), 30.0)

    def test_disarmed_uses_disarm_setting(self):
        fake = _fake_cam_server(armed=False)
        self.assertEqual(tn.TelemetryServer._desired_front_fps(fake), 10.0)

    def test_auto_engaged_overrides_disarm(self):
        """カメラを使う自動運転がengage中なら、DISARM中でも上限を無視する
        （実運用では自動運転はARM中にしかengageできないが、判定の優先度として
        auto_engagedがarmed判定より先に評価されることを確認する）。"""
        fake = _fake_cam_server(armed=False, auto_engaged=True, auto_mode="line_trace")
        self.assertEqual(tn.TelemetryServer._desired_front_fps(fake), tn.CAM_FPS_MAX)

    def test_auto_engaged_with_non_camera_mode_does_not_override(self):
        fake = _fake_cam_server(armed=True, auto_engaged=True, auto_mode="e2e_lidar")
        self.assertEqual(tn.TelemetryServer._desired_front_fps(fake), 30.0)


class TestDesiredRearEnabled(unittest.TestCase):
    """`_desired_rear_enabled`（DISARM中は既定で後方カメラを止める。2026-09-03）。"""

    def test_armed_uses_armed_setting(self):
        fake = _fake_cam_server(armed=True)
        self.assertTrue(tn.TelemetryServer._desired_rear_enabled(fake))

    def test_disarmed_uses_disarm_setting(self):
        fake = _fake_cam_server(armed=False)
        self.assertFalse(tn.TelemetryServer._desired_rear_enabled(fake))


class TestCameraPumpMaskClient(unittest.IsolatedAsyncioTestCase):
    """`_camera_pump`（実機で発覚: `/ws/camera/mask`にクライアントが1人でも繋がると
    `topics`（front/rearしか無い）を`camera_clients`（front/rear/maskの3種）で
    引いて`KeyError`になり、タスクごと死んでfront/rearの配信まで巻き添えで
    止まっていた。2026-09-03、実機での「前後とも映像が見れない」報告から特定）。
    """

    async def test_mask_client_does_not_crash_the_pump(self):
        fake = SimpleNamespace(
            _running=True,
            _jpeg=object(),         # `is None`判定を通過させるだけのダミー
            camera_hz=1000.0,       # 待たずに何周かループを回す
            camera_clients={"front": set(), "rear": set(), "mask": {object()}},
            sub=SimpleNamespace(latest={}),
        )

        async def stop_soon():
            await asyncio.sleep(0.02)
            fake._running = False

        stopper = asyncio.create_task(stop_soon())
        await tn.TelemetryServer._camera_pump(fake)   # KeyErrorなら例外でテスト失敗
        await stopper


if __name__ == "__main__":
    unittest.main()
