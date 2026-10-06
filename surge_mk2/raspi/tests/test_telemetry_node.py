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
import time
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


class TestDecodeCmdRejectsNonFinite(unittest.TestCase):
    """`_decode_cmd`（issue #1）。NaN/Inf は `ValueError` として呼び元へ返し、
    呼び元（`_on_control`）はそれを捨てて接続を維持する——ここで検査せずに
    `command_from_cmd` まで通しても最終防衛線が動くが（多層防御）、ここで
    早めに `bad_cmds` へ数えられた方が原因調査がしやすい。"""

    def _decode(self, **m):
        fake = SimpleNamespace(controller_name="test")
        return tn.TelemetryServer._decode_cmd(fake, m)

    def test_normal_values_pass_through(self):
        cmd = self._decode(type="cmd", speed=1.0, steer=0.2)
        self.assertEqual(cmd.target_speed, 1.0)
        self.assertEqual(cmd.target_steer, 0.2)

    def test_nan_speed_string_is_rejected(self):
        """msgspec が拒むのは JSON の NaN リテラルだけで、
        `{"speed": "nan"}` の文字列は `float()` を素通りする。"""
        with self.assertRaises(ValueError):
            self._decode(type="cmd", speed="nan")

    def test_inf_accel_limit_is_rejected(self):
        with self.assertRaises(ValueError):
            self._decode(type="cmd", accel_limit=float("inf"))


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



class TestCamE2eModelSelection(unittest.TestCase):
    """カメラE2E（`cam_e2e`）のモデル選択。一覧・選択・保存・engage の解除。

    以前は `cam_e2e/model` を publish するコードが無く、GUI から選べなかった。
    """

    def setUp(self) -> None:
        self._orig = (tn.CAM_E2E_MODELS_DIR, tn.CAM_E2E_MODEL_CONF)
        self._tmp = tempfile.TemporaryDirectory()
        root = Path(self._tmp.name)
        tn.CAM_E2E_MODELS_DIR = root / "models" / "cam_e2e"
        tn.CAM_E2E_MODELS_DIR.mkdir(parents=True)
        tn.CAM_E2E_MODEL_CONF = root / "cam_e2e_model.json"
        for name in ("v1", "v2"):
            (tn.CAM_E2E_MODELS_DIR / f"{name}.onnx").write_bytes(b"dummy")
        (tn.CAM_E2E_MODELS_DIR / "v1.json").write_text(json.dumps({"note": "屋内コース"}))
        self.addCleanup(self._cleanup)

    def _cleanup(self) -> None:
        tn.CAM_E2E_MODELS_DIR, tn.CAM_E2E_MODEL_CONF = self._orig
        self._tmp.cleanup()

    def _server(self, *, mode="cam_e2e", engaged=True):
        class Pub:
            def __init__(self):
                self.sent = []

            def send(self, topic, msg):
                self.sent.append((topic, msg))

        srv = tn.TelemetryServer.__new__(tn.TelemetryServer)
        srv.pub = Pub()
        srv._cam_e2e_model = ""
        srv._auto_engaged = engaged
        srv._auto_mode = mode
        srv.auto_ctrl_published = 0

        def _publish_auto_ctrl():
            srv.auto_ctrl_published += 1
        srv._publish_auto_ctrl = _publish_auto_ctrl
        return srv

    def test_list_uses_its_own_key_and_carries_the_note(self) -> None:
        result = tn.TelemetryServer._cam_e2e_models_list(None)
        self.assertEqual(result["type"], "cam_e2e_models")
        files = {f["name"]: f for f in result["cam_e2e_model_files"]}
        self.assertEqual(sorted(files), ["v1", "v2"])
        self.assertEqual(files["v1"]["note"], "屋内コース")
        self.assertFalse(files["v2"]["has_config"])

    def test_select_publishes_saves_and_drops_engage(self) -> None:
        srv = self._server()
        srv._on_cam_e2e_model({"name": "v1"})

        self.assertEqual(srv._cam_e2e_model, "v1")
        self.assertEqual(srv._cam_e2e_model_status(), {"name": "v1"})
        sent = [m for t, m in srv.pub.sent if t == tn.TOPIC_CAM_E2E_MODEL]
        self.assertEqual([m.name for m in sent], ["v1"])
        self.assertFalse(srv._auto_engaged, "モデルを変えたのに engage が残っている")
        self.assertEqual(srv.auto_ctrl_published, 1)
        self.assertEqual(json.loads(tn.CAM_E2E_MODEL_CONF.read_text()), {"name": "v1"})

    def test_other_modes_keep_their_engage(self) -> None:
        srv = self._server(mode="slam2d")
        srv._on_cam_e2e_model({"name": "v1"})
        self.assertTrue(srv._auto_engaged)

    def test_unknown_name_is_ignored(self) -> None:
        srv = self._server()
        srv._on_cam_e2e_model({"name": "../../etc/passwd"})
        srv._on_cam_e2e_model({"name": "no-such"})
        self.assertEqual(srv._cam_e2e_model, "")
        self.assertEqual(srv.pub.sent, [])
        self.assertTrue(srv._auto_engaged)

    def test_saved_selection_comes_back_only_if_the_file_still_exists(self) -> None:
        srv = self._server()
        tn.CAM_E2E_MODEL_CONF.write_text(json.dumps({"name": "v2"}))
        srv._load_cam_e2e_model_conf()
        self.assertEqual(srv._cam_e2e_model, "v2")

        srv._cam_e2e_model = ""
        tn.CAM_E2E_MODEL_CONF.write_text(json.dumps({"name": "deleted"}))
        srv._load_cam_e2e_model_conf()
        self.assertEqual(srv._cam_e2e_model, "")

    def test_engaging_cam_e2e_raises_the_front_camera_rate(self) -> None:
        self.assertIn("cam_e2e", tn.CAMERA_AUTO_MODES)

    def test_models_dir_matches_the_node(self) -> None:
        """一覧を出す場所と、ノードが読みに行く場所が同じであること。"""
        from raspi.nodes import cam_e2e_node
        self.assertEqual(self._orig[0], cam_e2e_node.DEFAULT_MODELS_DIR)


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

    def test_download_bundles_routes(self):
        """★ 経路の設定（`.routes.json`）を npz に同梱して返す（Pi へ上げても経由点が残る）。"""
        import io
        mapstore.save_map(
            "course_r", resolution=0.05, origin_x=0.0, origin_y=0.0,
            trinary=np.zeros((4, 4), dtype=np.uint8), centerline_xy=np.zeros((0, 2)),
            raceline_xy=np.zeros((0, 2)), raceline_v=np.zeros(0))
        mapstore.save_routes("course_r", '{"groups": {}, "active": "auto"}')
        resp = tn.TelemetryServer._serve_map_file(None, "/maps/course_r")
        self.assertEqual(resp.status_code, 200)
        with np.load(io.BytesIO(resp.body), allow_pickle=False) as z:
            self.assertIn('"auto"', str(z[mapstore.ROUTES_KEY]))

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
        _cam_rear_enabled_armed=True, _cam_rear_enabled_disarm=False,
        camera_clients={"front": set(), "rear": set(), "mask": set()},
        _mcap_proc=None, _mcap_starting=False, _logger_running=False)
    fake._vehicle_armed = lambda: tn.TelemetryServer._vehicle_armed(fake)
    fake._cam_in_use_while_disarmed = \
        lambda cam: tn.TelemetryServer._cam_in_use_while_disarmed(fake, cam)
    return fake


class TestServeSnapshot(unittest.TestCase):
    """`GET /snapshot/<cam>.png`（📷 撮影ボタン）。共有メモリの最新フレームを PNG で返す。"""

    def setUp(self):
        from raspi.bus import FrameRing
        from raspi.core.jpeg import RingJpeg

        self.name = f"surge_test_snap_{id(self)}"
        self.ring = FrameRing.create(self.name, 8, 6, "BGR888", n_slots=2)
        img = np.zeros((6, 8, 3), dtype=np.uint8)
        img[:, :, 2] = 200                       # メモリ上 BGR の R = 赤
        desc = self.ring.write(img, t_capture_ns=1, frame_id=0)
        self.jpeg = RingJpeg(70)
        ref = SimpleNamespace(shm_name=self.name, ring_seq=desc.seq,
                              t_pub=time.monotonic_ns())
        self.fake = SimpleNamespace(_jpeg=self.jpeg,
                                    sub=SimpleNamespace(latest={tn.TOPIC_IMAGE_FRONT: ref}))

    def tearDown(self):
        self.jpeg.close()
        self.ring.unlink()

    def test_returns_png_with_size_and_crop_in_filename(self):
        resp = tn.TelemetryServer._serve_snapshot(self.fake, "/snapshot/front.png")
        self.assertEqual(resp.status_code, 200)
        self.assertTrue(resp.body.startswith(b"\x89PNG"))
        name = resp.headers["X-Surge-Filename"]
        lens = tn._SAFETY.cam_front_lens
        self.assertTrue(name.startswith(f"surge_front_{lens + '_' if lens else ''}8x6_crop"), name)
        self.assertIn(name, resp.headers["Content-Disposition"])

    def test_png_keeps_colors(self):
        """メモリ上の BGR を名前どおり扱い、PNG では赤が赤のまま残る。"""
        try:
            import cv2
        except ImportError:
            self.skipTest("cv2 が無い")
        resp = tn.TelemetryServer._serve_snapshot(self.fake, "/snapshot/front.png")
        bgr = cv2.imdecode(np.frombuffer(resp.body, np.uint8), cv2.IMREAD_COLOR)[0, 0]
        self.assertEqual(tuple(int(v) for v in bgr), (0, 0, 200))

    def test_camera_without_frames_is_404(self):
        resp = tn.TelemetryServer._serve_snapshot(self.fake, "/snapshot/rear.png")
        self.assertEqual(resp.status_code, 404)

    def test_unknown_camera_is_404(self):
        resp = tn.TelemetryServer._serve_snapshot(self.fake, "/snapshot/mask.png")
        self.assertEqual(resp.status_code, 404)

    def test_no_camera_is_503(self):
        self.fake._jpeg = None
        resp = tn.TelemetryServer._serve_snapshot(self.fake, "/snapshot/front.png")
        self.assertEqual(resp.status_code, 503)

    def test_stale_frame_of_stopped_camera_is_409(self):
        """capture を止めたカメラの残り1枚を撮らない（DISARM 中の省電力停止、2026-10-04）。"""
        ref = self.fake.sub.latest[tn.TOPIC_IMAGE_FRONT]
        ref.t_pub = time.monotonic_ns() - tn.SNAPSHOT_MAX_AGE_NS - 1
        resp = tn.TelemetryServer._serve_snapshot(self.fake, "/snapshot/front.png")
        self.assertEqual(resp.status_code, 409)


class TestUndistortSetting(unittest.TestCase):
    """配信映像の魚眼補正（`camera` メッセージの `undistort`、2026-10-03）。"""

    def _fake(self):
        calls = []
        fake = SimpleNamespace(
            _cam_undistort=False, camera_hz=30.0,
            _clamp_cam_hz=tn.TelemetryServer._clamp_cam_hz,
            _save_camera_conf=lambda: calls.append("save"),
            _publish_cam_config=lambda: calls.append("pub"))
        return fake, calls

    def test_toggle_is_saved(self):
        fake, calls = self._fake()
        tn.TelemetryServer._on_camera(fake, {"type": "camera", "undistort": True})
        self.assertTrue(fake._cam_undistort)
        self.assertIn("save", calls)

    def test_non_bool_is_ignored(self):
        fake, _ = self._fake()
        tn.TelemetryServer._on_camera(fake, {"type": "camera", "undistort": "yes"})
        self.assertFalse(fake._cam_undistort)

    def test_encode_uses_undistorter_only_when_on_and_calibrated(self):
        seen = {}

        class FakeJpeg:
            def encode_latest(self, name, expect_seq=None, transform=None):
                seen["transform"] = transform
                return b"jpg", 0

        undist = object()
        fake = SimpleNamespace(_jpeg=FakeJpeg(), _cam_undistort=True,
                               _undistorters={"front": undist})
        ref = SimpleNamespace(shm_name="x", ring_seq=1)
        tn.TelemetryServer._encode_frame(fake, ref, "front")
        self.assertIs(seen["transform"], undist)
        tn.TelemetryServer._encode_frame(fake, ref, "rear")       # 未校正
        self.assertIsNone(seen["transform"])
        fake._cam_undistort = False
        tn.TelemetryServer._encode_frame(fake, ref, "front")
        self.assertIsNone(seen["transform"])


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

    def test_disarmed_viewer_does_not_override_setting(self):
        """DISARM中は GUI で後方映像を表示していても設定どおり止める（2026-10-04、バンビ判断）。"""
        fake = _fake_cam_server(armed=False)
        fake.camera_clients["rear"].add(object())
        fake._mcap_proc = object()
        self.assertFalse(tn.TelemetryServer._desired_rear_enabled(fake))


class TestDesiredFrontEnabled(unittest.TestCase):
    """`_desired_front_enabled`（DISARM中は使う者がいなければ前カメラを止める。2026-10-04）。"""

    def _front(self, fake):
        return tn.TelemetryServer._desired_front_enabled(fake)

    def test_armed_always_enabled(self):
        self.assertTrue(self._front(_fake_cam_server(armed=True)))

    def test_before_first_vehicle_state_is_enabled(self):
        self.assertTrue(self._front(_fake_cam_server(armed=None)))

    def test_disarmed_and_unused_is_stopped(self):
        self.assertFalse(self._front(_fake_cam_server(armed=False)))

    def test_disarmed_but_used_is_enabled(self):
        for attr, val in (("camera_clients", {"front": {object()}, "rear": set()}),
                          ("_mcap_proc", object()), ("_mcap_starting", True),
                          ("_logger_running", True)):
            with self.subTest(attr):
                fake = _fake_cam_server(armed=False)
                setattr(fake, attr, val)
                self.assertTrue(self._front(fake))

    def test_rear_viewer_does_not_wake_front(self):
        fake = _fake_cam_server(armed=False)
        fake.camera_clients["rear"].add(object())
        self.assertFalse(self._front(fake))


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


class TestTrackRoiSelectNeedsArm(unittest.TestCase):
    """DISARM 中の対象選択は受け付けない（2026-10-04。cam_track_node は DISARM 中は
    追跡しないので、受け付けると枠が出ないまま ARM 後に突然追跡が始まる）。"""

    def _fake(self, armed):
        calls = []
        fake = _fake_cam_server(armed=armed)
        fake._track_roi_box = (0.0, 0.0, 0.0, 0.0)
        fake._track_select_seq = 0
        fake._publish_auto_ctrl = lambda: calls.append("auto")
        fake._publish_track_roi = lambda: calls.append("roi")
        return fake, calls

    _BOX = {"x0": 0.4, "y0": 0.4, "x1": 0.6, "y1": 0.6}

    def test_disarmed_is_ignored(self):
        fake, calls = self._fake(armed=False)
        tn.TelemetryServer._on_track_roi_select(fake, dict(self._BOX))
        self.assertEqual(fake._track_select_seq, 0)
        self.assertEqual(calls, [])

    def test_armed_is_accepted(self):
        fake, calls = self._fake(armed=True)
        tn.TelemetryServer._on_track_roi_select(fake, dict(self._BOX))
        self.assertEqual(fake._track_select_seq, 1)
        self.assertEqual(fake._track_roi_box, (0.4, 0.4, 0.6, 0.6))
        self.assertIn("roi", calls)


if __name__ == "__main__":
    unittest.main()
