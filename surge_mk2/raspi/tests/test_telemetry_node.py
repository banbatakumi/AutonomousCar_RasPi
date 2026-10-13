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
        self.assertIn("経路選択走行", err)
        # 地図を作る planner は 2 つとも保存できる（GUI に出るのは `slam2d_route` だけ）
        fake = _fake_server(mode="slam2d_route", phase="RACE", auto_map=_sample_auto_map())
        self.assertTrue(tn.TelemetryServer._maps_save(fake, "course")[0])

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
            _cam_wake=asyncio.Event(),
        )

        async def stop_soon():
            await asyncio.sleep(0.02)
            fake._running = False

        stopper = asyncio.create_task(stop_soon())
        await tn.TelemetryServer._camera_pump(fake)   # KeyErrorなら例外でテスト失敗
        await stopper


class TestCameraPumpOnArrival(unittest.IsolatedAsyncioTestCase):
    """`_camera_pump` はフレームが届いた周に送る（2026-10-12。刻みで見に行くと、撮ってから
    送るまでに配信周期ぶんの待ちが乗っていた）。`gui_hz` は送る頻度の上限として残る。"""

    def _fake(self, hz):
        sent = []

        async def send_or_drop(ws, payload, clients):
            sent.append((time.monotonic(), payload))
            return True

        fake = SimpleNamespace(
            _running=True, _jpeg=object(), camera_hz=hz,
            camera_clients={"front": {object()}, "rear": set(), "mask": set()},
            sub=SimpleNamespace(latest={}), _cam_wake=asyncio.Event(),
            _encode_frame=lambda ref, cam="": b"jpg%d" % ref.ring_seq,
            _send_or_drop=send_or_drop)
        return fake, sent

    @staticmethod
    def _frame(fake, seq):
        fake.sub.latest[tn.TOPIC_IMAGE_FRONT] = SimpleNamespace(ring_seq=seq)
        fake._cam_wake.set()

    async def _stop(self, fake, pump):
        fake._running = False
        fake._cam_wake.set()
        await asyncio.wait_for(pump, 1.0)

    async def test_frame_is_sent_without_waiting_for_the_period(self):
        fake, sent = self._fake(hz=2.0)                # 周期 500ms
        pump = asyncio.create_task(tn.TelemetryServer._camera_pump(fake))
        await asyncio.sleep(0.05)
        t0 = time.monotonic()
        self._frame(fake, 1)
        await asyncio.sleep(0.05)
        await self._stop(fake, pump)
        self.assertEqual([p for _, p in sent], [b"jpg1"])
        self.assertLess(sent[0][0] - t0, 0.03)

    async def test_gui_hz_caps_the_rate_and_sends_the_newest(self):
        fake, sent = self._fake(hz=10.0)               # 周期 100ms
        pump = asyncio.create_task(tn.TelemetryServer._camera_pump(fake))
        await asyncio.sleep(0.01)
        self._frame(fake, 1)
        await asyncio.sleep(0.03)
        self._frame(fake, 2)                           # 上限に当たる
        await asyncio.sleep(0.03)
        self._frame(fake, 3)                           # 同上。送れる時刻には 3 が最新
        await asyncio.sleep(0.12)
        await self._stop(fake, pump)
        self.assertEqual([p for _, p in sent], [b"jpg1", b"jpg3"])
        self.assertGreater(sent[1][0] - sent[0][0], 0.08)

    async def test_same_rate_frames_are_not_delayed_by_jitter(self):
        """カメラと `gui_hz` が同じ fps のとき、少し早く届いたフレームを次の刻みまで待たせない。"""
        fake, sent = self._fake(hz=10.0)
        pump = asyncio.create_task(tn.TelemetryServer._camera_pump(fake))
        await asyncio.sleep(0.01)
        self._frame(fake, 1)
        await asyncio.sleep(0.088)                     # 周期より 12ms 早い
        t1 = time.monotonic()
        self._frame(fake, 2)
        await asyncio.sleep(0.03)
        await self._stop(fake, pump)
        self.assertEqual([p for _, p in sent], [b"jpg1", b"jpg2"])
        self.assertLess(sent[1][0] - t1, 0.008)


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


class TestSnapshotSendsSlowPartsOnlyOnChange(unittest.TestCase):
    """10Hz 以下でしか変わらないもの（診断・planner の判断・点群など）は、変わったときだけ
    `/ws/telemetry` に載せる（2026-10-13）。50Hz で同じ物を送り直していた頃は、実機の
    1フレーム 4.4kB のうち 3.3kB がそれだった。"""

    def _server(self):
        import msgspec

        from raspi.msgs.types import (TOPIC_AUTO_STATE, TOPIC_DIAG_LINK, TOPIC_VEHICLE_STATE,
                                      AutoState, LinkDiag, VehicleState)

        srv = tn.TelemetryServer.__new__(tn.TelemetryServer)
        srv._snapshot_sent = {}
        srv.controller = None
        srv.controller_name = ""
        link = LinkDiag(health="OK")
        link.seq, link.t_pub = 1, 100
        auto = AutoState()
        auto.seq, auto.t_pub = 1, 100
        vs = VehicleState(speed=1.0)
        vs.seq, vs.t_pub = 1, 100
        srv.sub = type("S", (), {"latest": {
            TOPIC_DIAG_LINK: link, TOPIC_AUTO_STATE: auto, TOPIC_VEHICLE_STATE: vs}})()
        return srv, link, auto, lambda: msgspec.msgpack.decode(srv._snapshot())

    def test_same_version_is_sent_once(self):
        srv, link, auto, snap = self._server()
        first, second = snap(), snap()
        self.assertEqual(first["link"]["health"], "OK")
        self.assertIsNotNone(first["auto"])
        self.assertIsNone(second["link"])
        self.assertIsNone(second["auto"])
        # 車両状態は毎回載せる（計器はこれで動く）
        self.assertEqual(second["vs"]["speed"], 1.0)

    def test_new_version_is_sent_again(self):
        srv, link, auto, snap = self._server()
        snap()
        link.seq, link.t_pub = 2, 200
        s = snap()
        self.assertIsNotNone(s["link"])
        self.assertIsNone(s["auto"])

    def test_publisher_restart_with_same_seq_is_still_new(self):
        """発行元が再起動すると `seq` は 1 から数え直す。`t_pub` も見ているので取りこぼさない。"""
        srv, link, auto, snap = self._server()
        snap()
        link.t_pub = 999                                   # seq は 1 のまま
        self.assertIsNotNone(snap()["link"])

    def test_new_viewer_gets_everything_once(self):
        """後から繋いだ GUI にも、次のフレームで全部を1回載せる（`_telemetry_channel`）。"""
        import asyncio

        srv, link, auto, snap = self._server()
        srv.telemetry_clients = set()
        snap()
        self.assertIsNone(snap()["link"])

        class _Ws:
            async def wait_closed(self):
                return None

        asyncio.run(srv._telemetry_channel(_Ws()))
        s = snap()
        self.assertIsNotNone(s["link"])
        self.assertIsNotNone(s["auto"])


# ── 止める側の安全条件（`/ws/control` と送信） ──────────────────────────────

import raspi.tests.test_cmd_path as _cp  # noqa: E402  （`TelemetryServer` の組み立てを借りる）


class _StuckWs:
    """受信が止まった相手。`send()` が戻らない。"""

    def __init__(self):
        self.aborted = False
        self.transport = SimpleNamespace(abort=self._abort)

    def _abort(self):
        self.aborted = True

    async def send(self, payload):
        await asyncio.Event().wait()


class _GoodWs:
    def __init__(self):
        self.got = []

    async def send(self, payload):
        self.got.append(payload)


class TestStalledClient(unittest.IsolatedAsyncioTestCase):
    """受信が止まった1台が、他のクライアントと操縦者の操作を止めないこと。"""

    def setUp(self):
        self._orig = tn.SEND_TIMEOUT_S
        tn.SEND_TIMEOUT_S = 0.05
        self.addCleanup(setattr, tn, "SEND_TIMEOUT_S", self._orig)

    async def test_send_gives_up_and_cuts_the_stuck_client(self):
        stuck, good = _StuckWs(), _GoodWs()
        clients = {stuck, good}
        t0 = time.monotonic()
        res = await asyncio.gather(*(tn.TelemetryServer._send_or_drop(ws, b"x", clients)
                                     for ws in (stuck, good)))
        self.assertLess(time.monotonic() - t0, 0.5)
        self.assertEqual(res, [False, True])
        self.assertEqual(clients, {good})
        self.assertTrue(stuck.aborted)
        self.assertEqual(good.got, [b"x"])

    async def test_status_broadcast_reaches_the_others(self):
        srv = _cp.TestControlOwnership._server(self)
        del srv._send_json                              # 本物の送信を使う
        stuck, good = _StuckWs(), _GoodWs()
        srv.control_clients = {stuck, good}
        await asyncio.wait_for(srv._broadcast_control_status(), 0.5)
        self.assertEqual(len(good.got), 1)
        self.assertTrue(stuck.aborted)

    async def test_control_handler_does_not_wait_for_the_broadcast(self):
        """status の配信が終わらなくても、操縦者の次のメッセージ（E-Stop）は処理される。"""
        srv = _cp.TestControlOwnership._server(self)
        started = asyncio.Event()

        async def _never():
            started.set()
            await asyncio.Event().wait()
        srv._broadcast_control_status = _never
        a = object()
        await asyncio.wait_for(srv._on_control(a, b'{"type":"take_control","name":"A"}'), 0.2)
        await asyncio.wait_for(srv._on_control(a, b'{"type":"cmd","mode":1,"arm":true,"speed":0.5}'), 0.2)
        self.assertTrue(srv.published[-1][1].arm)
        await asyncio.wait_for(srv._on_control(a, b'{"type":"estop"}'), 0.2)
        self.assertEqual(srv.published[-1][1].mode, 0)
        await asyncio.wait_for(started.wait(), 0.2)     # 配信そのものは裏で始まっている
        for t in list(srv._bg_tasks):
            t.cancel()


class TestEstopHold(unittest.IsolatedAsyncioTestCase):
    """GUI の E-Stop のあとは、操縦者が ARM を入れ直すまで ARM を通さない。"""

    async def test_arm_is_dropped_until_the_operator_sends_arm_false(self):
        srv = _cp.TestControlOwnership._server(self)
        a = object()
        send = lambda **m: srv._on_control(a, json.dumps(m))   # noqa: E731
        await send(type="take_control", name="A")
        await send(type="cmd", mode=1, arm=True, speed=0.5)
        self.assertTrue(srv.published[-1][1].arm)
        await send(type="estop")
        # ARM を保持したままの GUI が操縦権を取り直しても走らない
        await send(type="take_control", name="A")
        await send(type="cmd", mode=1, arm=True, speed=0.5, light_mode=2)
        m = srv.published[-1][1]
        self.assertEqual((m.arm, m.target_speed, m.source), (False, 0.0, "estop_hold"))
        self.assertEqual(m.light_mode, 2)               # 灯火は通す
        srv._publish_cmd(time.monotonic_ns())           # 定期送信でも同じ
        self.assertFalse(srv.published[-1][1].arm)
        # ARM を外してから入れ直せば通る
        await send(type="cmd", mode=0, arm=False)
        await send(type="cmd", mode=1, arm=True, speed=0.5)
        m = srv.published[-1][1]
        self.assertEqual((m.arm, m.target_speed), (True, 0.5))

    async def test_stm32_estop_drops_engage(self):
        from raspi.msgs.types import TOPIC_DIAG_LINK
        srv = _cp.TestControlOwnership._server(self)
        sent = []
        srv._publish_auto_ctrl = lambda: sent.append(srv._auto_engaged)
        srv._auto_mode, srv._auto_engaged = "ftg", True
        srv.sub.latest[TOPIC_DIAG_LINK] = SimpleNamespace(estop_active=False)
        srv._on_estop_change()
        self.assertTrue(srv._auto_engaged)
        srv.sub.latest[TOPIC_DIAG_LINK] = SimpleNamespace(estop_active=True)
        srv._on_estop_change()
        self.assertFalse(srv._auto_engaged)
        self.assertEqual(sent, [False])
        # 立ち上がりだけ。E-Stop が立ったままでも、後から人が engage したものは落とさない
        srv._auto_engaged = True
        srv._on_estop_change()
        self.assertTrue(srv._auto_engaged)
        for t in list(srv.__dict__.get("_bg_tasks", ())):
            t.cancel()


class TestControlInputGuards(unittest.IsolatedAsyncioTestCase):
    """壊れた入力で制御チャネルが落ちないこと、操縦者以外が走行中の自律を変えられないこと。"""

    async def test_out_of_range_numbers_are_dropped_not_raised(self):
        srv = _cp.TestControlOwnership._server(self)
        a = object()
        await srv._on_control(a, b'{"type":"take_control","name":"A"}')
        for body in ('"speed":1' + "0" * 400, '"mode":1e30', '"mode":7',
                     '"light_mode":1180591620717411303424', '"steer":[1]'):
            before = srv.bad_cmds
            await srv._on_control(a, ('{"type":"cmd","arm":true,' + body + "}").encode())
            self.assertEqual(srv.bad_cmds, before + 1, body)
        self.assertIsNone(srv._last_cmd)
        self.assertIs(srv.controller, a)                # 操縦権は持ったまま

    async def test_unexpected_exception_is_counted_not_raised(self):
        srv = _cp.TestControlOwnership._server(self)

        def _boom(m):
            raise RuntimeError("boom")
        srv._on_fan = _boom
        await srv._on_control(object(), b'{"type":"fan","mode":"auto"}')
        self.assertEqual(srv.bad_cmds, 1)

    async def test_non_ascii_token_is_just_rejected(self):
        srv = _cp.TestControlOwnership._server(self)
        srv.token = "secret"
        a = object()
        await srv._on_control(a, '{"type":"take_control","token":"あいう"}'.encode())
        self.assertIsNone(srv.controller)
        self.assertEqual((srv.auth_rejects, srv.bad_cmds), (1, 0))
        await srv._on_control(a, b'{"type":"take_control","token":"secret"}')
        self.assertIs(srv.controller, a)

    async def test_only_the_operator_changes_an_engaged_run(self):
        srv = _cp.TestControlOwnership._server(self)
        a, b = object(), object()
        await srv._on_control(a, b'{"type":"take_control","name":"A"}')
        await srv._on_control(a, b'{"type":"auto","mode":"ftg","engaged":true}')
        self.assertTrue(srv._auto_engaged)
        params = dict(srv._auto_params)
        key = next(iter(params))
        await srv._on_control(b, json.dumps({"type": "auto", "params": {key: 0.0},
                                             "park_target": True, "park_x": 1, "park_y": 1,
                                             "park_yaw": 0, "clear_map": True}).encode())
        self.assertEqual(srv._auto_params, params)
        self.assertTrue(srv._auto_engaged)
        self.assertTrue(any(o.get("reason") == "auto_busy" for w, o in srv.sent if w is b))
        # 解除は誰でもできる（一緒に送ってきた他の項目は捨てる）
        await srv._on_control(b, json.dumps({"type": "auto", "engaged": False,
                                             "params": {key: 0.0}}).encode())
        self.assertFalse(srv._auto_engaged)
        self.assertEqual(srv._auto_params, params)
        # engage していなければ従来どおり誰でも触れる
        await srv._on_control(b, b'{"type":"auto","mode":""}')
        self.assertEqual(srv._auto_mode, "")
        for t in list(srv.__dict__.get("_bg_tasks", ())):
            t.cancel()

    async def test_shutdown_is_refused_while_armed(self):
        from raspi.msgs.types import TOPIC_VEHICLE_STATE
        srv = _cp.TestControlOwnership._server(self)
        calls = []

        async def _shutdown():
            calls.append(1)
        srv._shutdown_pi = _shutdown
        srv.sub.latest[TOPIC_VEHICLE_STATE] = VehicleState(armed=True)
        await srv._on_control(object(), b'{"type":"shutdown"}')
        self.assertEqual(calls, [])
        self.assertTrue(any(o.get("type") == "shutdown_denied" for _, o in srv.sent))
        srv.sub.latest[TOPIC_VEHICLE_STATE] = VehicleState(armed=False)
        await srv._on_control(object(), b'{"type":"shutdown"}')
        self.assertEqual(calls, [1])


class TestPumpSupervision(unittest.IsolatedAsyncioTestCase):
    """定期タスクが例外で終わったら、サーバを止めて異常終了にする（systemd が上げ直す）。"""

    async def _stop_after(self, coro):
        stop = asyncio.get_running_loop().create_future()
        t = asyncio.create_task(coro, name="_bus_pump")
        t.add_done_callback(lambda t: tn.TelemetryServer._on_pump_done(t, stop))
        await asyncio.sleep(0.01)
        return stop, t

    async def test_exception_stops_the_server(self):
        async def _boom():
            raise ValueError("bad")
        stop, _ = await self._stop_after(_boom())
        self.assertTrue(stop.done())
        with self.assertRaisesRegex(RuntimeError, "_bus_pump.*ValueError"):
            stop.result()

    async def test_normal_return_and_cancel_do_not(self):
        async def _ok():
            return None
        stop, _ = await self._stop_after(_ok())
        self.assertFalse(stop.done())
        stop, t = await self._stop_after(asyncio.sleep(10))
        t.cancel()
        await asyncio.sleep(0.01)
        self.assertFalse(stop.done())


class TestLogsDeleteGuard(unittest.TestCase):
    """`logs_delete` は記録中のファイルを消さない（消すと書き手は消えた実体へ書き続ける）。"""

    def setUp(self):
        self._td = tempfile.TemporaryDirectory()
        self.addCleanup(self._td.cleanup)
        self.srv = tn.TelemetryServer.__new__(tn.TelemetryServer)
        self.srv.logs_dir = Path(self._td.name)
        self.srv._sfl_active = False

    def _touch(self, name, age_s):
        import os
        p = self.srv.logs_dir / name
        p.write_bytes(b"x")
        t = time.time() - age_s
        os.utime(p, (t, t))
        return p

    def test_old_file_is_deleted(self):
        p = self._touch("old.mcap", 600)
        self.assertEqual(self.srv._logs_delete("old.mcap"), "")
        self.assertFalse(p.exists())

    def test_file_being_written_is_kept(self):
        p = self._touch("now.mcap", 1)
        self.assertIn("記録中", self.srv._logs_delete("now.mcap"))
        self.assertTrue(p.exists())

    def test_newest_sfl_is_kept_while_sfl_recording_is_requested(self):
        """flush の間が空いて mtime が古く見えても、記録を頼んでいる間の最新の `.sfl` は消さない。"""
        old = self._touch("a.sfl", 900)
        cur = self._touch("b.sfl", 120)
        self.srv._sfl_active = True
        self.assertIn("記録中", self.srv._logs_delete("b.sfl"))
        self.assertEqual(self.srv._logs_delete("a.sfl"), "")
        self.assertTrue(cur.exists())
        self.assertFalse(old.exists())
        self.srv._sfl_active = False
        self.assertEqual(self.srv._logs_delete("b.sfl"), "")

    def test_path_escape_is_ignored(self):
        self.assertEqual(self.srv._logs_delete("../x"), "")


class TestConfLoadersTolerateNonDict(unittest.TestCase):
    """設定ファイルが dict 以外の JSON（`[]`・`null`）でも起動を落とさない。"""

    def test_all_loaders(self):
        from unittest import mock
        srv = tn.TelemetryServer.__new__(tn.TelemetryServer)
        with tempfile.TemporaryDirectory() as td:
            for const, loader in (("CAMERA_CONF", "_load_camera_conf"),
                                  ("SIGNAL_CONF", "_load_signal_conf"),
                                  ("CAM_MODEL_CONF", "_load_cam_model_conf"),
                                  ("E2E_MODEL_CONF", "_load_e2e_model_conf"),
                                  ("CAM_E2E_MODEL_CONF", "_load_cam_e2e_model_conf"),
                                  ("AUTO_CONF", "_load_auto_conf"),
                                  ("DRIVE_SETTINGS_CONF", "_load_drive_settings")):
                for text in ("[]", "null", "3"):
                    f = Path(td) / f"{const}.json"
                    f.write_text(text)
                    with mock.patch.object(tn, const, f):
                        getattr(srv, loader)()          # 例外にならなければよい


class TestAutoConfDeferredSave(unittest.IsolatedAsyncioTestCase):
    """スライダの連打で `auto.json` を毎回は書かない。最後の値を1回、終了時は待たずに書く。"""

    def _server(self):
        srv = _cp.TestControlOwnership._server(self)
        self.saves = []
        srv._save_auto_conf = lambda: self.saves.append(dict(srv._auto_params))
        return srv

    async def test_burst_is_written_once_with_the_last_value(self):
        from unittest import mock
        srv = self._server()
        srv._on_auto({"mode": "ftg"})
        key = next(iter(srv._auto_params))
        with mock.patch.object(tn, "AUTO_CONF_SAVE_DELAY_S", 0.03):
            srv._auto_conf_timer.cancel()
            srv._auto_conf_timer = None
            for _ in range(20):
                srv._on_auto({"params": {key: srv._auto_params[key]}})
            self.assertEqual(self.saves, [])
            await asyncio.sleep(0.08)
        self.assertEqual(len(self.saves), 1)
        self.assertEqual(self.saves[0], srv._auto_params)

    async def test_close_flushes_a_pending_save(self):
        srv = self._server()
        srv._on_auto({"mode": "ftg"})
        self.assertEqual(self.saves, [])
        srv._fan = SimpleNamespace(set_auto=lambda: None)
        srv._cpufreq = SimpleNamespace(restore=lambda: None)
        srv.sub.close = lambda: None
        srv.pub.close = lambda: None
        srv.close()
        self.assertEqual(len(self.saves), 1)
        srv._auto_conf_timer.cancel()


class TestMcapRestartWaitsForThePreviousRelay(unittest.IsolatedAsyncioTestCase):
    async def test_start_is_refused_while_the_old_pump_is_draining(self):
        srv = _cp.TestControlOwnership._server(self)
        srv._mcap_starting = False
        srv._mcap_pump_task = asyncio.create_task(asyncio.sleep(10))
        await srv._mcap_start(image_hz=5.0)
        self.assertIsNone(srv._mcap_proc)
        self.assertIn("終了処理中", srv._mcap_error)
        srv._mcap_pump_task.cancel()
