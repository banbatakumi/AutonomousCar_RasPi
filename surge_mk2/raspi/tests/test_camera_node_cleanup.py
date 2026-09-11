"""`camera_node.py` の後始末・構築失敗まわりの回帰テスト（2026-09-11 レビュー対応）。

picamera2 が無い開発機（Mac）でも通るように、`CameraWorker`/`CameraNode` の
`__init__` を素通りして `__new__` + 手動での属性設定で組み立てる
（`test_auto.py` の `TelemetryServer.__new__` と同じ手法）。
"""

from __future__ import annotations

import io
import sys
import unittest
from contextlib import redirect_stderr
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.core import cleanup  # noqa: E402
from raspi.nodes.camera_node import CameraNode, CameraWorker  # noqa: E402


class _FakeCam:
    def __init__(self, *, stop_error: Exception | None = None) -> None:
        self.stopped = False
        self.closed = False
        self._stop_error = stop_error

    def stop(self) -> None:
        if self._stop_error:
            raise self._stop_error
        self.stopped = True

    def close(self) -> None:
        self.closed = True


class _FakeRing:
    def __init__(self, name: str, *, unlink_error: Exception | None = None) -> None:
        self.name = name
        self.unlink_called = False
        self._unlink_error = unlink_error

    def unlink(self) -> None:
        self.unlink_called = True
        if self._unlink_error:
            raise self._unlink_error


class TestCameraWorkerCloseProtectsRingUnlink(unittest.TestCase):
    """`CameraWorker.close()`: `ring.unlink()` が投げても他の後始末を止めない。

    `FrameRing.unlink()` は `FileNotFoundError` 以外の `OSError`（例:
    `PermissionError`）を握りつぶさない。`ring.unlink()` を `quiet_close`
    保護外で直接呼ぶと、`CameraNode.close()` の `for w in self.workers`
    ループが1台目の例外で止まり2台目以降がリークする（レビュー指摘）。
    """

    def test_ring_unlink_oserror_is_swallowed(self):
        cam = _FakeCam()
        ring = _FakeRing("surge_cam0", unlink_error=PermissionError("busy"))
        worker = CameraWorker.__new__(CameraWorker)
        worker.cam = cam
        worker.ring = ring

        before = cleanup.failure_count()
        worker.close()                          # 例外を投げないこと

        self.assertTrue(cam.stopped)
        self.assertTrue(cam.closed)
        self.assertTrue(ring.unlink_called)
        # quiet_close に記録され、原因追跡ができること
        self.assertEqual(cleanup.failure_count(), before + 1)
        what, reason = cleanup.recent_failures()[-1][1:]
        self.assertIn("surge_cam0", what)
        self.assertIn("PermissionError", reason)

    def test_cam_stop_error_does_not_block_ring_unlink(self):
        """`cam.stop()`/`close()`側の例外も、`ring.unlink()`の実行を妨げない
        （両者は対称に別々の`quiet_close`で保護されている）。"""
        cam = _FakeCam(stop_error=OSError("device gone"))
        ring = _FakeRing("surge_cam1")
        worker = CameraWorker.__new__(CameraWorker)
        worker.cam = cam
        worker.ring = ring

        worker.close()

        self.assertTrue(ring.unlink_called)


class _FakeWorker:
    """`CameraNode.__init__`のfor-loop化を検証するための偽`CameraWorker`。

    `fail_indices`に含まれる`idx`はコンストラクタで例外を投げる
    （2台目のカメラが壊れているケースを模す）。
    """

    instances: list["_FakeWorker"] = []
    fail_indices: set[int] = set()

    def __init__(self, idx, size, fmt, fps, n_slots, on_frame=None) -> None:
        if idx in _FakeWorker.fail_indices:
            raise RuntimeError(f"cam{idx} 初期化失敗")
        self.idx = idx
        self.closed = False
        _FakeWorker.instances.append(self)

    def close(self) -> None:
        self.closed = True


class TestCameraNodeInitCleansUpOnPartialFailure(unittest.TestCase):
    """`CameraNode.__init__`: 2台目以降の構築失敗時、既に構築済みの
    ワーカーが後始末されてから例外が再送出されること（レビュー指摘）。
    """

    def setUp(self):
        _FakeWorker.instances = []
        _FakeWorker.fail_indices = set()

    def test_first_camera_is_closed_when_second_fails_to_construct(self):
        _FakeWorker.fail_indices = {1}
        with patch("raspi.nodes.camera_node.CameraWorker", _FakeWorker):
            with self.assertRaises(RuntimeError):
                CameraNode([0, 1], (640, 480), "RGB888", 30.0, 8)

        self.assertEqual(len(_FakeWorker.instances), 1)
        self.assertTrue(_FakeWorker.instances[0].closed,
                        "2台目の構築失敗時、1台目がクリーンアップされていない")

    def test_all_succeed_leaves_nothing_closed(self):
        with patch("raspi.nodes.camera_node.CameraWorker", _FakeWorker):
            node = CameraNode([0, 1], (640, 480), "RGB888", 30.0, 8)

        self.assertEqual(len(node.workers), 2)
        self.assertFalse(any(w.closed for w in node.workers))


class _FakeCfgSub:
    def __init__(self, error: Exception) -> None:
        self._error = error

    def poll(self, timeout_ms):
        raise self._error


class TestConfigLoopLogsOnPollFailure(unittest.TestCase):
    """`CameraNode._config_loop()`: `poll()`が例外を投げたら、静かに
    スレッドを終了するのではなく警告を出す（レビュー指摘）。
    """

    def test_poll_exception_prints_warning_and_returns(self):
        node = CameraNode.__new__(CameraNode)
        node.workers = []
        node._cfg_sub = _FakeCfgSub(ConnectionError("ソケットが壊れた"))
        node._cfg_running = True

        buf = io.StringIO()
        with redirect_stderr(buf):
            node._config_loop()                 # 例外を外に投げず、breakして戻ること

        err = buf.getvalue()
        self.assertIn("cam/config", err)
        self.assertIn("ConnectionError", err)


if __name__ == "__main__":
    unittest.main()
