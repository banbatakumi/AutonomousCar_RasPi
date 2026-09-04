"""`confirm_and_close` のテスト。`JobRunner`は実プロセスを起動せず、
`running_job_keys`/`terminate_all`をモックした軽量スタブで差し替えて検証する。
"""

import sys
import tkinter as tk
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

from ml_common.close_handler import confirm_and_close  # noqa: E402


def _has_display() -> bool:
    try:
        root = tk.Tk()
        root.withdraw()
        root.destroy()
        return True
    except tk.TclError:
        return False


_HAS_DISPLAY = _has_display()


class _FakeJobRunner:
    def __init__(self, running: list[str], labels: dict[str, str]):
        self._running = running
        self._labels = labels
        self.terminated = False

    def running_job_keys(self) -> list[str]:
        return self._running

    def label_of(self, key: str) -> str | None:
        return self._labels.get(key)

    def terminate_all(self) -> list[str]:
        self.terminated = True
        return list(self._running)


@unittest.skipUnless(_HAS_DISPLAY, "ディスプレイが無い環境（ヘッドレスCI等）ではスキップ")
class TestConfirmAndClose(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()

    def tearDown(self):
        try:
            self.root.destroy()
        except tk.TclError:
            pass

    def test_closes_immediately_when_nothing_confirm_worthy_is_running(self):
        jobs = _FakeJobRunner(running=["tensorboard"], labels={"tensorboard": "TB"})
        destroyed = []
        self.root.destroy = lambda: destroyed.append(True)  # type: ignore[method-assign]
        confirm_and_close(self.root, jobs, confirm_job_keys=frozenset({"train"}))
        self.assertTrue(jobs.terminated)
        self.assertTrue(destroyed)

    def test_asks_and_closes_when_user_confirms(self):
        jobs = _FakeJobRunner(running=["train"], labels={"train": "学習(v1)"})
        destroyed = []
        self.root.destroy = lambda: destroyed.append(True)  # type: ignore[method-assign]
        with patch("ml_common.close_handler.messagebox.askyesno", return_value=True):
            confirm_and_close(self.root, jobs, confirm_job_keys=frozenset({"train"}))
        self.assertTrue(jobs.terminated)
        self.assertTrue(destroyed)

    def test_asks_and_keeps_open_when_user_declines(self):
        jobs = _FakeJobRunner(running=["train"], labels={"train": "学習(v1)"})
        destroyed = []
        self.root.destroy = lambda: destroyed.append(True)  # type: ignore[method-assign]
        with patch("ml_common.close_handler.messagebox.askyesno", return_value=False):
            confirm_and_close(self.root, jobs, confirm_job_keys=frozenset({"train"}))
        self.assertFalse(jobs.terminated)
        self.assertFalse(destroyed)

    def test_custom_confirm_message_is_used(self):
        jobs = _FakeJobRunner(running=["train"], labels={"train": "学習(v1)"})
        seen_messages = []

        def fake_askyesno(_title, message):
            seen_messages.append(message)
            return True

        self.root.destroy = lambda: None  # type: ignore[method-assign]
        with patch("ml_common.close_handler.messagebox.askyesno", side_effect=fake_askyesno):
            confirm_and_close(self.root, jobs, confirm_job_keys=frozenset({"train"}),
                              confirm_message=lambda label: f"custom: {label}")
        self.assertEqual(seen_messages, ["custom: 学習(v1)"])


if __name__ == "__main__":
    unittest.main()
