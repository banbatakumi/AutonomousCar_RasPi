"""`confirm_and_close` のテスト。`JobRunner`は実プロセスを起動せず、
`running_job_keys`/`terminate_all`をモックした軽量スタブで差し替えて検証する。

`confirm_and_close` がウィンドウに対して呼ぶのは `destroy()` だけなので、root も
`MagicMock` で足りる（Tk を起動しない＝ディスプレイの無い CI でも回る）。
`ml_cam`・`ml_cam_e2e` のアプリ側のテストは、ここに無いアプリ固有の
`_CONFIRM_STOP_JOB_KEYS` との配線だけを見る。
"""

import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

from ml_common.close_handler import confirm_and_close  # noqa: E402


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


class TestConfirmAndClose(unittest.TestCase):
    def setUp(self):
        self.root = MagicMock()

    def test_closes_immediately_when_nothing_confirm_worthy_is_running(self):
        jobs = _FakeJobRunner(running=["tensorboard"], labels={"tensorboard": "TB"})
        with patch("ml_common.close_handler.messagebox.askyesno") as ask:
            confirm_and_close(self.root, jobs, confirm_job_keys=frozenset({"train"}))
        ask.assert_not_called()
        self.assertTrue(jobs.terminated)
        self.root.destroy.assert_called_once()

    def test_closes_without_asking_when_no_job_is_running(self):
        jobs = _FakeJobRunner(running=[], labels={})
        with patch("ml_common.close_handler.messagebox.askyesno") as ask:
            confirm_and_close(self.root, jobs, confirm_job_keys=frozenset({"train"}))
        ask.assert_not_called()
        self.root.destroy.assert_called_once()

    def test_asks_and_closes_when_user_confirms(self):
        jobs = _FakeJobRunner(running=["train"], labels={"train": "学習(v1)"})
        with patch("ml_common.close_handler.messagebox.askyesno", return_value=True) as ask:
            confirm_and_close(self.root, jobs, confirm_job_keys=frozenset({"train"}))
        ask.assert_called_once()
        self.assertTrue(jobs.terminated)
        self.root.destroy.assert_called_once()

    def test_asks_and_keeps_open_when_user_declines(self):
        jobs = _FakeJobRunner(running=["train"], labels={"train": "学習(v1)"})
        with patch("ml_common.close_handler.messagebox.askyesno", return_value=False):
            confirm_and_close(self.root, jobs, confirm_job_keys=frozenset({"train"}))
        self.assertFalse(jobs.terminated)
        self.root.destroy.assert_not_called()

    def test_custom_confirm_message_is_used(self):
        jobs = _FakeJobRunner(running=["train"], labels={"train": "学習(v1)"})
        seen_messages = []

        def fake_askyesno(_title, message):
            seen_messages.append(message)
            return True

        with patch("ml_common.close_handler.messagebox.askyesno", side_effect=fake_askyesno):
            confirm_and_close(self.root, jobs, confirm_job_keys=frozenset({"train"}),
                              confirm_message=lambda label: f"custom: {label}")
        self.assertEqual(seen_messages, ["custom: 学習(v1)"])


if __name__ == "__main__":
    unittest.main()
