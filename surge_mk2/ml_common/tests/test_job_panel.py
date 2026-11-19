"""`JobPanelBase` のテスト。ウィンドウは作らず、ログの書き先と root を軽いフェイクにして、
ジョブの完了後の処理がコールバックで届く順番だけを見る。"""

import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

from ml_common.job_panel import JobPanelBase  # noqa: E402


class _Sink:
    def __init__(self):
        self.text = ""

    def append(self, text):
        self.text += text


class _Panel(JobPanelBase):
    def __init__(self, cwd, **kw):
        self._init_job_panel(object(), cwd, **kw)
        self.log_sink = _Sink()
        self.events = []

    def _after_job(self, ok):
        self.events.append(("after", ok))

    def _on_jobs_changed(self):
        self.events.append("changed")


def _drain_until(panel, pred, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        panel.jobs.drain(panel._append_log)
        if pred():
            return True
        time.sleep(0.02)
    return False


class TestJobPanel(unittest.TestCase):
    def test_on_done_gets_success_after_the_common_hook(self):
        with tempfile.TemporaryDirectory() as d:
            p = _Panel(Path(d))
            p._run("train", [sys.executable, "-c", "print('hi')"], "学習",
                   on_done=lambda ok: p.events.append(("done", ok)))
            self.assertTrue(_drain_until(p, lambda: ("done", True) in p.events))
            self.assertEqual(p.events[-3:], [("after", True), ("done", True), "changed"])
            self.assertIn("hi", p.log_sink.text)

    def test_failure_is_reported_as_false(self):
        with tempfile.TemporaryDirectory() as d:
            p = _Panel(Path(d))
            got = []
            p._run("x", [sys.executable, "-c", "raise SystemExit(3)"], "失敗", on_done=got.append)
            self.assertTrue(_drain_until(p, lambda: got))
            self.assertEqual(got, [False])

    def test_single_job_mode_refuses_a_second_job(self):
        with tempfile.TemporaryDirectory() as d:
            p = _Panel(Path(d))
            p._run("a", [sys.executable, "-c", "import time; time.sleep(0.5)"], "A")
            with patch("ml_common.job_panel.messagebox.showwarning") as warn:
                p._run("b", [sys.executable, "-c", "pass"], "B")
            warn.assert_called_once()
            self.assertEqual([k for k, _ in p.jobs.active_jobs()], ["a"])
            p.jobs.terminate_all()
            _drain_until(p, lambda: not p.jobs.is_busy())

    def test_note_requires_a_model_name(self):
        with tempfile.TemporaryDirectory() as d:
            p = _Panel(Path(d))
            p._current_model_dir = lambda: None
            with patch("ml_common.job_panel.messagebox.showerror") as err:
                p._save_note()
            err.assert_called_once()


if __name__ == "__main__":
    unittest.main()
