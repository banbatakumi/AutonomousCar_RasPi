"""`JobRunner` のテスト。実プロセス（`sys.executable -c ...`）を軽量に起動して検証する
（`Popen`をモックすると、ワーカースレッド内のqueue配線まで検証できないため）。
"""

import sys
import tempfile
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

from ml_common.job_runner import JobRunner  # noqa: E402


def _wait_for(predicate, timeout: float = 5.0, interval: float = 0.02) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return predicate()


class TestJobRunnerSingleJob(unittest.TestCase):
    def test_start_runs_and_reports_log_and_done(self):
        with tempfile.TemporaryDirectory() as d:
            runner = JobRunner(Path(d), allow_concurrent=False)
            logs: list[str] = []
            done_codes: list[int] = []
            cmd = [sys.executable, "-c", "print('hello')"]
            ok = runner.start("job", cmd, "テスト", on_done=lambda code: done_codes.append(code))
            self.assertTrue(ok)

            def drained_and_done() -> bool:
                runner.drain(logs.append)
                return bool(done_codes)

            self.assertTrue(_wait_for(drained_and_done))

            self.assertTrue(any("hello" in t for t in logs))
            self.assertTrue(any("完了" in t for t in logs))
            self.assertEqual(done_codes, [0])
            self.assertFalse(runner.is_busy())

    def test_disallows_concurrent_start_when_busy(self):
        with tempfile.TemporaryDirectory() as d:
            runner = JobRunner(Path(d), allow_concurrent=False)
            long_cmd = [sys.executable, "-c", "import time; time.sleep(2)"]
            ok1 = runner.start("job", long_cmd, "長い処理")
            self.assertTrue(ok1)
            ok2 = runner.start("job2", [sys.executable, "-c", "print('x')"], "別処理")
            self.assertFalse(ok2)
            runner.stop("job")

    def test_disallows_same_job_key_twice(self):
        with tempfile.TemporaryDirectory() as d:
            runner = JobRunner(Path(d), allow_concurrent=True)
            long_cmd = [sys.executable, "-c", "import time; time.sleep(2)"]
            ok1 = runner.start("job", long_cmd, "長い処理")
            ok2 = runner.start("job", long_cmd, "同じキー")
            self.assertTrue(ok1)
            self.assertFalse(ok2)
            runner.stop("job")


class TestJobRunnerConcurrent(unittest.TestCase):
    def test_allows_concurrent_jobs_with_different_keys(self):
        with tempfile.TemporaryDirectory() as d:
            runner = JobRunner(Path(d), allow_concurrent=True)
            cmd = [sys.executable, "-c", "import time; time.sleep(0.3); print('done')"]
            ok1 = runner.start("a", cmd, "ジョブA")
            ok2 = runner.start("b", cmd, "ジョブB")
            self.assertTrue(ok1)
            self.assertTrue(ok2)
            self.assertEqual(set(k for k, _ in runner.active_jobs()), {"a", "b"})

            logs: list[str] = []

            def drained_and_idle() -> bool:
                runner.drain(logs.append)
                return not runner.is_busy()

            self.assertTrue(_wait_for(drained_and_idle, timeout=5.0))


class TestJobRunnerStop(unittest.TestCase):
    def test_stop_terminates_running_process(self):
        with tempfile.TemporaryDirectory() as d:
            runner = JobRunner(Path(d), allow_concurrent=True)
            cmd = [sys.executable, "-c", "import time; time.sleep(30)"]
            runner.start("job", cmd, "長時間処理")
            self.assertTrue(_wait_for(lambda: "job" in runner.running_job_keys()))
            stopped = runner.stop("job")
            self.assertTrue(stopped)

            logs: list[str] = []

            def drained_and_idle() -> bool:
                runner.drain(logs.append)
                return not runner.is_busy()

            self.assertTrue(_wait_for(drained_and_idle))

    def test_stop_unknown_job_key_returns_false(self):
        with tempfile.TemporaryDirectory() as d:
            runner = JobRunner(Path(d))
            self.assertFalse(runner.stop("nope"))


class TestJobRunnerPrefixLabels(unittest.TestCase):
    def test_prefix_labels_prepends_label_to_log_lines(self):
        with tempfile.TemporaryDirectory() as d:
            runner = JobRunner(Path(d), allow_concurrent=True, prefix_labels=True)
            cmd = [sys.executable, "-c", "print('line1')"]
            runner.start("job", cmd, "ラベル")
            logs: list[str] = []

            def drained_and_has_line1() -> bool:
                runner.drain(logs.append)
                return any("line1" in t for t in logs)

            self.assertTrue(_wait_for(drained_and_has_line1))
            matching = [t for t in logs if "line1" in t]
            self.assertTrue(any(t.startswith("[ラベル]") for t in matching))

    def test_without_prefix_labels_lines_are_untouched(self):
        with tempfile.TemporaryDirectory() as d:
            runner = JobRunner(Path(d), allow_concurrent=False, prefix_labels=False)
            cmd = [sys.executable, "-c", "print('line1')"]
            runner.start("job", cmd, "ラベル")
            logs: list[str] = []

            def drained_and_has_line1() -> bool:
                runner.drain(logs.append)
                return any("line1" in t for t in logs)

            self.assertTrue(_wait_for(drained_and_has_line1))
            matching = [t for t in logs if "line1" in t]
            self.assertTrue(all(not t.startswith("[ラベル]") for t in matching))


class TestDrainSurvivesCallbackErrors(unittest.TestCase):
    def test_exception_in_a_callback_does_not_stop_the_drain(self):
        """GUI の更新が例外を投げても drain を抜けさせない（抜けると `root.after` が再登録されず
        ログもジョブ一覧も二度と更新されない）。後ろのイベントまで処理される。"""
        import contextlib
        import io
        with tempfile.TemporaryDirectory() as d:
            runner = JobRunner(Path(d), allow_concurrent=False)
            done: list[int] = []
            runner.start("job", [sys.executable, "-c", "print('hello')"], "テスト", on_done=done.append)
            seen: list[str] = []

            def on_log(text: str) -> None:
                seen.append(text)
                raise RuntimeError("boom")

            def finished() -> bool:
                with contextlib.redirect_stderr(io.StringIO()):
                    runner.drain(on_log)       # 例外を投げずに戻ること
                return bool(done)

            self.assertTrue(_wait_for(finished))
            self.assertTrue(any("hello" in t for t in seen))


if __name__ == "__main__":
    unittest.main()
