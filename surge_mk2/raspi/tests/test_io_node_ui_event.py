"""io_node の `ui/event` 処理（`_on_ui_event`）の単体テスト（issue #10）。

    python3 -m unittest discover -s raspi/tests -t .
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.msgs.types import UiEvent  # noqa: E402
from raspi.nodes.io_node import IoNode  # noqa: E402
from raspi.tests.test_io_node_log import FakeLink  # noqa: E402


class TestUiEventSeqReset(unittest.TestCase):
    """`msg.seq` は Publisher（telemetry_node）がトピックごとに持つ連番で、
    telemetry_node が再起動すると 1 から振り直される。io_node 側の基準が
    古いままだと、再起動後のイベントが「既知の番号以下」として捨てられ続ける。
    """

    def _node(self) -> IoNode:
        return IoNode(FakeLink())

    def test_normal_increasing_seq_is_applied(self):
        node = self._node()
        node._on_ui_event(UiEvent(kind="abs_enable", value=True, seq=1))
        node._on_ui_event(UiEvent(kind="abs_enable", value=False, seq=2))
        self.assertEqual(node._ui_event_seq, 2)

    def test_exact_duplicate_seq_is_ignored(self):
        """同じ `seq` の再処理を防ぐ（本来の `<=` の目的）。"""
        node = self._node()
        node._on_ui_event(UiEvent(kind="abs_enable", value=True, seq=5))
        self.assertEqual(node._ui_event_seq, 5)
        node._on_ui_event(UiEvent(kind="abs_enable", value=False, seq=5))
        self.assertEqual(node._ui_event_seq, 5)

    def test_publisher_restart_seq_reset_is_recognized(self):
        """seq=5 まで進んだ後、発行元（telemetry_node）が再起動して seq=1 から
        再送されてきたら、リセットとみなして新しいイベントを取りこぼさないこと。
        """
        node = self._node()
        node._on_ui_event(UiEvent(kind="tc_enable", value=True, seq=5))
        self.assertEqual(node._ui_event_seq, 5)

        node._on_ui_event(UiEvent(kind="tc_enable", value=False, seq=1))
        self.assertEqual(node._ui_event_seq, 1,
                         "再起動後の seq=1 が『既知の番号以下』として捨てられている")

        # 続くイベントも通常どおり処理され続けること
        node._on_ui_event(UiEvent(kind="tc_enable", value=True, seq=2))
        self.assertEqual(node._ui_event_seq, 2)

    def test_first_event_after_a_restart_is_not_dropped_when_the_seq_repeats(self):
        """再起動の前に1件だけ処理していると、再起動後の最初の1件も `seq=1`。時刻が違えば別物。"""
        link = FakeLink()
        node = IoNode(link)
        node._on_ui_event(UiEvent(kind="gui_connect", seq=1, t_pub=1_000))
        before = len(link.sent)
        node._on_ui_event(UiEvent(kind="abs_enable", value=False, seq=1, t_pub=9_000))
        self.assertEqual(len(link.sent) - before, 1, "CONFIG_SET が送られていない＝捨てられた")
        # まったく同じもの（同じ seq・同じ時刻）は二重処理しない
        node._on_ui_event(UiEvent(kind="abs_enable", value=False, seq=1, t_pub=9_000))
        self.assertEqual(len(link.sent) - before, 1)


class TestQuietStream(unittest.TestCase):
    """出力先（`logs/surge-io.out`）に書けなくても、`print` で io_node を落とさない。"""

    def test_print_survives_a_full_disk(self):
        from raspi.nodes.io_node import _QuietStream

        class Full:
            encoding = "utf-8"

            def write(self, text):
                raise OSError(28, "No space left on device")

            def flush(self):
                raise OSError(28, "No space left on device")

        out = _QuietStream(Full())
        print("!! ディスク空き 0.0%", file=out, flush=True)
        self.assertEqual(out.encoding, "utf-8")


if __name__ == "__main__":
    unittest.main(verbosity=2)
