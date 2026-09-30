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


if __name__ == "__main__":
    unittest.main(verbosity=2)
