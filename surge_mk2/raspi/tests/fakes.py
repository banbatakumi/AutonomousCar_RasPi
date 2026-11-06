"""`raspi/tests` で使い回す身代わり（バス・ディスク）。複数のテストで同じものを書き写さないために置く。"""

from unittest import mock


class FakeSub:
    """`Subscriber` の身代わり。`latest` を読むだけ・`poll` は何も待たない。"""

    def __init__(self, latest=None):
        self.latest = latest or {}

    def poll(self, timeout_ms):
        return []


class FakePub:
    """`Publisher` の身代わり。送ったものを `sent` に `(topic, msg)` で溜める。"""

    def __init__(self):
        self.sent = []

    def send(self, topic, msg):
        self.sent.append((topic, msg))


def fake_disk_usage(total_gb: float, free_gb: float):
    """`shutil.disk_usage` の戻り値と同じ形（total, used, free）。"""
    total = int(total_gb * 1e9)
    free = int(free_gb * 1e9)
    return mock.Mock(total=total, used=total - free, free=free)
