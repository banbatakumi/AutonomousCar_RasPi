"""`raspi/tools/mdns_unicast.py` のパケット判定と応答の組み立て。

    python3 -m unittest discover -s raspi/tests -t .
"""
from __future__ import annotations

import socket
import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from raspi.tools.mdns_unicast import build_answer, wants_answer  # noqa: E402

FQDN = "surge-mk2.local"
IP = socket.inet_aton("192.168.68.83")


def _name(fqdn: str) -> bytes:
    return b"".join(bytes([len(p)]) + p for p in fqdn.encode().split(b".")) + b"\0"


def _query(*questions: tuple[str, int, int], answers: bytes = b"", an: int = 0,
           flags: int = 0) -> bytes:
    body = b"".join(_name(n) + struct.pack(">HH", t, c) for n, t, c in questions)
    return struct.pack(">HHHHHH", 0, flags, len(questions), an, 0, 0) + body + answers


def _a_record(fqdn: str, ip: bytes) -> bytes:
    return _name(fqdn) + struct.pack(">HHIH", 1, 1, 120, 4) + ip


class WantsAnswerTest(unittest.TestCase):
    def test_answers_a_query_for_own_name(self):
        self.assertTrue(wants_answer(_query((FQDN, 1, 1)), FQDN, IP))

    def test_answers_when_the_unicast_response_bit_is_set(self):
        # Apple の resolver が最初に投げる形。最上位ビットはクラスではない
        self.assertTrue(wants_answer(_query((FQDN, 1, 0x8001)), FQDN, IP))

    def test_name_is_case_insensitive(self):
        self.assertTrue(wants_answer(_query(("Surge-MK2.Local", 1, 1)), FQDN, IP))

    def test_answers_any_query(self):
        self.assertTrue(wants_answer(_query((FQDN, 255, 1)), FQDN, IP))

    def test_ignores_other_names(self):
        self.assertFalse(wants_answer(_query(("other.local", 1, 1)), FQDN, IP))

    def test_ignores_aaaa_only_query(self):
        self.assertFalse(wants_answer(_query((FQDN, 28, 1)), FQDN, IP))

    def test_finds_own_name_among_several_questions(self):
        pkt = _query(("_airplay._tcp.local", 12, 1), (FQDN, 28, 1), (FQDN, 1, 0x8001))
        self.assertTrue(wants_answer(pkt, FQDN, IP))

    def test_follows_a_compression_pointer(self):
        # 2問目の名前が1問目（オフセット12）を指す。Apple は AAAA と A をこう並べる
        pkt = (struct.pack(">HHHHHH", 0, 0, 2, 0, 0, 0)
               + _name(FQDN) + struct.pack(">HH", 28, 1)
               + b"\xc0\x0c" + struct.pack(">HH", 1, 1))
        self.assertTrue(wants_answer(pkt, FQDN, IP))

    def test_ignores_responses(self):
        # avahi 自身の返事に答えると応答の応酬になる
        self.assertFalse(wants_answer(_query((FQDN, 1, 1), flags=0x8400), FQDN, IP))

    def test_suppressed_when_the_peer_already_knows_the_answer(self):
        pkt = _query((FQDN, 1, 1), answers=_a_record(FQDN, IP), an=1)
        self.assertFalse(wants_answer(pkt, FQDN, IP))

    def test_not_suppressed_by_a_stale_known_answer(self):
        # IP が変わった後。相手の持っている答えは古いので返す
        stale = _a_record(FQDN, socket.inet_aton("192.168.68.10"))
        self.assertTrue(wants_answer(_query((FQDN, 1, 1), answers=stale, an=1), FQDN, IP))

    def test_malformed_packets_are_dropped(self):
        good = _query((FQDN, 1, 1))
        for pkt in (b"", good[:11], good[:-3], good[:12] + b"\xc0\x0c" + good[14:],
                    struct.pack(">HHHHHH", 0, 0, 5, 0, 0, 0)):
            self.assertFalse(wants_answer(pkt, FQDN, IP), pkt)


class BuildAnswerTest(unittest.TestCase):
    def test_carries_one_authoritative_a_record(self):
        pkt = build_answer(FQDN, IP)
        _id, flags, qd, an, ns, ar = struct.unpack(">HHHHHH", pkt[:12])
        self.assertEqual((flags, qd, an, ns, ar), (0x8400, 0, 1, 0, 0))
        self.assertEqual(pkt[12:12 + len(_name(FQDN))], _name(FQDN))
        rtype, rclass, ttl, rdlen = struct.unpack(">HHIH", pkt[-14:-4])
        self.assertEqual((rtype, rclass & 0x7FFF, rdlen), (1, 1, 4))
        self.assertTrue(rclass & 0x8000)          # cache-flush
        self.assertGreater(ttl, 0)
        self.assertEqual(pkt[-4:], IP)

    def test_own_answer_is_not_treated_as_a_query(self):
        self.assertFalse(wants_answer(build_answer(FQDN, IP), FQDN, IP))


if __name__ == "__main__":
    unittest.main()
