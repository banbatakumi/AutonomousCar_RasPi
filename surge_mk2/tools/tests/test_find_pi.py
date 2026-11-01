"""`tools/find_pi.py` の、ネットワーク無しで確かめられる所（Pi 側の `mdns_unicast` と突き合わせる）。"""

from __future__ import annotations

import socket
import struct

from raspi.tools.mdns_unicast import build_answer, wants_answer
from tools.find_pi import build_query, gui_url, parse_a_records

FQDN = "surge-mk2.local"
IP = socket.inet_aton("192.168.68.77")


def test_query_is_answered_by_pi_helper():
    """組んだ問い合わせに、Pi 側の mdns_unicast が答える（同じ形式を読み書きしている）。"""
    q = build_query(FQDN, 0x1234)
    assert struct.unpack(">H", q[:2])[0] == 0x1234
    assert wants_answer(q, FQDN, IP)
    assert not wants_answer(build_query("other.local", 1), FQDN, IP)


def test_parse_pi_helper_answer():
    """mdns_unicast の応答（cache-flush ビット付きのクラス）から IP を取り出す。"""
    assert parse_a_records(build_answer(FQDN, IP), FQDN) == ["192.168.68.77"]


def test_parse_legacy_answer_with_question_echoed():
    """avahi の legacy unicast 応答は質問を写して返す。質問節を読み飛ばして答えを拾う。"""
    q = build_query(FQDN, 7)
    ans = build_answer(FQDN, IP)
    name_and_rr = ans[12:]
    pkt = (struct.pack(">HHHHHH", 7, 0x8400, 1, 1, 0, 0)
           + q[12:]                                   # 質問節
           + name_and_rr)
    assert parse_a_records(pkt, FQDN) == ["192.168.68.77"]


def test_parse_ignores_queries_other_names_and_garbage():
    assert parse_a_records(build_query(FQDN, 1), FQDN) == []           # 問い合わせ
    assert parse_a_records(build_answer("other.local", IP), FQDN) == []
    assert parse_a_records(b"\x00\x00\x84", FQDN) == []
    assert parse_a_records(build_answer(FQDN, IP)[:-2], FQDN) == []    # 尻切れ


def test_gui_url():
    assert gui_url("192.168.68.77") == "http://192.168.68.77:8000/"
