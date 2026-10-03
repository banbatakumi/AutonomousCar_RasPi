"""`<ホスト名>.local` の問い合わせに**ユニキャストで**答える、avahi の補助。

## なぜ要るか（2026-10-04 に実測）

Wi-Fi だけで繋いだ端末（iPad・iPhone・LAN ケーブルを抜いた Mac）から
`http://surge-mk2.local:8000/` が開けなかった。Safari は青いバーが最初で止まったまま。

- IP 直打ちなら Wi-Fi 同士でも GUI は全部届く（経路は生きている）
- Wi-Fi 端末の問い合わせ（マルチキャスト）は Pi に届いている
- avahi の返事（マルチキャスト）が **Wi-Fi 端末にだけ届かない**（有線の端末には届く）。
  アクセスポイントが無線端末宛てのマルチキャストを中継していない

つまり名前が引けないだけ。ルーターの設定で直る話だが、車は持ち出す先の Wi-Fi を
選べないので、Pi 側で「返事を問い合わせ元に直接送る」ことで塞ぐ。

## 何をするか

5353/udp のマルチキャストを avahi と並んで聞き、自分のホスト名の A レコードを
問われたら、**問い合わせ元の IP:ポートへユニキャストで** 答える。それだけ。

- avahi は止めない・設定も変えない（サービス広告・AAAA・競合検出は avahi の仕事のまま）
- 返すのは A レコードだけ。IPv6 リンクローカルは Wi-Fi 同士で届かないことを
  同じ日に確認しているので、ここでは広告しない
- 既に正しい答えを持っている相手（known-answer）には返さない

## 受け取ってもらえる条件

mDNS の受信側は、なりすまし対策として**頼んでいないユニキャスト応答を捨てる**。
Apple の resolver は名前を初めて引くとき QU（ユニキャスト応答可）で問い合わせるので、
その返事として受理される。QU でない問い合わせにも同じく返すが、捨てられても害は無い。

    python -m raspi.tools.mdns_unicast            # 自分のホスト名で
    python -m raspi.tools.mdns_unicast --name foo # foo.local として（試験用）
"""
from __future__ import annotations

import argparse
import select
import socket
import struct
import time

MDNS_GROUP = "224.0.0.251"
MDNS_PORT = 5353
TYPE_A = 1
TYPE_ANY = 255
CLASS_IN = 1
#: avahi がホスト名の A レコードに付けている値に合わせる
TTL_S = 120
#: インタフェースの出入り（Wi-Fi の再接続）に追従してグループへ入り直す間隔
REJOIN_PERIOD_S = 10.0


def _read_name(pkt: bytes, off: int) -> tuple[str, int]:
    """圧縮ポインタを辿って名前を読む。戻り値は（小文字の名前, 次の位置）。"""
    labels: list[str] = []
    end = -1                      # 最初のポインタの直後＝呼び出し側が続きを読む位置
    hops = 0
    while True:
        n = pkt[off]
        if n & 0xC0 == 0xC0:
            if end < 0:
                end = off + 2
            off = ((n & 0x3F) << 8) | pkt[off + 1]
            hops += 1
            if hops > 16:         # ポインタの輪で回り続けない
                raise ValueError("compression loop")
            continue
        if n & 0xC0:
            raise ValueError("bad label")
        off += 1
        if n == 0:
            break
        labels.append(pkt[off:off + n].decode("ascii", "replace").lower())
        off += n
    return ".".join(labels), (end if end >= 0 else off)


def wants_answer(pkt: bytes, fqdn: str, ip: bytes) -> bool:
    """この問い合わせに `fqdn` の A レコードを返すべきか。

    返すのは「問い合わせであり、`fqdn` の A（または ANY）を問うていて、相手が
    まだ正しい答えを持っていない」ときだけ。壊れたパケットは黙って捨てる。
    """
    try:
        if len(pkt) < 12:
            return False
        flags, qd, an = struct.unpack(">HHH", pkt[2:8])
        if flags & 0x8000:                    # 応答（avahi や他の機器の返事）
            return False
        off, asked = 12, False
        for _ in range(qd):
            name, off = _read_name(pkt, off)
            qtype, qclass = struct.unpack(">HH", pkt[off:off + 4])
            off += 4
            # 最上位ビットは QU（ユニキャスト応答可）の印でクラスではない
            if name == fqdn and qtype in (TYPE_A, TYPE_ANY) and qclass & 0x7FFF == CLASS_IN:
                asked = True
        if not asked:
            return False
        for _ in range(an):                   # known-answer: 相手が既に持っている答え
            name, off = _read_name(pkt, off)
            rtype, _rclass, _ttl, rdlen = struct.unpack(">HHIH", pkt[off:off + 10])
            off += 10
            if name == fqdn and rtype == TYPE_A and pkt[off:off + rdlen] == ip:
                return False
            off += rdlen
        return True
    except (IndexError, struct.error, ValueError):
        return False


def build_answer(fqdn: str, ip: bytes) -> bytes:
    """`fqdn` の A レコード1件だけを載せた mDNS 応答。"""
    name = b"".join(bytes([len(p)]) + p for p in fqdn.encode("ascii").split(b".")) + b"\0"
    return (struct.pack(">HHHHHH", 0, 0x8400, 0, 1, 0, 0) + name
            # クラスの最上位ビット＝cache-flush（このホスト名の持ち主は自分だけ）
            + struct.pack(">HHIH", TYPE_A, 0x8000 | CLASS_IN, TTL_S, 4) + ip)


def _source_ip_toward(peer: str) -> bytes | None:
    """`peer` へ送るときに使われる自分の IPv4。インタフェースごとに違うので毎回引く。"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((peer, MDNS_PORT))          # UDP の connect は経路を引くだけで何も送らない
        return socket.inet_aton(s.getsockname()[0])
    except OSError:
        return None
    finally:
        s.close()


def _join_all(sock: socket.socket) -> None:
    """ループバック以外の全インタフェースで mDNS グループに入る（入り済みは無視）。"""
    for index, ifname in socket.if_nameindex():
        if ifname == "lo":
            continue
        mreqn = struct.pack("4s4si", socket.inet_aton(MDNS_GROUP), b"\0" * 4, index)
        try:
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreqn)
        except OSError:
            pass      # 既に入っている（EADDRINUSE）か、アドレス未取得。次の周回でまた試す


def serve(fqdn: str) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # **グループのアドレスに bind する。** 0.0.0.0:5353 に bind すると、5353 宛ての
    # ユニキャスト（avahi 自身の問い合わせへの返事）をこちらが横取りしてしまう
    sock.bind((MDNS_GROUP, MDNS_PORT))
    answered = 0
    next_join = 0.0
    next_report = time.monotonic() + 60.0
    print(f"# mdns_unicast: {fqdn} の A レコードをユニキャストで返す", flush=True)
    while True:
        now = time.monotonic()
        if now >= next_join:
            _join_all(sock)
            next_join = now + REJOIN_PERIOD_S
        if now >= next_report:
            # 1時間に1行。「動いているのに引けない」ときに問い合わせが来ているかを見る
            print(f"# mdns_unicast: 応答 {answered} 回", flush=True)
            next_report = now + 3600.0
        ready, _, _ = select.select([sock], [], [], REJOIN_PERIOD_S)
        if not ready:
            continue
        try:
            pkt, (peer, port) = sock.recvfrom(9000)
        except OSError:
            continue
        ip = _source_ip_toward(peer)
        if ip is None or not wants_answer(pkt, fqdn, ip):
            continue
        try:
            sock.sendto(build_answer(fqdn, ip), (peer, port))
            answered += 1
        except OSError:
            pass                               # 経路が消えた瞬間など。次の問い合わせで返す


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--name", default=None,
                    help="`.local` を除いたホスト名（既定: この機械のホスト名）")
    args = ap.parse_args()
    host = (args.name or socket.gethostname()).split(".")[0].lower()
    serve(f"{host}.local")


if __name__ == "__main__":
    main()
