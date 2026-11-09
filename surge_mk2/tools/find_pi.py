"""Pi の IPv4 を mDNS で直接引く（Mac 側）。**macOS の名前解決（mDNSResponder）を通さない。**

    .venv/bin/python -m tools.find_pi            # IP を1行出す（見つからなければ終了コード 1）
    .venv/bin/python -m tools.find_pi --open     # 見つけた IP で実機 GUI をブラウザで開く

## なぜ要るか

Wi-Fi だけで繋いだ Mac で、`surge-mk2.local` が引けなくなり GUI が「読み込み中」や
INIT のまま止まった。IP 直打ちなら GUI も WS も全部届く（経路は生きている）。

- その Mac は Wi-Fi 上の **224.0.0.251（mDNS）とブロードキャストを受け取れていなかった**
  （Pi から各10回送って 0/10。ユニキャストと 239.x は 10/10）。Pi の avahi のマルチキャスト
  応答も、IP が変わったときの告知も届かない
- Pi 側の `raspi/tools/mdns_unicast.py` が QU 問い合わせにユニキャストで答えるので普段は引けるが、
  Pi の再起動などの後に mDNSResponder が**問い合わせ自体を出さない状態**で固まることがあった
  （10分見ていて1通も出ない。Wi-Fi を入れ直すと戻る）。有線ではマルチキャストが届くので起きない

## 何をするか

送信元ポートを 5353 以外にした問い合わせ（RFC 6762 §6.7 の "legacy unicast"）を、
IPv4 を持つ全インタフェースから 224.0.0.251 へ投げる。応答側（avahi・mdns_unicast）は
**問い合わせ元へユニキャストで**返すので、マルチキャストが届かない Mac でも受け取れ、
mDNSResponder の状態にも左右されない。Pi 側は何も変えなくてよい。

複数返ったら**先に返った順**に並べる（直結と Wi-Fi の両方が生きていれば、普通は直結が先）。

**IP で開いた GUI は WS も IP に繋ぐ**（`gui/src/ws/url.ts` は同一オリジン）ので、開いた後は
名前解決に頼らない。telemetry_node の Origin 検査は IP リテラルの Host を通す（`_host_ok`）。
"""

from __future__ import annotations

import argparse
import ipaddress
import select
import socket
import struct
import subprocess
import sys
import time
import webbrowser

from raspi.tools.mdns_unicast import CLASS_IN, MDNS_GROUP, MDNS_PORT, TYPE_A, _read_name

DEFAULT_NAME = "surge-mk2"
GUI_PORT = 8000
#: 1回の問い合わせで応答を待つ時間 [s]。Wi-Fi 越しの実測は約20ms（初回は 90ms）
WAIT_S = 0.6
TRIES = 3


def build_query(fqdn: str, qid: int) -> bytes:
    """`fqdn` の A を問う mDNS 問い合わせ。**ID を非0にする**（legacy unicast の応答は ID を写して返る）。"""
    name = b"".join(bytes([len(p)]) + p for p in fqdn.encode("ascii").split(b".")) + b"\0"
    return struct.pack(">HHHHHH", qid, 0, 1, 0, 0, 0) + name + struct.pack(">HH", TYPE_A, CLASS_IN)


def parse_a_records(pkt: bytes, fqdn: str) -> list[str]:
    """応答から `fqdn` の A レコードの IPv4 を取り出す。壊れた・関係ない応答は空。"""
    try:
        flags, qd, an, ns, ar = struct.unpack(">HHHHH", pkt[2:12])
        if not flags & 0x8000:                 # 問い合わせ（自分や他人のもの）
            return []
        off = 12
        for _ in range(qd):                    # legacy unicast の応答は質問を写して返る
            _, off = _read_name(pkt, off)
            off += 4
        ips: list[str] = []
        for _ in range(an + ns + ar):
            name, off = _read_name(pkt, off)
            rtype, rclass, _ttl, rdlen = struct.unpack(">HHIH", pkt[off:off + 10])
            off += 10
            if (name == fqdn and rtype == TYPE_A and rclass & 0x7FFF == CLASS_IN
                    and rdlen == 4):
                ips.append(socket.inet_ntoa(pkt[off:off + 4]))
            off += rdlen
        return ips
    except (IndexError, struct.error, ValueError, OSError):   # OSError: 尻切れの rdata（inet_ntoa）
        return []


def _local_ipv4s() -> list[str]:
    """問い合わせを出すインタフェースの IPv4（ループバック以外）。"""
    try:
        out = subprocess.run(["ifconfig"], capture_output=True, text=True, timeout=3,
                             check=False).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    ips = []
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[0] == "inet":
            try:
                ip = ipaddress.IPv4Address(parts[1])
            except ValueError:
                continue
            if not ip.is_loopback:
                ips.append(str(ip))
    return ips


def find_pi(name: str = DEFAULT_NAME, *, wait_s: float = WAIT_S, tries: int = TRIES) -> list[str]:
    """`<name>.local` の IPv4 を、先に返った順に返す。見つからなければ空。"""
    fqdn = f"{name.removesuffix('.local').lower()}.local"
    socks = []
    for local in _local_ipv4s():
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # 送信元を5353以外（任意ポート）にする ＝ legacy unicast。応答はこのソケットに直接来る
            s.bind((local, 0))
            s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(local))
            s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 255)
        except OSError:
            s.close()
            continue
        socks.append(s)
    found: list[str] = []
    try:
        for attempt in range(tries):
            qid = (int(time.monotonic() * 1000) + attempt) & 0xFFFF or 1
            pkt = build_query(fqdn, qid)
            for s in socks:
                try:
                    s.sendto(pkt, (MDNS_GROUP, MDNS_PORT))
                except OSError:
                    pass                       # 経路の無い IF（リンクが落ちた直後など）
            deadline = time.monotonic() + wait_s
            while socks and (left := deadline - time.monotonic()) > 0:
                ready, _, _ = select.select(socks, [], [], left)
                for s in ready:
                    try:
                        data, _ = s.recvfrom(9000)
                    except OSError:
                        continue
                    for ip in parse_a_records(data, fqdn):
                        if ip not in found:
                            found.append(ip)
            if found:
                return found
    finally:
        for s in socks:
            s.close()
    return found


def gui_url(ip: str) -> str:
    return f"http://{ip}:{GUI_PORT}/"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--name", default=DEFAULT_NAME, help="`.local` を除いたホスト名")
    ap.add_argument("--open", action="store_true", help="見つけた IP で実機 GUI を開く")
    args = ap.parse_args()
    ips = find_pi(args.name)
    if not ips:
        print(f"{args.name}.local が見つからない（Pi の電源・Wi-Fi・surge-mdns を確認）",
              file=sys.stderr)
        return 1
    if args.open:
        url = gui_url(ips[0])
        others = f"（ほかに {', '.join(ips[1:])}）" if ips[1:] else ""
        print(f"{args.name}.local = {ips[0]}{others} → {url} を開く", flush=True)
        webbrowser.open(url)
    else:
        print(ips[0])
    return 0


if __name__ == "__main__":
    sys.exit(main())
