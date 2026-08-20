"""Wi-Fi接続状態（SSID・電波強度）— NetworkManager管理下のwlan0を問い合わせる。

SSIDは `nmcli`（NetworkManager）から、電波強度(RSSI dBm)は `/proc/net/wireless`
から読む。二系統に分けているのは、SSIDはnmcli以外に取得手段がなく（surge_mk2は
NetworkManager前提、`architecture.md` §9.1）、RSSIは `/proc/net/wireless` を
読むだけで済み `nmcli` のプロセス起動より軽いため。

`nmcli` のサブプロセス起動は数十〜百数十msかかる（実機実測 ~0.1s）。20Hzの
`/ws/telemetry` には乗せず、`raspi/io/fan.py` と同じく低頻度ポーリング
（`telemetry_node.py` の `_wifi_pump`、`/ws/control` 経由）で使う前提。
"""

from __future__ import annotations

import subprocess
from dataclasses import dataclass
from pathlib import Path

__all__ = ["WifiState", "FakeWifi", "NmcliWifi", "open_wifi"]

WIRELESS_PROC = Path("/proc/net/wireless")
NMCLI_TIMEOUT_S = 1.0


@dataclass(slots=True)
class WifiState:
    ssid: str | None       # 未接続なら None
    rssi_dbm: int | None   # 読めなければ None
    available: bool        # この機体でWi-Fi状態の問い合わせ自体が使えるか


class FakeWifi:
    """`nmcli`/`/proc/net/wireless` のない環境（Mac・テスト）用。常に未接続扱い。"""

    __slots__ = ("available",)

    def __init__(self) -> None:
        self.available = True

    def read(self) -> WifiState:
        return WifiState(ssid=None, rssi_dbm=None, available=True)


class NmcliWifi:
    """実機用。`nmcli` でSSID、`/proc/net/wireless` でRSSIを読む。

    片方だけ読めなくても他方は返す。`available=False` でも呼び出し側はこれを見て
    GUIのWi-Fi表示を「取得不可」にする（`fan.py` の `available` と同じ扱い）。
    """

    __slots__ = ("available", "_iface")

    def __init__(self, iface: str = "wlan0") -> None:
        self._iface = iface
        self.available = WIRELESS_PROC.exists() and _nmcli_present()

    def read(self) -> WifiState:
        return WifiState(ssid=self._read_ssid(), rssi_dbm=self._read_rssi(),
                          available=self.available)

    def _read_ssid(self) -> str | None:
        try:
            out = subprocess.run(
                ["nmcli", "-t", "-f", "GENERAL.CONNECTION,GENERAL.STATE",
                 "dev", "show", self._iface],
                capture_output=True, text=True, timeout=NMCLI_TIMEOUT_S, check=False)
        except (OSError, subprocess.SubprocessError):
            return None
        conn: str | None = None
        connected = False
        for line in out.stdout.splitlines():
            if line.startswith("GENERAL.CONNECTION:"):
                conn = line.split(":", 1)[1].strip()
            elif line.startswith("GENERAL.STATE:"):
                connected = "connected" in line
        if not connected or not conn or conn == "--":
            return None
        return conn

    def _read_rssi(self) -> int | None:
        """`/proc/net/wireless` の `level`(dBm) 列を読む。

        書式（実機確認済み）:
        ` face | status | link level noise | ...`
        ` wlan0: 0000   70.  -36.  -256  ...`
        status/link/level の順で、levelがインデックス2（0始まり）。
        """
        try:
            lines = WIRELESS_PROC.read_text().splitlines()
        except OSError:
            return None
        prefix = f"{self._iface}:"
        for raw in lines:
            line = raw.strip()
            if not line.startswith(prefix):
                continue
            fields = line[len(prefix):].split()
            if len(fields) < 3:
                return None
            try:
                return int(float(fields[2]))
            except ValueError:
                return None
        return None


def _nmcli_present() -> bool:
    try:
        subprocess.run(["nmcli", "-v"], capture_output=True,
                        timeout=NMCLI_TIMEOUT_S, check=False)
        return True
    except (OSError, subprocess.SubprocessError):
        return False


def open_wifi(iface: str = "wlan0") -> NmcliWifi | FakeWifi:
    """Wi-Fi状態の問い合わせを開く。`nmcli`/`/proc/net/wireless` が無い環境
    （Mac開発機など）でも例外を出さず `FakeWifi` を返す。"""
    try:
        return NmcliWifi(iface)
    except Exception:
        return FakeWifi()
