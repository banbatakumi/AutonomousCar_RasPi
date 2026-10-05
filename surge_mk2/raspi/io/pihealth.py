"""RasPi 本体の健全性（CPU 使用率・メモリ・周波数・スロットリング）— 診断タブ用。

`raspi/io/wifi.py` と同じ作り: 低頻度（1Hz）で `telemetry_node` の `_pi_health_pump`
が `asyncio.to_thread` 越しに読み、`/ws/control` の `status.pi` に載せる。
**指令を出すイベントループの中では読まない。**

読むのは `/proc` と sysfs だけ（1回あたり数十μs）。例外はスロットリングで、
sysfs に出ていない機体では `vcgencmd get_throttled` を使う。こちらはサブプロセスの
起動を挟むので `THROTTLED_EVERY` 回に1回だけ読む。

Pi 以外（Mac・テスト）では `/proc/stat` が無く `available=False` になる。
"""

from __future__ import annotations

import shutil
import subprocess
from dataclasses import asdict, dataclass
from pathlib import Path

__all__ = ["PiHealth", "PiHealthReader", "parse_cpu_times", "parse_meminfo",
           "parse_throttled", "THROTTLED_BITS"]

PROC_STAT = Path("/proc/stat")
PROC_MEMINFO = Path("/proc/meminfo")
PROC_LOADAVG = Path("/proc/loadavg")
CPUFREQ_DIR = Path("/sys/devices/system/cpu/cpufreq/policy0")
#: ファームウェアのスロットリング状態を sysfs に出す機体での置き場所（無ければ `vcgencmd`）。
#: ★ Pi 5 実機でこのパスがあるかは未確認（2026-10-05）。無くても `vcgencmd` へ落ちるだけ
THROTTLED_SYSFS = (
    Path("/sys/devices/platform/soc/soc:firmware/get_throttled"),
)
VCGENCMD_TIMEOUT_S = 1.0
#: `vcgencmd` を使う場合に、何回の `read()` につき1回読むか（1Hz で呼ばれて 5秒に1回）
THROTTLED_EVERY = 5

#: `get_throttled` のビット（Raspberry Pi 公式の定義）。下位4つが「今」、上位4つが
#: 「起動してから一度でも」
THROTTLED_BITS = {
    "undervoltage": 0x1,
    "freq_capped": 0x2,
    "throttled": 0x4,
    "soft_temp_limit": 0x8,
    "undervoltage_occurred": 0x10000,
    "freq_capped_occurred": 0x20000,
    "throttled_occurred": 0x40000,
    "soft_temp_limit_occurred": 0x80000,
}


@dataclass(slots=True)
class PiHealth:
    available: bool                 #: この機体で読めるか（Pi 以外では False）
    cpu_pct: float | None = None    #: 全コア平均の使用率 [%]（前回の読みからの区間）
    cpu_max_pct: float | None = None  #: いちばん忙しいコアの使用率 [%]
    load1: float | None = None      #: 1分のロードアベレージ
    mem_used_pct: float | None = None  #: (MemTotal − MemAvailable) / MemTotal [%]
    mem_total_mb: float | None = None
    cpu_khz: int | None = None      #: 今の動作周波数
    cpu_max_khz: int | None = None  #: 今の上限（ARM/DISARM で切り替わる。`cpufreq.py`）
    #: `get_throttled` の生値。読めない機体では None
    throttled: int | None = None

    def as_dict(self) -> dict:
        return asdict(self)


def parse_cpu_times(text: str) -> dict[str, tuple[int, int]]:
    """`/proc/stat` → `{"cpu": (busy, total), "cpu0": ..., ...}`（jiffies の累積）。

    idle と iowait を「空き」に数える。"""
    out: dict[str, tuple[int, int]] = {}
    for line in text.splitlines():
        if not line.startswith("cpu"):
            continue
        f = line.split()
        try:
            v = [int(x) for x in f[1:9]]
        except ValueError:
            continue
        if len(v) < 5:
            continue
        total = sum(v)
        out[f[0]] = (total - v[3] - v[4], total)
    return out


def parse_meminfo(text: str) -> tuple[float, float] | None:
    """`/proc/meminfo` → `(used_pct, total_mb)`。必要な行が無ければ None。"""
    kb: dict[str, int] = {}
    for line in text.splitlines():
        name, _, rest = line.partition(":")
        if name in ("MemTotal", "MemAvailable"):
            try:
                kb[name] = int(rest.split()[0])
            except (ValueError, IndexError):
                return None
    total, avail = kb.get("MemTotal"), kb.get("MemAvailable")
    if not total or avail is None:
        return None
    return 100.0 * (total - avail) / total, total / 1024.0


def parse_throttled(text: str) -> int | None:
    """`throttled=0x50000`（vcgencmd）でも `50000`（sysfs、16進）でも読む。"""
    s = text.strip()
    if "=" in s:
        s = s.split("=", 1)[1]
    try:
        return int(s, 16)
    except ValueError:
        return None


def _read_int(path: Path) -> int | None:
    try:
        return int(path.read_text().strip())
    except (OSError, ValueError):
        return None


class PiHealthReader:
    """`read()` を呼ぶたびに、前回からの区間の CPU 使用率を返す（初回は None）。"""

    def __init__(self, *, proc_stat: Path = PROC_STAT, proc_meminfo: Path = PROC_MEMINFO,
                 proc_loadavg: Path = PROC_LOADAVG, cpufreq_dir: Path = CPUFREQ_DIR,
                 throttled_sysfs: tuple[Path, ...] = THROTTLED_SYSFS,
                 use_vcgencmd: bool = True) -> None:
        self._stat = proc_stat
        self._meminfo = proc_meminfo
        self._loadavg = proc_loadavg
        self._cpufreq = cpufreq_dir
        self.available = proc_stat.exists()
        self._prev: dict[str, tuple[int, int]] = {}
        self._throttled_path = next((p for p in throttled_sysfs if p.exists()), None)
        self._vcgencmd = (shutil.which("vcgencmd")
                          if use_vcgencmd and self.available
                          and self._throttled_path is None else None)
        self._throttled: int | None = None
        self._n = 0

    def read(self) -> PiHealth:
        if not self.available:
            return PiHealth(available=False)
        h = PiHealth(available=True)
        try:
            cur = parse_cpu_times(self._stat.read_text())
        except OSError:
            cur = {}
        pcts: dict[str, float] = {}
        for name, (busy, total) in cur.items():
            prev = self._prev.get(name)
            if prev is not None and total > prev[1]:
                pcts[name] = 100.0 * (busy - prev[0]) / (total - prev[1])
        self._prev = cur
        h.cpu_pct = pcts.get("cpu")
        cores = [v for k, v in pcts.items() if k != "cpu"]
        h.cpu_max_pct = max(cores) if cores else None
        try:
            h.load1 = float(self._loadavg.read_text().split()[0])
        except (OSError, ValueError, IndexError):
            pass
        try:
            mem = parse_meminfo(self._meminfo.read_text())
        except OSError:
            mem = None
        if mem is not None:
            h.mem_used_pct, h.mem_total_mb = mem
        h.cpu_khz = _read_int(self._cpufreq / "scaling_cur_freq")
        h.cpu_max_khz = _read_int(self._cpufreq / "scaling_max_freq")
        h.throttled = self._read_throttled()
        self._n += 1
        return h

    def _read_throttled(self) -> int | None:
        if self._throttled_path is not None:
            try:
                return parse_throttled(self._throttled_path.read_text())
            except OSError:
                return None
        if self._vcgencmd is None:
            return None
        if self._n % THROTTLED_EVERY == 0:
            try:
                out = subprocess.run([self._vcgencmd, "get_throttled"], capture_output=True,
                                     text=True, timeout=VCGENCMD_TIMEOUT_S, check=False)
                self._throttled = parse_throttled(out.stdout) if out.returncode == 0 else None
            except (OSError, subprocess.SubprocessError):
                self._throttled = None
        return self._throttled
