"""CPU の上限周波数を ARM/DISARM で切り替える（省電力、2026-10-04）。

`docs/power_audit_2026-10.md` の実測（制御系 11V 側の電流）:

| 条件 | 上限 2.4GHz（従来） | 上限を下げたとき |
|---|---|---|
| 待機（DISARM） | 基準 | 1.5GHz で **−56mA**（1.8GHz では −3mA しか変わらない） |
| カメラ seg 推論 30Hz | 1008mA・推論 p50 26.2ms | 2.0GHz で **946mA**・28.4ms |

Pi 5 は 1.5GHz が電圧の底で、それより上は電圧が段々に上がる。ondemand は
待機中の小さな負荷でも 1.7GHz 以上へ跳ねるので、DISARM 中は 1.5GHz に抑える。
ARM 中は 2.0GHz（1.8GHz では seg 推論の p95 が 34.5ms でフレーム周期 33ms を超えた）。

`scaling_max_freq` は既定 `root:root 644` で pi から書けない。surge-telemetry の
unit の drop-in（`install_services.sh`、`ExecStartPre=+`）が gpio グループに
書き込み権限だけを渡す（`99-surge-fan.rules` と同じ思想）。書けない環境
（Mac・テスト・権限未適用）では黙って何もしない。
"""

from __future__ import annotations

import os
from pathlib import Path

__all__ = ["CpuFreqCap", "open_cpufreq"]

POLICY_DIR = Path("/sys/devices/system/cpu/cpufreq/policy0")


class CpuFreqCap:
    """`scaling_max_freq` を読み書きする。今の値と同じなら書かない（sysfs を読んで比べる）。

    前回書いた値を覚えて比べるのではなく実際の値を読むのは、外から書き換えられても
    （Pi 上のテスト・手作業）次の呼び出しで直すため。"""

    __slots__ = ("available", "_max_path", "_hw_max_khz", "_hw_min_khz")

    def __init__(self, policy_dir: Path = POLICY_DIR) -> None:
        self.available = False
        self._max_path = policy_dir / "scaling_max_freq"
        self._hw_max_khz = 0
        self._hw_min_khz = 0
        try:
            self._hw_max_khz = int((policy_dir / "cpuinfo_max_freq").read_text())
            self._hw_min_khz = int((policy_dir / "cpuinfo_min_freq").read_text())
        except (OSError, ValueError):
            return
        self.available = os.access(self._max_path, os.W_OK)

    def set_max_khz(self, khz: int) -> bool:
        """上限を `khz` にする（ハードウェアの範囲へクランプ）。書けたら True。"""
        if not self.available:
            return False
        khz = max(self._hw_min_khz, min(self._hw_max_khz, int(khz)))
        try:
            if self._max_path.read_text().strip() == str(khz):
                return True
            self._max_path.write_text(str(khz))
        except OSError:
            return False
        return True

    def restore(self) -> bool:
        """ハードウェア上限へ戻す（プロセス終了時。他の用途の邪魔をしない）。"""
        return self.set_max_khz(self._hw_max_khz) if self.available else False


def open_cpufreq() -> CpuFreqCap:
    return CpuFreqCap()
