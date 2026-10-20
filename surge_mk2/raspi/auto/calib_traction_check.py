"""校正 9: TC・ABS の確かめ。`config/vehicle.toml` の値（STM32 に入っている値）のまま、直線で全開の発進と目一杯の
減速を行い、スリップ率が目標に保たれているかと、**実際に出る加速度・減速度**を測る。解析は `tools/calib/traction.py`、
設計は `docs/calibration.md` §4.9。

前進の片道は 4 回: 「全開の発進 → 車速指令で目一杯の減速（TC。slam2d の減速と同じ経路）」を 2 回、
「全開の発進 → 最大の制動（ABS）」を 2 回。後退の片道は低速で戻る。測った加速度・減速度は slam2d の上限
（走行中の学習の初期値）になる。

必要なスペース: 直線 2m＋車長。約 35 秒。
"""

from __future__ import annotations

from .calib_tyre import TractionLegs

__all__ = ["CalibTractionCheck"]


class CalibTractionCheck(TractionLegs):
    id = "calib_traction_check"
    name = "校正 9: TC・ABS の確かめ"
    description = ("今の TC・ABS の設定のまま、直線で全開の発進と目一杯の減速（車速指令2回・制動2回）を行い、"
                   "スリップ率の保ち方と実際に出る加速度・減速度を測る。直線2m＋車長。約35秒")

    #: 前進の片道ごとの止め方（True = 車速指令、False = 制動）
    PATTERN = (True, False, True, False)

    def leg_count(self) -> int:
        return 2 * len(self.PATTERN)

    def decel_by_speed(self, leg: int) -> bool:
        return self.PATTERN[(leg // 2) % len(self.PATTERN)]

    def brake_decel_guess(self, leg: int) -> float:
        return 1.6

    def leg_title(self, leg: int) -> str:
        how = "減速は車速指令" if self.decel_by_speed(leg) else "減速は制動"
        return f"{how}・{leg // 2 + 1}/{len(self.PATTERN)}"
