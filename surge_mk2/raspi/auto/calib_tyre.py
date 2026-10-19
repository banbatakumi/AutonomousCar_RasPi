"""校正 7: タイヤ（スリップ率 → 前後力の曲線）。解析は `tools/calib/tyre.py`、設計は `docs/calibration.md` §4.7。

## 測り方: TC・ABS 自身にスリップ率を保たせる

前進の片道ごとに **TC と ABS の目標スリップ率を上書き**（`fw_overrides`）して、

1. 全開の発進（トルク直接指令 0.13N·m。TC が各輪のスリップ率を目標に保つ）→ `v_top` まで
2. 最大の制動（0.13N·m。ABS がロックしかけている側を目標に保つ）→ 止まるまで

を行い、目標を `LEVELS` で振る。後退の片道は低速で元の位置へ戻るだけ。

スリップ率がほぼ一定に保たれている間は車輪の回転の加速が車体の加速とそろう（車輪の慣性の項が小さい）ので、
**前後力は車体の加速度から素直に出る**（F = 質量 ×（加速度＋転がり抵抗））。TC を切って空転・ロックさせる
旧方式は、空転中の車輪の慣性の項が駆動トルクと同じ大きさになり、力の推定が車輪の速度の微分と遅れの扱いで
決まってしまっていた。

測るのは「TC・ABS が保つスリップ率 → そのときの前後力の合計」で、TC・ABS の設計にそのまま使える形
（左右のタイヤのグリップが違っても、そのままの形で入る。ABS は左右共通で、滑りやすい側に合わせる）。

## 安全

- 制動は試験そのものなので、止まりきれる地点を、その段の目標スリップ率で出る減速度の見積もりで決める
- 後輪をわざと滑らせるので、前輪と後輪の照合（`SensorGuard`）はしない。MD の無応答は見る

必要なスペース: 直線 2m＋車長。約 45 秒。
"""

from __future__ import annotations

from ..msgs.types import VehicleState
from ._calib_common import Step, StraightTest, stop_distance

__all__ = ["CalibTyre", "TractionLegs", "LEVELS"]

#: 振る目標スリップ率（TC と ABS で同じ値）
LEVELS: tuple[float, ...] = (0.03, 0.06, 0.10, 0.15, 0.22, 0.32)
MAX_TORQUE_NM = 0.13
#: 車速指令で減速するときの目標車速のランプ [m/s²]（グリップより十分大きく。PI が負のトルクを振り切る）
DECEL_RAMP_M_S2 = 8.0


class TractionLegs(StraightTest):
    """前進で「全開の発進 → 最大の制動」、後退で低速に戻る、の往復。片道ごとの上書きは `leg_overrides()`。"""

    wheel_check = False
    SETTINGS = {
        "v_top": 1.8,
        "return_speed": 0.6,
        "prep_s": 0.4,
        #: 制動に移る判断の余裕 [m]
        "brake_margin_m": 0.15,
    }

    def leg_overrides(self, leg: int) -> dict[str, float]:
        return {}

    def decel_by_speed(self, leg: int) -> bool:
        """True ならこの片道の止め方は車速指令（TC）、False なら制動（ABS）。"""
        return False

    def brake_decel_guess(self, leg: int) -> float:
        """その片道の制動で出る減速度の保守側の見積もり [m/s²]（止まり始める地点を決めるだけ）。"""
        return 1.0

    def leg_stop_decel(self, leg: int) -> float:
        # 土台の打ち切りは、手順が制動に移る判断（見積もり＋余裕）より後に来るようにする
        return self.brake_decel_guess(leg) * 0.9 if leg % 2 == 0 else self.stop_decel_m_s2

    def _reset_state(self) -> None:
        super()._reset_state()
        self._cur_leg = -1
        self._stage_name = "prep"
        self._stage_from = 0.0

    def leg_step(self, leg: int, t: float, vs: VehicleState, traveled: float) -> Step:
        s = self.settings
        if leg % 2 == 1:
            # 後退: 低速で戻る（通路の端＝置いた位置で土台が止める）
            return Step("speed", s["return_speed"], "return", accel_limit=1.0, note="戻る")
        if leg != self._cur_leg:
            self._cur_leg, self._stage_name, self._stage_from = leg, "prep", t
        ov = self.leg_overrides(leg)
        v = abs(vs.speed)
        el = t - self._stage_from
        if self._stage_name == "prep" and el >= s["prep_s"]:
            self._stage_name, self._stage_from = "drive", t
        if self._stage_name == "drive":
            need = stop_distance(v, 3.0, self.brake_decel_guess(leg)) + s["brake_margin_m"]
            if v >= s["v_top"] or need >= self.room:
                self._stage_name, self._stage_from = "brake", t
        if self._stage_name == "brake" and v < 0.15:
            return Step(done=True)
        if self._stage_name == "prep":
            # 上書きが STM32 に入るまで止まって待つ（入ったかは記録の /diag/link で確かめられる）
            return Step("brake", 0.02, "prep", overrides=ov, note="設定を入れて待つ")
        if self._stage_name == "drive":
            return Step("torque", MAX_TORQUE_NM, "drive", overrides=ov, note="全開の発進")
        if self.decel_by_speed(leg):
            # 車速指令で目一杯減速（PI が負のトルクを振り切り、TC が滑りを抑える。slam2d の減速と同じ経路）
            return Step("speed", 0.0, "decel", accel_limit=DECEL_RAMP_M_S2, overrides=ov, stopping=True,
                        note="車速指令で目一杯の減速")
        return Step("brake", MAX_TORQUE_NM, "brake", overrides=ov, note="最大の制動")


class CalibTyre(TractionLegs):
    id = "calib_tyre"
    name = "校正 7: タイヤ（スリップ率→前後力）"
    description = ("TC・ABS の目標スリップ率を6段に振り、直線で全開の発進と最大の制動を行う（前進6回・後退は低速で戻る）。"
                   "スリップ率と前後力の曲線を測る。直線2m＋車長。約45秒")

    def leg_count(self) -> int:
        return 2 * len(LEVELS)

    def level(self, leg: int) -> float:
        return LEVELS[(leg // 2) % len(LEVELS)]

    def leg_overrides(self, leg: int) -> dict[str, float]:
        k = self.level(leg)
        return {"tc_enable": 1.0, "abs_enable": 1.0, "tv_enable": 0.0, "tc_slip_target": k, "abs_slip_target": k}

    def brake_decel_guess(self, leg: int) -> float:
        # 目標スリップ率が小さい段ほど制動力が小さい（曲線の立ち上がり）。0.03 で約 1m/s²
        return 0.9 if self.level(leg) < 0.05 else 1.4

    def leg_title(self, leg: int) -> str:
        return f"目標 {self.level(leg):.2f}・{leg // 2 + 1}/{len(LEVELS)}"
