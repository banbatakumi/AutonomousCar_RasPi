"""校正 7: タイヤ（スリップ率 → 前後力の曲線）。解析は `tools/calib/tyre.py`、設計は `docs/calibration.md` §4.7。

## 測り方: TC・ABS を切り、トルクをゆっくり上げる（準静的なランプ）

TC・ABS 自身にスリップ率を保たせる旧方式は、未調整の制御器が低いスリップ率を保てず（目標 0.03 でも実際は
0.065 以上）、制御器の揺れが点に載り、一番知りたい肩と傾きが測れなかった。今は制御器を使わない。

前進の片道ごとに、`fw_overrides` で TC・ABS・TV を切り（片輪浮き対策は残す。後輪周速 7.5m/s の歯止め）、

1. `pre`: 車速指令で `v_drive`（制動は `v_brake`）まで上げる
2. `trim`: その車速を保つ（ゼロ点: ほぼ加速していない間のスリップ率の読み。前輪の倍率誤差などの偏りを解析が引く）
3. `dramp`（駆動）/ `bramp`（制動）: トルクを 0（保つトルク）から `max_torque_nm` まで `ramp_s` 秒で直線に上げる

ランプが遅ければ車輪の慣性の項は小さく、スリップ率は 0 から連続して増える（低いスリップ率ではタイヤが硬く、
スリップは力にすぐ追従する）。**前後力は車体の加速度から出す**（F = 質量 ×（加速度＋転がり抵抗）。車輪側の
慣性を使わない）。制御器に依らず、低スリップ側から山の先までが 1 本のランプで測れる。ランプの速さを変えて
（`drive_ramp_s`・`brake_ramp_s`）、速さによらないことも見る。後退の片道は低速で元の位置へ戻るだけ。

## 安全

- 駆動は、どちらかの後輪のスリップが `slip_cut` を超えたら、車速が `v_top` を超えたら、ランプが終わって短く
  保ったらやめる（`plan()` は 10Hz なので最大 100ms 遅れる。後輪周速はその間に 1〜2m/s 増える程度で、
  ファームの 7.5m/s の歯止めに届かない）。制動は最大トルクで保ち、車速が `v_end` を切ったらやめる（ロックしても
  車速が低いので止まる距離は短い）
- 前輪と後輪の照合（`SensorGuard`）はしない（後輪を滑らせるので）。MD の無応答は見る
- ランプの区間が終わると、土台が止める（`stop`。そこでは上書きが外れて TC・ABS が入る）

必要なスペース: 直線 2m＋車長。約 60 秒。
"""

from __future__ import annotations

from ..msgs.convert import MAX_TARGET_TORQUE_NM
from ..msgs.types import VehicleState
from ._calib_common import Step, StraightTest, stop_distance

__all__ = ["CalibTyre", "TractionLegs", "TYRE_OVERRIDES"]

#: タイヤの試験の間だけ切るもの（制御器に依らず曲線を測る。片輪浮き対策は歯止めとして残す）
TYRE_OVERRIDES = {"tc_enable": 0.0, "abs_enable": 0.0, "tv_enable": 0.0}
MAX_TORQUE_NM = MAX_TARGET_TORQUE_NM
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


class CalibTyre(StraightTest):
    id = "calib_tyre"
    name = "校正 7: タイヤ（スリップ率→前後力）"
    description = ("TC・ABS を切り、直線で駆動と制動のトルクをゆっくり 0 から最大まで上げて（各3回）、スリップ率と前後力の"
                   "曲線を測る。直線2m＋車長。約60秒")

    wheel_check = False
    #: 土台が片道の終わりに止める制動トルク（1輪あたり）と、止まりきれる減速度の見積もり
    stop_torque_nm = 0.08
    stop_decel_m_s2 = 1.8

    SETTINGS = {
        #: 往復の数。駆動と制動を交互に（0・2・4 が駆動、1・3・5 が制動）
        "round_trips": 6,
        #: ランプの長さ [s]（往復の組ごとに順に。速さによらないことを見る）
        "drive_ramp_s": (0.8, 0.55, 0.8),
        "brake_ramp_s": (0.5, 0.4, 0.5),
        #: ランプに入る前の車速 [m/s] と、加速度・ゼロ点を読む時間 [s]
        "v_drive": 0.6, "v_brake": 1.4, "accel": 1.5, "trim_s": 0.2,
        "max_torque_nm": MAX_TORQUE_NM,
        #: 駆動のランプをやめる条件: どちらかの後輪のスリップ率・車速 [m/s]・ランプの後に保つ時間 [s]
        "slip_cut": 0.3, "v_top": 2.2, "hold_s": 0.15,
        #: 制動のランプをやめる車速 [m/s]
        "v_end": 0.5,
        "return_speed": 0.6,
    }

    def _reset_state(self) -> None:
        super()._reset_state()
        self._cur_leg = -1
        self._stage_name = "pre"
        self._stage_from = 0.0
        self._trim_tq = 0.0
        self._trim_n = 0

    def leg_count(self) -> int:
        return int(self.settings["round_trips"]) * 2

    def kind(self, leg: int) -> str:
        return "drive" if (leg // 2) % 2 == 0 else "brake"

    def ramp_s(self, leg: int) -> float:
        k = self.kind(leg)
        ramps = self.settings[f"{k}_ramp_s"]
        return float(ramps[(leg // 4) % len(ramps)])

    def leg_title(self, leg: int) -> str:
        return f"{'駆動' if self.kind(leg) == 'drive' else '制動'}のランプ {leg // 2 + 1}/{self.leg_count() // 2}"

    def leg_step(self, leg: int, t: float, vs: VehicleState, traveled: float) -> Step:
        s = self.settings
        if leg % 2 == 1:
            return Step("speed", s["return_speed"], "return", accel_limit=1.0, note="戻る")
        if leg != self._cur_leg:
            self._cur_leg, self._stage_name, self._stage_from = leg, "pre", t
            self._trim_tq, self._trim_n = 0.0, 0
        kind = self.kind(leg)
        v = abs(vs.speed)
        v0 = s["v_drive"] if kind == "drive" else s["v_brake"]
        if self._stage_name == "pre" and v >= v0 - 0.04:
            self._stage_name, self._stage_from = "trim", t
        if self._stage_name == "trim":
            self._trim_tq += 0.5 * (vs.torque_cmd[0] + vs.torque_cmd[1]) if len(vs.torque_cmd) >= 2 else 0.0
            self._trim_n += 1
            if t - self._stage_from >= s["trim_s"]:
                self._stage_name, self._stage_from = "ramp", t
        ov = dict(TYRE_OVERRIDES)
        if self._stage_name == "pre":
            return Step("speed", v0, "pre", accel_limit=s["accel"], overrides=ov, note=f"車速 {v0:.1f}m/s まで")
        if self._stage_name == "trim":
            return Step("speed", v0, "trim", accel_limit=s["accel"], overrides=ov, note="ゼロ点（定速）")
        el = t - self._stage_from
        ramp = self.ramp_s(leg)
        frac = min(1.0, el / ramp)
        tmax = float(s["max_torque_nm"])
        if kind == "drive":
            slips = [abs(x) for x in vs.tc_slip] if vs.tc_slip else [0.0]
            if max(slips) > s["slip_cut"] or v >= s["v_top"] or el >= ramp + s["hold_s"]:
                return Step(done=True)
            t0 = max(0.0, self._trim_tq / max(self._trim_n, 1))
            tq = t0 + (tmax - t0) * frac
            return Step("torque", tq, "dramp", overrides=ov, note=f"駆動トルクを上げる {tq:.3f}N·m")
        if v < s["v_end"]:
            return Step(done=True)
        # 0 は「最大の制動」に読み替わるので、ランプの頭は 0 にしない
        tq = max(0.004, tmax * frac)
        return Step("brake", tq, "bramp", overrides=ov, note=f"制動トルクを上げる {tq:.3f}N·m")
