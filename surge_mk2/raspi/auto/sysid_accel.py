"""システム同定 — 加減速試験（`config/vehicle.toml` `[dynamics]` の実測手順）。

直進で `target_speed` への加速→急制動を繰り返し、`VehicleState.accel`（x成分。
縦加速度）の応答から `drive_accel_m_s2`（最大加速度）・`brake_decel_m_s2`
（最大減速度）を測る。ログの解析は Mac 側の `tools/sysid/fit.py` の
`fit_accel()` が行う——このプランナーはステップ入力を送るだけで、値の
推定はしない（他の`sysid_*`プランナーと同じ設計）。

## なぜこの試験が要るか

`sim/vehicle.py` の摩擦円連成（`a_lat_max = sqrt((mu*g)^2 - a_long^2)`）が
使う縦加速度の上限は、これまで`MAX_BRAKE_TORQUE_NM`（後輪モータのハードウェア
仕様値）からの逆算に頼っていた。しかし`mu`は`sysid_corner`で実測されている
のに対し、こちらは未実測のまま——実際にタイヤが発生できる加減速度（グリップ
限界を含む）は、モータのトルク仕様とは別物になりうる（`sim/vehicle.py`の
モジュールdocstring「縦加速度の上限」参照）。`mu`と同じ土俵で実測することで、
摩擦円連成が実車に対してどこまで正しく効いているかを検証できる。

## TC（トラクションコントロール）介入込みの値を測る

実機STM32にはTC/TVが実装済み（`docs/architecture.md`）で、`torque_cmd`は
「TC適用後の最終指令値」——ここで測る`accel_x`は、モータのトルク仕様値
ではなく**TCが実際に許した範囲での加減速度**になる（シムはタイヤモデルを
持たずTC自体は再現していないので、その結果だけを実測値として折り込む
狙い）。`tools/sysid/fit.py`の`fit_accel()`は`VehicleState.tc_active`を見て、
加速・制動の両フェーズで実際にTCが介入したかを確認する（介入していなければ
「タイヤが滑り出す本当の上限」を測れていない可能性が高いのでエラーにする）。

**TCのゲイン（`DRIVE_TC_CUT_GAIN`/`DRIVE_TC_RECOVER_RATE`）は実機で調整中**
（`docs/uart_protocol.md` v0.13の変更履歴参照）。測定後にゲインを再調整したら、
`drive_accel_m_s2`/`brake_decel_m_s2`も変わりうるので測り直すこと。

## 直進のみ・後輪駆動なので`sysid_speed`の延長

`target_steer`は常に0（直進）。加速→制動の繰り返しは`sysid_speed`の
前進⇄後退ステップと似た構成だが、こちらは「速度応答の時定数」ではなく
「実際に出た加速度そのもの」を見るため、`brake=True`による急制動フェーズを
明示的に挟む。

## 加速→制動のあと、後退して元の位置へ戻ってから次のサイクルへ（2026-09-01追加）

以前は加速→制動を`cycles`回繰り返すだけで、車体は毎サイクル正味前進し続け、
必要な走行スペースが繰り返し回数ぶん伸びた（`target_speed`最大3.0m/s×
`accel_hold_s`最大5.0sだと1サイクルだけで10m以上進みうる）。バンビの
「スペースがあまりない」という指摘を受け、制動フェーズの後に**測定不要の
「戻り」フェーズ**を挟み、`VehicleState.odom_dist`（前輪の累積走行距離。
このプランナーは`target_steer`が常に0なので射影補正なしでそのまま前後方向の
変位として使える）を見ながらサイクル開始位置まで後退させる。`return_speed`
で指定した速度まで`target_speed`を負にして走らせるだけ（`sysid_speed.py`の
後退ステップと同じ仕組み）で、`brake`フラグは立てない。

**戻りフェーズはmcapの自動測定（`tools/sysid/fit.py`の`fit_accel()`）に
影響しない**——`fit_accel()`の集計フィルタは
`not s.brake and s.target_speed > 0.05`（加速側）・`s.brake`（制動側）で、
戻りフェーズは`target_speed`が負（`> 0.05`を満たさない）かつ`brake=False`
なので、どちらのフィルタにも一度も引っかからず自動的に除外される
（`_check_tc_engaged`の判定条件も同様）。mcap自体には戻り区間のサンプルも
そのまま記録される（`logger_node.py`は区間を選ばず全部録る設計）が、
解析側が無視するので実害は無い。

オドメトリが更新されない等の異常で戻りきれない場合に無限後退させないよう、
制動フェーズ終了時点での前進距離から`_return_timeout_s()`で安全装置の
上限時間を計算し、それを超えたら測れていなくても次のサイクルへ進む
（`sysid_corner`等の他の安全装置と同じ「測定不能なら止まらず進める」設計）。

## 試験開始/中止を押すまでステップは進まない

他の`sysid_*`プランナーと同じ`TestGate`（`_sysid_common.py`）を使う。
詳しい経緯はそちらのdocstring参照。
"""

from __future__ import annotations

from ..msgs.types import AutoState, Scan, VehicleState
from ._sysid_common import TestGate
from .base import ParamSpec, Planner

__all__ = ["SysIdAccel"]


class SysIdAccel(Planner):
    id = "sysid_accel"
    name = "システム同定: 加減速"
    description = "直進で加速→急制動を繰り返し、drive_accel_m_s2・brake_decel_m_s2を測る"
    category = "sysid"
    stats = ()

    #: フェーズ番号。`_PHASE_LABELS`・`plan()`と対応
    _ACCEL, _BRAKE, _RETURN = 0, 1, 2
    _PHASE_LABELS = {_ACCEL: "加速", _BRAKE: "制動", _RETURN: "戻り"}

    #: 戻りフェーズの「元の位置」判定の許容誤差 [m]
    _RETURN_TOLERANCE_M = 0.05
    #: 戻りフェーズの安全装置（オドメトリ不調等で戻りきれない場合）の下限時間 [s]。
    #: 実際の上限は`_return_timeout_s()`で進んだ距離から計算する
    _RETURN_TIMEOUT_MIN_S = 5.0

    params = (
        ParamSpec(key="target_speed", label="目標速度", min=0.3, max=3.0, step=0.1,
                  default=1.5, unit="m/s",
                  note="この速度まで加速してから急制動する。速いほど加速度の頭打ちを"
                       "踏みやすいが、加速・惰行・制動距離ぶんのオープンスペースが要る"),
        ParamSpec(key="accel_hold_s", label="加速フェーズの保持時間", min=0.5, max=5.0, step=0.1,
                  default=2.0, unit="s",
                  note="目標速度に達してからもこの時間だけ走り続ける（速度指令の"
                       "一次遅れが収束しきるまでの余裕）"),
        ParamSpec(key="brake_hold_s", label="制動フェーズの保持時間", min=0.5, max=5.0, step=0.1,
                  default=1.5, unit="s",
                  note="完全停止した後もこの時間だけ制動を維持する"),
        ParamSpec(key="return_speed", label="戻り速度", min=0.2, max=1.5, step=0.1,
                  default=0.6, unit="m/s",
                  note="制動フェーズの後、次のサイクルの前にサイクル開始位置まで"
                       "後退する速さ。測定には使わない区間（mcapの自動測定には"
                       "影響しない——drive_accel_m_s2/brake_decel_m_s2の集計は"
                       "target_speed>0またはbrakeの区間だけを見る）"),
        ParamSpec(key="cycles", label="繰り返し回数", min=1, max=8, step=1,
                  default=4, unit="回",
                  note="加速→制動→戻りを1組として繰り返す回数。多いほど"
                       "フィッティングが安定する"),
    )

    def __init__(self) -> None:
        self._gate = TestGate()
        self._t = 0.0
        self._cycle_i = 0
        self._phase = self._ACCEL
        #: サイクル開始時点の前輪オドメトリ [m]（`VehicleState.odom_dist`の平均）。
        #: `target_steer`は常に0なので射影補正なしでそのまま前後方向の変位として使える
        self._origin_odom: float | None = None
        self._return_timeout_s = self._RETURN_TIMEOUT_MIN_S

    def reset(self) -> None:
        self._gate.reset()
        self._t = 0.0
        self._cycle_i = 0
        self._phase = self._ACCEL
        self._origin_odom = None
        self._return_timeout_s = self._RETURN_TIMEOUT_MIN_S

    def set_engaged(self, engaged: bool) -> None:
        """`planning_node.py`がplan()の直前に呼ぶ（ダックタイピング）。"""
        self._gate.set_engaged(engaged)

    @staticmethod
    def _front_odom_m(vs: VehicleState) -> float:
        return (vs.odom_dist[0] + vs.odom_dist[1]) / 2.0

    def plan(self, scan: Scan, vs: VehicleState | None,
             p: dict[str, float], dt: float) -> AutoState:
        st = AutoState(mode=self.id, planner=self.name)
        st.ready = True
        st.target_steer = 0.0

        engaged, armed, just_started = self._gate.tick(vs)
        if just_started:
            self._t = 0.0
            self._cycle_i = 0
            self._phase = self._ACCEL
            self._origin_odom = None
            self._return_timeout_s = self._RETURN_TIMEOUT_MIN_S
        if not engaged:
            st.target_speed = 0.0
            st.reason = "試験開始を押してください"
            return st
        if not armed:
            st.target_speed = 0.0
            st.reason = "ARM待ち（Enterを押してください）"
            return st
        # `armed`が真なら`TestGate.tick()`の実装上vsは必ずある
        assert vs is not None
        if self._origin_odom is None:
            self._origin_odom = self._front_odom_m(vs)

        n_cycles = int(p["cycles"])
        if self._cycle_i >= n_cycles:
            st.target_speed = 0.0
            st.reason = "完了"
            return st

        self._t += dt
        if self._phase == self._ACCEL:
            done = self._t >= p["accel_hold_s"]
        elif self._phase == self._BRAKE:
            done = self._t >= p["brake_hold_s"]
            if done:
                # 戻りフェーズの安全装置——ここまで進んだ距離から、余裕を持った
                # タイムアウトを逆算する（`return_speed`で戻りきる時間の3倍+2秒）
                traveled = self._front_odom_m(vs) - self._origin_odom
                self._return_timeout_s = max(
                    self._RETURN_TIMEOUT_MIN_S, abs(traveled) / p["return_speed"] * 3.0 + 2.0)
        else:  # _RETURN
            traveled = self._front_odom_m(vs) - self._origin_odom
            done = traveled <= self._RETURN_TOLERANCE_M or self._t >= self._return_timeout_s

        if done:
            self._t = 0.0
            self._phase = {self._ACCEL: self._BRAKE, self._BRAKE: self._RETURN,
                           self._RETURN: self._ACCEL}[self._phase]
            if self._phase == self._ACCEL:
                self._cycle_i += 1
                self._origin_odom = self._front_odom_m(vs)
                if self._cycle_i >= n_cycles:
                    st.target_speed = 0.0
                    st.reason = "完了"
                    return st

        if self._phase == self._ACCEL:
            st.target_speed = p["target_speed"]
        elif self._phase == self._BRAKE:
            st.brake = True
        else:
            st.target_speed = -p["return_speed"]
        st.reason = f"{self._PHASE_LABELS[self._phase]} {self._cycle_i + 1}/{n_cycles}"
        return st
