"""システム同定 — 旋回グリップ試験（`config/vehicle.toml` `[dynamics]` の実測手順）。

固定舵角のまま `target_speed` を段階的に上げて円旋回させ、指令曲率
`tan(steer_actual)/wheelbase` と実測曲率 `yaw_rate/speed`（`VehicleState`）が
乖離し始める速度から `mu`（横方向グリップ限界）を測る。3試験の中で唯一
車両が実際に走り回るためリスクが最も高い——オープンスペースが必須で、
既存の安全機構（人間の ARM 保持・超音波 `auto_stop`・GUI の disengage）に
加えて、**周囲クリアランスを LiDAR で見ながら段階を進める**追加の安全層を
プランナー自身が持つ（`FollowTheGap` などの通常 planner と同じ
`scan_window()` を流用）。

## 舵角は`max_steer`まで振ってよい（2026-08-31、バンビの指摘）

舵角を大きくするほど旋回半径が小さくなり（曲率 `tan(steer)/wheelbase`）、
**より低い速度でグリップ限界（`mu*g`）に到達できる**——`sysid_steer.py`と
同じ理由で、ここでも上限いっぱいまで振った方がよい（むしろ低い速度で
飽和を踏める分、安全にもつながる）。`target_steer`は`plan()`で
`self.vehicle.max_steer`にクランプする。

## 試験開始/中止を押すまで段は進まない（2026-08-31、実機での不具合修正）

試験開始を押す前から勝手に段が進む・試験中止を押しても止まらない・
ARM保持中に試験開始し直せない、という3つの不具合があった。`TestGate`
（`_sysid_common.py`）で`engaged`（試験開始/中止の状態）を直接ゲートに使う。
詳しい経緯はそちらのdocstring参照。

## 速度は「刻み幅」ではなく「最終段の目標速度」で指定する（2026-08-31、バンビの指摘）

最初の実装は`v_step`（1段ごとの刻み）で、最終的にどこまで速度が上がるかは
`v_start + (stages-1)*v_step`を暗算しないと分からなかった。グリップの
頭打ちに到達するには十分な速度まで振る必要がある（`tools/sysid/fit.py`の
`_check_corner_saturated()`参照）ので、**「最後の段でいくつまで出すか」を
直接指定できる方が試験を組み立てやすい**。`v_max`（最終段の目標速度）を
`ParamSpec`にし、刻み幅は`(v_max - v_start) / (stages - 1)`で内部計算する。
"""

from __future__ import annotations

import math

from ..core.vehicle import Vehicle
from ..msgs.types import AutoState, Scan, VehicleState
from ._sysid_common import TestGate
from .base import ParamSpec, Planner, scan_window

__all__ = ["SysIdCorner"]


class SysIdCorner(Planner):
    id = "sysid_corner"
    name = "システム同定: 旋回グリップ"
    description = "固定舵角で速度を段階的に上げながら円旋回し、muを測る。オープンスペース必須"
    category = "sysid"
    stats = ("nearest",)

    params = (
        ParamSpec(key="steer_deg", label="固定舵角", min=10.0, max=30.0, step=1.0,
                  default=30.0, unit="°",
                  note="この舵角のまま円旋回する（実際の舵角はmax_steerで頭打ちにする）。"
                       "大きいほど旋回半径が小さくなり、より低い速度でグリップ限界に到達できる"),
        ParamSpec(key="v_start", label="開始速度", min=0.1, max=1.0, step=0.05,
                  default=0.2, unit="m/s",
                  note="最初の段の速度。低めから始めて安全に確認する"),
        ParamSpec(key="v_max", label="最終段の目標速度", min=0.3, max=3.0, step=0.1,
                  default=2.0, unit="m/s",
                  note="最後の段でこの速度まで到達する（刻み幅は段数から自動計算）。"
                       "滑り出す速度の目安は sqrt(mu*g*wheelbase/tan(steer_deg))——"
                       "vehicle.tomlの仮値mu=0.8・steer_deg=30°なら約1.77m/s。実際のタイヤは"
                       "もっと効く場合もあるので少し高めの2.0を既定にしてある。"
                       "グリップの頭打ちに届かないと解析ツールがエラーで教えてくれるので、"
                       "その場合はここを上げて録り直すこと"),
        ParamSpec(key="stage_s", label="1段の保持時間", min=1.0, max=8.0, step=0.5,
                  default=3.0, unit="s",
                  note="各速度段でこれだけ円旋回を続け、yaw_rate/speedを安定させる"),
        ParamSpec(key="stages", label="段数", min=2, max=15, step=1,
                  default=10, unit="段",
                  note="開始速度から最終段の目標速度までを何段に分けるか。muの精度自体には"
                       "あまり効かない（v_maxが頭打ちに届くかどうかの方が本質）が、多いほど"
                       "1段あたりの速度刻みが細かくなり、滑り出しに穏やかに近づける"),
        ParamSpec(key="margin_m", label="安全マージン", min=0.1, max=1.0, step=0.05,
                  default=0.3, unit="m",
                  note="全周でこの距離を切ったら即座に停止する"),
    )

    def __init__(self) -> None:
        self.vehicle = Vehicle.load()
        self._gate = TestGate()
        self._t = 0.0
        self._stage = 0

    def reset(self) -> None:
        self._gate.reset()
        self._t = 0.0
        self._stage = 0

    def set_engaged(self, engaged: bool) -> None:
        """`planning_node.py`がplan()の直前に呼ぶ（ダックタイピング）。"""
        self._gate.set_engaged(engaged)

    def plan(self, scan: Scan, vs: VehicleState | None,
             p: dict[str, float], dt: float) -> AutoState:
        st = AutoState(mode=self.id, planner=self.name)

        engaged, armed, just_started = self._gate.tick(vs)
        if just_started:
            self._t = 0.0
            self._stage = 0
        if not engaged:
            st.ready = True
            st.reason = "試験開始を押してください"
            return st
        if not armed:
            st.ready = True
            st.reason = "ARM待ち（Enterを押してください）"
            return st

        # ── 安全: 全周のクリアランスを見る。切れたら段を進めていても即座に停止 ──
        w = scan_window(scan, 360.0, max(1.0, p["margin_m"] * 4))
        nearest = min(w.dist) if w.dist else 0.0
        st.nearest = nearest
        if nearest < p["margin_m"]:
            st.reason = f"周囲クリアランス不足（最近傍 {nearest * 100:.0f}cm）で停止"
            return st                          # ready=False ＝ 制動

        stages = int(p["stages"])

        if self._stage >= stages:
            st.reason = "完了"
            return st

        self._t += dt
        if self._t >= p["stage_s"]:
            self._t = 0.0
            self._stage += 1
            if self._stage >= stages:
                st.reason = "完了"
                return st

        steer = min(math.radians(p["steer_deg"]), self.vehicle.max_steer)
        # 刻み幅は「開始速度→最終段の目標速度」を段数-1等分して逆算する。
        # stages=1なら刻みようがないのでv_startのまま（下のmax(1,...)で0除算を避ける）。
        # v_maxをv_start未満に設定してしまった場合、負の刻みで速度が段ごとに
        # 下がっていく（＝意図と逆向きにスイープする）事故を避けるため0で床を打つ
        v_step = max(0.0, p["v_max"] - p["v_start"]) / max(1, stages - 1)
        speed = p["v_start"] + self._stage * v_step
        st.target_steer = steer
        st.target_speed = speed
        st.ready = True
        st.reason = f"段 {self._stage + 1}/{stages}（{speed:.2f}m/s・舵角{p['steer_deg']:.0f}°）"
        return st
