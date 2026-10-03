"""カメラ系知覚ノード（cam_perception / cam_e2e / line_perception）の
ACTIVE/IDLE 判定（省電力ゲート）を1箇所に集約する。

3ノードとも「`auto/ctrl`（`AutoCtrl.mode`）が該当モードで選ばれている間だけ推論する」
という同じ設計を持つが、これは `VehicleState.armed` を一切見ないため、DISARM中
（駐車中の待機）でもモードが選ばれているだけで CNN 推論が回り続ける省電力上のバグに
なっていた。**DISARM中のみ推論を止める。ARM中は engage の有無に関わらず現状維持**
（各ノードの docstring にある「engage する前に走行開始ボタンを押さずにモデルだけ
選び直せる」という ARM 中限定の利便性を壊さないため）。

`telemetry_node._vehicle_armed()` と同じ規約：`VehicleState` 未受信（`vs is None`）は
「わからない」を安全側＝ARM相当として扱う（起動直後の一瞬だけ推論が止まって見える
回帰を避けるため）。3ノードに同じロジックを複製すると、この規約が将来ノードごとに
食い違う恐れがあるのでここに集約する。
"""

from __future__ import annotations

import time
from collections.abc import Container

from raspi.msgs import AutoCtrl, VehicleState

__all__ = ["vehicle_armed", "cam_infer_active", "IdlePacer"]

#: IDLE中のループ周期。ACTIVE中は `poll(20)` が vehicle_state（100Hz）や
#: image/front（30Hz）の到着で毎回起きるので約130Hzで回る。IDLE中はそれを
#: 待たずにこの間隔で眠る。ハートビート（10Hz）・ROI選択などへの反応はこれで十分
IDLE_SLEEP_S = 0.05
#: IDLE中の結果（failed_frame 等）の publish 間隔。購読側（planner・GUI）が
#: 「止まっている」と分かれば足りる。ACTIVE→IDLE へ移った直後は即座に1回送る
IDLE_PUBLISH_PERIOD_NS = 500_000_000


def vehicle_armed(vs: VehicleState | None) -> bool:
    """`vs is None`（`VehicleState` 未受信）は安全側＝ARM相当として扱う。"""
    return True if vs is None else bool(vs.armed)


def cam_infer_active(auto_ctrl: AutoCtrl | None, vs: VehicleState | None,
                      modes: Container[str]) -> bool:
    """`auto_ctrl.mode` が `modes` のいずれかで選ばれており、かつ DISARM 中でない間だけ True。"""
    mode_selected = auto_ctrl is not None and auto_ctrl.mode in modes
    return mode_selected and vehicle_armed(vs)


class IdlePacer:
    """カメラ系ノードの IDLE 中の起床・publish 頻度を落とす（2026-10-04、省電力）。

    以前は IDLE 中も約130Hzで起き、中身の無い結果（failed_frame 等）を毎回
    publish していた。購読側（planning_node は全 planner の入力を購読している）も
    同じ回数だけ受けて捨てる。4ノードを止めると制御系電流が約19mA下がることを
    実測した（docs/power_audit_2026-10.md P6）。

    使い方（各ノードの `run()`）::

        for _ in sub.poll(pacer.poll_timeout_ms(self._active)): ...
        ...
        if pacer.should_publish(active, now): pub.send(...)
        ...
        pacer.idle_sleep(active)        # ループの末尾
    """

    __slots__ = ("_next_pub_ns",)

    def __init__(self) -> None:
        self._next_pub_ns = 0

    @staticmethod
    def poll_timeout_ms(active: bool) -> int:
        # IDLE 中は末尾の `idle_sleep()` で眠るので、poll は溜まった分を拾うだけ
        return 20 if active else 0

    def should_publish(self, active: bool, now_ns: int) -> bool:
        if active:
            self._next_pub_ns = 0          # 次に IDLE へ入った瞬間に1回送る
            return True
        if now_ns >= self._next_pub_ns:
            self._next_pub_ns = now_ns + IDLE_PUBLISH_PERIOD_NS
            return True
        return False

    @staticmethod
    def idle_sleep(active: bool) -> None:
        if not active:
            time.sleep(IDLE_SLEEP_S)
