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

from collections.abc import Container

from raspi.msgs import AutoCtrl, VehicleState

__all__ = ["vehicle_armed", "cam_infer_active"]


def vehicle_armed(vs: VehicleState | None) -> bool:
    """`vs is None`（`VehicleState` 未受信）は安全側＝ARM相当として扱う。"""
    return True if vs is None else bool(vs.armed)


def cam_infer_active(auto_ctrl: AutoCtrl | None, vs: VehicleState | None,
                      modes: Container[str]) -> bool:
    """`auto_ctrl.mode` が `modes` のいずれかで選ばれており、かつ DISARM 中でない間だけ True。"""
    mode_selected = auto_ctrl is not None and auto_ctrl.mode in modes
    return mode_selected and vehicle_armed(vs)
