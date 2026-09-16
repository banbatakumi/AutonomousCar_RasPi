"""平面姿勢の最小限の代数 — 合成・相対姿勢・角度の畳み込み。

## なぜ`slam2d.core.types`を使わないのか

同じ内容（`Pose2D`とその合成）は`slam2d/core/types.py`にもあり、
`raspi/auto/slam2d_raceline.py`はそちらを使う。しかし**`park_to_point`は
`slam2d`に依存させない**方針にした（2026-09-16、バンビの判断）:

- `slam2d`は大域整合性の機構（ポーズグラフ・ループ閉じ・`g2opy`）を含む
  ライブラリで、現状これが不安定。駐車が必要とするのは2フレームの局所登録
  （`raspi/nav/scan2scan.py`）だけで、大域整合性は一切要らない
- 駐車planner が`slam2d`をimportすると、そのパッケージが入っていない実機
  構成で**駐車モードごと起動しなくなる**。依存の方向を減らす方が壊れにくい

姿勢の積分そのものは`raspi/nav/deskew.py`の`integrate_pose()`（円弧で積む
自転車モデル1ステップ）が既にあるので、ここには持たない。**同じ式を2箇所に
書かない。**

角度は全て [rad]・反時計回り正、座標は x=前 / y=左（`docs/architecture.md` §5.1）。
"""

from __future__ import annotations

import math

__all__ = ["Pose", "wrap_angle", "compose", "between"]

#: 平面姿勢 `(x, y, yaw)`。**dataclassにしない**——`tuple`のままなら
#: `integrate_pose()`の戻り値をそのまま渡せて、変換の層が増えない
Pose = tuple[float, float, float]


def wrap_angle(a: float) -> float:
    """角度を `(-π, π]` へ畳む。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def compose(a: Pose, b: Pose) -> Pose:
    """`a`フレームで表した`b`を、`a`の親フレームへ移す（SE(2)の積 `a ∘ b`）。"""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + b[0] * c - b[1] * s,
            a[1] + b[0] * s + b[1] * c,
            wrap_angle(a[2] + b[2]))


def between(a: Pose, b: Pose) -> Pose:
    """`a`から見た`b`（どちらも同じ親フレームでの姿勢）＝ `a⁻¹ ∘ b`。"""
    c, s = math.cos(-a[2]), math.sin(-a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return (c * dx - s * dy, s * dx + c * dy, wrap_angle(b[2] - a[2]))
