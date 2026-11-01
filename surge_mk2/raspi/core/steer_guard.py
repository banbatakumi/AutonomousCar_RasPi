"""舵が指令に追従しないことを見つける（io_node が ARM を外してラッチする）。

2026-10-31 の会場の試走 `surge_20261031_062217` で、起動後の最初の操舵で舵が動かなかった:
指令（STM32 のエコー）+0.21 → −0.15rad に対して実舵角は 0.00 のまま、ステア MD の電流は ±4.8A
（上限）に張り付き、ヨーレートもほぼ 0。そのまま 1.4 秒まっすぐ走って壁に当たった（17.5m/s²）。
衝突後も −0.19rad で 1.5 秒固まり、後退したら外れた。**STM32 も Pi も何も言わなかった。**

判定: **ARM 中に「|指令 − 実舵角| > `ERR_RAD`」かつ「|ステア MD の電流| ≥ `CURRENT_A`」が `HOLD_S` 続いた。**
9 本の記録で、正常な走行（据え切り・急な切り返しを含む）でこの状態が続いたのは最長 0.06 秒、
062217 の固着は 0.51 秒・1.63 秒。0.25 秒なら 062217 は衝突の約 0.7 秒前に止まる。
電流を条件に入れるのは、指令の急な変化に舵が追いつく途中（電流は上限まで行かない）と、
舵が「押しているのに動かない」を分けるため。
"""

from __future__ import annotations

__all__ = ["CURRENT_A", "ERR_RAD", "HOLD_S", "SteerGuard"]

#: 指令と実舵角の差がこれを超えたら「追従していない」[rad]（約 4.6°）
ERR_RAD = 0.08
#: ステア MD の電流がこれ以上なら「押している」[A]。上限は約 4.8A（記録の最大 4.8〜5.1A）
CURRENT_A = 3.5
#: 上の 2 つが続いたら止める時間 [s]
HOLD_S = 0.25


class SteerGuard:
    """TELEMETRY を 1 つずつ渡し、止めるべきになったら `update` が True を返す（1 回だけ）。"""

    def __init__(self, err_rad: float = ERR_RAD, current_a: float = CURRENT_A,
                 hold_s: float = HOLD_S) -> None:
        self.err_rad = err_rad
        self.current_a = current_a
        self.hold_ns = int(hold_s * 1e9)
        self._since_ns: int | None = None
        #: 止めた理由（GUI・記録に出す）。空なら止めていない
        self.reason = ""

    def reset(self) -> None:
        """ARM を外した（人が入れ直す）。次の判定を始めからにする。"""
        self._since_ns = None
        self.reason = ""

    def update(self, t_ns: int, armed: bool, cmd_rad: float, actual_rad: float,
               current_a: float) -> bool:
        """:return: この呼び出しで止めるべきになったら True（止めた後は `reset` まで False）"""
        if self.reason:
            return False
        err = abs(cmd_rad - actual_rad)
        if not armed or err <= self.err_rad or abs(current_a) < self.current_a:
            self._since_ns = None
            return False
        if self._since_ns is None:
            self._since_ns = t_ns
        if t_ns - self._since_ns < self.hold_ns:
            return False
        self.reason = (f"舵が指令に追従しない（指令 {cmd_rad:+.2f}rad に実舵角 {actual_rad:+.2f}rad、"
                       f"ステア MD {current_a:+.1f}A が {(t_ns - self._since_ns) / 1e9:.2f}s 続いた）")
        return True
