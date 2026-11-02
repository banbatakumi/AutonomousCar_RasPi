"""舵が指令に追従しないことを見つけ、その場で止めて待つ（io_node が制動・再開・DISARM する）。

2026-10-31 の会場の試走 `surge_20261031_062217` で、起動後の最初の操舵で舵が動かなかった:
指令（STM32 のエコー）+0.21 → −0.15rad に対して実舵角は 0.00 のまま、ステア MD の電流は ±4.8A
（上限）に張り付き、ヨーレートもほぼ 0。そのまま 1.4 秒まっすぐ走って壁に当たった（17.5m/s²）。
衝突後も −0.19rad で 1.5 秒固まり、後退したら外れた。**STM32 も Pi も何も言わなかった。**

2026-11-02 の `surge_20261102_094700` でも走り出しの操舵で同じことが起きた（指令 +0.16 に実舵角 0.015、
4.8A、1.4m/s でヨーレート 0.1rad/s＝舵 0.02rad 相当。機構の点検では異常なし）。3 回目の ARM の後、
指令 0 のまま 4.8A で押している向きと**逆へ** −0.34rad まで動いてから戻り、以後は正常に追従した
（MD の転流がずれてトルクが出ていない／逆に出ている形。外れ方は時間で直ることがある）。

判定（`update`）: **ARM 中に「|指令 − 実舵角| > `ERR_RAD`」かつ「|ステア MD の電流| ≥ `CURRENT_A`」かつ
「|車速| ≥ `MIN_SPEED`」が `HOLD_S` 続いた。** 電流を条件に入れるのは、指令の急な変化に舵が追いつく途中
（電流は上限まで行かない）と、舵が「押しているのに動かない」を分けるため。車速を入れるのは、停車中の
据え切り（床との摩擦で上限電流のまま届かない。`surge_20261102_095106` 214.6s で 0.25s 続いた）を外すため。
停車中に舵が動かなくても危なくない。走り出せば 0.25s で見つかる。

見つけたら `PAUSE`（io_node が最大制動で止め、**ARM のまま**舵の指令を見つけた時の値で据え置く）。
据え置いた指令に実舵角が `RESUME_S` 追従したら `RESUME`（指令を通常へ戻す。走行は続く）。
`PAUSE_MAX_S` 待っても追従しない、または 1 回の ARM の間に `MAX_TRIPS` 回目を見つけたら `GIVE_UP`
（io_node が DISARM し、ARM を入れ直すまでラッチ。直らない故障で走り続けない）。

16 本の記録（2026-10-30〜11-02）の再生: 正常な走行で条件が続いたのは最長 0.04 秒（車速の条件が無いと
095106 の据え切りで 0.25 秒）。062217 は 60.89s（衝突の約 0.7 秒前）、094700 は 10.01s（1.7m/s）で止める。
"""

from __future__ import annotations

__all__ = ["CURRENT_A", "ERR_RAD", "GIVE_UP", "HOLD_S", "MAX_TRIPS", "MIN_SPEED", "PAUSE",
           "PAUSE_MAX_S", "RESUME", "RESUME_S", "SteerGuard"]

#: 指令と実舵角の差がこれを超えたら「追従していない」[rad]（約 4.6°）
ERR_RAD = 0.08
#: ステア MD の電流がこれ以上なら「押している」[A]。上限は約 4.8A（記録の最大 4.8〜5.1A）
CURRENT_A = 3.5
#: 車速の絶対値がこれ未満の間は判定しない [m/s]（停車中の据え切りを外す）
MIN_SPEED = 0.3
#: 上の 3 つが続いたら止める時間 [s]
HOLD_S = 0.25
#: 止めた後、据え置いた指令に実舵角がこれだけ続けて追従したら再開する [s]
RESUME_S = 0.3
#: 止めてからこれだけ待っても追従しなければ諦めて DISARM [s]（上限電流で押し続けるとステア MD が熱を持つ）
PAUSE_MAX_S = 5.0
#: 1 回の ARM の間にこの回数目を見つけたら、止めて待たずに DISARM
MAX_TRIPS = 3

#: `update` の戻り値
PAUSE = "pause"
RESUME = "resume"
GIVE_UP = "give_up"


class SteerGuard:
    """TELEMETRY を 1 つずつ渡す。状態が変わったら `update` が `PAUSE`・`RESUME`・`GIVE_UP` を返す。"""

    def __init__(self, err_rad: float = ERR_RAD, current_a: float = CURRENT_A,
                 hold_s: float = HOLD_S, min_speed: float = MIN_SPEED,
                 resume_s: float = RESUME_S, pause_max_s: float = PAUSE_MAX_S,
                 max_trips: int = MAX_TRIPS) -> None:
        self.err_rad = err_rad
        self.current_a = current_a
        self.hold_ns = int(hold_s * 1e9)
        self.min_speed = min_speed
        self.resume_ns = int(resume_s * 1e9)
        self.pause_max_ns = int(pause_max_s * 1e9)
        self.max_trips = max_trips
        self._since_ns: int | None = None
        #: 止めている（`PAUSE` から `RESUME`・`GIVE_UP` まで）
        self.paused = False
        self._pause_ns = 0
        self._ok_since_ns: int | None = None
        #: この ARM の間に見つけた回数
        self.trips = 0
        #: 直近に止めた理由（GUI・記録に出す）
        self.message = ""
        #: 諦めた理由。空でなければ `reset` まで何も返さない
        self.reason = ""

    def reset(self) -> None:
        """ARM を外した（人が入れ直す）。次の判定を始めからにする。"""
        self._since_ns = None
        self.paused = False
        self._ok_since_ns = None
        self.trips = 0
        self.message = ""
        self.reason = ""

    def update(self, t_ns: int, armed: bool, cmd_rad: float, actual_rad: float,
               current_a: float, speed_m_s: float) -> str:
        """:return: この呼び出しで状態が変わったら `PAUSE`・`RESUME`・`GIVE_UP`、変わらなければ空"""
        if self.reason or not armed:
            return ""
        err = abs(cmd_rad - actual_rad)
        if self.paused:
            if err <= self.err_rad:
                if self._ok_since_ns is None:
                    self._ok_since_ns = t_ns
                if t_ns - self._ok_since_ns >= self.resume_ns:
                    self.paused = False
                    self._since_ns = None
                    return RESUME
            else:
                self._ok_since_ns = None
            if t_ns - self._pause_ns >= self.pause_max_ns:
                self.paused = False
                self.reason = f"{self.message}。{(t_ns - self._pause_ns) / 1e9:.0f}s 待っても追従しない"
                return GIVE_UP
            return ""
        if err <= self.err_rad or abs(current_a) < self.current_a or abs(speed_m_s) < self.min_speed:
            self._since_ns = None
            return ""
        if self._since_ns is None:
            self._since_ns = t_ns
        if t_ns - self._since_ns < self.hold_ns:
            return ""
        self.trips += 1
        self.message = (f"舵が指令に追従しない（指令 {cmd_rad:+.2f}rad に実舵角 {actual_rad:+.2f}rad、"
                        f"ステア MD {current_a:+.1f}A が {(t_ns - self._since_ns) / 1e9:.2f}s 続いた）")
        self._since_ns = None
        if self.trips >= self.max_trips:
            self.reason = f"{self.message}。この ARM で {self.trips} 回目"
            return GIVE_UP
        self.paused = True
        self._pause_ns = t_ns
        self._ok_since_ns = None
        return PAUSE
