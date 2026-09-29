"""走行中の経路切替と、周回数・時間で終わるミッション。

## 切替は「乗れる場所まで保留」（Autoware の reroute と同じ考え方）

経路のグループ（A〜D）ごとにレーシングラインを事前に作っておき、走行中は
それを差し替える。差し替えは要求した瞬間ではなく、次の条件がすべてそろった
周期に行う:

1. 車が新しい経路の上に居る（横のずれ `tol` 以内・向きの差 `yaw_tol` 以内）
2. 新しい経路のこの先 `horizon` の目標速度まで、今の速度から `a_brake` で
   減速し切れる（切り替えた瞬間に急制動しない）

こうすると意味どおりに動く:

- **分岐の手前**で要求すれば、両経路はまだ重なっているのでその場で乗り換え、
  その分岐から新しい枝へ入る
- **分岐を過ぎてから**要求すれば、両経路が合流したところで乗り換え、次の周の
  分岐から新しい枝へ入る（今の枝を無理に引き返さない）

経路どうしは別々に最適化しているので、重なっている区間でも数cmずれる。
`tol`（既定15cm）はそれを飲む幅で、Pure Pursuit は数cmの段差なら舵を跳ねさせない。

## ミッション

`{"laps": n, "time_s": t, "then": "P1"}` —— n周（または t 秒）走ったら
停止点 P1 へ向かって止まる。周回は今の経路の添字が末尾→先頭へ回ったことで
数える（地図作成の周回判定のような回頭の積算は要らない。経路の上に居るので）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .purepursuit import nearest_index
from .raceline import RaceLine

__all__ = ["RouteSwitcher", "Mission", "SwitchResult", "lap_crossed"]


@dataclass
class SwitchResult:
    switched: bool = False
    hint: int = -1                          #: 乗り換えた先の経路で使う添字
    reason: str = ""                        #: 保留中の理由（GUI 用）


class RouteSwitcher:
    """今の経路と、乗り換え待ちの経路。"""

    def __init__(self) -> None:
        self.active_key = ""
        self.active: RaceLine | None = None
        self.pending_key = ""
        self.pending: RaceLine | None = None
        self.source = ""                    #: 最後に切替を要求したのは誰か（"gui"/"signal"等）

    def reset(self) -> None:
        self.__init__()

    def set_active(self, key: str, line: RaceLine) -> None:
        """保留なしで今の経路にする（走り出す前・停止経路へ移るとき）。"""
        self.active_key, self.active = key, line
        self.pending_key, self.pending = "", None

    def request(self, key: str, line: RaceLine, source: str = "") -> None:
        if self.active is None:
            self.set_active(key, line)
            self.source = source
            return
        if key == self.active_key and line is self.active:
            self.pending_key, self.pending = "", None      # 今の経路に戻す要求＝取り消し
            self.source = source
            return
        self.pending_key, self.pending = key, line
        self.source = source

    def step(self, pose: tuple[float, float, float], v_now: float, *, tol: float,
             yaw_tol: float = math.radians(25.0), a_brake: float = 1.8,
             horizon: float = 1.5) -> SwitchResult:
        new = self.pending
        if new is None:
            return SwitchResult()
        j = nearest_index(new, pose[0], pose[1])
        d = float(math.hypot(new.xy[j, 0] - pose[0], new.xy[j, 1] - pose[1]))
        if d > tol:
            return SwitchResult(reason=f"{self.pending_key}へ乗り換える地点を待っている"
                                       f"（{d:.2f}m 離れている）")
        n = len(new)
        k = (j + 1) % n if new.closed else min(j + 1, n - 1)
        if k == j and j > 0:
            k, j = j, j - 1
        yaw = math.atan2(new.xy[k, 1] - new.xy[j, 1], new.xy[k, 0] - new.xy[j, 0])
        dyaw = abs((yaw - pose[2] + math.pi) % (2.0 * math.pi) - math.pi)
        if dyaw > yaw_tol:
            return SwitchResult(reason=f"{self.pending_key}へ乗り換える地点を待っている"
                                       f"（向きが{math.degrees(dyaw):.0f}°違う）")
        step = new.length / (n if new.closed else max(1, n - 1))
        ahead = max(1, int(horizon / max(step, 1e-3)))
        off = np.arange(ahead + 1)
        if not new.closed:
            off = off[j + off < n]
        idx = (j + off) % n
        # この先 s [m] の点の目標速度 v まで、今の速度から減速し切れるか
        s = off * step
        reach = np.sqrt(np.maximum(new.v[idx] ** 2 + 2.0 * a_brake * s, 0.0))
        if v_now > float(reach.min()) + 0.15:
            return SwitchResult(reason=f"{self.pending_key}の速度まで落とせないので待っている")
        self.active_key, self.active = self.pending_key, new
        self.pending_key, self.pending = "", None
        return SwitchResult(switched=True, hint=int(j))


@dataclass
class Mission:
    laps: int = 0                           #: 0 = 周回数では終わらない
    time_s: float = 0.0                     #: 0 = 時間では終わらない
    then: str = ""                          #: 終わったら向かう停止点の名前。空なら走り続ける

    @classmethod
    def from_dict(cls, d: dict | None) -> "Mission":
        d = d or {}
        return cls(laps=int(d.get("laps", 0) or 0), time_s=float(d.get("time_s", 0.0) or 0.0),
                   then=str(d.get("then", "") or ""))

    def active(self) -> bool:
        return bool(self.then) and (self.laps > 0 or self.time_s > 0.0)

    def due(self, laps_done: int, elapsed_s: float) -> bool:
        if not self.active():
            return False
        return ((self.laps > 0 and laps_done >= self.laps)
                or (self.time_s > 0.0 and elapsed_s >= self.time_s))

    def describe(self, laps_done: int, elapsed_s: float) -> str:
        if not self.active():
            return ""
        parts = []
        if self.laps > 0:
            parts.append(f"{laps_done}/{self.laps}周")
        if self.time_s > 0.0:
            parts.append(f"{elapsed_s:.0f}/{self.time_s:.0f}s")
        return "・".join(parts) + f" → {self.then}"


def lap_crossed(prev_idx: int, idx: int, n: int) -> bool:
    """閉じた経路の添字が末尾側から先頭側へ回ったか（1周した）。"""
    return prev_idx >= 0 and prev_idx > 0.75 * n and idx < 0.25 * n
