"""経路の設定（経由点のグループ・停止点・ミッション・信号との対応）。

地図1枚につき1つ、`saved_maps/<name>.routes.json` に置く（`mapstore.save_routes`）。
GUI の経由点エディタ・`sim.bench --routes`・planner がすべてこの形を読み書きする。

```json
{
  "groups": {"A": [[x, y], [x, y, yaw], ...], "B": [...]},
  "active": "A",
  "stops": {"P1": {"x": 1.0, "y": 2.0, "yaw": 1.57, "mode": "park"}},
  "mission": {"laps": 3, "time_s": 180, "then": "P1"},
  "signal_map": {"left": "B", "right": "A"},
  "explore_traj": [[x, y], ...]
}
```

- `groups` のキーは A〜D。経由点を1つも置いていないグループは存在しないのと同じ
- 経由点の3要素目 `yaw` は「その向きで通る」指定（省略可）
- `stops[*].mode`: `"stop"` はその場所で止まるだけ、`"park"` は手前で止まってから
  `park_to_point` へ引き継いでその姿勢（`yaw` 必須）へ入れる
- `signal_map`: カメラの信号認識（`route/select` トピック）が送ってくる値 → グループ
- `explore_traj`: 地図作成の軌跡（間引き済み）。**道路グラフの「通ってよい向き」
  （逆走禁止）を保存地図から復元するのに要る**（`nav/roadgraph._directions`）
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field

import numpy as np

from ..nav.route import Waypoint

__all__ = ["GROUPS", "RouteConfig", "Stop"]

GROUPS = ("A", "B", "C", "D")
_MAX_WAYPOINTS = 64
_MAX_TRAJ = 5000


@dataclass
class Stop:
    x: float
    y: float
    yaw: float | None = None
    mode: str = "stop"                      #: "stop" / "park"


@dataclass
class RouteConfig:
    groups: dict[str, list[Waypoint]] = field(default_factory=dict)
    active: str = "A"
    stops: dict[str, Stop] = field(default_factory=dict)
    mission: dict = field(default_factory=dict)
    signal_map: dict[str, str] = field(default_factory=dict)
    explore_traj: np.ndarray | None = None

    # ── 変換 ──

    @classmethod
    def from_dict(cls, d: dict | None) -> "RouteConfig":
        """検証しながら読む。**不正な値は ValueError（理由は日本語）。**"""
        d = d or {}
        if not isinstance(d, dict):
            raise ValueError("経路の設定がオブジェクトではない")
        groups: dict[str, list[Waypoint]] = {}
        for k, pts in (d.get("groups") or {}).items():
            if k not in GROUPS:
                raise ValueError(f"グループ名『{k}』は使えない（A〜D）")
            if not isinstance(pts, list) or len(pts) > _MAX_WAYPOINTS:
                raise ValueError(f"グループ{k}の経由点が不正（最大{_MAX_WAYPOINTS}個）")
            wps = [_waypoint(p, f"グループ{k}の経由点{i + 1}") for i, p in enumerate(pts)]
            if wps:
                groups[k] = wps
        stops = {}
        for name, s in (d.get("stops") or {}).items():
            if not isinstance(name, str) or not name or len(name) > 16:
                raise ValueError("停止点の名前が不正（1〜16文字）")
            if not isinstance(s, dict):
                raise ValueError(f"停止点『{name}』が不正")
            mode = str(s.get("mode", "stop"))
            if mode not in ("stop", "park"):
                raise ValueError(f"停止点『{name}』の mode は stop か park")
            yaw = s.get("yaw")
            if mode == "park" and yaw is None:
                raise ValueError(f"停止点『{name}』は駐車なので向き（yaw）が要る")
            stops[name] = Stop(x=_num(s.get("x"), name), y=_num(s.get("y"), name),
                               yaw=None if yaw is None else _num(yaw, name), mode=mode)
        mission = d.get("mission") or {}
        if not isinstance(mission, dict):
            raise ValueError("ミッションが不正")
        then = str(mission.get("then", "") or "")
        if then and then not in stops:
            raise ValueError(f"ミッションの行き先『{then}』という停止点が無い")
        mission = {"laps": max(0, int(mission.get("laps", 0) or 0)),
                   "time_s": max(0.0, float(mission.get("time_s", 0.0) or 0.0)),
                   "then": then}
        sig = {}
        for k, g in (d.get("signal_map") or {}).items():
            if g not in GROUPS:
                raise ValueError(f"信号『{k}』の行き先『{g}』はグループ名ではない")
            sig[str(k)] = str(g)
        active = str(d.get("active", "A") or "A")
        if active not in GROUPS:
            raise ValueError(f"グループ名『{active}』は使えない（A〜D）")
        traj = d.get("explore_traj")
        tr = None
        if traj is not None:
            tr = np.asarray(traj, dtype=np.float64)
            if tr.ndim != 2 or tr.shape[1] != 2 or len(tr) > _MAX_TRAJ \
                    or not np.isfinite(tr).all():
                raise ValueError("explore_traj が不正")
        return cls(groups=groups, active=active, stops=stops, mission=mission,
                   signal_map=sig, explore_traj=tr)

    @classmethod
    def from_json(cls, s: str) -> "RouteConfig":
        try:
            d = json.loads(s) if s else {}
        except ValueError as e:
            raise ValueError(f"経路の設定が JSON として読めない: {e}") from e
        return cls.from_dict(d)

    def to_dict(self, *, with_traj: bool = True) -> dict:
        d: dict = {
            "groups": {k: [_wp_list(w) for w in v] for k, v in self.groups.items()},
            "active": self.active,
            "stops": {k: {"x": s.x, "y": s.y, "yaw": s.yaw, "mode": s.mode}
                      for k, s in self.stops.items()},
            "mission": dict(self.mission),
            "signal_map": dict(self.signal_map),
        }
        if with_traj and self.explore_traj is not None:
            d["explore_traj"] = np.round(self.explore_traj, 3).tolist()
        return d

    def to_json(self, *, with_traj: bool = True) -> str:
        return json.dumps(self.to_dict(with_traj=with_traj), ensure_ascii=False)

    def same_groups(self, other: "RouteConfig") -> bool:
        """経路を作り直す必要があるか（経由点が変わったか）。"""
        return (self.to_dict(with_traj=False)["groups"]
                == other.to_dict(with_traj=False)["groups"])

    @staticmethod
    def decimate(traj: np.ndarray, spacing: float = 0.2) -> np.ndarray:
        xy = np.asarray(traj, dtype=np.float64)[:, :2]
        if len(xy) < 2:
            return xy
        out = [xy[0]]
        for p in xy[1:]:
            if math.hypot(*(p - out[-1])) >= spacing:
                out.append(p)
        return np.array(out[-_MAX_TRAJ:])


def _num(v, where: str) -> float:
    try:
        f = float(v)
    except (TypeError, ValueError) as e:
        raise ValueError(f"{where}の数値が不正") from e
    if not math.isfinite(f):
        raise ValueError(f"{where}の数値が不正")
    return f


def _waypoint(p, where: str) -> Waypoint:
    if not isinstance(p, (list, tuple)) or len(p) not in (2, 3):
        raise ValueError(f"{where}は [x, y] か [x, y, yaw]")
    yaw = None if len(p) == 2 or p[2] is None else _num(p[2], where)
    return Waypoint(_num(p[0], where), _num(p[1], where), yaw)


def _wp_list(w: Waypoint) -> list:
    return [round(w.x, 3), round(w.y, 3)] + ([] if w.yaw is None else [round(w.yaw, 4)])
