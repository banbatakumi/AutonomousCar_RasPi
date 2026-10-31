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
  "avoid": [[x, y], ...],
  "avoid_rects": [[x0, y0, x1, y1], ...],
  "explore_traj": [[x, y], ...]
}
```

- `groups` のキーは A〜D。経由点を1つも置いていないグループは存在しないのと同じ。**planner は経由点を自動では書き込まない**
  （自動経路 `auto` は planner が内部だけで作り、グループの有無にかかわらず選べる、`slam2d_route.py`）
- 経由点の3要素目 `yaw` は「その向きで通る」指定（省略可）
- `stops[*].mode`: `"stop"` はその場所で止まるだけ、`"park"` は手前で止まってから
  `park_to_point` へ引き継いでその姿勢（`yaw` 必須）へ入れる
- `avoid`: 避ける点。いちばん近い道（道路グラフのエッジ）を**自動経路・経由点の経路・停止経路の
  どれにも使わない**（ルールで通れない近道など。地図だけからは分からない）。★点の吸着は
  「いちばん近い道」なので、壁の切れ目（隙間）の上を指すと隣の本線に吸着し、そこが唯一の道なら
  周回が1つも無くなる。範囲で指すなら `avoid_rects` を使う
- `avoid_rects`: 通ってはいけない範囲（軸に平行な矩形、map フレーム）。**道路グラフを作る前に
  地図の上で壁として塗る**（`paint_avoid`）ので、範囲の外の道はそのまま使え、範囲に掛かる壁の
  隙間は閉じる。レーシングラインもこの壁から車体半幅＋余裕だけ離れる
- `signal_map`: カメラの信号認識（`route/select` トピック）が送ってくる値 → グループ（`auto` も可）
- `active`: 走り出す経路。A〜D か `"auto"`（自動経路、既定）
- `explore_traj`: 地図作成の軌跡（間引き済み）。**道路グラフの「通ってよい向き」
  （逆走禁止）を保存地図から復元するのに要る**（`nav/roadgraph._directions`）
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field

import numpy as np

from ..nav.route import Waypoint

__all__ = ["GROUPS", "ROUTE_KEYS", "AUTO_KEY", "RouteConfig", "Stop", "paint_avoid"]

GROUPS = ("A", "B", "C", "D")
#: 自動経路のキー（経由点は持たない。planner が地図から作る）
AUTO_KEY = "auto"
#: 走行中に選べる経路（`active`・`signal_map` の行き先・GUI の切替ボタン）
ROUTE_KEYS = (AUTO_KEY,) + GROUPS
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
    active: str = AUTO_KEY
    stops: dict[str, Stop] = field(default_factory=dict)
    mission: dict = field(default_factory=dict)
    signal_map: dict[str, str] = field(default_factory=dict)
    avoid: list[tuple[float, float]] = field(default_factory=list)
    #: 通ってはいけない範囲 `(x0, y0, x1, y1)`（x0≤x1・y0≤y1 に揃えて持つ）
    avoid_rects: list[tuple[float, float, float, float]] = field(default_factory=list)
    explore_traj: np.ndarray | None = None

    # ── 変換 ──

    @classmethod
    def from_dict(cls, d: dict | None) -> "RouteConfig":
        """検証しながら読む。**不正な値は ValueError（理由は日本語）。**"""
        d = d or {}
        if not isinstance(d, dict):
            raise ValueError("経路の設定がオブジェクトではない")
        groups: dict[str, list[Waypoint]] = {}
        for k, pts in _obj(d.get("groups"), "groups").items():
            if k not in GROUPS:
                raise ValueError(f"グループ名『{k}』は使えない（A〜D）")
            if not isinstance(pts, list) or len(pts) > _MAX_WAYPOINTS:
                raise ValueError(f"グループ{k}の経由点が不正（最大{_MAX_WAYPOINTS}個）")
            wps = [_waypoint(p, f"グループ{k}の経由点{i + 1}") for i, p in enumerate(pts)]
            if wps:
                groups[k] = wps
        stops = {}
        for name, s in _obj(d.get("stops"), "stops").items():
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
        mission = {"laps": max(0, int(_num(mission.get("laps") or 0, "ミッションの周回数"))),
                   "time_s": max(0.0, _num(mission.get("time_s") or 0.0, "ミッションの時間")),
                   "then": then}
        sig = {}
        for k, g in _obj(d.get("signal_map"), "signal_map").items():
            if not isinstance(g, str) or g not in ROUTE_KEYS:
                raise ValueError(f"信号『{k}』の行き先『{g}』はグループ名ではない")
            sig[str(k)] = str(g)
        active = str(d.get("active", AUTO_KEY) or AUTO_KEY)
        if active not in ROUTE_KEYS:
            raise ValueError(f"グループ名『{active}』は使えない（auto・A〜D）")
        avoid_raw = d.get("avoid") or []
        if not isinstance(avoid_raw, list) or len(avoid_raw) > _MAX_WAYPOINTS:
            raise ValueError(f"避ける点が不正（最大{_MAX_WAYPOINTS}個）")
        avoid = []
        for i, q in enumerate(avoid_raw):
            w = _waypoint(q, f"避ける点{i + 1}")
            avoid.append((w.x, w.y))
        rects_raw = d.get("avoid_rects") or []
        if not isinstance(rects_raw, list) or len(rects_raw) > _MAX_WAYPOINTS:
            raise ValueError(f"避ける範囲が不正（最大{_MAX_WAYPOINTS}個）")
        rects = []
        for i, q in enumerate(rects_raw):
            where = f"避ける範囲{i + 1}"
            if not isinstance(q, (list, tuple)) or len(q) != 4:
                raise ValueError(f"{where}は [x0, y0, x1, y1]")
            x0, y0, x1, y1 = (_num(v, where) for v in q)
            rects.append((min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)))
        traj = d.get("explore_traj")
        tr = None
        if traj is not None:
            try:
                tr = np.asarray(traj, dtype=np.float64)
            except (TypeError, ValueError) as e:
                raise ValueError("explore_traj が不正") from e
            if tr.ndim != 2 or tr.shape[1] != 2 or len(tr) > _MAX_TRAJ \
                    or not np.isfinite(tr).all():
                raise ValueError("explore_traj が不正")
        return cls(groups=groups, active=active, stops=stops, mission=mission,
                   signal_map=sig, avoid=avoid, avoid_rects=rects, explore_traj=tr)

    @classmethod
    def from_json(cls, s: str) -> "RouteConfig":
        try:
            d = json.loads(s) if s else {}
        except ValueError as e:
            raise ValueError(f"経路の設定が JSON として読めない: {e}") from e
        try:
            return cls.from_dict(d)
        except (TypeError, AttributeError, OverflowError) as e:
            # 型の食い違いは `from_dict` が個別に `ValueError` にしているが、取りこぼしが
            # あっても呼び出し側（`except ValueError`）を抜けて planning_node まで届かせない
            raise ValueError(f"経路の設定の形式が不正: {e!r}") from e

    def to_dict(self, *, with_traj: bool = True) -> dict:
        d: dict = {
            "groups": {k: [_wp_list(w) for w in v] for k, v in self.groups.items()},
            "active": self.active,
            "stops": {k: {"x": s.x, "y": s.y, "yaw": s.yaw, "mode": s.mode}
                      for k, s in self.stops.items()},
            "mission": dict(self.mission),
            "signal_map": dict(self.signal_map),
            "avoid": [[round(x, 3), round(y, 3)] for x, y in self.avoid],
            "avoid_rects": [[round(v, 3) for v in r] for r in self.avoid_rects],
        }
        if with_traj and self.explore_traj is not None:
            d["explore_traj"] = np.round(self.explore_traj, 3).tolist()
        return d

    def to_json(self, *, with_traj: bool = True) -> str:
        return json.dumps(self.to_dict(with_traj=with_traj), ensure_ascii=False)

    def same_groups(self, other: "RouteConfig") -> bool:
        """経路を作り直す必要が無いか（経由点・避ける点・避ける範囲が同じか）。"""
        a, b = self.to_dict(with_traj=False), other.to_dict(with_traj=False)
        return (a["groups"] == b["groups"] and a["avoid"] == b["avoid"]
                and a["avoid_rects"] == b["avoid_rects"])

    def same_rects(self, other: "RouteConfig") -> bool:
        """避ける範囲が同じか（違えば道路グラフから作り直す）。"""
        return (self.to_dict(with_traj=False)["avoid_rects"]
                == other.to_dict(with_traj=False)["avoid_rects"])

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


def paint_avoid(trinary: np.ndarray, resolution: float, origin: tuple[float, float],
                rects) -> np.ndarray:
    """避ける範囲を占有（2）で塗った3値地図の**写し**（範囲が無ければ元をそのまま返す）。

    範囲に少しでも掛かるセルを塗る（細い範囲でも隙間を確実に閉じる）。**経路を作るときだけに
    使い、自己位置推定・障害物検出の地図には塗らない**（そこに実際の壁は無い）。
    """
    if not rects:
        return trinary
    out = trinary.copy()
    h, w = out.shape
    res = float(resolution)
    for x0, y0, x1, y1 in rects:
        c0 = max(0, int(math.floor((x0 - origin[0]) / res)))
        c1 = min(w, int(math.ceil((x1 - origin[0]) / res)))
        r0 = max(0, int(math.floor((y0 - origin[1]) / res)))
        r1 = min(h, int(math.ceil((y1 - origin[1]) / res)))
        if c0 < c1 and r0 < r1:
            out[r0:r1, c0:c1] = 2
    return out


def _obj(v, where: str) -> dict:
    """無指定（`None`・空）は空として読む。それ以外でオブジェクトでなければ不正。"""
    if not v:
        return {}
    if not isinstance(v, dict):
        raise ValueError(f"{where} がオブジェクトではない")
    return v


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
