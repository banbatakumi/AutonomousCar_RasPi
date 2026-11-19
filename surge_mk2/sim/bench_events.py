"""`sim.bench` が人の代わりに planner へ出す要求（地図の読み込み・「走行」・経路/信号の切り替え）。

`sim/bench.py` から分けた。計画 1 回ごとに `BenchEvents.step` を呼ぶ（呼ぶ時機と順序は分ける前と同じ）。
"""

from __future__ import annotations

import json
from typing import Callable

__all__ = ["BenchEvents"]


class BenchEvents:
    """:param map_name: `--map` の地図名。最初の `plan()` の後に読み込ませる
        （`slam2d_route` は `plan()` で受けたパラメータで経路を作るため）
    :param routes_world: `--routes` の JSON（シムの世界座標）。`--map` なら地図と一緒に読ませる
    :param to_map_frame: 世界座標の経路の設定を map フレームへ直す関数（`Bench._to_map_frame`）
    :param auto_race: DONE（地図作成が終わって人の「走行」を待つ段）で、自動保存した地図を
        読み込ませて走り出させる（GUI の「レーシングライン走行」を押すのと同じ）
    :param events: `(時刻, 種類, 値)`。種類は "route"（`request_route`）/"signal"（`request_signal`）
    """

    def __init__(self, planner, *, map_name: str | None, routes_world: dict | None,
                 to_map_frame: Callable[[dict], dict], locate_hint: tuple[float, float] | None,
                 auto_race: bool, events: list[tuple[float, str, str]] | None,
                 notes: list[str], quiet: bool) -> None:
        self.planner = planner
        self.map_name = map_name
        self.routes_world = routes_world
        self.to_map_frame = to_map_frame
        self.locate_hint = locate_hint
        self.auto_race = auto_race
        self.events = sorted(events or [])
        self.notes = notes
        self.quiet = quiet
        self.load_pending = bool(map_name)

    def step(self, st, t: float) -> None:
        if self.load_pending:
            self.load_pending = False
            self._load_map(t)
            return
        if (self.auto_race and st.phase == "DONE"
                and getattr(self.planner, "_saved_map_name", "")):
            name = self.planner._saved_map_name
            if not self.quiet:
                print(f"  {t:5.1f}s 地図『{name}』で走行を開始する（bench が押す）", flush=True)
            self.planner.request_load(name)
        while self.events and self.events[0][0] <= t:
            _, kind, value = self.events.pop(0)
            fn = getattr(self.planner, "request_route" if kind == "route" else "request_signal",
                         None)
            if fn is not None:
                fn(value, "bench")
                self.notes.append(f"{t:5.1f}s {kind} → {value}")
                if not self.quiet:
                    print(f"  {t:5.1f}s 切替要求 {kind}={value}", flush=True)

    def _load_map(self, t: float) -> None:
        if self.routes_world is not None:
            # 経路の設定は地図と一緒に読ませる（読み込み後に渡すと、道路グラフが
            # できる前なので経由点のグループが作られない）
            from raspi.auto import mapstore
            cfg = self.to_map_frame(self.routes_world)
            # 地図作成の軌跡（自動経路のスタートと逆走禁止の元）は地図側のものを残す
            # （GUI の保存 `request_routes` と同じ。消すと自動経路が保存した線をなぞるだけになる）
            try:
                old = json.loads(mapstore.load_routes(self.map_name) or "{}")
            except ValueError:
                old = {}
            if "explore_traj" in old and "explore_traj" not in cfg:
                cfg["explore_traj"] = old["explore_traj"]
            mapstore.save_routes(self.map_name, json.dumps(cfg))
        self.planner.request_load(self.map_name)
        err = getattr(self.planner, "_load_error", "")
        if err:
            raise SystemExit(err)
        if self.locate_hint is not None:
            self.planner.request_locate_hint(*self.locate_hint)
        if not self.quiet:
            print(f"  {t:5.1f}s 保存済み地図『{self.map_name}』を読み込んだ", flush=True)
