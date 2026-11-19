"""コースエディタの中心線モード（`sim/editor.py` の docstring「中心線モード」）。

`Editor` に混ぜる部品。状態（`center_loop`・`width`）は `Editor` のインスタンスが持つ。
"""

from __future__ import annotations

import json
from pathlib import Path

from sim import track
from sim import ui as U
from sim.course import Course, DEFAULT_COURSE_DIR
from sim.editor_base import CENTER_RESOLUTION
from sim.sketch import loop_to_path


class CenterlineMode:
    """中心線モード（頂点のループが中心線、道幅から壁を作る）。"""

    def set_width(self, d: float) -> None:
        self.width = max(0.3, min(3.0, round(self.width + d, 3)))
        self._mark_dirty()

    def _preview_centerline(self) -> Course | None:
        if len(self.center_loop.vertices) < 2:
            return None
        if self._dirty or self._cached_course is None:
            origin, path = loop_to_path(self.center_loop)
            meta = {"path": path, "origin": list(origin), "width": self.width,
                    "resolution": CENTER_RESOLUTION, "loop": self.center_loop.closed,
                    "obstacles": self.obstacles}
            t = track.build(meta)
            self._cached_course = Course(
                name=self.name, path=Path(f"<editor>/{self.name}"),
                resolution=t["resolution"], origin=t["origin"], start=t["start"],
                grid=t["grid"], centerline=t["centerline"], width=t["width"],
                obstacles=t.get("obstacles"))
            self._dirty = False
        return self._cached_course

    def _save_centerline(self) -> None:
        if len(self.center_loop.vertices) < 2:
            self.say("頂点がありません")
            return
        origin, path = loop_to_path(self.center_loop)
        p = DEFAULT_COURSE_DIR / f"{self.name}.json"
        m = {"name": self.name, "width": self.width, "resolution": CENTER_RESOLUTION,
            "origin": list(origin), "loop": self.center_loop.closed, "path": path,
            "obstacles": [list(o) for o in self.obstacles],
            "note": "sim.editor（頂点スケッチ、中心線モード）で作成"}
        p.write_text(json.dumps(m, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        self.say(f"保存: {p.name}"
                 + ("（閉ループ）" if self.center_loop.closed else "（開いたまま）"))

    def _draw_centerline_panel(self, x: int, y: int, inner: int) -> int:
        y = self.ui.section(self.screen, "コース", x, y, inner)
        y = self.ui.kv(self.screen, x, y, inner, "道幅 [m]  (W / Shift+W)", f"{self.width:4.2f}")
        y = self.ui.kv(self.screen, x, y, inner, "頂点 [個]", f"{len(self.center_loop)}")
        course = self._cached_course if not self._dirty else None
        if course is not None:
            y = self.ui.kv(self.screen, x, y, inner, "寸法 [m]",
                           f"{course.size_m[0]:4.1f} x {course.size_m[1]:4.1f}", U.DIM)
        y = self.ui.kv(self.screen, x, y, inner, "閉ループ",
                       "閉じた" if self.center_loop.closed else "未閉鎖",
                       U.OK if self.center_loop.closed else U.WARN, mono=False)
        return y + 8
