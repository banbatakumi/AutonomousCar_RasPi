"""コースエディタの壁モード（`sim/editor.py` の docstring「壁モード」）。

`Editor` に混ぜる部品。状態（`wall_loops`・`wall_current`・`wall_thickness`・`wall_start`）は
`Editor` のインスタンスが持つ。
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pygame

from sim import track
from sim import ui as U
from sim import wall_track
from sim.course import Course, DEFAULT_COURSE_DIR
from sim.editor_base import WALL_MARGIN, WALL_RESOLUTION, Btn
from sim.sketch import Loop, loop_to_path


class WallMode:
    """壁モード（頂点のループが壁そのもの、中心線は自動導出）。"""

    def _all_wall_loops(self) -> list[Loop]:
        return self.wall_loops + ([self.wall_current] if self.wall_current.vertices else [])

    def finish_current_line(self) -> None:
        """Enter: 編集中の折れ線を**閉じずにそのまま**確定し、次の壁を描き始め
        られるようにする。壁モード専用——中心線モードはコースにつき中心線が
        1本しか無いので「次の線」という概念が無い。

        閉じたループ（`_on_loop_closed`）と違い、ここで確定した壁は開いたまま
        `wall_loops` に入る。閉じていない壁が骨格化でどう扱われるかは
        `sim/wall_track.py` 参照——コース内の空間を分断しない短い壁（障害物や
        シケインの板）なら、骨格の袋小路の枝として刈り取られるだけで問題なく
        通る。両側の壁をまたいで完全に分断する壁（分岐・複数ループ）を置いた
        場合は、保存自体は止まらないが中心線が `null` になる（`_save_wall()`）。
        """
        if self.mode != "wall":
            self.say("中心線モードでは使えません（コースにつき中心線は1本です）")
            return
        if len(self.wall_current.vertices) < 2 or not self.wall_current.edges:
            self.say("頂点が足りません（2点以上、1辺以上必要です）")
            return
        self.wall_loops.append(self.wall_current)
        self.wall_current = Loop()
        self._mark_dirty()
        self.say(f"壁を確定しました（開いたまま、計 {len(self.wall_loops)} 本）。次の壁を描けます")

    def delete_wall_loop(self, i: int) -> None:
        if 0 <= i < len(self.wall_loops):
            del self.wall_loops[i]
            self._mark_dirty()
            self.say(f"壁ループ {i + 1} を削除しました")

    def set_wall_thickness(self, d: float) -> None:
        self.wall_thickness = max(0.01, min(0.3, round(self.wall_thickness + d, 3)))
        self._mark_dirty()

    def _preview_wall(self) -> Course | None:
        # 編集中のループ(`wall_current`)は、頂点を1つ置いただけ（辺がまだ無い）の
        # 状態でも `_all_wall_loops()` に含まれる。`sample_loop`/`loop_to_path` は
        # 2頂点・1辺以上を要求するので、まだ1点しか無いループはラスタライズ対象
        # から外す（外さないと「壁モードでキャンバスを1回クリックしただけで
        # ValueError で落ちる」——実際に踏んだ罠）
        loops = [l for l in self._all_wall_loops() if len(l.vertices) >= 2 and l.edges]
        if not loops:
            return None
        if self._dirty or self._cached_course is None:
            try:
                grid, origin = wall_track.rasterize_walls(
                    loops, self.wall_thickness, WALL_RESOLUTION, WALL_MARGIN)
            except wall_track.WallExtractionError:
                self._cached_course = None
                self._dirty = False
                return None
            obstacles = None
            if self.obstacles:
                track.stamp_discs(grid, origin, WALL_RESOLUTION, self.obstacles)
                obstacles = np.asarray(self.obstacles, dtype=np.float64)
            self._cached_course = Course(
                name=self.name, path=Path(f"<editor>/{self.name}"),
                resolution=WALL_RESOLUTION, origin=origin,
                start=self.wall_start or (0.0, 0.0, 0.0), grid=grid,
                centerline=self._wall_centerline_preview, width=None,
                obstacles=obstacles)
            self._dirty = False
        return self._cached_course

    def generate_wall_preview(self) -> None:
        """壁モードの骨格化＋中心線導出を明示的に1回実行する（重いので自動では
        呼ばない）。保存の直前にも同じ処理を行うので、これは事前確認用。

        **これが失敗しても保存は妨げない。** 分岐・複数ループの壁は中心線に
        単純化できないので、ここで出るエラーは「中心線プレビューだけ出せない」
        という意味で、コース自体は保存できる（`_save_wall()`は`centerline: null`
        で保存を続ける）。
        """
        if not self.wall_loops:
            self.say("閉じた壁ループがありません")
            return
        if self.wall_start is None:
            self.say("スタート地点を配置してください")
            return
        try:
            grid, origin = wall_track.rasterize_walls(
                self.wall_loops, self.wall_thickness, WALL_RESOLUTION, WALL_MARGIN)
            cl = wall_track.derive_centerline(grid, origin, WALL_RESOLUTION,
                                              self.wall_start[:2])
        except wall_track.WallExtractionError as e:
            self.say(f"中心線プレビューのみ不可（分岐/複数ループ）: {e} "
                     "— 保存(S)は問題なくできます")
            return
        self._wall_centerline_preview = cl
        self._dirty = True
        self.say(f"中心線を生成しました（{len(cl)} 点）")

    def _save_wall(self) -> None:
        """壁を保存する。中心線の導出（骨格化→単一ループへ刈り込み）に失敗しても
        **保存は止めない**——分岐・複数ループのあるコースを描く自由を優先する
        （`sim/wall_track.py` 参照）。その場合 `centerline` は `null` で保存され、
        `sim/slam_bench.py` の `TruthDriver` のように中心線を前提とする機能だけが
        使えなくなる（占有格子ベースの経路計画や手動運転は影響を受けない）。
        """
        if not self.wall_loops:
            self.say("閉じた壁ループがありません")
            return
        if self.wall_start is None:
            self.say("スタート地点を配置してください")
            return
        try:
            grid, origin = wall_track.rasterize_walls(
                self.wall_loops, self.wall_thickness, WALL_RESOLUTION, WALL_MARGIN)
            centerline = wall_track.derive_centerline(grid, origin, WALL_RESOLUTION,
                                                      self.wall_start[:2])
        except wall_track.WallExtractionError as e:
            centerline = None
            self._centerline_warning = str(e)

        walls_json = []
        for loop in self.wall_loops:
            o, path = loop_to_path(loop)
            walls_json.append({"origin": list(o), "path": path})

        p = DEFAULT_COURSE_DIR / f"{self.name}.json"
        note = "sim.editor（頂点スケッチ、壁モード）で作成。centerlineは自動導出"
        if centerline is None:
            note = ("sim.editor（頂点スケッチ、壁モード）で作成。分岐/複数ループのため"
                    "centerlineは導出できず null（pure pursuit等の追従には使えない）")
        m = {"name": self.name, "mode": "wall", "resolution": WALL_RESOLUTION,
            "wall_thickness": self.wall_thickness, "margin": WALL_MARGIN,
            "walls": walls_json,
            "centerline": centerline.tolist() if centerline is not None else None,
            "start": list(self.wall_start),
            "obstacles": [list(o) for o in self.obstacles],
            "note": note}
        p.write_text(json.dumps(m, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        if centerline is None:
            self.say(f"保存できました: {p.name}（中心線なし: {self._centerline_warning}。"
                     "pure pursuit等の追従以外は問題なく使えます）")
        else:
            self.say(f"保存: {p.name}（中心線を自動生成、{len(centerline)} 点）")

    def _draw_wall_panel(self, x: int, y: int, inner: int) -> int:
        y = self.ui.section(self.screen, "壁", x, y, inner)
        y = self.ui.kv(self.screen, x, y, inner, "壁の厚み [m]  (W / Shift+W)",
                       f"{self.wall_thickness:4.2f}")
        y = self.ui.kv(self.screen, x, y, inner, "壁ループ [本]", f"{len(self.wall_loops)}")
        y = self.ui.kv(self.screen, x, y, inner, "編集中の頂点 [個]", f"{len(self.wall_current)}")

        r = pygame.Rect(x, y, inner, 26)
        b = Btn(r, "act", None, "この壁を確定（開いたまま）  Enter")
        b.value = self.finish_current_line
        self.buttons.append(b)
        self.ui.button(self.screen, r, b.label, hover=(self.hover is b))
        y += 32

        for i, loop in enumerate(self.wall_loops[-6:]):
            real_i = len(self.wall_loops) - min(6, len(self.wall_loops)) + i
            r = pygame.Rect(x, y, inner, 24)
            state = "閉" if loop.closed else "開"
            txt = f"壁 {real_i + 1}［{state}］: 頂点 {len(loop)}"
            self.screen.blit(self.ui.lbl.render(txt, True, U.DIM), (x, y + 4))
            dr = pygame.Rect(x + inner - 24, y, 24, 22)
            b = Btn(dr, "wall_del", real_i, "×")
            self.buttons.append(b)
            self.ui.button(self.screen, dr, "×", hover=(self.hover is b), color=U.BAD)
            y += 26

        y += 6
        r = pygame.Rect(x, y, inner, 28)
        b = Btn(r, "act", None, "スタート地点を配置")
        b.value = self.begin_place_start
        self.buttons.append(b)
        self.ui.button(self.screen, r, b.label, on=self.placing_start, hover=(self.hover is b))
        y += 36
        started = "配置済み" if self.wall_start is not None else "未配置"
        y = self.ui.kv(self.screen, x, y, inner, "スタート地点", started,
                       U.OK if self.wall_start is not None else U.WARN, mono=False)

        y += 4
        r = pygame.Rect(x, y, inner, 28)
        b = Btn(r, "act", None, "中心線プレビューを生成")
        b.value = self.generate_wall_preview
        self.buttons.append(b)
        self.ui.button(self.screen, r, b.label, hover=(self.hover is b), color=U.ACCENT)
        y += 34
        self.screen.blit(self.ui.sm.render(
            "分岐・複数ループの形状でも保存はできる（中心線がnullになるだけ）", True, U.FAINT), (x, y))
        y += 16
        self.screen.blit(self.ui.sm.render(
            "中心線が無いと使えないのは pure pursuit 追従など一部の機能のみ", True, U.FAINT), (x, y))
        y += 20
        return y
