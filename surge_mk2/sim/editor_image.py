"""コースエディタの画像モード（`sim/editor.py` の docstring「画像モード」）。

`Editor` に混ぜる部品。状態（`img_*`）は `Editor` のインスタンスが持つ。
"""

from __future__ import annotations

import json
import re
from pathlib import Path

import numpy as np
import pygame

from sim import image_course as IC
from sim import track
from sim import ui as U
from sim.course import Course, DEFAULT_COURSE_DIR
from sim.editor_base import IMG_DEFAULT_WIDTH_M, IMG_TOOLS, Btn


class ImageMode:
    """画像モード（写真・図・PNG から占有格子コースを作る）。"""

    def _min_area_px(self) -> float:
        return self.img_min_area * self.img_scale ** 2

    def _wall_mask(self) -> np.ndarray | None:
        """現在のパラメータでの壁マスク。パラメータが変わったときだけ作り直す。"""
        if self.img_rgb is None:
            return None
        key = (self.img_ref, self.img_tol, self.img_close, round(self._min_area_px(), 1),
               tuple(self.img_ops))
        if key != self._img_mask_key or self._img_mask is None:
            self._img_mask = IC.build_mask(self.img_rgb, self.img_ref, self.img_tol,
                                           self.img_close, self._min_area_px(), self.img_ops)
            self._img_mask_key = key
        return self._img_mask

    def _preview_image(self) -> Course | None:
        mask = self._wall_mask()
        if mask is None:
            return None
        if self._dirty or self._cached_course is None:
            samp = IC.sample_grid(mask, self.img_scale, self.img_origin)
            grid = samp.grid
            obstacles = None
            if self.obstacles:
                track.stamp_discs(grid, samp.origin, IC.RES, self.obstacles)
                obstacles = np.asarray(self.obstacles, dtype=np.float64)
            self._cached_course = Course(
                name=self.name, path=Path(f"<editor>/{self.name}"),
                resolution=IC.RES, origin=samp.origin,
                start=self.wall_start or (0.0, 0.0, 0.0), grid=grid, obstacles=obstacles)
            self._dirty = False
        return self._cached_course

    def _img_bbox_px(self) -> tuple[int, int, int, int] | None:
        mask = self._wall_mask()
        if mask is None:
            return None
        ys, xs = np.nonzero(mask)
        if ys.size == 0:
            return None
        return int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())

    def _install_image(self, rgb: np.ndarray, path: Path) -> None:
        self.img_rgb = rgb
        self.img_path = path
        self.img_ref = IC.guess_wall_color(rgb)
        self.img_tol = 100 if self.img_ref == (0, 0, 0) else 30
        self.img_close = 0 if self.img_ref == (0, 0, 0) else 2
        self.img_min_area = 0.0 if self.img_ref == (0, 0, 0) else 0.03
        self.img_ops = []
        self.img_origin = (0.0, 0.0)
        self.img_scale = max(1.0, rgb.shape[1] / IMG_DEFAULT_WIDTH_M)
        self.img_tool = "blob"
        self._src_surf = None
        self._src_cache = {}
        self._img_mask_key = None
        self.wall_start = None
        self.obstacles = []
        self.mode = "image"

    def load_image(self, path: str | Path) -> None:
        """写真・図を読み込んで画像モードを始める。縮尺は仮（壁の幅が
        `IMG_DEFAULT_WIDTH_M` になる値）なので、実寸は「外形の幅」で合わせる。"""
        p = Path(path)
        try:
            rgb = IC.load_rgb(p)
        except Exception as e:
            self.say(f"画像を開けません: {e}")
            return
        self._install_image(rgb, p)
        bb = self._img_bbox_px()
        if bb is not None:
            self.img_scale = max(1.0, (bb[2] - bb[0] + 1) / IMG_DEFAULT_WIDTH_M)
        self.name = re.sub(r"[^A-Za-z0-9_-]", "", p.stem)[:32] or "new_course"
        self._fit_view()
        self._mark_dirty()
        self.say(f"{p.name} を読み込みました。壁の色を確認し、外形の幅を実寸に合わせてください")

    def _open_png_course(self, png: Path, meta: dict) -> None:
        """既存の PNG コースを画像モードで開く（ノイズ・障害物の整理用）。"""
        rgb = IC.load_rgb(png)
        self._install_image(rgb, png)
        res = float(meta.get("resolution", 0.02))
        self.img_scale = 1.0 / res
        self.img_origin = tuple(float(v) for v in meta.get("origin", (0.0, 0.0)))
        self.img_ref, self.img_tol, self.img_close, self.img_min_area = (0, 0, 0), 100, 0, 0.0
        if meta.get("start"):
            self.wall_start = tuple(float(v) for v in meta["start"])
        self.obstacles = [(float(x), float(y), float(r)) for x, y, r in meta.get("obstacles", [])]
        self.name = png.stem
        self._fit_view()
        self._mark_dirty()
        self.say(f"{png.name} を開きました（PNG コース）")

    def pick_image_file(self) -> None:
        """macOS のファイル選択ダイアログ（osascript）。Tk は pygame と同居できない。"""
        import subprocess
        try:
            out = subprocess.run(
                ["osascript", "-e", 'POSIX path of (choose file with prompt "コース画像を選択" '
                 'of type {"public.image"})'], capture_output=True, text=True, timeout=300)
        except Exception as e:
            self.say(f"ダイアログを開けません: {e}（画像をウィンドウにドロップしてください）")
            return
        path = out.stdout.strip()
        if out.returncode == 0 and path:
            self.load_image(path)

    def set_img_wall_color(self, rgb: tuple[int, int, int]) -> None:
        self.img_ref = rgb
        self._mark_dirty()
        self.say(f"壁の色を {rgb} にしました")

    def adj_img(self, what: str, d: float) -> None:
        if self.img_rgb is None:
            return
        if what == "tol":
            self.img_tol = int(max(0, min(100, self.img_tol + d)))
        elif what == "close":
            self.img_close = int(max(0, min(12, self.img_close + d)))
        elif what == "area":
            self.img_min_area = max(0.0, round(self.img_min_area + d, 4))
        elif what == "width":
            self._set_img_width(d)
        self._mark_dirty()

    def _set_img_width(self, d: float) -> None:
        """壁の外接矩形の幅を `d` [m] 増減する＝縮尺を変える。スタートと障害物の位置は
        画像に対して動かさない（世界座標を縮尺ぶん引き伸ばす。半径は実寸なのでそのまま）。"""
        bb = self._img_bbox_px()
        if bb is None:
            self.say("壁が見つかりません（色の許容を上げるかスポイトで指定してください）")
            return
        wpx = bb[2] - bb[0] + 1
        new_w = max(0.5, wpx / self.img_scale + d)
        new_scale = wpx / new_w
        f = self.img_scale / new_scale
        ox, oy = self.img_origin
        if self.wall_start is not None:
            x, y, yaw = self.wall_start
            self.wall_start = (ox + (x - ox) * f, oy + (y - oy) * f, yaw)
        self.obstacles = [(ox + (x - ox) * f, oy + (y - oy) * f, r) for x, y, r in self.obstacles]
        self.img_scale = new_scale

    def _img_width_m(self) -> float | None:
        bb = self._img_bbox_px()
        return None if bb is None else (bb[2] - bb[0] + 1) / self.img_scale

    def set_img_tool(self, tool: str) -> None:
        self.img_tool = tool
        self.placing_start = False
        self.placing_obstacle = False
        self.say({"eyedrop": "壁の上をクリックして壁の色を指定",
                  "blob": "消したい壁の塊をクリック",
                  "erase": "消したい範囲をドラッグ",
                  "fill": "壁にしたい範囲をドラッグ"}[tool])

    def toggle_img_src(self) -> None:
        self.img_show_src = not self.img_show_src

    def _world_to_img(self, x: float, y: float) -> tuple[float, float]:
        h = self.img_rgb.shape[0]
        ox, oy = self.img_origin
        return (x - ox) * self.img_scale, h - 1 - (y - oy) * self.img_scale

    def _img_canvas_up(self) -> None:
        """画像モードのツール操作（スタート・障害物の配置は共通処理が先に取る）。"""
        if self.img_rgb is None:
            return
        h, w = self.img_rgb.shape[:2]
        ax, ay = self._world_to_img(*self.press_world)
        if self.img_tool in ("eyedrop", "blob"):
            if not (0 <= ax < w and 0 <= ay < h):
                return
            if self.img_tool == "eyedrop":
                x, y = int(ax), int(ay)
                patch = self.img_rgb[max(0, y - 1):y + 2, max(0, x - 1):x + 2].reshape(-1, 3)
                self.set_img_wall_color(tuple(int(v) for v in patch.mean(axis=0)))
            else:
                self.img_ops.append(("blob", ax, ay))
                self._mark_dirty()
        else:
            bx, by = self._world_to_img(*self.to_world(*self.mouse_screen))
            if abs(bx - ax) < 1 and abs(by - ay) < 1:
                return
            self.img_ops.append((self.img_tool, min(ax, bx), min(ay, by),
                                 max(ax, bx), max(ay, by)))
            self._mark_dirty()

    def _save_image(self) -> None:
        mask = self._wall_mask()
        if mask is None:
            self.say("画像がありません。画像ファイルをドロップしてください")
            return
        if self.wall_start is None:
            self.say("スタート地点を配置してください")
            return
        samp = IC.sample_grid(mask, self.img_scale, self.img_origin)
        try:
            grid, shift = IC.crop_for_save(samp.grid, samp.origin)
        except ValueError as e:
            self.say(str(e))
            return
        sx, sy, syaw = self.wall_start
        start = [round(sx - shift[0], 3), round(sy - shift[1], 3), round(syaw, 4)]
        obstacles = [[round(x - shift[0], 3), round(y - shift[1], 3), round(r, 3)]
                     for x, y, r in self.obstacles]
        col, row = int(start[0] / IC.RES), int(start[1] / IC.RES)
        gh, gw = grid.shape
        if not (0 <= row < gh and 0 <= col < gw) or grid[row, col]:
            self.say("スタート地点が壁の中（または範囲外）です。置き直してください")
            return

        from PIL import Image
        png = DEFAULT_COURSE_DIR / f"{self.name}.png"
        Image.fromarray((np.flipud(~grid).astype(np.uint8)) * 255, "L").save(png)
        m = {"name": self.name, "resolution": IC.RES, "origin": [0.0, 0.0], "start": start,
             "wall_threshold": 128,
             "note": f"sim.editor（画像モード）で {self.img_path.name if self.img_path else '画像'} "
                     f"から作成。縮尺 {self.img_scale:.1f}px/m"}
        if obstacles:
            m["obstacles"] = obstacles
        (DEFAULT_COURSE_DIR / f"{self.name}.json").write_text(
            json.dumps(m, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        self.say(f"保存: {png.name} + .json（{gw * IC.RES:.1f} x {gh * IC.RES:.1f} m）")

    def _draw_image_source(self) -> None:
        """元画像に、壁として拾った所を赤く重ねて見せる（`V`）。"""
        h, w = self.img_rgb.shape[:2]
        dw = max(1, int(w / self.img_scale * self.view_scale))
        dh = max(1, int(h / self.img_scale * self.view_scale))
        ox, oy = self.to_px(self.img_origin[0], self.img_origin[1] + h / self.img_scale)
        if self._src_surf is None:
            self._src_surf = pygame.surfarray.make_surface(np.transpose(self.img_rgb, (1, 0, 2)))
        ck = (dw, dh)
        if ck not in self._src_cache:
            if len(self._src_cache) > 6:
                self._src_cache.clear()
            self._src_cache[ck] = pygame.transform.smoothscale(self._src_surf, ck)
        self.screen.blit(self._src_cache[ck], (int(ox), int(oy)))
        mk = ("mask", ck, hash(self._img_mask_key))
        if mk not in self._src_cache:
            m = self._wall_mask()
            rgba = np.zeros((h, w, 4), dtype=np.uint8)
            rgba[m] = (255, 40, 40, 140)
            surf = pygame.image.frombuffer(rgba.tobytes(), (w, h), "RGBA")
            self._src_cache[mk] = pygame.transform.smoothscale(surf, ck)
        self.screen.blit(self._src_cache[mk], (int(ox), int(oy)))

    def _draw_image_panel(self, x: int, y: int, inner: int) -> int:
        y = self.ui.section(self.screen, "画像から作成", x, y, inner)
        if self.img_rgb is None:
            self.screen.blit(self.ui.lbl.render("画像をウィンドウにドロップ", True, U.WARN), (x, y))
            y += 24
            r = pygame.Rect(x, y, inner, 28)
            b = Btn(r, "act", self.pick_image_file, "画像を開く  I")
            self.buttons.append(b)
            self.ui.button(self.screen, r, b.label, hover=(self.hover is b), color=U.ACCENT)
            return y + 36
        y = self.ui.kv(self.screen, x, y, inner, "画像",
                       (self.img_path.name if self.img_path else "")[-22:], U.DIM, mono=False)
        # 壁の色（見本つき）
        pygame.draw.rect(self.screen, self.img_ref, pygame.Rect(x, y + 2, 18, 18), border_radius=3)
        pygame.draw.rect(self.screen, U.LINE, pygame.Rect(x, y + 2, 18, 18), 1, border_radius=3)
        self.screen.blit(self.ui.lbl.render(f"壁の色 {self.img_ref}", True, U.DIM), (x + 26, y + 3))
        y += 26

        a = lambda what, d: (lambda: self.adj_img(what, d))          # noqa: E731
        y = self._adj_row(x, y, inner, "色の許容", f"{self.img_tol}",
                          [("−5", a("tol", -5)), ("+5", a("tol", 5))])
        y = self._adj_row(x, y, inner, "穴埋め [px]", f"{self.img_close}",
                          [("−", a("close", -1)), ("+", a("close", 1))])
        y = self._adj_row(x, y, inner, "最小サイズ [cm²]", f"{self.img_min_area * 1e4:.0f}",
                          [("−50", a("area", -0.005)), ("+50", a("area", 0.005))])
        wm = self._img_width_m()
        y = self._adj_row(x, y, inner, "外形の幅 [m]", "—" if wm is None else f"{wm:.2f}",
                          [("−1", a("width", -1.0)), ("−.1", a("width", -0.1)),
                           ("+.1", a("width", 0.1)), ("+1", a("width", 1.0))])

        n = len(IMG_TOOLS)
        bw = (inner - 6 * (n - 1)) // n
        for i, (tool, label) in enumerate(IMG_TOOLS):
            r = pygame.Rect(x + i * (bw + 6), y, bw, 26)
            b = Btn(r, "img_tool", tool, label)
            self.buttons.append(b)
            self.ui.button(self.screen, r, label,
                           on=(self.img_tool == tool and not (self.placing_start or self.placing_obstacle)),
                           hover=(self.hover is b))
        y += 32
        r = pygame.Rect(x, y, inner, 26)
        label = f"表示: {'元画像（拾った壁=赤）' if self.img_show_src else '結果'}  V"
        b = Btn(r, "act", self.toggle_img_src, label)
        self.buttons.append(b)
        self.ui.button(self.screen, r, label, on=self.img_show_src, hover=(self.hover is b))
        y += 32
        r = pygame.Rect(x, y, inner, 26)
        b = Btn(r, "act", self.begin_place_start, "スタート地点を配置")
        self.buttons.append(b)
        self.ui.button(self.screen, r, b.label, on=self.placing_start, hover=(self.hover is b))
        y += 30
        ok = self.wall_start is not None
        y = self.ui.kv(self.screen, x, y, inner, "スタート地点", "配置済み" if ok else "未配置",
                       U.OK if ok else U.WARN, mono=False)
        r = pygame.Rect(x, y, inner, 26)
        b = Btn(r, "act", self.pick_image_file, "別の画像を開く  I")
        self.buttons.append(b)
        self.ui.button(self.screen, r, b.label, hover=(self.hover is b))
        return y + 32
