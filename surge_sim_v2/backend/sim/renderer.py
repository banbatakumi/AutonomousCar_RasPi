"""pygame 描画エンジン（シミュ専用）。

役割：シミュレーションの真実を表示する確認ウィンドウ。
キーボードの速度・ステア操作は受け付けない（操作は WebUI のブラウザで行う）。

画面：1200x800。左=コース俯瞰図（真実の世界）、右=COURSE SELECT / SIM CTRL。
SharedState から読み込んだデータのみを描画する（直接バックエンドにアクセスしない）。

操作：SPACE=Pause、ESC=Quit、右パネルのコース／ボタンはクリック可能。
"""
from __future__ import annotations

import math

import numpy as np
import pygame

from core.shared_state import SharedState

PANEL_W = 240
MAX_RANGE = 12.0

# 色
C_BG = (18, 18, 22)
C_PANEL = (30, 30, 38)
C_WALL = (235, 235, 235)
C_CENTER = (0, 200, 200)
C_RACING = (240, 210, 0)
C_LIDAR = (230, 60, 60)
C_VEHICLE = (60, 130, 240)
C_EST = (240, 150, 40)        # SLAM 推定位置（オレンジ）
C_TEXT = (220, 220, 220)
C_TEXT_DIM = (140, 140, 150)
C_BTN = (55, 55, 70)
C_BTN_ACTIVE = (70, 110, 180)


class SimRenderer:
    def __init__(self, config: dict, shared_state: SharedState,
                 courses: dict, current_course_id: str,
                 get_course_render, callbacks: dict) -> None:
        """
        config            : sim.yaml の simulation 節
        shared_state      : 描画データ源
        courses           : {id: {"name":...}}
        current_course_id : 現在のコース id
        get_course_render : () -> {"walls","center_line","racing_line"}
        callbacks         : {"course_change","pause_toggle","reset","speed_change"}
        """
        self.config = config
        self.shared = shared_state
        self.courses = courses
        self.current_course_id = current_course_id
        self.get_course_render = get_course_render
        self.cb = callbacks

        self.width = int(config.get("screen_width", 1200))
        self.height = int(config.get("screen_height", 800))
        self.show_center = bool(config.get("show_center_line", False))
        self.show_racing = bool(config.get("show_racing_line", False))
        self.speed_multipliers = list(config.get("speed_multipliers", [0.5, 1.0, 2.0]))

        self._draw_w = self.width - PANEL_W
        self._running = True

        # スケーリングパラメータ（_compute_transform で更新）
        self._scale = 50.0
        self._off = (0.0, 0.0)

        # クリック領域 {name: pygame.Rect}
        self._hitboxes: dict = {}

        pygame.init()
        pygame.display.set_caption("SURGE Mark.2 — SIM Viewer")
        self.screen = pygame.display.set_mode((self.width, self.height))
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("menlo,consolas,monospace", 14)
        self.font_big = pygame.font.SysFont("menlo,consolas,monospace", 18, bold=True)

    # ---- 座標変換 ---------------------------------------------------------
    def _compute_transform(self, walls) -> None:
        if walls:
            pts = np.array(walls, dtype=float).reshape(-1, 2)
            min_x, min_y = pts.min(axis=0)
            max_x, max_y = pts.max(axis=0)
        else:
            min_x, min_y, max_x, max_y = 0.0, 0.0, 6.0, 4.0

        margin = float(self.config.get("map_margin", 0.5))
        min_x -= margin; min_y -= margin
        max_x += margin; max_y += margin

        span_x = max(max_x - min_x, 1e-3)
        span_y = max(max_y - min_y, 1e-3)
        scale = min(self._draw_w / span_x, self.height / span_y)

        # 中央寄せ
        used_w = span_x * scale
        used_h = span_y * scale
        off_x = (self._draw_w - used_w) / 2.0 - min_x * scale
        off_y = (self.height - used_h) / 2.0 - min_y * scale
        self._scale = scale
        self._off = (off_x, off_y)

    def _w2s(self, x: float, y: float) -> tuple[int, int]:
        sx = x * self._scale + self._off[0]
        sy = self.height - (y * self._scale + self._off[1])   # y 反転
        return int(sx), int(sy)

    # ---- メインループ -----------------------------------------------------
    def run(self) -> None:
        while self._running:
            self._handle_events()
            self._draw()
            pygame.display.flip()
            self.clock.tick(60)
        pygame.quit()

    def stop(self) -> None:
        self._running = False

    def _handle_events(self) -> None:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                self._running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    self._running = False
                elif ev.key == pygame.K_SPACE:
                    self._fire("pause_toggle")
            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                self._handle_click(ev.pos)

    def _handle_click(self, pos) -> None:
        for name, rect in self._hitboxes.items():
            if rect.collidepoint(pos):
                if name.startswith("course:"):
                    cid = name.split(":", 1)[1]
                    self.current_course_id = cid
                    self._fire("course_change", cid)
                elif name == "pause":
                    self._fire("pause_toggle")
                elif name == "reset":
                    self._fire("reset")
                elif name == "speed":
                    self._cycle_speed()
                break

    def _cycle_speed(self) -> None:
        cur = self.shared.get_speed_multiplier()
        try:
            idx = self.speed_multipliers.index(cur)
        except ValueError:
            idx = self.speed_multipliers.index(1.0) if 1.0 in self.speed_multipliers else 0
        nxt = self.speed_multipliers[(idx + 1) % len(self.speed_multipliers)]
        self._fire("speed_change", nxt)

    def _fire(self, name: str, *args) -> None:
        fn = self.cb.get(name)
        if fn is not None:
            fn(*args)

    # ---- 描画 -------------------------------------------------------------
    def _draw(self) -> None:
        self.screen.fill(C_BG)
        self._hitboxes = {}

        course = self.get_course_render() or {}
        walls = course.get("walls") or []
        self._compute_transform(walls)

        # Layer0: 壁
        for (a, b) in walls:
            pygame.draw.line(self.screen, C_WALL, self._w2s(*a), self._w2s(*b), 2)

        # Layer1: 中心線（シアン破線）
        if self.show_center:
            self._draw_polyline(course.get("center_line"), C_CENTER, dashed=True, closed=True)

        # Layer2: レーシングライン（黄）
        if self.show_racing:
            self._draw_polyline(course.get("racing_line"), C_RACING, dashed=False, closed=True)

        # Layer3: LiDAR スキャン点（赤）
        self._draw_lidar()

        # Layer4: 車両（青三角形）
        self._draw_vehicle()

        # Layer5: UI パネル
        self._draw_panel()

        # フッタ
        footer = self.font.render(
            "SPACE:Pause  ESC:Quit  青=真値 橙=SLAM推定  ※操作はブラウザで行う", True, C_TEXT_DIM)
        self.screen.blit(footer, (12, self.height - 24))

    def _draw_polyline(self, pts, color, dashed=False, closed=False) -> None:
        if pts is None or len(pts) < 2:
            return
        pts = list(pts)
        if closed:
            pts = pts + [pts[0]]
        screen_pts = [self._w2s(p[0], p[1]) for p in pts]
        if dashed:
            for i in range(len(screen_pts) - 1):
                if i % 2 == 0:
                    pygame.draw.line(self.screen, color, screen_pts[i], screen_pts[i + 1], 2)
        else:
            pygame.draw.lines(self.screen, color, False, screen_pts, 2)

    def _draw_lidar(self) -> None:
        scan = self.shared.get_lidar()
        veh = self.shared.get_vehicle()
        if scan is None:
            return
        d = np.asarray(scan.distances)
        ang = np.radians(np.asarray(scan.angles) + veh.heading)
        hit = d < MAX_RANGE - 1e-3
        xs = veh.x + d * np.cos(ang)
        ys = veh.y + d * np.sin(ang)
        for i in np.where(hit)[0]:
            pygame.draw.circle(self.screen, C_LIDAR, self._w2s(xs[i], ys[i]), 2)

    def _draw_vehicle(self) -> None:
        # 真値（青三角）
        veh = self.shared.get_vehicle()
        self._draw_triangle(veh.x, veh.y, veh.heading, C_VEHICLE)

        # SLAM 推定位置（オレンジ）：真値とのズレを目視できる
        loc = self.shared.get_localization()
        if loc is not None and loc.source == "slam":
            self._draw_triangle(loc.x, loc.y, loc.heading, C_EST, filled=False)

    def _draw_triangle(self, x, y, heading_deg, color, filled=True) -> None:
        h = math.radians(heading_deg)
        nose = (x + 0.16 * math.cos(h), y + 0.16 * math.sin(h))
        left = (x + 0.10 * math.cos(h + 2.5), y + 0.10 * math.sin(h + 2.5))
        right = (x + 0.10 * math.cos(h - 2.5), y + 0.10 * math.sin(h - 2.5))
        pts = [self._w2s(*nose), self._w2s(*left), self._w2s(*right)]
        pygame.draw.polygon(self.screen, color, pts, 0 if filled else 2)

    def _draw_panel(self) -> None:
        x0 = self.width - PANEL_W
        pygame.draw.rect(self.screen, C_PANEL, (x0, 0, PANEL_W, self.height))

        y = 16
        self.screen.blit(self.font_big.render("COURSE SELECT", True, C_TEXT), (x0 + 16, y))
        y += 30
        for cid, c in self.courses.items():
            rect = pygame.Rect(x0 + 12, y, PANEL_W - 24, 28)
            active = (cid == self.current_course_id)
            pygame.draw.rect(self.screen, C_BTN_ACTIVE if active else C_BTN, rect, border_radius=4)
            label = c["name"][:20]
            self.screen.blit(self.font.render(label, True, C_TEXT), (rect.x + 8, rect.y + 6))
            self._hitboxes[f"course:{cid}"] = rect
            y += 34

        y += 16
        self.screen.blit(self.font_big.render("SIM CTRL", True, C_TEXT), (x0 + 16, y))
        y += 30

        paused = self.shared.is_paused()
        pause_rect = pygame.Rect(x0 + 12, y, PANEL_W - 24, 28)
        pygame.draw.rect(self.screen, C_BTN_ACTIVE if paused else C_BTN, pause_rect, border_radius=4)
        self.screen.blit(self.font.render("▶ Resume" if paused else "⏸ Pause", True, C_TEXT),
                         (pause_rect.x + 8, pause_rect.y + 6))
        self._hitboxes["pause"] = pause_rect
        y += 34

        reset_rect = pygame.Rect(x0 + 12, y, PANEL_W - 24, 28)
        pygame.draw.rect(self.screen, C_BTN, reset_rect, border_radius=4)
        self.screen.blit(self.font.render("↺ Reset", True, C_TEXT),
                         (reset_rect.x + 8, reset_rect.y + 6))
        self._hitboxes["reset"] = reset_rect
        y += 34

        speed_rect = pygame.Rect(x0 + 12, y, PANEL_W - 24, 28)
        pygame.draw.rect(self.screen, C_BTN, speed_rect, border_radius=4)
        mult = self.shared.get_speed_multiplier()
        self.screen.blit(self.font.render(f"Speed: {mult:g}x  (click)", True, C_TEXT),
                         (speed_rect.x + 8, speed_rect.y + 6))
        self._hitboxes["speed"] = speed_rect
        y += 44

        # 読み取り専用 STATUS
        self.screen.blit(self.font_big.render("STATUS", True, C_TEXT), (x0 + 16, y))
        y += 28
        veh = self.shared.get_vehicle()
        mode = self.shared.get_mode().value
        lines = [
            f"mode : {mode}",
            f"spd  : {veh.speed:5.2f} m/s",
            f"steer: {veh.steer_angle:5.1f} deg",
            f"pos  : {veh.x:4.2f},{veh.y:4.2f}",
            f"hdg  : {veh.heading:5.1f} deg",
            f"paused: {paused}",
        ]
        for ln in lines:
            self.screen.blit(self.font.render(ln, True, C_TEXT_DIM), (x0 + 16, y))
            y += 20
