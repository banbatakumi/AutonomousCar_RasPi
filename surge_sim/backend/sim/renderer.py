"""pygame描画エンジン。

UIは `UIView`（core/telemetry.py）だけを読んで描画する。SIMローカルでも
ネットワーク(実機・遠隔)でも、UI側のコードは同じ UIView を消費するだけ。

- 読み取り(描画): get_view() が返す UIView（テレメトリ＋LiDAR＋シーン）
- 書き込み(操作): controller への直接呼び出し（ステップ③でCommandFrame化予定）

画面レイアウト 1800x900（ディスプレイに合わせ自動縮小）:
    ┌──────────────┬──────────────┬─────────┐
    │ LEFT (sim)   │ RIGHT(SLAM/  │ INFO    │
    │              │   Graph tab) │ COURSE  │
    └──────────────┴──────────────┴─────────┘

操作: ↑↓:Speed ←→:Steer A:Auto(map) F:Reactive(LiDAR) R:Reset SPACE:Pause TAB:RightPane ESC:Quit
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np
import pygame

# ---- 色定義 --------------------------------------------------------------
C_BG = (18, 18, 22)
C_PANE_BG = (10, 10, 14)
C_PANEL_BG = (28, 28, 34)
C_WALL = (235, 235, 235)
C_LIDAR = (235, 70, 70)
C_VEHICLE = (70, 140, 235)
C_TRUTH = (120, 120, 130)
C_CENTER = (40, 210, 210)
C_RACING = (235, 210, 40)
C_SLAM = (60, 200, 120)
C_TEXT = (230, 230, 230)
C_TEXT_DIM = (150, 150, 155)
C_ACCENT = (90, 170, 250)
C_GRID = (55, 55, 62)
C_BTN = (55, 58, 70)
C_BTN_ACTIVE = (70, 130, 220)
C_BTN_DISABLED = (40, 40, 46)
C_GRAPH_SPEED = (90, 200, 250)
C_GRAPH_STEER = (250, 180, 80)
C_SEP = (60, 60, 70)


class SimRenderer:
    """シミュレータ・ビューア（描画専用）。

    操作系（速度/ステア/モード/マッピング/コース切替等）はすべて Web UI が担い、
    pygame はシミュレーション描画・コース描画・計器表示のみを行う。
    描画データは get_view()(UIView) と、SLAM成果物は controller から読み取る。
    """

    INFO_W = 300

    def __init__(self, config: dict, course_names: list, get_view, controller):
        """
        Args:
            config: 統合設定（vehicle/simulation/display等）
            course_names: コース名リスト（表示用）
            get_view: () -> UIView  描画データ取得（in-process or network）
            controller: SLAM成果物(occupancy/racing_line)の読み取り元（描画専用）
        """
        self.config = config
        self.course_names = list(course_names)
        self.get_view = get_view
        self.controller = controller

        sim = config["simulation"]
        self.base_w = int(sim["screen_width"])
        self.base_h = int(sim["screen_height"])
        self.margin = float(sim["map_margin"])
        self.speed_multipliers = list(sim.get("speed_multipliers", [0.5, 1.0, 2.0]))

        disp = config.get("display", {})
        self.show_lidar = bool(disp.get("show_lidar", True))
        self.show_center_line = bool(disp.get("show_center_line", False))
        self.show_racing_line = bool(disp.get("show_racing_line", False))
        self.show_slam_overlay = bool(disp.get("show_slam_overlay", False))
        self.graph_seconds = float(disp.get("graph_buffer_seconds", 5.0))

        pygame.init()
        pygame.display.set_caption("SURGE Mark.2 Simulator")

        # ディスプレイに収まるよう自動縮小
        self.ui_scale = self._fit_scale(self.base_w, self.base_h)
        self.screen_w = int(self.base_w * self.ui_scale)
        self.screen_h = int(self.base_h * self.ui_scale)
        info_w = int(self.INFO_W * self.ui_scale)

        pane_area = self.screen_w - info_w
        half = pane_area // 2
        self.left_rect = pygame.Rect(0, 0, half, self.screen_h)
        self.right_rect = pygame.Rect(half, 0, pane_area - half, self.screen_h)
        self.info_rect = pygame.Rect(pane_area, 0, info_w, self.screen_h)

        self.right_tab = "graph"
        self.graph_buf: deque = deque(maxlen=2000)

        self._scale = 1.0
        self._off_x = self._off_y = 0.0
        self._min_x = self._max_y = 0.0

        self.screen = pygame.display.set_mode((self.screen_w, self.screen_h))
        self.clock = pygame.time.Clock()
        self.font_s = pygame.font.SysFont("Menlo,Monaco,monospace", max(11, int(14 * self.ui_scale)))
        self.font_m = pygame.font.SysFont("Menlo,Monaco,monospace", max(13, int(18 * self.ui_scale)))
        self.font_l = pygame.font.SysFont("Menlo,Monaco,monospace", max(15, int(22 * self.ui_scale)), bold=True)

        self._current_course_name = None

        # 初期ビューでシーンを取得しスケール確定
        view = self.get_view()
        self._scene = view.scene
        self._scene_id = id(view.scene)
        self._compute_scale()

    # ------------------------------------------------------------------
    def _fit_scale(self, w: int, h: int) -> float:
        try:
            info = pygame.display.Info()
            desk_w, desk_h = info.current_w, info.current_h
        except pygame.error:
            return 1.0
        if desk_w <= 0 or desk_h <= 0:
            return 1.0
        avail_w = desk_w * 0.98
        avail_h = desk_h * 0.92
        return min(avail_w / w, avail_h / h, 1.0)

    # ==================================================================
    # スケーリング
    # ==================================================================
    def _scene_walls(self) -> list:
        """シーンの壁 [[ [x1,y1],[x2,y2] ], ...]（無ければ空）。"""
        return self._scene.walls or []

    def _compute_scale(self) -> None:
        xs, ys = [], []
        for seg in self._scene_walls():
            (x1, y1), (x2, y2) = seg
            xs += [x1, x2]
            ys += [y1, y2]
        if not xs:
            xs, ys = [0, 1], [0, 1]
        min_x, max_x = min(xs) - self.margin, max(xs) + self.margin
        min_y, max_y = min(ys) - self.margin, max(ys) + self.margin
        world_w = max(max_x - min_x, 1e-3)
        world_h = max(max_y - min_y, 1e-3)

        pad = 20
        avail_w = self.left_rect.width - 2 * pad
        avail_h = self.left_rect.height - 2 * pad
        self._scale = min(avail_w / world_w, avail_h / world_h)

        draw_w = world_w * self._scale
        draw_h = world_h * self._scale
        self._off_x = self.left_rect.left + pad + (avail_w - draw_w) / 2.0
        self._off_y = self.left_rect.top + pad + (avail_h - draw_h) / 2.0
        self._min_x = min_x
        self._max_y = max_y

    def w2s(self, x: float, y: float) -> tuple[int, int]:
        sx = self._off_x + (x - self._min_x) * self._scale
        sy = self._off_y + (self._max_y - y) * self._scale
        return int(sx), int(sy)

    # ==================================================================
    # メインループ
    # ==================================================================
    def run(self) -> None:
        import os
        debug = os.environ.get("SURGE_DEBUG")
        frames = 0
        running = True
        try:
            while running:
                frame_dt = self.clock.tick(60) / 1000.0
                view = self.get_view()
                self._sync_scene(view)
                running = self._handle_events()
                self._update_graph_buffer(view)
                self._draw(view)
                pygame.display.flip()
                frames += 1
                if debug and frames <= 3:
                    print(f"[SURGE_DEBUG] frame {frames} drawn", flush=True)
        finally:
            if debug:
                print(f"[SURGE_DEBUG] loop exited after {frames} frames", flush=True)
            pygame.quit()

    def _sync_scene(self, view) -> None:
        """シーンが変わったら（コース切替）スケール再計算＋グラフクリア。"""
        if id(view.scene) != self._scene_id:
            self._scene = view.scene
            self._scene_id = id(view.scene)
            self._compute_scale()
            self.graph_buf.clear()

    # ==================================================================
    # イベント処理（ビューア専用：操作系はWeb UI。pygameは表示とビュー切替のみ）
    # ==================================================================
    def _handle_events(self) -> bool:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return False
                if event.key == pygame.K_TAB:   # 右ペイン切替（表示のみ）
                    self.right_tab = "slam" if self.right_tab == "graph" else "graph"
        return True

    # ==================================================================
    # グラフバッファ
    # ==================================================================
    def _update_graph_buffer(self, view) -> None:
        f = view.frame
        self.graph_buf.append((f.t, f.vehicle["speed"], f.vehicle["steer"]))
        t_now = f.t
        while self.graph_buf and (t_now - self.graph_buf[0][0]) > self.graph_seconds:
            self.graph_buf.popleft()

    # ==================================================================
    # 描画
    # ==================================================================
    def _draw(self, view) -> None:
        self.screen.fill(C_BG)
        self._draw_left_pane(view)
        self._draw_right_pane(view)
        self._draw_info_panel(view)
        self._draw_hint_bar()

    # ----- 左ペイン ---------------------------------------------------
    def _draw_left_pane(self, view) -> None:
        f = view.frame
        pygame.draw.rect(self.screen, C_PANE_BG, self.left_rect)
        prev_clip = self.screen.get_clip()
        self.screen.set_clip(self.left_rect)

        # Layer0: 壁
        for seg in self._scene_walls():
            (x1, y1), (x2, y2) = seg
            pygame.draw.line(self.screen, C_WALL, self.w2s(x1, y1), self.w2s(x2, y2), 2)

        # Layer1: SLAM占有格子オーバーレイ（構築済みなら）
        occ = getattr(self.controller, "occupancy", None)
        if occ is not None:
            self._draw_occupancy(occ)

        # Layer2/3: 中心線・レーシングライン
        slam_center = getattr(self.controller, "slam_center", None)
        racing = getattr(self.controller, "racing_line", None)
        if slam_center is not None:
            self._draw_path(np.asarray(slam_center, dtype=float), C_CENTER, dashed=True)
        else:
            cl = self._scene.center_line
            if (f.drive_mode == "auto" or (self.show_center_line and f.drive_mode != "reactive")) and cl:
                self._draw_path(np.asarray(cl, dtype=float), C_CENTER, dashed=True)
        if racing is not None:
            self._draw_path(np.asarray(racing, dtype=float), C_RACING, dashed=False)

        # Layer4: LiDAR点（推定姿勢を基準に変換）
        if self.show_lidar and view.lidar is not None:
            self._draw_lidar(view.lidar, f.pose_est)

        # 進路ターゲット: auto=黄(先読み点), reactive=緑(ギャップ方向)
        tp = f.planner.get("target_point")
        if f.drive_mode != "manual" and tp is not None:
            color = C_SLAM if f.drive_mode == "reactive" else C_RACING
            tx, ty = self.w2s(tp[0], tp[1])
            vx, vy = self.w2s(f.pose_est["x"], f.pose_est["y"])
            pygame.draw.line(self.screen, color, (vx, vy), (tx, ty), 2)
            pygame.draw.circle(self.screen, color, (tx, ty), self.s(7), 2)

        # 真値オーバーレイ（SIMデバッグ用・推定とズレた時のみ表示）
        pt = f.pose_truth
        if pt is not None and self._pose_differs(pt, f.pose_est):
            self._draw_vehicle(pt["x"], pt["y"], pt["heading"], C_TRUTH, outline_only=True)

        # Layer5: 車両（推定姿勢で描画）
        pe = f.pose_est
        self._draw_vehicle(pe["x"], pe["y"], pe["heading"], C_VEHICLE)

        self.screen.set_clip(prev_clip)
        self._label(self.font_m, "SIMULATION", self.left_rect.left + 12,
                    self.left_rect.top + 8, C_TEXT_DIM)
        pygame.draw.line(self.screen, C_SEP,
                         (self.left_rect.right, 0), (self.left_rect.right, self.screen_h), 1)

    @staticmethod
    def _pose_differs(a: dict, b: dict) -> bool:
        return (abs(a["x"] - b["x"]) > 0.02 or abs(a["y"] - b["y"]) > 0.02
                or abs(a["heading"] - b["heading"]) > 1.0)

    def _draw_lidar(self, scan, pose: dict) -> None:
        ang = np.radians(scan.angles) + math.radians(pose["heading"])
        d = scan.distances
        valid = d < (12.0 - 1e-3)
        xs = pose["x"] + d * np.cos(ang)
        ys = pose["y"] + d * np.sin(ang)
        for i in np.where(valid)[0]:
            sx, sy = self.w2s(xs[i], ys[i])
            pygame.draw.circle(self.screen, C_LIDAR, (sx, sy), 2)

    def _draw_occupancy(self, grid) -> None:
        """占有格子の occupied セルを半透明グリーンで重ねる。"""
        g = grid.grid
        occ = np.argwhere(g == 1)
        if occ.size == 0:
            return
        res = grid.resolution
        sz = max(int(res * self._scale) + 1, 2)
        col = (40, 150, 90)
        for (cy, cx) in occ:
            wx = grid.origin_x + (cx + 0.5) * res
            wy = grid.origin_y + (cy + 0.5) * res
            sx, sy = self.w2s(wx, wy)
            pygame.draw.rect(self.screen, col, (sx - sz // 2, sy - sz // 2, sz, sz))

    def _draw_path(self, path, color, dashed: bool = False) -> None:
        n = len(path)
        if n < 2:
            return
        for i in range(n):
            if dashed and (i % 2 == 1):
                continue
            a = path[i]
            b = path[(i + 1) % n]
            pygame.draw.line(self.screen, color, self.w2s(a[0], a[1]),
                             self.w2s(b[0], b[1]), 2)

    def _draw_vehicle(self, x, y, heading, color, outline_only: bool = False) -> None:
        L = 0.18 * self._scale
        W = 0.10 * self._scale
        cx, cy = self.w2s(x, y)
        h = math.radians(heading)
        tip = (cx + L * math.cos(h), cy - L * math.sin(h))
        rear_l = (cx - 0.5 * L * math.cos(h) - W * math.sin(h),
                  cy + 0.5 * L * math.sin(h) - W * math.cos(h))
        rear_r = (cx - 0.5 * L * math.cos(h) + W * math.sin(h),
                  cy + 0.5 * L * math.sin(h) + W * math.cos(h))
        if outline_only:
            pygame.draw.polygon(self.screen, color, [tip, rear_l, rear_r], 1)
        else:
            pygame.draw.polygon(self.screen, color, [tip, rear_l, rear_r])
            pygame.draw.polygon(self.screen, (200, 220, 255), [tip, rear_l, rear_r], 1)

    # ----- 右ペイン ---------------------------------------------------
    def _draw_right_pane(self, view) -> None:
        pygame.draw.rect(self.screen, C_PANE_BG, self.right_rect)
        tab_y = self.right_rect.top + self.s(10)
        tab_w = self.s(120)
        tab_h = self.s(34)
        x = self.right_rect.left + self.s(12)
        for name, label in [("slam", "SLAM"), ("graph", "Graph")]:
            rect = pygame.Rect(x, tab_y, tab_w, tab_h)
            active = (self.right_tab == name)
            pygame.draw.rect(self.screen, C_BTN_ACTIVE if active else C_BTN,
                             rect, border_radius=5)
            self._label_center(self.font_m, label, rect, C_TEXT)
            x += tab_w + self.s(10)

        content = pygame.Rect(self.right_rect.left + self.s(12), tab_y + tab_h + self.s(14),
                              self.right_rect.width - self.s(24),
                              self.right_rect.height - tab_h - self.s(40))
        if self.right_tab == "graph":
            self._draw_graph(content)
        else:
            self._draw_slam_placeholder(content)

        pygame.draw.line(self.screen, C_SEP,
                         (self.right_rect.right, 0), (self.right_rect.right, self.screen_h), 1)

    def _draw_graph(self, area: pygame.Rect) -> None:
        gap = 30
        h = (area.height - gap) // 2
        speed_rect = pygame.Rect(area.left, area.top, area.width, h)
        steer_rect = pygame.Rect(area.left, area.top + h + gap, area.width, h)

        data = list(self.graph_buf)
        times = np.array([d[0] for d in data]) if data else np.array([])
        speeds = np.array([d[1] for d in data]) if data else np.array([])
        steers = np.array([d[2] for d in data]) if data else np.array([])

        max_speed = float(self.config["vehicle"]["max_speed"])
        max_steer = float(self.config["vehicle"]["max_steer_angle"])

        self._plot(speed_rect, "Speed [m/s]", times, speeds, -max_speed, max_speed, C_GRAPH_SPEED)
        self._plot(steer_rect, "Steer [deg]", times, steers, -max_steer, max_steer, C_GRAPH_STEER)

    def _plot(self, rect, title, times, values, vmin, vmax, color) -> None:
        pygame.draw.rect(self.screen, C_PANEL_BG, rect, border_radius=4)
        pygame.draw.rect(self.screen, C_GRID, rect, 1, border_radius=4)
        self._label(self.font_s, title, rect.left + 8, rect.top + 6, C_TEXT_DIM)
        zero_y = rect.bottom - (0 - vmin) / (vmax - vmin) * rect.height
        pygame.draw.line(self.screen, C_GRID, (rect.left, int(zero_y)), (rect.right, int(zero_y)), 1)
        self._label(self.font_s, f"{vmax:+.1f}", rect.right - 50, rect.top + 6, C_TEXT_DIM)
        self._label(self.font_s, f"{vmin:+.1f}", rect.right - 50, rect.bottom - 20, C_TEXT_DIM)
        if times.size < 2:
            return
        t0, t1 = times[0], times[-1]
        span = max(t1 - t0, self.graph_seconds * 0.25)
        pts = []
        for t, vv in zip(times, values):
            px = min(rect.left + (t - t0) / span * rect.width, rect.right)
            v = max(vmin, min(vmax, vv))
            py = rect.bottom - (v - vmin) / (vmax - vmin) * rect.height
            pts.append((px, py))
        if len(pts) >= 2:
            pygame.draw.lines(self.screen, color, False, pts, 2)

    def _draw_slam_placeholder(self, area: pygame.Rect) -> None:
        pygame.draw.rect(self.screen, C_PANEL_BG, area, border_radius=4)
        msg = "SLAM map — Phase3で有効化"
        surf = self.font_m.render(msg, True, C_TEXT_DIM)
        self.screen.blit(surf, (area.centerx - surf.get_width() // 2,
                                area.centery - surf.get_height() // 2))

    # ----- INFOパネル -------------------------------------------------
    def _draw_info_panel(self, view) -> None:
        f = view.frame
        pygame.draw.rect(self.screen, C_PANEL_BG, self.info_rect)
        x = self.info_rect.left + self.s(16)
        y = self.s(14)

        self._label(self.font_l, "INFO", x, y, C_ACCENT)
        y += self.s(36)

        veh = f.vehicle
        pe = f.pose_est
        lidar_min = 12.0
        if view.lidar is not None and view.lidar.distances.size:
            lidar_min = float(np.min(view.lidar.distances))

        mode_label = {"manual": "MANUAL", "auto": "AUTO(map)",
                      "reactive": "REACTIVE"}.get(f.drive_mode, f.drive_mode)
        mapping = getattr(self.controller, "mapping", False)
        has_rl = getattr(self.controller, "racing_line", None) is not None
        slam_txt = "MAPPING" if mapping else ("RL ready" if has_rl else "—")
        rows = [
            ("Mode", mode_label),
            ("SLAM", slam_txt),
            ("Speed", f"{veh['speed']:6.2f} m/s"),
            ("Accel", f"{veh['accel']:6.2f} m/s2"),
            ("Steer", f"{veh['steer']:6.2f} deg"),
            ("X", f"{pe['x']:6.2f} m"),
            ("Y", f"{pe['y']:6.2f} m"),
            ("Heading", f"{pe['heading']:6.2f} deg"),
            ("LiDAR Min", f"{lidar_min:6.2f} m"),
            ("Loc src", f"{pe['src']:>8}"),
            ("Time", f"{f.t:6.2f} s"),
        ]
        for k, vtxt in rows:
            self._label(self.font_s, k, x, y, C_TEXT_DIM)
            self._label(self.font_m, vtxt, x + self.s(96), y - 2, C_TEXT)
            y += self.s(26)

        y += self.s(6)
        pygame.draw.line(self.screen, C_SEP, (self.info_rect.left + self.s(10), y),
                         (self.info_rect.right - self.s(10), y), 1)
        y += self.s(12)

        # STATUS（読み取り専用。操作は Web UI）
        sc = f.sim_ctrl or {"paused": False, "speed_mult": 1.0}
        paused = bool(sc.get("paused", False))
        mult = float(sc.get("speed_mult", 1.0))
        self._label(self.font_l, "STATUS", x, y, C_ACCENT)
        y += self.s(32)
        status = "PAUSED" if paused else "RUNNING"
        for k, vtxt, col in [
            ("State", status, C_GRAPH_STEER if paused else C_SLAM),
            ("Course", self._current_course_name or "—", C_TEXT),
            ("Speed x", f"{mult:g}x", C_TEXT),
        ]:
            self._label(self.font_s, k, x, y, C_TEXT_DIM)
            self._label(self.font_m, vtxt, x + self.s(96), y - 2, col)
            y += self.s(26)

        y += self.s(10)
        pygame.draw.line(self.screen, C_SEP, (self.info_rect.left + self.s(10), y),
                         (self.info_rect.right - self.s(10), y), 1)
        y += self.s(14)
        self._label(self.font_s, "VIEW ONLY", x, y, C_TEXT_DIM)
        y += self.s(22)
        self._label(self.font_s, "操作は Web UI から", x, y, C_TEXT_DIM)

    def _draw_hint_bar(self) -> None:
        hint = "VIEWER (操作はWeb UI)   TAB:右ペイン切替   ESC:終了"
        surf = self.font_s.render(hint, True, C_TEXT_DIM)
        self.screen.blit(surf, (self.s(12), self.screen_h - self.s(22)))

    # ----- 描画ヘルパ -------------------------------------------------
    def s(self, px: float) -> int:
        return int(px * self.ui_scale)

    def _label(self, font, text, x, y, color) -> None:
        self.screen.blit(font.render(text, True, color), (x, y))

    def _label_center(self, font, text, rect, color) -> None:
        surf = font.render(text, True, color)
        self.screen.blit(surf, (rect.centerx - surf.get_width() // 2,
                                rect.centery - surf.get_height() // 2))

    def set_current_course(self, name: str) -> None:
        self._current_course_name = name
