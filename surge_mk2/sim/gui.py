"""シミュレータGUI — 俯瞰ビューと、シミュレーションのパラメータ調整。

    python -m sim.gui

**シミュレーション固有の操作はすべてここにある。** コース切替・LiDAR のノイズ量・
欠損率・遅延・リンクのビット誤り率。これらは実機に対応物が無い概念なので、
実機 GUI（React）には持ち込まない（`sim/__init__.py` の設計の原則）。

実機 GUI に出るのは「SIM バッジ」だけで、それは利便性ではなく安全のため。

## 画面の作り

右のパネルは**常時は走行の数値**（速度・舵角・位置・超音波）を出す。運転中に読むのは
こちらで、シムのパラメータは set-and-forget なので `P` で開くページに退けてある
（実機 GUI が設定を ⚙ ドロワーに隠しているのと同じ作法）。

ウィンドウは**リサイズできる**。コースは横長なので表示倍率は幅で頭打ちになり、
レイアウトを詰めるより窓を広げる方がずっと効く（1240px で 88px/m、1600px で 126px/m）。

## シムには干渉しない（が、窓を閉じたら終わり）

`sim/truth` を受けて描くだけ。操作は `sim/ctrl` に投げる。シミュレーション本体の
状態は一切持たない。

ただし `sim.run` から起動された場合、**この窓を閉じるとセッション全体が終わる**
（端末に戻って Ctrl-C を押させないため）。異常終了（二重起動・描画のバグ）は
巻き添えにしない — `sim.run` が終了コードで区別している。

## 見た目について

pygame は放っておくと安っぽく見える。効いたのは次の3つ。

1. **コース PNG を生で貼らない。** 白背景＋黒壁のまま貼るとダークな画面から浮いて
   「スクショを貼った」ように見える。占有格子から作り直し、壁の縁だけ明るくする
2. **アンチエイリアスを使う。** `set_at` の 1px 点と `draw.polygon` のギザギザが
   安っぽさの正体。点はグロー付きのスプライトを blit し、多角形は `gfxdraw` で描く
3. **余白と字を規則に乗せる。** 余白は 4px の倍数だけ、数字は等幅（Menlo）で桁を
   揃える、見出しは字間を空けた大文字

macOS の SDL は高 DPI のバッキングストアを返さない（surface と window が同寸）ので、
Retina での物理解像度は稼げない。**そこは諦めて、上の3つで見た目を作る。**
"""

from __future__ import annotations

import math
import signal
import subprocess
import sys
from collections import deque
from pathlib import Path

import pygame
import pygame.gfxdraw

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim import ui as U                    # noqa: E402
from sim.channel import ViewerChannel      # noqa: E402
from sim.course import Course              # noqa: E402
from sim.params import TUNABLES            # noqa: E402
from sim.ui import (                       # noqa: E402
    ACCENT, BAD, BG, DIM, FAINT, FG, HAIR, LINE, OK, PAD, PANEL, PANEL2, POINT,
    TRAIL, WARN)

ROOT = Path(__file__).resolve().parents[1]

INIT_W, INIT_H = 1400, 900
MIN_W, MIN_H = 760, 520
PANEL_W = 300

TRAIL_MAX = 1500
STALE_MS = 700

class Act:
    """パネル下部の操作ボタン。**ショートカットを覚えなくても全部の機能に届く**ため、
    キーに割り当てた操作はここにも必ず出す。ラベルにキーを併記して学習にもなるように。
    """

    __slots__ = ("rect", "fn", "label", "color")

    def __init__(self, rect, fn, label: str, color=None) -> None:
        self.rect, self.fn, self.label, self.color = rect, fn, label, color


class Row:
    """設定ページの1行。"""

    __slots__ = ("kind", "key", "label", "rect")

    def __init__(self, kind: str, key: str, label: str, rect) -> None:
        self.kind = kind          # "course" | "param"
        self.key = key
        self.label = label
        self.rect = rect


class Viewer:
    def __init__(self) -> None:
        pygame.init()
        pygame.display.set_caption("SURGE Mk.2 シミュレータ")
        self.screen = pygame.display.set_mode((INIT_W, INIT_H), pygame.RESIZABLE)
        self.clock = pygame.time.Clock()

        self.ui = U.Ui()

        self.ch = ViewerChannel()
        self.truth: dict | None = None
        self.last_rx_ms = 0
        self.trail: deque[tuple[float, float]] = deque(maxlen=TRAIL_MAX)
        self.rows: list[Row] = []
        self.acts: list[Act] = []
        self.hover_act = -1
        self.sel = 0
        self.hover = -1
        self.settings = False          #: True なら右パネルは設定ページ
        self._trail_course = ""
        self._course_cache: dict[tuple, Course] = {}
        self._editor = None            #: 起動したコースエディタ（別プロセス）
        self._msg = ""
        self._msg_until = 0
        self._dot = U.make_dot(POINT)
        self._resize(INIT_W, INIT_H)

    def _resize(self, w: int, h: int) -> None:
        self.w, self.h = max(MIN_W, w), max(MIN_H, h)
        self.view = pygame.Rect(0, 0, self.w - PANEL_W, self.h)
        self._overlay = pygame.Surface(self.view.size, pygame.SRCALPHA)
        U.clear_map_cache()            # 倍率が変わるので作り直す

    # ── ループ ──

    def run(self) -> None:
        # `kill` されたときも「窓を閉じた」と同じに畳む。**終了コードは 0 のまま。**
        # `sim.run` は 0 か否かで「人が閉じた」と「落ちた」を区別するので、
        # ここで例外を投げて非0で死ぬと、事故扱いされてシムが止まらなくなる
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: setattr(self, "_quit", True))
        self._quit = False
        running = True
        while running and not self._quit:
            running = self._events()
            t = self.ch.poll()
            if t is not None:
                self._on_truth(t)
            self._draw()
            self.clock.tick(60)
        self.ch.close()
        pygame.quit()

    def _on_truth(self, t: dict) -> None:
        first = self.truth is None
        self.truth = t
        self.last_rx_ms = pygame.time.get_ticks()
        # 起動直後の選択は「今走っているコース」に合わせる。既定で先頭を選んでいると、
        # 選択の帯と現在のコース（●）が別の行に出て、どちらが現在なのか紛らわしい
        if first and t["course_path"] in t["courses"]:
            self.sel = t["courses"].index(t["course_path"])
        if t["course"] != self._trail_course:
            self.trail.clear()
            self._trail_course = t["course"]
        p = (t["x"], t["y"])
        if not self.trail or (p[0] - self.trail[-1][0]) ** 2 + (p[1] - self.trail[-1][1]) ** 2 > 4e-4:
            self.trail.append(p)

    # ── 入力 ──

    def _events(self) -> bool:
        self.hover = self.hover_act = -1
        mouse = pygame.mouse.get_pos()
        for i, r in enumerate(self.rows):
            if r.rect.collidepoint(mouse):
                self.hover = i
        for i, a in enumerate(self.acts):
            if a.rect.collidepoint(mouse):
                self.hover_act = i
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                return False
            if e.type == pygame.VIDEORESIZE:
                self.screen = pygame.display.set_mode((max(MIN_W, e.w), max(MIN_H, e.h)),
                                                      pygame.RESIZABLE)
                self._resize(e.w, e.h)
            elif e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
                self._click(e.pos)
            elif e.type == pygame.MOUSEWHEEL and self.hover >= 0:
                self.sel = self.hover
                self._adjust(1 if e.y > 0 else -1, coarse=False)
            elif e.type == pygame.KEYDOWN:
                if not self._key(e):
                    return False
        return True

    def _click(self, pos) -> None:
        for a in self.acts:
            if a.rect.collidepoint(pos):
                a.fn()
                return
        for i, r in enumerate(self.rows):
            if r.rect.collidepoint(pos):
                self.sel = i
                if r.kind == "course":
                    self.ch.send(type="set_course", path=r.key)
                return

    def _key(self, e) -> bool:
        k, mods = e.key, e.mod
        if k in (pygame.K_ESCAPE, pygame.K_q):
            return False
        if k == pygame.K_m:
            self._open_editor()
        elif k == pygame.K_p:
            self.settings = not self.settings
        elif k == pygame.K_r:
            self._reset_pose()
        elif k == pygame.K_e:
            self._toggle_estop()
        elif k == pygame.K_t:
            self.trail.clear()
        elif k == pygame.K_n:
            self._next_course()
        elif k == pygame.K_d:
            self._reset_params()
        elif k in (pygame.K_UP, pygame.K_DOWN):
            self._move_sel(-1 if k == pygame.K_UP else 1)
        elif k in (pygame.K_LEFT, pygame.K_RIGHT):
            self._adjust(1 if k == pygame.K_RIGHT else -1, coarse=bool(mods & pygame.KMOD_SHIFT))
        elif k == pygame.K_RETURN:
            r = self.rows[self.sel] if self.sel < len(self.rows) else None
            if r and r.kind == "course":
                self.ch.send(type="set_course", path=r.key)
        return True

    def _move_sel(self, d: int) -> None:
        # 設定ページを閉じているときに ↑↓ を押したら、まず開く
        if not self.settings:
            self.settings = True
            return
        if self.rows:
            self.sel = (self.sel + d) % len(self.rows)

    def _adjust(self, sign: int, coarse: bool) -> None:
        if not self.settings:
            self.settings = True
            return
        if self.sel >= len(self.rows) or not self.truth:
            return
        r = self.rows[self.sel]
        if r.kind != "param":
            return
        t = next(t for t in TUNABLES if t.key == r.key)
        cur = self.truth["params"].get(r.key, 0.0)
        step = t.step * (10 if coarse else 1)
        self.ch.send(type="set_param", key=r.key,
                     value=max(t.lo, min(t.hi, cur + sign * step)))

    def _reset_params(self) -> None:
        from sim.params import SimParams
        d = SimParams()
        for t in TUNABLES:
            self.ch.send(type="set_param", key=t.key, value=getattr(d, t.key))

    def _open_editor(self) -> None:
        """コースエディタを**別プロセスで**開く。

        画面に組み込まないのは、ここが「走っている物を見る画面」で、エディタは
        「未保存の状態を持つ作成ツール」だから。同じプロセスに入れると
        シムを止めた瞬間に作りかけのコースが消えるし、走らせながら編集もできない。

        **`start_new_session=True` を外さないこと。** `sim.run` は子のプロセスグループごと
        止めるので、同じグループに入れるとシムを終了した瞬間にエディタも道連れになる。
        """
        if self._editor is not None and self._editor.poll() is None:
            self._flash("コースエディタは既に開いています")
            return
        args = [sys.executable, "-m", "sim.editor"]
        # 今のコースがセンターライン方式なら、それを開いた状態で始める
        cur = (self.truth or {}).get("course_path", "")
        if cur.endswith(".json"):
            args.append(cur)
        try:
            self._editor = subprocess.Popen(args, cwd=ROOT, start_new_session=True)
            self._flash("コースエディタを開きました（保存したら N で読み直し）")
        except Exception as e:
            self._flash(f"エディタを開けません: {e}")

    def _reset_pose(self) -> None:
        self.ch.send(type="reset")
        self.trail.clear()

    def _toggle_estop(self) -> None:
        self.ch.send(type="estop", value=not (self.truth or {}).get("estop", False))

    def _flash(self, msg: str) -> None:
        self._msg = msg
        self._msg_until = pygame.time.get_ticks() + 4000

    def _next_course(self) -> None:
        if not self.truth:
            return
        cs = self.truth["courses"]
        if not cs:
            return
        cur = self.truth["course_path"]
        i = cs.index(cur) + 1 if cur in cs else 0
        self.ch.send(type="set_course", path=cs[i % len(cs)])

    # ── 描画 ──

    def _draw(self) -> None:
        self.screen.fill(BG)
        stale = pygame.time.get_ticks() - self.last_rx_ms > STALE_MS
        if self.truth is None:
            self._waiting()
        else:
            self._draw_map(stale)
            self._draw_panel(stale)
        pygame.display.flip()

    def _waiting(self) -> None:
        cx, cy = self.view.centerx, self.view.centery - 40
        pygame.draw.rect(self.screen, PANEL, pygame.Rect(self.w - PANEL_W, 0, PANEL_W, self.h))
        pygame.draw.line(self.screen, LINE, (self.w - PANEL_W, 0), (self.w - PANEL_W, self.h))
        t = self.ui.hd.render("シミュレータからの真値を待っています", True, FG)
        self.screen.blit(t, (cx - t.get_width() // 2, cy))
        for i, s in enumerate(("別のターミナルで:", "", ".venv/bin/python -m sim.run")):
            f = self.ui.num if s.startswith(".venv") else self.ui.lbl
            g = f.render(s, True, ACCENT if s.startswith(".venv") else DIM)
            self.screen.blit(g, (cx - g.get_width() // 2, cy + 44 + i * 22))

    # -- コース --

    def _course(self, path: str) -> Course:
        """**更新時刻もキーに含める。** パスだけで覚えると、エディタで直した
        コースを開き直したときに古い地図を出し続ける。シム側（`SimChannel._apply`）は
        毎回読み直すので、**車は新しい形を走っているのに俯瞰ビューだけ古い**という、
        一番たちの悪いずれ方をする。
        """
        try:
            key = (path, Path(path).stat().st_mtime_ns)
        except OSError:
            key = (path, 0)
        if key not in self._course_cache:
            self._course_cache = {key: Course.load(path)}   # 1枚あれば足りる
            U.clear_map_cache()
        return self._course_cache[key]

    def _draw_map(self, stale: bool) -> None:
        t = self.truth
        try:
            course = self._course(t["course_path"])
        except Exception:
            return
        cw, chh = course.size_m
        scale = min((self.view.w - PAD * 2) / cw, (self.view.h - PAD * 2 - 24) / chh)
        dw, dh = int(cw * scale), int(chh * scale)
        ox = self.view.x + (self.view.w - dw) // 2
        oy = self.view.y + (self.view.h - dh) // 2

        self.screen.blit(U.map_surface(course, dw, dh), (ox, oy))
        pygame.draw.rect(self.screen, LINE, pygame.Rect(ox - 1, oy - 1, dw + 2, dh + 2), 1)

        def to_px(x, y):
            return (ox + (x - course.origin[0]) * scale,
                    oy + dh - (y - course.origin[1]) * scale)

        board = pygame.Rect(ox, oy, dw, dh)
        ov = self._overlay
        ov.fill((0, 0, 0, 0))
        off = (-self.view.x, -self.view.y)
        # **盤面の外にはみ出させない。** はみ出すとどこまでがコースか読めなくなる
        ov.set_clip(board.move(*off))

        # 軌跡。古いほど薄くする（一様な線だとどちらに走っているか読めない）
        pts = [to_px(*p) for p in self.trail]
        if len(pts) > 2:
            half = len(pts) // 2
            pygame.draw.aalines(ov, (*TRAIL, 70), False,
                                [(x + off[0], y + off[1]) for x, y in pts[:half + 1]])
            pygame.draw.aalines(ov, (*TRAIL, 190), False,
                                [(x + off[0], y + off[1]) for x, y in pts[half:]])

        # LiDAR の観測点。**ノイズと欠損が入った後**のもので、真値ではない。
        # 車体（真値）との差が、そのままセンサの誤差として見える
        yaw = t["yaw"]
        mx, my = t.get("lidar_mount", (0.12, 0.0))
        lx = t["x"] + mx * math.cos(yaw) - my * math.sin(yaw)
        ly = t["y"] + mx * math.sin(yaw) + my * math.cos(yaw)
        dot, r = self._dot, self._dot.get_width() // 2
        for deg, mm in enumerate(t["scan_mm"]):
            if mm <= 0:
                continue
            a = yaw + math.radians(deg)
            d = mm / 1000.0
            px, py = to_px(lx + d * math.cos(a), ly + d * math.sin(a))
            ov.blit(dot, (px + off[0] - r, py + off[1] - r))
        ov.set_clip(None)
        self.screen.blit(ov, self.view.topleft)

        self.screen.set_clip(board)
        self._draw_vehicle(t, to_px, scale)
        self.screen.set_clip(None)

        self._scalebar(ox, oy + dh, scale)
        name = self.ui.tracked(course.name.upper(), DIM, 11, 3)
        self.screen.blit(name, (ox + 2, oy - 18))
        if stale:
            self.ui.pill(self.screen, "シム未接続", BAD, ox + dw - 100, oy + 10)

    def _draw_vehicle(self, t: dict, to_px, scale: float) -> None:
        yaw = t["yaw"]
        c, s = math.cos(yaw), math.sin(yaw)
        pts = [to_px(t["x"] + px * c - py * s, t["y"] + px * s + py * c)
               for px, py in t["footprint"]]
        ipts = [(int(x), int(y)) for x, y in pts]
        col = BAD if t["collided"] else (WARN if t["armed"] else OK)
        pygame.gfxdraw.filled_polygon(self.screen, ipts, (*col, 55))
        pygame.gfxdraw.aapolygon(self.screen, ipts, (*col, 235))

        # 車輪。**前輪は実舵角、後輪は車体向き。** 1本だけ描くと車体を貫く槍に見えるので、
        # 左右2本ずつ出す。指令ではなく「実際に切れている角」を見せるのが要点
        half = max(abs(py) for _, py in t["footprint"]) * 0.85
        wl = 0.055 * scale
        for ax, ang, colw in ((0.25, yaw + t["steer_actual"], ACCENT), (0.0, yaw, FAINT)):
            for side in (+1, -1):
                wx, wy = to_px(t["x"] + ax * c - side * half * s,
                               t["y"] + ax * s + side * half * c)
                dx, dy = wl * math.cos(ang), -wl * math.sin(ang)
                pygame.draw.aaline(self.screen, colw, (wx - dx, wy - dy), (wx + dx, wy + dy))

    def _scalebar(self, x: int, y: int, scale: float) -> None:
        w = int(scale)                      # 1 m
        y += 12
        pygame.draw.line(self.screen, FAINT, (x, y), (x + w, y))
        for dx in (0, w):
            pygame.draw.line(self.screen, FAINT, (x + dx, y - 3), (x + dx, y + 3))
        g = self.ui.sm.render("1 m", True, FAINT)
        self.screen.blit(g, (x + w + 8, y - g.get_height() // 2))

    # -- パネル --

    def _draw_panel(self, stale: bool) -> None:
        x0 = self.w - PANEL_W
        pygame.draw.rect(self.screen, PANEL, pygame.Rect(x0, 0, PANEL_W, self.h))
        pygame.draw.line(self.screen, LINE, (x0, 0), (x0, self.h))
        x = x0 + PAD
        inner = PANEL_W - PAD * 2

        self.screen.blit(self.ui.hd.render("SURGE Mk.2", True, FG), (x, PAD))
        self.screen.blit(self.ui.tracked("SETTINGS" if self.settings else "SIMULATOR",
                                       ACCENT, 10, 3), (x, PAD + 21))
        y = PAD + 52
        if self.settings:
            self._panel_settings(x, y, inner)
        else:
            self._panel_live(x, y, inner, stale)
        self._hints(x, inner)

    def _panel_live(self, x: int, y: int, inner: int, stale: bool) -> None:
        t = self.truth
        self.rows = []

        mode = ("DISARM", "MANUAL", "AUTO")[t["mode"] if t["mode"] in (0, 1, 2) else 0]
        chips = [(mode, ACCENT if t["mode"] else FAINT),
                 ("ARMED" if t["armed"] else "DISARM", WARN if t["armed"] else FAINT)]
        if t["estop"]:
            chips.append(("E-STOP", BAD))
        if t["uart_timeout"]:
            chips.append(("COMMAND 途絶", BAD))
        if t["auto_stop"]:
            chips.append(("自動停止 作動", BAD))
        if t["collided"]:
            chips.append(("衝突中", BAD))
        if stale:
            chips.append(("シム未接続", BAD))
        cx = x
        for label, col in chips:
            w = self.ui.pill_f.size(label)[0] + 20
            if cx + w > x + inner:          # 折り返し
                cx, y = x, y + 28
            cx = self.ui.pill(self.screen, label, col, cx, y) + 6
        y += 42

        y = self.ui.section(self.screen, "走行", x, y, inner)
        g = self.ui.big.render(f"{t['speed']:+.2f}", True, FG)
        self.screen.blit(g, (x, y))
        self.screen.blit(self.ui.sm.render("m/s", True, DIM),
                         (x + g.get_width() + 6, y + g.get_height() - 15))
        y += g.get_height() + 8
        y = self.ui.kv(self.screen, x, y, inner, "舵角 実", f"{math.degrees(t['steer_actual']):+6.1f}°")
        y = self.ui.kv(self.screen, x, y, inner, "舵角 指令", f"{math.degrees(t['steer_cmd']):+6.1f}°", DIM)
        y = self.ui.kv(self.screen, x, y, inner, "ヨーレート", f"{t['yaw_rate']:+5.2f} rad/s")

        y = self.ui.section(self.screen, "姿勢", x, y + 10, inner)
        y = self.ui.kv(self.screen, x, y, inner, "位置 x, y", f"{t['x']:6.2f}, {t['y']:5.2f}")
        y = self.ui.kv(self.screen, x, y, inner, "方位", f"{math.degrees(t['yaw']):+6.0f}°")

        y = self.ui.section(self.screen, "センサ・衝突", x, y + 10, inner)
        y = self.ui.kv(self.screen, x, y, inner, "超音波 前", _m(t["us_front"]))
        y = self.ui.kv(self.screen, x, y, inner, "超音波 後", _m(t["us_rear"]))
        # 有効点数。**ノイズや欠損をいじったときの効きが数字で見える**
        scan = t["scan_mm"]
        n = sum(1 for m in scan if m > 0)
        y = self.ui.kv(self.screen, x, y, inner, "点群 有効", f"{n:3d} / {len(scan) or 360}",
                     FG if n > 300 else WARN)
        # **単位の「回」は等幅フォント（Menlo）に無い。** 数字だけ出して単位は左のラベルへ
        y = self.ui.kv(self.screen, x, y, inner, "衝突 [回]", f"{t['collisions']}",
                     BAD if t["collisions"] else FG)

        y = self.ui.section(self.screen, "コース", x, y + 10, inner)
        self.screen.blit(self.ui.lbl.render(t["course"], True, FG), (x, y))
        y += 22
        try:
            c = self._course(t["course_path"])
            y = self.ui.kv(self.screen, x, y, inner, "寸法", f"{c.size_m[0]:4.1f} x {c.size_m[1]:4.1f} m", DIM)
            if c.width is not None:
                self.ui.kv(self.screen, x, y, inner, "道幅", f"{c.width:4.2f} m", DIM)
        except Exception:
            pass

    def _panel_settings(self, x: int, y: int, inner: int) -> None:
        t = self.truth
        self.rows = []
        y = self.ui.section(self.screen, "COURSE", x, y, inner)
        for path in t["courses"]:
            cur = path == t["course_path"]
            r = Row("course", path, Path(path).stem, pygame.Rect(x - 8, y - 3, inner + 16, 24))
            self._add(r)
            if cur:
                pygame.gfxdraw.filled_circle(self.screen, x + 4, y + 8, 3, ACCENT)
            self.screen.blit(self.ui.lbl.render(r.label, True, FG if cur else DIM), (x + 16, y))
            y += 25

        group = ""
        for tun in TUNABLES:
            if tun.group != group:
                group = tun.group
                y = self.ui.section(self.screen, group, x, y + 10, inner)
            r = Row("param", tun.key, tun.label, pygame.Rect(x - 8, y - 3, inner + 16, 26))
            self._add(r)
            val = t["params"].get(tun.key, 0.0)
            self.screen.blit(self.ui.lbl.render(tun.label, True, DIM), (x, y))
            g = self.ui.num.render(tun.fmt.format(val), True, FG)
            self.screen.blit(g, (x + inner - g.get_width(), y))
            # 値が範囲のどこにあるかを 2px の帯で出す。数字だけだと相場が分からない
            frac = (val - tun.lo) / (tun.hi - tun.lo) if tun.hi > tun.lo else 0.0
            by = y + 20
            pygame.draw.line(self.screen, HAIR, (x, by), (x + inner, by), 2)
            if frac > 0:
                pygame.draw.line(self.screen, ACCENT, (x, by),
                                 (x + max(1, int(inner * min(1.0, frac))), by), 2)
            y += 28
        if self.sel >= len(self.rows):
            self.sel = 0

    def _hints(self, x: int, inner: int) -> None:
        """操作ボタンと、キーだけの操作の説明。**下端に貼り付ける。**"""
        estop = (self.truth or {}).get("estop", False)
        acts = ([("設定を閉じる  P", lambda: setattr(self, "settings", False), None),
                 ("既定値に戻す  D", self._reset_params, None),
                 ("次のコース  N", self._next_course, None),
                 ("コースエディタ  M", self._open_editor, None)]
                if self.settings else
                [("シムの設定  P", lambda: setattr(self, "settings", True), None),
                 ("次のコース  N", self._next_course, None),
                 ("姿勢リセット  R", self._reset_pose, None),
                 ("軌跡消去  T", self.trail.clear, None),
                 ("コースエディタ  M", self._open_editor, None),
                 ("E-STOP 解除  E" if estop else "E-STOP  E", self._toggle_estop, BAD)])

        hint = ("↑↓ 選択   ←→ 増減 (Shift 10x)   Q 終了" if self.settings
                else "Q 終了")
        show_msg = self._msg and pygame.time.get_ticks() < self._msg_until
        rows = (len(acts) + 1) // 2
        h = rows * 32 + 18 + (22 if show_msg else 0)
        top = self.h - PAD - h
        pygame.draw.line(self.screen, HAIR, (x, top - 12), (x + inner, top - 12))

        self.acts = []
        bw = (inner - 8) // 2
        for i, (label, fn, col) in enumerate(acts):
            r = pygame.Rect(x + (i % 2) * (bw + 8), top + (i // 2) * 32, bw, 26)
            a = Act(r, fn, label, col)
            self.ui.button(self.screen, r, label,
                           hover=(len(self.acts) == self.hover_act), color=col)
            self.acts.append(a)
        self.screen.blit(self.ui.sm.render(hint, True, FAINT), (x, top + rows * 32 + 2))
        if show_msg:
            self.screen.blit(self.ui.sm.render(self._msg, True, ACCENT),
                             (x, self.h - PAD - 14))

    def _add(self, r: Row) -> None:
        """行を登録し、選択中なら背景を敷く。"""
        i = len(self.rows)
        if i == self.sel:
            pygame.draw.rect(self.screen, PANEL2, r.rect, border_radius=5)
            pygame.draw.rect(self.screen, ACCENT,
                             pygame.Rect(r.rect.x, r.rect.y + 3, 2, r.rect.h - 6))
        elif i == self.hover:
            pygame.draw.rect(self.screen, HAIR, r.rect, border_radius=5)
        self.rows.append(r)


# ── 小物 ────────────────────────────────────────────────────────────────

def _m(v: float) -> str:
    """超音波は 0 が「無効」。**0.00 m と出すと「目の前に壁」に見える。**"""
    return f"{v:5.2f} m" if v > 0 else "   — "


def main() -> int:
    try:
        v = Viewer()
    except OSError as e:
        # **一番ありがちなのは sim.gui の二重起動。** 生の OSError を出すと
        # 「アドレスが使用中」としか分からず、何が使っているのか見えない
        from sim.channel import TRUTH_PORT
        print(f"!! シミュレータGUI を開けません: {e}\n"
              f"   UDP {TRUTH_PORT} を別の sim.gui が使っている可能性があります。\n"
              f"   lsof -nP -iUDP:{TRUTH_PORT}   で確認して落としてください",
              file=sys.stderr)
        return 2
    v.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
