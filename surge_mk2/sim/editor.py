"""コースエディタ — 直線と円弧をクリックでつないでコースを作る。

    python -m sim.editor                 # 新規
    python -m sim.editor circuit         # 既存を開いて編集

**センターライン方式（`sim/track.py`）の区間の並びを組み立てる UI。** 中身は
「配列に要素を足す」だけなので、保存したファイルは手で書いたものと完全に同じ。
エディタで作って手で微調整する、逆もできる。

シミュレータとは独立に動く（走らせながら別ウィンドウで編集してよい）。保存したら
シミュレータGUI の `N` か設定ページから選び直せば読み込まれる。

## 壁ではなく中心線を置く

パレットにあるのは「壁」ではなく「進む区間」。左右の壁は道幅から自動生成されるので
**常に平行**で、継ぎ目に隙間ができない。スタート姿勢も中心線の始点になるため
**壁に埋まりようがない**（PNG 方式で実際に踏んだ罠）。

## プレビューは本番と同じ描画を通す

`sim/ui.py` の `map_surface` を俯瞰ビューと共有している。**エディタで見た物と
走らせた物が違って見えたら、エディタを信用できなくなる。**
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import pygame

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim import ui as U                        # noqa: E402
from sim.course import Course, DEFAULT_COURSE_DIR  # noqa: E402
from sim.track import build                    # noqa: E402

INIT_W, INIT_H = 1400, 900
MIN_W, MIN_H = 900, 620
PANEL_W = 320

#: パレット。**ここを書き換えれば選択肢が変わる**（コードの他の場所は触らなくてよい）
STRAIGHTS = (0.5, 1.0, 2.0, 3.0)          # [m]
RADII = (0.5, 1.0, 1.5, 2.0)              # [m]
#: **どれも 360 を割り切る角度にしてある。** 同じ角度を繰り返せば必ず閉じる
ANGLES = (30, 45, 60, 90, 180)            # [deg]

WIDTH_STEP = 0.05
DEFAULT_WIDTH = 1.0
#: 中心線の始点。**固定でよい**（ラスタライザが外接矩形から原点を決めるため）
ORIGIN = (1.0, 1.0, 0.0)

#: 閉ループとみなす許容。`sim/track.py` の警告と同じ基準
LOOP_POS_TOL = 0.10       # [m]
LOOP_ANG_TOL = 5.0        # [deg]


class Btn:
    __slots__ = ("rect", "kind", "value", "label")

    def __init__(self, rect, kind, value, label):
        self.rect, self.kind, self.value, self.label = rect, kind, value, label


class Editor:
    def __init__(self, open_name: str | None = None) -> None:
        pygame.init()
        pygame.display.set_caption("SURGE Mk.2 コースエディタ")
        self.screen = pygame.display.set_mode((INIT_W, INIT_H), pygame.RESIZABLE)
        self.clock = pygame.time.Clock()
        self.ui = U.Ui()

        self.segments: list[list] = []
        self.width = DEFAULT_WIDTH
        self.name = "new_course"
        self.radius = RADII[1]
        self.angle = ANGLES[2]
        self.buttons: list[Btn] = []
        self.hover: Btn | None = None
        self.message = ""
        self.msg_until = 0
        self.naming = False
        self._clear_armed = 0
        self._course: Course | None = None
        self._dirty = True
        self._resize(INIT_W, INIT_H)
        if open_name:
            self.open(open_name)

    def _resize(self, w: int, h: int) -> None:
        self.w, self.h = max(MIN_W, w), max(MIN_H, h)
        self.view = pygame.Rect(0, 0, self.w - PANEL_W, self.h)
        U.clear_map_cache()

    # ── コース ──

    def meta(self, loop: bool = False) -> dict:
        """:param loop: **呼び出し側が渡す。** ここで `closed()` を呼ぶと
            `meta → closed → course → meta` の無限再帰になる（実際にやった）
        """
        return {"name": self.name, "width": self.width, "resolution": 0.02,
                "origin": list(ORIGIN), "loop": loop,
                "path": [list(s) for s in self.segments]}

    def course(self) -> Course | None:
        """区間から Course を組み立てる。**変わったときだけ作り直す。**"""
        if not self.segments:
            return None
        if self._dirty:
            t = build(self.meta(loop=False))           # 警告はここでは出さない
            self._course = Course(name=self.name, path=Path(f"<editor>/{self.name}"),
                                  resolution=t["resolution"], origin=t["origin"],
                                  start=t["start"], grid=t["grid"],
                                  centerline=t["centerline"], width=t["width"])
            self._dirty = False
        return self._course

    def turn_total(self) -> float:
        """これまでの総回頭 [deg]。**360 の倍数でなければ絶対に閉じない。**

        45°や30°で閉ループを作るのが難しいのは、この数字が見えていないから。
        「あと何度回ればよいか」が分かれば、必要な区間数はすぐ数えられる。
        """
        return sum(s[2] for s in self.segments if s[0] == "arc")

    def gap_frame(self) -> tuple[float, float, float] | None:
        """終点から見た始点の (前方[m], 左[m], 回頭差[deg])。

        世界座標の差をそのまま出しても「どちらに直せばよいか」が分からない。
        **進行方向を基準にすると「あと 1.2m 前」「0.3m 左にずれている」と読める。**
        """
        c = self.course()
        if c is None or len(c.centerline) < 2:
            return None
        x0, y0, th0 = c.centerline[0]
        x1, y1, th1 = c.centerline[-1]
        # **必ず素の float にする。** `centerline` は numpy 配列なので、そのまま
        # 区間に入れると `np.float64` が混じり、JSON 保存が TypeError で落ちる
        dx, dy = float(x0 - x1), float(y0 - y1)
        th0, th1 = float(th0), float(th1)
        fwd = dx * math.cos(th1) + dy * math.sin(th1)
        left = -dx * math.sin(th1) + dy * math.cos(th1)
        dth = math.degrees((th0 - th1 + math.pi) % (2 * math.pi) - math.pi)
        return fwd, left, dth

    def close_length(self) -> float | None:
        """直線 1 本で閉じられるならその長さ。無理なら None。"""
        g = self.gap_frame()
        if g is None:
            return None
        fwd, left, dth = g
        if abs(dth) <= LOOP_ANG_TOL and abs(left) <= LOOP_POS_TOL and fwd > 0.05:
            return round(fwd, 2)
        return None

    def close_straight(self) -> None:
        d = self.close_length()
        if d is None:
            g = self.gap_frame()
            if g is None:
                self.say("区間がありません")
            else:
                fwd, left, dth = g
                self.say(f"直線だけでは閉じません（回頭差 {dth:.0f}° / 横ずれ {left:+.2f}m）")
            return
        self.add(["straight", d])
        self.say(f"直線 {d} m を足して閉じました")

    def closed(self) -> tuple[bool, float, float]:
        """(閉じているか, 位置のずれ[m], 向きのずれ[deg])。"""
        c = self.course()
        if c is None or len(c.centerline) < 2:
            return False, 0.0, 0.0
        a, b = c.centerline[0], c.centerline[-1]
        gap = math.hypot(b[0] - a[0], b[1] - a[1])
        dth = abs(math.degrees((b[2] - a[2] + math.pi) % (2 * math.pi) - math.pi))
        return (gap <= LOOP_POS_TOL and dth <= LOOP_ANG_TOL), gap, dth

    def add(self, seg: list) -> None:
        self.segments.append(seg)
        self._dirty = True

    def undo(self) -> None:
        if self.segments:
            self.segments.pop()
            self._dirty = True

    def clear(self, confirm: bool = True) -> None:
        """全消去。**取り消せないので2度押しを要求する。**

        直前を消す `Backspace` と違い、これは1キーで作業が全部消える。
        パレットの隣に置くボタンでもあるので、誤爆すると痛い。
        """
        if not self.segments:
            return
        now = pygame.time.get_ticks()
        if confirm and now > self._clear_armed:
            self._clear_armed = now + 2500
            self.say("もう一度押すと全消去します")
            return
        self._clear_armed = 0
        self.segments.clear()
        self._dirty = True
        self.say("全消去しました")

    def set_width(self, d: float) -> None:
        self.width = max(0.3, min(3.0, round(self.width + d, 3)))
        self._dirty = True

    # ── 読み書き ──

    def open(self, name: str) -> None:
        p = Path(name)
        if not p.exists():
            p = DEFAULT_COURSE_DIR / f"{Path(name).stem}.json"
        try:
            m = json.loads(p.read_text(encoding="utf-8"))
            if "path" not in m:
                self.say(f"{p.name} は PNG 方式なので編集できません")
                return
            self.segments = [list(s) for s in m["path"]]
            self.width = float(m.get("width", DEFAULT_WIDTH))
            self.name = p.stem
            self._dirty = True
            self.say(f"{p.name} を開きました（{len(self.segments)} 区間）")
        except Exception as e:
            self.say(f"開けません: {e}")

    def save(self) -> None:
        if not self.segments:
            self.say("区間がありません")
            return
        loop, gap, dth = self.closed()
        p = DEFAULT_COURSE_DIR / f"{self.name}.json"
        m = self.meta(loop=loop)
        m["note"] = "sim.editor で作成"
        p.write_text(json.dumps(m, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        self.say(f"保存: {p.name}"
                 + ("（閉ループ）" if loop else f"（開いたまま: ずれ {gap:.2f}m）"))

    def say(self, msg: str) -> None:
        self.message = msg
        self.msg_until = pygame.time.get_ticks() + 4000

    # ── ループ ──

    def run(self) -> None:
        while self._events():
            self._draw()
            self.clock.tick(60)
        pygame.quit()

    def _events(self) -> bool:
        self.hover = None
        mouse = pygame.mouse.get_pos()
        for b in self.buttons:
            if b.rect.collidepoint(mouse):
                self.hover = b
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                return False
            if e.type == pygame.VIDEORESIZE:
                self.screen = pygame.display.set_mode(
                    (max(MIN_W, e.w), max(MIN_H, e.h)), pygame.RESIZABLE)
                self._resize(e.w, e.h)
            elif e.type == pygame.MOUSEBUTTONDOWN and e.button == 1 and self.hover:
                self._press(self.hover)
            elif e.type == pygame.KEYDOWN:
                if self.naming:
                    if not self._naming_key(e):
                        return True
                elif not self._key(e):
                    return False
        return True

    def _press(self, b: Btn) -> None:
        if b.kind == "straight":
            self.add(["straight", b.value])
        elif b.kind == "radius":
            self.radius = b.value
        elif b.kind == "angle":
            self.angle = b.value
        elif b.kind == "turn":
            self.add(["arc", self.radius, self.angle * b.value])
        elif b.kind == "close":
            self.close_straight()
        elif b.kind == "act":
            b.value()

    def _key(self, e) -> bool:
        k = e.key
        if k in (pygame.K_ESCAPE, pygame.K_q):
            return False
        if k == pygame.K_BACKSPACE:
            self.undo()
        elif k == pygame.K_c:
            self.clear()
        elif k == pygame.K_s:
            self.naming = True
        elif k == pygame.K_o:
            self._open_next()
        elif k == pygame.K_RETURN:
            self.close_straight()
        elif k == pygame.K_LEFT:
            self.add(["arc", self.radius, self.angle])
        elif k == pygame.K_RIGHT:
            self.add(["arc", self.radius, -self.angle])
        elif k == pygame.K_UP:
            self.add(["straight", STRAIGHTS[1]])
        elif k == pygame.K_w:
            self.set_width(WIDTH_STEP if (e.mod & pygame.KMOD_SHIFT) else -WIDTH_STEP)
        return True

    def _naming_key(self, e) -> bool:
        if e.key == pygame.K_RETURN:
            self.naming = False
            self.save()
        elif e.key == pygame.K_ESCAPE:
            self.naming = False
        elif e.key == pygame.K_BACKSPACE:
            self.name = self.name[:-1]
        elif e.unicode and e.unicode.isprintable():
            # **半角英数だけ。** ファイル名になるうえ、pygame は IME を扱えない
            ch = e.unicode
            if ch.isascii() and (ch.isalnum() or ch in "-_"):
                self.name = (self.name + ch)[:32]
        return True

    def _open_next(self) -> None:
        files = sorted(p for p in DEFAULT_COURSE_DIR.glob("*.json")
                       if "path" in json.loads(p.read_text(encoding="utf-8")))
        if not files:
            self.say("編集できるコースがありません")
            return
        stems = [p.stem for p in files]
        i = stems.index(self.name) + 1 if self.name in stems else 0
        self.open(str(files[i % len(files)]))

    # ── 描画 ──

    def _draw(self) -> None:
        self.screen.fill(U.BG)
        self._draw_preview()
        self._draw_panel()
        if self.naming:
            self._draw_naming()
        pygame.display.flip()

    def _draw_preview(self) -> None:
        c = self.course()
        if c is None:
            g = self.ui.hd.render("右のパレットから区間を選んでください", True, U.DIM)
            self.screen.blit(g, (self.view.centerx - g.get_width() // 2,
                                 self.view.centery - 10))
            return
        cw, chh = c.size_m
        scale = min((self.view.w - U.PAD * 2) / cw, (self.view.h - U.PAD * 2 - 24) / chh)
        dw, dh = int(cw * scale), int(chh * scale)
        ox = self.view.x + (self.view.w - dw) // 2
        oy = self.view.y + (self.view.h - dh) // 2

        key = f"edit:{self.width}:{self.segments}"
        self.screen.blit(U.map_surface(c, dw, dh, key=key), (ox, oy))
        pygame.draw.rect(self.screen, U.LINE, pygame.Rect(ox - 1, oy - 1, dw + 2, dh + 2), 1)

        def to_px(x, y):
            return (ox + (x - c.origin[0]) * scale, oy + dh - (y - c.origin[1]) * scale)

        # 始点（緑）と終点（琥珀）。**向きが見えないとどこから走り出すか分からない**
        a, b = c.centerline[0], c.centerline[-1]
        loop, gap, _ = self.closed()
        U.arrow(self.screen, *to_px(a[0], a[1]), a[2], max(14, 0.32 * scale), U.OK)
        U.arrow(self.screen, *to_px(b[0], b[1]), b[2], max(14, 0.32 * scale),
                U.OK if loop else U.ACCENT)
        if not loop and len(c.centerline) > 2:
            # 閉じるまでの距離を線で見せる。数字だけだとどっちに戻ればよいか分からない
            pygame.draw.aaline(self.screen, U.FAINT, to_px(b[0], b[1]), to_px(a[0], a[1]))

        # 1m のスケールバー
        y = oy + dh + 12
        pygame.draw.line(self.screen, U.FAINT, (ox, y), (ox + int(scale), y))
        for dx in (0, int(scale)):
            pygame.draw.line(self.screen, U.FAINT, (ox + dx, y - 3), (ox + dx, y + 3))
        g = self.ui.sm.render("1 m", True, U.FAINT)
        self.screen.blit(g, (ox + int(scale) + 8, y - g.get_height() // 2))

    def _draw_panel(self) -> None:
        x0 = self.w - PANEL_W
        pygame.draw.rect(self.screen, U.PANEL, pygame.Rect(x0, 0, PANEL_W, self.h))
        pygame.draw.line(self.screen, U.LINE, (x0, 0), (x0, self.h))
        x = x0 + U.PAD
        inner = PANEL_W - U.PAD * 2
        self.buttons = []

        self.screen.blit(self.ui.hd.render("SURGE Mk.2", True, U.FG), (x, U.PAD))
        self.screen.blit(self.ui.tracked("COURSE EDITOR", U.ACCENT, 10, 3), (x, U.PAD + 21))
        y = U.PAD + 52

        y = self.ui.section(self.screen, "直線 [m]", x, y, inner)
        y = self._row(x, y, inner, STRAIGHTS, "straight",
                      lambda v: f"{v:g}", sel=None)

        y = self.ui.section(self.screen, "カーブ 半径 [m]", x, y + 8, inner)
        y = self._row(x, y, inner, RADII, "radius", lambda v: f"{v:g}", sel=self.radius)

        y = self.ui.section(self.screen, "カーブ 角度 [deg]", x, y + 8, inner)
        y = self._row(x, y, inner, ANGLES, "angle", lambda v: f"{v}", sel=self.angle)

        bw = (inner - 10) // 2
        for i, (label, sign) in enumerate((("← 左に曲げる", 1), ("右に曲げる →", -1))):
            r = pygame.Rect(x + i * (bw + 10), y + 6, bw, 30)
            b = Btn(r, "turn", sign, label)
            self.buttons.append(b)
            self.ui.button(self.screen, r, label, hover=(self.hover is b))
        y += 48

        y = self.ui.section(self.screen, "コース", x, y + 8, inner)
        y = self.ui.kv(self.screen, x, y, inner, "道幅 [m]  (W / Shift+W)",
                       f"{self.width:4.2f}")
        y = self.ui.kv(self.screen, x, y, inner, "区間 [個]", f"{len(self.segments)}")
        c = self.course()
        if c is not None:
            y = self.ui.kv(self.screen, x, y, inner, "寸法 [m]",
                           f"{c.size_m[0]:4.1f} x {c.size_m[1]:4.1f}", U.DIM)
            length = sum(s[1] if s[0] == "straight" else abs(math.radians(s[2])) * s[1]
                         for s in self.segments)
            y = self.ui.kv(self.screen, x, y, inner, "全長 [m]", f"{length:5.1f}", U.DIM)
        loop, gap, dth = self.closed()
        # **等幅フォントに日本語が無い**ので mono=False。閉じていないときは
        # 「あとどれだけ戻ればよいか」を出す（数字だけ見せても直しようがない）
        y = self.ui.kv(self.screen, x, y, inner, "閉ループ",
                       "閉じた" if loop else f"ずれ {gap:.2f} m / {dth:.0f}°",
                       U.OK if loop else U.WARN, mono=False)

        # **閉ループを作る鍵はこの2行。** 総回頭が 360 の倍数でなければ絶対に閉じないし、
        # 始点までの差を進行方向基準で見れば「あと何を足せばよいか」がそのまま読める
        turn = self.turn_total()
        rest = round((-turn) % 360)
        y = self.ui.kv(self.screen, x, y, inner, "総回頭 [deg]",
                       f"{turn:+7.0f}" + ("" if rest == 0 else f"  (あと {rest})"),
                       U.OK if rest == 0 and self.segments else U.WARN, mono=False)
        g = self.gap_frame()
        if g is not None:
            fwd, left, _ = g
            y = self.ui.kv(self.screen, x, y, inner, "始点まで 前 / 左 [m]",
                           f"{fwd:+5.2f} /{left:+5.2f}", U.DIM)

        d = self.close_length()
        r = pygame.Rect(x, y + 4, inner, 28)
        b = Btn(r, "close", 0, "直線で閉じる")
        self.buttons.append(b)
        label = ("すでに閉じています" if loop else
                 f"直線 {d:g} m で閉じる" if d else "直線では閉じられない")
        self.ui.button(self.screen, r, label, on=bool(d), hover=(self.hover is b))
        y += 38

        y = self.ui.section(self.screen, "区間の並び", x, y + 8, inner)
        # **最後の数個だけ出す。** 全部出すとパネルが溢れるし、直すのは末尾だから
        tail = self.segments[-8:]
        if len(self.segments) > len(tail):
            self.screen.blit(self.ui.sm.render(f"… 他 {len(self.segments)-len(tail)} 個",
                                               True, U.FAINT), (x, y))
            y += 16
        for s in tail:
            txt = (f"直線 {s[1]:g} m" if s[0] == "straight"
                   else f"R{s[1]:g} {'左' if s[2] > 0 else '右'} {abs(s[2]):g}°")
            self.screen.blit(self.ui.lbl.render(txt, True, U.DIM), (x, y))
            y += 18

        # 操作ボタン。**ショートカットを覚えなくても全機能に届くようにする。**
        # ラベルにキーを併記して、使ううちに覚えられるように
        acts = [("直前を削除  ⌫", self.undo, None),
                ("全消去  C", self.clear, U.BAD),
                ("既存を開く  O", self._open_next, None),
                ("保存  S", lambda: setattr(self, "naming", True), U.ACCENT)]
        hint = "クリック / ← → ↑ でも区間を足せる    Q 終了"
        show_msg = self.message and pygame.time.get_ticks() < self.msg_until
        rows = (len(acts) + 1) // 2
        h = rows * 32 + 18 + (22 if show_msg else 0)
        top = self.h - U.PAD - h
        pygame.draw.line(self.screen, U.HAIR, (x, top - 12), (x + inner, top - 12))
        bw = (inner - 8) // 2
        for i, (label, fn, col) in enumerate(acts):
            r = pygame.Rect(x + (i % 2) * (bw + 8), top + (i // 2) * 32, bw, 26)
            b = Btn(r, "act", fn, label)
            self.buttons.append(b)
            self.ui.button(self.screen, r, label, hover=(self.hover is b), color=col)
        self.screen.blit(self.ui.sm.render(hint, True, U.FAINT), (x, top + rows * 32 + 2))
        if show_msg:
            self.screen.blit(self.ui.sm.render(self.message, True, U.ACCENT),
                             (x, self.h - U.PAD - 14))

    def _row(self, x: int, y: int, inner: int, values, kind: str, fmt, sel) -> int:
        n = len(values)
        bw = (inner - 8 * (n - 1)) // n
        for i, v in enumerate(values):
            r = pygame.Rect(x + i * (bw + 8), y, bw, 28)
            b = Btn(r, kind, v, fmt(v))
            self.buttons.append(b)
            self.ui.button(self.screen, r, fmt(v),
                           on=(sel is not None and v == sel), hover=(self.hover is b))
        return y + 36

    def _draw_naming(self) -> None:
        w, h = 420, 120
        r = pygame.Rect((self.w - w) // 2, (self.h - h) // 2, w, h)
        pygame.draw.rect(self.screen, U.PANEL2, r, border_radius=8)
        pygame.draw.rect(self.screen, U.ACCENT, r, 1, border_radius=8)
        self.screen.blit(self.ui.lbl.render("ファイル名（半角英数）", True, U.DIM),
                         (r.x + 20, r.y + 18))
        g = self.ui.num.render(self.name + "_", True, U.FG)
        self.screen.blit(g, (r.x + 20, r.y + 46))
        self.screen.blit(self.ui.sm.render(
            f"Enter で {DEFAULT_COURSE_DIR.name}/{self.name}.json に保存   Esc で中止",
            True, U.FAINT), (r.x + 20, r.y + 82))


def main() -> int:
    Editor(sys.argv[1] if len(sys.argv) > 1 else None).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
