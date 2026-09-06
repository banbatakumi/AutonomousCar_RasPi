"""コースエディタ — 頂点をクリック/ドラッグでつないでコースを作る（Fusion のスケッチ
のような操作感）。中心線モードと壁モードの2つを切り替えられる。

    python -m sim.editor                 # 新規
    python -m sim.editor circuit         # 既存を開いて編集

## 中心線モード

頂点をクリックで置いていき、結んだ折れ線が中心線のループになる。そこから選んだ
道幅ぶんのコースが自動生成される（壁は常に平行、継ぎ目に隙間ができない）。
保存フォーマットは従来通り（`path`＝直線/円弧のタートル区間列）——中身は
`sim/sketch.py::loop_to_path()` が変換するので、手で書いた既存コースと
完全に同じファイルが出てくる。

## 壁モード

操作は中心線モードと同じだが、置いた頂点のループが**中心線ではなく壁そのもの**
になる。何本でも壁ループを描ける（典型的には外壁+内壁の2本）。中心線は
壁で囲まれた自由空間を細線化して**副産物として自動導出**する
（`sim/wall_track.py`）。壁モードでは中心線も道幅も人間が決めないので、
**スタート地点**（位置+向き）を別途置く必要がある。

## どちらのモードも

- クリック（ほぼ動かさずに離す）＝直線の頂点。ドラッグ＝ドラッグを始めた
  位置までは直線、そこから離した位置まで**接線円弧**（直前の直線の向きに
  必ず接するので、Fusion のスケッチの line/arc ツールと同じ感覚で使える）
- 頂点の近くには常に吸着する（ループを閉じるため）。グリッドへの吸着は
  ON/OFF・間隔を切り替えられる
- 障害物（孤立した円盤）も置ける。直径は選択肢から選ぶ
- プレビューは `sim/ui.py::map_surface` を俯瞰ビューと共有する。**エディタで
  見た物と走らせた物が違って見えたら、エディタを信用できなくなる。**
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
import pygame

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim import track                          # noqa: E402
from sim import ui as U                         # noqa: E402
from sim import wall_track                      # noqa: E402
from sim.course import Course, DEFAULT_COURSE_DIR  # noqa: E402
from sim.sketch import (                        # noqa: E402
    Loop,
    loop_to_path,
    path_to_loop,
    sample_loop,
    snap_to_grid,
    snap_to_vertex,
)

INIT_W, INIT_H = 1400, 900
MIN_W, MIN_H = 900, 620
PANEL_W = 340

#: グリッドスナップの間隔の選択肢 [m]
GRID_SPACINGS = (0.25, 0.5, 1.0)
#: 頂点スナップの半径（スクリーン座標 [px]）。**常時ON**——ループを閉じるための機能
VERTEX_SNAP_PX = 14.0
#: これを超えて動かしたら「クリック」ではなく「ドラッグ（円弧）」とみなす[px]
DRAG_ARC_PX = 6.0

DEFAULT_WIDTH = 1.0
WIDTH_STEP = 0.05
CENTER_RESOLUTION = 0.02

DEFAULT_WALL_THICKNESS = 0.03
WALL_THICKNESS_STEP = 0.01
WALL_RESOLUTION = 0.02
WALL_MARGIN = 0.4

#: 障害物の直径の選択肢 [m]
OBSTACLE_DIAMETERS = (0.1, 0.2, 0.3, 0.5)

#: 何も描いていないときの表示範囲 [m]（原点付近）。長い直線も引けるよう、
#: 頂点ぴったりよりだいぶ広めに取る
_EMPTY_BOUNDS = (-2.0, -2.0, 10.0, 8.0)
_BOUNDS_MARGIN = 0.6

#: マウスホイール1ノッチあたりの拡大率
_ZOOM_STEP = 1.15
#: 拡大率の範囲 [px/m]
_MIN_SCALE = 8.0
_MAX_SCALE = 250.0


def _dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


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

        self.mode = "centerline"                # "centerline" か "wall"
        self.center_loop = Loop()
        self.width = DEFAULT_WIDTH

        self.role = "eval"                      # "train" | "eval" | "both"

        self.wall_loops: list[Loop] = []
        self.wall_current = Loop()
        self.wall_thickness = DEFAULT_WALL_THICKNESS
        self.wall_start: tuple[float, float, float] | None = None
        self.placing_start = False
        self._wall_centerline_preview = None

        self.grid_snap = True
        self.grid_spacing = GRID_SPACINGS[0]        # 0.25m

        #: 障害物（孤立した円盤）。モードに関係なく1つの物として持つ——
        #: `Course.obstacles` そのものがモードを問わない (x,y,半径) の配列なので
        self.obstacles: list[tuple[float, float, float]] = []
        self.obstacle_diameter = OBSTACLE_DIAMETERS[1]  # 0.2m
        self.placing_obstacle = False

        self.name = "new_course"
        self.buttons: list[Btn] = []
        self.hover: Btn | None = None
        self.message = ""
        self.msg_until = 0
        self.naming = False
        self._clear_armed = 0

        self.press_screen: tuple[float, float] | None = None
        self.press_world: tuple[float, float] | None = None
        self.mouse_screen: tuple[float, float] = (0.0, 0.0)

        self._cached_course: Course | None = None
        self._dirty = True

        self.view_center: tuple[float, float] = (0.0, 0.0)
        self.view_scale: float = 60.0

        self._resize(INIT_W, INIT_H)
        self._fit_view()
        if open_name:
            self.open(open_name)

    def _resize(self, w: int, h: int) -> None:
        self.w, self.h = max(MIN_W, w), max(MIN_H, h)
        self.view = pygame.Rect(0, 0, self.w - PANEL_W, self.h)
        U.clear_map_cache()

    # ── 状態 ──

    def _active_loop(self) -> Loop:
        return self.center_loop if self.mode == "centerline" else self.wall_current

    def _all_wall_loops(self) -> list[Loop]:
        return self.wall_loops + ([self.wall_current] if self.wall_current.vertices else [])

    def _mark_dirty(self) -> None:
        self._dirty = True
        self._wall_centerline_preview = None

    def say(self, msg: str) -> None:
        self.message = msg
        self.msg_until = pygame.time.get_ticks() + 4000

    # ── 座標変換 ──
    #
    # ビュー（中心の世界座標 `view_center` + 拡大率 `view_scale` [px/m]）は
    # コースの内容とは独立に持つ。**内容に自動フィットさせ続けると、新規コースを
    # 作り始めた瞬間の既定の表示範囲がとても狭く、長い直線を引きたくても画面の
    # 外に置けない**（毎フレーム今の頂点だけに合わせてズームし直すため、常に
    # 「今描いた分がぎりぎり収まる」範囲にしかならない）。フィットは
    # 新規作成・全消去・既存を開いたときに1回だけ行い、それ以外はスクロールで
    # 自由に拡大縮小できるようにする。

    def _content_bounds(self) -> tuple[float, float, float, float]:
        pts: list[tuple[float, float]] = []
        if self.mode == "centerline":
            pts.extend(self.center_loop.vertices)
        else:
            for loop in self.wall_loops:
                pts.extend(loop.vertices)
            pts.extend(self.wall_current.vertices)
            if self.wall_start is not None:
                pts.append(self.wall_start[:2])
        pts.extend((ox, oy) for ox, oy, _r in self.obstacles)
        if not pts:
            return _EMPTY_BOUNDS
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        return (min(xs) - _BOUNDS_MARGIN, min(ys) - _BOUNDS_MARGIN,
                max(xs) + _BOUNDS_MARGIN, max(ys) + _BOUNDS_MARGIN)

    def _fit_view(self, bounds: tuple[float, float, float, float] | None = None) -> None:
        """内容（または渡した範囲）がちょうど収まるようビューを合わせ直す。"""
        x0, y0, x1, y1 = bounds if bounds is not None else self._content_bounds()
        cw, ch = max(x1 - x0, 0.5), max(y1 - y0, 0.5)
        scale = min((self.view.w - U.PAD * 2) / cw, (self.view.h - U.PAD * 2 - 24) / ch)
        self.view_scale = max(_MIN_SCALE, min(_MAX_SCALE, scale))
        self.view_center = ((x0 + x1) / 2.0, (y0 + y1) / 2.0)

    def to_px(self, x: float, y: float) -> tuple[float, float]:
        cx, cy = self.view_center
        return (self.view.centerx + (x - cx) * self.view_scale,
                self.view.centery - (y - cy) * self.view_scale)

    def to_world(self, px: float, py: float) -> tuple[float, float]:
        cx, cy = self.view_center
        return (cx + (px - self.view.centerx) / self.view_scale,
                cy - (py - self.view.centery) / self.view_scale)

    def _visible_world_bounds(self) -> tuple[float, float, float, float]:
        corners = [self.to_world(self.view.x, self.view.y),
                  self.to_world(self.view.x + self.view.w, self.view.y),
                  self.to_world(self.view.x, self.view.y + self.view.h),
                  self.to_world(self.view.x + self.view.w, self.view.y + self.view.h)]
        xs = [c[0] for c in corners]
        ys = [c[1] for c in corners]
        return (min(xs), min(ys), max(xs), max(ys))

    def _zoom(self, notches: float, anchor_screen: tuple[float, float]) -> None:
        """マウスホイールでの拡大縮小。**カーソルの位置を世界座標のまま固定する**
        （カーソル直下の点がズーム後も同じ画面位置に留まる、地図アプリと同じ挙動）。
        """
        if notches == 0:
            return
        ax, ay = self.to_world(*anchor_screen)
        self.view_scale = max(_MIN_SCALE, min(_MAX_SCALE,
                                              self.view_scale * (_ZOOM_STEP ** notches)))
        self.view_center = (
            ax - (anchor_screen[0] - self.view.centerx) / self.view_scale,
            ay - (self.view.centery - anchor_screen[1]) / self.view_scale,
        )

    # ── スナップ ──

    def _snap(self, pt_world: tuple[float, float], loop: Loop) -> tuple[tuple[float, float], int | None]:
        """グリッド→頂点の順にスナップする。頂点スナップが勝てば `pt_world` を
        その頂点の座標で上書きし、添字を返す（`_commit_vertex` が「始点に吸着
        した＝閉じる」を判定するのに使う）。
        """
        if self.grid_snap:
            pt_world = snap_to_grid(pt_world, self.grid_spacing)
        idx = None
        if loop.vertices:
            screen_verts = [self.to_px(*v) for v in loop.vertices]
            idx = snap_to_vertex(self.to_px(*pt_world), screen_verts, VERTEX_SNAP_PX)
            if idx is not None:
                pt_world = loop.vertices[idx]
        return pt_world, idx

    # ── ジオメトリの編集 ──

    def _commit_vertex(self, loop: Loop, is_arc: bool,
                       anchor: tuple[float, float], end: tuple[float, float],
                       close_idx: int | None) -> None:
        """クリック＝直線1本。ドラッグ＝`anchor`（ドラッグ開始位置）までの直線＋
        そこから `end`（離した位置）までの接線円弧、の2本を一度に足す。
        """
        if not loop.vertices:
            loop.add_line(end)
            self._mark_dirty()
            return
        if is_arc:
            loop.add_line(anchor)          # ドラッグ開始位置までは直線
        if close_idx == 0 and len(loop.vertices) >= 2:
            loop.close(as_arc=is_arc)
            self._on_loop_closed(loop)
        else:
            if is_arc:
                loop.add_arc(end)
            else:
                loop.add_line(end)
            if close_idx is not None:
                # 始点以外の既存頂点に重なった。そのまま続けると自己接触した
                # 折れ線になり、壁モードの骨格抽出が「分岐」として弾く事故に
                # つながる（中心線モードでも同じ点を二重に持つ不自然な形に
                # なる）ので、重なった時点でいったん線を確定する
                self._finalize_on_overlap(loop)
        self._mark_dirty()

    def _finalize_on_overlap(self, loop: Loop) -> None:
        """始点以外の既存頂点に重なったら、そこでいったん線を確定する
        （`_commit_vertex` 参照）。壁モードは `finish_current_line()` と同じく
        開いたまま次の壁へ、中心線モードはコースにつき1本しか無いので
        メッセージだけ出す。
        """
        if self.mode == "wall":
            self.wall_loops.append(loop)
            self.wall_current = Loop()
            self.say(f"頂点が重なったのでここで壁を確定しました（開いたまま、計 {len(self.wall_loops)} 本）")
        else:
            self.say("頂点が重なったのでここで線を確定しました")

    def _on_loop_closed(self, loop: Loop) -> None:
        if self.mode == "wall":
            self.wall_loops.append(loop)
            self.wall_current = Loop()
            self.say(f"壁ループを閉じました（計 {len(self.wall_loops)} 本）")
        else:
            self.say("中心線のループを閉じました")

    def undo(self) -> None:
        if self.placing_obstacle and self.obstacles:
            self.obstacles.pop()
            self._mark_dirty()
            return
        if self.mode == "centerline":
            self.center_loop.undo()
        elif self.wall_current.vertices:
            self.wall_current.undo()
        elif self.wall_loops:
            self.wall_current = self.wall_loops.pop()
            self.wall_current.undo()          # 閉じるのを取り消して再編集可能にする
        self._mark_dirty()

    def clear(self, confirm: bool = True, reset_name: bool = False) -> None:
        """全消去。`reset_name=True`（「新規コースを作成」ボタン）なら、ファイル名・
        道幅・壁の厚みも初期値へ戻す——保存済みの名前のまま新しいコースを作って
        上書き保存してしまう事故を防ぐ。
        """
        empty = (not self.center_loop.vertices and not self.wall_loops
                and not self.wall_current.vertices and self.wall_start is None
                and not self.obstacles)
        if empty and not reset_name:
            return
        if not empty:
            now = pygame.time.get_ticks()
            if confirm and now > self._clear_armed:
                self._clear_armed = now + 2500
                self.say("もう一度押すと" + ("新規コースを作成します（未保存の変更は消えます）"
                                           if reset_name else "全消去します"))
                return
        self._clear_armed = 0
        self.center_loop = Loop()
        self.wall_loops = []
        self.wall_current = Loop()
        self.wall_start = None
        self.obstacles = []
        self.placing_obstacle = False
        if reset_name:
            self.name = "new_course"
            self.width = DEFAULT_WIDTH
            self.wall_thickness = DEFAULT_WALL_THICKNESS
            self.role = "eval"
        self._fit_view()
        self._mark_dirty()
        self.say("新規コースを作成しました" if reset_name else "全消去しました")

    def new_course(self) -> None:
        self.clear(confirm=True, reset_name=True)

    def finish_current_line(self) -> None:
        """Enter: 編集中の折れ線を**閉じずにそのまま**確定し、次の壁を描き始め
        られるようにする。壁モード専用——中心線モードはコースにつき中心線が
        1本しか無いので「次の線」という概念が無い。

        閉じたループ（`_on_loop_closed`）と違い、ここで確定した壁は開いたまま
        `wall_loops` に入る。閉じていない壁が骨格化でどう扱われるかは
        `sim/wall_track.py` 参照——コース内の空間を分断しない短い壁（障害物や
        シケインの板）なら、骨格の袋小路の枝として刈り取られるだけで問題なく
        通る。両側の壁をまたいで完全に分断する壁を置いた場合だけ、保存時に
        「壁の形状が複雑すぎます」で弾かれる。
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

    def set_width(self, d: float) -> None:
        self.width = max(0.3, min(3.0, round(self.width + d, 3)))
        self._mark_dirty()

    def set_wall_thickness(self, d: float) -> None:
        self.wall_thickness = max(0.01, min(0.3, round(self.wall_thickness + d, 3)))
        self._mark_dirty()

    def set_grid_spacing(self, v: float) -> None:
        self.grid_spacing = v

    def toggle_grid_snap(self) -> None:
        self.grid_snap = not self.grid_snap

    def set_mode(self, mode: str) -> None:
        if mode != self.mode:
            self.mode = mode
            self._mark_dirty()

    def set_role(self, role: str) -> None:
        if role != self.role:
            self.role = role

    def begin_place_start(self) -> None:
        self.placing_start = True
        self.placing_obstacle = False
        self.say("キャンバスをクリック/ドラッグしてスタート地点を置いてください")

    def toggle_placing_obstacle(self) -> None:
        self.placing_obstacle = not self.placing_obstacle
        if self.placing_obstacle:
            self.placing_start = False
            self.say("キャンバスをクリックして障害物を置いてください（もう一度押すと終了）")

    def set_obstacle_diameter(self, v: float) -> None:
        self.obstacle_diameter = v

    def delete_obstacle(self, i: int) -> None:
        if 0 <= i < len(self.obstacles):
            del self.obstacles[i]
            self._mark_dirty()
            self.say(f"障害物 {i + 1} を削除しました")

    # ── プレビュー用コース ──

    def preview_course(self) -> Course | None:
        if self.mode == "centerline":
            return self._preview_centerline()
        return self._preview_wall()

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
            self.say(f"中心線を計算できません: {e}")
            return
        self._wall_centerline_preview = cl
        self._dirty = True
        self.say(f"中心線を生成しました（{len(cl)} 点）")

    # ── 読み書き ──

    def _editable_course_files(self) -> list[Path]:
        out = []
        for p in sorted(DEFAULT_COURSE_DIR.glob("*.json")):
            try:
                m = json.loads(p.read_text(encoding="utf-8"))
            except Exception:
                continue
            if "path" in m or m.get("mode") == "wall":
                out.append(p)
        return out

    def open(self, name: str) -> None:
        p = Path(name)
        if not p.exists():
            p = DEFAULT_COURSE_DIR / f"{Path(name).stem}.json"
        try:
            m = json.loads(p.read_text(encoding="utf-8"))
        except Exception as e:
            self.say(f"開けません: {e}")
            return

        if m.get("mode") == "wall":
            self.mode = "wall"
            self.wall_loops = [path_to_loop(tuple(w["origin"]), w["path"]) for w in m["walls"]]
            self.wall_current = Loop()
            self.wall_thickness = float(m.get("wall_thickness", DEFAULT_WALL_THICKNESS))
            start = m.get("start") or m["centerline"][0]
            self.wall_start = tuple(float(v) for v in start)
            self.name = p.stem
            self.say(f"{p.name} を開きました（壁 {len(self.wall_loops)} ループ）")
        elif "path" in m:
            self.mode = "centerline"
            self.center_loop = path_to_loop(tuple(m["origin"]), m["path"])
            self.width = float(m.get("width", DEFAULT_WIDTH))
            self.name = p.stem
            self.say(f"{p.name} を開きました（{len(self.center_loop.vertices)} 頂点）")
        else:
            self.say(f"{p.name} は PNG 方式なので編集できません")
            return
        role = str(m.get("role", "eval"))
        self.role = role if role in ("train", "eval", "both") else "eval"
        self.obstacles = [(float(x), float(y), float(r)) for x, y, r in m.get("obstacles", [])]
        self._fit_view()
        self._mark_dirty()

    def _open_next(self) -> None:
        files = self._editable_course_files()
        if not files:
            self.say("編集できるコースがありません")
            return
        stems = [p.stem for p in files]
        i = stems.index(self.name) + 1 if self.name in stems else 0
        self.open(str(files[i % len(files)]))

    def save(self) -> None:
        if self.mode == "centerline":
            self._save_centerline()
        else:
            self._save_wall()

    def _save_centerline(self) -> None:
        if len(self.center_loop.vertices) < 2:
            self.say("頂点がありません")
            return
        origin, path = loop_to_path(self.center_loop)
        p = DEFAULT_COURSE_DIR / f"{self.name}.json"
        m = {"name": self.name, "width": self.width, "resolution": CENTER_RESOLUTION,
            "origin": list(origin), "loop": self.center_loop.closed, "path": path,
            "obstacles": [list(o) for o in self.obstacles], "role": self.role,
            "note": "sim.editor（頂点スケッチ、中心線モード）で作成"}
        p.write_text(json.dumps(m, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        self.say(f"保存: {p.name}"
                 + ("（閉ループ）" if self.center_loop.closed else "（開いたまま）"))

    def _save_wall(self) -> None:
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
            self.say(f"保存できません: {e}")
            return

        walls_json = []
        for loop in self.wall_loops:
            o, path = loop_to_path(loop)
            walls_json.append({"origin": list(o), "path": path})

        p = DEFAULT_COURSE_DIR / f"{self.name}.json"
        m = {"name": self.name, "mode": "wall", "resolution": WALL_RESOLUTION,
            "wall_thickness": self.wall_thickness, "margin": WALL_MARGIN,
            "walls": walls_json, "centerline": centerline.tolist(),
            "start": list(self.wall_start),
            "obstacles": [list(o) for o in self.obstacles], "role": self.role,
            "note": "sim.editor（頂点スケッチ、壁モード）で作成。centerlineは自動導出"}
        p.write_text(json.dumps(m, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        self.say(f"保存: {p.name}（中心線を自動生成、{len(centerline)} 点）")

    # ── ループ ──

    def run(self) -> None:
        while self._events():
            self._draw()
            self.clock.tick(60)
        pygame.quit()

    def _events(self) -> bool:
        self.hover = None
        self.mouse_screen = pygame.mouse.get_pos()
        for b in self.buttons:
            if b.rect.collidepoint(self.mouse_screen):
                self.hover = b
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                return False
            if e.type == pygame.VIDEORESIZE:
                self.screen = pygame.display.set_mode(
                    (max(MIN_W, e.w), max(MIN_H, e.h)), pygame.RESIZABLE)
                self._resize(e.w, e.h)
            elif e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
                if self.hover:
                    self._press(self.hover)
                elif self.view.collidepoint(self.mouse_screen) and not self.naming:
                    self._canvas_down()
            elif e.type == pygame.MOUSEBUTTONUP and e.button == 1:
                if self.press_screen is not None:
                    self._canvas_up()
            elif e.type == pygame.MOUSEWHEEL:
                if self.view.collidepoint(self.mouse_screen) and not self.naming:
                    self._zoom(e.y, self.mouse_screen)
            elif e.type == pygame.KEYDOWN:
                if self.naming:
                    if not self._naming_key(e):
                        return True
                elif not self._key(e):
                    return False
        return True

    def _canvas_down(self) -> None:
        world = self.to_world(*self.mouse_screen)
        if self.placing_start or self.placing_obstacle:
            self.press_screen = self.mouse_screen
            self.press_world = snap_to_grid(world, self.grid_spacing) if self.grid_snap else world
            return
        loop = self._active_loop()
        snapped, _idx = self._snap(world, loop)
        self.press_screen = self.mouse_screen
        self.press_world = snapped

    def _canvas_up(self) -> None:
        assert self.press_screen is not None and self.press_world is not None
        is_drag = _dist(self.mouse_screen, self.press_screen) > DRAG_ARC_PX

        if self.placing_start:
            end_world = self.to_world(*self.mouse_screen)
            yaw = (math.atan2(end_world[1] - self.press_world[1],
                              end_world[0] - self.press_world[0]) if is_drag else 0.0)
            self.wall_start = (self.press_world[0], self.press_world[1], yaw)
            self.placing_start = False
            self._mark_dirty()
            self.say("スタート地点を配置しました")
        elif self.placing_obstacle:
            # ドラッグしても位置はダウン位置のまま（直径はパネルで選ぶ既定値固定
            # なので、離した位置で何かを決める意味が無い）
            x, y = self.press_world
            self.obstacles.append((x, y, self.obstacle_diameter / 2.0))
            self._mark_dirty()
            self.say(f"障害物を配置しました（計 {len(self.obstacles)} 個）")
        else:
            loop = self._active_loop()
            end, idx = self._snap(self.to_world(*self.mouse_screen), loop)
            self._commit_vertex(loop, is_drag, self.press_world, end, idx)

        self.press_screen = None
        self.press_world = None

    def _press(self, b: Btn) -> None:
        if b.kind == "act":
            b.value()
        elif b.kind == "mode":
            self.set_mode(b.value)
        elif b.kind == "role":
            self.set_role(b.value)
        elif b.kind == "grid_spacing":
            self.set_grid_spacing(b.value)
        elif b.kind == "wall_del":
            self.delete_wall_loop(b.value)
        elif b.kind == "obstacle_diameter":
            self.set_obstacle_diameter(b.value)
        elif b.kind == "obstacle_del":
            self.delete_obstacle(b.value)

    def _key(self, e) -> bool:
        k = e.key
        if k in (pygame.K_ESCAPE, pygame.K_q):
            if self.placing_start or self.placing_obstacle:
                self.placing_start = False
                self.placing_obstacle = False
                return True
            return False
        if k == pygame.K_BACKSPACE:
            self.undo()
        elif k == pygame.K_c:
            self.clear()
        elif k == pygame.K_n:
            self.new_course()
        elif k == pygame.K_s:
            self.naming = True
        elif k == pygame.K_o:
            self._open_next()
        elif k == pygame.K_g:
            self.toggle_grid_snap()
        elif k == pygame.K_b:
            self.toggle_placing_obstacle()
        elif k == pygame.K_RETURN:
            self.finish_current_line()
        elif k == pygame.K_TAB:
            self.set_mode("wall" if self.mode == "centerline" else "centerline")
        elif k == pygame.K_w:
            if self.mode == "centerline":
                self.set_width(WIDTH_STEP if (e.mod & pygame.KMOD_SHIFT) else -WIDTH_STEP)
            else:
                self.set_wall_thickness(
                    WALL_THICKNESS_STEP if (e.mod & pygame.KMOD_SHIFT) else -WALL_THICKNESS_STEP)
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
            ch = e.unicode
            if ch.isascii() and (ch.isalnum() or ch in "-_"):
                self.name = (self.name + ch)[:32]
        return True

    # ── 描画 ──

    def _draw(self) -> None:
        self.screen.fill(U.BG)
        self._draw_preview()
        self._draw_panel()
        if self.naming:
            self._draw_naming()
        pygame.display.flip()

    def _live_gesture_points(self) -> list[tuple[float, float]] | None:
        """まだ確定していない、今まさに置こうとしている区間のプレビュー点列。
        ドラッグ中は「直前の頂点→ドラッグ開始位置」の直線と「そこから現在の
        マウス位置」への接線円弧の両方を見せる（`_commit_vertex` と同じ構成）。
        """
        loop = self._active_loop()
        if not loop.vertices or self.placing_start or loop.closed:
            # 閉じた後も `vertices[-1]`（＝閉じる直前の点）からマウスへ線を
            # 引き続けると、ループを閉じた後もそこから伸びる線が残って見える
            # （実際に踏んだ罠）。閉じたループにはもう区間を足せないので、
            # プレビューも出さない
            return None
        last = loop.vertices[-1]
        end, _idx = self._snap(self.to_world(*self.mouse_screen), loop)
        is_drag = (self.press_screen is not None
                  and _dist(self.mouse_screen, self.press_screen) > DRAG_ARC_PX)
        tmp = Loop()
        tmp.add_line(last)
        if is_drag:
            tmp.add_line(self.press_world)
            tmp.add_arc(end)
        else:
            tmp.add_line(end)
        try:
            return [(x, y) for x, y, _ in sample_loop(tmp, step=0.05)]
        except Exception:
            return [last, end]

    def _draw_preview(self) -> None:
        course = self.preview_course()
        if course is None:
            msg = ("右下のパネルでモードを選び、キャンバスをクリックして"
                   "頂点を置いてください")
            g = self.ui.hd.render(msg, True, U.DIM)
            self.screen.blit(g, (self.view.centerx - g.get_width() // 2,
                                 self.view.centery - 10))
        else:
            dw = max(1, int(course.size_m[0] * self.view_scale))
            dh = max(1, int(course.size_m[1] * self.view_scale))
            ox, oy = self.to_px(course.origin[0], course.origin[1] + course.size_m[1])
            key = (f"edit:{self.mode}:{self.center_loop.vertices}:{self.center_loop.edges}:"
                  f"{[(l.vertices, l.edges) for l in self._all_wall_loops()]}:"
                  f"{self.width}:{self.wall_thickness}:{self.view_scale:.3f}:{self.obstacles}")
            self.screen.blit(U.map_surface(course, dw, dh, key=key), (int(ox), int(oy)))
            pygame.draw.rect(self.screen, U.LINE,
                             pygame.Rect(int(ox) - 1, int(oy) - 1, dw + 2, dh + 2), 1)

        self._draw_grid_overlay()
        self._draw_vertices()
        self._draw_obstacles()
        self._draw_live_gesture()
        if self.wall_start is not None:
            x, y, yaw = self.wall_start
            px, py = self.to_px(x, y)
            U.arrow(self.screen, px, py, yaw, max(14, 0.32 * self.view_scale), U.OK)

    def _draw_grid_overlay(self) -> None:
        """スナップ先のグリッドを実際に見せる。**「間隔の数値」ではなく
        「グリッドに吸着する」ことが直感的に分かるように**、選んだ間隔の格子線を
        常にキャンバスへ描く（頂点が1つも無い、コースがまだ描けない段階でも
        最初の頂点をどこに置けるかが見える）。スクロールでズームしても
        **今見えている範囲**に合わせて引き直す——コースの内容に合わせた範囲では
        なく、ビュー（`view_center`/`view_scale`）が唯一の基準。
        """
        if not self.grid_snap:
            return
        spacing = self.grid_spacing
        if spacing <= 0:
            return
        x0, y0, x1, y1 = self._visible_world_bounds()
        dw, dh = self.view.w, self.view.h
        surf = pygame.Surface((dw, dh), pygame.SRCALPHA)
        color = (*U.POINT, 50)
        x = math.floor(x0 / spacing) * spacing
        while x <= x1:
            px = self.to_px(x, 0.0)[0] - self.view.x
            pygame.draw.line(surf, color, (px, 0), (px, dh))
            x += spacing
        y = math.floor(y0 / spacing) * spacing
        while y <= y1:
            py = self.to_px(0.0, y)[1] - self.view.y
            pygame.draw.line(surf, color, (0, py), (dw, py))
            y += spacing
        self.screen.blit(surf, (self.view.x, self.view.y))

    def _draw_vertices(self) -> None:
        loops = ([self.center_loop] if self.mode == "centerline" else self._all_wall_loops())
        for loop in loops:
            for i, v in enumerate(loop.vertices):
                px, py = self.to_px(*v)
                color = U.ACCENT if i == 0 else U.POINT
                self.screen.blit(U.make_dot(color, r=4), (px - 4, py - 4))

    def _draw_obstacles(self) -> None:
        """障害物は円盤としてグリッドへ焼き込まれる（`map_surface` が壁と同じ色で
        描く）ので、それとは別に縁取りだけ重ねて「これはクリックで消せる障害物」
        だと分かるようにする。
        """
        for x, y, r in self.obstacles:
            px, py = self.to_px(x, y)
            pr = max(3, r * self.view_scale)
            pygame.draw.circle(self.screen, U.BAD, (int(px), int(py)), int(pr), 2)

    def _draw_live_gesture(self) -> None:
        pts = self._live_gesture_points()
        if pts is None or len(pts) < 2:
            return
        px_pts = [self.to_px(x, y) for x, y in pts]
        pygame.draw.aalines(self.screen, U.ACCENT, False, px_pts)

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

        y = self.ui.section(self.screen, "モード", x, y, inner)
        bw = (inner - 8) // 2
        for i, (mode, label) in enumerate((("centerline", "中心線"), ("wall", "壁"))):
            r = pygame.Rect(x + i * (bw + 8), y, bw, 28)
            b = Btn(r, "mode", mode, label)
            self.buttons.append(b)
            self.ui.button(self.screen, r, label, on=(self.mode == mode), hover=(self.hover is b))
        y += 38

        y = self.ui.section(self.screen, "役割（学習プールでの扱い）", x, y + 4, inner)
        n_role = 3
        bw_role = (inner - 8 * (n_role - 1)) // n_role
        for i, (role, label) in enumerate((("train", "学習"), ("eval", "評価"), ("both", "両方"))):
            r = pygame.Rect(x + i * (bw_role + 8), y, bw_role, 28)
            b = Btn(r, "role", role, label)
            self.buttons.append(b)
            self.ui.button(self.screen, r, label, on=(self.role == role), hover=(self.hover is b))
        y += 38

        y = self.ui.section(self.screen, "グリッド", x, y + 4, inner)
        r = pygame.Rect(x, y, inner, 28)
        b = Btn(r, "act", None, f"グリッドにスナップ: {'ON' if self.grid_snap else 'OFF'}  (G)")
        b.value = self.toggle_grid_snap
        self.buttons.append(b)
        self.ui.button(self.screen, r, b.label, on=self.grid_snap, hover=(self.hover is b))
        y += 34
        self.screen.blit(self.ui.sm.render("グリッド間隔 [m]（キャンバスに実線で表示）",
                                          True, U.DIM), (x, y))
        y += 18
        n = len(GRID_SPACINGS)
        bw = (inner - 8 * (n - 1)) // n
        for i, sp in enumerate(GRID_SPACINGS):
            r = pygame.Rect(x + i * (bw + 8), y, bw, 26)
            b = Btn(r, "grid_spacing", sp, f"{sp:g}m")
            self.buttons.append(b)
            self.ui.button(self.screen, r, b.label, on=(sp == self.grid_spacing),
                           hover=(self.hover is b))
        y += 34
        self.screen.blit(self.ui.sm.render("頂点への吸着は常時ON", True, U.FAINT), (x, y))
        y += 20

        y = self._draw_obstacle_panel(x, y + 8, inner)

        if self.mode == "centerline":
            y = self._draw_centerline_panel(x, y + 8, inner)
        else:
            y = self._draw_wall_panel(x, y + 8, inner)

        self._draw_actions(x, inner)

    def _draw_obstacle_panel(self, x: int, y: int, inner: int) -> int:
        y = self.ui.section(self.screen, "障害物", x, y, inner)
        r = pygame.Rect(x, y, inner, 28)
        label = f"障害物を配置: {'ON' if self.placing_obstacle else 'OFF'}  (B)"
        b = Btn(r, "act", None, label)
        b.value = self.toggle_placing_obstacle
        self.buttons.append(b)
        self.ui.button(self.screen, r, label, on=self.placing_obstacle, hover=(self.hover is b))
        y += 34

        self.screen.blit(self.ui.sm.render("直径 [m]", True, U.DIM), (x, y))
        y += 18
        n = len(OBSTACLE_DIAMETERS)
        bw = (inner - 8 * (n - 1)) // n
        for i, d in enumerate(OBSTACLE_DIAMETERS):
            r = pygame.Rect(x + i * (bw + 8), y, bw, 26)
            b = Btn(r, "obstacle_diameter", d, f"{d:g}m")
            self.buttons.append(b)
            self.ui.button(self.screen, r, b.label, on=(d == self.obstacle_diameter),
                           hover=(self.hover is b))
        y += 34

        shown = self.obstacles[-4:]
        for i, (_ox, _oy, r_obs) in enumerate(shown):
            real_i = len(self.obstacles) - len(shown) + i
            row = pygame.Rect(x, y, inner, 24)
            txt = f"障害物 {real_i + 1}: 直径 {r_obs * 2:g}m"
            self.screen.blit(self.ui.lbl.render(txt, True, U.DIM), (x, y + 4))
            dr = pygame.Rect(x + inner - 24, y, 24, 22)
            b = Btn(dr, "obstacle_del", real_i, "×")
            self.buttons.append(b)
            self.ui.button(self.screen, dr, "×", hover=(self.hover is b), color=U.BAD)
            y += 26
        if self.obstacles:
            y = self.ui.kv(self.screen, x, y + 2, inner, "障害物 [個]", f"{len(self.obstacles)}")
        return y + 6

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
        y += 40
        return y

    def _draw_actions(self, x: int, inner: int) -> None:
        acts = [("新規コースを作成  N", self.new_course, None),
                ("直前を取り消す  ⌫", self.undo, None),
                ("全消去  C", self.clear, U.BAD),
                ("既存を開く  O", self._open_next, None),
                ("保存  S", lambda: setattr(self, "naming", True), U.ACCENT)]
        hint = ("クリック=直線頂点 / ドラッグ=円弧    Enter 壁を確定    B 障害物    "
               "スクロール ズーム    Tab モード切替    Q 終了")
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
