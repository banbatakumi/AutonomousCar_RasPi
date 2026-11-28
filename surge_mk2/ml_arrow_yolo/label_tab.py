"""②ラベル付けタブ（矢印信号の YOLO）。画像の上をドラッグして枠を描き、クラスを付けて確定する。

操作（キーはこのタブが表のときだけ効く。入力欄にフォーカスがあるときは奪わない）:

    ドラッグ          枠を描く（今のクラスで）
    クリック          枠を選ぶ（選んだ枠は太線）
    右クリック        その位置の枠を消す
    1〜9             選んだ枠のクラスを変える（選んでいなければ「今のクラス」を変える）
    Delete/BackSpace  選んだ枠を消す
    Enter            確定して次へ（枠が無ければ「看板なし＝背景」として確定）
    ←/→ (a/d)        前/次へ（変えた枠は確定として保存する）
    n                次の未確定へ
    c                前のコマの枠を写す（続いたコマはほぼ同じ位置なので、写して直す方が速い）

下書き（`drafts/`、前のモデルが付けた枠）は点線で出す。Enter か編集で確定するまで学習に使わない。
"""

from __future__ import annotations

import tkinter as tk
from pathlib import Path
from tkinter import ttk
from typing import Callable

from dataset import Box, RunPaths, label_stats, list_images, read_classes, read_labels, write_labels

__all__ = ["LabelTab", "CLASS_COLORS", "box_at"]

CLASS_COLORS = ["#3fa9f5", "#ff5a5a", "#ffd23f", "#7ddc5a", "#c77dff", "#ff9f43", "#00d2d3",
                "#f368e0", "#c8d6e5"]

#: 小さすぎる枠（誤クリック）は捨てる。画像に対する割合
_MIN_BOX = 0.005

FILTERS = ("全部", "未確定", "確定済み", "下書きあり")


def box_at(boxes: list[Box], x: float, y: float) -> int | None:
    """正規化座標 (x, y) を含む枠のうち、一番小さいものの番号（重なりでは内側を選ぶ）。"""
    hits = [(b.area, i) for i, b in enumerate(boxes) if b.x0 <= x <= b.x1 and b.y0 <= y <= b.y1]
    return min(hits)[1] if hits else None


class LabelTab:
    def __init__(self, parent: ttk.Notebook, *, get_run_dir: Callable[[], Path | None],
                 log: Callable[[str], None]) -> None:
        self.get_run_dir = get_run_dir
        self.log = log
        self.frame = ttk.Frame(parent, padding=6)
        self.paths: RunPaths | None = None
        self.classes: list[str] = []
        self.all_images: list[Path] = []
        self.items: list[Path] = []
        self.i = 0
        self.boxes: list[Box] = []
        self.state = ""                 # "確定" / "下書き" / "未確定"
        self.dirty = False
        self.selected: int | None = None
        self.cur_class = tk.IntVar(value=0)
        self.filter_var = tk.StringVar(value="全部")
        self._photo = None
        self._img_size = (1, 1)
        self._view = (1.0, 0.0, 0.0)    # scale, ox, oy
        self._drag: tuple[float, float] | None = None
        self._drag_item = None
        self._press_hit: int | None = None
        self._pil = None
        self._build()

    # ── 画面 ──

    def _build(self) -> None:
        top = ttk.Frame(self.frame)
        top.pack(fill="x")
        ttk.Label(top, text="表示:").pack(side="left")
        cb = ttk.Combobox(top, textvariable=self.filter_var, values=FILTERS, width=10, state="readonly")
        cb.pack(side="left", padx=(0, 8))
        cb.bind("<<ComboboxSelected>>", lambda _e: self.reload(keep_current=True))
        ttk.Button(top, text="◀", width=3, command=lambda: self.step(-1)).pack(side="left")
        ttk.Button(top, text="▶", width=3, command=lambda: self.step(1)).pack(side="left")
        ttk.Button(top, text="次の未確定 (n)", command=self.next_unlabeled).pack(side="left", padx=4)
        ttk.Button(top, text="確定して次へ (Enter)", command=self.confirm_and_next).pack(side="left", padx=4)
        ttk.Button(top, text="前のコマの枠を写す (c)", command=self.copy_prev).pack(side="left", padx=4)
        ttk.Button(top, text="枠を全部消す", command=self.clear_boxes).pack(side="left", padx=4)
        self.pos_label = ttk.Label(top, text="")
        self.pos_label.pack(side="right")

        self.class_bar = ttk.Frame(self.frame)
        self.class_bar.pack(fill="x", pady=(6, 2))

        self.canvas = tk.Canvas(self.frame, background="#202020", highlightthickness=0, cursor="crosshair")
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda _e: self._redraw())
        self.canvas.bind("<ButtonPress-1>", self._on_press)
        self.canvas.bind("<B1-Motion>", self._on_motion)
        self.canvas.bind("<ButtonRelease-1>", self._on_release)
        self.canvas.bind("<Button-3>", self._on_right)
        self.canvas.bind("<Button-2>", self._on_right)          # Mac のトラックパッドの副ボタン

        bottom = ttk.Frame(self.frame)
        bottom.pack(fill="x", pady=(4, 0))
        self.info_label = ttk.Label(bottom, text="", foreground="gray")
        self.info_label.pack(side="left")
        self.stats_label = ttk.Label(bottom, text="", foreground="gray")
        self.stats_label.pack(side="right")

    def _build_class_bar(self) -> None:
        for w in self.class_bar.winfo_children():
            w.destroy()
        ttk.Label(self.class_bar, text="クラス:").pack(side="left")
        for i, name in enumerate(self.classes[:9]):
            tk.Radiobutton(self.class_bar, text=f"{i + 1} {name}", variable=self.cur_class, value=i,
                           indicatoron=False, padx=8, pady=2, selectcolor=CLASS_COLORS[i % len(CLASS_COLORS)],
                           command=self._on_class_button).pack(side="left", padx=2)
        if self.cur_class.get() >= len(self.classes):
            self.cur_class.set(0)

    # ── 読み込み ──

    def reload(self, keep_current: bool = False) -> None:
        """モデル名・フィルタが変わった・タブを開いたときに読み直す。"""
        current = self.items[self.i] if keep_current and self.items else None
        self._save_if_dirty()
        run_dir = self.get_run_dir()
        self.paths = RunPaths(run_dir) if run_dir is not None else None
        self.classes = read_classes(self.paths) if self.paths else []
        self._build_class_bar()
        self.all_images = list_images(self.paths) if self.paths else []
        self._apply_filter()
        if current is not None and current in self.items:
            self.i = self.items.index(current)
        elif not keep_current:
            # 開いたら最初の未確定へ（続きから付けられるように）
            self.i = next((k for k, p in enumerate(self.items) if not self._is_labeled(p)), 0)
        self.i = min(self.i, max(0, len(self.items) - 1))
        self._load_current()

    def _is_labeled(self, img: Path) -> bool:
        return self.paths is not None and self.paths.label_path(img).is_file()

    def _apply_filter(self) -> None:
        f = self.filter_var.get()
        p = self.paths
        if p is None:
            self.items = []
        elif f == "未確定":
            self.items = [x for x in self.all_images if not p.label_path(x).is_file()]
        elif f == "確定済み":
            self.items = [x for x in self.all_images if p.label_path(x).is_file()]
        elif f == "下書きあり":
            self.items = [x for x in self.all_images
                          if not p.label_path(x).is_file() and p.draft_path(x).is_file()]
        else:
            self.items = list(self.all_images)

    def _load_current(self) -> None:
        self.selected = None
        self.dirty = False
        if not self.items or self.paths is None:
            self.boxes, self.state = [], ""
            self._photo = None
            self._redraw()
            self._update_labels()
            return
        img = self.items[self.i]
        boxes = read_labels(self.paths.label_path(img))
        if boxes is not None:
            self.boxes, self.state = boxes, "確定"
        else:
            draft = read_labels(self.paths.draft_path(img))
            self.boxes, self.state = (draft, "下書き") if draft is not None else ([], "未確定")
        self._load_image(img)
        self._redraw()
        self._update_labels()

    def _load_image(self, img: Path) -> None:
        from PIL import Image

        try:
            self._pil = Image.open(img).convert("RGB")
        except OSError as e:
            self.log(f"画像が読めません: {img.name}: {e}\n")
            self._pil = None
        self._img_size = self._pil.size if self._pil is not None else (1, 1)

    # ── 保存・移動 ──

    def _save(self) -> None:
        if self.paths is None or not self.items:
            return
        write_labels(self.paths.label_path(self.items[self.i]), self.boxes)
        self.state, self.dirty = "確定", False

    def _save_if_dirty(self) -> None:
        if self.dirty:
            self._save()

    def step(self, d: int) -> None:
        if not self.items:
            return
        self._save_if_dirty()
        self.i = max(0, min(len(self.items) - 1, self.i + d))
        self._load_current()

    def confirm_and_next(self) -> None:
        if not self.items:
            return
        self._save()
        if self.filter_var.get() in ("未確定", "下書きあり"):
            # 確定した画像はこの表示の対象から外れる。同じ位置が次の画像になる
            cur = self.items[self.i]
            self._apply_filter()
            if cur in self.items:
                self.i = min(self.items.index(cur) + 1, len(self.items) - 1)
            self.i = min(self.i, max(0, len(self.items) - 1))
            self._load_current()
        elif self.i < len(self.items) - 1:
            self.step(1)
        else:
            self._update_labels()

    def next_unlabeled(self) -> None:
        self._save_if_dirty()
        for k in list(range(self.i + 1, len(self.items))) + list(range(0, self.i)):
            if not self._is_labeled(self.items[k]):
                self.i = k
                self._load_current()
                return
        self.log("未確定の画像はありません\n")

    def copy_prev(self) -> None:
        """全体の並びで1つ前の画像の枠（確定か下書き）を写す。"""
        if not self.items or self.paths is None:
            return
        cur = self.items[self.i]
        k = self.all_images.index(cur)
        if k == 0:
            return
        prev = self.all_images[k - 1]
        boxes = read_labels(self.paths.label_path(prev))
        if boxes is None:
            boxes = read_labels(self.paths.draft_path(prev)) or []
        self.boxes = [Box(b.cls, b.x0, b.y0, b.x1, b.y1) for b in boxes]
        self.dirty = True
        self.selected = None
        self._redraw()
        self._update_labels()

    def clear_boxes(self) -> None:
        self.boxes = []
        self.selected = None
        self.dirty = True
        self._redraw()
        self._update_labels()

    def stop(self) -> None:
        """タブを離れるときに呼ぶ（変えた枠を保存する）。"""
        self._save_if_dirty()
        self._update_labels()

    # ── 描画 ──

    def _redraw(self) -> None:
        c = self.canvas
        c.delete("all")
        cw, ch = max(1, c.winfo_width()), max(1, c.winfo_height())
        if self._pil is None or not self.items:
            c.create_text(cw // 2, ch // 2, fill="gray",
                          text="画像がありません（①フレーム抽出で記録から取り出してください）")
            return
        from PIL import ImageTk

        w, h = self._img_size
        s = min(cw / w, ch / h)
        dw, dh = max(1, int(w * s)), max(1, int(h * s))
        ox, oy = (cw - dw) // 2, (ch - dh) // 2
        self._view = (s, ox, oy)
        self._photo = ImageTk.PhotoImage(self._pil.resize((dw, dh)))
        c.create_image(ox, oy, image=self._photo, anchor="nw")
        dash = (5, 3) if self.state == "下書き" and not self.dirty else None
        for i, b in enumerate(self.boxes):
            color = CLASS_COLORS[b.cls % len(CLASS_COLORS)]
            x0, y0 = self._to_canvas(b.x0, b.y0)
            x1, y1 = self._to_canvas(b.x1, b.y1)
            c.create_rectangle(x0, y0, x1, y1, outline=color, width=4 if i == self.selected else 2,
                               dash=dash)
            name = self.classes[b.cls] if 0 <= b.cls < len(self.classes) else f"#{b.cls}"
            c.create_text(x0 + 2, y0 - 2, text=name, anchor="sw", fill=color, font=("", 12, "bold"))

    def _to_canvas(self, x: float, y: float) -> tuple[float, float]:
        s, ox, oy = self._view
        w, h = self._img_size
        return ox + x * w * s, oy + y * h * s

    def _to_norm(self, cx: float, cy: float) -> tuple[float, float]:
        s, ox, oy = self._view
        w, h = self._img_size
        return (cx - ox) / (w * s), (cy - oy) / (h * s)

    def _update_labels(self) -> None:
        if not self.items:
            self.pos_label.config(text="0 / 0")
            self.info_label.config(text="")
        else:
            img = self.items[self.i]
            self.pos_label.config(text=f"{self.i + 1} / {len(self.items)}")
            state = self.state + ("（変更あり・移動で確定）" if self.dirty else "")
            self.info_label.config(text=f"{img.name}　{state}　枠 {len(self.boxes)}")
        if self.paths is not None:
            st = label_stats(self.paths, len(self.classes))
            per = "・".join(f"{n} {k}" for n, k in zip(self.classes, st["per_class"]))
            self.stats_label.config(
                text=f"確定 {st['labeled']}/{st['images']}（背景 {st['background']}）　下書き {st['drafts']}　{per}")
        else:
            self.stats_label.config(text="")

    # ── マウス ──

    def _on_press(self, e) -> None:
        if not self.items:
            return
        x, y = self._to_norm(e.x, e.y)
        hit = box_at(self.boxes, x, y)
        self._drag = (x, y)
        self._press_hit = hit
        self._drag_item = None

    def _on_motion(self, e) -> None:
        if self._drag is None:
            return
        if self._drag_item is not None:
            self.canvas.delete(self._drag_item)
        x0, y0 = self._to_canvas(*self._drag)
        color = CLASS_COLORS[self.cur_class.get() % len(CLASS_COLORS)]
        self._drag_item = self.canvas.create_rectangle(x0, y0, e.x, e.y, outline=color, width=2)

    def _on_release(self, e) -> None:
        if self._drag is None:
            return
        x0, y0 = self._drag
        x1, y1 = self._to_norm(e.x, e.y)
        self._drag = None
        b = Box(self.cur_class.get(), x0, y0, x1, y1).normalized()
        if (b.x1 - b.x0) >= _MIN_BOX and (b.y1 - b.y0) >= _MIN_BOX:
            self.boxes.append(b)
            self.selected = len(self.boxes) - 1
            self.dirty = True
        else:
            self.selected = self._press_hit        # 動かさずに離した＝クリックで選ぶ
        self._redraw()
        self._update_labels()

    def _on_right(self, e) -> None:
        x, y = self._to_norm(e.x, e.y)
        hit = box_at(self.boxes, x, y)
        if hit is not None:
            del self.boxes[hit]
            self.selected = None
            self.dirty = True
            self._redraw()
            self._update_labels()

    def _on_class_button(self) -> None:
        if self.selected is not None and self.selected < len(self.boxes):
            self.boxes[self.selected].cls = self.cur_class.get()
            self.dirty = True
            self._redraw()
            self._update_labels()

    # ── キー（app.py から回ってくる） ──

    def handle_key(self, e) -> bool:
        k = e.keysym
        if k in ("Right", "d"):
            self.step(1)
        elif k in ("Left", "a"):
            self.step(-1)
        elif k in ("Return", "KP_Enter"):
            self.confirm_and_next()
        elif k == "n":
            self.next_unlabeled()
        elif k == "c":
            self.copy_prev()
        elif k in ("Delete", "BackSpace"):
            if self.selected is not None and self.selected < len(self.boxes):
                del self.boxes[self.selected]
                self.selected = None
                self.dirty = True
                self._redraw()
                self._update_labels()
        elif len(e.char) == 1 and e.char.isdigit() and e.char != "0":
            n = int(e.char) - 1
            if n < len(self.classes):
                self.cur_class.set(n)
                self._on_class_button()
        else:
            return False
        return True
