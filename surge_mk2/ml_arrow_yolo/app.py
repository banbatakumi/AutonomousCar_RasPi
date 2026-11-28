"""ml_arrow_yolo/app.py — 矢印信号の YOLO のデータ作成・ラベル付け・学習・書き出しをターミナル無しで行う操作パネル。

    .venv/bin/python ml_arrow_yolo/app.py
    （または `launcher.command`（surge_mk2 直下）の「学習」→「ml_arrow_yolo」）

タブは作業の順に4枚:

    ① フレーム抽出   録った .mcap から前方カメラの JPEG を取り出す
    ② ラベル付け     矢印の LED 表示を枠で囲んでクラスを付ける（前のモデルで下書きもできる）
    ③ 学習           ultralytics で YOLO を学習する（左右反転なし・色の拡張は弱め）
    ④ エクスポート   車が読む ONNX と .json を `models/arrow_yolo/` に書く

①③④と下書きは `extract_frames.py`・`train.py`・`export_onnx.py`・`prelabel.py` をサブプロセスで
呼ぶだけ（このプロセスは ultralytics・torch を import しない）。置き場とラベルの約束は `dataset.py`。

    ml_arrow_yolo/runs/<モデル名>/images/      … ①の出力・②の入力
    ml_arrow_yolo/runs/<モデル名>/labels/      … ②で確定したラベル（③の入力）
    ml_arrow_yolo/runs/<モデル名>/best.pt      … ③の出力・④の入力
    ml_arrow_yolo/runs/<モデル名>/note.txt     … 備考（④で .json の note になり、車の GUI の一覧に出る）
    models/arrow_yolo/<モデル名>.onnx          … ④の出力（車の GUI が選ぶ場所）

**モデル名を変えても画像を持ち越せる**: ①の「別のモデル名から画像とラベルを写す」で、前の版の
データに記録を足して学習し直す。
"""

from __future__ import annotations

import shutil
import sys
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

ML_DIR = Path(__file__).resolve().parent
REPO_ROOT = ML_DIR.parent
sys.path.insert(0, str(ML_DIR))
sys.path.insert(0, str(REPO_ROOT))

from ml_common.job_panel import JobPanelBase  # noqa: E402
from ml_common.naming import list_versioned_names, next_versioned_name  # noqa: E402
from ml_common.paths import make_rel  # noqa: E402

from dataset import (  # noqa: E402
    DEFAULT_CLASSES,
    RunPaths,
    label_stats,
    list_images,
    read_classes,
    write_classes,
)
from label_tab import LabelTab  # noqa: E402

__all__ = [
    "ML_DIR", "REPO_ROOT", "RUNS_DIR", "MODELS_DIR",
    "build_extract_cmd", "build_train_cmd", "build_export_cmd", "build_prelabel_cmd",
    "copy_dataset", "describe_status", "App",
]

RUNS_DIR = ML_DIR / "runs"
#: 車側（`raspi/nodes/arrow_signal_node.py` の `DEFAULT_MODELS_DIR`）と同じ場所
MODELS_DIR = REPO_ROOT / "models" / "arrow_yolo"
#: 記録の既定の置き場（Pi から取ってきた .mcap）
LOGS_DIR = REPO_ROOT / "logs"

rel = make_rel(REPO_ROOT)


# ── コマンド組み立て（Tkinter を一切知らない純粋関数） ──

def build_extract_cmd(python: str, mcap_files: list[str], out_dir: str, *, cam: str = "front",
                      min_interval_ms: int = 200) -> list[str]:
    return [python, str(ML_DIR / "extract_frames.py"), *mcap_files, "--out", out_dir,
            "--cam", cam, "--min-interval-ms", str(min_interval_ms)]


def build_train_cmd(python: str, run_dir: str, *, model: str = "yolo11n.pt", epochs: int = 100,
                    imgsz: int = 320, batch: int = 16, val_frac: float = 0.2,
                    device: str = "") -> list[str]:
    cmd = [python, str(ML_DIR / "train.py"), "--run", run_dir, "--model", model,
           "--epochs", str(epochs), "--imgsz", str(imgsz), "--batch", str(batch),
           "--val-frac", f"{val_frac:g}"]
    if device:
        cmd += ["--device", device]
    return cmd


def build_export_cmd(python: str, run_dir: str, out_path: str, *, imgsz: int = 320) -> list[str]:
    return [python, str(ML_DIR / "export_onnx.py"), "--run", run_dir, "--out", out_path,
            "--imgsz", str(imgsz)]


def build_prelabel_cmd(python: str, run_dir: str, model_path: str, *, conf: float = 0.3,
                       overwrite: bool = False) -> list[str]:
    cmd = [python, str(ML_DIR / "prelabel.py"), "--run", run_dir, "--model", model_path,
           "--conf", f"{conf:g}"]
    if overwrite:
        cmd.append("--overwrite")
    return cmd


def copy_dataset(src: RunPaths, dst: RunPaths) -> int:
    """`src` の画像・確定ラベル・クラスを `dst` に写す（同名は上書きしない）。写した画像の枚数。"""
    n = 0
    for sub in ("images", "labels"):
        s, d = src.run_dir / sub, dst.run_dir / sub
        if not s.is_dir():
            continue
        d.mkdir(parents=True, exist_ok=True)
        for p in s.iterdir():
            if p.is_file() and not (d / p.name).exists():
                shutil.copy2(p, d / p.name)
                n += sub == "images"
    if src.classes.is_file() and not dst.classes.is_file():
        shutil.copy2(src.classes, dst.classes)
    return n


def describe_status(run_dir: Path) -> str:
    paths = RunPaths(run_dir)
    classes = read_classes(paths)
    st = label_stats(paths, len(classes))
    mark = "✓学習済み" if paths.best.is_file() else "未学習"
    return f"画像 {st['images']}枚・確定 {st['labeled']}枚（背景 {st['background']}）・{mark}"


# ── GUI ──

class App(JobPanelBase):
    model_name_hint = "①タブでモデル名を入力してください"

    def __init__(self, root: tk.Tk) -> None:
        self._init_job_panel(root, REPO_ROOT)
        root.title("SURGE Mk.2 — 矢印信号の YOLO")
        root.geometry("1100x900")
        self._build_widgets()
        self._start_job_panel()

    def _build_widgets(self) -> None:
        self.model_name_var = tk.StringVar(value=next_versioned_name(list_versioned_names(RUNS_DIR)))
        self.status_var = tk.StringVar(value="")
        self.images_path_var = tk.StringVar(value="")
        self.export_out_var = tk.StringVar(value="")

        self.log_console = self._build_log_and_status(log_height=7, at_bottom=True)

        nb = ttk.Notebook(self.root)
        nb.pack(side="top", fill="both", expand=True, padx=8, pady=8)
        self.notebook = nb
        self._build_extract_tab(nb)
        self.label_tab = LabelTab(nb, get_run_dir=self._current_model_dir, log=self._append_log)
        nb.add(self.label_tab.frame, text="② ラベル付け")
        self._build_train_tab(nb)
        self._build_export_tab(nb)
        nb.bind("<<NotebookTabChanged>>", self._on_tab_changed)
        self.root.bind("<Key>", self._on_key)
        self._bind_model_name(self.model_name_var)

    # ── 配線 ──

    def _current_model_dir(self) -> Path | None:
        name = self.model_name_var.get().strip()
        if not name or "/" in name or "\\" in name or name.startswith("."):
            return None
        return RUNS_DIR / name

    def _on_model_name_changed(self) -> None:
        d = self._current_model_dir()
        if d is not None:
            self.images_path_var.set(rel(d / "images"))
            self.export_out_var.set(rel(MODELS_DIR / f"{d.name}.onnx"))
            self.status_var.set(describe_status(d))
            self.classes_var.set(", ".join(read_classes(RunPaths(d))))
        else:
            self.images_path_var.set("（モデル名を入力してください）")
            self.export_out_var.set("")
            self.status_var.set("")
        self._show_note(d)
        self._on_tab_changed()

    def _after_job(self, ok: bool) -> None:
        d = self._current_model_dir()
        if d is not None:
            self.status_var.set(describe_status(d))

    def _on_tab_changed(self, _event=None) -> None:
        if not hasattr(self, "notebook"):
            return
        if self.notebook.select() == str(self.label_tab.frame):
            self.label_tab.reload()
        else:
            self.label_tab.stop()
            d = self._current_model_dir()
            if d is not None:
                self.status_var.set(describe_status(d))

    def _on_key(self, event):
        cls = event.widget.winfo_class()
        if cls in ("TEntry", "Entry", "Text", "TCombobox", "TSpinbox", "Spinbox"):
            return None
        if cls == "TButton" and event.keysym in ("space", "Return"):
            return None
        if self.notebook.select() == str(self.label_tab.frame) and self.label_tab.handle_key(event):
            return "break"
        return None

    # ── ① フレーム抽出 ──

    def _build_extract_tab(self, nb: ttk.Notebook) -> None:
        f = ttk.Frame(nb, padding=10)
        nb.add(f, text="① フレーム抽出")
        f.columnconfigure(1, weight=1)

        ttk.Label(f, text="モデル名:").grid(row=0, column=0, sticky="w")
        self.model_combo = ttk.Combobox(f, textvariable=self.model_name_var, width=20)
        self.model_combo.grid(row=0, column=1, sticky="w")
        self.model_combo["postcommand"] = lambda: self.model_combo.configure(
            values=list_versioned_names(RUNS_DIR))
        ttk.Label(f, text="既存を選べば続きから、新しい名前なら新規。②〜④タブはここの名前を使います",
                  foreground="gray").grid(row=1, column=0, columnspan=3, sticky="w")
        ttk.Label(f, textvariable=self.status_var, foreground="gray").grid(
            row=2, column=0, columnspan=3, sticky="w", pady=(2, 8))

        ttk.Label(f, text="クラス:").grid(row=3, column=0, sticky="w")
        self.classes_var = tk.StringVar(value=", ".join(DEFAULT_CLASSES))
        ttk.Entry(f, textvariable=self.classes_var, width=50).grid(row=3, column=1, sticky="w")
        ttk.Button(f, text="保存", command=self._save_classes).grid(row=3, column=2, sticky="w")
        ttk.Label(f, text="カンマ区切り。車は left / right / straight を方向として読み、それ以外の名前"
                          "（例: left_dark＝濃い色の表示）は「非該当」として無視します。\n"
                          "ラベルを付け始めた後に順番を変えないこと（番号で保存しているため。足すのは末尾に）",
                  foreground="gray").grid(row=4, column=0, columnspan=3, sticky="w", pady=(0, 8))

        ttk.Label(f, text="記録 (.mcap):").grid(row=5, column=0, sticky="nw")
        self._mcaps: list[str] = []
        self.mcap_list = tk.Listbox(f, height=6)
        self.mcap_list.grid(row=5, column=1, sticky="ew")
        btns = ttk.Frame(f)
        btns.grid(row=5, column=2, sticky="nw", padx=4)
        ttk.Button(btns, text="追加…", command=self._pick_mcaps).pack(fill="x")
        ttk.Button(btns, text="外す", command=self._remove_mcap).pack(fill="x", pady=2)

        opt = ttk.Frame(f)
        opt.grid(row=6, column=1, sticky="w", pady=6)
        ttk.Label(opt, text="間隔の下限 [ms]:").pack(side="left")
        self.interval_var = tk.IntVar(value=200)
        ttk.Spinbox(opt, from_=0, to=5000, increment=50, textvariable=self.interval_var,
                    width=7).pack(side="left", padx=(0, 12))
        ttk.Label(opt, text="カメラ:").pack(side="left")
        self.cam_var = tk.StringVar(value="front")
        ttk.Combobox(opt, textvariable=self.cam_var, values=("front", "left"), width=7,
                     state="readonly").pack(side="left")
        ttk.Label(f, text="記録は GUI の MCAP 記録を「画像付き」で録ったもの（前方カメラは 5Hz）。"
                          "止まった車の前で同じ絵が並ばないよう、間隔の下限で間引きます",
                  foreground="gray").grid(row=7, column=0, columnspan=3, sticky="w")

        ttk.Label(f, text="出力先:").grid(row=8, column=0, sticky="w", pady=(8, 0))
        ttk.Label(f, textvariable=self.images_path_var, foreground="gray").grid(
            row=8, column=1, sticky="w", pady=(8, 0))
        ttk.Button(f, text="▶ 抽出する", command=self._run_extract).grid(
            row=9, column=1, sticky="w", pady=8)

        sep = ttk.Separator(f)
        sep.grid(row=10, column=0, columnspan=3, sticky="ew", pady=8)
        ttk.Label(f, text="別のモデル名から画像とラベルを写す:").grid(row=11, column=0, columnspan=2, sticky="w")
        self.copy_src_var = tk.StringVar()
        src = ttk.Combobox(f, textvariable=self.copy_src_var, width=20, state="readonly")
        src.grid(row=12, column=1, sticky="w")
        src["postcommand"] = lambda: src.configure(values=list_versioned_names(RUNS_DIR))
        ttk.Button(f, text="写す", command=self._copy_from).grid(row=12, column=2, sticky="w")

        ttk.Label(f, text="備考（④で .json の note になり、車の GUI のモデル一覧に出ます）:").grid(
            row=13, column=0, columnspan=3, sticky="w", pady=(12, 0))
        self.note_text = tk.Text(f, height=3, width=80)
        self.note_text.grid(row=14, column=0, columnspan=2, sticky="ew")
        ttk.Button(f, text="備考を保存", command=self._save_note).grid(row=14, column=2, sticky="nw")

    def _save_classes(self) -> None:
        d = self._require_model_dir()
        if d is None:
            return
        names = [n.strip() for n in self.classes_var.get().split(",") if n.strip()]
        if not names:
            messagebox.showerror("入力エラー", "クラスを1つ以上入れてください")
            return
        if len(set(names)) != len(names):
            messagebox.showerror("入力エラー", "同じ名前のクラスがあります")
            return
        if not any(n in ("left", "right", "straight") for n in names):
            if not messagebox.askyesno("確認", "left / right / straight のどれも無いと、車は方向を読めません。"
                                       "このまま保存しますか？"):
                return
        old = read_classes(RunPaths(d))
        if RunPaths(d).labels.is_dir() and names[:len(old)] != old[:len(names)]:
            if not messagebox.askyesno("確認", f"今のクラス {old} と順番が違います。付けたラベルの意味が"
                                       "変わりますが、保存しますか？"):
                return
        write_classes(RunPaths(d), names)
        self._append_log(f"\nクラスを保存しました: {names}\n")

    def _pick_mcaps(self) -> None:
        init = LOGS_DIR if LOGS_DIR.is_dir() else REPO_ROOT
        files = filedialog.askopenfilenames(initialdir=str(init), title="記録を選ぶ",
                                            filetypes=[("MCAP", "*.mcap"), ("すべて", "*")])
        for p in files:
            if p not in self._mcaps:
                self._mcaps.append(p)
                self.mcap_list.insert("end", Path(p).name)

    def _remove_mcap(self) -> None:
        for i in reversed(self.mcap_list.curselection()):
            self.mcap_list.delete(i)
            del self._mcaps[i]

    def _run_extract(self) -> None:
        d = self._require_model_dir()
        if d is None:
            return
        if not self._mcaps:
            messagebox.showerror("入力エラー", "記録 (.mcap) を追加してください")
            return
        if not RunPaths(d).classes.is_file():
            write_classes(RunPaths(d), [n.strip() for n in self.classes_var.get().split(",") if n.strip()]
                          or DEFAULT_CLASSES)
        cmd = build_extract_cmd(self.python, list(self._mcaps), str(d / "images"),
                                cam=self.cam_var.get(), min_interval_ms=int(self.interval_var.get()))
        self._run("extract", cmd, "フレーム抽出")

    def _copy_from(self) -> None:
        d = self._require_model_dir()
        src = self.copy_src_var.get()
        if d is None or not src:
            return
        if src == d.name:
            messagebox.showerror("入力エラー", "同じモデル名です")
            return
        n = copy_dataset(RunPaths(RUNS_DIR / src), RunPaths(d))
        self._append_log(f"\n{src} から画像 {n}枚（とラベル）を写しました\n")
        self._on_model_name_changed()

    # ── ③ 学習 ──

    def _build_train_tab(self, nb: ttk.Notebook) -> None:
        f = ttk.Frame(nb, padding=10)
        nb.add(f, text="③ 学習")

        ttk.Label(f, textvariable=self.status_var, foreground="gray").grid(
            row=0, column=0, columnspan=4, sticky="w", pady=(0, 8))
        self.base_var = tk.StringVar(value="yolo11n.pt")
        self.epochs_var = tk.IntVar(value=100)
        self.imgsz_var = tk.IntVar(value=320)
        self.batch_var = tk.IntVar(value=16)
        self.val_var = tk.DoubleVar(value=0.2)
        self.device_var = tk.StringVar(value="")
        rows = [
            ("元にする重み", ttk.Combobox(f, textvariable=self.base_var, width=14,
                                        values=("yolo11n.pt", "yolo11s.pt", "yolov8n.pt"))),
            ("epoch 数", ttk.Spinbox(f, from_=1, to=1000, textvariable=self.epochs_var, width=8)),
            ("入力の大きさ", ttk.Combobox(f, textvariable=self.imgsz_var, width=8,
                                       values=(256, 320, 416, 480, 640))),
            ("バッチ", ttk.Spinbox(f, from_=1, to=128, textvariable=self.batch_var, width=8)),
            ("検証に回す割合", ttk.Spinbox(f, from_=0.0, to=0.5, increment=0.05,
                                        textvariable=self.val_var, width=8)),
            ("デバイス", ttk.Combobox(f, textvariable=self.device_var, width=8,
                                    values=("", "mps", "cpu", "0"))),
        ]
        for i, (label, w) in enumerate(rows, start=1):
            ttk.Label(f, text=label + ":").grid(row=i, column=0, sticky="w", pady=2)
            w.grid(row=i, column=1, sticky="w", pady=2)
        ttk.Label(f, text="・n（nano）が Pi で一番軽い。入力の大きさは④と同じ値で書き出す（Pi の推論時間に効く）\n"
                          "・検証は記録ごとに分ける（記録が1本なら時刻の後ろを検証に）。デバイスは空なら自動（Mac は mps）\n"
                          "・左右反転はしない・色の拡張は弱め（`train.py` の TRAIN_AUG）。初回は元の重みをダウンロードする",
                  foreground="gray", justify="left").grid(row=len(rows) + 1, column=0, columnspan=4,
                                                          sticky="w", pady=8)
        ttk.Button(f, text="▶ 学習する", command=self._run_train).grid(
            row=len(rows) + 2, column=1, sticky="w")
        ttk.Label(f, text="結果のグラフ・混同行列は runs/<モデル名>/train/ に出ます",
                  foreground="gray").grid(row=len(rows) + 3, column=0, columnspan=4, sticky="w", pady=4)
        ttk.Button(f, text="結果のフォルダを開く", command=self._open_train_dir).grid(
            row=len(rows) + 4, column=1, sticky="w")

    def _run_train(self) -> None:
        d = self._require_model_dir()
        if d is None:
            return
        self.label_tab.stop()                       # 変えかけの枠を保存してから
        st = label_stats(RunPaths(d), len(read_classes(RunPaths(d))))
        if st["labeled"] == 0:
            messagebox.showerror("ラベルがありません", "②ラベル付けで確定した画像がありません")
            return
        if st["labeled"] < 50 and not messagebox.askyesno(
                "確認", f"確定した画像が {st['labeled']}枚しかありません。学習しますか？"):
            return
        try:
            cmd = build_train_cmd(self.python, str(d), model=self.base_var.get(),
                                  epochs=int(self.epochs_var.get()), imgsz=int(self.imgsz_var.get()),
                                  batch=int(self.batch_var.get()), val_frac=float(self.val_var.get()),
                                  device=self.device_var.get().strip())
        except (tk.TclError, ValueError):
            messagebox.showerror("入力エラー", "数値の欄を確かめてください")
            return
        self._run("train", cmd, "学習")

    def _open_train_dir(self) -> None:
        d = self._require_model_dir()
        if d is None:
            return
        t = d / "train"
        if not t.is_dir():
            messagebox.showinfo("まだありません", "③で学習すると作られます")
            return
        import subprocess

        opener = "open" if sys.platform == "darwin" else "xdg-open"
        subprocess.Popen([opener, str(t)])

    # ── ④ エクスポート・下書き ──

    def _build_export_tab(self, nb: ttk.Notebook) -> None:
        f = ttk.Frame(nb, padding=10)
        nb.add(f, text="④ エクスポート")
        ttk.Label(f, text="書き出し先:").grid(row=0, column=0, sticky="w")
        ttk.Label(f, textvariable=self.export_out_var, foreground="gray").grid(row=0, column=1, sticky="w")
        ttk.Label(f, text="入力の大きさ:").grid(row=1, column=0, sticky="w", pady=4)
        ttk.Combobox(f, textvariable=self.imgsz_var, width=8, values=(256, 320, 416, 480, 640)).grid(
            row=1, column=1, sticky="w", pady=4)
        ttk.Button(f, text="▶ ONNX に書き出す", command=self._run_export).grid(row=2, column=1, sticky="w", pady=4)
        ttk.Label(f, text="書き出した後、車と同じ読み口（raspi/core/arrow_yolo.py）で読み直して確かめます。\n"
                          "Pi の models/arrow_yolo/ に .onnx と .json を置き、車の GUI の「矢印信号」→「YOLO」で選びます",
                  foreground="gray", justify="left").grid(row=3, column=0, columnspan=3, sticky="w", pady=4)

        ttk.Separator(f).grid(row=4, column=0, columnspan=3, sticky="ew", pady=12)
        ttk.Label(f, text="前のモデルで下書きを付ける（②の手間を減らす）", font=("", 12, "bold")).grid(
            row=5, column=0, columnspan=3, sticky="w")
        ttk.Label(f, text="未確定の画像に、選んだ ONNX の検出を下書き（点線）として付けます。"
                          "②で直して Enter で確定してください",
                  foreground="gray").grid(row=6, column=0, columnspan=3, sticky="w")
        self.prelabel_model_var = tk.StringVar()
        cb = ttk.Combobox(f, textvariable=self.prelabel_model_var, width=24, state="readonly")
        cb.grid(row=7, column=1, sticky="w", pady=4)
        cb["postcommand"] = lambda: cb.configure(values=sorted(
            p.stem for p in MODELS_DIR.glob("*.onnx")) if MODELS_DIR.is_dir() else [])
        ttk.Label(f, text="モデル:").grid(row=7, column=0, sticky="w")
        self.prelabel_conf_var = tk.DoubleVar(value=0.3)
        ttk.Label(f, text="スコア下限:").grid(row=8, column=0, sticky="w")
        ttk.Spinbox(f, from_=0.05, to=0.95, increment=0.05, textvariable=self.prelabel_conf_var,
                    width=8).grid(row=8, column=1, sticky="w")
        self.prelabel_overwrite_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(f, text="既にある下書きも付け直す", variable=self.prelabel_overwrite_var).grid(
            row=9, column=1, sticky="w")
        ttk.Button(f, text="▶ 下書きを付ける", command=self._run_prelabel).grid(row=10, column=1, sticky="w", pady=4)

    def _run_export(self) -> None:
        d = self._require_model_dir()
        if d is None:
            return
        if not RunPaths(d).best.is_file():
            messagebox.showerror("未学習", "③で学習してから書き出してください")
            return
        out = MODELS_DIR / f"{d.name}.onnx"
        if out.exists() and not messagebox.askyesno("上書き", f"{rel(out)} を上書きしますか？"):
            return
        try:
            imgsz = int(self.imgsz_var.get())
        except (tk.TclError, ValueError):
            messagebox.showerror("入力エラー", "入力の大きさを確かめてください")
            return
        self._save_note_quietly(d)
        self._run("export", build_export_cmd(self.python, str(d), str(out), imgsz=imgsz), "エクスポート")

    def _save_note_quietly(self, d: Path) -> None:
        from ml_common.notes import write_note

        write_note(d, self.note_text.get("1.0", "end-1c"))

    def _run_prelabel(self) -> None:
        d = self._require_model_dir()
        name = self.prelabel_model_var.get()
        if d is None:
            return
        if not name:
            messagebox.showerror("入力エラー", "下書きに使うモデルを選んでください")
            return
        if not list_images(RunPaths(d)):
            messagebox.showerror("画像がありません", "①で画像を抽出してください")
            return
        self.label_tab.stop()
        cmd = build_prelabel_cmd(self.python, str(d), str(MODELS_DIR / f"{name}.onnx"),
                                 conf=float(self.prelabel_conf_var.get()),
                                 overwrite=self.prelabel_overwrite_var.get())
        self._run("prelabel", cmd, "下書き")


def main() -> None:
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
