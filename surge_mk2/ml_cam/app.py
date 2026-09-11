"""ml_cam/app.py — アノテーション・学習をターミナル無しで操作するための最小限のGUI。

    .venv/bin/python ml_cam/app.py
    （または `ml_cam/start_app.command` をダブルクリック）

`ml_cam/extract_frames.py`・`ml_cam/annotate.py`・`ml_cam/train.py`・`ml_cam/export_onnx.py`を
サブプロセスとして呼び出すだけの薄い操作パネル。**推論・学習のロジックは
一切持たない**——ここが持つのはボタンとファイル選択ダイアログ、それに
子プロセスの標準出力をログ欄に流し込む配線だけ。中身のスクリプトを直接
書き換えれば、このGUI側は何も変えなくてよい。

ログ表示・サブプロセス実行基盤・run名管理・note.txt・学習曲線グラフ・
ウィンドウクローズ処理は `ml_cam_e2e/app.py`・`ml_lidar/app.py` と共通実装のため
`ml_common/` に切り出してある（2026-09-04）。SAMチェックポイントのダウンロードは
Popenを使わない`urllib`ダウンロードで他2アプリに無い処理のため、共通化せずこのまま。

Tkinter は Python 標準ライブラリ同梱なので、`ml_cam/requirements.txt` に
依存を追加しなくてよい。

## モデル名がそのままフレーム・学習出力先を兼ねる（`ml_lidar/app.py`と対称）

以前は「①フレーム抽出→②アノテーション→③学習→④エクスポート」の各タブで
毎回フォルダを参照・入力する必要があった。1回のフレーム抽出から作る
モデルは基本的に1個なので（バンビの指摘）、**①タブで決めた「モデル名」
1つを②③④タブが自動で使い回す**（`ml_lidar/app.py`の「run名」と同じ考え方）。

    ml_cam/runs/<モデル名>/frames/     … ①の出力・②③の入力
    ml_cam/runs/<モデル名>/best.pt     … ③の出力・④の入力
    ml_cam/runs/<モデル名>/note.txt    … 自由記述の備考（①タブで書ける）
    models/<モデル名>.onnx             … ④の出力（実車のGUIが選ぶ場所）

モデル名は`v1`・`v2`…の続きを自動提案するが、既存の名前を選べば続きから
作業できる（追加のフレーム抽出・再アノテーション・再学習）。
"""

from __future__ import annotations

import queue
import sys
import threading
import tkinter as tk
import urllib.request
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Callable

ML_CAM_DIR = Path(__file__).resolve().parent
REPO_ROOT = ML_CAM_DIR.parent
sys.path.insert(0, str(REPO_ROOT))

from ml_common.close_handler import confirm_and_close  # noqa: E402
from ml_common.dialogs import confirm_overwrite  # noqa: E402
from ml_common.epoch_log import make_epoch_line_parser  # noqa: E402
from ml_common.job_runner import JobRunner  # noqa: E402
from ml_common.log_console import LogConsole  # noqa: E402
from ml_common.naming import list_versioned_names, next_versioned_name  # noqa: E402
from ml_common.notes import read_note, write_note  # noqa: E402
from ml_common.paths import make_rel  # noqa: E402
from ml_common.train_graph import TrainCurveCanvas  # noqa: E402

__all__ = [
    "ML_CAM_DIR", "REPO_ROOT", "RUNS_DIR", "MODELS_DIR",
    "build_extract_cmd", "build_annotate_cmd", "build_train_cmd", "build_export_cmd",
    "build_preview_cmd", "parse_epoch_line",
    "list_model_names", "next_model_name", "describe_model_status",
    "read_note", "write_note",
    "default_sam_checkpoint_path", "download_file", "rel", "App",
]

RUNS_DIR = ML_CAM_DIR / "runs"
#: 実車側（`raspi/nodes/cam_perception_node.py`・`telemetry_node.py`）が
#: `models/<name>.onnx`をフラットに探す前提に合わせる（`ml_lidar`の
#: `models/e2e_lidar/`のようなサブディレクトリは切らない）
MODELS_DIR = REPO_ROOT / "models"
DEFAULT_CHECKPOINT_DIR = ML_CAM_DIR / "checkpoints"
#: これまでのトラブルシューティングで実際に使った SAM のチェックポイント
SAM_CHECKPOINT_URL = "https://dl.fbaipublicfiles.com/segment_anything/sam_vit_b_01ec64.pth"
#: 学習中でも壊れない・止めても惜しくない類のジョブは確認無しで終了時に落とす。
#: 学習だけは数十分〜数時間かかりうるので、ウィンドウを閉じる前に一言確認する
#: （`ml_lidar/app.py`と同じ考え方）
_CONFIRM_STOP_JOB_KEYS = frozenset({"train"})

rel = make_rel(REPO_ROOT)
list_model_names = list_versioned_names
next_model_name = next_versioned_name
parse_epoch_line = make_epoch_line_parser("val_iou")


def default_sam_checkpoint_path() -> Path:
    return DEFAULT_CHECKPOINT_DIR / "sam_vit_b_01ec64.pth"


def describe_model_status(model_dir: Path) -> str:
    """モデル名フィールド脇に出す短い状態表示。フレーム枚数・ラベル付け済み枚数・
    学習済みチェックポイントの有無を要約する。"""
    frames_dir = model_dir / "frames"
    frame_paths = sorted(frames_dir.glob("*.jpg")) if frames_dir.is_dir() else []
    labeled = sum(1 for f in frame_paths if (frames_dir / f"{f.stem}_mask.png").exists())
    mark = "✓学習済み" if (model_dir / "best.pt").exists() else "未学習"
    return f"フレーム {len(frame_paths)}枚（ラベル付け {labeled}枚）・{mark}"


# ── コマンド組み立て（Tkinter を一切知らない純粋関数。`ml_cam/tests/test_app.py` の対象） ──

def build_extract_cmd(python: str, mcap_files: list[str], out_dir: str, cam: str,
                      min_interval_ms: int = 0, target_count: int = 0) -> list[str]:
    """`min_interval_ms`・`target_count` は排他（両方>0なら `target_count` を優先）。"""
    cmd = [python, str(ML_CAM_DIR / "extract_frames.py"), *mcap_files, "--out", out_dir]
    if target_count > 0:
        cmd += ["--target-count", str(target_count)]
    elif min_interval_ms > 0:
        cmd += ["--min-interval-ms", str(min_interval_ms)]
    cmd += ["--cam", cam]
    return cmd


def build_annotate_cmd(python: str, frames_dir: str, checkpoint: str, model_type: str,
                       device: str, skip_labeled: bool, carry_points: bool = True) -> list[str]:
    cmd = [python, str(ML_CAM_DIR / "annotate.py"), frames_dir,
          "--checkpoint", checkpoint, "--model-type", model_type, "--device", device]
    if skip_labeled:
        cmd.append("--skip-labeled")
    if not carry_points:
        cmd.append("--no-carry-points")
    return cmd


def build_train_cmd(python: str, frames_dir: str, out_dir: str, epochs: int,
                    batch_size: int, size: str, no_pretrained: bool) -> list[str]:
    cmd = [python, str(ML_CAM_DIR / "train.py"), "--frames", frames_dir, "--out", out_dir,
          "--epochs", str(epochs), "--batch-size", str(batch_size), "--size", size]
    if no_pretrained:
        cmd.append("--no-pretrained")
    return cmd


def build_export_cmd(python: str, checkpoint: str, out_path: str, size: str) -> list[str]:
    return [python, str(ML_CAM_DIR / "export_onnx.py"), "--checkpoint", checkpoint,
           "--size", size, "--out", out_path]


def build_preview_cmd(python: str, frames_dir: str, model_path: str) -> list[str]:
    return [python, str(ML_CAM_DIR / "preview.py"), frames_dir, "--model", model_path]


def download_file(url: str, dest: Path, *, progress_cb: Callable[[int, int | None], None] | None = None,
                  chunk_size: int = 1 << 16) -> None:
    """`url` を `dest` に保存する（SAM チェックポイントのダウンロード想定）。

    **`.part` に書いてから最後に `rename` する。** 途中で失敗しても壊れた
    ファイルが `dest` の場所に残らないようにするため（残ると「ダウンロード
    済み」と誤判定してそのまま使ってしまう）。

    `pip-system-certs`（このプロジェクトで別途導入済み）により、SSL証明書の
    検証はmacOSのシステム証明書ストアを使う。
    """
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(dest.suffix + ".part")
    with urllib.request.urlopen(url) as resp, open(tmp, "wb") as f:
        total = getattr(resp, "length", None)
        downloaded = 0
        while True:
            chunk = resp.read(chunk_size)
            if not chunk:
                break
            f.write(chunk)
            downloaded += len(chunk)
            if progress_cb:
                progress_cb(downloaded, total)
    tmp.replace(dest)


# ── GUI ──

class App:
    """タブ5枚（抽出・アノテーション・学習・エクスポート・プレビュー）＋共有のログ欄。"""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        root.title("SURGE Mk.2 — 学習データ作成・学習")
        root.geometry("760x680")

        self.python = sys.executable
        self.jobs = JobRunner(REPO_ROOT, allow_concurrent=False, prefix_labels=False)
        #: SAMチェックポイントのダウンロード中フラグ。`self.jobs`はPopenベースの
        #: ジョブしか追跡できないため、排他制御だけ`_is_busy()`で合わせて見る
        self._downloading = False
        self.download_queue: "queue.Queue[tuple]" = queue.Queue()
        #: 直近でエクスポートしたモデル名。完了後に⑤プレビュータブへ引き継ぐために覚えておく
        self._last_exported_model_name = ""

        self._build_widgets()
        self.root.after(100, self._drain_log)
        self.root.protocol("WM_DELETE_WINDOW", lambda: confirm_and_close(
            self.root, self.jobs, confirm_job_keys=_CONFIRM_STOP_JOB_KEYS,
            confirm_message=lambda label: f"{label}が実行中です。終了すると学習が中断されます。"
            "\n\n本当に終了しますか？"))

    def _is_busy(self) -> bool:
        return self.jobs.is_busy() or self._downloading

    # ── 画面構築 ──

    def _build_widgets(self) -> None:
        # ①〜④タブが共有する「モデル名」と、そこから導かれる各パスの表示用変数。
        # `_on_model_name_changed` がモデル名の変更のたびにまとめて更新する
        # （`ml_lidar/app.py`の「run名がそのまま学習出力先とモデル名になる」と同じ考え方）
        self.model_name_var = tk.StringVar(value=next_model_name(list_model_names(RUNS_DIR)))
        self.model_status_var = tk.StringVar(value="")
        self.frames_path_var = tk.StringVar(value="")
        self.model_out_dir_var = tk.StringVar(value="")
        self.checkpoint_path_var = tk.StringVar(value="")
        self.export_out_path_var = tk.StringVar(value="")
        self.preview_frames_var = tk.StringVar(value="")

        nb = ttk.Notebook(self.root)
        nb.pack(fill="x", padx=8, pady=8)
        self._build_extract_tab(nb)
        self._build_annotate_tab(nb)
        self._build_train_tab(nb)
        self._build_export_tab(nb)
        self._build_preview_tab(nb)

        # ウィジェット（note_text等）が全部揃ってからトレースを張り、初期値を反映する
        self.model_name_var.trace_add("write", self._on_model_name_changed)
        self._on_model_name_changed()

        log_frame = ttk.LabelFrame(self.root, text="ログ")
        log_frame.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self.log_console = LogConsole(log_frame, height=16)
        self.log_widget = self.log_console.widget
        self.log_widget.pack(fill="both", expand=True, padx=4, pady=4)

        bottom = ttk.Frame(self.root)
        bottom.pack(fill="x", padx=8, pady=(0, 8))
        self.status_label = ttk.Label(bottom, text="待機中")
        self.status_label.pack(side="left")
        self.stop_btn = ttk.Button(bottom, text="停止", command=self._stop, state="disabled")
        self.stop_btn.pack(side="right")

    def _browse_dir(self, var: tk.StringVar) -> None:
        d = filedialog.askdirectory(initialdir=var.get() or str(REPO_ROOT))
        if d:
            var.set(rel(Path(d)))

    def _browse_file(self, var: tk.StringVar, *, filetypes) -> None:
        f = filedialog.askopenfilename(initialdir=str(REPO_ROOT), filetypes=filetypes)
        if f:
            var.set(rel(Path(f)))

    # ── モデル名（①〜④タブ共有） ──

    def _require_model_dir(self) -> Path | None:
        name = self.model_name_var.get().strip()
        if not name:
            messagebox.showerror("入力エラー", "①タブでモデル名を入力してください")
            return None
        return RUNS_DIR / name

    def _on_model_name_changed(self, *_args) -> None:
        name = self.model_name_var.get().strip()
        if name:
            model_dir = RUNS_DIR / name
            self.frames_path_var.set(rel(model_dir / "frames"))
            self.model_out_dir_var.set(rel(model_dir))
            self.checkpoint_path_var.set(rel(model_dir / "best.pt"))
            self.export_out_path_var.set(rel(MODELS_DIR / f"{name}.onnx"))
            self.preview_frames_var.set(rel(model_dir / "frames"))
            self.model_status_var.set(describe_model_status(model_dir))
            note = read_note(model_dir)
        else:
            self.frames_path_var.set("（モデル名を入力してください）")
            self.model_out_dir_var.set("")
            self.checkpoint_path_var.set("")
            self.export_out_path_var.set("")
            self.model_status_var.set("")
            note = ""
        self.note_text.delete("1.0", "end")
        self.note_text.insert("1.0", note)

    def _save_note(self) -> None:
        model_dir = self._require_model_dir()
        if model_dir is None:
            return
        write_note(model_dir, self.note_text.get("1.0", "end-1c"))
        self._append_log(f"\n備考を保存しました（{self.model_name_var.get().strip()}）\n")

    def _refresh_model_names(self) -> None:
        """モデル名コンボボックスのドロップダウンを開く直前に呼ばれ、既存モデル名で
        一覧を最新化する（`ttk.Combobox`の`postcommand`。新規名を打ち込む分は
        邪魔しない——一覧はあくまで既存モデルを選び直すための候補）。"""
        self.model_combo["values"] = list_model_names(RUNS_DIR)

    def _refresh_exported_models(self) -> None:
        """⑤タブのプレビュー対象モデル一覧（`models/*.onnx`）を最新化する。"""
        names = sorted(p.stem for p in MODELS_DIR.glob("*.onnx")) if MODELS_DIR.is_dir() else []
        self.preview_model_combo["values"] = names

    def _build_extract_tab(self, nb: ttk.Notebook) -> None:
        frame = ttk.Frame(nb, padding=10)
        nb.add(frame, text="① フレーム抽出")

        ttk.Label(frame, text="モデル名:").grid(row=0, column=0, sticky="w")
        self.model_combo = ttk.Combobox(frame, textvariable=self.model_name_var, width=20)
        self.model_combo.grid(row=0, column=1, sticky="w")
        self.model_combo["postcommand"] = self._refresh_model_names
        ttk.Label(frame, text="既存を選べば続きから作業、新しい名前なら新規モデル。\n"
                             "②③④タブはここで決めた名前を自動で使います",
                 foreground="gray").grid(row=1, column=0, columnspan=3, sticky="w")
        ttk.Label(frame, textvariable=self.model_status_var, foreground="gray").grid(
            row=2, column=0, columnspan=3, sticky="w", pady=(2, 0))

        ttk.Label(frame, text="フレーム出力先:").grid(row=3, column=0, sticky="w", pady=(8, 0))
        ttk.Label(frame, textvariable=self.frames_path_var, foreground="gray").grid(
            row=3, column=1, columnspan=2, sticky="w", pady=(8, 0))

        self._extract_files: list[str] = []
        files_var = tk.StringVar(value="(.mcap 未選択)")

        def pick_files() -> None:
            paths = filedialog.askopenfilenames(
                title="録画した .mcap を選ぶ（複数可）", filetypes=[("MCAP", "*.mcap")])
            if paths:
                self._extract_files = list(paths)
                names = ", ".join(Path(p).name for p in paths[:3])
                more = "…" if len(paths) > 3 else ""
                files_var.set(f"{len(paths)}個: {names}{more}")

        ttk.Button(frame, text="録画(.mcap)を選ぶ", command=pick_files).grid(
            row=4, column=0, sticky="w", pady=(8, 0))
        ttk.Label(frame, textvariable=files_var, wraplength=500).grid(
            row=4, column=1, columnspan=2, sticky="w", pady=(8, 0))

        cam_var = tk.StringVar(value="front")
        ttk.Label(frame, text="カメラ:").grid(row=5, column=0, sticky="w", pady=(8, 0))
        ttk.Combobox(frame, textvariable=cam_var, values=["front", "rear", "both"],
                    state="readonly", width=10).grid(row=5, column=1, sticky="w", pady=(8, 0))

        thin_mode = tk.StringVar(value="interval")
        ttk.Label(frame, text="間引き方法:").grid(row=6, column=0, sticky="w", pady=(8, 0))
        mode_frame = ttk.Frame(frame)
        mode_frame.grid(row=6, column=1, columnspan=2, sticky="w", pady=(8, 0))
        ttk.Radiobutton(mode_frame, text="間隔(ms)", variable=thin_mode,
                        value="interval").pack(side="left")
        ttk.Radiobutton(mode_frame, text="合計枚数", variable=thin_mode,
                        value="count").pack(side="left", padx=(10, 0))

        interval_var = tk.StringVar(value="500")
        ttk.Label(frame, text="間引き間隔(ms):").grid(row=7, column=0, sticky="w", pady=(4, 0))
        ttk.Entry(frame, textvariable=interval_var, width=10).grid(row=7, column=1, sticky="w", pady=(4, 0))

        count_var = tk.StringVar(value="500")
        ttk.Label(frame, text="合計枚数:").grid(row=8, column=0, sticky="w", pady=(4, 0))
        ttk.Entry(frame, textvariable=count_var, width=10).grid(row=8, column=1, sticky="w", pady=(4, 0))

        ttk.Label(frame, text="間隔(ms): 0で間引きなし。連続フレームはほぼ同じ構図なので\n"
                             "間引くほどアノテーションの手間が減る。\n"
                             "合計枚数: 選んだ全 .mcap・全カメラを通して、指定枚数に近づくよう\n"
                             "均等に間引く（複数ファイルでも合計でこの枚数程度になる）\n"
                             "同じモデル名で再実行すると、既存のフレームに追加されます。",
                 foreground="gray").grid(row=9, column=0, columnspan=3, sticky="w")

        def run() -> None:
            model_dir = self._require_model_dir()
            if model_dir is None:
                return
            if not self._extract_files:
                messagebox.showwarning("未選択", ".mcap ファイルを選んでください")
                return
            interval = 0
            count = 0
            if thin_mode.get() == "count":
                try:
                    count = int(count_var.get())
                except ValueError:
                    messagebox.showerror("入力エラー", "合計枚数は整数で入力してください")
                    return
            else:
                try:
                    interval = int(interval_var.get())
                except ValueError:
                    messagebox.showerror("入力エラー", "間引き間隔は整数で入力してください")
                    return
            cmd = build_extract_cmd(self.python, self._extract_files, str(model_dir / "frames"),
                                    cam_var.get(), interval, count)
            self._run("extract", cmd, "フレーム抽出")

        ttk.Button(frame, text="抽出実行", command=run).grid(row=10, column=0, sticky="w", pady=10)

        note_frame = ttk.LabelFrame(frame, text="備考（どんな変更をしたか・どのデータか等、自由に）")
        note_frame.grid(row=11, column=0, columnspan=3, sticky="we", pady=(4, 0))
        self.note_text = tk.Text(note_frame, height=3, wrap="word")
        self.note_text.pack(fill="both", expand=True, padx=4, pady=(4, 0))
        ttk.Button(note_frame, text="備考を保存", command=self._save_note).pack(
            anchor="e", padx=4, pady=4)

    def _build_annotate_tab(self, nb: ttk.Notebook) -> None:
        frame = ttk.Frame(nb, padding=10)
        nb.add(frame, text="② アノテーション")

        ttk.Label(frame, text="モデル名:").grid(row=0, column=0, sticky="w")
        ttk.Label(frame, textvariable=self.model_name_var, foreground="gray").grid(
            row=0, column=1, sticky="w")
        ttk.Label(frame, text="フレーム:").grid(row=1, column=0, sticky="w", pady=(4, 0))
        ttk.Label(frame, textvariable=self.frames_path_var, foreground="gray").grid(
            row=1, column=1, columnspan=2, sticky="w", pady=(4, 0))

        ckpt_var = tk.StringVar(value=rel(default_sam_checkpoint_path()))
        ttk.Label(frame, text="SAM チェックポイント:").grid(row=2, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(frame, textvariable=ckpt_var, width=45).grid(row=2, column=1, sticky="w", pady=(8, 0))
        ttk.Button(frame, text="参照",
                  command=lambda: self._browse_file(ckpt_var, filetypes=[("PyTorch checkpoint", "*.pth")])
                  ).grid(row=2, column=2, pady=(8, 0))

        def download_checkpoint() -> None:
            dest = REPO_ROOT / ckpt_var.get()
            if dest.exists() and not messagebox.askyesno(
                    "上書き確認", f"{ckpt_var.get()} は既にあります。再ダウンロードしますか？"):
                return
            self._download_sam_checkpoint(dest)

        ttk.Button(frame, text="↓ ダウンロード（初回のみ・数百MB）", command=download_checkpoint).grid(
            row=3, column=1, sticky="w", pady=(4, 0))

        model_type_var = tk.StringVar(value="vit_b")
        ttk.Label(frame, text="model-type:").grid(row=4, column=0, sticky="w", pady=(8, 0))
        ttk.Combobox(frame, textvariable=model_type_var, values=["vit_b", "vit_l", "vit_h", "default"],
                    state="readonly", width=10).grid(row=4, column=1, sticky="w", pady=(8, 0))

        device_var = tk.StringVar(value="cpu")
        ttk.Label(frame, text="device:").grid(row=5, column=0, sticky="w", pady=(8, 0))
        ttk.Combobox(frame, textvariable=device_var, values=["cpu", "mps"],
                    state="readonly", width=10).grid(row=5, column=1, sticky="w", pady=(8, 0))

        skip_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(frame, text="ラベル付け済みフレームは飛ばす",
                        variable=skip_var).grid(row=6, column=0, columnspan=2, sticky="w", pady=(8, 0))

        carry_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(frame, text="前フレームの点を引き継ぐ（推奨。Enterだけで進めやすくなる）",
                        variable=carry_var).grid(row=7, column=0, columnspan=2, sticky="w")

        def run() -> None:
            model_dir = self._require_model_dir()
            if model_dir is None:
                return
            cmd = build_annotate_cmd(self.python, str(model_dir / "frames"), ckpt_var.get(),
                                     model_type_var.get(), device_var.get(), skip_var.get(),
                                     carry_var.get())
            self._run("annotate", cmd, "アノテーション")

        ttk.Button(frame, text="アノテーション開始（別ウィンドウが開きます）", command=run).grid(
            row=8, column=0, columnspan=2, sticky="w", pady=10)

        ttk.Label(frame, text="操作（別ウィンドウ内）:\n"
                             "  左クリック         走行可能（前景）の点を追加\n"
                             "  Shift + 左クリック   除外（背景）の点を追加\n"
                             "  z                  直前の点を取り消す\n"
                             "  Enter              マスクを保存して次のフレームへ\n"
                             "  n / p              保存せず次/前のフレームへ\n"
                             "  q                  終了",
                 foreground="gray", justify="left").grid(
            row=9, column=0, columnspan=3, sticky="w")

    def _build_train_tab(self, nb: ttk.Notebook) -> None:
        frame = ttk.Frame(nb, padding=10)
        nb.add(frame, text="③ 学習")

        ttk.Label(frame, text="モデル名:").grid(row=0, column=0, sticky="w")
        ttk.Label(frame, textvariable=self.model_name_var, foreground="gray").grid(
            row=0, column=1, sticky="w")
        ttk.Label(frame, text="フレーム:").grid(row=1, column=0, sticky="w", pady=(4, 0))
        ttk.Label(frame, textvariable=self.frames_path_var, foreground="gray").grid(
            row=1, column=1, columnspan=2, sticky="w", pady=(4, 0))
        ttk.Label(frame, text="出力先:").grid(row=2, column=0, sticky="w", pady=(4, 0))
        ttk.Label(frame, textvariable=self.model_out_dir_var, foreground="gray").grid(
            row=2, column=1, columnspan=2, sticky="w", pady=(4, 0))

        epochs_var = tk.StringVar(value="30")
        ttk.Label(frame, text="エポック数:").grid(row=3, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(frame, textvariable=epochs_var, width=10).grid(row=3, column=1, sticky="w", pady=(8, 0))

        batch_var = tk.StringVar(value="8")
        ttk.Label(frame, text="バッチサイズ:").grid(row=4, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(frame, textvariable=batch_var, width=10).grid(row=4, column=1, sticky="w", pady=(8, 0))

        size_var = tk.StringVar(value="224x224")
        ttk.Label(frame, text="入力解像度:").grid(row=5, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(frame, textvariable=size_var, width=10).grid(row=5, column=1, sticky="w", pady=(8, 0))

        no_pretrained_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(frame, text="事前学習重みを使わない（オフライン環境向け）",
                        variable=no_pretrained_var).grid(row=6, column=0, columnspan=2, sticky="w", pady=(8, 0))

        def on_epoch_line(line: str) -> None:
            parsed = parse_epoch_line(line)
            if parsed is None:
                return
            epoch, loss, iou = parsed
            self.train_graph.add_point(epoch, loss, iou)

        def run() -> None:
            model_dir = self._require_model_dir()
            if model_dir is None:
                return
            try:
                epochs = int(epochs_var.get())
                batch = int(batch_var.get())
            except ValueError:
                messagebox.showerror("入力エラー", "エポック数・バッチサイズは整数で入力してください")
                return
            self.train_graph.reset()
            cmd = build_train_cmd(self.python, str(model_dir / "frames"), str(model_dir), epochs,
                                  batch, size_var.get(), no_pretrained_var.get())
            self._run("train", cmd, "学習", on_line=on_epoch_line)

        ttk.Button(frame, text="学習開始", command=run).grid(row=7, column=0, sticky="w", pady=10)

        graph_frame = ttk.LabelFrame(frame, text="学習曲線（赤=loss・青=val_iou）")
        graph_frame.grid(row=8, column=0, columnspan=3, sticky="we", pady=(4, 0))
        self.train_graph = TrainCurveCanvas(graph_frame, metric_label="val_iou")
        self.train_graph.widget.pack(padx=4, pady=4)

    def _build_export_tab(self, nb: ttk.Notebook) -> None:
        frame = ttk.Frame(nb, padding=10)
        nb.add(frame, text="④ エクスポート")

        ttk.Label(frame, text="モデル名:").grid(row=0, column=0, sticky="w")
        ttk.Label(frame, textvariable=self.model_name_var, foreground="gray").grid(
            row=0, column=1, sticky="w")
        ttk.Label(frame, text="チェックポイント:").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Label(frame, textvariable=self.checkpoint_path_var, foreground="gray").grid(
            row=1, column=1, columnspan=2, sticky="w", pady=(8, 0))

        size_var = tk.StringVar(value="224x224")
        ttk.Label(frame, text="入力解像度:").grid(row=2, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(frame, textvariable=size_var, width=10).grid(row=2, column=1, sticky="w", pady=(8, 0))
        ttk.Label(frame, text="③学習タブと同じ値にしてください", foreground="gray").grid(
            row=3, column=0, columnspan=3, sticky="w")

        ttk.Label(frame, text="出力先:").grid(row=4, column=0, sticky="w", pady=(8, 0))
        ttk.Label(frame, textvariable=self.export_out_path_var, foreground="gray").grid(
            row=4, column=1, columnspan=2, sticky="w", pady=(8, 0))

        def run() -> None:
            model_dir = self._require_model_dir()
            if model_dir is None:
                return
            name = self.model_name_var.get().strip()
            out_path = MODELS_DIR / f"{name}.onnx"
            if not confirm_overwrite(out_path, rel):
                return
            self._last_exported_model_name = name
            cmd = build_export_cmd(self.python, str(model_dir / "best.pt"), str(out_path),
                                   size_var.get())
            self._run("export", cmd, "エクスポート")

        ttk.Button(frame, text="エクスポート実行", command=run).grid(row=5, column=0, sticky="w", pady=10)

    def _build_preview_tab(self, nb: ttk.Notebook) -> None:
        frame = ttk.Frame(nb, padding=10)
        nb.add(frame, text="⑤ プレビュー")

        ttk.Label(frame, text="実車に乗せる前に、モデルの推論結果を確認します。\n"
                             "マスク付きフレームがあれば正解との差分も色分け表示します\n"
                             "（緑=一致・青=過検出・赤=見落とし）。",
                 foreground="gray").grid(row=0, column=0, columnspan=3, sticky="w")

        ttk.Label(frame, text="フレーム:").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(frame, textvariable=self.preview_frames_var, width=45).grid(
            row=1, column=1, sticky="w", pady=(8, 0))
        ttk.Button(frame, text="参照", command=lambda: self._browse_dir(self.preview_frames_var)).grid(
            row=1, column=2, pady=(8, 0))

        # ④でエクスポート済みのモデル（`models/*.onnx`）から選ぶドロップダウン。
        # パスを手打ち・参照する必要を無くす（2026-08-29、バンビの要望。`ml_lidar/app.py`
        # のrun一覧ドロップダウンと同じ考え方）
        self.preview_model_var = tk.StringVar(value="")
        ttk.Label(frame, text="モデル:").grid(row=2, column=0, sticky="w", pady=(8, 0))
        self.preview_model_combo = ttk.Combobox(frame, textvariable=self.preview_model_var,
                                                state="readonly", width=42)
        self.preview_model_combo.grid(row=2, column=1, columnspan=2, sticky="w", pady=(8, 0))
        self.preview_model_combo["postcommand"] = self._refresh_exported_models
        self._refresh_exported_models()
        if self.model_name_var.get().strip() in self.preview_model_combo["values"]:
            self.preview_model_var.set(self.model_name_var.get().strip())

        def run() -> None:
            name = self.preview_model_var.get().strip()
            if not name:
                messagebox.showwarning("未選択", "モデルを選んでください（④でエクスポートが必要です）")
                return
            cmd = build_preview_cmd(self.python, self.preview_frames_var.get(),
                                    str(MODELS_DIR / f"{name}.onnx"))
            self._run("preview", cmd, "プレビュー")

        ttk.Button(frame, text="プレビュー開始（別ウィンドウが開きます）", command=run).grid(
            row=3, column=0, columnspan=2, sticky="w", pady=10)

    # ── サブプロセス実行・ログ配線 ──

    def _append_log(self, text: str) -> None:
        self.log_console.append(text)

    def _run(self, job_key: str, cmd: list[str], label: str, *,
            on_line: Callable[[str], None] | None = None) -> None:
        if self._is_busy():
            messagebox.showwarning("実行中", "他の処理が終わってから実行してください")
            return
        self.status_label.config(text=f"{label} 実行中…")
        self.stop_btn.config(state="normal")
        self._append_log(f"\n$ {' '.join(cmd)}\n")
        self.jobs.start(job_key, cmd, label, on_line=on_line,
                        on_done=lambda code: self._on_job_done(label, code))

    def _on_job_done(self, label: str, code: int) -> None:
        ok = code == 0
        if ok and label in ("フレーム抽出", "学習"):
            # モデルの状態（フレーム枚数・学習済みか）が変わったので更新する
            self.model_status_var.set(
                describe_model_status(RUNS_DIR / self.model_name_var.get().strip()))
        if ok and label == "エクスポート":
            # ⑤タブのプレビュー対象に、今エクスポートしたモデルを選んでおく
            self._refresh_exported_models()
            self.preview_model_var.set(self._last_exported_model_name)
        self.status_label.config(text="待機中")
        self.stop_btn.config(state="disabled")

    def _download_sam_checkpoint(self, dest: Path) -> None:
        if self._is_busy():
            messagebox.showwarning("実行中", "他の処理が終わってから実行してください")
            return
        self._downloading = True
        self.status_label.config(text="SAM チェックポイントをダウンロード中…")
        self.stop_btn.config(state="disabled")           # ダウンロードは中断させない（壊れたファイルが残るため）
        self._append_log(f"\nダウンロード中: {SAM_CHECKPOINT_URL} → {rel(dest)}\n")

        def worker() -> None:
            last_reported = -1
            try:
                def progress(downloaded: int, total: int | None) -> None:
                    nonlocal last_reported
                    mb = downloaded / 1e6
                    step = int(mb // 20)                  # 20MBごとに1回だけログを出す
                    if step != last_reported:
                        last_reported = step
                        if total:
                            self.download_queue.put(("log", f"  {mb:.0f}MB / {total / 1e6:.0f}MB\n"))
                        else:
                            self.download_queue.put(("log", f"  {mb:.0f}MB\n"))

                download_file(SAM_CHECKPOINT_URL, dest, progress_cb=progress)
                self.download_queue.put(("log", "ダウンロード完了\n"))
                self.download_queue.put(("done", "SAMダウンロード", 0))
            except Exception as e:                        # noqa: BLE001
                self.download_queue.put(("log", f"ダウンロード失敗: {e}\n"))
                self.download_queue.put(("done", "SAMダウンロード", 1))

        threading.Thread(target=worker, daemon=True).start()

    def _stop(self) -> None:
        active = self.jobs.active_jobs()
        if active and self.jobs.stop(active[0][0]):
            self._append_log("\n（停止を指示しました）\n")

    def _drain_download_queue(self) -> None:
        try:
            while True:
                kind, *rest = self.download_queue.get_nowait()
                if kind == "log":
                    self._append_log(rest[0])
                elif kind == "done":
                    label, code = rest
                    ok = code == 0
                    self._append_log(f"\n[{label}] {'完了' if ok else f'終了コード {code}'}\n")
                    self._downloading = False
                    self.status_label.config(text="待機中")
        except queue.Empty:
            pass

    def _drain_log(self) -> None:
        self.jobs.drain(self._append_log)
        self._drain_download_queue()
        self.root.after(100, self._drain_log)


def main() -> int:
    root = tk.Tk()
    App(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
