"""ml_cam_e2e/app.py — カメラE2E（模倣学習）のデータ作成・選別・学習・評価をターミナル無しで行う操作パネル。

    .venv/bin/python ml_cam_e2e/app.py
    （または `launcher.command`（surge_mk2 直下） から起動）

タブは作業の順に6枚:

    ① ペア抽出    録った .mcap から (画像, 人の操作) の組を取り出す
    ② 確認・選別  映像と操作を見て、手本にしない区間を除外する
    ③ 偏り        舵・速度の分布を見て、直進過多の補正を決める
    ④ 学習
    ⑤ エクスポート  実車が読む ONNX と契約ファイルを書く
    ⑥ 評価        モデルの予測を手本と並べ、誤差の大きい場面を見る

①④⑤⑥の重い処理は `extract_pairs.py`・`train.py`・`export_onnx.py`・
`eval_model.py` をサブプロセスとして呼び出すだけ（`ml_cam/app.py` と同じ設計方針。
このプロセスは torch を import しない）。②③⑥の表示は `review_tab.py`・
`balance_tab.py`・`eval_tab.py` に分けてあり、どれを学習に使うかの判断は
`samples.py`（学習スクリプトと共有）に任せる。

ログ表示・サブプロセス実行基盤・run名管理・note.txt・学習曲線グラフ・
ウィンドウクローズ処理は `ml_cam/app.py`・`ml_lidar/app.py` と共通実装のため
`ml_common/` に切り出してある（2026-09-04）。

## セグメンテーション（`ml_cam/app.py`）と違い、アノテーションタブが無い

教師データは録画に既に入っている人間の操作そのもの——ラベル付け作業が
丸ごと不要。その代わり「悪い手本を外す」作業（②）が品質を決める。

## モデル名がそのままフレーム・学習出力先を兼ねる（`ml_cam/app.py`と同じ考え方）

    ml_cam_e2e/runs/<モデル名>/frames/            … ①の出力・②③④⑥の入力
    ml_cam_e2e/runs/<モデル名>/frames/exclusions.json … ②で付けた除外
    ml_cam_e2e/runs/<モデル名>/best.pt            … ④の出力・⑤の入力
    ml_cam_e2e/runs/<モデル名>/eval.csv           … ⑥の出力
    ml_cam_e2e/runs/<モデル名>/note.txt           … 自由記述の備考（①タブで書ける）
    models/cam_e2e/<モデル名>.onnx                … ⑤の出力（実車のGUIが選ぶ場所）
"""

from __future__ import annotations

import sys
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Callable

ML_CAM_E2E_DIR = Path(__file__).resolve().parent
REPO_ROOT = ML_CAM_E2E_DIR.parent
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

from balance_tab import BalanceTab  # noqa: E402
from eval_tab import EvalTab  # noqa: E402
from raspi.core.vehicle import Vehicle  # noqa: E402
from review_tab import ReviewTab  # noqa: E402

__all__ = [
    "ML_CAM_E2E_DIR", "REPO_ROOT", "RUNS_DIR", "MODELS_DIR",
    "build_extract_cmd", "build_train_cmd", "build_export_cmd", "build_eval_cmd",
    "extract_warning",
    "parse_epoch_line",
    "list_model_names", "next_model_name", "describe_model_status",
    "read_note", "write_note", "rel", "App",
]

RUNS_DIR = ML_CAM_E2E_DIR / "runs"
#: 実車側（`cam_e2e_node.py` の `DEFAULT_MODELS_DIR`）と同じ場所。セグメンテーション用
#: （`models/` 直下）と混ざらないよう分けてある
MODELS_DIR = REPO_ROOT / "models" / "cam_e2e"
#: 学習中でも壊れない・止めても惜しくない類のジョブは確認無しで終了時に落とす。
#: 学習だけは数十分〜数時間かかりうるので、ウィンドウを閉じる前に一言確認する
#: （`ml_lidar/app.py`と同じ考え方）
_CONFIRM_STOP_JOB_KEYS = frozenset({"train"})

rel = make_rel(REPO_ROOT)
list_model_names = list_versioned_names
next_model_name = next_versioned_name
parse_epoch_line = make_epoch_line_parser("val_mae")


def describe_model_status(model_dir: Path) -> str:
    frames_dir = model_dir / "frames"
    n = len(list(frames_dir.glob("*.jpg"))) if frames_dir.is_dir() else 0
    mark = "✓学習済み" if (model_dir / "best.pt").exists() else "未学習"
    return f"ペア {n}件・{mark}"


# ── コマンド組み立て（Tkinter を一切知らない純粋関数） ──

def build_extract_cmd(python: str, mcap_files: list[str], out_dir: str, cam: str,
                      max_gap_ms: int, min_interval_ms: int = 0,
                      target_count: int = 0, label_shift_ms: int = 0,
                      include_auto: bool = False) -> list[str]:
    cmd = [python, str(ML_CAM_E2E_DIR / "extract_pairs.py"), *mcap_files, "--out", out_dir,
          "--cam", cam, "--max-gap-ms", str(max_gap_ms)]
    if include_auto:
        cmd.append("--include-auto")
    if label_shift_ms:
        cmd += ["--label-shift-ms", str(label_shift_ms)]
    if target_count > 0:
        cmd += ["--target-count", str(target_count)]
    elif min_interval_ms > 0:
        cmd += ["--min-interval-ms", str(min_interval_ms)]
    return cmd


def extract_warning(lines: list[str]) -> str | None:
    """`extract_pairs.py` の出力から、0枚で終わったときに見せる文面を作る。
    1枚でも書けていれば `None`。"""
    total = next((line for line in reversed(lines) if line.startswith("# 合計 ")), None)
    if total is None or not total.startswith("# 合計 0枚"):
        return None
    reasons = next((line[2:].strip() for line in lines
                    if line.startswith("# 手本にしなかったフレーム")), "")
    skipped = [line[2:].strip() for line in lines if line.startswith("# skip:")]
    parts = ["選んだ記録から、手本に使えるフレームが1枚も取れませんでした。"]
    if reasons:
        parts.append(reasons)
    if "自動運転中" in reasons:
        parts.append("自動運転（slam2d_route など）の走行を手本にするなら、"
                     "「自動運転中の走行も手本にする」にチェックを入れて、もう一度実行してください。")
    if "トルクモード" in reasons:
        parts.append("トルクモードの走行は速度の手本になりません。速度モードで録ってください。")
    if skipped:
        parts.append("\n".join(skipped))
    if not reasons and not skipped:
        parts.append("記録に前カメラの画像が入っていない可能性があります"
                     "（画像つきで録ったか確認してください）。")
    return "\n\n".join(parts)


def build_train_cmd(python: str, frames_dir: str, out_dir: str, epochs: int,
                    batch_size: int, size: str, no_pretrained: bool, *,
                    balance: float = 0.5, speed_weight: float = 0.5,
                    no_flip: bool = False) -> list[str]:
    cmd = [python, str(ML_CAM_E2E_DIR / "train.py"), "--frames", frames_dir, "--out", out_dir,
          "--epochs", str(epochs), "--batch-size", str(batch_size), "--size", size,
          "--balance", f"{balance:.2f}", "--speed-weight", f"{speed_weight:g}"]
    if no_flip:
        cmd.append("--no-flip")
    if no_pretrained:
        cmd.append("--no-pretrained")
    return cmd


def build_export_cmd(python: str, checkpoint: str, out_path: str) -> list[str]:
    """解像度と正規化の基準は `train_config.json` から読まれるので渡さない。"""
    return [python, str(ML_CAM_E2E_DIR / "export_onnx.py"), "--checkpoint", checkpoint,
           "--out", out_path]


def build_eval_cmd(python: str, frames_dir: str, run_dir: str, model_path: str) -> list[str]:
    return [python, str(ML_CAM_E2E_DIR / "eval_model.py"), "--frames", frames_dir,
           "--run", run_dir, "--model", model_path]


# ── GUI ──

class App:
    """タブ6枚（抽出・確認/選別・偏り・学習・エクスポート・評価）＋共有のログ欄。"""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        root.title("SURGE Mk.2 — カメラE2E（模倣学習）")
        root.geometry("1010x860")

        self.python = sys.executable
        self.jobs = JobRunner(REPO_ROOT, allow_concurrent=False, prefix_labels=False)
        self._last_exported_model_name = ""
        self.max_steer = Vehicle.load().max_steer

        self._build_widgets()
        self.root.after(100, self._drain_log)
        self.root.protocol("WM_DELETE_WINDOW", lambda: confirm_and_close(
            self.root, self.jobs, confirm_job_keys=_CONFIRM_STOP_JOB_KEYS,
            confirm_message=lambda label: f"{label}が実行中です。終了すると学習が中断されます。"
            "\n\n本当に終了しますか？"))

    # ── 画面構築 ──

    def _build_widgets(self) -> None:
        self.model_name_var = tk.StringVar(value=next_model_name(list_model_names(RUNS_DIR)))
        self.model_status_var = tk.StringVar(value="")
        self.frames_path_var = tk.StringVar(value="")
        self.model_out_dir_var = tk.StringVar(value="")
        self.checkpoint_path_var = tk.StringVar(value="")
        self.export_out_path_var = tk.StringVar(value="")
        #: 直進過多の補正（0〜1）。③偏りタブのスライダと④学習タブが同じ値を見る
        self.balance_var = tk.DoubleVar(value=0.5)

        # 下から先に置く。ログ欄は固定の高さにして、残りをタブに渡す
        bottom = ttk.Frame(self.root)
        bottom.pack(side="bottom", fill="x", padx=8, pady=(0, 8))
        self.status_label = ttk.Label(bottom, text="待機中")
        self.status_label.pack(side="left")
        self.stop_btn = ttk.Button(bottom, text="停止", command=self._stop, state="disabled")
        self.stop_btn.pack(side="right")

        log_frame = ttk.LabelFrame(self.root, text="ログ")
        log_frame.pack(side="bottom", fill="x", padx=8, pady=(0, 8))
        self.log_console = LogConsole(log_frame, height=7)
        self.log_widget = self.log_console.widget
        self.log_widget.pack(fill="both", expand=True, padx=4, pady=4)

        nb = ttk.Notebook(self.root)
        nb.pack(side="top", fill="both", expand=True, padx=8, pady=8)
        self.notebook = nb
        self._build_extract_tab(nb)
        self.review_tab = ReviewTab(nb, get_frames_dir=self._frames_dir_or_none,
                                    max_steer=self.max_steer, log=self._append_log)
        nb.add(self.review_tab.frame, text="② 確認・選別")
        self.balance_tab = BalanceTab(nb, get_frames_dir=self._frames_dir_or_none,
                                      max_steer=self.max_steer, balance_var=self.balance_var)
        nb.add(self.balance_tab.frame, text="③ 偏り")
        self._build_train_tab(nb)
        self._build_export_tab(nb)
        self.eval_tab = EvalTab(nb, get_frames_dir=self._frames_dir_or_none,
                                get_run_dir=self._run_dir_or_none, models_dir=MODELS_DIR,
                                run_eval=self._run_eval, open_in_review=self._open_in_review)
        nb.add(self.eval_tab.frame, text="⑥ 評価")
        nb.bind("<<NotebookTabChanged>>", self._on_tab_changed)
        self.root.bind("<Key>", self._on_key)

        self.model_name_var.trace_add("write", self._on_model_name_changed)
        self._on_model_name_changed()

    # ── タブ間の配線 ──

    def _run_dir_or_none(self) -> Path | None:
        name = self.model_name_var.get().strip()
        return RUNS_DIR / name if name else None

    def _frames_dir_or_none(self) -> Path | None:
        run_dir = self._run_dir_or_none()
        return run_dir / "frames" if run_dir else None

    def _current_tab(self):
        """表に出ているタブの持ち主（②③⑥のどれか）。それ以外なら `None`。"""
        current = self.notebook.select()
        for tab in (self.review_tab, self.balance_tab, self.eval_tab):
            if str(tab.frame) == current:
                return tab
        return None

    def _on_tab_changed(self, _event=None) -> None:
        # **開くたびに読み直す。** ①で抽出を足した・②で除外を変えた直後に、
        # 古い内容のまま判断させない（数千枚の manifest でも一瞬で読める）
        tab = self._current_tab()
        if tab is not self.review_tab:
            self.review_tab.stop()
        if tab is self.review_tab or tab is self.balance_tab:
            tab.reload()
        elif tab is self.eval_tab:
            self.eval_tab.select_model(self.model_name_var.get().strip())
            self.eval_tab.load_results()

    def _on_key(self, event):
        # 入力欄で文字を打っている間は奪わない（メモ欄の "x" で除外が走らないように）
        cls = event.widget.winfo_class()
        if cls in ("TEntry", "Entry", "Text", "TCombobox"):
            return None
        # ボタンにフォーカスがあるときの space はそのボタンが自分で処理する。
        # ここでも拾うと「再生ボタンが押される＋再生/停止の切り替え」で2回反転して何も起きない
        if cls == "TButton" and event.keysym == "space":
            return None
        tab = self._current_tab()
        if tab is not None and hasattr(tab, "handle_key") and tab.handle_key(event):
            return "break"
        return None

    def _open_in_review(self, source_mcap: str, cam: str, t_ns: int) -> None:
        self.notebook.select(self.review_tab.frame)
        self.review_tab.open_at(source_mcap, cam, t_ns)

    def _run_eval(self, model_path: Path, on_done: Callable[[bool], None]) -> None:
        model_dir = self._require_model_dir()
        if model_dir is None:
            return
        cmd = build_eval_cmd(self.python, str(model_dir / "frames"), str(model_dir),
                             str(model_path))
        self._run("eval", cmd, "評価", on_done=on_done)

    def _browse_dir(self, var: tk.StringVar) -> None:
        d = filedialog.askdirectory(initialdir=var.get() or str(REPO_ROOT))
        if d:
            var.set(rel(Path(d)))

    # ── モデル名（①〜③タブ共有） ──

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
        # 表に出ているタブが②③⑥なら、新しいモデルの内容に差し替える
        self._on_tab_changed()

    def _save_note(self) -> None:
        model_dir = self._require_model_dir()
        if model_dir is None:
            return
        write_note(model_dir, self.note_text.get("1.0", "end-1c"))
        self._append_log(f"\n備考を保存しました（{self.model_name_var.get().strip()}）\n")

    def _refresh_model_names(self) -> None:
        self.model_combo["values"] = list_model_names(RUNS_DIR)

    def _build_extract_tab(self, nb: ttk.Notebook) -> None:
        frame = ttk.Frame(nb, padding=10)
        nb.add(frame, text="① ペア抽出")

        ttk.Label(frame, text="モデル名:").grid(row=0, column=0, sticky="w")
        self.model_combo = ttk.Combobox(frame, textvariable=self.model_name_var, width=20)
        self.model_combo.grid(row=0, column=1, sticky="w")
        self.model_combo["postcommand"] = self._refresh_model_names
        ttk.Label(frame, text="既存を選べば続きから作業、新しい名前なら新規モデル。\n"
                             "②〜⑥タブはここで決めた名前を自動で使います",
                 foreground="gray").grid(row=1, column=0, columnspan=3, sticky="w")
        ttk.Label(frame, textvariable=self.model_status_var, foreground="gray").grid(
            row=2, column=0, columnspan=3, sticky="w", pady=(2, 0))

        ttk.Label(frame, text="出力先:").grid(row=3, column=0, sticky="w", pady=(8, 0))
        ttk.Label(frame, textvariable=self.frames_path_var, foreground="gray").grid(
            row=3, column=1, columnspan=2, sticky="w", pady=(8, 0))

        self._extract_files: list[str] = []
        files_var = tk.StringVar(value="(.mcap 未選択)")

        def pick_files() -> None:
            paths = filedialog.askopenfilenames(
                title="録画した .mcap を選ぶ（複数可、手動運転区間のもの）",
                filetypes=[("MCAP", "*.mcap")])
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

        gap_var = tk.StringVar(value="100")
        ttk.Label(frame, text="操舵指令との時刻差の許容[ms]:").grid(row=6, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(frame, textvariable=gap_var, width=10).grid(row=6, column=1, sticky="w", pady=(8, 0))

        shift_var = tk.StringVar(value="0")
        ttk.Label(frame, text="手本を遅らせる[ms]:").grid(row=6, column=2, sticky="e", pady=(8, 0))
        ttk.Entry(frame, textvariable=shift_var, width=8).grid(row=6, column=3, sticky="w",
                                                               padx=(4, 0), pady=(8, 0))

        thin_mode = tk.StringVar(value="interval")
        ttk.Label(frame, text="間引き方法:").grid(row=7, column=0, sticky="w", pady=(8, 0))
        mode_frame = ttk.Frame(frame)
        mode_frame.grid(row=7, column=1, columnspan=2, sticky="w", pady=(8, 0))
        ttk.Radiobutton(mode_frame, text="間隔(ms)", variable=thin_mode,
                        value="interval").pack(side="left")
        ttk.Radiobutton(mode_frame, text="合計件数", variable=thin_mode,
                        value="count").pack(side="left", padx=(10, 0))

        interval_var = tk.StringVar(value="0")
        ttk.Label(frame, text="間引き間隔(ms):").grid(row=8, column=0, sticky="w", pady=(4, 0))
        ttk.Entry(frame, textvariable=interval_var, width=10).grid(row=8, column=1, sticky="w", pady=(4, 0))

        count_var = tk.StringVar(value="3000")
        ttk.Label(frame, text="合計件数:").grid(row=9, column=0, sticky="w", pady=(4, 0))
        ttk.Entry(frame, textvariable=count_var, width=10).grid(row=9, column=1, sticky="w", pady=(4, 0))

        include_auto_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(frame, text="自動運転中の走行も手本にする（slam2d_route など、"
                                    "うまく走れている自動運転をカメラで真似させる）",
                        variable=include_auto_var).grid(row=10, column=0, columnspan=4,
                                                        sticky="w", pady=(6, 0))

        ttk.Label(frame, text="ARM中・**速度モード**の指令が手本になります。既定は人の手動運転だけ\n"
                             "（待機中・トルクモードは常に除外。落ちた理由と枚数はログに出ます）。\n"
                             "画像は tools/record.sh --image-hz 15 で録ると枚数を稼げます"
                             "（実機GUIの記録は5Hz）。\n"
                             "「手本を遅らせる」は人の反応遅れの補正。まず 0 で学習し、⑥評価で"
                             "予測が遅れて見えたら 100〜200 を試します。\n"
                             "同じモデル名で再実行すると、既存のペアに追加されます。",
                 foreground="gray", justify="left").grid(row=11, column=0, columnspan=4,
                                                         sticky="w")

        def run() -> None:
            model_dir = self._require_model_dir()
            if model_dir is None:
                return
            if not self._extract_files:
                messagebox.showwarning("未選択", ".mcap ファイルを選んでください")
                return
            try:
                max_gap_ms = int(gap_var.get())
                label_shift_ms = int(shift_var.get())
            except ValueError:
                messagebox.showerror("入力エラー",
                                     "時刻差の許容・手本を遅らせる時間は整数で入力してください")
                return
            interval = 0
            count = 0
            if thin_mode.get() == "count":
                try:
                    count = int(count_var.get())
                except ValueError:
                    messagebox.showerror("入力エラー", "合計件数は整数で入力してください")
                    return
            else:
                try:
                    interval = int(interval_var.get())
                except ValueError:
                    messagebox.showerror("入力エラー", "間引き間隔は整数で入力してください")
                    return
            cmd = build_extract_cmd(self.python, self._extract_files, str(model_dir / "frames"),
                                    cam_var.get(), max_gap_ms, interval, count, label_shift_ms,
                                    include_auto_var.get())
            report: list[str] = []
            self._run("extract", cmd, "ペア抽出", on_line=report.append,
                      on_done=lambda ok: self._after_extract(ok, report))

        ttk.Button(frame, text="抽出実行", command=run).grid(row=12, column=0, sticky="w", pady=10)

        note_frame = ttk.LabelFrame(frame, text="備考（どんな変更をしたか・どのデータか等、自由に）")
        note_frame.grid(row=13, column=0, columnspan=4, sticky="we", pady=(4, 0))
        self.note_text = tk.Text(note_frame, height=3, wrap="word")
        self.note_text.pack(fill="both", expand=True, padx=4, pady=(4, 0))
        ttk.Button(note_frame, text="備考を保存", command=self._save_note).pack(
            anchor="e", padx=4, pady=4)

    def _after_extract(self, ok: bool, lines: list[str]) -> None:
        """抽出が終わったら、0枚だったときだけ理由を前面に出す。

        ログには理由が出ているが、流れていく文字は見落とす——「抽出は成功したのに
        ②に何も出ない」としか見えなかった（2026-10-06）。
        """
        message = extract_warning(lines) if ok else None
        if message:
            messagebox.showwarning("ペアが0枚でした", message)

    def _build_train_tab(self, nb: ttk.Notebook) -> None:
        frame = ttk.Frame(nb, padding=10)
        nb.add(frame, text="④ 学習")

        ttk.Label(frame, text="モデル名:").grid(row=0, column=0, sticky="w")
        ttk.Label(frame, textvariable=self.model_name_var, foreground="gray").grid(
            row=0, column=1, sticky="w")
        ttk.Label(frame, text="ペア:").grid(row=1, column=0, sticky="w", pady=(4, 0))
        ttk.Label(frame, textvariable=self.frames_path_var, foreground="gray").grid(
            row=1, column=1, columnspan=2, sticky="w", pady=(4, 0))
        ttk.Label(frame, text="出力先:").grid(row=2, column=0, sticky="w", pady=(4, 0))
        ttk.Label(frame, textvariable=self.model_out_dir_var, foreground="gray").grid(
            row=2, column=1, columnspan=2, sticky="w", pady=(4, 0))

        epochs_var = tk.StringVar(value="30")
        ttk.Label(frame, text="エポック数:").grid(row=3, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(frame, textvariable=epochs_var, width=10).grid(row=3, column=1, sticky="w", pady=(8, 0))

        batch_var = tk.StringVar(value="16")
        ttk.Label(frame, text="バッチサイズ:").grid(row=4, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(frame, textvariable=batch_var, width=10).grid(row=4, column=1, sticky="w", pady=(8, 0))

        size_var = tk.StringVar(value="224x128")
        ttk.Label(frame, text="入力解像度（幅x高さ）:").grid(row=5, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(frame, textvariable=size_var, width=10).grid(row=5, column=1, sticky="w", pady=(8, 0))
        ttk.Label(frame, text="前カメラ（640x360）と同じ 16:9。エクスポートはこの値を自動で使います",
                 foreground="gray").grid(row=5, column=2, sticky="w", pady=(8, 0))

        ttk.Label(frame, text="直進過多の補正:").grid(row=6, column=0, sticky="w", pady=(8, 0))
        balance_label = ttk.Label(frame, text="")
        ttk.Scale(frame, from_=0.0, to=1.0, variable=self.balance_var, length=120).grid(
            row=6, column=1, sticky="w", pady=(8, 0))
        balance_label.grid(row=6, column=2, sticky="w", pady=(8, 0))

        def show_balance(*_a) -> None:
            balance_label.config(text=f"{self.balance_var.get():.2f}（③偏りタブで分布を見て決める）")

        self.balance_var.trace_add("write", show_balance)
        show_balance()

        speed_weight_var = tk.StringVar(value="0.5")
        ttk.Label(frame, text="速度の重み:").grid(row=7, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(frame, textvariable=speed_weight_var, width=10).grid(
            row=7, column=1, sticky="w", pady=(8, 0))
        ttk.Label(frame, text="損失での速度の比重（舵=1）。舵を優先したいなら下げる",
                 foreground="gray").grid(row=7, column=2, sticky="w", pady=(8, 0))

        no_flip_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(frame, text="左右反転の水増しを使わない（片側通行など左右対称でないコース）",
                        variable=no_flip_var).grid(row=8, column=0, columnspan=3, sticky="w",
                                                   pady=(8, 0))
        no_pretrained_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(frame, text="事前学習重みを使わない（オフライン環境向け）",
                        variable=no_pretrained_var).grid(row=9, column=0, columnspan=3, sticky="w",
                                                         pady=(4, 0))

        def on_epoch_line(line: str) -> None:
            parsed = parse_epoch_line(line)
            if parsed is None:
                return
            epoch, loss, mae = parsed
            self.train_graph.add_point(epoch, loss, mae)

        def run() -> None:
            model_dir = self._require_model_dir()
            if model_dir is None:
                return
            try:
                epochs = int(epochs_var.get())
                batch = int(batch_var.get())
                speed_weight = float(speed_weight_var.get())
            except ValueError:
                messagebox.showerror("入力エラー",
                                     "エポック数・バッチサイズ・速度の重みは数値で入力してください")
                return
            self.train_graph.reset()
            cmd = build_train_cmd(self.python, str(model_dir / "frames"), str(model_dir), epochs,
                                  batch, size_var.get(), no_pretrained_var.get(),
                                  balance=self.balance_var.get(), speed_weight=speed_weight,
                                  no_flip=no_flip_var.get())
            self._run("train", cmd, "学習", on_line=on_epoch_line)

        ttk.Button(frame, text="学習開始", command=run).grid(row=10, column=0, sticky="w", pady=10)

        graph_frame = ttk.LabelFrame(
            frame, text="学習曲線（赤=loss・青=val_mae＝検証データでの舵の誤差。正規化値）")
        graph_frame.grid(row=11, column=0, columnspan=3, sticky="we", pady=(4, 0))
        self.train_graph = TrainCurveCanvas(graph_frame, metric_label="val_mae")
        self.train_graph.widget.pack(padx=4, pady=4)

    def _build_export_tab(self, nb: ttk.Notebook) -> None:
        frame = ttk.Frame(nb, padding=10)
        nb.add(frame, text="⑤ エクスポート")

        ttk.Label(frame, text="モデル名:").grid(row=0, column=0, sticky="w")
        ttk.Label(frame, textvariable=self.model_name_var, foreground="gray").grid(
            row=0, column=1, sticky="w")
        ttk.Label(frame, text="チェックポイント:").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Label(frame, textvariable=self.checkpoint_path_var, foreground="gray").grid(
            row=1, column=1, columnspan=2, sticky="w", pady=(8, 0))

        ttk.Label(frame, text="入力解像度と正規化の基準（最大舵角・速度の基準）は、学習時の値を\n"
                             "自動で使います。書き出した .onnx と .json は tools/deploy.sh で実車へ運び、\n"
                             "実機GUIの自動運転タブ「E2Eカメラ（模倣学習）」のモデル欄で選びます。",
                 foreground="gray", justify="left").grid(row=2, column=0, columnspan=3,
                                                         sticky="w", pady=(8, 0))

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
            cmd = build_export_cmd(self.python, str(model_dir / "best.pt"), str(out_path))
            self._run("export", cmd, "エクスポート")

        ttk.Button(frame, text="エクスポート実行", command=run).grid(row=5, column=0, sticky="w", pady=10)

    # ── サブプロセス実行・ログ配線 ──

    def _append_log(self, text: str) -> None:
        self.log_console.append(text)

    def _run(self, job_key: str, cmd: list[str], label: str, *,
            on_line: Callable[[str], None] | None = None,
            on_done: Callable[[bool], None] | None = None) -> None:
        if self.jobs.is_busy():
            messagebox.showwarning("実行中", "他の処理が終わってから実行してください")
            return
        self.status_label.config(text=f"{label} 実行中…")
        self.stop_btn.config(state="normal")
        self._append_log(f"\n$ {' '.join(cmd)}\n")
        self.jobs.start(job_key, cmd, label, on_line=on_line,
                        on_done=lambda code: self._on_job_done(label, code, on_done))

    def _on_job_done(self, label: str, code: int,
                     on_done: Callable[[bool], None] | None = None) -> None:
        ok = code == 0
        if ok and label in ("ペア抽出", "学習"):
            self.model_status_var.set(
                describe_model_status(RUNS_DIR / self.model_name_var.get().strip()))
        if ok and label == "エクスポート":
            self.eval_tab.select_model(self._last_exported_model_name)
        self.status_label.config(text="待機中")
        self.stop_btn.config(state="disabled")
        if on_done is not None:
            on_done(ok)

    def _stop(self) -> None:
        active = self.jobs.active_jobs()
        if active and self.jobs.stop(active[0][0]):
            self._append_log("\n（停止を指示しました）\n")

    def _drain_log(self) -> None:
        self.jobs.drain(self._append_log)
        self.root.after(100, self._drain_log)


def main() -> int:
    root = tk.Tk()
    App(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
