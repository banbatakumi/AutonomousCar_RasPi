"""ml_e2e/app.py — E2E 融合（LiDAR＋カメラ＋車両状態）の、標本作成・ラベル確認・学習・エクスポート・評価の操作パネル。

    .venv/bin/python ml_e2e/app.py
    （または `launcher.command`（surge_mk2 直下）から起動）

タブは作業の順に5枚:

    ① 標本作成    録った .mcap から (画像・点群・車両状態, 経路点・舵・速度) の組を作る。記録の一覧と取り外し
    ② ラベル確認  経路点が壁の内側を通るか・記録の舵と合うかの表と、画像・点群への重ね描き
    ③ 学習        検証に回す記録を選んで学習する
    ④ エクスポート  実車が読む ONNX と契約ファイルを書く
    ⑤ 評価        記録を実車と同じ推論ノードと planner に通して、手本と比べる

重い処理は `extract.py`・`check_labels.py`・`train.py`・`export_onnx.py`・`eval.py` をサブプロセスとして呼ぶだけ
（`ml_cam_e2e/app.py` と同じ設計。このプロセスは torch を import しない）。ログ表示・サブプロセスの実行・
モデル名の採番・備考・学習曲線・閉じるときの確認は `ml_common/` の共通部品。

## モデル名がそのまま置き場を決める

    ml_e2e/runs/<モデル名>/data/                 … ①の出力（記録ごとの .npz と frames/）・②③⑤の入力
    ml_e2e/runs/<モデル名>/data/dataset.json     … 抽出の設定と、記録ごとの内訳・元の .mcap の場所
    ml_e2e/runs/<モデル名>/data/check/           … ②の重ね描き
    ml_e2e/runs/<モデル名>/best.pt               … ③の出力・④の入力
    ml_e2e/runs/<モデル名>/note.txt              … 備考（①タブ。エクスポート時に契約へ同梱）
    models/e2e_fusion/<モデル名>.onnx            … ④の出力（実車が読む場所）

## 手本は自動で決まる

`cam_e2e` と違い「自動運転の走行も手本にする」の切り替えは無い。記録の状況で決まる（`extract.py`）:
`slam2d_route` が走っている区間は地図の最適ライン、それ以外の自動運転と人の手動運転は、その後に通った軌跡。
人の手動運転の速度は手本にしない。
"""

from __future__ import annotations

import json
import shutil
import sys
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Callable

ML_E2E_DIR = Path(__file__).resolve().parent
REPO_ROOT = ML_E2E_DIR.parent
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
    "ML_E2E_DIR", "REPO_ROOT", "RUNS_DIR", "MODELS_DIR",
    "build_extract_cmd", "build_check_cmd", "build_train_cmd", "build_export_cmd", "build_eval_cmd",
    "read_dataset", "record_rows", "default_val_records", "remove_record", "extract_warning",
    "describe_model_status", "parse_epoch_line", "App",
]

RUNS_DIR = ML_E2E_DIR / "runs"
#: 実車側（`raspi/nodes/e2e_fusion_node.py` の `DEFAULT_MODELS_DIR`）と同じ場所
MODELS_DIR = REPO_ROOT / "models" / "e2e_fusion"
#: 学習だけは長いので、ウィンドウを閉じる前に確認する
_CONFIRM_STOP_JOB_KEYS = frozenset({"train"})

rel = make_rel(REPO_ROOT)
parse_epoch_line = make_epoch_line_parser("val_mae")


# ── コマンド組み立て・データの読み書き（Tkinter を知らない純粋関数） ──

def build_extract_cmd(python: str, mcap_files: list[str], data_dir: str, *, label_shift_ms: int = 100,
                      include_self: bool = False) -> list[str]:
    cmd = [python, "-m", "ml_e2e.extract", *mcap_files, "--out", data_dir,
           "--label-shift-ms", str(label_shift_ms)]
    if include_self:
        cmd.append("--include-self")
    return cmd


def build_check_cmd(python: str, data_dir: str, draw: int) -> list[str]:
    return [python, "-m", "ml_e2e.check_labels", data_dir, "--draw", str(draw)]


def build_train_cmd(python: str, data_dir: str, out_dir: str, *, epochs: int, batch_size: int,
                    val: list[str] | None = None, init: str = "", no_flip: bool = False,
                    no_pretrained: bool = False) -> list[str]:
    cmd = [python, "-m", "ml_e2e.train", "--data", data_dir, "--out", out_dir,
           "--epochs", str(epochs), "--batch-size", str(batch_size)]
    if val:
        cmd += ["--val", *val]
    if init:
        cmd += ["--init", init]
    if no_flip:
        cmd.append("--no-flip")
    if no_pretrained:
        cmd.append("--no-pretrained")
    return cmd


def build_export_cmd(python: str, run_dir: str, name: str) -> list[str]:
    """契約は `train_config.json` から写されるので、ここでは何も渡さない。"""
    return [python, "-m", "ml_e2e.export_onnx", "--run", run_dir, "--name", name]


def build_eval_cmd(python: str, model_path: str, data_dir: str, mcap_files: list[str], *,
                   no_image: bool = False, no_lidar: bool = False) -> list[str]:
    cmd = [python, "-m", "ml_e2e.eval", "--model", model_path, "--data", data_dir, *mcap_files]
    if no_image:
        cmd.append("--no-image")
    if no_lidar:
        cmd.append("--no-lidar")
    return cmd


def read_dataset(data_dir: Path) -> dict:
    """`dataset.json`。無い・読めないなら空の形。"""
    try:
        meta = json.loads((data_dir / "dataset.json").read_text())
    except (OSError, ValueError):
        return {"config": {}, "records": {}}
    meta.setdefault("records", {})
    return meta


def record_rows(meta: dict) -> list[tuple[str, int, int, int, int, int, str]]:
    """記録ごとの `(名前, 枚数, ライン, 軌跡, 自動, 手動, 地図)`。標本が 0 の記録は出さない。"""
    rows = []
    for name in sorted(meta.get("records", {})):
        info = meta["records"][name]
        if not info.get("samples"):
            continue
        rows.append((name, int(info["samples"]), int(info.get("line", 0)), int(info.get("future", 0)),
                     int(info.get("auto", 0)), int(info.get("manual", 0)), ", ".join(info.get("maps", []))))
    return rows


def default_val_records(meta: dict) -> list[str]:
    """検証に回す記録の既定（`dataset.split_records` と同じ規則）: 同じ地図の記録が3本以上あるとき、その最後の1本。"""
    groups: dict[str, list[str]] = {}
    for name, *_rest, maps in record_rows(meta):
        groups.setdefault(maps.split(", ")[0] if maps else "", []).append(name)
    return sorted(g[-1] for key, g in groups.items() if key and len(g) >= 3)


def remove_record(data_dir: Path, name: str) -> None:
    """記録1本ぶんの標本（`.npz`・画像・`dataset.json` の項目）を消す。元の `.mcap` は消さない。"""
    (data_dir / f"{name}.npz").unlink(missing_ok=True)
    shutil.rmtree(data_dir / "frames" / name, ignore_errors=True)
    meta_path = data_dir / "dataset.json"
    meta = read_dataset(data_dir)
    if meta["records"].pop(name, None) is not None:
        meta_path.write_text(json.dumps(meta, ensure_ascii=False, indent=1))


def extract_warning(lines: list[str]) -> str | None:
    """`extract.py` の出力から、0枚で終わったときに見せる文面を作る。1枚でも作れていれば `None`。"""
    total = next((ln for ln in reversed(lines) if ln.startswith("# 合計 ")), None)
    if total is None or not total.startswith("# 合計 0枚"):
        return None
    parts = ["選んだ記録から、標本が1枚も作れませんでした。"]
    reasons = next((ln[2:].strip() for ln in lines if ln.startswith("# 使わなかったコマ")), "")
    if reasons:
        parts.append(reasons)
    skipped = [ln[2:].strip() for ln in lines if ln.startswith("# skip:") or "使わない（" in ln]
    if skipped:
        parts.append("\n".join(skipped))
    if "トルクモード" in reasons:
        parts.append("トルクモードの走行は手本になりません。速度モードで録ってください。")
    if not reasons and not skipped:
        parts.append("記録に前カメラの画像が入っていない可能性があります（画像つきで録ったか確認してください）。")
    return "\n\n".join(parts)


def describe_model_status(model_dir: Path) -> str:
    rows = record_rows(read_dataset(model_dir / "data"))
    mark = "✓学習済み" if (model_dir / "best.pt").exists() else "未学習"
    return f"記録 {len(rows)}本・標本 {sum(r[1] for r in rows)}枚・{mark}"


# ── GUI ──

class App:
    """タブ5枚（標本作成・ラベル確認・学習・エクスポート・評価）＋共有のログ欄。"""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        root.title("SURGE Mk.2 — E2E 融合（LiDAR＋カメラ）")
        root.geometry("1120x900")
        self.python = sys.executable
        self.jobs = JobRunner(REPO_ROOT, allow_concurrent=False, prefix_labels=False)
        self._check_images: list[Path] = []
        self._check_index = 0
        self._photo: tk.PhotoImage | None = None        # 参照を持たないと GC で消える

        self._build_widgets()
        self.root.after(100, self._drain_log)
        self.root.protocol("WM_DELETE_WINDOW", lambda: confirm_and_close(
            self.root, self.jobs, confirm_job_keys=_CONFIRM_STOP_JOB_KEYS,
            confirm_message=lambda label: f"{label}が実行中です。終了すると学習が中断されます。"
            "\n\n本当に終了しますか？"))

    # ── 画面構築 ──

    def _build_widgets(self) -> None:
        self.model_name_var = tk.StringVar(value=self._initial_name())
        self.model_status_var = tk.StringVar(value="")
        self.data_path_var = tk.StringVar(value="")
        self.export_out_path_var = tk.StringVar(value="")

        bottom = ttk.Frame(self.root)
        bottom.pack(side="bottom", fill="x", padx=8, pady=(0, 8))
        self.status_label = ttk.Label(bottom, text="待機中")
        self.status_label.pack(side="left")
        self.stop_btn = ttk.Button(bottom, text="停止", command=self._stop, state="disabled")
        self.stop_btn.pack(side="right")

        log_frame = ttk.LabelFrame(self.root, text="ログ")
        log_frame.pack(side="bottom", fill="x", padx=8, pady=(0, 8))
        self.log_console = LogConsole(log_frame, height=9)
        self.log_console.widget.pack(fill="both", expand=True, padx=4, pady=4)

        nb = ttk.Notebook(self.root)
        nb.pack(side="top", fill="both", expand=True, padx=8, pady=8)
        self.notebook = nb
        self._build_extract_tab(nb)
        self._build_check_tab(nb)
        self._build_train_tab(nb)
        self._build_export_tab(nb)
        self._build_eval_tab(nb)
        nb.bind("<<NotebookTabChanged>>", lambda _e: self._refresh())

        self.model_name_var.trace_add("write", lambda *_a: self._on_model_name_changed())
        self._on_model_name_changed()

    @staticmethod
    def _initial_name() -> str:
        """作業中のモデル（いちばん新しい run）。無ければ `v1`。"""
        names = list_versioned_names(RUNS_DIR)
        if not names:
            return "v1"
        return max(names, key=lambda n: (RUNS_DIR / n).stat().st_mtime)

    # ── モデル名（全タブ共有） ──

    def _model_dir(self) -> Path | None:
        name = self.model_name_var.get().strip()
        return RUNS_DIR / name if name else None

    def _require_model_dir(self) -> Path | None:
        d = self._model_dir()
        if d is None:
            messagebox.showerror("入力エラー", "①タブでモデル名を入力してください")
        return d

    def _on_model_name_changed(self) -> None:
        d = self._model_dir()
        if d is not None:
            self.data_path_var.set(rel(d / "data"))
            self.export_out_path_var.set(rel(MODELS_DIR / f"{d.name}.onnx"))
            note = read_note(d)
        else:
            self.data_path_var.set("（モデル名を入力してください）")
            self.export_out_path_var.set("")
            note = ""
        self.note_text.delete("1.0", "end")
        self.note_text.insert("1.0", note)
        self._refresh()

    def _refresh(self) -> None:
        """今のモデルの内容を全タブに映し直す（タブを開くたび・処理が終わるたび）。"""
        d = self._model_dir()
        meta = read_dataset(d / "data") if d is not None else {"records": {}}
        rows = record_rows(meta)
        self.model_status_var.set(describe_model_status(d) if d is not None else "")

        self.record_tree.delete(*self.record_tree.get_children())
        for row in rows:
            self.record_tree.insert("", "end", iid=row[0], values=row)

        # 検証に回す記録: 前に選んだものを残し、初めてなら既定を選ぶ
        names = [r[0] for r in rows]
        if names != list(self.val_list.get(0, "end")):
            self.val_list.delete(0, "end")
            for n in names:
                self.val_list.insert("end", n)
            for n in default_val_records(meta):
                self.val_list.selection_set(names.index(n))

        self._eval_meta = meta
        self._load_check_images()
        self.init_combo["values"] = [""] + [n for n in list_versioned_names(RUNS_DIR)
                                           if (RUNS_DIR / n / "best.pt").exists()
                                           and (d is None or n != d.name)]

    def _save_note(self) -> None:
        d = self._require_model_dir()
        if d is None:
            return
        write_note(d, self.note_text.get("1.0", "end-1c"))
        self._append_log(f"\n備考を保存しました（{d.name}）\n")

    # ── ① 標本作成 ──

    def _build_extract_tab(self, nb: ttk.Notebook) -> None:
        frame = ttk.Frame(nb, padding=10)
        nb.add(frame, text="① 標本作成")
        frame.columnconfigure(3, weight=1)

        ttk.Label(frame, text="モデル名:").grid(row=0, column=0, sticky="w")
        combo = ttk.Combobox(frame, textvariable=self.model_name_var, width=20)
        combo.grid(row=0, column=1, sticky="w")
        combo["postcommand"] = lambda: combo.configure(values=list_versioned_names(RUNS_DIR))
        ttk.Button(frame, text="新しい名前", command=lambda: self.model_name_var.set(
            next_versioned_name(list_versioned_names(RUNS_DIR)))).grid(row=0, column=2, sticky="w", padx=6)
        ttk.Label(frame, textvariable=self.model_status_var, foreground="gray").grid(
            row=0, column=3, sticky="w")
        ttk.Label(frame, text="出力先:").grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Label(frame, textvariable=self.data_path_var, foreground="gray").grid(
            row=1, column=1, columnspan=3, sticky="w", pady=(6, 0))

        self._extract_files: list[str] = []
        files_var = tk.StringVar(value="(.mcap 未選択)")

        def pick_files() -> None:
            paths = filedialog.askopenfilenames(title="録った .mcap を選ぶ（複数可・画像つきのもの）",
                                                filetypes=[("MCAP", "*.mcap")])
            if paths:
                self._extract_files = list(paths)
                names = ", ".join(Path(p).name for p in paths[:3])
                files_var.set(f"{len(paths)}個: {names}{'…' if len(paths) > 3 else ''}")

        ttk.Button(frame, text="記録(.mcap)を選ぶ", command=pick_files).grid(row=2, column=0, sticky="w",
                                                                           pady=(8, 0))
        ttk.Label(frame, textvariable=files_var, wraplength=700).grid(row=2, column=1, columnspan=3,
                                                                     sticky="w", pady=(8, 0))

        shift_var = tk.StringVar(value="100")
        ttk.Label(frame, text="舵・速度の手本を遅らせる[ms]:").grid(row=3, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(frame, textvariable=shift_var, width=8).grid(row=3, column=1, sticky="w", pady=(8, 0))
        ttk.Label(frame, text="直接出力の舵・速度だけに効く（経路点は実車が推測航法でずらす）。"
                             "同じモデルでは変えられない", foreground="gray").grid(
            row=3, column=2, columnspan=2, sticky="w", pady=(8, 0))
        include_self_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(frame, text="このモデル自身（E2E 融合）で走った区間も使う（経路点だけ。舵・速度は手本にしない）",
                        variable=include_self_var).grid(row=4, column=0, columnspan=4, sticky="w", pady=(6, 0))

        ttk.Label(frame, text="画像つきで録った記録（実機 GUI の MCAP ボタン、または tools/record.sh --image-hz 10）を選ぶ。\n"
                             "手本は記録の状況で自動で決まる: slam2d_route の走行 → 地図の最適ラインと計画速度／"
                             "それ以外の自動運転・人の手動運転 → その後に通った軌跡（人の速度は手本にしない）。\n"
                             "DISARM 中・停車中・トルクモードは使わない（理由ごとの枚数がログに出る）。"
                             "同じモデル名で再実行すると記録が追加される（同じ記録は作り直す）。",
                  foreground="gray", justify="left").grid(row=5, column=0, columnspan=4, sticky="w", pady=(6, 0))

        def run() -> None:
            d = self._require_model_dir()
            if d is None:
                return
            if not self._extract_files:
                messagebox.showwarning("未選択", ".mcap ファイルを選んでください")
                return
            try:
                shift = int(shift_var.get())
            except ValueError:
                messagebox.showerror("入力エラー", "手本を遅らせる時間は整数で入力してください")
                return
            report: list[str] = []
            cmd = build_extract_cmd(self.python, self._extract_files, str(d / "data"),
                                    label_shift_ms=shift, include_self=include_self_var.get())
            self._run("extract", cmd, "標本作成", on_line=report.append,
                      on_done=lambda ok: self._after_extract(ok, report))

        ttk.Button(frame, text="標本を作る", command=run).grid(row=6, column=0, sticky="w", pady=8)

        box = ttk.LabelFrame(frame, text="このモデルに入っている記録")
        box.grid(row=7, column=0, columnspan=4, sticky="nsew")
        frame.rowconfigure(7, weight=1)
        cols = (("name", "記録", 230), ("n", "枚数", 60), ("line", "ライン", 60), ("future", "軌跡", 60),
                ("auto", "自動", 60), ("manual", "手動", 60), ("maps", "地図", 220))
        self.record_tree = ttk.Treeview(box, columns=[c[0] for c in cols], show="headings", height=8)
        for key, title, width in cols:
            self.record_tree.heading(key, text=title)
            self.record_tree.column(key, width=width, anchor="w" if key in ("name", "maps") else "e")
        self.record_tree.pack(side="left", fill="both", expand=True, padx=4, pady=4)
        ttk.Button(box, text="選んだ記録を外す", command=self._remove_selected).pack(side="top", padx=4, pady=4)

        note_frame = ttk.LabelFrame(frame, text="備考（どのコース・どんな走りか等、自由に）")
        note_frame.grid(row=8, column=0, columnspan=4, sticky="we", pady=(6, 0))
        self.note_text = tk.Text(note_frame, height=2, wrap="word")
        self.note_text.pack(side="left", fill="both", expand=True, padx=4, pady=4)
        ttk.Button(note_frame, text="備考を保存", command=self._save_note).pack(side="right", padx=4, pady=4)

    def _after_extract(self, ok: bool, lines: list[str]) -> None:
        message = extract_warning(lines) if ok else None
        if message:
            messagebox.showwarning("標本が0枚でした", message)
        elif not ok and any("設定が今回と違います" in ln for ln in lines):
            messagebox.showwarning("設定が違います",
                                   "このモデルは別の設定（手本を遅らせる時間など）で標本を作ってあります。\n"
                                   "同じ設定に戻すか、新しいモデル名で作ってください。")

    def _remove_selected(self) -> None:
        d = self._model_dir()
        names = list(self.record_tree.selection())
        if d is None or not names:
            return
        if self.jobs.is_busy():
            messagebox.showwarning("実行中", "他の処理が終わってから実行してください")
            return
        if not messagebox.askyesno("記録を外す", f"{len(names)}本の記録の標本を消します"
                                   "（元の .mcap は消えません）。\n\n" + "\n".join(names)):
            return
        for n in names:
            remove_record(d / "data", n)
        self._append_log(f"\n記録を外しました: {', '.join(names)}\n")
        self._refresh()

    # ── ② ラベル確認 ──

    def _build_check_tab(self, nb: ttk.Notebook) -> None:
        frame = ttk.Frame(nb, padding=10)
        nb.add(frame, text="② ラベル確認")
        top = ttk.Frame(frame)
        top.pack(fill="x")
        draw_var = tk.StringVar(value="36")
        ttk.Label(top, text="重ね描きの枚数（記録ごとに最低1枚）:").pack(side="left")
        ttk.Entry(top, textvariable=draw_var, width=6).pack(side="left", padx=(4, 10))

        def run() -> None:
            d = self._require_model_dir()
            if d is None:
                return
            try:
                draw = int(draw_var.get())
            except ValueError:
                messagebox.showerror("入力エラー", "枚数は整数で入力してください")
                return
            shutil.rmtree(d / "data" / "check", ignore_errors=True)
            self._run("check", build_check_cmd(self.python, str(d / "data"), draw), "ラベル確認")

        ttk.Button(top, text="確認を実行", command=run).pack(side="left")
        ttk.Label(frame, text="表はログに出る。見る所: 「接触率」（経路が壁のすぐ近くを通る割合。高ければ姿勢の時刻か座標が違う）、\n"
                             "「舵の相関」（経路を追った舵と記録の舵。0.8 前後が普通。負や 0 付近は左右か時刻が違う）。\n"
                             "下の絵は、左が画像に投影した経路点、右が上から見た点群（白）と経路点（緑）。経路が走路の中を通っているか見る。",
                  foreground="gray", justify="left").pack(anchor="w", pady=(6, 4))
        nav = ttk.Frame(frame)
        nav.pack(fill="x")
        ttk.Button(nav, text="◀ 前", command=lambda: self._step_check(-1)).pack(side="left")
        ttk.Button(nav, text="次 ▶", command=lambda: self._step_check(1)).pack(side="left", padx=4)
        self.check_label_var = tk.StringVar(value="（重ね描きがまだ無い）")
        ttk.Label(nav, textvariable=self.check_label_var, foreground="gray").pack(side="left", padx=8)
        self.check_canvas = tk.Label(frame, background="black")
        self.check_canvas.pack(fill="both", expand=True, pady=(4, 0))

    def _load_check_images(self) -> None:
        d = self._model_dir()
        images = sorted((d / "data" / "check").glob("*.jpg")) if d is not None else []
        if images != self._check_images:
            self._check_images = images
            self._check_index = 0
        self._show_check()

    def _step_check(self, step: int) -> None:
        if self._check_images:
            self._check_index = (self._check_index + step) % len(self._check_images)
            self._show_check()

    def _show_check(self) -> None:
        if not self._check_images:
            self._photo = None
            self.check_canvas.configure(image="")
            self.check_label_var.set("（重ね描きがまだ無い。「確認を実行」で作る）")
            return
        path = self._check_images[self._check_index]
        try:
            import numpy as np
            from PIL import Image
            rgb = np.asarray(Image.open(path).convert("RGB"))
            h, w = rgb.shape[:2]
            self._photo = tk.PhotoImage(data=b"P6 %d %d 255\n" % (w, h) + rgb.tobytes(), format="PPM")
            self.check_canvas.configure(image=self._photo)
        except Exception as e:                           # noqa: BLE001  絵が読めなくても操作は続ける
            self.check_label_var.set(f"読めない: {path.name}（{e}）")
            return
        self.check_label_var.set(f"{self._check_index + 1}/{len(self._check_images)}  {path.name}")

    # ── ③ 学習 ──

    def _build_train_tab(self, nb: ttk.Notebook) -> None:
        frame = ttk.Frame(nb, padding=10)
        nb.add(frame, text="③ 学習")
        left = ttk.Frame(frame)
        left.grid(row=0, column=0, sticky="nw")

        ttk.Label(left, text="モデル名:").grid(row=0, column=0, sticky="w")
        ttk.Label(left, textvariable=self.model_name_var, foreground="gray").grid(row=0, column=1, sticky="w")
        epochs_var = tk.StringVar(value="20")
        ttk.Label(left, text="エポック数:").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(left, textvariable=epochs_var, width=8).grid(row=1, column=1, sticky="w", pady=(8, 0))
        batch_var = tk.StringVar(value="64")
        ttk.Label(left, text="バッチサイズ:").grid(row=2, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(left, textvariable=batch_var, width=8).grid(row=2, column=1, sticky="w", pady=(8, 0))
        init_var = tk.StringVar(value="")
        ttk.Label(left, text="追加学習の元:").grid(row=3, column=0, sticky="w", pady=(8, 0))
        self.init_combo = ttk.Combobox(left, textvariable=init_var, width=12, state="readonly")
        self.init_combo.grid(row=3, column=1, sticky="w", pady=(8, 0))
        ttk.Label(left, text="空なら最初から。選ぶとそのモデルの重みから始める\n（新しいコースの記録を足したとき）",
                  foreground="gray", justify="left").grid(row=4, column=0, columnspan=2, sticky="w")
        no_flip_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(left, text="左右反転の水増しを使わない", variable=no_flip_var).grid(
            row=5, column=0, columnspan=2, sticky="w", pady=(8, 0))
        no_pretrained_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(left, text="事前学習重みを使わない（オフライン）", variable=no_pretrained_var).grid(
            row=6, column=0, columnspan=2, sticky="w", pady=(4, 0))

        right = ttk.LabelFrame(frame, text="検証に回す記録（学習に使わない。選ばなければ全部で学習）")
        right.grid(row=0, column=1, sticky="nsew", padx=(16, 0))
        frame.columnconfigure(1, weight=1)
        self.val_list = tk.Listbox(right, selectmode="multiple", height=9, exportselection=False)
        self.val_list.pack(fill="both", expand=True, padx=4, pady=4)
        ttk.Label(right, text="同じコースの別の日・別の走行を1本ずつ選ぶ。\n同じ走行の一部を検証にしても実力は分からない",
                  foreground="gray", justify="left").pack(anchor="w", padx=4, pady=(0, 4))

        def on_line(line: str) -> None:
            parsed = parse_epoch_line(line)
            if parsed is not None:
                self.train_graph.add_point(*parsed)

        def run() -> None:
            d = self._require_model_dir()
            if d is None:
                return
            if not record_rows(read_dataset(d / "data")):
                messagebox.showwarning("標本が無い", "先に①で標本を作ってください")
                return
            try:
                epochs, batch = int(epochs_var.get()), int(batch_var.get())
            except ValueError:
                messagebox.showerror("入力エラー", "エポック数・バッチサイズは整数で入力してください")
                return
            val = [self.val_list.get(i) for i in self.val_list.curselection()]
            if len(val) == self.val_list.size():
                messagebox.showwarning("学習に使う記録が無い", "全部の記録を検証に回しています")
                return
            if (d / "best.pt").exists() and not messagebox.askyesno(
                    "上書きしますか？", f"{d.name} は学習済みです。学習し直すと上書きされます。"):
                return
            init = str(RUNS_DIR / init_var.get() / "best.pt") if init_var.get() else ""
            self.train_graph.reset()
            cmd = build_train_cmd(self.python, str(d / "data"), str(d), epochs=epochs, batch_size=batch,
                                  val=val, init=init, no_flip=no_flip_var.get(),
                                  no_pretrained=no_pretrained_var.get())
            self._run("train", cmd, "学習", on_line=on_line)

        ttk.Button(frame, text="学習開始", command=run).grid(row=1, column=0, sticky="w", pady=10)
        ttk.Label(frame, text="ログの見方: val_mae＝検証の経路点の誤差 [m]（全体）、near＝最初の 2m の誤差、b＝モデルが出す不確かさ、\n"
                             "steer・speed＝直接出力の舵・速度の誤差、v＝各点の速度の誤差。最後に「入力を伏せたときの経路誤差」が出る\n"
                             "（画像だけ・点群だけでも経路が出るか）。1エポックは標本1万枚で約1分（Mac）。",
                  foreground="gray", justify="left").grid(row=2, column=0, columnspan=2, sticky="w")
        graph_frame = ttk.LabelFrame(frame, text="学習曲線（赤=loss・青=val_mae＝検証の経路点の誤差 [m]）")
        graph_frame.grid(row=3, column=0, columnspan=2, sticky="we", pady=(6, 0))
        self.train_graph = TrainCurveCanvas(graph_frame, metric_label="val_mae")
        self.train_graph.widget.pack(padx=4, pady=4)

    # ── ④ エクスポート ──

    def _build_export_tab(self, nb: ttk.Notebook) -> None:
        frame = ttk.Frame(nb, padding=10)
        nb.add(frame, text="④ エクスポート")
        ttk.Label(frame, text="モデル名:").grid(row=0, column=0, sticky="w")
        ttk.Label(frame, textvariable=self.model_name_var, foreground="gray").grid(row=0, column=1, sticky="w")
        ttk.Label(frame, text="出力先:").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Label(frame, textvariable=self.export_out_path_var, foreground="gray").grid(
            row=1, column=1, sticky="w", pady=(8, 0))
        ttk.Label(frame, text="入出力の契約（解像度・視野・経路点の数・正規化の基準）は学習時の値をそのまま書く。\n"
                             "書き出した .onnx と .json は tools/deploy.sh で実車へ運ぶ。\n"
                             "実機 GUI の自動運転タブ「E2E 融合（LiDAR＋カメラ）」のモデル欄で選ぶ"
                             "（「自動」なら models/e2e_fusion/ のいちばん新しい .onnx）。",
                  foreground="gray", justify="left").grid(row=2, column=0, columnspan=2, sticky="w", pady=(8, 0))

        def run() -> None:
            d = self._require_model_dir()
            if d is None:
                return
            if not (d / "best.pt").exists():
                messagebox.showwarning("未学習", "先に③で学習してください")
                return
            if not confirm_overwrite(MODELS_DIR / f"{d.name}.onnx", rel):
                return
            self._run("export", build_export_cmd(self.python, str(d), d.name), "エクスポート")

        ttk.Button(frame, text="エクスポート実行", command=run).grid(row=3, column=0, sticky="w", pady=10)

    # ── ⑤ 評価 ──

    def _build_eval_tab(self, nb: ttk.Notebook) -> None:
        frame = ttk.Frame(nb, padding=10)
        nb.add(frame, text="⑤ 評価")
        self._eval_meta: dict = {"records": {}}
        self._eval_files: list[str] = []
        files_var = tk.StringVar(value="検証に回した記録（③で学習に使わなかったもの）")

        def pick_files() -> None:
            paths = filedialog.askopenfilenames(title="評価する .mcap を選ぶ（複数可）",
                                                filetypes=[("MCAP", "*.mcap")])
            self._eval_files = list(paths)
            files_var.set(f"{len(paths)}個: " + ", ".join(Path(p).name for p in paths[:3])
                          + ("…" if len(paths) > 3 else "") if paths
                          else "検証に回した記録（③で学習に使わなかったもの）")

        ttk.Button(frame, text="別の記録(.mcap)を選ぶ", command=pick_files).grid(row=0, column=0, sticky="w")
        ttk.Label(frame, textvariable=files_var, wraplength=700).grid(row=0, column=1, sticky="w", padx=8)
        mode_var = tk.StringVar(value="both")
        modes = ttk.Frame(frame)
        modes.grid(row=1, column=0, columnspan=2, sticky="w", pady=(8, 0))
        for text, value in (("画像＋点群", "both"), ("点群だけ（画像を伏せる）", "lidar"),
                            ("画像だけ（点群を伏せる）", "image")):
            ttk.Radiobutton(modes, text=text, variable=mode_var, value=value).pack(side="left", padx=(0, 12))

        def val_files(d: Path) -> list[str]:
            """③で検証に回した記録の、元の .mcap（場所が分かって残っているもの）。"""
            try:
                names = json.loads((d / "train_config.json").read_text()).get("val_records", [])
            except (OSError, ValueError):
                names = []
            paths = [self._eval_meta["records"].get(n, {}).get("path", "") for n in names]
            return [p for p in paths if p and Path(p).is_file()]

        def run() -> None:
            d = self._require_model_dir()
            if d is None:
                return
            model = MODELS_DIR / f"{d.name}.onnx"
            if not model.exists():
                messagebox.showwarning("未エクスポート", "先に④でエクスポートしてください")
                return
            files = self._eval_files or val_files(d)
            if not files:
                messagebox.showwarning("記録が無い", "検証に回した記録の元の .mcap が見つかりません"
                                       "（移動した・古い標本で場所が残っていない）。\n「別の記録(.mcap)を選ぶ」で選んでください。")
                return
            cmd = build_eval_cmd(self.python, str(model), str(d / "data"), files,
                                 no_image=mode_var.get() == "lidar", no_lidar=mode_var.get() == "image")
            self._run("eval", cmd, "評価")

        ttk.Button(frame, text="評価を実行", command=run).grid(row=2, column=0, sticky="w", pady=10)
        ttk.Label(frame, text=(
            "記録を、実車と同じ推論ノードと planner に通して手本と比べる（表はログに出る）。学習に使っていない記録で見ること。\n\n"
            "  走れる     経路を出せた割合。出せなかった理由は下に並ぶ\n"
            "  舵相関・舵RMS  planner が出した舵と、記録の指令の舵\n"
            "  直接RMS    モデルの舵の直接出力と、記録の指令の舵\n"
            "  速度比     planner の目標速度 ÷ 記録の指令の速度。1 より小さければ手本より遅い\n"
            "  近く・全体   経路点とラベルの距離（最初の 2m・全体）\n"
            "  b          モデルが出す不確かさ [m]\n"
            "  dδ/dψ・理想比  車の向きをずらした入力で、経路を追う舵がどれだけ戻るか。理想比が 1 に近いほど、ずれを戻せる\n\n"
            "★ 開ループの比較（車は記録どおりに動く）。周回できるかどうかは実走でしか分からない。"),
            foreground="gray", justify="left").grid(row=3, column=0, columnspan=2, sticky="w")

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
                        on_done=lambda code: self._on_job_done(code, on_done))

    def _on_job_done(self, code: int, on_done: Callable[[bool], None] | None = None) -> None:
        self.status_label.config(text="待機中")
        self.stop_btn.config(state="disabled")
        self._refresh()
        if on_done is not None:
            on_done(code == 0)

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
