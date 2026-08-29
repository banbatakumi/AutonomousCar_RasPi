"""ml_lidar/app.py — LiDAR E2E の学習・観戦・エクスポートをターミナル無しで
操作するための最小限のGUI（`ml/app.py`と対称）。

    .venv/bin/python ml_lidar/app.py
    （または `ml_lidar/start_app.command` をダブルクリック）

`ml_lidar/train_rl.py`・`export_onnx_rl.py`・`watch.py`・`tensorboard`を
サブプロセスとして呼び出すだけの薄い操作パネル。**学習・推論のロジックは
一切持たない**——`ml/app.py`と同じ設計方針。

## `ml/app.py`と違う点：複数ジョブが同時に動く

カメラ版は「抽出→アノテーション→学習→エクスポート」が順番に1つずつ進む
パイプラインだったので、常に1プロセスしか同時に動かない前提で作れた。
LiDAR E2E は**学習が数時間かかる裏で、TensorBoard・観戦(`watch.py`)・
（別runの）エクスポートを並行して動かしたい**（2026-08-28、バンビの要望）ので、
ジョブをキー付きの辞書（`self._active`）で管理し、複数プロセスを同時に
追跡できるようにしてある。

## run名がそのまま学習出力先とモデル名になる

`train_rl.py --out`（学習の出力先）と`export_onnx_rl.py --out`（エクスポート先の
モデル名）を別々に考えるのが混乱の元だった。ここでは**1つの「run名」**を学習前に
決め、`ml_lidar/runs/<run名>`が学習出力先、`models/e2e_lidar/<run名>.onnx`が
エクスポート先の初期値になる（エクスポート時に名前を変えることもできる）。
run名は`v1`・`v2`…の続きを自動提案する（`next_run_name()`）。

既存のrun名で学習開始すると、**続きから再開／上書きして新規／キャンセル**の
3択を聞く（2026-08-28、`train_rl.py --resume-from`に対応。再開は`best_model.zip`から
`PPO.load()`する）。
"""

from __future__ import annotations

import json
import os
import queue
import re
import subprocess
import sys
import threading
import tkinter as tk
import webbrowser
from pathlib import Path
from tkinter import messagebox, simpledialog, ttk
from tkinter.scrolledtext import ScrolledText
from typing import Callable

__all__ = [
    "ML_LIDAR_DIR", "REPO_ROOT", "RUNS_DIR", "MODELS_DIR", "DEFAULT_N_ENVS",
    "rel", "venv_bin", "list_run_names", "next_run_name", "discover_runs",
    "format_run_row", "read_note", "write_note", "build_train_cmd", "build_export_cmd",
    "build_watch_cmd", "build_tensorboard_cmd", "App",
]

ML_LIDAR_DIR = Path(__file__).resolve().parent
REPO_ROOT = ML_LIDAR_DIR.parent
RUNS_DIR = ML_LIDAR_DIR / "runs"
MODELS_DIR = REPO_ROOT / "models" / "e2e_lidar"
#: `train_rl.py --n-envs`の初期値。物理コア数に合わせる（コア数より増やしても
#: 速くならず、むしろ競合で遅くなりやすい）
DEFAULT_N_ENVS = os.cpu_count() or 8


def rel(p: Path) -> str:
    """`REPO_ROOT`からの相対パス文字列。`ml/app.py`の同名関数と同じ役目。"""
    try:
        return str(p.relative_to(REPO_ROOT))
    except ValueError:
        return str(p)


def venv_bin(python: str, name: str) -> str:
    """`python`（venvのpython実行ファイル）と同じ`bin/`にある別コマンドのパス。
    `tensorboard`は`ml_lidar/requirements.txt`で同じvenvに入る想定。"""
    return str(Path(python).parent / name)


# ── run名・run一覧（Tkinterを一切知らない純粋関数。`ml_lidar/tests/test_app.py`の対象） ──

_V_NAME_RE = re.compile(r"^v(\d+)$")


def list_run_names(runs_dir: Path) -> list[str]:
    if not runs_dir.exists():
        return []
    return sorted(p.name for p in runs_dir.iterdir() if p.is_dir())


def next_run_name(existing: list[str]) -> str:
    """`v1`・`v2`…のうち一番大きい番号の次を提案する。`v`始まりでない名前
    （最初期の`ppo_e2e`など）は無視する。該当が無ければ`v1`。"""
    nums = [int(m.group(1)) for name in existing if (m := _V_NAME_RE.match(name))]
    return f"v{max(nums) + 1}" if nums else "v1"


def discover_runs(runs_dir: Path) -> list[dict]:
    """`runs_dir`直下の各学習runのメタ情報を、更新が新しい順に返す。

    `run_config.json`（`train_rl.py`が書く。無ければ空dict）と、
    `best_model.zip`の有無（観戦・エクスポートに使えるか）を持たせる。
    """
    if not runs_dir.exists():
        return []
    runs = []
    for p in runs_dir.iterdir():
        if not p.is_dir():
            continue
        cfg_path = p / "run_config.json"
        cfg: dict = {}
        if cfg_path.exists():
            try:
                cfg = json.loads(cfg_path.read_text())
            except (json.JSONDecodeError, OSError):
                cfg = {}
        runs.append({
            "name": p.name,
            "has_best_model": (p / "best_model.zip").exists(),
            "config": cfg,
            "mtime": p.stat().st_mtime,
        })
    runs.sort(key=lambda r: r["mtime"], reverse=True)
    return runs


def format_run_row(run: dict) -> str:
    """runs一覧Listboxの1行分の表示文字列。"""
    mark = "✓" if run["has_best_model"] else "…"
    cfg = run["config"]
    # 最大舵角(max_steer)はもう学習ごとに変わらない（常にvehicle.toml由来）ので表示しない
    extra = f"speed={cfg.get('max_speed')}" if cfg else "run_config無し"
    return f"[{mark}] {run['name']}  ({extra})"


def read_note(run_dir: Path) -> str:
    """`<run_dir>/note.txt`の中身。無ければ空文字（バンビが自由に書く備考欄。
    `run_config.json`とは別ファイル——ハイパラの自動記録と人間の自由記述を混ぜない）。"""
    try:
        return (run_dir / "note.txt").read_text(encoding="utf-8")
    except OSError:
        return ""


def write_note(run_dir: Path, text: str) -> None:
    (run_dir / "note.txt").write_text(text, encoding="utf-8")


# ── コマンド組み立て ──

def build_train_cmd(python: str, name: str, *, timesteps: int, n_envs: int,
                    max_speed: float, early_stop_patience: int,
                    resume_from: str | None = None) -> list[str]:
    """最大舵角は渡さない——`train_rl.py`はもう`--max-steer`を持たず、
    `config/vehicle.toml`の車両物理限界を常に使う（2026-08-28、バンビの指示）。

    :param resume_from: 指定すると`--resume-from`を足す（既存のチェックポイントから
        続きを学習する。`train_rl.py`の`PPO.load()`経路）。省略時は従来通り新規学習。
    """
    cmd = [python, str(ML_LIDAR_DIR / "train_rl.py"),
          "--out", rel(RUNS_DIR / name),
          "--timesteps", str(timesteps),
          "--n-envs", str(n_envs),
          "--max-speed", str(max_speed),
          "--early-stop-patience", str(early_stop_patience)]
    if resume_from:
        cmd += ["--resume-from", resume_from]
    return cmd


def build_export_cmd(python: str, run_name: str, model_name: str, *,
                     checkpoint: str = "best_model.zip") -> list[str]:
    """`--max-speed`/`--max-steer`は渡さない——`export_onnx_rl.py`が
    `run_config.json`から自動で読む（2026-08-28に追加した仕組みをそのまま使う）。"""
    return [python, str(ML_LIDAR_DIR / "export_onnx_rl.py"),
           "--model", rel(RUNS_DIR / run_name / checkpoint),
           "--out", rel(MODELS_DIR / f"{model_name}.onnx")]


def build_watch_cmd(python: str, run_name: str, *, checkpoint: str = "best_model.zip",
                    panels: int = 6) -> list[str]:
    return [python, str(ML_LIDAR_DIR / "watch.py"),
           "--model", rel(RUNS_DIR / run_name / checkpoint),
           "--panels", str(panels)]


def build_tensorboard_cmd(tensorboard_bin: str, run_name: str) -> list[str]:
    return [tensorboard_bin, "--logdir", rel(RUNS_DIR / run_name / "tb")]


# ── GUI ──

class App:
    """タブ2枚（学習・run一覧）＋複数ジョブの状態表示＋共有のログ欄。"""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        root.title("SURGE Mk.2 — LiDAR E2E 学習・エクスポート")
        root.geometry("640x640")

        self.python = sys.executable
        #: job_key -> Popen（起動処理中は None）。`ml/app.py`は1個の`self.proc`
        #: だけだったが、ここは学習・TensorBoard・観戦・エクスポートが同時に
        #: 動きうるので辞書で複数追跡する
        self._active: dict[str, subprocess.Popen | None] = {}
        self._job_labels: dict[str, str] = {}
        self._on_done_cbs: dict[str, Callable[[int], None]] = {}
        self._jobs_listbox_keys: list[str] = []
        self.log_queue: "queue.Queue[tuple]" = queue.Queue()
        self._runs: list[dict] = []

        self._build_widgets()
        self.root.after(100, self._drain_log)

    # ── 画面構築 ──

    def _build_widgets(self) -> None:
        nb = ttk.Notebook(self.root)
        nb.pack(fill="x", padx=8, pady=8)
        self._build_train_tab(nb)
        self._build_runs_tab(nb)

        jobs_frame = ttk.LabelFrame(self.root, text="実行中のジョブ")
        jobs_frame.pack(fill="x", padx=8, pady=(0, 4))
        self.jobs_listbox = tk.Listbox(jobs_frame, height=3)
        self.jobs_listbox.pack(side="left", fill="both", expand=True, padx=4, pady=4)
        ttk.Button(jobs_frame, text="選択を停止", command=self._stop_selected_job).pack(
            side="right", padx=4, anchor="n", pady=4)

        log_frame = ttk.LabelFrame(self.root, text="ログ")
        log_frame.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self.log_widget = ScrolledText(log_frame, height=12, state="disabled")
        self.log_widget.pack(fill="both", expand=True, padx=4, pady=4)

        self.status_label = ttk.Label(self.root, text="待機中")
        self.status_label.pack(anchor="w", padx=8, pady=(0, 8))

    def _build_train_tab(self, nb: ttk.Notebook) -> None:
        frame = ttk.Frame(nb, padding=10)
        nb.add(frame, text="① 学習")

        self.name_var = tk.StringVar(value=next_run_name(list_run_names(RUNS_DIR)))
        ttk.Label(frame, text="run名（学習出力先・モデル名を兼ねる）:").grid(
            row=0, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.name_var, width=20).grid(row=0, column=1, sticky="w")
        ttk.Label(frame, text=f"{rel(RUNS_DIR)}/<run名> に出力。②のエクスポートでは\n"
                             "同じ名前をモデル名の初期値として提案する（変更可）",
                 foreground="gray").grid(row=1, column=0, columnspan=2, sticky="w")

        self.timesteps_var = tk.StringVar(value="2000000")
        ttk.Label(frame, text="timesteps:").grid(row=2, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(frame, textvariable=self.timesteps_var, width=12).grid(
            row=2, column=1, sticky="w", pady=(8, 0))

        self.n_envs_var = tk.StringVar(value=str(DEFAULT_N_ENVS))
        ttk.Label(frame, text=f"n_envs（このMacは物理コア{DEFAULT_N_ENVS}個）:").grid(
            row=3, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(frame, textvariable=self.n_envs_var, width=12).grid(
            row=3, column=1, sticky="w", pady=(8, 0))

        self.max_speed_var = tk.StringVar(value="1.5")
        ttk.Label(frame, text="max_speed [m/s]:").grid(row=4, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(frame, textvariable=self.max_speed_var, width=12).grid(
            row=4, column=1, sticky="w", pady=(8, 0))

        self.early_stop_var = tk.StringVar(value="10")
        ttk.Label(frame, text="early_stop_patience（0で無効化）:").grid(
            row=5, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(frame, textvariable=self.early_stop_var, width=12).grid(
            row=5, column=1, sticky="w", pady=(8, 0))

        ttk.Label(frame, text=f"max_steer（最大舵角）は{rel(REPO_ROOT / 'config' / 'vehicle.toml')}"
                             "の車両限界を常に使うため、\nここでは設定しません。",
                 foreground="gray").grid(row=6, column=0, columnspan=2, sticky="w", pady=(8, 0))

        ttk.Button(frame, text="学習開始（数時間かかります）", command=self._start_train).grid(
            row=7, column=0, columnspan=2, sticky="w", pady=12)
        ttk.Label(frame, text="学習中もタブ②からTensorBoard・観戦・（別runの）\n"
                             "エクスポートを並行して動かせます。",
                 foreground="gray").grid(row=8, column=0, columnspan=2, sticky="w")

    def _build_runs_tab(self, nb: ttk.Notebook) -> None:
        frame = ttk.Frame(nb, padding=10)
        nb.add(frame, text="② run一覧")

        top = ttk.Frame(frame)
        top.pack(fill="x")
        ttk.Button(top, text="更新", command=self._refresh_runs).pack(side="left")

        self.runs_listbox = tk.Listbox(frame, height=10, width=72)
        self.runs_listbox.pack(fill="both", expand=True, pady=(6, 0))
        self.runs_listbox.bind("<<ListboxSelect>>", lambda e: self._on_run_select())

        self.run_detail_var = tk.StringVar(value="（runを選んでください）")
        ttk.Label(frame, textvariable=self.run_detail_var, foreground="gray",
                 wraplength=560, justify="left").pack(fill="x", pady=(6, 0), anchor="w")

        btns = ttk.Frame(frame)
        btns.pack(fill="x", pady=(8, 0))
        self.tb_btn = ttk.Button(btns, text="TensorBoardを開く",
                                 command=self._open_tensorboard, state="disabled")
        self.tb_btn.pack(side="left")
        self.watch_btn = ttk.Button(btns, text="観戦を開始", command=self._start_watch,
                                    state="disabled")
        self.watch_btn.pack(side="left", padx=(8, 0))
        ttk.Label(btns, text="窓数:").pack(side="left", padx=(6, 0))
        self.watch_panels_var = tk.StringVar(value="6")
        ttk.Spinbox(btns, from_=1, to=16, textvariable=self.watch_panels_var,
                   width=3).pack(side="left")
        self.export_btn = ttk.Button(btns, text="ONNXエクスポート", command=self._start_export,
                                     state="disabled")
        self.export_btn.pack(side="left", padx=(8, 0))

        note_frame = ttk.LabelFrame(frame, text="備考（どんな変更をしたか・どのコースか等、自由に）")
        note_frame.pack(fill="both", pady=(8, 0))
        self.note_text = tk.Text(note_frame, height=4, wrap="word", state="disabled")
        self.note_text.pack(fill="both", expand=True, padx=4, pady=(4, 0))
        self.note_save_btn = ttk.Button(note_frame, text="備考を保存", command=self._save_note,
                                        state="disabled")
        self.note_save_btn.pack(anchor="e", padx=4, pady=4)

        self._refresh_runs()

    # ── run一覧 ──

    def _refresh_runs(self) -> None:
        self._runs = discover_runs(RUNS_DIR)
        self.runs_listbox.delete(0, "end")
        for r in self._runs:
            self.runs_listbox.insert("end", format_run_row(r))
        self._on_run_select()

    def _selected_run(self) -> dict | None:
        sel = self.runs_listbox.curselection()
        if not sel or sel[0] >= len(self._runs):
            return None
        return self._runs[sel[0]]

    def _on_run_select(self) -> None:
        run = self._selected_run()
        if run is None:
            self.run_detail_var.set("（runを選んでください）")
            self.tb_btn.config(state="disabled")
            self.watch_btn.config(state="disabled")
            self.export_btn.config(state="disabled")
            self.note_text.config(state="normal")
            self.note_text.delete("1.0", "end")
            self.note_text.config(state="disabled")
            self.note_save_btn.config(state="disabled")
            return
        cfg = run["config"]
        if cfg:
            # 最大舵角(max_steer)はもう学習ごとに変わらない（常にvehicle.toml由来）ので表示しない
            self.run_detail_var.set(
                f"max_speed={cfg.get('max_speed')}  "
                f"timesteps={cfg.get('timesteps')}  n_envs={cfg.get('n_envs')}")
        else:
            self.run_detail_var.set("run_config.json が無いrunです（この機能を足す前の学習、"
                                    "または学習開始直後）")
        self.tb_btn.config(state="normal")
        has_model = run["has_best_model"]
        self.watch_btn.config(state="normal" if has_model else "disabled")
        self.export_btn.config(state="normal" if has_model else "disabled")

        self.note_text.config(state="normal")
        self.note_text.delete("1.0", "end")
        self.note_text.insert("1.0", read_note(RUNS_DIR / run["name"]))
        self.note_save_btn.config(state="normal")

    def _save_note(self) -> None:
        run = self._selected_run()
        if run is None:
            return
        write_note(RUNS_DIR / run["name"], self.note_text.get("1.0", "end-1c"))
        self._append_log(f"\n備考を保存しました（{run['name']}）\n")

    def _open_tensorboard(self) -> None:
        run = self._selected_run()
        if run is None:
            return
        cmd = build_tensorboard_cmd(venv_bin(self.python, "tensorboard"), run["name"])
        self._start_job("tensorboard", cmd, f"TensorBoard({run['name']})")
        self.root.after(2500, lambda: webbrowser.open("http://localhost:6006"))

    def _start_watch(self) -> None:
        run = self._selected_run()
        if run is None:
            return
        try:
            panels = int(self.watch_panels_var.get())
        except ValueError:
            messagebox.showerror("入力エラー", "窓数は整数で入力してください")
            return
        cmd = build_watch_cmd(self.python, run["name"], panels=panels)
        self._start_job("watch", cmd, f"観戦({run['name']}, {panels}窓)")

    def _start_export(self) -> None:
        run = self._selected_run()
        if run is None:
            return
        name = simpledialog.askstring(
            "モデル名", f"{rel(MODELS_DIR)}/<名前>.onnx として書き出します",
            initialvalue=run["name"], parent=self.root)
        if not name:
            return
        out_path = MODELS_DIR / f"{name}.onnx"
        if out_path.exists() and not messagebox.askyesno(
                "上書き確認", f"{rel(out_path)} は既にあります。上書きしますか？"):
            return
        cmd = build_export_cmd(self.python, run["name"], name)
        self._start_job("export", cmd, f"エクスポート({name})")

    # ── 学習 ──

    def _start_train(self) -> None:
        name = self.name_var.get().strip()
        if not name:
            messagebox.showerror("入力エラー", "run名を入力してください")
            return
        try:
            timesteps = int(self.timesteps_var.get())
            n_envs = int(self.n_envs_var.get())
            max_speed = float(self.max_speed_var.get())
            early_stop = int(self.early_stop_var.get())
        except ValueError:
            messagebox.showerror("入力エラー", "数値の項目は正しい数値で入力してください")
            return

        resume_from = None
        run_dir = RUNS_DIR / name
        if run_dir.exists():
            # 「はい」＝続きから再開／「いいえ」＝上書きして新規／「キャンセル」＝何もしない、
            # の3択にaskyesnocancelを流用する（専用ダイアログを組むほどの規模ではないため）
            choice = messagebox.askyesnocancel(
                "run名が既存です",
                f"run '{name}' は既に存在します。\n\n"
                "「はい」: 続きから再開する（best_model.zipから読み込む。"
                "--timestepsは累計の目標値として扱われる点に注意）\n"
                "「いいえ」: 上書きして最初から学習し直す\n"
                "「キャンセル」: 何もしない")
            if choice is None:
                return
            if choice:
                best = run_dir / "best_model.zip"
                if not best.exists():
                    messagebox.showerror(
                        "再開できません",
                        f"{rel(best)} が見つかりません"
                        "（学習が最初の評価まで進んでいない可能性があります）")
                    return
                resume_from = rel(best)

        cmd = build_train_cmd(self.python, name, timesteps=timesteps, n_envs=n_envs,
                              max_speed=max_speed, early_stop_patience=early_stop,
                              resume_from=resume_from)
        self._start_job("train", cmd, f"学習({name})",
                        on_done=lambda code: self._on_train_done(code))

    def _on_train_done(self, code: int) -> None:
        if code == 0:
            self._refresh_runs()
            self.name_var.set(next_run_name(list_run_names(RUNS_DIR)))

    # ── サブプロセス実行・ログ配線（複数ジョブ対応） ──

    def _append_log(self, text: str) -> None:
        self.log_widget.config(state="normal")
        self.log_widget.insert("end", text)
        self.log_widget.see("end")
        self.log_widget.config(state="disabled")

    def _start_job(self, job_key: str, cmd: list[str], label: str, *,
                   on_done: Callable[[int], None] | None = None) -> None:
        if job_key in self._active:
            messagebox.showinfo("実行中", f"{label} は既に実行中です（先に停止してください）")
            return
        self._active[job_key] = None                  # 起動処理中プレースホルダ
        self._job_labels[job_key] = label
        if on_done is not None:
            self._on_done_cbs[job_key] = on_done
        self._append_log(f"\n$ {' '.join(cmd)}\n")
        self._refresh_jobs_listbox()

        def worker() -> None:
            code = -1
            try:
                proc = subprocess.Popen(cmd, cwd=str(REPO_ROOT), stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, text=True, bufsize=1)
                self._active[job_key] = proc
                for line in proc.stdout:                # type: ignore[union-attr]
                    self.log_queue.put(("log", f"[{label}] {line}"))
                code = proc.wait()
            except Exception as e:                       # noqa: BLE001 — GUIなので握って表示する
                self.log_queue.put(("log", f"[{label}] 実行に失敗しました: {e}\n"))
            finally:
                self.log_queue.put(("done", job_key, code))

        threading.Thread(target=worker, daemon=True).start()

    def _stop_selected_job(self) -> None:
        sel = self.jobs_listbox.curselection()
        if not sel or sel[0] >= len(self._jobs_listbox_keys):
            return
        job_key = self._jobs_listbox_keys[sel[0]]
        label = self._job_labels.get(job_key, job_key)
        # ★学習だけワンクリックで数時間ぶんの進捗を失いかねないので確認を挟む
        # （TensorBoard・観戦・エクスポートはすぐ止めても実害が小さいので確認しない）
        if job_key == "train" and not messagebox.askyesno(
                "学習を停止しますか？",
                f"{label} を停止します。\n\n"
                "直近の評価（--eval-freq分）より後の進捗は失われます。あとで①タブから"
                "同じrun名を選べば「続きから再開」できますが、完全に同じ結果には"
                "なりません。\n\n本当に停止しますか？"):
            return
        proc = self._active.get(job_key)
        if proc is None:
            messagebox.showinfo("起動中", "まだ起動処理中です。少し待ってから止めてください")
            return
        proc.terminate()
        self._append_log(f"\n[{label}] 停止を指示しました\n")

    def _refresh_jobs_listbox(self) -> None:
        self.jobs_listbox.delete(0, "end")
        self._jobs_listbox_keys = list(self._job_labels.keys())
        for key in self._jobs_listbox_keys:
            self.jobs_listbox.insert("end", self._job_labels[key])
        self.status_label.config(
            text=(", ".join(self._job_labels.values()) + " 実行中") if self._job_labels else "待機中")

    def _drain_log(self) -> None:
        try:
            while True:
                item = self.log_queue.get_nowait()
                if item[0] == "log":
                    self._append_log(item[1])
                elif item[0] == "done":
                    _, job_key, code = item
                    label = self._job_labels.pop(job_key, job_key)
                    self._active.pop(job_key, None)
                    self._append_log(f"\n[{label}] {'完了' if code == 0 else f'終了コード {code}'}\n")
                    cb = self._on_done_cbs.pop(job_key, None)
                    self._refresh_jobs_listbox()
                    if cb is not None:
                        cb(code)
        except queue.Empty:
            pass
        self.root.after(100, self._drain_log)


def main() -> int:
    root = tk.Tk()
    App(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
