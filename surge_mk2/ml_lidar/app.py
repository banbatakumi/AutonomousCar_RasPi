"""ml_lidar/app.py — LiDAR-only E2E強化学習の学習・観戦・エクスポートをターミナル無しで
操作するための最小限のGUI。

    .venv/bin/python ml_lidar/app.py

`ml_lidar/train_rl.py`・`watch.py`・`export_onnx_rl.py`・`eval_stats.py`を
サブプロセスとして呼び出すだけの薄い操作パネル。**学習・推論のロジックは
一切持たない**——`ml_cam/app.py`・`ml_cam_e2e/app.py`と同じ設計方針。

ログ表示・サブプロセス実行基盤・run名管理・note.txt・ウィンドウクローズ処理は
`ml_common/`に切り出してある共通実装をそのまま使う。

## `ml_cam_e2e/app.py`と違い、学習中もTensorBoard・観戦・エクスポートを並行実行できる

`JobRunner(allow_concurrent=True, prefix_labels=True)`——旧`ml_lidar/app.py`の
設計（「1つのrun名で学習出力先・エクスポート先・観戦対象を自動的に紐付ける」
「学習中でもTensorBoard・観戦・エクスポートを並行実行できる」「run一覧タブ」
「備考(note)機能」）は報酬・カリキュラムの複雑化とは別軸の話で、それ自体に
問題は無かったため踏襲する（`ML_LIDAR_V2_PROMPT.md`）。

## 学習タブが縦に長大化しないための方針

旧app.pyは「診断のたびに増えた一時的な切り分けフラグ（`reward_norm`無効化・
`log_std_init`等、日付入りコメント付き）」がGUIに恒久化して縦に長くなった。
ここでは常時表示する基本パラメータ（run名・timesteps・n_envs・速度/舵角上限等）
と、折りたたみ「詳細設定」（PPOハイパラ・ドメインランダム化の詳細）を分離する。
"""

from __future__ import annotations

import subprocess
import sys
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Callable

ML_LIDAR_DIR = Path(__file__).resolve().parent
REPO_ROOT = ML_LIDAR_DIR.parent
sys.path.insert(0, str(REPO_ROOT))

from ml_common.close_handler import confirm_and_close  # noqa: E402
from ml_common.job_runner import JobRunner  # noqa: E402
from ml_common.log_console import LogConsole  # noqa: E402
from ml_common.naming import list_versioned_names, next_versioned_name  # noqa: E402
from ml_common.notes import read_note, write_note  # noqa: E402
from ml_common.paths import make_rel  # noqa: E402
from ml_lidar.run_info import has_existing_run  # noqa: E402

__all__ = [
    "ML_LIDAR_DIR", "REPO_ROOT", "RUNS_DIR", "MODELS_DIR",
    "build_train_cmd", "build_tensorboard_cmd", "build_watch_cmd", "build_live_watch_cmd",
    "build_export_cmd", "build_eval_stats_cmd",
    "describe_run_status", "has_existing_run", "list_exported_model_names", "list_custom_course_names",
    "rel", "App",
]

RUNS_DIR = ML_LIDAR_DIR / "runs"
MODELS_DIR = REPO_ROOT / "models" / "e2e_lidar"
COURSES_DIR = REPO_ROOT / "sim" / "courses"
#: 学習中でも壊れない・止めても惜しくない類のジョブは確認無しで終了時に落とす。
#: 学習だけは数十分〜数時間かかりうるので、ウィンドウを閉じる前に一言確認する
_CONFIRM_STOP_JOB_KEYS = frozenset({"train"})

rel = make_rel(REPO_ROOT)


def describe_run_status(run_dir: Path) -> str:
    """`eval/evaluations.npz`（`EvalCallback`が書く）があれば、直近のbest評価を要約する。"""
    npz_path = run_dir / "eval" / "evaluations.npz"
    if not npz_path.exists():
        return "未学習"
    try:
        import numpy as np
        d = np.load(npz_path)
        best = float(d["results"].mean(axis=1).max())
        ts = int(d["timesteps"][-1])
    except Exception as e:                                            # noqa: BLE001
        return f"評価ログの読み込みに失敗: {e}"
    mark = "✓ONNX済" if (run_dir / "final_model.zip").exists() else ""
    return f"{ts:,}steps・best_reward={best:.1f} {mark}".strip()


def list_exported_model_names() -> list[str]:
    return sorted(p.stem for p in MODELS_DIR.glob("*.onnx")) if MODELS_DIR.is_dir() else []


def list_custom_course_names() -> list[str]:
    """`sim/courses/`のうち手作業エディタ製の実コース（PNG占有格子ではない）だけを挙げる。

    学習には一切使わない観戦専用（`ML_LIDAR_V2_PROMPT.md`）——ここに並ぶのは目視確認用。
    """
    if not COURSES_DIR.is_dir():
        return []
    pngs = {p.stem for p in COURSES_DIR.glob("*.png")}
    return sorted(p.stem for p in COURSES_DIR.glob("*.json") if p.stem not in pngs)


# ── コマンド組み立て（Tkinter を一切知らない純粋関数） ──

def build_train_cmd(python: str, run_name: str, args: dict) -> list[str]:
    """`args`は`--total-timesteps`等のダッシュ付きキー名→値。boolはTrueのときのみフラグを立てる。"""
    cmd = [python, "-m", "ml_lidar.train_rl", "--run-name", run_name]
    for key, value in args.items():
        if isinstance(value, bool):
            if value:
                cmd.append(f"--{key}")
        else:
            cmd += [f"--{key}", str(value)]
    return cmd


def build_tensorboard_cmd(tensorboard_bin: str, logdir: Path, port: int) -> list[str]:
    return [tensorboard_bin, "--logdir", str(logdir), "--port", str(port)]


def build_watch_cmd(python: str, model_name: str, course_name: str | None) -> list[str]:
    cmd = [python, "-m", "ml_lidar.watch", "--model", model_name]
    if course_name:
        cmd += ["--course", str(COURSES_DIR / f"{course_name}.json")]
    return cmd


def build_live_watch_cmd(python: str, run_name: str) -> list[str]:
    return [python, "-m", "ml_lidar.live_watch", "--run-name", run_name]


def build_export_cmd(python: str, model_path: str, name: str) -> list[str]:
    return [python, "-m", "ml_lidar.export_onnx_rl", "--model", model_path, "--name", name]


def build_eval_stats_cmd(python: str, model_path: str, episodes: int) -> list[str]:
    return [python, "-m", "ml_lidar.eval_stats", "--model", model_path, "--episodes", str(episodes)]


# ── GUI ──

class App:
    """タブ4枚（学習・観戦・エクスポート・run一覧）＋共有のログ欄・実行中ジョブ一覧。"""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        root.title("SURGE Mk.2 — ml_lidar 学習・観戦・エクスポート")
        root.geometry("820x760")

        self.python = sys.executable
        self.tensorboard_bin = str(Path(self.python).parent / "tensorboard")
        self.jobs = JobRunner(REPO_ROOT, allow_concurrent=True, prefix_labels=True)

        self._build_widgets()
        self.root.after(100, self._drain_log)
        self.root.protocol("WM_DELETE_WINDOW", lambda: confirm_and_close(
            self.root, self.jobs, confirm_job_keys=_CONFIRM_STOP_JOB_KEYS,
            confirm_message=lambda label: f"{label}が実行中です。終了すると学習が中断されます。"
            "\n\n本当に終了しますか？"))

    # ── 画面構築 ──

    def _build_widgets(self) -> None:
        self.run_name_var = tk.StringVar(value=next_versioned_name(list_versioned_names(RUNS_DIR)))
        self.run_status_var = tk.StringVar(value="")

        nb = ttk.Notebook(self.root)
        nb.pack(fill="both", expand=True, padx=8, pady=8)
        self._build_train_tab(nb)
        self._build_watch_tab(nb)
        self._build_export_tab(nb)
        self._build_runs_tab(nb)

        self.run_name_var.trace_add("write", self._on_run_name_changed)
        self._on_run_name_changed()

        log_frame = ttk.LabelFrame(self.root, text="ログ")
        log_frame.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self.log_console = LogConsole(log_frame, height=12)
        self.log_console.widget.pack(fill="both", expand=True, padx=4, pady=4)

        jobs_frame = ttk.LabelFrame(self.root, text="実行中のジョブ")
        jobs_frame.pack(fill="x", padx=8, pady=(0, 8))
        self.jobs_listbox = tk.Listbox(jobs_frame, height=4)
        self.jobs_listbox.pack(side="left", fill="both", expand=True, padx=4, pady=4)
        ttk.Button(jobs_frame, text="選択を停止", command=self._stop_selected).pack(
            side="right", padx=4, pady=4, anchor="n")

    # ── run名（①③タブ・run一覧タブで共有） ──

    def _run_dir(self) -> Path | None:
        name = self.run_name_var.get().strip()
        if not name:
            messagebox.showerror("入力エラー", "run名を入力してください")
            return None
        return RUNS_DIR / name

    def _on_run_name_changed(self, *_args) -> None:
        name = self.run_name_var.get().strip()
        if name:
            run_dir = RUNS_DIR / name
            self.run_status_var.set(describe_run_status(run_dir))
            note = read_note(run_dir)
            self.export_name_var.set(name)
        else:
            self.run_status_var.set("")
            note = ""
        self.note_text.delete("1.0", "end")
        self.note_text.insert("1.0", note)

    def _save_note(self) -> None:
        run_dir = self._run_dir()
        if run_dir is None:
            return
        run_dir.mkdir(parents=True, exist_ok=True)
        write_note(run_dir, self.note_text.get("1.0", "end-1c"))
        self._append_log(f"\n備考を保存しました（{self.run_name_var.get().strip()}）\n")

    def _refresh_run_names(self) -> None:
        names = list_versioned_names(RUNS_DIR)
        self.run_combo["values"] = names
        self.export_run_combo["values"] = names
        self.runs_listbox_names = names
        self.runs_listbox.delete(0, "end")
        for n in names:
            status = describe_run_status(RUNS_DIR / n)
            self.runs_listbox.insert("end", f"{n} — {status}")

    # ── ① 学習タブ ──

    def _build_train_tab(self, nb: ttk.Notebook) -> None:
        frame = ttk.Frame(nb, padding=10)
        nb.add(frame, text="① 学習")

        ttk.Label(frame, text="run名:").grid(row=0, column=0, sticky="w")
        self.run_combo = ttk.Combobox(frame, textvariable=self.run_name_var, width=20)
        self.run_combo.grid(row=0, column=1, sticky="w")
        self.run_combo["postcommand"] = self._refresh_run_names
        ttk.Label(frame, textvariable=self.run_status_var, foreground="gray").grid(
            row=1, column=0, columnspan=3, sticky="w", pady=(2, 8))

        basic = {
            "total-timesteps": ("学習ステップ総数", "1000000"),
            "n-envs": ("並列環境数", "8"),
            "episode-s": ("1エピソードの打ち切り秒数", "30"),
            "max-speed": ("最高速度[m/s]", "2.0"),
            "steer-rate-weight": ("ステア変化率ペナルティ重み", "0"),
            "sde-sample-freq": ("gSDEノイズ再サンプリング間隔[step]", "4"),
            "log-std-init": ("探索ノイズ初期標準偏差(log)", "-3"),
            "seed": ("乱数シード", "0"),
        }
        # 最大舵角は`config/vehicle.toml`固定（`ml_lidar/env.py`のEnvConfig docstring参照）
        # なので、ここには置かない
        self._train_basic_vars: dict[str, tk.StringVar] = {}
        for i, (key, (label, default)) in enumerate(basic.items(), start=2):
            var = tk.StringVar(value=default)
            self._train_basic_vars[key] = var
            ttk.Label(frame, text=f"{label}:").grid(row=i, column=0, sticky="w", pady=2)
            ttk.Entry(frame, textvariable=var, width=14).grid(row=i, column=1, sticky="w", pady=2)
            if key == "steer-rate-weight":
                ttk.Label(frame, foreground="gray", wraplength=380, justify="left",
                         text="0で無効。v7〜v10でL1罰則の重みを0/0.03/0.1/0.2と振ったところ"
                              "重みを付けるほど単調に悪化した(2026-09-09実測、PROGRESS.md参照)。"
                              "gSDEを検証する間は切り分けのため0のままにすること").grid(
                    row=i, column=2, sticky="w", padx=(8, 0), pady=2)
            if key == "sde-sample-freq":
                ttk.Label(frame, foreground="gray", wraplength=380, justify="left",
                         text="下の「gSDEを有効化」がオフなら無視される。小さいほど通常の"
                              "毎ステップノイズに近づき、大きいほど探索が単調になる").grid(
                    row=i, column=2, sticky="w", padx=(8, 0), pady=2)
            if key == "log-std-init":
                ttk.Label(frame, foreground="gray", wraplength=380, justify="left",
                         text="gSDE有効時、実効ノイズがsqrt(方策潜在層の次元)倍に膨らむため"
                              "既定0.0のままだとv11のように学習が崩壊する(2026-09-10実測)。"
                              "-3前後を既定にしてある。gSDE無効なら影響小さいが変更しなくてよい").grid(
                    row=i, column=2, sticky="w", padx=(8, 0), pady=2)

        use_sde_row = 2 + len(basic)
        self.use_sde_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(frame, text="gSDE(状態依存の相関探索ノイズ)を有効化",
                        variable=self.use_sde_var).grid(
            row=use_sde_row, column=0, columnspan=2, sticky="w", pady=(4, 2))

        next_row = use_sde_row + 1
        adv_toggle_row = next_row
        adv_body = ttk.Frame(frame)
        adv_shown = tk.BooleanVar(value=False)

        def _toggle_advanced() -> None:
            if adv_shown.get():
                adv_body.grid_remove()
                adv_shown.set(False)
                adv_btn.config(text="▶ 詳細設定（PPOハイパラ・ドメインランダム化）")
            else:
                adv_body.grid()
                adv_shown.set(True)
                adv_btn.config(text="▼ 詳細設定（PPOハイパラ・ドメインランダム化）")

        adv_btn = ttk.Button(frame, text="▶ 詳細設定（PPOハイパラ・ドメインランダム化）",
                             command=_toggle_advanced)
        adv_btn.grid(row=adv_toggle_row, column=0, columnspan=3, sticky="w", pady=(6, 0))
        adv_body.grid(row=adv_toggle_row + 1, column=0, columnspan=3, sticky="w")
        adv_body.grid_remove()

        advanced = {
            "fov-deg": ("視野角[deg]", "270"),
            "max-range": ("LiDAR最大レンジ[m]", "10.0"),
            "steer-tau": ("舵の平滑化[s]", "0.10"),
            "dynamics-jitter-frac": ("車両動特性のランダム化幅[割合]", "0.2"),
            "eval-freq": ("eval間隔[step]", "20000"),
            "n-eval-episodes": ("eval時のエピソード数(3の倍数推奨)", "9"),
            "checkpoint-freq": ("チェックポイント間隔[step]", "100000"),
            "learning-rate": ("学習率", "0.0003"),
            "n-steps": ("ロールアウト長", "2048"),
            "batch-size": ("バッチサイズ", "256"),
            "n-epochs": ("1更新あたりのエポック数", "10"),
            "gamma": ("割引率", "0.99"),
            "gae-lambda": ("GAE lambda", "0.95"),
            "clip-range": ("PPO clip range", "0.2"),
            "ent-coef": ("エントロピー係数", "0.0"),
            "features-dim": ("特徴抽出器の出力次元", "128"),
            "device": ("device", "auto"),
        }
        self._train_adv_vars: dict[str, tk.StringVar] = {}
        for i, (key, (label, default)) in enumerate(advanced.items()):
            var = tk.StringVar(value=default)
            self._train_adv_vars[key] = var
            ttk.Label(adv_body, text=f"{label}:").grid(row=i, column=0, sticky="w", pady=1)
            ttk.Entry(adv_body, textvariable=var, width=14).grid(row=i, column=1, sticky="w", pady=1)

        no_lidar_var = tk.BooleanVar(value=False)
        no_dyn_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(adv_body, text="LiDARノイズ・欠損を無効化",
                        variable=no_lidar_var).grid(
            row=len(advanced), column=0, columnspan=2, sticky="w", pady=(4, 0))
        ttk.Checkbutton(adv_body, text="車両動特性のランダム化を無効化",
                        variable=no_dyn_var).grid(
            row=len(advanced) + 1, column=0, columnspan=2, sticky="w")

        def run() -> None:
            run_name = self.run_name_var.get().strip()
            if not run_name:
                messagebox.showerror("入力エラー", "run名を入力してください")
                return

            run_dir = RUNS_DIR / run_name
            if has_existing_run(run_dir):
                choice = messagebox.askyesnocancel(
                    "既存の学習結果があります",
                    f"run「{run_name}」には既に学習結果があります。\n\n"
                    "「はい」= 続きから学習（最新チェックポイントから追加学習）\n"
                    "「いいえ」= 0から上書きして学習（既存の結果は消えます）\n"
                    "「キャンセル」= 何もしない")
                if choice is None:
                    return
                resume_or_overwrite = "resume" if choice else "overwrite"
            else:
                resume_or_overwrite = None

            args: dict = {}
            try:
                for key, var in {**self._train_basic_vars, **self._train_adv_vars}.items():
                    text = var.get().strip()
                    args[key] = text
            except Exception as e:                                    # noqa: BLE001
                messagebox.showerror("入力エラー", str(e))
                return
            args["no-randomize-lidar"] = no_lidar_var.get()
            args["no-randomize-dynamics"] = no_dyn_var.get()
            args["use-sde"] = self.use_sde_var.get()
            if resume_or_overwrite == "resume":
                args["resume"] = True
            elif resume_or_overwrite == "overwrite":
                args["overwrite"] = True
            cmd = build_train_cmd(self.python, run_name, args)
            self._run("train", cmd, "学習")

        ttk.Button(frame, text="学習開始", command=run).grid(
            row=adv_toggle_row + 2, column=0, sticky="w", pady=(10, 4))

        def open_tensorboard() -> None:
            run_dir = self._run_dir()
            if run_dir is None:
                return
            cmd = build_tensorboard_cmd(self.tensorboard_bin, run_dir / "tb", 6006)
            self._run("tensorboard", cmd, "TensorBoard")
            import webbrowser
            self.root.after(1500, lambda: webbrowser.open("http://localhost:6006"))

        ttk.Button(frame, text="TensorBoardを開く", command=open_tensorboard).grid(
            row=adv_toggle_row + 2, column=1, sticky="w", pady=(10, 4))

        note_frame = ttk.LabelFrame(frame, text="備考（どんな変更をしたか等、自由に）")
        note_frame.grid(row=adv_toggle_row + 3, column=0, columnspan=3, sticky="we", pady=(8, 0))
        self.note_text = tk.Text(note_frame, height=3, wrap="word")
        self.note_text.pack(fill="both", expand=True, padx=4, pady=(4, 0))
        ttk.Button(note_frame, text="備考を保存", command=self._save_note).pack(
            anchor="e", padx=4, pady=4)

    # ── ② 観戦タブ ──

    def _build_watch_tab(self, nb: ttk.Notebook) -> None:
        frame = ttk.Frame(nb, padding=10)
        nb.add(frame, text="② 観戦")

        live_frame = ttk.LabelFrame(frame, text="学習中runをライブ観戦（自動更新・複数コース並列）")
        live_frame.grid(row=0, column=0, columnspan=3, sticky="we")
        ttk.Label(live_frame,
                 text="学習中の`checkpoints/`を自動で追いかけて表示します（エクスポート不要）。\n"
                      "直線主体・コーナー主体・中間の3コースを同時に表示します。\n"
                      "学習と同時に見るとCPUを取り合うため、学習速度は多少落ちます。",
                 foreground="gray").grid(row=0, column=0, columnspan=3, sticky="w", padx=4, pady=(4, 0))

        self.live_watch_run_var = tk.StringVar(value="")
        ttk.Label(live_frame, text="run名:").grid(row=1, column=0, sticky="w", padx=4, pady=(6, 6))
        self.live_watch_run_combo = ttk.Combobox(live_frame, textvariable=self.live_watch_run_var,
                                                 width=20)
        self.live_watch_run_combo.grid(row=1, column=1, sticky="w", pady=(6, 6))
        self.live_watch_run_combo["postcommand"] = lambda: self.live_watch_run_combo.configure(
            values=list_versioned_names(RUNS_DIR))

        def run_live() -> None:
            name = self.live_watch_run_var.get().strip()
            if not name:
                messagebox.showwarning("未選択", "run名を選んでください")
                return
            cmd = build_live_watch_cmd(self.python, name)
            self._run("live_watch", cmd, "ライブ観戦")

        ttk.Button(live_frame, text="ライブ観戦開始（別ウィンドウ）", command=run_live).grid(
            row=1, column=2, sticky="w", padx=4, pady=(6, 6))

        final_frame = ttk.LabelFrame(frame, text="最終確認（エクスポート済みモデル）")
        final_frame.grid(row=1, column=0, columnspan=3, sticky="we", pady=(10, 0))
        ttk.Label(final_frame, text="実車と同じ推論コード（`raspi/auto/e2e_lidar.py`）で"
                                   "別ウィンドウに走行を表示します。",
                 foreground="gray").grid(row=0, column=0, columnspan=3, sticky="w", padx=4, pady=(4, 0))

        self.watch_model_var = tk.StringVar(value="")
        ttk.Label(final_frame, text="モデル:").grid(row=1, column=0, sticky="w", padx=4, pady=(8, 0))
        self.watch_model_combo = ttk.Combobox(final_frame, textvariable=self.watch_model_var,
                                              state="readonly", width=30)
        self.watch_model_combo.grid(row=1, column=1, sticky="w", pady=(8, 0))
        self.watch_model_combo["postcommand"] = lambda: self.watch_model_combo.configure(
            values=list_exported_model_names())

        self.watch_course_var = tk.StringVar(value="（ランダムコース）")
        ttk.Label(final_frame, text="コース:").grid(row=2, column=0, sticky="w", padx=4, pady=(8, 0))
        self.watch_course_combo = ttk.Combobox(final_frame, textvariable=self.watch_course_var,
                                               state="readonly", width=30)
        self.watch_course_combo.grid(row=2, column=1, sticky="w", pady=(8, 0))
        self.watch_course_combo["postcommand"] = lambda: self.watch_course_combo.configure(
            values=["（ランダムコース）", *list_custom_course_names()])
        ttk.Label(final_frame, text="自作コース（`sim/courses/`）は目視確認専用で、学習には使いません",
                 foreground="gray").grid(row=3, column=0, columnspan=3, sticky="w", padx=4)

        def run() -> None:
            name = self.watch_model_var.get().strip()
            if not name:
                messagebox.showwarning("未選択", "モデルを選んでください（③でエクスポートが必要です）")
                return
            course = self.watch_course_var.get().strip()
            course_name = None if (not course or course == "（ランダムコース）") else course
            cmd = build_watch_cmd(self.python, name, course_name)
            self._run("watch", cmd, "観戦")

        ttk.Button(final_frame, text="観戦開始（別ウィンドウが開きます）", command=run).grid(
            row=4, column=0, columnspan=2, sticky="w", padx=4, pady=10)

    # ── ③ エクスポートタブ ──

    def _build_export_tab(self, nb: ttk.Notebook) -> None:
        frame = ttk.Frame(nb, padding=10)
        nb.add(frame, text="③ エクスポート")

        ttk.Label(frame, text="run名:").grid(row=0, column=0, sticky="w")
        self.export_run_var = tk.StringVar(value="")
        self.export_run_combo = ttk.Combobox(frame, textvariable=self.export_run_var, width=20)
        self.export_run_combo.grid(row=0, column=1, sticky="w")
        self.export_run_combo["postcommand"] = self._refresh_run_names
        self.export_run_var.trace_add("write", lambda *_: self.run_name_var.set(
            self.export_run_var.get()))

        self.export_source_var = tk.StringVar(value="best_model")
        ttk.Label(frame, text="元にするモデル:").grid(row=1, column=0, sticky="w", pady=(8, 0))
        src_frame = ttk.Frame(frame)
        src_frame.grid(row=1, column=1, columnspan=2, sticky="w", pady=(8, 0))
        ttk.Radiobutton(src_frame, text="best_model（eval最良）", variable=self.export_source_var,
                        value="best_model").pack(side="left")
        ttk.Radiobutton(src_frame, text="final_model（学習終了時点）", variable=self.export_source_var,
                        value="final_model").pack(side="left", padx=(10, 0))

        ttk.Label(frame, text="出力名（models/e2e_lidar/<name>.onnx）:").grid(
            row=2, column=0, sticky="w", pady=(8, 0))
        self.export_name_var = tk.StringVar(value="")
        ttk.Entry(frame, textvariable=self.export_name_var, width=22).grid(
            row=2, column=1, sticky="w", pady=(8, 0))

        def source_path() -> Path | None:
            run_dir = RUNS_DIR / self.export_run_var.get().strip()
            if self.export_source_var.get() == "best_model":
                p = run_dir / "eval" / "best_model.zip"
            else:
                p = run_dir / "final_model.zip"
            if not p.exists():
                messagebox.showerror("未学習", f"{rel(p)} が見つかりません。①タブで学習してください")
                return None
            return p

        def run_export() -> None:
            if not self.export_run_var.get().strip():
                messagebox.showerror("入力エラー", "run名を選んでください")
                return
            src = source_path()
            if src is None:
                return
            name = self.export_name_var.get().strip()
            if not name:
                messagebox.showerror("入力エラー", "出力名を入力してください")
                return
            cmd = build_export_cmd(self.python, str(src), name)
            self._run("export", cmd, "エクスポート", on_done=lambda _c: None)

        ttk.Button(frame, text="エクスポート実行", command=run_export).grid(
            row=3, column=0, sticky="w", pady=10)

        ttk.Separator(frame, orient="horizontal").grid(
            row=4, column=0, columnspan=3, sticky="we", pady=8)

        ttk.Label(frame, text="複数シード・複数コースでの統計評価（完走率・衝突率など）",
                 foreground="gray").grid(row=5, column=0, columnspan=3, sticky="w")
        self.eval_episodes_var = tk.StringVar(value="60")
        ttk.Label(frame, text="エピソード数:").grid(row=6, column=0, sticky="w", pady=(4, 0))
        ttk.Entry(frame, textvariable=self.eval_episodes_var, width=10).grid(
            row=6, column=1, sticky="w", pady=(4, 0))

        def run_eval_stats() -> None:
            src = source_path()
            if src is None:
                return
            try:
                episodes = int(self.eval_episodes_var.get())
            except ValueError:
                messagebox.showerror("入力エラー", "エピソード数は整数で入力してください")
                return
            cmd = build_eval_stats_cmd(self.python, str(src), episodes)
            self._run("eval_stats", cmd, "統計評価")

        ttk.Button(frame, text="統計評価を実行", command=run_eval_stats).grid(
            row=7, column=0, sticky="w", pady=(6, 0))

    # ── ④ run一覧タブ ──

    def _build_runs_tab(self, nb: ttk.Notebook) -> None:
        frame = ttk.Frame(nb, padding=10)
        nb.add(frame, text="④ run一覧")

        self.runs_listbox_names: list[str] = []
        self.runs_listbox = tk.Listbox(frame, height=14)
        self.runs_listbox.grid(row=0, column=0, columnspan=3, sticky="nsew")
        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)

        def selected_name() -> str | None:
            sel = self.runs_listbox.curselection()
            if not sel or sel[0] >= len(self.runs_listbox_names):
                return None
            return self.runs_listbox_names[sel[0]]

        def use_selected() -> None:
            name = selected_name()
            if name:
                self.run_name_var.set(name)
                messagebox.showinfo("選択", f"「{name}」を①③タブのrun名に設定しました")

        def open_folder() -> None:
            name = selected_name()
            if not name:
                return
            path = RUNS_DIR / name
            path.mkdir(parents=True, exist_ok=True)
            try:
                subprocess.Popen(["open", str(path)])
            except Exception as e:                                    # noqa: BLE001
                messagebox.showerror("失敗", f"フォルダを開けませんでした: {e}")

        btns = ttk.Frame(frame)
        btns.grid(row=1, column=0, columnspan=3, sticky="w", pady=(6, 0))
        ttk.Button(btns, text="更新", command=self._refresh_run_names).pack(side="left")
        ttk.Button(btns, text="①③タブのrun名にする", command=use_selected).pack(
            side="left", padx=(6, 0))
        ttk.Button(btns, text="フォルダを開く", command=open_folder).pack(side="left", padx=(6, 0))

        self._refresh_run_names()

    # ── サブプロセス実行・ログ配線 ──

    def _append_log(self, text: str) -> None:
        self.log_console.append(text)

    def _run(self, job_key: str, cmd: list[str], label: str, *,
            on_line: Callable[[str], None] | None = None,
            on_done: Callable[[int], None] | None = None) -> None:
        self._append_log(f"\n$ {' '.join(cmd)}\n")
        ok = self.jobs.start(job_key, cmd, label, on_line=on_line,
                             on_done=lambda code: self._on_job_done(job_key, label, code, on_done))
        if not ok:
            messagebox.showwarning("実行中", f"{label}は既に実行中です")
        self._refresh_jobs_listbox()

    def _on_job_done(self, job_key: str, label: str, code: int,
                     extra: Callable[[int], None] | None) -> None:
        if job_key == "train" and code == 0:
            self.run_status_var.set(describe_run_status(RUNS_DIR / self.run_name_var.get().strip()))
        if job_key == "export" and code == 0:
            self.watch_model_combo.configure(values=list_exported_model_names())
        if extra is not None:
            extra(code)
        self._refresh_jobs_listbox()

    def _stop_selected(self) -> None:
        sel = self.jobs_listbox.curselection()
        if not sel:
            return
        active = self.jobs.active_jobs()
        if sel[0] >= len(active):
            return
        job_key, label = active[sel[0]]
        if self.jobs.stop(job_key):
            self._append_log(f"\n（{label} へ停止を指示しました）\n")

    def _refresh_jobs_listbox(self) -> None:
        self.jobs_listbox.delete(0, "end")
        for job_key, label in self.jobs.active_jobs():
            self.jobs_listbox.insert("end", f"{label} [{job_key}]")

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
