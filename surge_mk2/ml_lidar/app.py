"""ml_lidar/app.py — LiDAR-only E2E強化学習の学習・観戦・エクスポートをターミナル無しで
操作するための最小限のGUI。

    .venv/bin/python ml_lidar/app.py

`ml_lidar/train_rl.py`・`watch.py`・`export_onnx_rl.py`・`eval_stats.py`を
サブプロセスとして呼び出すだけの薄い操作パネル。**学習・推論のロジックは
一切持たない**——`ml_cam_e2e/app.py`と同じ設計方針。

ログ表示・サブプロセス実行基盤（ジョブの実行・備考は `ml_common/job_panel.JobPanelBase` の並行実行の形）・
run名管理・note.txt・ウィンドウクローズ処理は `ml_common/`に切り出してある共通実装をそのまま使う。

## `ml_cam_e2e/app.py`と違い、学習中もTensorBoard・観戦・エクスポートを並行実行できる

`JobRunner(allow_concurrent=True, prefix_labels=True)`——旧`ml_lidar/app.py`の
設計（「1つのrun名で学習出力先・エクスポート先・観戦対象を自動的に紐付ける」
「学習中でもTensorBoard・観戦・エクスポートを並行実行できる」「run一覧タブ」
「備考(note)機能」）は報酬・カリキュラムの複雑化とは別軸の話で、それ自体に
問題は無かったため踏襲する。

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
from tkinter import filedialog, messagebox, ttk

ML_LIDAR_DIR = Path(__file__).resolve().parent
REPO_ROOT = ML_LIDAR_DIR.parent
sys.path.insert(0, str(REPO_ROOT))

from ml_common.job_panel import CONFIRM_STOP_JOB_KEYS, JobPanelBase  # noqa: E402
from ml_common.log_window import LogWindow  # noqa: E402
from ml_common.naming import latest_run_name, list_versioned_names, next_versioned_name  # noqa: E402
from ml_common.paths import make_rel  # noqa: E402
from ml_common.sys_monitor import SysMonitorWindow  # noqa: E402
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
from ml_lidar.real_courses import load_real_course  # noqa: E402
#: 学習タブ「探索ノイズ初期標準偏差(log)」の既定（gSDE OFF時）。gSDEのチェックで-3と
#: 相互に切り替える判定にも使うので、表示値と同じ文字列で持つ
_LOG_STD_INIT_DEFAULT = "-1.0"
#: 閉じるときに確認するジョブ（学習だけ。`ml_common/job_panel.py`）
_CONFIRM_STOP_JOB_KEYS = CONFIRM_STOP_JOB_KEYS

rel = make_rel(REPO_ROOT)


def describe_run_status(run_dir: Path) -> str:
    """runの要約。**mean_rewardではなくラップ比を出す**（`eval/generalization_evaluations.npz`）。

    この環境のmean_rewardは1周ぶんの弧長=`total_length/dt`でほぼ定数になり、モデルの
    良し悪しを区別できない（v21の実測は全区間175〜187で振れただけなのに、対MCL理想
    ラップ比は2.15→1.02まで改善していた）。run一覧にrewardを出すと**進捗が止まって
    見える**ので、第一指標のラップ比を出す。旧runには列が無いのでrewardへフォールバックする。
    """
    gen_path = run_dir / "eval" / "generalization_evaluations.npz"
    npz_path = run_dir / "eval" / "evaluations.npz"
    if not gen_path.exists() and not npz_path.exists():
        return "未学習"
    # 実際に書き出された .onnx（models/e2e_lidar/<run名>.onnx か <run名>_*.onnx）があるときだけ
    n = run_dir.name
    exported = MODELS_DIR.is_dir() and any(
        p.stem == n or p.stem.startswith(n + "_") for p in MODELS_DIR.glob("*.onnx"))
    mark = "✓ONNX済" if exported else ""
    try:
        import numpy as np
        if gen_path.exists():
            d = np.load(gen_path)
            ts = int(d["timesteps"][-1])
            if "mean_lap_ratio" in d:
                lr = np.asarray(d["mean_lap_ratio"], dtype=float)
                if np.any(np.isfinite(lr)):
                    return (f"{ts:,}steps・lap比 最良={np.nanmin(lr):.3f} "
                            f"直近={lr[-1]:.3f} {mark}").strip()
            return f"{ts:,}steps・衝突率={float(d['collision_rate'][-1]):.1%} {mark}".strip()
        d = np.load(npz_path)
        best = float(d["results"].mean(axis=1).max())
        ts = int(d["timesteps"][-1])
    except Exception as e:                                            # noqa: BLE001
        return f"評価ログの読み込みに失敗: {e}"
    return f"{ts:,}steps・best_reward={best:.1f} {mark}".strip()


def list_exported_model_names() -> list[str]:
    return sorted(p.stem for p in MODELS_DIR.glob("*.onnx")) if MODELS_DIR.is_dir() else []


def list_custom_course_names() -> list[str]:
    """`sim/courses/`のうち手作業エディタ製の実コース（PNG占有格子ではない）だけを挙げる。

    学習には一切使わない観戦専用——ここに並ぶのは目視確認用。
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

class App(JobPanelBase):
    """タブ4枚（学習・観戦・エクスポート・run一覧）＋共有のログ欄・実行中ジョブ一覧。"""

    model_name_hint = "run名を入力してください"

    def __init__(self, root: tk.Tk) -> None:
        self._init_job_panel(root, REPO_ROOT, allow_concurrent=True, prefix_labels=True,
                             confirm_job_keys=_CONFIRM_STOP_JOB_KEYS)
        root.title("SURGE Mk.2 — ml_lidar 学習・観戦・エクスポート")
        root.geometry("820x560")

        self.tensorboard_bin = str(Path(self.python).parent / "tensorboard")
        self._suppress_export_sync = False
        self.sys_monitor = SysMonitorWindow(self.root)

        self._build_widgets()
        self._start_job_panel()

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

        self.log_window = LogWindow(self.root, height=24)
        self.log_sink = self.log_window
        self._bind_model_name(self.run_name_var)

        jobs_frame = ttk.LabelFrame(self.root, text="実行中のジョブ")
        jobs_frame.pack(fill="x", padx=8, pady=(0, 8))
        self.jobs_listbox = tk.Listbox(jobs_frame, height=4)
        self.jobs_listbox.pack(side="left", fill="both", expand=True, padx=4, pady=4)
        btns_frame = ttk.Frame(jobs_frame)
        btns_frame.pack(side="right", fill="y", padx=4, pady=4)
        ttk.Button(btns_frame, text="選択を停止", command=self._stop_selected).pack(
            fill="x", pady=(0, 2))
        ttk.Button(btns_frame, text="ログを表示", command=self.log_window.show).pack(
            fill="x", pady=2)
        ttk.Button(btns_frame, text="CPU/メモリを表示", command=self.sys_monitor.show).pack(
            fill="x", pady=(2, 0))

    # ── run名（①③タブ・run一覧タブで共有） ──

    def _current_model_dir(self) -> Path | None:
        name = self.run_name_var.get().strip()
        return RUNS_DIR / name if name else None

    def _on_model_name_changed(self) -> None:
        run_dir = self._current_model_dir()
        if run_dir is not None:
            self.run_status_var.set(describe_run_status(run_dir))
            self.export_name_var.set(run_dir.name)
        else:
            self.run_status_var.set("")
        self._show_note(run_dir)

    def _refresh_run_names(self) -> None:
        names = list_versioned_names(RUNS_DIR)
        self.run_combo["values"] = names
        self.export_run_combo["values"] = names
        if not self.export_run_var.get().strip() and names:
            self._suppress_export_sync = True
            self.export_run_var.set(latest_run_name(RUNS_DIR))
            self._suppress_export_sync = False
        self.runs_listbox_names = names
        self.runs_listbox.delete(0, "end")
        for n in names:
            status = describe_run_status(RUNS_DIR / n)
            self.runs_listbox.insert("end", f"{n} — {status}")

    def _scrollable_tab(self, nb: ttk.Notebook, text: str) -> ttk.Frame:
        """縦スクロールできるタブを作り、中身を載せる`Frame`を返す（内容がウィンドウより
        高くなっても下のボタンまで届くように。ホイール・トラックパッドでもスクロールする）。"""
        outer = ttk.Frame(nb)
        nb.add(outer, text=text)
        canvas = tk.Canvas(outer, highlightthickness=0)
        bar = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=bar.set)
        bar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        inner = ttk.Frame(canvas, padding=10)
        win = canvas.create_window((0, 0), window=inner, anchor="nw")
        inner.bind("<Configure>", lambda _e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda e: canvas.itemconfigure(win, width=e.width))

        def _wheel(e: tk.Event) -> None:
            if inner.winfo_height() <= canvas.winfo_height():
                return
            delta = -1 * (e.delta if sys.platform == "darwin" else e.delta // 120)
            canvas.yview_scroll(delta, "units")

        outer.bind("<Enter>", lambda _e: canvas.bind_all("<MouseWheel>", _wheel))
        outer.bind("<Leave>", lambda _e: canvas.unbind_all("<MouseWheel>"))
        return inner

    # ── ① 学習タブ ──

    def _build_train_tab(self, nb: ttk.Notebook) -> None:
        frame = self._scrollable_tab(nb, "① 学習")

        ttk.Label(frame, text="run名:").grid(row=0, column=0, sticky="w")
        self.run_combo = ttk.Combobox(frame, textvariable=self.run_name_var, width=20)
        self.run_combo.grid(row=0, column=1, sticky="w")
        self.run_combo["postcommand"] = self._refresh_run_names
        ttk.Label(frame, textvariable=self.run_status_var, foreground="gray").grid(
            row=1, column=0, columnspan=3, sticky="w", pady=(2, 8))

        basic = {
            # dt=0.10なので3M step=300,000シム秒。
            # 2Mに削るのは**危険**——過去のrun(v13/v14/v17/v19)のmean_speedは2M時点から
            # 最終まで+11〜26%伸び続けており、mean_rewardだけが2Mで飽和して見える
            # （経路不変なので実質「衝突せず完走したか」しか見ていない）。
            # 第一指標はlap_ratio=速度なので、飽和していないのは速度の方
            "total-timesteps": ("学習ステップ総数", "3000000"),
            "n-envs": ("並列環境数", "8"),
            "episode-s": ("1エピソードの打ち切り秒数", "30"),
            # LD06の回転周期(10Hz)=実車planning_nodeの計画周期に合わせた値。
            # 0.05(20Hz)にすると、スキャンが更新されないステップが54%生じるうえ
            # 実車(10Hz)と決定レートが2倍ずれる(env.py docstring「v21b」参照)
            "dt": ("制御周期[s]", "0.10"),
            "max-speed": ("最高速度[m/s]", "2.0"),
            # dt=0.10では1決定あたり最大0.1rad（max_steerの19%）。理想ライン追従に
            # 必要な舵角速度のp90(0.954rad/s)を賄える値（env.py EnvConfig docstring参照）。
            # 1.5では障害物の失敗は減らず、舵が35%忙しくなっただけだった
            "steer-rate-max-rad-s": ("舵角速度スケール[rad/s]", "1.0"),
            # 報酬罰則(steer-effort/raceline/steer-angle)は持たない。実測でタイム欠損の
            # 主因がライン取りではなく速度で、かつステア絶対値罰は文献(Evans 2021
            # arXiv:2103.10098)で「racing報酬中で最遅・コーナーを下手に曲がる」と
            # 実証済みのため(env.py docstring参照)。滑らかさは報酬項ではなく
            # 損失項(CAPS spatial / LCP勾配罰則)で攻める
            # SB3既定(0.0)。-3はgSDE有効時専用の調整値(train_rl.pyの--log-std-init
            # ヘルプ参照)なので、gSDEチェックボックスの状態に連動させて切り替える
            # 既定-1.0（σ 0.37。train_rl.pyの--log-std-init参照）
            "log-std-init": ("探索ノイズ初期標準偏差(log)", _LOG_STD_INIT_DEFAULT),
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

        use_sde_row = 2 + len(basic)
        # 既定OFF: v15以降の全runがuse_sde=Falseで学習されている
        # (モデルファイルから確認済み)。v11/v12で試したgSDEの
        # 改善は誤差程度で、決定論的方策の振動は探索ノイズではなく方策写像
        # そのものの粗さだと判明している(env.py docstring参照)
        self.use_sde_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(frame, text="gSDE(状態依存の相関探索ノイズ)を有効化",
                        variable=self.use_sde_var).grid(
            row=use_sde_row, column=0, columnspan=2, sticky="w", pady=(4, 2))

        def _sync_log_std_init(*_args: object) -> None:
            # log-std-initが既定値のまま(未手動編集)の場合だけ追従させる。
            # -3はgSDE有効時専用の調整値なのでOFF時に残ると探索が潰れる(過去の指摘事項)
            var = self._train_basic_vars["log-std-init"]
            if self.use_sde_var.get() and var.get() == _LOG_STD_INIT_DEFAULT:
                var.set("-3")
            elif not self.use_sde_var.get() and var.get() == "-3":
                var.set(_LOG_STD_INIT_DEFAULT)

        self.use_sde_var.trace_add("write", _sync_log_std_init)

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
            "obstacle-prob": ("静的障害物を置くコースの割合", "0.5"),
            "max-obstacles": ("障害物の最大個数/コース", "4"),
            # gSDE有効時しか効かないので詳細側に置く（既定はgSDE OFF）
            "sde-sample-freq": ("gSDEノイズ再サンプリング間隔[step]", "4"),
            "eval-freq": ("eval間隔[step]", "20000"),
            # 固定コース7種を1本ずつ。決定論的方策×決定論的環境なので繰り返しても
            # 同じ軌道が増えるだけ（train_rl.py モジュールdocstring参照）
            "n-eval-episodes": ("eval時のエピソード数(コース数7の倍数)", "7"),
            "checkpoint-freq": ("チェックポイント間隔[step]", "100000"),
            "learning-rate": ("学習率", "0.0003"),
            "n-steps": ("ロールアウト長", "2048"),
            "batch-size": ("バッチサイズ", "256"),
            "n-epochs": ("1更新あたりのエポック数", "10"),
            # ★0.99ではなく0.99²。dtを0.05→0.10にしたので、γを据え置くと実効
            # ホライズンが5s→10sへ暗黙に倍増し、**速度への報酬勾配が4割弱まる**
            # （Σprogressは経路・時間に不変で、速く走る動機は割引だけが作っている
            # ——γが1に近づくほどリターンが時間不変に近づく）。実測換算では
            # 速度1.6→2.0m/s(+25%)に対するリターン増が +16.6%→+9.8% になる。
            # 0.9801なら実時間ホライズン5sが保たれ、dtだけを変えた比較にもなる
            "gamma": ("割引率", "0.9801"),
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
            picked = [n for n, v in self._real_course_vars.items() if v.get()]
            picked_eval = [n for n, v in self._real_eval_vars.items() if v.get()]
            if picked:
                args["real-courses"] = "|".join(picked)
                args["real-course-prob"] = self._real_prob_var.get().strip()
            if picked_eval:
                args["real-eval-courses"] = "|".join(picked_eval)
            if resume_or_overwrite == "resume":
                args["resume"] = True
            elif resume_or_overwrite == "overwrite":
                args["overwrite"] = True
            cmd = build_train_cmd(self.python, run_name, args)
            self._run("train", cmd, "学習", on_done=self._refresh_run_status)

        ttk.Button(frame, text="学習開始", command=run).grid(
            row=adv_toggle_row + 2, column=0, sticky="w", pady=(10, 4))

        def open_tensorboard() -> None:
            run_dir = self._require_model_dir()
            if run_dir is None:
                return
            cmd = build_tensorboard_cmd(self.tensorboard_bin, run_dir / "tb", 6006)
            self._run("tensorboard", cmd, "TensorBoard")
            import webbrowser
            self.root.after(1500, lambda: webbrowser.open("http://localhost:6006"))

        ttk.Button(frame, text="TensorBoardを開く", command=open_tensorboard).grid(
            row=adv_toggle_row + 2, column=1, sticky="w", pady=(10, 4))

        course_frame = ttk.LabelFrame(
            frame, text="実コース（学習: 無し = ランダム生成のみ／評価: 無し = 学習と同じ、"
                        "学習も無しなら従来の7本）")
        course_frame.grid(row=adv_toggle_row + 4, column=0, columnspan=3, sticky="we", pady=(8, 0))
        #: パス -> (学習に使う, 評価に使う)
        self._real_course_vars: dict[str, tk.BooleanVar] = {}
        self._real_eval_vars: dict[str, tk.BooleanVar] = {}
        self._real_prob_var = tk.StringVar(value="1.0")
        self._real_course_box = ttk.Frame(course_frame)
        self._real_course_box.grid(row=0, column=0, columnspan=3, sticky="w")

        def _redraw_real_courses() -> None:
            for w in self._real_course_box.winfo_children():
                w.destroy()
            if not self._real_course_vars:
                ttk.Label(self._real_course_box, text="（未追加）", foreground="gray").grid(
                    row=0, column=0, padx=4)
                return
            ttk.Label(self._real_course_box, text="学習").grid(row=0, column=1, padx=6)
            ttk.Label(self._real_course_box, text="評価").grid(row=0, column=2, padx=6)
            for i, path in enumerate(self._real_course_vars, start=1):
                ttk.Label(self._real_course_box, text=Path(path).name).grid(
                    row=i, column=0, sticky="w", padx=4)
                ttk.Checkbutton(self._real_course_box, variable=self._real_course_vars[path]).grid(
                    row=i, column=1)
                ttk.Checkbutton(self._real_course_box, variable=self._real_eval_vars[path]).grid(
                    row=i, column=2)
                ttk.Button(self._real_course_box, text="削除", width=5,
                           command=lambda p=path: _remove_real_course(p)).grid(row=i, column=3)

        def _remove_real_course(path: str) -> None:
            self._real_course_vars.pop(path, None)
            self._real_eval_vars.pop(path, None)
            _redraw_real_courses()

        def _add_real_courses() -> None:
            paths = filedialog.askopenfilenames(
                title="コースのnpzを選択（複数可）", filetypes=[("地図 npz", "*.npz")],
                initialdir=str(Path.home() / "Downloads"))
            for path in paths:
                if path in self._real_course_vars:
                    continue
                try:
                    load_real_course(path)
                except Exception as e:                                # noqa: BLE001
                    messagebox.showerror("コースを読み込めません", f"{Path(path).name}\n\n{e}")
                    continue
                self._real_course_vars[path] = tk.BooleanVar(value=True)
                self._real_eval_vars[path] = tk.BooleanVar(value=True)
            _redraw_real_courses()

        _redraw_real_courses()
        ttk.Button(course_frame, text="npzを追加…", command=_add_real_courses).grid(
            row=1, column=0, sticky="w", padx=4, pady=2)
        ttk.Label(course_frame, text="実コースを使う確率(0-1、残りはランダム生成):").grid(
            row=1, column=1, sticky="e")
        ttk.Entry(course_frame, textvariable=self._real_prob_var, width=6).grid(
            row=1, column=2, sticky="w", padx=4)
        ttk.Label(course_frame, text="地図は raspi.auto.mapstore 形式のnpz（走行軌跡か中心線が必要）",
                  foreground="gray").grid(row=2, column=0, columnspan=3, sticky="w", padx=4)

        note_frame = ttk.LabelFrame(frame, text="備考（どんな変更をしたか等、自由に）")
        note_frame.grid(row=adv_toggle_row + 3, column=0, columnspan=3, sticky="we", pady=(8, 0))
        self.note_text = tk.Text(note_frame, height=3, wrap="word")
        self.note_text.pack(fill="both", expand=True, padx=4, pady=(4, 0))
        ttk.Button(note_frame, text="備考を保存", command=self._save_note).pack(
            anchor="e", padx=4, pady=4)

    # ── ② 観戦タブ ──

    def _build_watch_tab(self, nb: ttk.Notebook) -> None:
        frame = self._scrollable_tab(nb, "② 観戦")

        live_frame = ttk.LabelFrame(frame, text="学習中runをライブ観戦（自動更新・複数コース並列）")
        live_frame.grid(row=0, column=0, columnspan=3, sticky="we")

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
        frame = self._scrollable_tab(nb, "③ エクスポート")

        ttk.Label(frame, text="run名:").grid(row=0, column=0, sticky="w")
        self.export_run_var = tk.StringVar(value="")
        self.export_run_combo = ttk.Combobox(frame, textvariable=self.export_run_var, width=20)
        self.export_run_combo.grid(row=0, column=1, sticky="w")
        self.export_run_combo["postcommand"] = self._refresh_run_names

        def _sync_export_to_run(*_args) -> None:
            if not self._suppress_export_sync:
                self.run_name_var.set(self.export_run_var.get())
        self.export_run_var.trace_add("write", _sync_export_to_run)

        # ★既定は`eval/best_model.zip`ではない。SB3の`EvalCallback`は
        # **mean_rewardで選ぶ**が、この環境のmean_rewardは1周ぶんの弧長=total_length/dtで
        # ほぼ定数になり、モデルの良し悪しを区別できない——v21では`eval/best_model.zip`も
        # `best_model_generalized.zip`も約900k時点で凍り、対MCL理想ラップ比が
        # **1.286倍（final_modelは1.022倍）**という26%遅いモデルが実車へ出る状態だった。
        # `best_model_generalized`側はlap_ratio/lap_rateで選ぶよう直してある
        # （train_rl.py `GeneralizationEvalCallback`のdocstring参照）。
        # `eval/best_model.zip`はresume用に残るが、エクスポート先としては選べない
        self.export_source_var = tk.StringVar(value="best_model_generalized")
        ttk.Label(frame, text="元にするモデル:").grid(row=1, column=0, sticky="w", pady=(8, 0))
        src_frame = ttk.Frame(frame)
        src_frame.grid(row=1, column=1, columnspan=2, sticky="w", pady=(8, 0))
        ttk.Radiobutton(src_frame, text="best_model_generalized（汎化eval最良＝ラップ比）",
                        variable=self.export_source_var,
                        value="best_model_generalized").pack(side="left")
        ttk.Radiobutton(src_frame, text="final_model（学習終了時点）", variable=self.export_source_var,
                        value="final_model").pack(side="left", padx=(10, 0))

        ttk.Label(frame, text="出力名（models/e2e_lidar/<name>.onnx）:").grid(
            row=2, column=0, sticky="w", pady=(8, 0))
        self.export_name_var = tk.StringVar(value="")
        ttk.Entry(frame, textvariable=self.export_name_var, width=22).grid(
            row=2, column=1, sticky="w", pady=(8, 0))

        def source_path() -> Path | None:
            run_dir = RUNS_DIR / self.export_run_var.get().strip()
            if self.export_source_var.get() == "best_model_generalized":
                p = run_dir / "eval" / "best_model_generalized.zip"
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
            # 観戦タブのモデル一覧に、今エクスポートしたモデルを足す
            self._run("export", cmd, "エクスポート", on_done=lambda ok: self.watch_model_combo.configure(
                values=list_exported_model_names()) if ok else None)

        ttk.Button(frame, text="エクスポート実行", command=run_export).grid(
            row=3, column=0, sticky="w", pady=10)

        ttk.Separator(frame, orient="horizontal").grid(
            row=4, column=0, columnspan=3, sticky="we", pady=8)

        self.eval_episodes_var = tk.StringVar(value="300")
        ttk.Label(frame, text="エピソード数:").grid(row=5, column=0, sticky="w", pady=(4, 0))
        ttk.Entry(frame, textvariable=self.eval_episodes_var, width=10).grid(
            row=5, column=1, sticky="w", pady=(4, 0))

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
            row=6, column=0, sticky="w", pady=(6, 0))

    # ── ④ run一覧タブ ──

    def _build_runs_tab(self, nb: ttk.Notebook) -> None:
        frame = self._scrollable_tab(nb, "④ run一覧")

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

    def _refresh_run_status(self, ok: bool) -> None:
        if ok:
            self.run_status_var.set(describe_run_status(RUNS_DIR / self.run_name_var.get().strip()))

    def _on_jobs_changed(self) -> None:
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


def main() -> int:
    root = tk.Tk()
    App(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
