"""補助 GUI のランチャー（Tkinter、Mac 側）。

    .venv/bin/python -m tools.launcher
    （または surge_mk2 直下の launcher.command をダブルクリック）

メイン GUI（Web）以外の補助 GUI（シミュレータ・コースエディタ・システム同定・
カメラ校正・SLAM 再処理・ml 系の操作盤）を1つの窓から起動・停止する。アプリを増やすときは
`APPS` に1行足すだけでよい。

## 1つの GUI に統合せず、別プロセスで起動する理由

- pygame（sim.gui / sim.editor）と Tk は1プロセスの mainloop に同居できない
- `SysIdApp` / `SlamReplayApp` は `tk.Tk` 自体を継承しており、そのままでは埋め込めない
- torch（ml 系）と g2opy（slam）を同じプロセスに載せると、1本が落ちたとき全部落ちる

子プロセスの管理は ml 系アプリと同じ `ml_common` の `JobRunner` を使う。子の stdout は
このランチャーにパイプでつながっているため、**ランチャーを閉じると起動中のアプリも終了する**
（ランチャーだけ閉じると、子が壊れたパイプに書き込んで落ちるため）。
"""

from __future__ import annotations

import re
import sys
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Callable

from ml_common.close_handler import confirm_and_close
from ml_common.job_runner import JobRunner
from ml_common.log_console import LogConsole
from sim.run import _course_files

ROOT = Path(__file__).resolve().parents[1]

#: sim.run などが出す ANSI 色コード。ログ欄では読めないので落とす
_ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")


@dataclass(frozen=True)
class Options:
    """起動時に画面から渡す引数。"""
    course: str = ""
    browser: bool = True


@dataclass(frozen=True)
class AppSpec:
    key: str
    group: str
    title: str
    desc: str
    #: `sys.executable -u` の後ろに付ける引数列
    argv: Callable[[Options], list[str]]
    #: コース選択欄を出すか。`course_required=False` なら空欄（＝新規）を許す
    course: bool = False
    course_required: bool = True
    #: 「ブラウザを開く」チェックを出すか
    browser: bool = False


def _sim_run(o: Options) -> list[str]:
    return ["-m", "sim.run", "--course", o.course] + ([] if o.browser else ["--no-browser"])


def _sim_editor(o: Options) -> list[str]:
    return ["-m", "sim.editor"] + ([o.course] if o.course else [])


APPS: list[AppSpec] = [
    AppSpec("sim", "シミュレータ", "シミュレータ一式",
            "io_node(--sim)・telemetry・planning・俯瞰ビューを起動し、Web GUI を開く",
            _sim_run, course=True, browser=True),
    AppSpec("editor", "シミュレータ", "コースエディタ",
            "コースを描く（空欄なら新規）", _sim_editor, course=True, course_required=False),
    AppSpec("sysid", "解析", "システム同定",
            "mcap から動特性を同定し vehicle.toml に書き戻す",
            lambda o: ["-m", "tools.sysid.gui"]),
    AppSpec("ctrl_tune", "解析", "制御の調整",
            "TC・ABS・TV の車両モデルを mcap から同定し、パラメータを最適化して vehicle.toml に書き戻す",
            lambda o: ["-m", "tools.ctrl_tune.gui"]),
    AppSpec("cam_calib", "解析", "カメラ校正",
            "📷 で撮ったチェッカーボード写真から魚眼レンズを校正し vehicle.toml に書き戻す",
            lambda o: ["-m", "tools.cam_calib.gui"]),
    AppSpec("slam_replay", "解析", "SLAM 再処理",
            "記録を slam2d に通して _slam.mcap を作り Foxglove で開く",
            lambda o: ["-m", "tools.slam_replay_gui"]),
    AppSpec("ml_lidar", "学習", "ml_lidar",
            "LiDAR 強化学習の学習・観戦・エクスポート",
            lambda o: ["ml_lidar/app.py"]),
    AppSpec("ml_cam", "学習", "ml_cam",
            "カメラのセグメンテーション（抽出・アノテーション・学習・エクスポート）",
            lambda o: ["ml_cam/app.py"]),
    AppSpec("ml_cam_e2e", "学習", "ml_cam_e2e",
            "カメラ E2E（抽出・学習・エクスポート・プレビュー）",
            lambda o: ["ml_cam_e2e/app.py"]),
]


def course_names() -> list[str]:
    return [p.stem for p in _course_files()]


class _Row:
    """1アプリ分の行（状態・起動/停止・引数）。"""

    def __init__(self, parent: ttk.Frame, row: int, spec: AppSpec,
                 courses: list[str], on_start: Callable[[AppSpec], None],
                 on_stop: Callable[[AppSpec], None]) -> None:
        self.spec = spec
        self.course = tk.StringVar()
        self.browser = tk.BooleanVar(value=True)

        ttk.Label(parent, text=spec.title, font=("", 13, "bold")).grid(
            row=row, column=0, sticky="w", padx=(4, 12), pady=3)
        ttk.Label(parent, text=spec.desc, foreground="gray").grid(
            row=row, column=1, sticky="w")

        opts = ttk.Frame(parent)
        opts.grid(row=row, column=2, sticky="e", padx=8)
        if spec.course:
            values = courses if spec.course_required else [""] + courses
            if spec.course_required and courses:
                self.course.set("normal" if "normal" in courses else courses[0])
            ttk.Combobox(opts, textvariable=self.course, values=values,
                         width=18, state="readonly").pack(side="left")
        if spec.browser:
            ttk.Checkbutton(opts, text="ブラウザを開く", variable=self.browser).pack(
                side="left", padx=(6, 0))

        self.status = ttk.Label(parent, text="", width=8)
        self.status.grid(row=row, column=3)
        self.start_btn = ttk.Button(parent, text="起動", width=6,
                                    command=lambda: on_start(spec))
        self.start_btn.grid(row=row, column=4, padx=2)
        self.stop_btn = ttk.Button(parent, text="停止", width=6, state="disabled",
                                   command=lambda: on_stop(spec))
        self.stop_btn.grid(row=row, column=5, padx=(2, 4))

    def options(self) -> Options:
        return Options(course=self.course.get(), browser=self.browser.get())

    def set_running(self, running: bool) -> None:
        self.status.config(text="● 実行中" if running else "",
                           foreground="green" if running else "")
        self.start_btn.config(state="disabled" if running else "normal")
        self.stop_btn.config(state="normal" if running else "disabled")


class LauncherApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        root.title("SURGE Mk.2 補助ツール")
        root.geometry("1000x680")
        self.jobs = JobRunner(ROOT, allow_concurrent=True, prefix_labels=True)
        self.rows: dict[str, _Row] = {}

        courses = course_names()
        top = ttk.Frame(root, padding=8)
        top.pack(fill="x")
        groups: dict[str, ttk.Frame] = {}
        counts: dict[str, int] = {}
        for spec in APPS:
            if spec.group not in groups:
                lf = ttk.LabelFrame(top, text=spec.group, padding=4)
                lf.pack(fill="x", pady=4)
                lf.columnconfigure(1, weight=1)
                groups[spec.group] = lf
                counts[spec.group] = 0
            self.rows[spec.key] = _Row(groups[spec.group], counts[spec.group], spec,
                                       courses, self._start, self._stop)
            counts[spec.group] += 1

        bottom = ttk.Frame(root, padding=(8, 0, 8, 8))
        bottom.pack(fill="both", expand=True)
        head = ttk.Frame(bottom)
        head.pack(fill="x")
        ttk.Label(head, text="ログ").pack(side="left")
        ttk.Button(head, text="消去", command=self._clear_log).pack(side="right")
        self.log = LogConsole(bottom, height=12)
        self.log.widget.pack(fill="both", expand=True)

        root.protocol("WM_DELETE_WINDOW", self._on_close)
        root.after(100, self._drain)

    def _start(self, spec: AppSpec) -> None:
        row = self.rows[spec.key]
        opts = row.options()
        if spec.course and spec.course_required and not opts.course:
            messagebox.showwarning("コース未選択", "コースを選んでください。")
            return
        cmd = [sys.executable, "-u", *spec.argv(opts)]
        started = self.jobs.start(spec.key, cmd, spec.title,
                                  on_done=lambda code, k=spec.key: self.rows[k].set_running(False))
        if not started:
            messagebox.showinfo("起動中", f"{spec.title} は既に起動しています。")
            return
        row.set_running(True)
        self.log.append(f"[{spec.title}] 起動: {' '.join(cmd[2:])}\n")

    def _stop(self, spec: AppSpec) -> None:
        if not self.jobs.stop(spec.key):
            self.log.append(f"[{spec.title}] 起動処理中のため停止できませんでした。少し待って再度押してください\n")

    def _clear_log(self) -> None:
        w = self.log.widget
        w.config(state="normal")
        w.delete("1.0", "end")
        w.config(state="disabled")

    def _drain(self) -> None:
        self.jobs.drain(lambda s: self.log.append(_ANSI.sub("", s)))
        self.root.after(100, self._drain)

    def _on_close(self) -> None:
        running = self.jobs.running_job_keys()
        if running:
            names = "、".join(self.jobs.label_of(k) or k for k in running)
            if not messagebox.askyesno(
                    "終了しますか？",
                    f"実行中: {names}\n\nランチャーを閉じるとこれらも終了します。よろしいですか？"):
                return
        confirm_and_close(self.root, self.jobs)


def main() -> None:
    root = tk.Tk()
    LauncherApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
