"""システム同定 解析GUI（Tkinter、Mac側）。

    .venv/bin/python -m tools.sysid.gui

GUIの「システム同定」タブでダウンロードしたmcapファイル（試験ごとに1つ）を開き、
`tools/sysid/fit.py` で解析して `config/vehicle.toml` の `[dynamics]` に書き戻す。
解析の所見（残差・測れなかった項目）も一緒に表示する。

ネイティブGUIにしてあるのは、ファイル選択・結果の見比べ・適用の可否判断を
ターミナル操作なしで完結させるため（バンビの補助ツール全般の方針）。
"""

from __future__ import annotations

import tkinter as tk
import tomllib
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from sim.vehicle import VehicleSpec

from . import toml_update
from .analyze import TESTS, analyze

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_TOML = REPO_ROOT / "config" / "vehicle.toml"

#: 物理的に正でなければおかしいパラメータ。0.0や負値は測定失敗（配線ミス等）が
#: すり抜けたサインの可能性が高いので、GUI側でも独立に検知して警告する。
#: むだ時間・物理上限・加減速の上限・減衰の速度・遅延は0（＝無し・観測できず）があり得るので含めない。
#: 中立ずれ・アンダーステア勾配は符号つき
_MUST_BE_POSITIVE = {
    "tau_steer_s", "speed_plant_gain", "mu", "brake_decel_m_s2", "steer_gain", "steer_servo_gain",
}


def _is_suspicious(key: str, value: float) -> bool:
    """0または非現実的（負値）な測定結果かどうか。"""
    return key in _MUST_BE_POSITIVE and value <= 0.0


def _load_current_dynamics(toml_path: str) -> dict[str, float]:
    try:
        with open(toml_path, "rb") as f:
            d = tomllib.load(f)
    except (OSError, tomllib.TOMLDecodeError):
        return {}
    out = {k: float(v) for k, v in d.get("dynamics", {}).items() if isinstance(v, (int, float))}
    # ステアのリンクの換算は [control] にある（`toml_update.LINK_KEYS`）
    out.update({k: float(d["control"][k]) for k in toml_update.LINK_KEYS if k in d.get("control", {})})
    return out


class SysIdApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("システム同定 — vehicle.toml 更新")
        self.geometry("760x720")

        self.toml_path = tk.StringVar(value=str(DEFAULT_TOML))
        self.file_vars: dict[str, tk.StringVar] = {}
        self.results: dict[str, float] = {}
        self.check_vars: dict[str, tk.BooleanVar] = {}
        self.notes: dict[str, list[str]] = {}
        self.warned: dict[str, str] = {}

        self._build()

    # ── 画面構築 ──

    def _build(self) -> None:
        top = ttk.Frame(self, padding=8)
        top.pack(fill="x")
        ttk.Label(top, text="vehicle.toml:").pack(side="left")
        ttk.Entry(top, textvariable=self.toml_path, width=48).pack(side="left", padx=4, fill="x", expand=True)
        ttk.Button(top, text="参照", command=self._pick_toml).pack(side="left")

        for key, label, _params in TESTS:
            row = ttk.Frame(self, padding=(8, 4))
            row.pack(fill="x")
            ttk.Label(row, text=label, width=20).pack(side="left")
            var = tk.StringVar(value="（未選択）")
            self.file_vars[key] = var
            ttk.Label(row, textvariable=var, foreground="gray").pack(
                side="left", padx=4, fill="x", expand=True)
            ttk.Button(row, text="mcapを選ぶ", command=lambda k=key: self._pick_mcap(k)).pack(side="right")

        ttk.Button(self, text="解析", command=self._analyze).pack(pady=6)

        ttk.Separator(self).pack(fill="x", padx=8)
        self.result_frame = ttk.Frame(self, padding=8)
        self.result_frame.pack(fill="both", expand=True)

        bottom = ttk.Frame(self, padding=8)
        bottom.pack(fill="x")
        ttk.Button(bottom, text="適用", command=self._apply).pack(side="right")

    def _pick_toml(self) -> None:
        p = filedialog.askopenfilename(title="vehicle.toml", filetypes=[("TOML", "*.toml")])
        if p:
            self.toml_path.set(p)

    def _pick_mcap(self, key: str) -> None:
        p = filedialog.askopenfilename(title="mcapファイル", filetypes=[("mcap", "*.mcap")])
        if p:
            self.file_vars[key].set(p)

    # ── 解析 ──

    def _analyze(self) -> None:
        try:
            base = VehicleSpec.load(self.toml_path.get())
        except (OSError, ValueError) as e:
            messagebox.showerror("vehicle.toml", str(e))
            return
        sources = {key: self.file_vars[key].get() for key, _, _ in TESTS
                   if self.file_vars[key].get() not in ("", "（未選択）")}
        res = analyze(sources, base)
        self.results = res.results
        self.notes = res.notes
        self.warned = res.warned
        if res.errors:
            messagebox.showerror("解析エラー", "\n".join(res.errors))
        self._render_results()

    def _render_results(self) -> None:
        for w in self.result_frame.winfo_children():
            w.destroy()
        self.check_vars.clear()

        if not self.results:
            ttk.Label(self.result_frame,
                     text="解析結果がありません（mcapを選んで「解析」を押してください）").grid(row=0, column=0)
            return

        current = _load_current_dynamics(self.toml_path.get())
        ttk.Label(self.result_frame, text="適用").grid(row=0, column=0)
        ttk.Label(self.result_frame, text="パラメータ").grid(row=0, column=1, sticky="w")
        ttk.Label(self.result_frame, text="現在値 → 測定値").grid(row=0, column=2, sticky="w")

        for row, key in enumerate(sorted(self.results), start=1):
            new_val = self.results[key]
            suspicious = _is_suspicious(key, new_val)
            # ★（記録がモデルの形から外れている）の出た試験の値は既定で適用しない
            warned = self.warned.get(key)
            var = tk.BooleanVar(value=warned is None)
            self.check_vars[key] = var
            ttk.Checkbutton(self.result_frame, variable=var).grid(row=row, column=0)
            ttk.Label(self.result_frame, text=key, width=24).grid(row=row, column=1, sticky="w")
            old = current.get(key)
            old_s = f"{old:.4g}" if old is not None else "?"
            value_text = f"{old_s} → {new_val:.4g}"
            if suspicious:
                value_text += "  ⚠ 0または異常値の疑い"
            if warned is not None:
                value_text += f"  ★{warned}の残差が大きい（既定で適用しない。所見を読んで判断）"
            ttk.Label(self.result_frame, text=value_text,
                     foreground=("red" if suspicious or warned else "black")).grid(row=row, column=2, sticky="w")

        # 所見（残差・測れなかった項目）。値を適用するかの判断材料
        lines = [f"{label}: {n}" for label, ns in self.notes.items() for n in ns]
        if lines:
            box = tk.Text(self.result_frame, height=min(12, len(lines) + 1), wrap="word")
            box.insert("1.0", "\n".join(lines))
            box.configure(state="disabled")
            box.grid(row=len(self.results) + 1, column=0, columnspan=3, sticky="we", pady=(8, 0))

    # ── 適用 ──

    def _apply(self) -> None:
        if not self.results:
            messagebox.showinfo("適用", "先に解析してください")
            return
        chosen = {k: v for k, v in self.results.items()
                 if self.check_vars.get(k) is not None and self.check_vars[k].get()}
        if not chosen:
            messagebox.showinfo("適用", "適用するパラメータをチェックしてください")
            return

        suspicious_keys = sorted(k for k, v in chosen.items() if _is_suspicious(k, v))
        if suspicious_keys:
            proceed = messagebox.askyesno(
                "異常値の疑いがあります",
                "以下のパラメータが0または現実的でない値（負値等）です:\n"
                + "\n".join(f"・{k} = {chosen[k]:.4g}" for k in suspicious_keys)
                + "\n\n配線ミス等で測定に失敗している可能性があります。"
                  "このまま vehicle.toml に書き込みますか？",
                icon="warning",
            )
            if not proceed:
                return
        try:
            changed = toml_update.apply_dynamics(self.toml_path.get(), chosen)
        except Exception as e:  # noqa: BLE001
            messagebox.showerror("適用エラー", str(e))
            return
        messagebox.showinfo(
            "適用しました",
            (f"更新: {', '.join(changed)}" if changed else "変更はありませんでした（既存値と同じ）")
            + "\n\n★ヘッダの説明文・「★未実測」の注記・measuredフラグは自動更新して"
              "いません。手動で見直してください。",
        )


def main() -> None:
    SysIdApp().mainloop()


if __name__ == "__main__":
    main()
