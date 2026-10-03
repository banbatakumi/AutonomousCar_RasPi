"""カメラ校正 GUI（Tkinter、Mac 側）。ランチャーの「カメラ校正」から起動する。

    .venv/bin/python -m tools.cam_calib.gui

1. カメラ（前/後）・ボードの寸法・写真のフォルダ（既定 ~/Downloads）を選ぶ
2. 「写真を読み込む」で 📷 の写真（`surge_<cam>_…png`）を拾い、角点を検出する
3. 一覧で使わない写真を外し（ダブルクリックで切替）、「校正」
4. 結果（再投影誤差・画面カバー率・補正前後のプレビュー）を見て、
   よければ「vehicle.toml に書き込む」（`config/generate.py` まで走る）

ロジックは `calib.py`（CLI の `python -m tools.cam_calib` と共通）。ネイティブ GUI に
してあるのは、写真の取捨選択と結果の見比べをターミナル操作なしで完結させるため
（`tools/sysid/gui.py` と同じ方針）。
"""

from __future__ import annotations

import base64
import math
import threading
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from .calib import (DEFAULT_TOML, CalibResult, Detection, calibrate, coverage, detect,
                    parse_name, undistort_preview, write_vehicle_toml)

#: プレビューの最大幅 [px]（元画像と補正画像を横に並べた全体）
_PREVIEW_W = 900


class CamCalibApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("カメラ校正 — 魚眼レンズ（チェッカーボード）")
        self.geometry("980x860")

        self.cam = tk.StringVar(value="front")
        self.cols = tk.IntVar(value=9)
        self.rows = tk.IntVar(value=6)
        self.square_mm = tk.DoubleVar(value=25.0)
        self.folder = tk.StringVar(value=str(Path.home() / "Downloads"))
        self.toml_path = tk.StringVar(value=str(DEFAULT_TOML))
        self.status = tk.StringVar(value="写真のフォルダを選んで「写真を読み込む」")

        self.dets: dict[str, Detection] = {}     # Treeview の iid → 検出結果
        self.use: dict[str, bool] = {}
        self.result: CalibResult | None = None
        self._photo: tk.PhotoImage | None = None  # 参照を持っておかないと消える

        self._build()

    # ── 画面構築 ──

    def _build(self) -> None:
        top = ttk.Frame(self, padding=8)
        top.pack(fill="x")
        ttk.Label(top, text="カメラ").grid(row=0, column=0, sticky="w")
        for i, (v, t) in enumerate((("front", "前"), ("rear", "後"))):
            ttk.Radiobutton(top, text=t, value=v, variable=self.cam).grid(row=0, column=1 + i)
        ttk.Label(top, text="ボード 内側の角点").grid(row=0, column=3, padx=(16, 4))
        ttk.Spinbox(top, from_=3, to=20, width=4, textvariable=self.cols).grid(row=0, column=4)
        ttk.Label(top, text="×").grid(row=0, column=5)
        ttk.Spinbox(top, from_=3, to=20, width=4, textvariable=self.rows).grid(row=0, column=6)
        ttk.Label(top, text="1マス [mm]（定規で実測）").grid(row=0, column=7, padx=(16, 4))
        ttk.Entry(top, width=7, textvariable=self.square_mm).grid(row=0, column=8)
        ttk.Button(top, text="印刷用ボードを作る", command=self._make_board).grid(
            row=0, column=9, padx=(16, 0))

        row = ttk.Frame(self, padding=(8, 0))
        row.pack(fill="x")
        ttk.Label(row, text="写真のフォルダ").pack(side="left")
        ttk.Entry(row, textvariable=self.folder).pack(side="left", fill="x", expand=True, padx=4)
        ttk.Button(row, text="参照", command=self._pick_folder).pack(side="left")
        ttk.Button(row, text="写真を読み込む", command=self._load).pack(side="left", padx=4)

        mid = ttk.Frame(self, padding=8)
        mid.pack(fill="both", expand=True)
        self.tree = ttk.Treeview(mid, columns=("use", "found", "err"), height=10)
        self.tree.heading("#0", text="写真")
        self.tree.heading("use", text="使う")
        self.tree.heading("found", text="ボード")
        self.tree.heading("err", text="誤差 / 理由")
        self.tree.column("#0", width=420)
        self.tree.column("use", width=50, anchor="center")
        self.tree.column("found", width=60, anchor="center")
        self.tree.column("err", width=300)
        sb = ttk.Scrollbar(mid, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=sb.set)
        self.tree.pack(side="left", fill="both", expand=True)
        sb.pack(side="left", fill="y")
        self.tree.bind("<Double-1>", self._toggle_use)
        self.tree.bind("<<TreeviewSelect>>", lambda _e: self._show_preview())

        btns = ttk.Frame(self, padding=(8, 0))
        btns.pack(fill="x")
        self.calib_btn = ttk.Button(btns, text="校正", command=self._calibrate, state="disabled")
        self.calib_btn.pack(side="left")
        self.write_btn = ttk.Button(btns, text="vehicle.toml に書き込む", command=self._write,
                                    state="disabled")
        self.write_btn.pack(side="left", padx=6)
        ttk.Label(btns, text="vehicle.toml:").pack(side="left", padx=(16, 2))
        ttk.Entry(btns, textvariable=self.toml_path, width=40).pack(side="left", fill="x",
                                                                   expand=True)

        self.summary = tk.Text(self, height=7, wrap="word")
        self.summary.pack(fill="x", padx=8, pady=6)
        self.preview = ttk.Label(self, anchor="center", text="（写真を選ぶと補正前後を並べて表示）")
        self.preview.pack(fill="both", expand=True, padx=8)
        ttk.Label(self, textvariable=self.status, foreground="gray").pack(fill="x", padx=8, pady=4)

    # ── 操作 ──

    def _board(self) -> tuple[int, int]:
        return int(self.cols.get()), int(self.rows.get())

    def _square_m(self) -> float:
        return float(self.square_mm.get()) / 1000.0

    def _pick_folder(self) -> None:
        d = filedialog.askdirectory(initialdir=self.folder.get())
        if d:
            self.folder.set(d)

    def _make_board(self) -> None:
        from .board import save_board

        cols, rows = self._board()
        path = filedialog.asksaveasfilename(
            defaultextension=".pdf", initialfile=f"checkerboard_{cols}x{rows}.pdf",
            filetypes=[("PDF", "*.pdf"), ("PNG", "*.png")])
        if not path:
            return
        save_board(path, (cols, rows), self._square_m())
        messagebox.showinfo("印刷用ボード",
                            f"保存しました: {path}\n\n「実際のサイズ」(100%) で印刷し、平らな板に貼って、"
                            "1マスを定規で測った値を「1マス [mm]」に入れてください。")

    def _load(self) -> None:
        cam = self.cam.get()
        folder = Path(self.folder.get()).expanduser()
        paths = sorted(p for p in folder.glob(f"surge_{cam}_*.png") if parse_name(p))
        if not paths:
            messagebox.showwarning("写真が無い",
                                   f"{folder} に surge_{cam}_*.png がありません。\n"
                                   "GUI のタブバーの 📷 で撮ってください。")
            return
        self.tree.delete(*self.tree.get_children())
        self.dets.clear()
        self.use.clear()
        self.result = None
        self.write_btn.config(state="disabled")
        self.calib_btn.config(state="disabled")
        board = self._board()
        self.status.set(f"{len(paths)} 枚の角点を検出中…")

        def work() -> None:
            for i, p in enumerate(paths):
                d = detect(p, board)
                self.after(0, self._add_detection, d, i + 1, len(paths))
            self.after(0, self._detect_done)

        threading.Thread(target=work, daemon=True).start()

    def _add_detection(self, d: Detection, i: int, n: int) -> None:
        iid = str(d.path)
        self.dets[iid] = d
        self.use[iid] = d.ok
        self.tree.insert("", "end", iid=iid, text=d.path.name,
                         values=("✓" if d.ok else "", "✓" if d.ok else "✗",
                                 "" if d.ok else d.error))
        self.status.set(f"検出中… {i}/{n}")

    def _detect_done(self) -> None:
        dets = list(self.dets.values())
        n_ok = sum(d.ok for d in dets)
        self.status.set(f"ボードが見つかった写真 {n_ok}/{len(dets)} 枚・画面カバー率 "
                        f"{coverage(dets) * 100:.0f}%（端・四隅まで写すほどよい）")
        self.calib_btn.config(state="normal" if n_ok else "disabled")

    def _toggle_use(self, event) -> None:
        iid = self.tree.identify_row(event.y)
        if not iid or not self.dets[iid].ok:
            return
        self.use[iid] = not self.use[iid]
        vals = list(self.tree.item(iid, "values"))
        vals[0] = "✓" if self.use[iid] else ""
        self.tree.item(iid, values=vals)

    def _calibrate(self) -> None:
        chosen = [d for iid, d in self.dets.items() if self.use.get(iid)]
        board, square = self._board(), self._square_m()
        self.calib_btn.config(state="disabled")
        self.status.set(f"{len(chosen)} 枚で校正中…")

        def work() -> None:
            try:
                res = calibrate(chosen, board, square)
            except Exception as e:                          # noqa: BLE001
                self.after(0, self._calib_failed, str(e))
                return
            self.after(0, self._calib_done, res)

        threading.Thread(target=work, daemon=True).start()

    def _calib_failed(self, msg: str) -> None:
        self.calib_btn.config(state="normal")
        self.status.set("校正に失敗")
        messagebox.showerror("校正に失敗", msg)

    def _calib_done(self, res: CalibResult) -> None:
        self.result = res
        self.calib_btn.config(state="normal")
        self.write_btn.config(state="normal")
        for iid in self.dets:
            p = Path(iid)
            vals = list(self.tree.item(iid, "values"))
            if p in res.per_image:
                vals[2] = f"{res.per_image[p]:.3f}px"
            elif p in res.rejected:
                vals[2] = f"除外: {res.rejected[p]}"
            self.tree.item(iid, values=vals)
        c = res.calib
        good = "良好" if res.rms < 0.5 else "やや大きい（ブレ・ピンボケ・ボードのたわみを確認）"
        self.summary.delete("1.0", "end")
        self.summary.insert("end", "\n".join([
            f"{self.cam.get()} カメラ  {c.width}x{c.height}（クロップ前）  使用 {len(res.per_image)} 枚"
            f"  除外 {len(res.rejected)} 枚",
            f"再投影誤差 RMS {res.rms:.3f}px — {good}（目安 0.5px 未満）",
            f"fx={c.fx:.2f}  fy={c.fy:.2f}  cx={c.cx:.2f}  cy={c.cy:.2f}",
            f"k = {', '.join(f'{v:.6f}' for v in c.k)}",
            f"水平画角 {math.degrees(res.hfov):.1f}°   画面カバー率 "
            f"{coverage(list(self.dets.values())) * 100:.0f}%",
            "写真を選ぶと補正前後のプレビュー。直線（ボードの縁・壁の角）が直線に戻っていれば OK",
        ]))
        self.status.set("校正できた。結果を確認して「vehicle.toml に書き込む」")
        self._show_preview()

    def _show_preview(self) -> None:
        if self.result is None:
            return
        sel = self.tree.selection()
        iid = sel[0] if sel else next(iter(self.dets), None)
        if iid is None:
            return
        try:
            import cv2

            d = self.dets[iid]
            img = undistort_preview(d.path, self.result.calib, d.crop)
            s = min(1.0, _PREVIEW_W / img.shape[1])
            if s < 1.0:
                img = cv2.resize(img, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode(".png", img)
            if not ok:
                return
            self._photo = tk.PhotoImage(data=base64.b64encode(buf.tobytes()))
            self.preview.config(image=self._photo, text="")
        except Exception as e:                              # noqa: BLE001
            self.preview.config(image="", text=f"プレビューを作れない: {e}")

    def _write(self) -> None:
        if self.result is None:
            return
        cam = self.cam.get()
        if not messagebox.askyesno(
                "vehicle.toml に書き込む",
                f"[sensors.cam_{cam}.fisheye] を書き込み、config/generate.py を実行します。\n"
                f"（再投影誤差 {self.result.rms:.3f}px）"):
            return
        try:
            write_vehicle_toml(cam, self.result, self.toml_path.get())
        except Exception as e:                              # noqa: BLE001
            messagebox.showerror("書き込みに失敗", str(e))
            return
        messagebox.showinfo("書き込んだ",
                            "vehicle.toml を更新しました。\n"
                            "Pi へは tools/deploy.sh --restart で反映してください"
                            "（GUI の再ビルドと telemetry・認識ノードの再起動まで入ります）。")
        self.status.set(f"書き込んだ: {self.toml_path.get()}")


def main() -> None:
    CamCalibApp().mainloop()


if __name__ == "__main__":
    main()
