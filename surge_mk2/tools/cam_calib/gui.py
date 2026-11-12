"""カメラ校正 GUI（Tkinter、Mac 側）。ランチャーの「カメラ校正」から起動する。

    .venv/bin/python -m tools.cam_calib.gui

1. カメラ（前/後/左/ステレオ）・レンズ（`vehicle.toml` の `lenses.<名前>`。既定は今選ばれているもの）・
   ボードの寸法・写真のフォルダ（既定 ~/Downloads）を選ぶ
2. 「写真を読み込む」で 📷 の写真（`surge_<cam>_…png`）を拾い、角点を検出する
3. 一覧で使わない写真を外し（ダブルクリックで切替）、「校正」
4. 結果（再投影誤差・画面カバー率・補正前後のプレビュー）を見て、
   よければ「vehicle.toml に書き込む」（`config/generate.py` まで走る）

**ステレオ**（`docs/stereo_plan.md` 段階2）を選ぶと、右（`front`）と左（`left`）の写真を両方読み込み、
「校正」で内部パラメータ2つ → 左 → 右の R・T まで求める。プレビューは平行化した左 | 右に横線を
引いたもの（同じ角が同じ線に乗っていれば OK）。書き込みは左右の `fisheye` と `[sensors.stereo]`。
合格（縦ずれの中央値 0.5px 以下・基線 75±2mm）でなければ書き込まない。

ロジックは `calib.py`（CLI の `python -m tools.cam_calib` と共通）。ネイティブ GUI に
してあるのは、写真の取捨選択と結果の見比べをターミナル操作なしで完結させるため
（`tools/calib/gui.py` と同じ方針）。
"""

from __future__ import annotations

import base64
import math
import threading
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from .calib import (BASELINE_TOL_M, DEFAULT_TOML, MAX_RECT_DY_PX, NOMINAL_BASELINE_M, CalibResult,
                    Detection, StereoCalibration, calibrate, calibrate_stereo_set, coverage, detect,
                    lens_profiles, parse_name, rectified_preview, undistort_preview,
                    write_stereo_set, write_vehicle_toml)

#: カメラの選択肢（値, 表示）。"stereo" は front（右）と left の組
_CAM_CHOICES = (("front", "右（主）"), ("left", "左"), ("stereo", "ステレオ"))

#: プレビューの最大幅 [px]（元画像と補正画像を横に並べた全体）
_PREVIEW_W = 900


class CamCalibApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("カメラ校正 — 魚眼レンズ（チェッカーボード）")
        self.geometry("980x860")

        self.cam = tk.StringVar(value="front")
        #: 校正するレンズ（書き込み先のプロファイル）。"" ＝ プロファイルを使わない書き方
        self.lens = tk.StringVar(value="")
        self.cols = tk.IntVar(value=9)
        self.rows = tk.IntVar(value=6)
        self.square_mm = tk.DoubleVar(value=25.0)
        self.folder = tk.StringVar(value=str(Path.home() / "Downloads"))
        self.toml_path = tk.StringVar(value=str(DEFAULT_TOML))
        self.status = tk.StringVar(value="写真のフォルダを選んで「写真を読み込む」")

        self.dets: dict[str, Detection] = {}     # Treeview の iid → 検出結果
        self.use: dict[str, bool] = {}
        self.result: CalibResult | None = None
        #: ステレオのとき（`self.result` は使わない）
        self.stereo_result: StereoCalibration | None = None
        self._photo: tk.PhotoImage | None = None  # 参照を持っておかないと消える

        self._build()

    # ── 画面構築 ──

    def _build(self) -> None:
        top = ttk.Frame(self, padding=8)
        top.pack(fill="x")
        ttk.Label(top, text="カメラ").grid(row=0, column=0, sticky="w")
        cams = ttk.Frame(top)
        cams.grid(row=0, column=1, columnspan=2, sticky="w")
        for v, t in _CAM_CHOICES:
            ttk.Radiobutton(cams, text=t, value=v, variable=self.cam,
                            command=self._refresh_lenses).pack(side="left")
        lens_box = ttk.Frame(top)
        lens_box.grid(row=1, column=0, columnspan=10, sticky="w", pady=(4, 0))
        ttk.Label(lens_box, text="レンズ").pack(side="left")
        self.lens_combo = ttk.Combobox(lens_box, textvariable=self.lens, width=14, state="readonly")
        self.lens_combo.pack(side="left", padx=4)
        self.lens_note = ttk.Label(lens_box, foreground="gray")
        self.lens_note.pack(side="left")
        self.lens_combo.bind("<<ComboboxSelected>>", lambda _e: self._refresh_lenses(keep=True))
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
        self._refresh_lenses()

    def _refresh_lenses(self, keep: bool = False) -> None:
        """`vehicle.toml` のレンズのプロファイルを読み直して選択肢にする。

        既定は今そのカメラに選ばれているレンズ（＝付いているはずのレンズ）。
        """
        if self.cam.get() == "stereo":
            try:
                lr, _ = lens_profiles("front", self.toml_path.get())
                ll, _ = lens_profiles("left", self.toml_path.get())
            except Exception as e:                          # noqa: BLE001
                lr = ll = ""
                self.lens_note.config(text=f"vehicle.toml を読めない: {e}")
            self._stereo_lenses = (lr, ll)
            self.lens.set("")
            self.lens_combo.config(values=[], state="disabled")
            self.lens_note.config(text=f"右 \"{lr}\"・左 \"{ll}\"（今付いているレンズ。写真もこの名前のものだけ読む）")
            return
        try:
            current, names = lens_profiles(self.cam.get(), self.toml_path.get())
        except Exception as e:                              # noqa: BLE001
            current, names = "", []
            self.lens_note.config(text=f"vehicle.toml を読めない: {e}")
        self.lens_combo.config(values=names, state="readonly" if names else "disabled")
        if not keep or self.lens.get() not in names:
            self.lens.set(current if current in names else (names[0] if names else ""))
        if not names:
            self.lens_note.config(text="（vehicle.toml にレンズのプロファイルが無い）")
        elif self.lens.get() == current:
            self.lens_note.config(text="今付いているレンズ（vehicle.toml の lens）")
        else:
            self.lens_note.config(
                text=f"★今選ばれているのは \"{current}\"。書き込んでも lens を変えるまで使われない")

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

    def _photo_paths(self, folder: Path) -> list[Path]:
        """読み込む写真。別レンズで撮った写真は除く（名前にレンズが無い古い写真は残す）。"""
        if self.cam.get() == "stereo":
            want = dict(zip(("front", "left"), getattr(self, "_stereo_lenses", ("", ""))))
        else:
            want = {self.cam.get(): self.lens.get()}
        out = []
        for cam, lens in want.items():
            out += sorted(p for p in folder.glob(f"surge_{cam}_*.png")
                          if (n := parse_name(p)) and (not lens or n.lens in ("", lens)))
        return out

    def _load(self) -> None:
        cam = self.cam.get()
        folder = Path(self.folder.get()).expanduser()
        paths = self._photo_paths(folder)
        if not paths:
            pat = "surge_front_*.png・surge_left_*.png" if cam == "stereo" else f"surge_{cam}_*.png"
            messagebox.showwarning("写真が無い",
                                   f"{folder} に {pat} がありません。\n"
                                   "GUI のタブバーの 📷 で撮ってください。")
            return
        self.tree.delete(*self.tree.get_children())
        self.dets.clear()
        self.use.clear()
        self.result = None
        self.stereo_result = None
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
        stereo = self.cam.get() == "stereo"

        def work() -> None:
            try:
                if stereo:
                    def of(cam):
                        return [d for d in chosen if (n := parse_name(d.path)) and n.cam == cam]
                    res = calibrate_stereo_set(of("front"), of("left"), board, square)
                else:
                    res = calibrate(chosen, board, square)
            except Exception as e:                          # noqa: BLE001
                self.after(0, self._calib_failed, str(e))
                return
            self.after(0, self._stereo_done if stereo else self._calib_done, res)

        threading.Thread(target=work, daemon=True).start()

    def _calib_failed(self, msg: str) -> None:
        self.calib_btn.config(state="normal")
        self.status.set("校正に失敗")
        messagebox.showerror("校正に失敗", msg)

    def _calib_done(self, res: CalibResult) -> None:
        self.result = res
        self.stereo_result = None
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
            f"{self.cam.get()} カメラ・レンズ {self.lens.get() or '（名前なし）'}  "
            f"{c.width}x{c.height}（クロップ前）  使用 {len(res.per_image)} 枚"
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

    def _stereo_done(self, sc: StereoCalibration) -> None:
        self.stereo_result = sc
        self.result = None
        st = sc.stereo
        self.calib_btn.config(state="normal")
        self.write_btn.config(state="normal" if st.passed else "disabled")
        errs = {**sc.right.per_image, **sc.left.per_image}
        why = {**sc.right.rejected, **sc.left.rejected}
        pair_err = {}
        for pr in sc.pairs:
            if pr.right.path in st.per_pair:
                pair_err[pr.right.path] = pair_err[pr.left.path] = st.per_pair[pr.right.path]
        for p, r in st.rejected.items():
            why[p] = r
        unpaired = set(sc.unpaired)
        for iid in self.dets:
            p = Path(iid)
            vals = list(self.tree.item(iid, "values"))
            if p in why:
                vals[2] = f"除外: {why[p]}"
            elif p in errs:
                vals[2] = (f"単体 {errs[p]:.3f}px・組 {pair_err[p]:.3f}px" if p in pair_err
                           else f"単体 {errs[p]:.3f}px・" + ("相手なし" if p in unpaired else "組に使わない"))
            self.tree.item(iid, values=vals)
        s = st.stereo
        import numpy as np
        ang = math.degrees(math.acos(max(-1.0, min(1.0, (float(np.trace(s.R_mat)) - 1.0) / 2.0))))
        self.summary.delete("1.0", "end")
        self.summary.insert("end", "\n".join([
            f"右 {sc.right.lens}: RMS {sc.right.rms:.3f}px（{len(sc.right.per_image)} 枚）   "
            f"左 {sc.left.lens}: RMS {sc.left.rms:.3f}px（{len(sc.left.per_image)} 枚）",
            f"ステレオ {len(st.per_pair)} 組（相手なし {len(sc.unpaired)} 枚・除外 {len(st.rejected)} 組）"
            f"  再投影誤差 RMS {st.rms:.3f}px",
            f"基線 {s.baseline * 1000:.2f}mm（合格 {NOMINAL_BASELINE_M * 1000:.0f}±{BASELINE_TOL_M * 1000:.0f}mm）"
            f"  T = ({', '.join(f'{v * 1000:.1f}' for v in s.T)}) mm  左右の向きの差 {ang:.2f}°",
            f"平行化後の縦ずれ 中央値 {st.rect_dy_median:.2f}px（合格 {MAX_RECT_DY_PX}px 以下）"
            f"  95% {st.rect_dy_p95:.2f}px  最大 {st.rect_dy_max:.2f}px",
            f"合否: {'合格' if st.passed else '不合格（撮り直しを勧める。書き込めない）'}",
            "写真を選ぶと平行化した左 | 右。同じ角が同じ緑の線に乗っていれば OK",
        ]))
        self.status.set("ステレオ校正できた。" + ("結果を確認して「vehicle.toml に書き込む」"
                                                 if st.passed else "不合格"))
        self._show_preview()

    def _show_preview(self) -> None:
        if self.stereo_result is not None:
            self._show_stereo_preview()
            return
        if self.result is None:
            return
        sel = self.tree.selection()
        iid = sel[0] if sel else next(iter(self.dets), None)
        if iid is None:
            return
        try:
            d = self.dets[iid]
            self._set_preview(undistort_preview(d.path, self.result.calib, d.crop))
        except Exception as e:                              # noqa: BLE001
            self.preview.config(image="", text=f"プレビューを作れない: {e}")

    def _show_stereo_preview(self) -> None:
        sc = self.stereo_result
        sel = self.tree.selection()
        p = Path(sel[0]) if sel else None
        pair = next((pr for pr in sc.pairs if p in (pr.right.path, pr.left.path)),
                    sc.pairs[0] if sc.pairs else None)
        if pair is None:
            return
        try:
            self._set_preview(rectified_preview(pair, sc.right.calib, sc.left.calib,
                                                sc.stereo.stereo))
        except Exception as e:                              # noqa: BLE001
            self.preview.config(image="", text=f"プレビューを作れない: {e}")

    def _set_preview(self, img) -> None:
        try:
            import cv2

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
        if self.stereo_result is not None:
            self._write_stereo()
            return
        if self.result is None:
            return
        cam = self.cam.get()
        lens = self.lens.get()
        where = f"lenses.{lens}." if lens else ""
        if not messagebox.askyesno(
                "vehicle.toml に書き込む",
                f"[sensors.cam_{cam}.{where}fisheye] を書き込み、config/generate.py を実行します。\n"
                f"（再投影誤差 {self.result.rms:.3f}px）"):
            return
        try:
            write_vehicle_toml(cam, self.result, self.toml_path.get(), lens=lens or None)
        except Exception as e:                              # noqa: BLE001
            messagebox.showerror("書き込みに失敗", str(e))
            return
        messagebox.showinfo("書き込んだ",
                            "vehicle.toml を更新しました。\n"
                            "Pi へは tools/deploy.sh --restart で反映してください"
                            "（GUI の再ビルドと telemetry・認識ノードの再起動まで入ります）。")
        self.status.set(f"書き込んだ: {self.toml_path.get()}")

    def _write_stereo(self) -> None:
        sc = self.stereo_result
        if not sc.stereo.passed:
            messagebox.showerror("書き込まない", "合否が不合格。撮り直してください。")
            return
        if not messagebox.askyesno(
                "vehicle.toml に書き込む",
                f"lenses.{sc.right.lens}.fisheye（右）・lenses.{sc.left.lens}.fisheye（左）・"
                f"[sensors.stereo] を書き込み、config/generate.py を実行します。\n"
                f"（基線 {sc.stereo.baseline * 1000:.2f}mm・縦ずれ 中央値 {sc.stereo.rect_dy_median:.2f}px）"):
            return
        try:
            write_stereo_set(sc, self.toml_path.get())
        except Exception as e:                              # noqa: BLE001
            messagebox.showerror("書き込みに失敗", str(e))
            return
        messagebox.showinfo("書き込んだ",
                            "vehicle.toml を更新しました。\n"
                            "Pi へは tools/deploy.sh --restart で反映してください。")
        self.status.set(f"書き込んだ: {self.toml_path.get()}")


def main() -> None:
    CamCalibApp().mainloop()


if __name__ == "__main__":
    main()
