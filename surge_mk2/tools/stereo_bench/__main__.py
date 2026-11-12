"""ステレオ照合のベンチの CLI。使い方はパッケージの docstring（`tools/stereo_bench/__init__.py`）。"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

from .bench import PRESETS, Geometry, run, wall_check, with_rows
from .data import load_frames


def _rows(s: str) -> tuple[int, int]:
    a, b = (int(v) for v in s.split(":"))
    return a, b


def _fmt(v, pct=False, nd=1):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return "—"
    return f"{v * 100:.{nd}f}%" if pct else f"{v:.{nd}f}"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("mcap", nargs="+", help="記録（logger_node / GUI の MCAP 記録）")
    ap.add_argument("--configs", default="bm,sgbm,bm-half,sgbm-half",
                    help=f"比べる方式（{', '.join(PRESETS)}）")
    ap.add_argument("--rows", type=_rows, default=None, metavar="V0:V1",
                    help="照合する平行化画像の行（640×480 の座標）。既定は全部")
    ap.add_argument("--pair-tol-ms", type=float, default=0.5,
                    help="左右を組にする撮影時刻の差の上限 [ms]（段階3 より前の記録は 40 で止まった場面だけ）")
    ap.add_argument("--stride", type=int, default=1, help="何組おきに使うか")
    ap.add_argument("--max-frames", type=int, default=200, help="1記録あたりの組の上限")
    ap.add_argument("--max-speed", type=float, default=0.05,
                    help="LiDAR と比べる組の車速の上限 [m/s]（負で問わない）")
    ap.add_argument("--band", type=float, default=0.02,
                    help="LiDAR の走査面の高さ ± [m] に入るステレオの点を比べる")
    ap.add_argument("--requality", type=int, default=None, metavar="Q",
                    help="画像を JPEG 品質 Q で焼き直してから照合する（記録より高くはできない）")
    ap.add_argument("--wall", type=float, default=None, metavar="M",
                    help="物差しの確認: LiDAR の中心から正面の壁までの実測距離 [m]")
    ap.add_argument("--threads", type=int, default=1,
                    help="OpenCV のスレッド数（既定 1 ＝ Pi の 1 コアの見積もり用）")
    ap.add_argument("--save", default=None, metavar="DIR",
                    help="平行化した左と視差を並べた絵を書き出す（最初の --save-n 組）")
    ap.add_argument("--save-n", type=int, default=5)
    ap.add_argument("--toml", default=None, help="vehicle.toml（既定は config/vehicle.toml）")
    args = ap.parse_args(argv)

    import cv2

    from raspi.core.vehicle import Vehicle

    cv2.setNumThreads(args.threads)
    try:
        configs = [with_rows(PRESETS[n.strip()], args.rows) for n in args.configs.split(",")]
    except KeyError as e:
        ap.error(f"知らない方式 {e}（{', '.join(PRESETS)}）")
    try:
        geom = Geometry.from_vehicle(Vehicle.load(args.toml) if args.toml else Vehicle.load())
    except ValueError as e:
        print(f"!! {e}", file=sys.stderr)
        return 1

    frames = []
    for p in args.mcap:
        fs = load_frames(p, pair_tol_ns=int(args.pair_tol_ms * 1e6), stride=args.stride,
                         max_frames=args.max_frames)
        n_scan = sum(f.scan is not None for f in fs)
        print(f"{Path(p).name}: 組 {len(fs)}（点群あり {n_scan}）"
              + (f"  左右の時刻差 最大 {max(abs(f.dt_ns) for f in fs) / 1e6:.2f}ms" if fs else ""))
        frames += fs
    if not frames:
        print("!! 左右の組が無い（/viz/image/front と /viz/image/left が両方要る。"
              "左右を同期中に録ったか、--pair-tol-ms を確かめる）", file=sys.stderr)
        return 1
    print(f"平行化: {geom.rect.out_w}×{geom.rect.out_h}・f = {geom.rect.f:.1f}px・"
          f"基線 {geom.rect.baseline * 1000:.1f}mm・最も近い距離（視差 96, 640 幅）"
          f"{geom.rect.f * geom.rect.baseline / 96:.2f}m"
          + (f"・JPEG を品質 {args.requality} で焼き直し" if args.requality else ""))

    if args.wall is not None:
        w = wall_check(frames, geom, configs, args.wall, requality=args.requality, band_m=args.band)
        print(f"\n=== 物差しの確認（壁 {args.wall:.3f}m・正面 ±2°） ===")
        for k, vals in w.items():
            if k == "wall_m":
                continue
            v = np.array(vals, float)
            v = v[np.isfinite(v)]
            if v.size == 0:
                print(f"  {k:10s} 値なし")
                continue
            med = float(np.median(v))
            print(f"  {k:10s} 中央値 {med:.3f}m（壁との差 {(med - args.wall) * 100:+.1f}cm・"
                  f"{(med / args.wall - 1) * 100:+.1f}%）  ばらつき σ {np.std(v) * 100:.1f}cm  {v.size} 組")
        return 0

    saver = None
    if args.save:
        out = Path(args.save)
        out.mkdir(parents=True, exist_ok=True)

        def saver(i, cfg, gl, d):
            if i >= args.save_n:
                return
            vis = np.clip(d / cfg.num_disp * 255, 0, 255).astype(np.uint8)
            vis = cv2.applyColorMap(vis, cv2.COLORMAP_JET)
            vis[d <= 0] = 0
            if cfg.scale != 1.0:
                vis = cv2.resize(vis, (gl.shape[1], vis.shape[0] * gl.shape[1] // vis.shape[1]),
                                 interpolation=cv2.INTER_NEAREST)
            v0, v1 = cfg.rows or (0, gl.shape[0])
            img = np.hstack([cv2.cvtColor(gl[v0:v1], cv2.COLOR_GRAY2BGR), vis[: v1 - v0]])
            cv2.imwrite(str(out / f"{i:03d}_{cfg.name}.png"), img)

    results = run(frames, geom, configs, requality=args.requality, band_m=args.band,
                  max_speed=None if args.max_speed < 0 else args.max_speed, save=saver)
    print("\n方式        時間/組  視差あり  比べた方位  偏り    |誤差|中央  |誤差|90%  5%以内   "
          "|誤差|中央（<1m / 1-2m / ≥2m）")
    for r in results:
        s = r.summary()
        print(f"{s['name']:10s} {_fmt(s['ms']):>6}ms  {_fmt(s['valid'], True):>7}  {s['bins']:>8}  "
              f"{_fmt(s['bias'], True):>7}  {_fmt(s['abs_med'], True):>8}  {_fmt(s['abs_p90'], True):>8}  "
              f"{_fmt(s['within5'], True, 0):>6}   {_fmt(s['abs_med_lt1'], True)} / "
              f"{_fmt(s['abs_med_1to2'], True)} / {_fmt(s['abs_med_ge2'], True)}")
    print("\n（偏り・誤差は (ステレオ − LiDAR)/LiDAR。LiDAR の走査面 ±"
          f"{args.band * 100:.0f}cm の点を 1° の方位ごとに中央値でまとめて比べた）")
    if args.save:
        print(f"絵: {args.save}（左: 平行化した左、右: 視差。黒は視差なし）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
