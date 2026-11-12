"""カメラ校正の CLI。画面版は `python -m tools.cam_calib.gui`（ランチャーの「カメラ校正」）。

    # 校正して結果を見る（vehicle.toml は変えない）
    .venv/bin/python -m tools.cam_calib --cam front --board 9x6 --square 0.025 ~/Downloads/surge_front_*.png
    # 問題なければ書き戻す（config/generate.py も走る）
    .venv/bin/python -m tools.cam_calib --cam front --board 9x6 --square 0.025 ~/Downloads/surge_front_*.png --write
    # 書き込み先のレンズ（vehicle.toml の lenses.<名前>）は写真の名前から。明示するなら --lens wide160
    # ステレオ: 右（front）と左（left）の写真をまとめて渡すと、内部パラメータ2つ → R・T（docs/stereo_plan.md 段階2）
    .venv/bin/python -m tools.cam_calib --stereo --board 9x6 --square 0.025 ~/Downloads/surge_{front,left}_*.png --write
    # 印刷用ボード
    .venv/bin/python -m tools.cam_calib --make-board 9x6 --square 0.025 -o board.pdf
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

from .calib import (CAMS, DEFAULT_TOML, MAX_RECT_DY_PX, BASELINE_TOL_M, NOMINAL_BASELINE_M,
                    calibrate, calibrate_stereo_set, coverage, detect, lens_profiles, parse_name,
                    rectified_preview, undistort_preview, write_stereo_set, write_vehicle_toml)


def _board(s: str) -> tuple[int, int]:
    c, r = (int(v) for v in s.lower().split("x"))
    return c, r


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("images", nargs="*", help="📷 で撮った PNG（surge_<cam>_…png）")
    ap.add_argument("--cam", choices=CAMS,
                    help="校正するカメラ。省略時はファイル名から（混在はエラー）")
    ap.add_argument("--board", type=_board, default=(9, 6),
                    help="内側の角点の数 列x行（既定 9x6 ＝ 白黒 10x7 マス）")
    ap.add_argument("--square", type=float, default=0.025,
                    help="1マスの実寸 [m]（印刷後に定規で測った値）")
    ap.add_argument("--crop", type=float, default=None,
                    help="下端クロップ率。省略時はファイル名から")
    ap.add_argument("--lens", default=None,
                    help="レンズのプロファイル名（vehicle.toml の lenses.<名前>）。写真を絞り込み、"
                         "書き込み先にもなる。省略時は写真のファイル名のレンズ")
    ap.add_argument("--stereo", action="store_true",
                    help="ステレオ校正（front＝右と left の写真を両方渡す。--cam・--lens は使わない）")
    ap.add_argument("--write", action="store_true",
                    help="結果を config/vehicle.toml に書き戻す")
    ap.add_argument("--toml", default=str(DEFAULT_TOML))
    ap.add_argument("--preview-dir", default=None,
                    help="補正前後を並べたプレビュー PNG の出力先")
    ap.add_argument("--make-board", type=_board, default=None, metavar="COLSxROWS",
                    help="印刷用ボードを作る（内側の角点の数）")
    ap.add_argument("-o", "--output", default="checkerboard.pdf")
    ap.add_argument("--dpi", type=int, default=300)
    args = ap.parse_args(argv)

    if args.make_board:
        from .board import save_board
        save_board(args.output, args.make_board, args.square, args.dpi)
        print(f"保存: {args.output}（1マス {args.square * 1000:.1f}mm・{args.dpi}dpi。"
              "「実際のサイズ」で印刷し、定規で測った値を --square に入れる）")
        return 0

    paths = [Path(p).expanduser() for p in args.images]
    if not paths:
        ap.error("写真を指定する")
    if args.stereo:
        if args.cam or args.lens:
            ap.error("--stereo では --cam・--lens を使わない（レンズは写真の名前から）")
        return _stereo(args, paths)
    named = {parsed.cam for parsed in map(parse_name, paths) if parsed}
    cam = args.cam or (next(iter(named)) if len(named) == 1 else None)
    if cam is None and len(named) > 1:
        ap.error(f"左右のカメラの写真が混ざっている（{sorted(named)}）。--cam で選ぶ")
    if args.write and cam is None:
        ap.error("--cam を指定する（ファイル名から決まらない）")
    if args.cam:
        # 別カメラの写真（📷 は左右を同時に保存する）は黙って除く
        paths = [p for p in paths if (n := parse_name(p)) is None or n.cam == args.cam]
    if args.lens:
        # 別レンズで撮った写真は除く（名前にレンズが無い古い写真は残す）
        paths = [p for p in paths
                 if (n := parse_name(p)) is None or n.lens in ("", args.lens)]

    dets = []
    for p in paths:
        d = detect(p, args.board, args.crop)
        dets.append(d)
        print(f"  {'✓' if d.ok else '✗'} {p.name}{'' if d.ok else '  ' + d.error}")
    try:
        res = calibrate(dets, args.board, args.square)
    except ValueError as e:
        print(f"!! {e}", file=sys.stderr)
        return 1

    c = res.calib
    lens = args.lens or res.lens
    print(f"\n=== {cam or '?'} カメラ・レンズ {lens or '（名前なし）'}"
          f"（{c.width}x{c.height}・クロップ前） ===")
    print(f"再投影誤差 RMS {res.rms:.3f}px（目安 0.5px 未満）  使用 {len(res.per_image)} 枚")
    print(f"fx={c.fx:.2f} fy={c.fy:.2f} cx={c.cx:.2f} cy={c.cy:.2f}")
    print(f"k = {', '.join(f'{v:.6f}' for v in c.k)}")
    print(f"水平画角 {math.degrees(res.hfov):.1f}°  画面カバー率 {coverage(dets) * 100:.0f}%"
          "（端・四隅まで写すほどよい。目安 80%以上）")
    for p, e in sorted(res.per_image.items(), key=lambda kv: -kv[1]):
        print(f"  {e:6.3f}px  {p.name}")
    for p, why in res.rejected.items():
        print(f"  除外  {p.name}: {why}")

    if args.preview_dir:
        import cv2
        out = Path(args.preview_dir)
        out.mkdir(parents=True, exist_ok=True)
        for d in dets:
            if d.ok:
                cv2.imwrite(str(out / f"undist_{d.path.name}"),
                            undistort_preview(d.path, c, d.crop))
        print(f"プレビュー: {out}")

    if args.write:
        try:
            target = write_vehicle_toml(cam, res, args.toml, lens=args.lens)
        except ValueError as e:
            print(f"!! {e}", file=sys.stderr)
            return 1
        where = f"lenses.{target}." if target else ""
        print(f"書き戻した: {args.toml} [sensors.cam_{cam}.{where}fisheye]"
              "（Pi へは tools/deploy.sh --restart）")
        cur, _ = lens_profiles(cam, args.toml)
        if target and cur != target:
            print(f"!! 今選ばれているレンズは \"{cur}\"。この校正値を使うには "
                  f"[sensors.cam_{cam}] の lens を \"{target}\" にする")
    else:
        print("\n（--write で config/vehicle.toml に書き戻す）")
    return 0


def _stereo(args, paths: list[Path]) -> int:
    by_cam: dict[str, list] = {"front": [], "left": []}
    for p in paths:
        n = parse_name(p)
        if n is None or n.cam not in by_cam:
            print(f"  -  {p.name}  （front/left の 📷 の写真ではないので使わない）")
            continue
        d = detect(p, args.board, args.crop)
        by_cam[n.cam].append(d)
        print(f"  {'✓' if d.ok else '✗'} {p.name}{'' if d.ok else '  ' + d.error}")
    try:
        sc = calibrate_stereo_set(by_cam["front"], by_cam["left"], args.board, args.square)
    except ValueError as e:
        print(f"!! {e}", file=sys.stderr)
        return 1
    for name, r in (("右（front）", sc.right), ("左（left）", sc.left)):
        c = r.calib
        print(f"\n=== {name}・レンズ {r.lens or '（名前なし）'} ===")
        print(f"再投影誤差 RMS {r.rms:.3f}px  使用 {len(r.per_image)} 枚  除外 {len(r.rejected)} 枚")
        print(f"fx={c.fx:.2f} fy={c.fy:.2f} cx={c.cx:.2f} cy={c.cy:.2f}  水平画角 {math.degrees(r.hfov):.1f}°")
    st = sc.stereo
    s = st.stereo
    print(f"\n=== ステレオ（{len(st.per_pair)} 組・相手なし {len(sc.unpaired)} 枚・除外 {len(st.rejected)} 組） ===")
    print(f"再投影誤差 RMS {st.rms:.3f}px")
    print(f"T = ({', '.join(f'{v * 1000:.2f}' for v in s.T)}) mm   基線 {s.baseline * 1000:.2f}mm"
          f"（合格 {NOMINAL_BASELINE_M * 1000:.0f}±{BASELINE_TOL_M * 1000:.0f}mm）")
    import numpy as np
    ang = math.degrees(math.acos(max(-1.0, min(1.0, (float(np.trace(s.R_mat)) - 1.0) / 2.0))))
    print(f"左右の向きの差 {ang:.2f}°")
    print(f"平行化後の縦ずれ 中央値 {st.rect_dy_median:.2f}px（合格 {MAX_RECT_DY_PX}px 以下）"
          f"  95% {st.rect_dy_p95:.2f}px  最大 {st.rect_dy_max:.2f}px")
    print(f"合否: {'合格' if st.passed else '不合格'}")
    for p, e in sorted(st.per_pair.items(), key=lambda kv: -kv[1]):
        print(f"  {e:6.3f}px  {p.name}")
    for p, why in st.rejected.items():
        print(f"  除外  {p.name}: {why}")
    for p in sc.unpaired:
        print(f"  相手なし  {p.name}")

    if args.preview_dir:
        import cv2
        out = Path(args.preview_dir)
        out.mkdir(parents=True, exist_ok=True)
        for pr in sc.pairs:
            cv2.imwrite(str(out / f"rect_{pr.right.path.name}"),
                        rectified_preview(pr, sc.right.calib, sc.left.calib, s))
        print(f"平行化のプレビュー（左 | 右・横線）: {out}")

    if args.write:
        if not st.passed:
            print("!! 不合格なので書き込まない（撮り直しを勧める）", file=sys.stderr)
            return 1
        try:
            lr, ll = write_stereo_set(sc, args.toml)
        except ValueError as e:
            print(f"!! {e}", file=sys.stderr)
            return 1
        print(f"書き戻した: {args.toml} の lenses.{lr}.fisheye・lenses.{ll}.fisheye・[sensors.stereo]"
              "（Pi へは tools/deploy.sh --restart）")
    else:
        print("\n（--write で config/vehicle.toml に書き戻す）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
