"""カメラ校正の CLI。画面版は `python -m tools.cam_calib.gui`（ランチャーの「カメラ校正」）。

    # 校正して結果を見る（vehicle.toml は変えない）
    .venv/bin/python -m tools.cam_calib --cam front --board 9x6 --square 0.025 ~/Downloads/surge_front_*.png
    # 問題なければ書き戻す（config/generate.py も走る）
    .venv/bin/python -m tools.cam_calib --cam front --board 9x6 --square 0.025 ~/Downloads/surge_front_*.png --write
    # 印刷用ボード
    .venv/bin/python -m tools.cam_calib --make-board 9x6 --square 0.025 -o board.pdf
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

from .calib import (DEFAULT_TOML, calibrate, coverage, detect, parse_name,
                    undistort_preview, write_vehicle_toml)


def _board(s: str) -> tuple[int, int]:
    c, r = (int(v) for v in s.lower().split("x"))
    return c, r


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("images", nargs="*", help="📷 で撮った PNG（surge_<cam>_…png）")
    ap.add_argument("--cam", choices=("front", "rear"),
                    help="校正するカメラ。省略時はファイル名から（混在はエラー）")
    ap.add_argument("--board", type=_board, default=(9, 6),
                    help="内側の角点の数 列x行（既定 9x6 ＝ 白黒 10x7 マス）")
    ap.add_argument("--square", type=float, default=0.025,
                    help="1マスの実寸 [m]（印刷後に定規で測った値）")
    ap.add_argument("--crop", type=float, default=None,
                    help="下端クロップ率。省略時はファイル名から")
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
    named = {parsed[0] for parsed in map(parse_name, paths) if parsed}
    cam = args.cam or (next(iter(named)) if len(named) == 1 else None)
    if cam is None and len(named) > 1:
        ap.error(f"前後のカメラの写真が混ざっている（{sorted(named)}）。--cam で選ぶ")
    if args.write and cam is None:
        ap.error("--cam を指定する（ファイル名から決まらない）")
    if args.cam:
        # 別カメラの写真（📷 は前後を同時に保存する）は黙って除く
        paths = [p for p in paths if (parse_name(p) or (args.cam,))[0] == args.cam]

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
    print(f"\n=== {cam or '?'} カメラ（{c.width}x{c.height}・クロップ前） ===")
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
        write_vehicle_toml(cam, res, args.toml)
        print(f"書き戻した: {args.toml} [sensors.cam_{cam}.fisheye]（Pi へは tools/deploy.sh --restart）")
    else:
        print("\n（--write で config/vehicle.toml に書き戻す）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
