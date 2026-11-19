"""ml_stereo_e2e/train.py — ステレオ E2E モデルを学習する。

    .venv/bin/python -m ml_stereo_e2e.train --data ml_stereo_e2e/runs/v1/data --out ml_stereo_e2e/runs/v1

`<out>/best.pt`（検証の経路誤差が最小）・`last.pt`・`train_config.json`（入出力の契約。`export_onnx.py` が読む）を書く。

損失・入力を伏せる確率・検証の分け方（記録単位）・向きの拡張の作り方・学習ループ（`fit`）・共通の引数は
`ml_e2e/train.py` と同じ（関数をそのまま使う。ここに書くのは視差に関わる差分だけ）。
違いは2番目の入力が視差の帯であることと、左右反転をしないこと（`dataset.py`）。視差の帯の設定（照合・行の帯・
縮めた大きさ）は `dataset.json` の `stereo` を契約へ写す（Pi の `stereo/disp` と照合される）。

**校正の値（視差の帯の焦点距離・主点、カメラの fx、`max_steer`・`steer_link`）は `dataset.json` の記録ごとの
`calib_values`（抽出したときの記録の校正）だけを使い、今の `vehicle.toml` は見ない**（レビュー M10）。

検証で出す数字は `ml_e2e` と同じ（経路の平均誤差・最初の 2m・速度）に加え、画像・視差・車両状態を伏せたときの経路誤差。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import torch

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))

from ml_common.train_utils import pick_device, seed_all  # noqa: E402
from ml_e2e.train import add_common_args, fit, image_crop, list_frames, make_loaders, read_path_ds  # noqa: E402
from ml_stereo_e2e.dataset import (StereoDataset, StereoSpec, load_records, split_records,  # noqa: E402
                                   stereo_sample_weights)
from ml_stereo_e2e.model import StereoE2EModel  # noqa: E402
from raspi.core.cam_e2e_preproc import PREPROC_VERSION_STEREO, STEREO_CAMS  # noqa: E402
from raspi.core.stereo_e2e_preproc import PREPROC_VERSION, disp_margin_cols  # noqa: E402

__all__ = ["main"]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(ap, second="disp",
                    no_flip_help="（何もしない。左右反転の水増しはもともと無い。`ml_e2e/app.py` と同じ引数を受けるため）")
    args = ap.parse_args()

    seed_all(args.seed)
    w, h = (int(x) for x in args.size.lower().split("x"))
    meta = json.loads((args.data / "dataset.json").read_text())
    stereo = meta.get("stereo")
    if not stereo:
        print("dataset.json に stereo（視差の設定）が無い。`ml_stereo_e2e.extract` で作った標本を使う", file=sys.stderr)
        return 2
    ds = read_path_ds(meta, args)
    if ds is None:
        return 2
    if args.yaw_aug_deg <= 0:
        print("--yaw-aug-deg は正の値が要る（左右2台の前処理は左右の切り落としが要る）", file=sys.stderr)
        return 2
    records, _ = load_records(args.data)
    calib = {r.name: meta["records"].get(r.name, {}).get("calib_values") for r in records}
    no_calib = [name for name, c in calib.items() if not c]
    if not records or no_calib:
        print("視差のある記録が無い、または校正の値（calib_values）の無い記録がある"
              f"{'（' + '・'.join(no_calib) + '）' if no_calib else ''}。"
              "`ml_stereo_e2e.extract` で標本を作り直してください", file=sys.stderr)
        return 2
    # 舵角の意味（max_steer・steer_link）は全記録で同じでなければ混ぜられない
    steer = {(round(c["max_steer"], 6), tuple(round(x, 6) for x in c["steer_link"])) for c in calib.values()}
    if len(steer) != 1:
        print(f"記録によって max_steer・steer_link が違う（{sorted(steer)}）。同じ校正の記録だけで学習する",
              file=sys.stderr)
        return 2
    (max_steer, steer_link), = steer
    src_w = {c["image_w"] for c in calib.values()}
    if len(src_w) != 1:
        print(f"記録によって画像の幅が違う（{sorted(src_w)}）", file=sys.stderr)
        return 2
    src_w = src_w.pop()
    fx_px = float(np.median([c["fx_px"] for c in calib.values()]))
    fx_px_left = float(np.median([c["fx_px_left"] for c in calib.values()]))
    band_geom = {name: (float(c["band_f"]), float(c["band_cx"])) for name, c in calib.items()}
    disp_hw_full = (int(stereo["disp_hw"][0]), int(stereo["disp_hw"][1]))
    # 視差の帯の窓: どの記録の帯でも、±yaw_aug_deg のずらしで空いた列が入らない幅
    crop_cols = max(disp_margin_cols(math.radians(args.yaw_aug_deg), f, cx, disp_hw_full[1])
                    for f, cx in band_geom.values())
    disp_hw = (disp_hw_full[0], disp_hw_full[1] - 2 * crop_cols)
    f_med = float(np.median([g[0] for g in band_geom.values()]))
    cx_med = float(np.median([g[1] for g in band_geom.values()]))

    frames, frames_left = list_frames(args.data)
    if not frames or not frames_left:
        print(f"{args.data / 'frames'} に左右の画像がありません", file=sys.stderr)
        return 2
    crop_x, body_mask, body_mask_left = image_crop(args.data, frames, frames_left, yaw_aug_deg=args.yaw_aug_deg,
                                                   fx_px=fx_px, fx_px_left=fx_px_left, src_w=src_w)
    print(f"# 向きの拡張: ±{args.yaw_aug_deg:g}°  fx={fx_px:.1f}px  左右の切り落とし {crop_x * 100:.1f}%  "
          f"視差の帯 {disp_hw_full[1]}列のうち両端 {crop_cols}列を落とした窓")

    spec = StereoSpec(size=(w, h), crop_x=crop_x, k=args.k, stride=args.stride, max_steer=max_steer,
                      disp_hw=disp_hw)
    train_r, val_r = split_records(records, args.val)
    if not train_r:
        print("学習に使える記録がありません", file=sys.stderr)
        return 2
    print("# 学習: " + " ".join(f"{r.name}({int(r.data['disp_ok'].sum())})" for r in train_r))
    print("# 検証: " + (" ".join(f"{r.name}({int(r.data['disp_ok'].sum())})" for r in val_r) or "なし"))

    train_ds = StereoDataset(train_r, args.data, spec, band_geom=band_geom, crop_cols=crop_cols, augment=True,
                             yaw_max_deg=args.yaw_aug_deg, fx_px=fx_px, body_mask=body_mask,
                             photo_strength=args.photo_aug, fx_px_left=fx_px_left, body_mask_left=body_mask_left)
    val_ds = StereoDataset(val_r, args.data, spec, band_geom=band_geom, crop_cols=crop_cols)
    xy_scale = train_ds.xy_scale
    weights = stereo_sample_weights(train_r, spec.max_steer, alpha=args.balance) if args.balance > 0 else None
    train_loader, val_loader = make_loaders(train_ds, val_ds, weights, args)

    device = pick_device()
    model = StereoE2EModel(disp_hw=disp_hw, k=spec.k, xy_scale=xy_scale, pretrained=not args.no_pretrained)
    if args.init is not None:
        model.load_state_dict(torch.load(args.init, map_location="cpu"))
        print(f"# {args.init} から追加学習")
    model.to(device)
    print(f"# device: {device}  学習 {len(train_ds)}件 / 検証 {len(val_ds)}件  経路点 {spec.k}×{ds:g}m "
          f"（{xy_scale:g}m）  視差の帯 {disp_hw[0]}×{disp_hw[1]}")

    args.out.mkdir(parents=True, exist_ok=True)
    config = {
        "preproc_version": PREPROC_VERSION,
        "image_preproc_version": PREPROC_VERSION_STEREO,
        "cams": list(STEREO_CAMS),
        "input_size": [w, h], "crop_x": crop_x,
        "disp_hw": list(disp_hw_full), "disp_crop_cols": crop_cols, "disp_num_disp": int(stereo["num_disp"]), "disp_rows": list(stereo["rows"]),
        "disp_scale": float(stereo["scale"]), "disp_version": int(stereo["disp_version"]),
        "disp_algo": stereo["algo"], "disp_block": int(stereo["block"]), "disp_uniqueness": int(stereo["uniqueness"]),
        "disp_col_f": f_med, "disp_col_cx": cx_med,
        "k": spec.k, "ds": ds, "xy_scale": xy_scale,
        "max_steer": spec.max_steer, "speed_ref": spec.speed_ref, "yaw_rate_ref": spec.yaw_rate_ref,
        "steer_link": list(steer_link),
        "yaw_aug_deg": args.yaw_aug_deg, "photo_aug": args.photo_aug,
        "drop": [args.drop_image, args.drop_disp, args.drop_state],
        "epochs": args.epochs, "seed": args.seed, "stride": args.stride,
        "train_records": [r.name for r in train_r], "val_records": [r.name for r in val_r],
        "maps": sorted({m for r in train_r for m in r.maps}),
    }

    def write_config(**extra) -> None:
        (args.out / "train_config.json").write_text(json.dumps({**config, **extra}, indent=2, ensure_ascii=False))

    write_config()
    fit(model, train_loader, val_loader, device, spec, xy_scale, ds, args, second="disp", second_label="視差",
        drop_second=args.drop_disp, write_config=write_config)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
