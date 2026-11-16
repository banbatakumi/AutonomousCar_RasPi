"""ml_stereo_e2e/train.py — ステレオ E2E モデルを学習する。

    .venv/bin/python -m ml_stereo_e2e.train --data ml_stereo_e2e/runs/v1/data --out ml_stereo_e2e/runs/v1

`<out>/best.pt`（検証の経路誤差が最小）・`last.pt`・`train_config.json`（入出力の契約。`export_onnx.py` が読む）を書く。

損失・入力を伏せる確率・検証の分け方（記録単位）・向きの拡張の作り方は `ml_e2e/train.py` と同じ（関数をそのまま使う）。
違いは2番目の入力が視差の帯であることと、左右反転をしないこと（`dataset.py`）。視差の帯の設定（照合・行の帯・
縮めた大きさ）は `dataset.json` の `stereo` を契約へ写す（Pi の `stereo/disp` と照合される）。

検証で出す数字は `ml_e2e` と同じ（経路の平均誤差・最初の 2m・速度）に加え、画像・視差・車両状態を伏せたときの経路誤差。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import DataLoader, WeightedRandomSampler

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))
sys.path.insert(0, str(_ROOT / "ml_cam_e2e"))

from yaw_aug import BODY_MASK_LEFT_NAME, camera_fx_px, ensure_body_mask, front_fx_px, margin_for  # noqa: E402

from ml_e2e.train import _avail, _to, evaluate, losses, pick_device  # noqa: E402
from ml_stereo_e2e.dataset import (StereoDataset, StereoSpec, load_records, split_records,  # noqa: E402
                                   stereo_sample_weights)
from ml_stereo_e2e.model import StereoE2EModel  # noqa: E402
from raspi.core.cam_e2e_preproc import PREPROC_VERSION_STEREO, STEREO_CAMS  # noqa: E402
from raspi.core.stereo_e2e_preproc import PREPROC_VERSION  # noqa: E402
from raspi.core.vehicle import Vehicle  # noqa: E402

__all__ = ["main"]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", type=Path, required=True, help="`extract.py` の出力フォルダ")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--epochs", type=int, default=20)
    ap.add_argument("--batch-size", type=int, default=64)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--size", default="224x128", help="画像の入力解像度 幅x高さ")
    ap.add_argument("--k", type=int, default=32, help="経路点の数")
    ap.add_argument("--stride", type=int, default=1, help="抽出した経路点を何個おきに使うか")
    ap.add_argument("--val", nargs="*", default=None, help="検証に回す記録の名前（既定は地図ごとに最後の1本）")
    ap.add_argument("--balance", type=float, default=0.5, help="直進過多の補正 0〜1")
    ap.add_argument("--no-flip", action="store_true",
                    help="（何もしない。左右反転の水増しはもともと無い。`ml_e2e/app.py` と同じ引数を受けるため）")
    ap.add_argument("--yaw-aug-deg", type=float, default=6.0, help="向きをずらす拡張の最大角[deg]（正の値）")
    ap.add_argument("--photo-aug", type=float, default=1.0)
    ap.add_argument("--drop-image", type=float, default=0.15)
    ap.add_argument("--drop-disp", type=float, default=0.15)
    ap.add_argument("--drop-state", type=float, default=0.2)
    ap.add_argument("--w-v", type=float, default=1.0)
    ap.add_argument("--init", type=Path, default=None, help="この重みから始める（追加学習）")
    ap.add_argument("--no-pretrained", action="store_true")
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    vehicle = Vehicle.load()
    w, h = (int(x) for x in args.size.lower().split("x"))
    meta = json.loads((args.data / "dataset.json").read_text())
    stereo = meta.get("stereo")
    if not stereo:
        print("dataset.json に stereo（視差の設定）が無い。`ml_stereo_e2e.extract` で作った標本を使う", file=sys.stderr)
        return 2
    ds = float(meta["config"]["ds"]) * args.stride
    if args.k * args.stride > int(meta["config"]["k"]):
        print(f"経路点が足りません（抽出は {meta['config']['k']} 点）", file=sys.stderr)
        return 2
    if args.yaw_aug_deg <= 0:
        print("--yaw-aug-deg は正の値が要る（左右2台の前処理は左右の切り落としが要る）", file=sys.stderr)
        return 2
    cprs = [r["cols_per_rad"] for r in meta["records"].values() if r.get("cols_per_rad")]
    if not cprs:
        print("視差のある記録がありません。先に `ml_stereo_e2e.extract` で標本を作ってください", file=sys.stderr)
        return 2
    cols_per_rad = float(np.median(cprs))

    all_frames = sorted((args.data / "frames").glob("*/*.jpg"))
    frames = [p for p in all_frames if not p.stem.endswith("_left")]
    frames_left = [p for p in all_frames if p.stem.endswith("_left")]
    from PIL import Image
    src_w = Image.open(frames[0]).size[0]
    fx_px = front_fx_px(vehicle, src_w)
    fx_px_left = camera_fx_px(vehicle, "left", src_w)
    crop_x = margin_for(args.yaw_aug_deg, max(fx_px, fx_px_left)) / src_w
    body_mask = ensure_body_mask(args.data, frames)
    body_mask_left = ensure_body_mask(args.data, frames_left, BODY_MASK_LEFT_NAME)
    print(f"# 向きの拡張: ±{args.yaw_aug_deg:g}°  fx={fx_px:.1f}px  左右の切り落とし {crop_x * 100:.1f}%  "
          f"視差の帯 {cols_per_rad:.1f}列/rad")

    disp_hw = (int(stereo["disp_hw"][0]), int(stereo["disp_hw"][1]))
    spec = StereoSpec(size=(w, h), crop_x=crop_x, k=args.k, stride=args.stride, max_steer=vehicle.max_steer,
                      disp_hw=disp_hw)
    records, _ = load_records(args.data)
    train_r, val_r = split_records(records, args.val)
    if not train_r:
        print("学習に使える記録がありません", file=sys.stderr)
        return 2
    print("# 学習: " + " ".join(f"{r.name}({int(r.data['disp_ok'].sum())})" for r in train_r))
    print("# 検証: " + (" ".join(f"{r.name}({int(r.data['disp_ok'].sum())})" for r in val_r) or "なし"))

    train_ds = StereoDataset(train_r, args.data, spec, cols_per_rad=cols_per_rad, augment=True,
                             yaw_max_deg=args.yaw_aug_deg, fx_px=fx_px, body_mask=body_mask,
                             photo_strength=args.photo_aug, fx_px_left=fx_px_left, body_mask_left=body_mask_left)
    val_ds = StereoDataset(val_r, args.data, spec)
    xy_scale = train_ds.xy_scale
    loader_kw = dict(batch_size=args.batch_size, num_workers=args.workers, persistent_workers=args.workers > 0)
    if args.balance > 0:
        sampler = WeightedRandomSampler(stereo_sample_weights(train_r, spec.max_steer, alpha=args.balance),
                                        num_samples=len(train_ds), replacement=True)
        train_loader = DataLoader(train_ds, sampler=sampler, drop_last=True, **loader_kw)
    else:
        train_loader = DataLoader(train_ds, shuffle=True, drop_last=True, **loader_kw)
    val_loader = DataLoader(val_ds, shuffle=False, **loader_kw) if len(val_ds) else None

    device = pick_device()
    model = StereoE2EModel(disp_hw=disp_hw, k=spec.k, xy_scale=xy_scale, pretrained=not args.no_pretrained)
    if args.init is not None:
        model.load_state_dict(torch.load(args.init, map_location="cpu"))
        print(f"# {args.init} から追加学習")
    model.to(device)
    opt = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)
    steps = args.epochs * len(train_loader)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=args.lr, total_steps=max(1, steps), pct_start=0.1)
    print(f"# device: {device}  学習 {len(train_ds)}件 / 検証 {len(val_ds)}件  経路点 {spec.k}×{ds:g}m "
          f"（{xy_scale:g}m）  視差の帯 {disp_hw[0]}×{disp_hw[1]}")

    args.out.mkdir(parents=True, exist_ok=True)
    config = {
        "preproc_version": PREPROC_VERSION,
        "image_preproc_version": PREPROC_VERSION_STEREO,
        "cams": list(STEREO_CAMS),
        "input_size": [w, h], "crop_x": crop_x,
        "disp_hw": list(disp_hw), "disp_num_disp": int(stereo["num_disp"]), "disp_rows": list(stereo["rows"]),
        "disp_scale": float(stereo["scale"]), "disp_version": int(stereo["disp_version"]),
        "disp_algo": stereo["algo"], "disp_block": int(stereo["block"]), "disp_uniqueness": int(stereo["uniqueness"]),
        "disp_cols_per_rad": cols_per_rad,
        "k": spec.k, "ds": ds, "xy_scale": xy_scale,
        "max_steer": spec.max_steer, "speed_ref": spec.speed_ref, "yaw_rate_ref": spec.yaw_rate_ref,
        "steer_link": list(vehicle.steer_link),
        "yaw_aug_deg": args.yaw_aug_deg, "photo_aug": args.photo_aug,
        "drop": [args.drop_image, args.drop_disp, args.drop_state],
        "epochs": args.epochs, "seed": args.seed, "stride": args.stride,
        "train_records": [r.name for r in train_r], "val_records": [r.name for r in val_r],
        "maps": sorted({m for r in train_r for m in r.maps}),
    }

    def write_config(**extra) -> None:
        (args.out / "train_config.json").write_text(json.dumps({**config, **extra}, indent=2, ensure_ascii=False))

    def ev(avail=(1.0, 1.0, 1.0)):
        return evaluate(model, val_loader, device, spec, xy_scale, ds, avail=avail, second="disp")

    write_config()
    best = math.inf
    best_metrics: dict = {}
    t0 = time.time()
    for epoch in range(1, args.epochs + 1):
        model.train()
        agg = {"path": 0.0, "v": 0.0}
        nb = 0
        for batch in train_loader:
            batch = _to(batch, device)
            a = _avail(batch["image"].shape[0], device, drop_image=args.drop_image,
                       drop_lidar=args.drop_disp, drop_state=args.drop_state)
            ls = losses(model(batch["image"], batch["disp"], batch["state"], a), batch, spec.k, xy_scale)
            loss = ls["path"] + args.w_v * ls["v"]
            opt.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
            opt.step()
            sched.step()
            for key in agg:
                agg[key] += float(ls[key].detach())
            nb += 1
        tr = {key: val / max(1, nb) for key, val in agg.items()}
        line = f"epoch {epoch:3d}/{args.epochs}  loss={sum(tr.values()):.4f}"
        if val_loader is not None:
            m = ev()
            # `ml_common/epoch_log.py` は `loss=` の直後の `val_mae=` を読む。項目は後ろに足す
            line += (f"  val_mae={m['path_m']:.4f}  near={m['near_m']:.3f}m  b={m['b_m']:.2f}m"
                     f"  v={m['v_mps']:.2f}m/s")
            if m["path_m"] < best:
                best, best_metrics = m["path_m"], m
                torch.save(model.state_dict(), args.out / "best.pt")
        print(line + f"  ({time.time() - t0:.0f}s)", flush=True)
    if val_loader is None:
        torch.save(model.state_dict(), args.out / "best.pt")
    torch.save(model.state_dict(), args.out / "last.pt")

    extra: dict = {"best": best_metrics}
    if val_loader is not None:
        model.load_state_dict(torch.load(args.out / "best.pt", map_location=device))
        no_img, no_disp, no_st = ev((0.0, 1.0, 1.0)), ev((1.0, 0.0, 1.0)), ev((1.0, 1.0, 0.0))
        extra.update(no_image=no_img, no_disp=no_disp, no_state=no_st)
        print(f"# 最良: 経路 {best_metrics['path_m']:.3f}m（近く {best_metrics['near_m']:.3f}m）  "
              f"点の速度 {best_metrics['v_mps']:.2f}m/s")
        print(f"# 入力を伏せたときの経路誤差: 画像なし {no_img['path_m']:.3f}m（近く {no_img['near_m']:.3f}m）  "
              f"視差なし {no_disp['path_m']:.3f}m（近く {no_disp['near_m']:.3f}m）  車両状態なし {no_st['path_m']:.3f}m")
    write_config(**extra)
    print(f"# 完了 → {args.out}/best.pt")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
