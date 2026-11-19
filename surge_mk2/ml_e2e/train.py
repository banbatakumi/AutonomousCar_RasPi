"""ml_e2e/train.py — 融合 E2E モデルを学習する。

    .venv/bin/python -m ml_e2e.train --data ml_e2e/runs/v1/data --out ml_e2e/runs/v1

`<out>/best.pt`（検証の経路誤差が最小）・`last.pt`・`train_config.json`（入出力の契約。`export_onnx.py` が読む）を書く。

## 損失

- **経路点**: ラプラス分布の負の対数尤度 `|誤差|/b + log b`（x・y それぞれ。`b` はモデルが出す不確かさ [m]）。
  `b` を出させない素の L1 も少し足す（`b` を大きくして逃げる初期の崩れを抑える）
- **各点の速度**: L1。手本にしてよい標本だけ（`v_ok`）

## 入力を伏せる

標本ごとに、画像・点群・車両状態のどれかを確率 `--drop-*` で伏せる（画像と点群を同時には伏せない）。
理由は `model.py` の docstring。左右2台の画像は組で伏せる（推論でも組が揃わなければ画像を伏せる）。

## 左右2台

`ml_e2e.extract` が左右の組で取り出した標本で学習する。画像の前処理は版3（`cam_e2e_preproc.PREPROC_VERSION_STEREO`。
左右の切り落としが要るので `--yaw-aug-deg` は正の値）。車体マスクと fx はカメラごとに持つ（左は `body_mask_left.png`）。

## 検証

記録単位で分ける（`dataset.split_records`）。出す数字は、経路点の平均誤差 [m]（全点・最初の 2m）、
各点の速度の誤差 [m/s]、それに**画像を伏せたとき・点群を伏せたときの経路誤差**
（片方の入力だけで経路が出るか）。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Callable

import torch
from torch.utils.data import DataLoader, WeightedRandomSampler

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))
sys.path.insert(0, str(_ROOT / "ml_cam_e2e"))

from yaw_aug import BODY_MASK_LEFT_NAME, camera_fx_px, ensure_body_mask, front_fx_px, margin_for  # noqa: E402

from ml_common.train_utils import BestTracker, make_optimizer, pick_device, seed_all  # noqa: E402
from ml_e2e.dataset import FusionDataset, InputSpec, load_records, sample_weights, split_records  # noqa: E402
from ml_e2e.model import FusionModel, load_weights  # noqa: E402
from raspi.core.cam_e2e_preproc import PREPROC_VERSION_STEREO, STEREO_CAMS  # noqa: E402
from raspi.core.e2e_fusion_preproc import PREPROC_VERSION  # noqa: E402
from raspi.core.vehicle import Vehicle  # noqa: E402

__all__ = ["pick_device", "split_output", "losses", "evaluate", "add_common_args", "read_path_ds", "list_frames",
           "image_crop", "make_loaders", "fit", "main"]

#: 素の L1 を足す重み（経路点の NLL に対して）
_L1_WEIGHT = 0.5
#: 「近く」の経路誤差を出す範囲 [m]
_NEAR_M = 2.0


def split_output(out: torch.Tensor, k: int):
    """モデルの出力 → `(xy (N,K,2), v (N,K), b (N,K))`。`xy` は正規化値、`b` は m。"""
    xy = torch.stack([out[:, :k], out[:, k:2 * k]], dim=2)
    return xy, out[:, 2 * k:3 * k], out[:, 3 * k:4 * k]


def _masked_mean(x: torch.Tensor, m: torch.Tensor) -> torch.Tensor:
    return (x * m).sum() / m.sum().clamp(min=1.0)


def losses(out: torch.Tensor, batch: dict, k: int, xy_scale: float) -> dict[str, torch.Tensor]:
    xy, v, b = split_output(out, k)
    pt_ok = batch["pt_ok"]
    err_m = (xy - batch["path"]).abs().sum(dim=2) * xy_scale          # |dx|+|dy| [m]
    nll = _masked_mean(err_m / b + 2.0 * torch.log(b), pt_ok)
    l1 = _masked_mean(err_m, pt_ok)
    v_mask = pt_ok * batch["v_ok"][:, None]
    return {
        "path": nll + _L1_WEIGHT * l1,
        "v": _masked_mean((v - batch["path_v"]).abs(), v_mask),
    }


def _avail(n: int, device, *, drop_image: float = 0.0, drop_lidar: float = 0.0, drop_state: float = 0.0
           ) -> torch.Tensor:
    a = torch.ones(n, 3, device=device)
    u = torch.rand(n, device=device)
    a[:, 0] = (u >= drop_image).float()                                   # 画像を伏せる
    a[:, 1] = ((u < drop_image) | (u >= drop_image + drop_lidar)).float()  # 点群を伏せる（画像とは排他）
    a[:, 2] = (torch.rand(n, device=device) >= drop_state).float()
    return a


def _to(batch: dict, device) -> dict:
    return {key: val.to(device) for key, val in batch.items()}


@torch.no_grad()
def evaluate(model: FusionModel, loader, device, spec: InputSpec, xy_scale: float, ds: float,
             avail: tuple[float, float, float] = (1.0, 1.0, 1.0), *, second: str = "lidar") -> dict[str, float]:
    """検証の数字（物理量）。`avail` で入力を伏せた評価ができる。

    `second` はモデルの2番目の入力の名前（`ml_stereo_e2e` は視差の `disp`）。
    """
    model.eval()
    k = spec.k
    near = max(1, int(_NEAR_M / ds))
    s = {key: 0.0 for key in ("pt", "pt_n", "near", "near_n", "v", "v_n", "b")}
    for batch in loader:
        batch = _to(batch, device)
        n = batch["image"].shape[0]
        a = torch.tensor(avail, device=device).expand(n, 3)
        xy, v, b = split_output(model(batch["image"], batch[second], batch["state"], a), k)
        dist = torch.linalg.norm(xy - batch["path"], dim=2) * xy_scale
        ok = batch["pt_ok"]
        s["pt"] += float((dist * ok).sum()); s["pt_n"] += float(ok.sum())
        s["near"] += float((dist[:, :near] * ok[:, :near]).sum()); s["near_n"] += float(ok[:, :near].sum())
        s["b"] += float((b * ok).sum())
        vm = ok * batch["v_ok"][:, None]
        s["v"] += float(((v - batch["path_v"]).abs() * vm).sum()); s["v_n"] += float(vm.sum())

    def div(a, b):
        return s[a] / s[b] if s[b] > 0 else math.nan

    return {"path_m": div("pt", "pt_n"), "near_m": div("near", "near_n"), "b_m": div("b", "pt_n"),
            "v_mps": div("v", "v_n") * spec.speed_ref}


# ── `ml_stereo_e2e/train.py` と共通の手順（2番目の入力が点群か視差かだけが違う） ──

def add_common_args(ap: argparse.ArgumentParser, *, second: str, no_flip_help: str | None = None) -> None:
    """融合系の学習で共通の引数。`second` は2番目の入力の名前（`--drop-<second>` になる）。"""
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
    ap.add_argument("--no-flip", action="store_true", help=no_flip_help)
    ap.add_argument("--yaw-aug-deg", type=float, default=6.0, help="向きをずらす拡張の最大角[deg]（正の値）")
    ap.add_argument("--photo-aug", type=float, default=1.0)
    ap.add_argument("--drop-image", type=float, default=0.15)
    ap.add_argument(f"--drop-{second}", type=float, default=0.15)
    ap.add_argument("--drop-state", type=float, default=0.2)
    ap.add_argument("--w-v", type=float, default=1.0)
    ap.add_argument("--init", type=Path, default=None, help="この重みから始める（追加学習）")
    ap.add_argument("--no-pretrained", action="store_true")
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--seed", type=int, default=0)


def read_path_ds(meta: dict, args) -> float | None:
    """経路点の間隔 [m]。抽出した点の数が `--k`×`--stride` に足りなければ `None`（理由は表示済み）。"""
    if args.k * args.stride > int(meta["config"]["k"]):
        print(f"経路点が足りません（抽出は {meta['config']['k']} 点）", file=sys.stderr)
        return None
    return float(meta["config"]["ds"]) * args.stride


def list_frames(data_dir: Path) -> tuple[list[Path], list[Path]]:
    """`(右の画像, 左の画像)`。左は名前の末尾が `_left`。"""
    all_frames = sorted((data_dir / "frames").glob("*/*.jpg"))
    return ([p for p in all_frames if not p.stem.endswith("_left")],
            [p for p in all_frames if p.stem.endswith("_left")])


def image_crop(data_dir: Path, frames: list[Path], frames_left: list[Path], *, yaw_aug_deg: float,
               fx_px: float, fx_px_left: float, src_w: int):
    """`(左右の切り落とし crop_x, 右の車体マスク, 左の車体マスク)`。切り落としは左右の fx の大きい方で決める
    （どちらのカメラでも、±`yaw_aug_deg` のずらしで空いた列が窓に入らない幅）。"""
    crop_x = margin_for(yaw_aug_deg, max(fx_px, fx_px_left)) / src_w
    return (crop_x, ensure_body_mask(data_dir, frames),
            ensure_body_mask(data_dir, frames_left, BODY_MASK_LEFT_NAME))


def make_loaders(train_ds, val_ds, weights, args):
    """`(学習の loader, 検証の loader または None)`。`weights` があれば直進過多の補正の重み付き抽選。"""
    loader_kw = dict(batch_size=args.batch_size, num_workers=args.workers, persistent_workers=args.workers > 0)
    if weights is not None:
        sampler = WeightedRandomSampler(weights, num_samples=len(train_ds), replacement=True)
        train_loader = DataLoader(train_ds, sampler=sampler, drop_last=True, **loader_kw)
    else:
        train_loader = DataLoader(train_ds, shuffle=True, drop_last=True, **loader_kw)
    val_loader = DataLoader(val_ds, shuffle=False, **loader_kw) if len(val_ds) else None
    return train_loader, val_loader


def fit(model, train_loader, val_loader, device, spec, xy_scale: float, ds: float, args, *,
        second: str, second_label: str, drop_second: float, write_config: Callable[..., None]) -> None:
    """学習ループ（エポックごとの検証・best/last の保存・最後に入力を伏せた評価）。

    `second` はモデルの2番目の入力（batch のキー。`lidar` か `disp`）、`second_label` は表示の名前。
    best は検証の経路誤差（`path_m`）が最小のエポック。検証が無ければ最後のエポックを best.pt にする。
    """
    opt = make_optimizer(model.parameters(), lr=args.lr, weight_decay=1e-4)
    steps = args.epochs * len(train_loader)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=args.lr, total_steps=max(1, steps), pct_start=0.1)

    def ev(avail=(1.0, 1.0, 1.0)):
        return evaluate(model, val_loader, device, spec, xy_scale, ds, avail=avail, second=second)

    best = BestTracker("min")
    t0 = time.time()
    for epoch in range(1, args.epochs + 1):
        model.train()
        agg = {"path": 0.0, "v": 0.0}
        nb = 0
        for batch in train_loader:
            batch = _to(batch, device)
            a = _avail(batch["image"].shape[0], device, drop_image=args.drop_image,
                       drop_lidar=drop_second, drop_state=args.drop_state)
            ls = losses(model(batch["image"], batch[second], batch["state"], a), batch, spec.k, xy_scale)
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
            if best.update(m["path_m"], m):
                torch.save(model.state_dict(), args.out / "best.pt")
        print(line + f"  ({time.time() - t0:.0f}s)", flush=True)
    if val_loader is None:
        torch.save(model.state_dict(), args.out / "best.pt")
    torch.save(model.state_dict(), args.out / "last.pt")

    best_metrics = best.info or {}
    extra: dict = {"best": best_metrics}
    if val_loader is not None:
        model.load_state_dict(torch.load(args.out / "best.pt", map_location=device))
        no_img, no_2nd, no_st = ev((0.0, 1.0, 1.0)), ev((1.0, 0.0, 1.0)), ev((1.0, 1.0, 0.0))
        extra.update({"no_image": no_img, f"no_{second}": no_2nd, "no_state": no_st})
        print(f"# 最良: 経路 {best_metrics['path_m']:.3f}m（近く {best_metrics['near_m']:.3f}m）  "
              f"点の速度 {best_metrics['v_mps']:.2f}m/s")
        print(f"# 入力を伏せたときの経路誤差: 画像なし {no_img['path_m']:.3f}m（近く {no_img['near_m']:.3f}m）  "
              f"{second_label}なし {no_2nd['path_m']:.3f}m（近く {no_2nd['near_m']:.3f}m）  "
              f"車両状態なし {no_st['path_m']:.3f}m")
    write_config(**extra)
    print(f"# 完了 → {args.out}/best.pt")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(ap, second="lidar")
    ap.add_argument("--fov-deg", type=int, default=270)
    ap.add_argument("--max-range", type=float, default=8.0)
    args = ap.parse_args()

    seed_all(args.seed)
    vehicle = Vehicle.load()
    w, h = (int(x) for x in args.size.lower().split("x"))
    meta = json.loads((args.data / "dataset.json").read_text())
    ds = read_path_ds(meta, args)
    if ds is None:
        return 2

    margin_deg = int(math.ceil(args.yaw_aug_deg))
    frames, frames_left = list_frames(args.data)
    if not frames:
        print("画像がありません。先に `ml_e2e.extract` で標本を作ってください", file=sys.stderr)
        return 2
    if not frames_left:
        print("左の画像がありません。`ml_e2e.extract` で左右の組を取り出し直してください", file=sys.stderr)
        return 2
    if args.yaw_aug_deg <= 0:
        print("--yaw-aug-deg は正の値が要る（左右2台の前処理は左右の切り落としが要る）", file=sys.stderr)
        return 2
    from PIL import Image
    src_w = Image.open(frames[0]).size[0]
    fx_px = front_fx_px(vehicle, src_w)
    fx_px_left = camera_fx_px(vehicle, "left", src_w)
    crop_x, body_mask, body_mask_left = image_crop(args.data, frames, frames_left, yaw_aug_deg=args.yaw_aug_deg,
                                                   fx_px=fx_px, fx_px_left=fx_px_left, src_w=src_w)
    note = (f"車体マスク 右 {body_mask.mean() * 100:.1f}%・左 {body_mask_left.mean() * 100:.1f}%"
            f"  fx(左)={fx_px_left:.1f}px")
    print(f"# 向きの拡張: ±{args.yaw_aug_deg:g}°  fx={fx_px:.1f}px  左右の切り落とし {crop_x * 100:.1f}%  "
          f"{note}")

    spec = InputSpec(size=(w, h), crop_x=crop_x, fov_deg=args.fov_deg, max_range=args.max_range,
                     k=args.k, stride=args.stride, max_steer=vehicle.max_steer)
    records, _ = load_records(args.data, spec, margin_deg=margin_deg)
    train_r, val_r = split_records(records, args.val)
    if not train_r:
        print("学習に使える記録がありません", file=sys.stderr)
        return 2
    print("# 学習: " + " ".join(f"{r.name}({len(r)})" for r in train_r))
    print("# 検証: " + (" ".join(f"{r.name}({len(r)})" for r in val_r) or "なし"))

    train_ds = FusionDataset(train_r, args.data, spec, augment=True, flip=not args.no_flip,
                             yaw_max_deg=args.yaw_aug_deg, margin_deg=margin_deg,
                             fx_px=fx_px, body_mask=body_mask, photo_strength=args.photo_aug,
                             fx_px_left=fx_px_left, body_mask_left=body_mask_left)
    val_ds = FusionDataset(val_r, args.data, spec, margin_deg=margin_deg)
    xy_scale = train_ds.xy_scale
    weights = sample_weights(train_r, spec.max_steer, alpha=args.balance) if args.balance > 0 else None
    train_loader, val_loader = make_loaders(train_ds, val_ds, weights, args)

    device = pick_device()
    model = FusionModel(lidar_ch=spec.history + 1, lidar_n=spec.fov_deg + 1, k=spec.k, xy_scale=xy_scale,
                        pretrained=not args.no_pretrained)
    if args.init is not None:
        load_weights(model, args.init)
        print(f"# {args.init} から追加学習")
    model.to(device)
    print(f"# device: {device}  学習 {len(train_ds)}件 / 検証 {len(val_ds)}件  経路点 {spec.k}×{ds:g}m "
          f"（{xy_scale:g}m）  max_steer={spec.max_steer:.3f}rad")

    args.out.mkdir(parents=True, exist_ok=True)
    config = {
        "preproc_version": PREPROC_VERSION,
        "image_preproc_version": PREPROC_VERSION_STEREO,
        "cams": list(STEREO_CAMS),
        "input_size": [w, h], "crop_x": crop_x, "fov_deg": spec.fov_deg, "max_range": spec.max_range,
        "history": spec.history, "k": spec.k, "ds": ds, "xy_scale": xy_scale,
        "max_steer": spec.max_steer, "speed_ref": spec.speed_ref, "yaw_rate_ref": spec.yaw_rate_ref,
        "steer_link": list(vehicle.steer_link),
        "yaw_aug_deg": args.yaw_aug_deg, "photo_aug": args.photo_aug,
        "drop": [args.drop_image, args.drop_lidar, args.drop_state], "flip": not args.no_flip,
        "epochs": args.epochs, "seed": args.seed, "stride": args.stride,
        "train_records": [r.name for r in train_r], "val_records": [r.name for r in val_r],
        "maps": sorted({m for r in train_r for m in r.maps}),
    }

    def write_config(**extra) -> None:
        (args.out / "train_config.json").write_text(json.dumps({**config, **extra}, indent=2, ensure_ascii=False))

    write_config()
    fit(model, train_loader, val_loader, device, spec, xy_scale, ds, args, second="lidar", second_label="点群",
        drop_second=args.drop_lidar, write_config=write_config)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
