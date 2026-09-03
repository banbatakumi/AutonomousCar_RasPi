"""ml_cam_e2e/train.py — カメラE2E（操舵の直接回帰）の学習ループ。

    python3 ml_cam_e2e/train.py --frames ml_cam_e2e/runs/v1/frames --epochs 30 \\
        --out ml_cam_e2e/runs/v1

`ml_cam_e2e/dataset.list_pairs()` で `manifest.csv` の全ペアを集め、train/val に
分けて `DriveRegressionModel` を学習する。毎エポック検証MAE（操舵角の平均絶対誤差、
正規化値と実角度の両方）を表示する。`ml_cam/train.py` と対称の構成。
"""

from __future__ import annotations

import argparse
import json
import random
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # repo root

import torch  # noqa: E402
from torch.utils.data import DataLoader  # noqa: E402

from dataset import DriveDataset, list_pairs  # noqa: E402
from model import DriveRegressionModel  # noqa: E402
from raspi.core.vehicle import Vehicle  # noqa: E402

__all__ = ["split_pairs", "mae_score", "train_one_epoch", "evaluate", "pick_device"]


def split_pairs(pairs: list, val_ratio: float = 0.15, seed: int = 0) -> tuple[list, list]:
    """train/val に分ける。**シャッフルしてから切る**（`ml_cam/train.py` と同じ理由——
    走行順のまま先頭/末尾で切ると val がその周回だけに偏る）。"""
    rng = random.Random(seed)
    shuffled = pairs[:]
    rng.shuffle(shuffled)
    if len(shuffled) < 2:
        return shuffled, []
    n_val = max(1, int(round(len(shuffled) * val_ratio)))
    return shuffled[n_val:], shuffled[:n_val]


def mae_score(pred: torch.Tensor, target: torch.Tensor) -> float:
    """正規化値（-1..1）での平均絶対誤差。"""
    return float((pred - target).abs().mean().item())


def train_one_epoch(model, loader, optimizer, device) -> float:
    model.train()
    total_loss = 0.0
    n = 0
    loss_fn = torch.nn.MSELoss()
    for img, steer in loader:
        img, steer = img.to(device), steer.to(device)
        optimizer.zero_grad()
        out = model(img)
        loss = loss_fn(out, steer)
        loss.backward()
        optimizer.step()
        total_loss += loss.item() * img.size(0)
        n += img.size(0)
    return total_loss / max(n, 1)


@torch.no_grad()
def evaluate(model, loader, device) -> float:
    model.eval()
    maes = []
    ns = []
    for img, steer in loader:
        img, steer = img.to(device), steer.to(device)
        out = model(img)
        maes.append(mae_score(out, steer))
        ns.append(img.size(0))
    return sum(m * n for m, n in zip(maes, ns)) / sum(ns) if ns else 0.0


def pick_device() -> "torch.device":
    if torch.backends.mps.is_available():
        return torch.device("mps")
    if torch.cuda.is_available():
        return torch.device("cuda")
    return torch.device("cpu")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", type=Path, default=Path("ml_cam_e2e/data/frames"))
    ap.add_argument("--out", type=Path, default=Path("ml_cam_e2e/runs/latest"))
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--batch-size", type=int, default=16)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--size", default="224x224")
    ap.add_argument("--val-ratio", type=float, default=0.15)
    ap.add_argument("--no-pretrained", action="store_true",
                    help="ImageNet 事前学習重みを使わない（オフライン環境・動作確認向け）")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    torch.manual_seed(args.seed)

    w, h = (int(v) for v in args.size.lower().split("x"))
    pairs = list_pairs(args.frames)
    if len(pairs) < 4:
        print(f"ペアが {len(pairs)} 件しかありません。"
              f"先に `ml_cam_e2e/extract_pairs.py` でデータを作ってください", file=sys.stderr)
        return 2

    max_steer = Vehicle.load().max_steer
    train_pairs, val_pairs = split_pairs(pairs, args.val_ratio, seed=args.seed)
    train_ds = DriveDataset(train_pairs, size=(w, h), augment=True, max_steer=max_steer)
    val_ds = DriveDataset(val_pairs, size=(w, h), augment=False, max_steer=max_steer)
    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=max(1, args.batch_size), shuffle=False)

    device = pick_device()
    print(f"# device: {device}  学習 {len(train_pairs)}件 / 検証 {len(val_pairs)}件  "
          f"max_steer={max_steer:.3f}rad")

    model = DriveRegressionModel(pretrained=not args.no_pretrained).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)

    args.out.mkdir(parents=True, exist_ok=True)
    best_mae = float("inf")
    t0 = time.time()
    for epoch in range(1, args.epochs + 1):
        loss = train_one_epoch(model, train_loader, optimizer, device)
        mae = evaluate(model, val_loader, device) if val_pairs else float("nan")
        mae_deg = mae * max_steer * 180.0 / 3.141592653589793 if val_pairs else float("nan")
        print(f"epoch {epoch:3d}/{args.epochs}  loss={loss:.4f}  val_mae={mae:.4f}"
              f"  ({mae_deg:.1f}deg, {time.time() - t0:.0f}s)")
        if val_pairs and mae < best_mae:
            best_mae = mae
            torch.save(model.state_dict(), args.out / "best.pt")

    if not val_pairs:
        torch.save(model.state_dict(), args.out / "best.pt")
        best_mae = float("nan")
    torch.save(model.state_dict(), args.out / "last.pt")
    (args.out / "train_config.json").write_text(json.dumps({
        "input_size": [w, h], "epochs": args.epochs, "best_val_mae": best_mae,
        "max_steer": max_steer, "n_train": len(train_pairs), "n_val": len(val_pairs),
    }, indent=2))
    print(f"# 完了。最良 val_mae={best_mae:.4f} → {args.out}/best.pt")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
