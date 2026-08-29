"""ml_cam/train.py — 走行可否セグメンテーションの学習ループ。

    python3 ml_cam/train.py --frames ml_cam/data/frames --epochs 30 --out ml_cam/runs/latest

`ml_cam/dataset.list_labeled_pairs()` でラベル付け済みの (フレーム, マスク) を
集め、train/val に分けて `DrivableSegModel` を学習する。毎エポック IoU
（走行可能領域の一致率）を表示するので、過学習していないか（train損失は
下がるのに val_iou が下がりだしていないか）を見ながら回せばよい——
ハイパーパラメータは既定値のままで大きく外すことは無い想定。
"""

from __future__ import annotations

import argparse
import json
import random
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import torch  # noqa: E402
from torch.utils.data import DataLoader  # noqa: E402

from dataset import DrivableDataset, list_labeled_pairs  # noqa: E402
from model import DrivableSegModel  # noqa: E402

__all__ = ["iou_score", "split_pairs", "train_one_epoch", "evaluate", "pick_device"]


def split_pairs(pairs: list, val_ratio: float = 0.15,
                seed: int = 0) -> tuple[list, list]:
    """train/val に分ける。**シャッフルしてから切る。**

    `extract_frames.py` は走行順にフレームを並べるので、シャッフルせずに
    先頭/末尾で切ると val が「その周回だけ」に偏り、精度を過小/過大評価する。
    """
    rng = random.Random(seed)
    shuffled = pairs[:]
    rng.shuffle(shuffled)
    if len(shuffled) < 2:
        return shuffled, []
    n_val = max(1, int(round(len(shuffled) * val_ratio)))
    return shuffled[n_val:], shuffled[:n_val]


def iou_score(pred: torch.Tensor, target: torch.Tensor, *, threshold: float = 0.5,
             eps: float = 1e-6) -> float:
    """走行可能領域（正例）の IoU。**両方とも正例が無ければ「一致」扱い**
    （分母0を「不一致」として罰すると、走行不可しか写っていない検証画像が
    IoU を不当に下げる）。
    """
    p = (pred >= threshold).float()
    t = (target >= 0.5).float()
    inter = (p * t).sum().item()
    union = ((p + t) >= 1).float().sum().item()
    if union == 0:
        return 1.0
    return inter / (union + eps)


def train_one_epoch(model, loader, optimizer, device) -> float:
    model.train()
    total_loss = 0.0
    n = 0
    loss_fn = torch.nn.BCELoss()
    for img, msk in loader:
        img, msk = img.to(device), msk.to(device)
        optimizer.zero_grad()
        out = model(img)
        loss = loss_fn(out, msk)
        loss.backward()
        optimizer.step()
        total_loss += loss.item() * img.size(0)
        n += img.size(0)
    return total_loss / max(n, 1)


@torch.no_grad()
def evaluate(model, loader, device) -> float:
    model.eval()
    ious = []
    for img, msk in loader:
        img, msk = img.to(device), msk.to(device)
        out = model(img)
        ious.append(iou_score(out, msk))
    return sum(ious) / len(ious) if ious else 0.0


def pick_device() -> "torch.device":
    if torch.backends.mps.is_available():
        return torch.device("mps")
    if torch.cuda.is_available():
        return torch.device("cuda")
    return torch.device("cpu")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", type=Path, default=Path("ml_cam/data/frames"))
    ap.add_argument("--masks", type=Path, default=None,
                    help="マスクの場所（既定は --frames と同じ。ml_cam/annotate.py の既定出力先）")
    ap.add_argument("--out", type=Path, default=Path("ml_cam/runs/latest"))
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--batch-size", type=int, default=8)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--size", default="224x224")
    ap.add_argument("--val-ratio", type=float, default=0.15)
    ap.add_argument("--no-pretrained", action="store_true",
                    help="ImageNet 事前学習重みを使わない（オフライン環境・動作確認向け）")
    args = ap.parse_args()

    w, h = (int(v) for v in args.size.lower().split("x"))
    pairs = list_labeled_pairs(args.frames, args.masks)
    if len(pairs) < 4:
        print(f"ラベル付き画像が {len(pairs)} 枚しかありません。"
              f"先に `ml_cam/annotate.py` でラベル付けしてください", file=sys.stderr)
        return 2

    train_pairs, val_pairs = split_pairs(pairs, args.val_ratio)
    train_ds = DrivableDataset(train_pairs, size=(w, h), augment=True)
    val_ds = DrivableDataset(val_pairs, size=(w, h), augment=False)
    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=max(1, args.batch_size), shuffle=False)

    device = pick_device()
    print(f"# device: {device}  学習 {len(train_pairs)}枚 / 検証 {len(val_pairs)}枚")

    model = DrivableSegModel(pretrained=not args.no_pretrained).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)

    args.out.mkdir(parents=True, exist_ok=True)
    best_iou = -1.0
    t0 = time.time()
    for epoch in range(1, args.epochs + 1):
        loss = train_one_epoch(model, train_loader, optimizer, device)
        iou = evaluate(model, val_loader, device) if val_pairs else float("nan")
        print(f"epoch {epoch:3d}/{args.epochs}  loss={loss:.4f}  val_iou={iou:.3f}"
              f"  ({time.time() - t0:.0f}s)")
        if val_pairs and iou > best_iou:
            best_iou = iou
            torch.save(model.state_dict(), args.out / "best.pt")

    # 検証データが無い（データがごく少ない）ときは最終エポックを best として残す
    if not val_pairs:
        torch.save(model.state_dict(), args.out / "best.pt")
        best_iou = float("nan")
    torch.save(model.state_dict(), args.out / "last.pt")
    (args.out / "train_config.json").write_text(json.dumps({
        "input_size": [w, h], "epochs": args.epochs, "best_val_iou": best_iou,
        "n_train": len(train_pairs), "n_val": len(val_pairs),
    }, indent=2))
    print(f"# 完了。最良 val_iou={best_iou:.3f} → {args.out}/best.pt")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
