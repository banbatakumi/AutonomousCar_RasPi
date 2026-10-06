"""ml_cam_e2e/train.py — カメラE2E（操舵と速度の直接回帰）の学習ループ。

    python3 ml_cam_e2e/train.py --frames ml_cam_e2e/runs/v1/frames --epochs 30 \\
        --out ml_cam_e2e/runs/v1

`samples.py` が決めた「学習に使うペア」を時間のかたまり単位で train/val に分け、
`DriveRegressionModel` を学習する。毎エポック、検証データでの操舵と速度の
平均絶対誤差を表示する。`ml_cam/train.py` と対称の構成。

## 直進過多への対処は「重み付きの抽選」

舵0付近が大半を占めるデータをそのまま回すと、常に直進と答えるのが誤差を
最小にする。`--balance`（0〜1）で舵の区間ごとに抽選の重みを変え、カーブを
多めに見せる（`samples.balance_weights`）。枚数を捨てる間引きはしない。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # repo root

import torch  # noqa: E402
from torch.utils.data import DataLoader, WeightedRandomSampler  # noqa: E402

import samples as S  # noqa: E402
from dataset import DriveDataset  # noqa: E402
from model import DriveRegressionModel  # noqa: E402
from raspi.core.cam_e2e_preproc import PREPROC_VERSION  # noqa: E402
from raspi.core.vehicle import Vehicle  # noqa: E402

__all__ = ["select_samples", "loss_fn", "train_one_epoch", "evaluate", "pick_device"]

#: SmoothL1 の切り替え点（正規化値）。これより大きい誤差は L1 として効くので、
#: 手本の中の外れた操作（一瞬の切りすぎ等）に引きずられにくい
_HUBER_BETA = 0.1


def select_samples(frames_dir: Path) -> tuple[list[S.Sample], Counter]:
    """学習に使うサンプルと、使わなかった理由の内訳を返す。"""
    exclusions = S.load_exclusions(frames_dir)
    used: list[S.Sample] = []
    dropped: Counter = Counter()
    for s in S.load_manifest(frames_dir):
        if S.is_excluded(s, exclusions):
            dropped["除外区間"] += 1
        elif (reason := S.drop_reason(s)) is not None:
            dropped[reason] += 1
        else:
            used.append(s)
    return used, dropped


def loss_fn(out: torch.Tensor, target: torch.Tensor, speed_weight: float) -> torch.Tensor:
    huber = torch.nn.functional.smooth_l1_loss
    return (huber(out[:, 0], target[:, 0], beta=_HUBER_BETA)
            + speed_weight * huber(out[:, 1], target[:, 1], beta=_HUBER_BETA))


def train_one_epoch(model, loader, optimizer, device, speed_weight: float) -> float:
    model.train()
    total_loss = 0.0
    n = 0
    for img, target in loader:
        img, target = img.to(device), target.to(device)
        optimizer.zero_grad()
        loss = loss_fn(model(img), target, speed_weight)
        loss.backward()
        optimizer.step()
        total_loss += loss.item() * img.size(0)
        n += img.size(0)
    return total_loss / max(n, 1)


@torch.no_grad()
def evaluate(model, loader, device) -> tuple[float, float]:
    """検証データでの `(操舵のMAE, 速度のMAE)`。どちらも正規化値。"""
    model.eval()
    err = torch.zeros(2)
    n = 0
    for img, target in loader:
        out = model(img.to(device)).cpu()
        err += (out - target).abs().sum(dim=0)
        n += img.size(0)
    if n == 0:
        return math.nan, math.nan
    return float(err[0] / n), float(err[1] / n)


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
    ap.add_argument("--size", default="224x128",
                    help="入力解像度 幅x高さ。前カメラ（640x360）と同じ 16:9 が既定")
    ap.add_argument("--val-ratio", type=float, default=0.15)
    ap.add_argument("--block-s", type=float, default=5.0,
                    help="検証データを切り出す時間のかたまり[s]（samples.py 参照）")
    ap.add_argument("--balance", type=float, default=0.5,
                    help="直進過多の補正 0〜1。0=補正なし、1=舵の全区間を同じ確率で選ぶ")
    ap.add_argument("--speed-weight", type=float, default=0.5,
                    help="損失での速度の重み（操舵=1 に対して）")
    ap.add_argument("--speed-ref", type=float, default=0.0,
                    help="速度の正規化基準[m/s]。0 なら学習データの最高速度")
    ap.add_argument("--no-flip", action="store_true",
                    help="左右反転の拡張を使わない（片側通行など左右対称でないコース向け）")
    ap.add_argument("--no-pretrained", action="store_true",
                    help="ImageNet 事前学習重みを使わない（オフライン環境・動作確認向け）")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    torch.manual_seed(args.seed)

    w, h = (int(v) for v in args.size.lower().split("x"))
    used, dropped = select_samples(args.frames)
    if dropped:
        print("# 学習に使わないペア: "
              + "・".join(f"{k} {n}件" for k, n in dropped.most_common()))
    if len(used) < 4:
        print(f"学習に使えるペアが {len(used)} 件しかありません。"
              f"先に `ml_cam_e2e/extract_pairs.py` でデータを作ってください", file=sys.stderr)
        return 2

    max_steer = Vehicle.load().max_steer
    speed_ref = args.speed_ref if args.speed_ref > 0 else S.auto_speed_ref(used)
    train_s, val_s = S.split(used, block_s=args.block_s, val_ratio=args.val_ratio,
                             seed=args.seed)
    if not train_s:
        print("学習側に1件も振り分けられませんでした（データが短すぎます）", file=sys.stderr)
        return 2
    if not val_s:
        print("# 警告: 検証データが0件です（記録が短い）。val_mae は nan になり、"
              "最後のエポックを best.pt にします", file=sys.stderr)

    common = dict(size=(w, h), max_steer=max_steer, speed_ref=speed_ref)
    train_ds = DriveDataset(train_s, augment=True, flip=not args.no_flip, **common)
    val_ds = DriveDataset(val_s, augment=False, **common)
    if args.balance > 0:
        weights = S.balance_weights([S.steer_norm(s, max_steer) for s in train_s],
                                    alpha=args.balance)
        sampler = WeightedRandomSampler(weights, num_samples=len(train_s), replacement=True)
        train_loader = DataLoader(train_ds, batch_size=args.batch_size, sampler=sampler)
    else:
        train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=max(1, args.batch_size), shuffle=False)

    device = pick_device()
    print(f"# device: {device}  学習 {len(train_s)}件 / 検証 {len(val_s)}件  "
          f"max_steer={max_steer:.3f}rad  speed_ref={speed_ref:.2f}m/s")

    model = DriveRegressionModel(pretrained=not args.no_pretrained).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)

    args.out.mkdir(parents=True, exist_ok=True)
    config = {
        "input_size": [w, h], "preproc_version": PREPROC_VERSION,
        "max_steer": max_steer, "speed_ref": speed_ref,
        "block_s": args.block_s, "val_ratio": args.val_ratio, "seed": args.seed,
        "balance": args.balance, "speed_weight": args.speed_weight,
        "flip": not args.no_flip, "epochs": args.epochs,
        "n_train": len(train_s), "n_val": len(val_s),
    }

    def write_config(**extra) -> None:
        (args.out / "train_config.json").write_text(json.dumps({**config, **extra}, indent=2))

    # **学習の前に書く。** 途中で止めても best.pt は残るので、そのとき
    # エクスポート（解像度と正規化基準をここから読む）ができるようにしておく
    write_config()

    best_score = math.inf
    best = (math.nan, math.nan)
    t0 = time.time()
    for epoch in range(1, args.epochs + 1):
        loss = train_one_epoch(model, train_loader, optimizer, device, args.speed_weight)
        mae, speed_mae = evaluate(model, val_loader, device)
        # `ml_common/epoch_log.py` は `loss=` の直後の `val_mae=` を読む。
        # 項目を足すときは後ろに足す
        print(f"epoch {epoch:3d}/{args.epochs}  loss={loss:.4f}  val_mae={mae:.4f}"
              f"  val_speed_mae={speed_mae:.4f}"
              f"  ({math.degrees(mae * max_steer):.1f}deg, {speed_mae * speed_ref:.2f}m/s,"
              f" {time.time() - t0:.0f}s)", flush=True)
        score = mae + args.speed_weight * speed_mae
        if val_s and score < best_score:
            best_score = score
            best = (mae, speed_mae)
            torch.save(model.state_dict(), args.out / "best.pt")

    if not val_s:
        torch.save(model.state_dict(), args.out / "best.pt")
    torch.save(model.state_dict(), args.out / "last.pt")
    write_config(best_val_mae=best[0], best_val_speed_mae=best[1])
    print(f"# 完了。最良 val_mae={best[0]:.4f}  val_speed_mae={best[1]:.4f} → {args.out}/best.pt")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
