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
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # repo root

import torch  # noqa: E402
from torch.utils.data import DataLoader  # noqa: E402

from dataset import DrivableDataset, list_labeled_pairs  # noqa: E402
from model import DrivableSegModel  # noqa: E402
from raspi.core.cam_e2e_preproc import PREPROC_VERSION  # noqa: E402

__all__ = ["iou_score", "split_pairs", "split_pairs_blocks", "train_one_epoch", "evaluate",
           "pick_device"]


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


def _frame_block(frame: Path, block_s: float) -> str | None:
    """フレームが属する時間ブロックの名前。`extract_frames.py`の命名
    `<mcap名>_<cam>_<撮像ns>.jpg`から「元ファイル+カメラ」と時刻ブロックを作る。
    形式に合わない名前は`None`（呼び出し側がフレーム単位に倒す）。"""
    head, sep, tail = frame.stem.rpartition("_")
    if not sep or not tail.isdigit():
        return None
    block = int(tail) // max(1, int(block_s * 1_000_000_000))
    return f"{head}:{block}"


def split_pairs_blocks(pairs: list, val_ratio: float = 0.15, seed: int = 0,
                       block_s: float = 5.0) -> tuple[list, list]:
    """train/val に**時間ブロック単位**で分ける（`ml_cam_e2e/samples.py`と同じ考え方）。

    連続フレームはほぼ同じ絵なので、フレーム単位でシャッフルすると、train の隣のフレームが
    val に漏れて val_iou が過大になる。`block_s`秒のかたまり（元ファイル・カメラ別）を
    まるごと振り分ける。ブロックを`seed`で決まる順に並べ、val のフレーム数が
    `val_ratio`に届くまで val に回す（trainを1ブロックは残す）。ブロックが1つしか
    取れない（名前が`extract_frames.py`形式でない・短い録画）ときはフレーム単位
    （`split_pairs`）へ倒す。
    """
    blocks: dict[str, list] = {}
    for pair in pairs:
        key = _frame_block(Path(pair[0]), block_s)
        if key is None:
            return split_pairs(pairs, val_ratio, seed)
        blocks.setdefault(key, []).append(pair)
    if len(blocks) < 2:
        return split_pairs(pairs, val_ratio, seed)
    order = sorted(blocks, key=lambda k: zlib.crc32(f"{k}:{seed}".encode()))
    n_val_target = max(1, int(round(len(pairs) * val_ratio)))
    val_keys: list[str] = []
    n_val = 0
    for key in order[:-1]:                 # trainに最低1ブロックは残す
        if n_val >= n_val_target:
            break
        val_keys.append(key)
        n_val += len(blocks[key])
    chosen = set(val_keys)
    val = [pair for key in val_keys for pair in blocks[key]]
    train = [pair for key in order if key not in chosen for pair in blocks[key]]
    return train, val


def iou_score(pred: torch.Tensor, target: torch.Tensor, *, threshold: float = 0.5,
             eps: float = 1e-6) -> float:
    """走行可能領域（正例）の IoU。**画像ごとに計算してからバッチ内で平均**
    （macro-average）。バッチ全体を一括で`.sum()`するmicro-averageだと、
    バッチ内で走行可能領域が広い画像ほど支配的になり、画像単位の精度を
    正しく反映しない。**両方とも正例が無ければ「一致」扱い**（分母0を
    「不一致」として罰すると、走行不可しか写っていない検証画像が
    IoU を不当に下げる）。
    """
    p = (pred >= threshold).float()
    t = (target >= 0.5).float()
    dims = tuple(range(1, p.dim()))            # バッチ次元(0)以外全部で画像単位に集約
    inter = (p * t).sum(dim=dims)
    union = ((p + t) >= 1).float().sum(dim=dims)
    per_image = torch.where(union == 0, torch.ones_like(union), inter / (union + eps))
    return float(per_image.mean().item())


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
    ap.add_argument("--block-s", type=float, default=5.0,
                    help="train/valを分ける時間ブロックの長さ[s]。隣のフレームがvalに漏れない"
                         "ようにブロック単位で振り分ける")
    ap.add_argument("--no-pretrained", action="store_true",
                    help="ImageNet 事前学習重みを使わない（オフライン環境・動作確認向け）")
    ap.add_argument("--seed", type=int, default=0,
                    help="train/val分割と重み初期化・DataLoaderのシャッフル順を固定する")
    args = ap.parse_args()

    # `split_pairs`のseedはこれで既に固定されるが、モデルの重み初期化・
    # augmentation（`dataset.py`の`np.random.rand()`）・DataLoaderのシャッフル順は
    # 別系統の乱数源なので、ここで明示的に固定しないと再現性が取れない
    torch.manual_seed(args.seed)

    w, h = (int(v) for v in args.size.lower().split("x"))
    pairs = list_labeled_pairs(args.frames, args.masks)
    if len(pairs) < 4:
        print(f"ラベル付き画像が {len(pairs)} 枚しかありません。"
              f"先に `ml_cam/annotate.py` でラベル付けしてください", file=sys.stderr)
        return 2

    train_pairs, val_pairs = split_pairs_blocks(pairs, args.val_ratio, seed=args.seed,
                                                block_s=args.block_s)
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
        "input_size": [w, h], "preproc_version": PREPROC_VERSION,
        "epochs": args.epochs, "best_val_iou": best_iou,
        "n_train": len(train_pairs), "n_val": len(val_pairs),
    }, indent=2))
    print(f"# 完了。最良 val_iou={best_iou:.3f} → {args.out}/best.pt")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
