"""矢印信号の YOLO 学習データの置き場・ラベルの読み書き・分割（Tkinter も ultralytics も知らない純粋な部品）。

    ml_arrow_yolo/runs/<モデル名>/images/<記録名>__<時刻ns>.jpg   … ①の出力（車の前方カメラの JPEG）
    ml_arrow_yolo/runs/<モデル名>/labels/<同名>.txt               … ②で確定したラベル（YOLO 形式）
    ml_arrow_yolo/runs/<モデル名>/drafts/<同名>.txt               … 前のモデルで付けた下書き（②で確定するまで学習に使わない）
    ml_arrow_yolo/runs/<モデル名>/classes.txt                     … クラス名（1行1つ。行番号がクラス番号）

## ラベルの約束

- 1行 `cls cx cy w h`（画像に対する 0〜1 の割合）。ultralytics の形式そのもの
- **ラベルのファイルがある＝確定済み。空のファイルは「看板が写っていない」と確定した背景画像。**
  ファイルが無い画像は未確定で、学習に使わない（見ていない画像を背景として教えない）
- 画像の名前の `__` より前が記録名。学習と検証は**記録ごと**に分ける（隣り合うほぼ同じコマが両方に
  入ると、検証の値が良く見えすぎる）
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

__all__ = [
    "DEFAULT_CLASSES", "Box", "RunPaths",
    "read_classes", "write_classes", "read_labels", "write_labels", "parse_label_text",
    "format_label_text", "source_of", "list_images", "label_stats", "split_by_source",
    "write_dataset_yaml",
]

#: 既定のクラス。`raspi/core/arrow_yolo.py` が方向として読む名前（`left`/`right`/`straight`）。
#: 濃い色の表示などを「非該当」として教えたいなら、別の名前のクラスを足す（`left_dark` 等）
DEFAULT_CLASSES = ["left", "right", "straight"]

#: 画像の名前の区切り（記録名 `__` 時刻）
SOURCE_SEP = "__"


@dataclass
class Box:
    """1つの枠。座標は画像に対する 0〜1 の割合（左上 x0,y0・右下 x1,y1）。"""

    cls: int
    x0: float
    y0: float
    x1: float
    y1: float

    def normalized(self) -> "Box":
        """角の順序を整え、0〜1 に収めた枠。"""
        x0, x1 = sorted((self.x0, self.x1))
        y0, y1 = sorted((self.y0, self.y1))
        c = lambda v: min(1.0, max(0.0, v))  # noqa: E731
        return Box(self.cls, c(x0), c(y0), c(x1), c(y1))

    @property
    def area(self) -> float:
        return max(0.0, self.x1 - self.x0) * max(0.0, self.y1 - self.y0)


class RunPaths:
    """モデル名1つぶんの置き場。"""

    def __init__(self, run_dir: Path) -> None:
        self.run_dir = Path(run_dir)
        self.images = self.run_dir / "images"
        self.labels = self.run_dir / "labels"
        self.drafts = self.run_dir / "drafts"
        self.classes = self.run_dir / "classes.txt"
        self.best = self.run_dir / "best.pt"

    def label_path(self, image: Path) -> Path:
        return self.labels / f"{image.stem}.txt"

    def draft_path(self, image: Path) -> Path:
        return self.drafts / f"{image.stem}.txt"


def read_classes(paths: RunPaths) -> list[str]:
    if paths.classes.is_file():
        names = [ln.strip() for ln in paths.classes.read_text(encoding="utf-8").splitlines()]
        names = [n for n in names if n]
        if names:
            return names
    return list(DEFAULT_CLASSES)


def write_classes(paths: RunPaths, names: list[str]) -> None:
    paths.run_dir.mkdir(parents=True, exist_ok=True)
    paths.classes.write_text("\n".join(names) + "\n", encoding="utf-8")


def parse_label_text(text: str) -> list[Box]:
    """YOLO 形式の本文 → 枠の一覧。壊れた行は飛ばす。"""
    boxes = []
    for line in text.splitlines():
        parts = line.split()
        if len(parts) != 5:
            continue
        try:
            c = int(parts[0])
            cx, cy, w, h = (float(v) for v in parts[1:])
        except ValueError:
            continue
        boxes.append(Box(c, cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2).normalized())
    return boxes


def format_label_text(boxes: list[Box]) -> str:
    lines = []
    for b in boxes:
        b = b.normalized()
        w, h = b.x1 - b.x0, b.y1 - b.y0
        if w <= 0 or h <= 0:
            continue
        lines.append(f"{b.cls} {(b.x0 + b.x1) / 2:.6f} {(b.y0 + b.y1) / 2:.6f} {w:.6f} {h:.6f}")
    return "\n".join(lines) + ("\n" if lines else "")


def read_labels(path: Path) -> list[Box] | None:
    """ラベルのファイル → 枠の一覧。ファイルが無ければ `None`（未確定）。空なら `[]`（背景）。"""
    if not path.is_file():
        return None
    return parse_label_text(path.read_text(encoding="utf-8"))


def write_labels(path: Path, boxes: list[Box]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(format_label_text(boxes), encoding="utf-8")
    tmp.replace(path)


def source_of(image: Path) -> str:
    """画像の名前 → 記録名（`<記録名>__<時刻>.jpg`）。区切りが無ければ名前全体。"""
    stem = image.stem
    return stem.split(SOURCE_SEP)[0] if SOURCE_SEP in stem else stem


def list_images(paths: RunPaths) -> list[Path]:
    """画像の一覧（記録名・時刻の順）。"""
    if not paths.images.is_dir():
        return []
    return sorted(p for p in paths.images.iterdir() if p.suffix.lower() in (".jpg", ".jpeg", ".png"))


def label_stats(paths: RunPaths, n_classes: int) -> dict:
    """`{"images", "labeled", "background", "drafts", "per_class": [...]}`（②・③タブの表示用）。"""
    imgs = list_images(paths)
    per_class = [0] * n_classes
    labeled = background = drafts = 0
    for img in imgs:
        boxes = read_labels(paths.label_path(img))
        if boxes is None:
            if paths.draft_path(img).is_file():
                drafts += 1
            continue
        labeled += 1
        if not boxes:
            background += 1
        for b in boxes:
            if 0 <= b.cls < n_classes:
                per_class[b.cls] += 1
    return {"images": len(imgs), "labeled": labeled, "background": background,
            "drafts": drafts, "per_class": per_class}


def split_by_source(images: list[Path], val_frac: float) -> tuple[list[Path], list[Path]]:
    """確定済みの画像 → `(学習, 検証)`。記録が2つ以上なら**記録ごと**に検証へ回す。

    記録が1つだけなら、時刻の後ろ `val_frac` を検証にする（隣り合うコマが両方に入るのを
    境目の1か所だけにする）。`val_frac` が 0 なら全部学習で、検証は学習と同じもの（ultralytics は検証が要る）。
    """
    images = sorted(images)
    if not images:
        return [], []
    if val_frac <= 0:
        return images, images
    groups: dict[str, list[Path]] = {}
    for p in images:
        groups.setdefault(source_of(p), []).append(p)
    target = max(1, round(len(images) * val_frac))
    if len(groups) >= 2:
        val: list[Path] = []
        # 名前の後ろ（新しい記録）から検証に回す。全部を検証にはしない
        for name in sorted(groups, reverse=True)[:-1]:
            if len(val) >= target:
                break
            val += groups[name]
        val_set = set(val)
        return [p for p in images if p not in val_set], sorted(val)
    n_train = max(1, len(images) - target)
    return images[:n_train], images[n_train:] or images[-1:]


def write_dataset_yaml(paths: RunPaths, train: list[Path], val: list[Path], names: list[str]) -> Path:
    """ultralytics の `data=` に渡す yaml と、画像の一覧（`train.txt`・`val.txt`）を書く。

    ultralytics は画像のパスの `/images/` を `/labels/` に置き換えてラベルを探すので、
    置き場（`images/`・`labels/`）はそのまま使える。
    """
    out = paths.run_dir / "dataset"
    out.mkdir(parents=True, exist_ok=True)
    (out / "train.txt").write_text("\n".join(str(p.resolve()) for p in train) + "\n", encoding="utf-8")
    (out / "val.txt").write_text("\n".join(str(p.resolve()) for p in val) + "\n", encoding="utf-8")
    lines = [f"path: {out.resolve()}", "train: train.txt", "val: val.txt", "names:"]
    lines += [f"  {i}: {n}" for i, n in enumerate(names)]
    yaml = out / "data.yaml"
    yaml.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return yaml
