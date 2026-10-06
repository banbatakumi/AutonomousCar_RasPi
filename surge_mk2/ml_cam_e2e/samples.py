"""ml_cam_e2e/samples.py — 抽出済みペア（`manifest.csv`）の読み込みと、学習に使う/使わないの判断。

**torch を import しない純粋関数だけ**を置く。学習（`dataset.py`・`train.py`）、
評価（`eval_model.py`）、操作パネル（`app.py` の各タブ）が同じ判断を共有する
——「GUIで除外したのに学習には入っていた」「評価の val と学習の val が別物」
という食い違いを、定義を1箇所にすることで防ぐ。

## 何を学習に使わないか

1. **人が除外した区間**（`exclusions.json`。確認・選別タブで付ける）。
   コースアウト・やり直し・壁に当たって戻した区間など、手本にしたくない運転。
   **時刻の範囲で持つ**——行番号やファイル名で持つと、同じ記録を間引き率を
   変えて抽出し直したときに外れてしまう。
2. **後退**。1枚の絵から前進と区別できず、復帰操作であって手本ではない。
3. **静止したままの待機**（速度指令ほぼ0・実速度もほぼ0）。「止まった絵には
   速度0」を大量に教えると発進できないモデルになる。
   走行中にスロットルを抜いた瞬間は残す（「ここで減速」の手本）。

## 検証データは時間のかたまりで分ける

隣り合うフレームはほぼ同じ絵なので、1枚ずつ無作為に分けると検証データに
学習データの「隣の1枚」が入り、誤差が実力より小さく出る。記録ごとに
`block_s` 秒のかたまりを作り、かたまり単位で振り分ける。振り分けは
（記録名・かたまり番号・seed）のハッシュで決めるので、データを足しても
既存のかたまりの割り当ては動かない。
"""

from __future__ import annotations

import csv
import json
import math
import zlib
from dataclasses import asdict, dataclass
from pathlib import Path

__all__ = [
    "MANIFEST_COLUMNS", "EXCLUSIONS_FILE", "STOP_CMD_SPEED", "STOP_ACTUAL_SPEED",
    "STRAIGHT_NORM",
    "Sample", "Exclusion",
    "load_rgb", "load_manifest", "load_exclusions", "save_exclusions", "is_excluded",
    "drop_reason", "usable", "is_val", "split",
    "speed_label", "steer_norm", "speed_norm", "auto_speed_ref",
    "histogram", "balance_weights", "balanced_histogram", "per_record_stats",
]

MANIFEST_COLUMNS = ["file", "source_mcap", "cam", "t_capture_ns",
                    "target_steer", "target_speed", "speed_actual", "brake"]
EXCLUSIONS_FILE = "exclusions.json"

#: [m/s] 速度指令がこれ以下なら「スロットルを入れていない」
STOP_CMD_SPEED = 0.02
#: [m/s] 実速度がこれ未満なら「止まっている」。前輪エンコーダは粗いので 0 とは比べない
STOP_ACTUAL_SPEED = 0.05
#: 直進とみなす舵（正規化値）。記録ごとの直進率の集計に使う
STRAIGHT_NORM = 0.1

NS = 1_000_000_000


@dataclass(frozen=True)
class Sample:
    """`manifest.csv` の1行。"""

    path: Path
    source_mcap: str
    cam: str
    t_ns: int
    target_steer: float        #: [rad] 人の舵指令。左が正
    target_speed: float        #: [m/s] 人の速度指令。負は後退
    speed_actual: float        #: [m/s] そのときの実速度。記録に無ければ NaN
    brake: bool


@dataclass(frozen=True)
class Exclusion:
    """人が「手本にしない」と決めた区間（両端を含む）。"""

    source_mcap: str
    cam: str
    t_start_ns: int
    t_end_ns: int
    note: str = ""


def load_rgb(path: Path):
    """JPEG → RGB の (H, W, 3) uint8。学習・評価・操作パネルが同じ関数で読む。"""
    import numpy as np
    from PIL import Image

    with Image.open(path) as img:
        return np.asarray(img.convert("RGB"), dtype=np.uint8)


def _float(text: str | None) -> float:
    try:
        return float(text) if text not in (None, "") else math.nan
    except ValueError:
        return math.nan


def load_manifest(frames_dir: Path, *, require_file: bool = True) -> list[Sample]:
    """`manifest.csv` を読み、（記録・カメラ・時刻）順に並べて返す。

    同じ記録を2回抽出すると同じ行が2つ付く（manifest は追記式）ので、
    ファイル名で重複を落とす（後勝ち）。画像の無い行は捨てる。
    """
    manifest = frames_dir / "manifest.csv"
    if not manifest.exists():
        return []
    by_file: dict[str, Sample] = {}
    with open(manifest, newline="") as f:
        for row in csv.DictReader(f):
            path = frames_dir / row["file"]
            if require_file and not path.exists():
                continue
            by_file[row["file"]] = Sample(
                path=path, source_mcap=row["source_mcap"], cam=row["cam"],
                t_ns=int(row["t_capture_ns"]),
                target_steer=float(row["target_steer"]),
                target_speed=float(row["target_speed"]),
                speed_actual=_float(row.get("speed_actual")),
                brake=row.get("brake", "") in ("1", "True", "true"))
    return sorted(by_file.values(), key=lambda s: (s.source_mcap, s.cam, s.t_ns))


# ── 人が付けた除外区間 ──

def load_exclusions(frames_dir: Path) -> list[Exclusion]:
    path = frames_dir / EXCLUSIONS_FILE
    if not path.exists():
        return []
    return [Exclusion(**item) for item in json.loads(path.read_text(encoding="utf-8"))]


def save_exclusions(frames_dir: Path, exclusions: list[Exclusion]) -> None:
    ordered = sorted(exclusions, key=lambda e: (e.source_mcap, e.cam, e.t_start_ns))
    (frames_dir / EXCLUSIONS_FILE).write_text(
        json.dumps([asdict(e) for e in ordered], ensure_ascii=False, indent=2),
        encoding="utf-8")


def is_excluded(sample: Sample, exclusions: list[Exclusion]) -> bool:
    return any(e.source_mcap == sample.source_mcap and e.cam == sample.cam
               and e.t_start_ns <= sample.t_ns <= e.t_end_ns for e in exclusions)


# ── 機械的に落とすもの ──

def drop_reason(sample: Sample) -> str | None:
    """学習に使わない理由。使うなら `None`（モジュールdocstring参照）。"""
    if sample.target_speed < -STOP_CMD_SPEED:
        return "後退"
    if sample.brake or sample.target_speed <= STOP_CMD_SPEED:
        # 実速度が記録に無いときは指令だけで判断する（待機を混ぜるより安全側）
        if math.isnan(sample.speed_actual) or abs(sample.speed_actual) < STOP_ACTUAL_SPEED:
            return "静止"
    return None


def usable(sample: Sample, exclusions: list[Exclusion]) -> bool:
    return drop_reason(sample) is None and not is_excluded(sample, exclusions)


# ── 学習/検証の振り分け ──

def is_val(source_mcap: str, t_ns: int, *, block_s: float = 5.0,
           val_ratio: float = 0.15, seed: int = 0) -> bool:
    block = t_ns // max(1, int(block_s * NS))
    h = zlib.crc32(f"{source_mcap}:{block}:{seed}".encode())
    return (h % 10_000) < val_ratio * 10_000


def split(samples: list[Sample], *, block_s: float = 5.0, val_ratio: float = 0.15,
          seed: int = 0) -> tuple[list[Sample], list[Sample]]:
    train: list[Sample] = []
    val: list[Sample] = []
    for s in samples:
        (val if is_val(s.source_mcap, s.t_ns, block_s=block_s, val_ratio=val_ratio,
                       seed=seed) else train).append(s)
    return train, val


# ── ラベル ──

def speed_label(sample: Sample) -> float:
    """[m/s] 速度の手本。ブレーキ中は 0（「ここで止める」）。"""
    return 0.0 if sample.brake else max(0.0, sample.target_speed)


def steer_norm(sample: Sample, max_steer: float) -> float:
    if max_steer <= 0:
        return 0.0
    return max(-1.0, min(1.0, sample.target_steer / max_steer))


def speed_norm(sample: Sample, speed_ref: float) -> float:
    if speed_ref <= 0:
        return 0.0
    return max(0.0, min(1.0, speed_label(sample) / speed_ref))


def auto_speed_ref(samples: list[Sample], floor: float = 0.1) -> float:
    """[m/s] `speed_norm=1` が表す速度。学習に使うサンプルの最高速度にする。

    固定値（車両の上限 3m/s など）にすると、ゆっくり走った手本では出力が
    0 付近に固まって分解能を捨てる。データの最高速度に合わせれば 0..1 を
    使い切れる。手本より速く走らせたいときは planner の `speed_scale` を使う。
    """
    return max([floor] + [speed_label(s) for s in samples])


# ── 偏りの集計 ──

def histogram(values: list[float], lo: float, hi: float, n_bins: int) -> list[int]:
    counts = [0] * n_bins
    span = hi - lo
    for v in values:
        if math.isnan(v) or span <= 0:
            continue
        i = int((v - lo) / span * n_bins)
        counts[max(0, min(n_bins - 1, i))] += 1
    return counts


def balance_weights(steer_norms: list[float], *, n_bins: int = 15,
                    alpha: float = 0.5) -> list[float]:
    """サンプルごとの抽選の重み。舵の区間に属する枚数 `n` に対して `1 / n**alpha`。

    直線が長いコースでは舵0付近が大半を占め、そのまま学習すると「常に直進」が
    誤差を一番小さくする解になる。`alpha=0` は重み付けなし、`alpha=1` は
    全区間が同じ確率で選ばれる（少数のカーブを繰り返し見るので過学習しやすい）。
    """
    counts = histogram(steer_norms, -1.0, 1.0, n_bins)
    weights = []
    for v in steer_norms:
        i = max(0, min(n_bins - 1, int((v + 1.0) / 2.0 * n_bins)))
        weights.append(1.0 / (counts[i] ** alpha) if counts[i] else 0.0)
    return weights


def balanced_histogram(counts: list[int], alpha: float) -> list[float]:
    """重み付け後に各区間から選ばれる期待枚数（合計は元の枚数と同じ）。"""
    total = sum(counts)
    mass = [c ** (1.0 - alpha) if c else 0.0 for c in counts]
    norm = sum(mass)
    return [m * total / norm if norm else 0.0 for m in mass]


def per_record_stats(samples: list[Sample], exclusions: list[Exclusion],
                     max_steer: float) -> list[dict]:
    """記録（`source_mcap`）ごとの内訳。偏りタブの表に使う。"""
    order: list[str] = []
    groups: dict[str, list[Sample]] = {}
    for s in samples:
        if s.source_mcap not in groups:
            groups[s.source_mcap] = []
            order.append(s.source_mcap)
        groups[s.source_mcap].append(s)
    rows = []
    for name in order:
        g = groups[name]
        excluded = [s for s in g if is_excluded(s, exclusions)]
        dropped = [s for s in g if not is_excluded(s, exclusions) and drop_reason(s)]
        used = [s for s in g if usable(s, exclusions)]
        norms = [steer_norm(s, max_steer) for s in used]
        left = sum(1 for v in norms if v > STRAIGHT_NORM)
        right = sum(1 for v in norms if v < -STRAIGHT_NORM)
        times = [s.t_ns for s in g]
        rows.append({
            "source_mcap": name, "total": len(g), "excluded": len(excluded),
            "dropped": len(dropped), "used": len(used),
            "duration_s": (max(times) - min(times)) / NS if times else 0.0,
            "straight_ratio": (len(used) - left - right) / len(used) if used else 0.0,
            "left": left, "right": right,
        })
    return rows
