"""学習スクリプトの共通部品（`ml_cam`・`ml_cam_e2e`・`ml_e2e`・`ml_stereo_e2e` の `train.py`）。

各 `train.py` が `pick_device`・乱数の固定・best の選び方・train/val の分け方をそれぞれ複製しており、
すでに不揃いになっていた（装置の優先順・best の初期値の流儀・時間ブロックの分け方が2通り）。
片方に入れた修正がもう片方に入らないので、ここに1つだけ置く。

**torch・numpy は関数の中で import する。** `group_split` は学習 GUI（このプロセスは torch を読まない）からも
`ml_cam_e2e/samples.py` 経由で使うため。
"""

from __future__ import annotations

import math
import random
import zlib
from typing import Callable, Hashable, Iterable, TypeVar

__all__ = ["pick_device", "seed_all", "make_optimizer", "train_epoch", "BestTracker", "time_block",
           "is_val_group", "group_split"]

T = TypeVar("T")

#: `is_val_group` の割合の分解能（0.01% 刻み）
_HASH_BINS = 10_000


def pick_device():
    """CUDA → MPS → CPU の順に使えるものを返す。

    以前は `ml_cam`・`ml_cam_e2e` が MPS を先に、`ml_e2e` が CUDA を先に見ていた。両方が使える計算機は
    無い（MPS は Mac だけ、CUDA は Mac に無い）ので、どちらの順でも選ばれる装置は同じ。
    """
    import torch

    if torch.cuda.is_available():
        return torch.device("cuda")
    if torch.backends.mps.is_available():
        return torch.device("mps")
    return torch.device("cpu")


def seed_all(seed: int) -> None:
    """`random`・numpy・torch の乱数をまとめて固定する。

    分割（`group_split`）は乱数を使わないが、重みの初期化・水増し（`np.random`・`random` を使うものがある）・
    DataLoader のシャッフル順はそれぞれ別系統の乱数源なので、どれかを固定し忘れると再現性が取れない
    （`ml_cam` は torch だけを固定しており、`dataset.py` の `np.random` の水増しが毎回変わっていた）。
    DataLoader の worker は torch が worker ごとに別の種を配る。
    """
    import numpy as np
    import torch

    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)


def make_optimizer(params, lr: float, weight_decay: float = 0.0):
    """最適化器を AdamW に揃える。

    `ml_cam`・`ml_cam_e2e` は Adam、`ml_e2e`・`ml_stereo_e2e` は AdamW（`weight_decay=1e-4`）だった。
    **AdamW は `weight_decay=0` なら Adam と同じ更新式**（減衰の項が消え、torch の実装も同じ経路を通る）なので、
    `ml_cam`・`ml_cam_e2e` は `weight_decay=0` を渡して学習結果を変えずに種類だけを揃える。減衰を入れるかは
    モデルごとに学習し比べて決めること（ここで一律に入れると、既存の run と比べられなくなる）。
    """
    import torch

    return torch.optim.AdamW(params, lr=lr, weight_decay=weight_decay)


def train_epoch(model, loader, optimizer, device, loss_of: Callable) -> float:
    """`(入力, 手本)` の組を返す loader で1エポック学習し、標本数で重み付けした損失の平均を返す
    （`ml_cam`・`ml_cam_e2e` の `train_one_epoch`。損失だけ `loss_of(出力, 手本)` で差し替える）。"""
    model.train()
    total_loss = 0.0
    n = 0
    for x, target in loader:
        x, target = x.to(device), target.to(device)
        optimizer.zero_grad()
        loss = loss_of(model(x), target)
        loss.backward()
        optimizer.step()
        total_loss += loss.item() * x.size(0)
        n += x.size(0)
    return total_loss / max(n, 1)


class BestTracker:
    """検証の値の「最良」を追う。`mode="min"`（誤差）か `"max"`（IoU など）。

    初期値は `±inf`、比較は厳密な大小（同じ値では更新しない）、**NaN は決して最良にならない**
    （検証データが無い・壊れたエポックで best.pt を上書きしない）。以前は `-1.0` 始まり（IoU）と
    `math.inf` 始まり（誤差）が混在していた。
    """

    def __init__(self, mode: str = "min") -> None:
        if mode not in ("min", "max"):
            raise ValueError(f"mode は 'min' か 'max'（{mode!r}）")
        self.mode = mode
        self.best = math.inf if mode == "min" else -math.inf
        #: 最良を出したときに一緒に覚えておく値（例: その時の指標の dict）
        self.info = None

    def update(self, value: float, info=None) -> bool:
        """`value` が今までの最良より良ければ覚えて `True`。"""
        if value is None or math.isnan(value):
            return False
        better = value < self.best if self.mode == "min" else value > self.best
        if better:
            self.best = value
            self.info = info
        return better

    @property
    def found(self) -> bool:
        """一度でも有限の値で更新されたか。"""
        return math.isfinite(self.best)

    def best_or_nan(self) -> float:
        return self.best if self.found else math.nan


def time_block(t_ns: int, block_s: float) -> int:
    """時刻 [ns] が属する `block_s` 秒のかたまりの番号。"""
    return int(t_ns) // max(1, int(block_s * 1_000_000_000))


def _hash_bin(key: str, seed: int) -> int:
    """`key`・`seed` から決まる 0〜`_HASH_BINS`-1 の値（crc32 なので Python の起動ごとに変わらない）。
    文字列の形 `"<key>:<seed>"` は `ml_cam_e2e/samples.is_val` の頃から変えていない（既存の run の
    `eval_model` が train/val を同じに言い直せるように）。"""
    return zlib.crc32(f"{key}:{seed}".encode()) % _HASH_BINS


def is_val_group(key: str, *, val_ratio: float, seed: int = 0) -> bool:
    """かたまり `key`（例 `"<元ファイル>:<時間ブロック番号>"`）を検証に回すか。

    **かたまりの名前だけで決まる**（他にどんなデータがあるかに依らない）。だから
    - 記録を足しても既存のかたまりの振り分けが動かない（前の run と同じ検証データで比べられる）
    - 1枚だけを見て train/val を言える（`ml_cam_e2e/eval_model.split_of` が学習後に使う）
    """
    return _hash_bin(key, seed) < val_ratio * _HASH_BINS


def group_split(items: Iterable[T], key: Callable[[T], Hashable], *, val_ratio: float, seed: int = 0,
                keep_both: bool = False) -> tuple[list[T], list[T]]:
    """`items` を `key(item)` のかたまり単位で train/val に分ける（同じかたまりは片側だけ）。

    連続した走行の隣り合う絵はほぼ同じなので、1枚ずつ振り分けると train の隣が val に漏れて検証の値が良く出すぎる。
    振り分けは `is_val_group`（かたまりの名前のハッシュ）で決める。val の割合はかたまりの数が多ければ `val_ratio` に
    近づくが、少ないとばらつく。

    `keep_both=True` なら、かたまりが2つ以上あるのに片側が空になったとき、ハッシュの値がいちばん境目に近い
    かたまりを1つ移して両側を残す（ラベル付きの画像が少ない `ml_cam` 向け）。その場合だけは `is_val_group` と
    食い違うので、学習後に1枚ずつ train/val を言い直す使い方（`ml_cam_e2e`）では使わない。

    以前は `ml_cam` が「ハッシュの順に並べ、val の枚数が `val_ratio` に届くまで詰める」方式、`ml_cam_e2e` がこの方式
    だった。前者は割合がちょうどになる代わりに、記録を足すと既存のかたまりの振り分けまで動く（前の run と検証データが
    変わり、足した記録の分だけ以前の学習データが検証に混ざる）ので、こちらに揃えた。
    """
    keyed = [(str(key(item)), item) for item in items]
    groups = dict.fromkeys(k for k, _ in keyed)
    val_keys = {k for k in groups if is_val_group(k, val_ratio=val_ratio, seed=seed)}
    if keep_both and len(groups) >= 2:
        if not val_keys:
            val_keys = {min(groups, key=lambda k: _hash_bin(k, seed))}
        elif len(val_keys) == len(groups):
            val_keys.discard(max(groups, key=lambda k: _hash_bin(k, seed)))
    # 元の並びのまま振り分ける（`ml_cam_e2e/samples.split` の頃と同じ並び）
    train = [item for k, item in keyed if k not in val_keys]
    val = [item for k, item in keyed if k in val_keys]
    return train, val
