"""C のソース1つを共有ライブラリに組んで ctypes で読む（`slam2d/core/native.py`・`raspi/nav/native.py` が使う）。

初回の読み込みで `cc` で組み、`~/.cache/surge_mk2/` に置く（ファイル名にソースと組み方の指紋・CPU の
種類を入れる。リポジトリの中に置かないのは、`deploy.sh` の rsync で Mac の .so が Pi へ行かないように
するため）。**組めなければ `None`** を返し、呼び出し側は Python 版で回す（結果は同じ。遅いだけ）。
`SURGE_NO_NATIVE=1` なら C を使わない（テストで両方を比べる）。

★ `-ffp-contract=off`: 積和を1命令に融合させない（aarch64 の gcc は既定で融合し、丸めが Python 版と
  変わる）。`-ffast-math` も使わない。各ソースは Python 版と同じ順の演算で書き、結果をビット一致させる
"""

from __future__ import annotations

import ctypes
import hashlib
import os
import platform
import subprocess
import tempfile
from pathlib import Path

import numpy as np

__all__ = ["FLAGS", "load", "addr", "f64"]

FLAGS = ("-O2", "-fPIC", "-shared", "-ffp-contract=off", "-fno-fast-math")


def _cache_dir() -> Path:
    base = os.environ.get("XDG_CACHE_HOME") or str(Path.home() / ".cache")
    return Path(base) / "surge_mk2"


def _build(source: Path, prefix: str) -> Path:
    tag = hashlib.sha256(source.read_bytes() + " ".join(FLAGS).encode()).hexdigest()[:16]
    out = _cache_dir() / f"{prefix}-{tag}-{platform.machine()}.so"
    if out.exists():
        return out
    out.parent.mkdir(parents=True, exist_ok=True)
    # 同時に起動した複数のプロセス（経路のワーカー2本など）が作っても壊れないよう、一時名で
    # 作ってから置き換える
    fd, tmp = tempfile.mkstemp(dir=out.parent, suffix=".so")
    os.close(fd)
    try:
        subprocess.run([os.environ.get("CC", "cc"), *FLAGS, "-o", tmp, str(source), "-lm"],
                       check=True, capture_output=True, timeout=60)
        os.replace(tmp, out)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)
    return out


def load(source: Path, prefix: str, sigs: dict):
    """`source` を組んで読み、`sigs`（関数名 → (戻り値の型, 引数の型の列)）を設定したライブラリ。
    組めない・`SURGE_NO_NATIVE` なら `None`。"""
    if os.environ.get("SURGE_NO_NATIVE"):
        return None
    try:
        so = ctypes.CDLL(str(_build(source, prefix)))
        for name, (res, args) in sigs.items():
            f = getattr(so, name)
            f.restype, f.argtypes = res, args
    except Exception:                       # noqa: BLE001  コンパイラが無い等。Python 版で回す
        return None
    return so


def f64(a) -> np.ndarray:
    """C へ渡せる連続な float64 の配列（既にそうなら写さない）。"""
    return np.ascontiguousarray(a, dtype=np.float64)


def addr(a: np.ndarray) -> int:
    """配列の先頭のアドレス（`a.ctypes.data_as` より速い）。"""
    return a.__array_interface__["data"][0]
