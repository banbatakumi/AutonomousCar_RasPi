"""ステレオの後処理の C 版（`stereo_native.c`）を、初回の import 時にその機械でビルドして ctypes で呼ぶ。

    from raspi.core.stereo_native import native_obstacles   # 使えなければ None

- ビルド先は `raspi/core/_native_build/`（git にも rsync にも載せない。Mac と Pi で別物になるため）。
  ファイル名にソースのハッシュと機種を入れるので、`.c` を直せば次の起動で作り直される
- コンパイラが無い・失敗したら None を返し、呼び出し側（`stereo_match.obstacles_from_disparity`）は numpy 版で続ける
- 結果は numpy 版と同じ（最下位の桁の丸めを除く。`raspi/tests/test_stereo_native.py`）。Pi 5・SGBM の帯（150〜260 行）で約 3ms
"""

from __future__ import annotations

import ctypes
import hashlib
import os
import platform
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

__all__ = ["native_obstacles", "load", "BUILD_DIR"]

_SRC = Path(__file__).with_name("stereo_native.c")
BUILD_DIR = Path(__file__).with_name("_native_build")
#: `-ffast-math` は付けない（丸め・比較を numpy 版と揃えるため）
_CFLAGS = ["-O2", "-shared", "-fPIC", "-std=c99", "-D_DEFAULT_SOURCE"]

_lib = None
_tried = False


def _lib_path() -> Path:
    h = hashlib.sha1(_SRC.read_bytes() + " ".join(_CFLAGS).encode()).hexdigest()[:10]
    return BUILD_DIR / f"stereo_native-{sys.platform}-{platform.machine()}-{h}.so"


def _build(out: Path) -> None:
    BUILD_DIR.mkdir(exist_ok=True)
    cc = os.environ.get("CC", "cc")
    # 同時に2つ起動しても壊れないよう、一時ファイルに作ってから置き換える
    fd, tmp = tempfile.mkstemp(suffix=".so", dir=BUILD_DIR)
    os.close(fd)
    try:
        subprocess.run([cc, *_CFLAGS, str(_SRC), "-o", tmp, "-lm"], check=True,
                       capture_output=True, text=True, timeout=60)
        os.replace(tmp, out)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)
    # 同じ機種向けの古いビルド（前の版のソース）を消す。読み込み中の別プロセスがあっても unlink なら壊れない
    for old in BUILD_DIR.glob(f"stereo_native-{sys.platform}-{platform.machine()}-*.so"):
        if old != out:
            try:
                old.unlink()
            except OSError:
                pass


def load():
    """ビルド済みのライブラリ（無ければビルドする）。使えなければ None。結果は覚えておく。"""
    global _lib, _tried
    if _tried:
        return _lib
    _tried = True
    try:
        path = _lib_path()
        if not path.exists():
            _build(path)
        lib = ctypes.CDLL(str(path))
        f = lib.stereo_obstacles
        dp = ctypes.POINTER(ctypes.c_double)
        L, D = ctypes.c_long, ctypes.c_double
        f.argtypes = [ctypes.POINTER(ctypes.c_float), L, L, L, L,
                      D, D, D, D, D, D, dp, dp,
                      D, D, D, D, L, D, D, D,
                      dp, dp, ctypes.POINTER(ctypes.c_long)]
        f.restype = L
        _lib = lib
    except (OSError, subprocess.SubprocessError, AttributeError) as e:
        msg = getattr(e, "stderr", "") or str(e)
        print(f"!! stereo_native を使えない（numpy 版で続ける）: {msg.strip()[:300]}", file=sys.stderr, flush=True)
        _lib = None
    return _lib


def native_obstacles(rect, d: np.ndarray, R: np.ndarray, t: np.ndarray, q, *, scale: float,
                     skip_cols: int) -> tuple[list[float], int, int] | None:
    """C 版の後処理。`(dist[360], 障害物とみなした点の数, 有効画素の数)`。使えなければ None。

    :param R, t: 平行化座標 → base_link（`P_base = R·P + t`）
    :param q: `ObstacleParams`
    """
    lib = load()
    if lib is None:
        return None
    d = np.ascontiguousarray(d, dtype=np.float32)
    h, w = d.shape
    R = np.ascontiguousarray(R, dtype=np.float64).reshape(9)
    t = np.ascontiguousarray(t, dtype=np.float64).reshape(3)
    work = np.empty(2 * h * w, np.float64)
    dist = np.zeros(360, np.float64)
    n_valid = ctypes.c_long(0)
    dp = ctypes.POINTER(ctypes.c_double)
    n = lib.stereo_obstacles(
        d.ctypes.data_as(ctypes.POINTER(ctypes.c_float)), h, w, w, int(skip_cols),
        float(rect.f), float(rect.baseline), float(rect.cx), float(rect.cy), float(rect.rows[0]),
        float(scale), R.ctypes.data_as(dp), t.ctypes.data_as(dp),
        float(q.z_min), float(q.z_max), float(q.r_min), float(q.r_max), int(q.min_points),
        float(q.near_pct), float(q.min_height_span), float(q.slice_m),
        work.ctypes.data_as(dp), dist.ctypes.data_as(dp), ctypes.byref(n_valid))
    if n < 0:
        return None
    return dist.tolist(), int(n), int(n_valid.value)
