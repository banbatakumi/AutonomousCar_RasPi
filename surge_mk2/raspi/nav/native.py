"""レーシングラインの計算の重い内側ループを C で回す（`_native.c` を ctypes で読む）。

## なぜ C か

経路の仕上げ（`raceline.refine_time`）は見積もりのラップ時間を数万回測る。その中身の速度
プロファイルは前後に1点ずつ進む漸化式で numpy にできず、Python のループが Pi で 1 回約 0.6ms。
最小曲率の QP（`raceline.min_curvature_alpha`）は帯行列なのに密行列で解いていて、Pi の numpy は
参照 BLAS（Mac の 100 倍超遅い）なので n³ が効く。magp2（50m）の自動経路は Pi で仕上げなし 15s・
仕上げ込み 75s かかっていた。Pi には scipy も numba も無い。

## 組み方

`slam2d/core/cbuild.py`（初回の import で `cc` が組み、`~/.cache/surge_mk2/` に置く）。**組めなければ
`lib` は `None` で、呼び出し側は Python 版で回す**（結果は同じ。遅いだけ）。`SURGE_NO_NATIVE=1` で C を
使わない。帯行列の解法以外は Python 版とビット一致する（`raspi/tests/test_native.py`）
"""

from __future__ import annotations

import ctypes
from pathlib import Path

from slam2d.core.cbuild import addr, f64, load

__all__ = ["lib", "SOURCE", "addr", "f64"]

SOURCE = Path(__file__).with_name("_native.c")


def _load():
    i64, dbl, ptr = ctypes.c_int64, ctypes.c_double, ctypes.c_void_p
    sig = {
        "sp_closed": (None, [i64, ptr, ptr, ptr, dbl, dbl, dbl]),
        "sp_open_back": (None, [i64, ptr, ptr, ptr, dbl, dbl]),
        "sp_open_fwd": (None, [i64, ptr, ptr, ptr, dbl, dbl]),
        "curvature": (None, [i64, ptr, ctypes.c_int, ptr]),
        "chord_curvature": (None, [i64, ptr, i64, ctypes.c_int, ptr]),
        "speed_kappa": (None, [i64, ptr, ctypes.c_int, dbl, ptr]),
        "body_min_edt": (None, [i64, ptr, ptr, ctypes.c_int, i64, ptr, ptr, ptr, i64, i64,
                                dbl, dbl, dbl, ptr]),
        "band_solve": (ctypes.c_int, [i64, ptr, ptr, ptr, dbl, dbl, dbl, ptr, ptr]),
        "seg_len": (None, [i64, ptr, ctypes.c_int, ptr]),
        "all_inside": (ctypes.c_int, [i64, ptr, ptr, i64, i64, dbl, dbl, dbl]),
        "zs_thin": (None, [i64, i64, ptr, i64]),
        "refine_kappa": (None, [i64, ptr, dbl, ptr, ptr, ptr]),
        "refine_eval": (ctypes.c_int, [i64, ptr, ptr, ptr, ptr, ptr, i64, i64, dbl, dbl, dbl,
                                       ptr, i64, i64, dbl, dbl, dbl,
                                       i64, ptr, ptr, ctypes.c_float, dbl,
                                       dbl, dbl, dbl, dbl, dbl, dbl, ptr]),
    }
    return load(SOURCE, "native", sig)


#: 読み込んだライブラリ（作れなければ `None`）
lib = _load()
