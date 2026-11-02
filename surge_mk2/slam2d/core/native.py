"""SLAM の重い内側ループを C で回す（`_native.c` を `cbuild.py` で組んで ctypes で読む）。

## なぜ C か

`OccGrid.raycast`（追跡の「壁の向こうの点」の判定 `Frontend._not_behind_wall`）は、レイ約1000本を全部
最後の歩まで numpy の一時配列で進め、Pi で1周期 約6ms——走行中の `plan()` の約半分——かかっていた。
`_carve`（地図作成の「素通り」の数え）も同じ形。C は当たった所・終端で止める。
`SurfaceMap.associate`（ICP の対応付け）と、総当たりの採点（`register.search_steps`・`localize`）も C。
採点は numpy 版が「候補 × 点」の配列（LOCATE で数十万要素）を作ってから足していた。
面要素の法線の材料（`SurfaceMap.__init__` の 25 近傍の和。地図作成中に3周期に1回）も C。
組めなければ `lib` は `None` で、Python 版で回る（結果は同じ。遅いだけ）。
"""

from __future__ import annotations

import ctypes
from pathlib import Path

from .cbuild import addr, f64, load

__all__ = ["lib", "SOURCE", "SmapC", "addr", "f64"]

SOURCE = Path(__file__).with_name("_native.c")


class SmapC(ctypes.Structure):
    """`_native.c` の `smap_t`（`SurfaceMap` の配列。`SurfaceMap._c_map` が作る）。"""
    _fields_ = [("h", ctypes.c_int64), ("w", ctypes.c_int64),
                ("o0", ctypes.c_double), ("o1", ctypes.c_double), ("res", ctypes.c_double),
                ("nearest", ctypes.c_void_p), ("known", ctypes.c_void_p),
                ("qx", ctypes.c_void_p), ("qy", ctypes.c_void_p),
                ("nx", ctypes.c_void_p), ("ny", ctypes.c_void_p),
                ("planar", ctypes.c_void_p), ("support", ctypes.c_void_p),
                ("gate", ctypes.c_double), ("max_dist", ctypes.c_double)]


def _load():
    i64, flt, dbl, ptr = ctypes.c_int64, ctypes.c_float, ctypes.c_double, ctypes.c_void_p
    sm = ctypes.POINTER(SmapC)
    return load(SOURCE, "slam2d", {
        "sm_associate": (None, [sm, i64, ptr, ptr, ptr, ptr, ptr, ptr, ptr, ptr, ptr]),
        "sm_score_rows": (None, [sm, i64, ptr, ptr, i64, ptr, ptr, dbl, ptr]),
        "normal_sums": (None, [i64, ptr, ptr, ptr, i64, i64, ptr, ptr, i64,
                               ptr, ptr, ptr, ptr, ptr, ptr]),
        "ray_first_hit": (None, [i64, ptr, ptr, i64, ptr, ptr, i64, ptr, ptr, i64, i64,
                                 flt, flt, flt, ptr]),
        "carve_touch": (None, [i64, dbl, dbl, ptr, ptr, ptr, i64, ptr, dbl, dbl, dbl, dbl,
                               i64, i64, ptr, ctypes.c_uint16, ptr]),
    })


#: 読み込んだライブラリ（作れなければ `None`）
lib = _load()
