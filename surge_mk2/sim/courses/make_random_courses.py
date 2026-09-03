"""シケイン・ヘアピン付きの複雑なコースを固定シードで生成し、PNG+JSONとして永続化する。

    python -m sim.courses.make_random_courses

`sim/random_course.py::generate_circuit_course()`は本来RL訓練(`sim/gym_env.py`)向けに
実行のたびランダム生成するprocedural コースだが、SLAM(`slam2d/`)の複雑コース精度評価
（長距離ドリフト対策の計画参照）にはコース形状を固定して再現できる必要がある。
`make_courses.py`と対になる位置づけで、複数シードから複雑なコースを選んで
固定ファイル化する。

生成パラメータ（シード値）はこのファイル内の`_SEEDS`に残してあるので、
再生成すれば同じ形状が得られる。
"""

from __future__ import annotations

import json

import numpy as np
from PIL import Image

from ..random_course import generate_circuit_course
from .make_courses import HERE

#: 採用するシード。`generate_circuit_course`は`_CIRCUIT_HAIRPIN_PROB=0.6`・
#: `_CIRCUIT_CHICANE_N_RANGE=(2, 5)`で毎回形状が変わるため、複数シードを試し、
#: シケイン・ヘアピンが十分含まれる（＝ユーザー提示の画像のような複雑さになる）
#: ものを手動で選んである。
_SEEDS: dict[str, int] = {
    "circuit_chicane_a": 9,
    "circuit_chicane_b": 14,
    "circuit_chicane_c": 22,
}

WIDTH = 1.0          # [m] 他の同梱コース(circuit.json等)と揃える
RESOLUTION = 0.02     # [m/px]


def _save_course(name: str, course) -> None:
    """`Course.grid`(bool, True=壁, row0=y最小)を`sim/course.py::Course.load()`
    が読める PNG+JSON として書き出す（`make_courses.py::_save()`と同じJSON規約）。

    PNGは画像表示の慣習(row0=上=y最大)に合わせるため、`grid`を上下反転してから
    書く——`Course.load()`側が読み込み時に`np.flipud`で戻す非対称な変換
    （`sim/course.py`冒頭docstring参照）と対にする。
    """
    pixels = np.where(np.flipud(course.grid), 0, 255).astype(np.uint8)
    Image.fromarray(pixels, mode="L").save(HERE / f"{name}.png")

    meta = {
        "name": name,
        "note": "generate_circuit_course()の固定シード出力（複雑コースSLAM評価用）",
        "resolution": course.resolution,
        "origin": list(course.origin),
        "start": list(course.start),
        "wall_threshold": 128,
    }
    (HERE / f"{name}.json").write_text(
        json.dumps(meta, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    h, w = course.grid.shape
    print(f"  {name}.png  {w}x{h}px  {w * course.resolution:.1f}x{h * course.resolution:.1f}m")

    # 中心線は再現用に別途npzで保存する（Course.load()はPNG読み込み時
    # centerlineをNoneにするため、slam2d側の高速ハーネスが必要とする
    # 真の中心線はここでしか残せない）
    np.savez(HERE / f"{name}_centerline.npz", centerline=course.centerline, width=WIDTH)


def main() -> None:
    print(f"# 複雑コース生成 (解像度 {RESOLUTION} m/px) -> {HERE}")
    for name, seed in _SEEDS.items():
        rng = np.random.default_rng(seed)
        course = generate_circuit_course(rng, name=name, width=WIDTH, resolution=RESOLUTION)
        _save_course(name, course)


if __name__ == "__main__":
    main()
