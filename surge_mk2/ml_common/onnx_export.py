"""ONNX 書き出しの共通部分（`ml_cam`・`ml_cam_e2e`・`ml_e2e`・`ml_stereo_e2e` の `export_onnx.py`）。

4本の `export_onnx.py` が `torch.onnx.export` の設定・PyTorch と ONNXRuntime の突き合わせ・
契約 JSON の書き出し・前処理の版の検査をそれぞれ複製していた。片方に入れた修正（`external_data=False`
の罠など）がもう片方に入らないので、ここに1つだけ置く。各 `export_onnx.py` に残すのは、モデルの組み立てと
「契約に何を書くか」（＝実車のノードと結ぶ約束の中身）だけ。

**torch・onnxruntime は関数の中で import する。** 学習 GUI（このプロセスは torch を読まない）が
`ml_common` を import しても重くならないようにするため。
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Callable, Sequence

import numpy as np

__all__ = ["OPSET", "export", "verify_parity", "write_contract", "check_version", "read_train_config"]

#: **18 未満を指定すると失敗する。** torch の dynamo エクスポータ（既定）は `interpolate` を新しい opset の
#: 演算子表現に変換してから下位変換を試みるが、対応できず盛大な警告を出したうえで結局 18 にフォールバックする
#: （実測）。最初から 18 を指定して無駄な変換の試行錯誤を避ける。**onnxruntime>=1.18 は opset 18 を扱える。**
OPSET = 18


def export(model, dummy_inputs: Sequence[np.ndarray] | np.ndarray, out_path: Path, *,
           input_names: Sequence[str] = ("input",), output_names: Sequence[str] = ("output",)) -> None:
    """`model`（eval にしておく）を `out_path` へ書き出す。`dummy_inputs` は形を決めるための numpy 配列
    （入力が1つなら配列そのもの、複数なら `input_names` と同じ並びのタプル）。"""
    import torch

    arrays = (dummy_inputs,) if isinstance(dummy_inputs, np.ndarray) else tuple(dummy_inputs)
    tensors = tuple(torch.from_numpy(a) for a in arrays)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(
        model, tensors[0] if len(tensors) == 1 else tensors, str(out_path),
        input_names=list(input_names), output_names=list(output_names),
        opset_version=OPSET,
        # **`external_data=False` が必須。** 既定(True)だと重みが `<out_path>.data` という別ファイルに
        # 切り出され、`.onnx` 単体をコピー/配置すると壊れる（`ml_lidar/export_onnx_rl.py` が同じ罠を実際に
        # 踏んで判明。Pi の `models/<種類>/` へは `.onnx` と `.json` だけを運ぶ運用なので、ここが漏れると
        # ノードのロードが実機でだけ失敗する）
        external_data=False,
    )


def verify_parity(onnx_path: Path, model, make_inputs: Callable[[np.random.Generator], Sequence[np.ndarray]], *,
                  input_names: Sequence[str] = ("input",), n_samples: int = 4, atol: float = 1e-3,
                  seed: int = 0) -> float:
    """PyTorch と ONNXRuntime の出力差の最大値を返す。`atol` を超えたら例外。

    **書き出せただけでは何も保証されない**——opset や演算子の対応漏れで静かに違う結果を返すことがあるので、
    乱数の入力で数値を突き合わせる。`make_inputs(rng)` は `input_names` と同じ並びの numpy 配列（float32）を返す。
    """
    import onnxruntime as ort
    import torch

    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    model.eval()
    rng = np.random.default_rng(seed)
    max_err = 0.0
    with torch.no_grad():
        for _ in range(n_samples):
            arrs = tuple(make_inputs(rng))
            ref = model(*(torch.from_numpy(a) for a in arrs)).numpy()
            got = session.run(None, dict(zip(input_names, arrs)))[0]
            max_err = max(max_err, float(np.abs(ref - got).max()))
    if max_err > atol:
        raise ValueError(f"PyTorch と ONNXRuntime の出力が最大 {max_err:.2e} ずれています（許容 {atol}）。"
                         "opset やモデル構造（補間の実装差など）を見直してください")
    return max_err


def write_contract(out_path: Path, contract: dict[str, Any]) -> Path:
    """`out_path` と同名の `.json` に契約を書き、そのパスを返す（備考 `note` の日本語をそのまま読めるよう
    `ensure_ascii=False`・UTF-8）。"""
    path = out_path.with_suffix(".json")
    path.write_text(json.dumps(contract, ensure_ascii=False, indent=2), encoding="utf-8")
    return path


def check_version(what: str, trained, current) -> None:
    """学習時の版 `trained` が今の版 `current` と**一致しなければ例外**。

    版が違う重みに今の版を書いて出すと、ノードの照合（契約の版を見る）をすり抜けて、学習時と違う前処理・
    入出力で推論することになる（train/inference skew）。古い版を黙って今の版に書き換えることはしない。
    `train_config.json` に版が無い（`None`）のも同じ扱い。
    """
    if trained != current:
        raise ValueError(f"学習時の{what}の版 {trained} が今の版 {current} と違う。今の版で学習し直してください")


def read_train_config(run_dir: Path) -> dict:
    """`run_dir/train_config.json`。無い・読めないなら空の dict（足りない値は呼び出し側が検査する）。"""
    try:
        return json.loads((run_dir / "train_config.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
