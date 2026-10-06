"""ml_cam_e2e/eval_model.py — エクスポートしたモデルを抽出済みの全フレームに掛け、予測と手本を並べる。

    python3 ml_cam_e2e/eval_model.py --frames ml_cam_e2e/runs/v1/frames \\
        --run ml_cam_e2e/runs/v1 --model models/cam_e2e/v1.onnx

結果は `<run>/eval.csv`。操作パネルの「評価」タブがこれを読んで時系列と
誤差の大きい場面を表示する。

## 実車と同じ経路で推論する

`raspi/nodes/cam_e2e_node.py` の `load_model()`・`RegressionModel` をそのまま使う。
PyTorch のチェックポイントではなく ONNX を、実車と同じ前処理・同じ契約の
読み方で通すので、ここで見た出力がそのまま実車で出る値になる（JPEG 経由か
生フレームかの差だけが残る）。

## val と train を分けて見る

`train_config.json` の `block_s`・`val_ratio`・`seed` で学習時と同じ振り分けを
再現する。**モデルの実力は val の誤差**。train の誤差だけ小さいなら覚え込み。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # repo root

import samples as S  # noqa: E402

__all__ = ["EVAL_COLUMNS", "SPLIT_TRAIN", "SPLIT_VAL", "SPLIT_UNUSED",
           "split_of", "read_eval", "summarize", "worst_segments"]

EVAL_COLUMNS = ["file", "source_mcap", "cam", "t_capture_ns", "split",
                "steer_true", "steer_pred", "speed_true", "speed_pred"]
SPLIT_TRAIN = "train"
SPLIT_VAL = "val"
#: 除外区間・後退・静止など、学習に使っていないフレーム。予測は出すが誤差の集計には入れない
SPLIT_UNUSED = "unused"

NS = 1_000_000_000


def split_of(sample: S.Sample, exclusions: list[S.Exclusion], train_cfg: dict) -> str:
    if not S.usable(sample, exclusions):
        return SPLIT_UNUSED
    is_val = S.is_val(sample.source_mcap, sample.t_ns,
                      block_s=float(train_cfg.get("block_s", 5.0)),
                      val_ratio=float(train_cfg.get("val_ratio", 0.15)),
                      seed=int(train_cfg.get("seed", 0)))
    return SPLIT_VAL if is_val else SPLIT_TRAIN


def read_eval(csv_path: Path) -> list[dict]:
    """`eval.csv` を読む。数値列は float/int に直す。"""
    rows = []
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            row["t_capture_ns"] = int(row["t_capture_ns"])
            for key in ("steer_true", "steer_pred", "speed_true", "speed_pred"):
                row[key] = float(row[key])
            rows.append(row)
    return rows


def summarize(rows: list[dict]) -> dict[str, dict]:
    """`{"val": {"n", "steer_mae_deg", "speed_mae"}, "train": {...}}`。"""
    out = {}
    for name in (SPLIT_VAL, SPLIT_TRAIN):
        part = [r for r in rows if r["split"] == name]
        n = len(part)
        out[name] = {
            "n": n,
            "steer_mae_deg": (math.degrees(sum(abs(r["steer_pred"] - r["steer_true"])
                                               for r in part) / n) if n else math.nan),
            "speed_mae": (sum(abs(r["speed_pred"] - r["speed_true"]) for r in part) / n
                          if n else math.nan),
        }
    return out


def worst_segments(rows: list[dict], k: int = 20, merge_s: float = 1.0) -> list[dict]:
    """操舵の誤差が大きい場面を上から `k` 個。

    誤差の大きいフレームは連続して現れる（同じコーナーの数枚）ので、1枚ずつ
    並べると上位が全部同じ場面になる。同じ記録の `merge_s` 秒以内は1つの
    場面にまとめ、その中で最も誤差の大きい1枚を代表にする。
    学習に使っていないフレーム（`unused`）は対象外。

    返す dict: `index`（`rows` の添字）・`source_mcap`・`t_capture_ns`・
    `err_deg`・`split`・`n_frames`。
    """
    cand = [(abs(r["steer_pred"] - r["steer_true"]), i) for i, r in enumerate(rows)
            if r["split"] != SPLIT_UNUSED]
    cand.sort(reverse=True)
    picked: list[dict] = []
    for err, i in cand:
        r = rows[i]
        near = next((p for p in picked if p["source_mcap"] == r["source_mcap"]
                     and abs(p["t_capture_ns"] - r["t_capture_ns"]) <= merge_s * NS), None)
        if near is not None:
            near["n_frames"] += 1
            continue
        if len(picked) >= k:
            # 上位 k 場面が出そろった後も、既存の場面に属する枚数だけは数え続ける
            continue
        picked.append({"index": i, "source_mcap": r["source_mcap"],
                       "t_capture_ns": r["t_capture_ns"], "err_deg": math.degrees(err),
                       "split": r["split"], "n_frames": 1})
    return picked


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", type=Path, required=True)
    ap.add_argument("--run", type=Path, required=True,
                    help="学習の出力先（train_config.json を読み、eval.csv を書く）")
    ap.add_argument("--model", type=Path, required=True, help="エクスポートした .onnx")
    ap.add_argument("--out", type=Path, default=None, help="既定は <run>/eval.csv")
    args = ap.parse_args()

    # 重い import はここで（操作パネルは上の純粋関数だけを import する）
    from raspi.nodes.cam_e2e_node import load_model

    samples = S.load_manifest(args.frames)
    if not samples:
        print(f"{args.frames} にペアがありません", file=sys.stderr)
        return 2
    try:
        model = load_model(args.model)
    except (ValueError, FileNotFoundError) as e:
        print(f"モデルを読めません: {e}", file=sys.stderr)
        return 2

    cfg_path = args.run / "train_config.json"
    train_cfg = json.loads(cfg_path.read_text()) if cfg_path.exists() else {}
    if not train_cfg:
        print(f"# 警告: {cfg_path} が無いので、既定の振り分けで val/train を決めます",
              file=sys.stderr)
    exclusions = S.load_exclusions(args.frames)

    out_path = args.out or args.run / "eval.csv"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(EVAL_COLUMNS)
        for i, s in enumerate(samples, 1):
            steer_n, speed_n = model.infer(S.load_rgb(s.path))
            row = {
                "file": s.path.name, "source_mcap": s.source_mcap, "cam": s.cam,
                "t_capture_ns": s.t_ns, "split": split_of(s, exclusions, train_cfg),
                # 手本も学習時と同じ範囲に切ってから比べる（モデルが出せない値との差を
                # 誤差に数えない）
                "steer_true": S.steer_norm(s, model.max_steer) * model.max_steer,
                "steer_pred": steer_n * model.max_steer,
                "speed_true": S.speed_norm(s, model.speed_ref) * model.speed_ref,
                "speed_pred": speed_n * model.speed_ref,
            }
            w.writerow([row[c] for c in EVAL_COLUMNS])
            rows.append(row)
            if i % 200 == 0 or i == len(samples):
                print(f"# {i}/{len(samples)}", flush=True)

    summary = summarize(rows)
    for name, label in ((SPLIT_VAL, "検証"), (SPLIT_TRAIN, "学習")):
        s = summary[name]
        print(f"# {label} {s['n']}件  操舵MAE {s['steer_mae_deg']:.2f}deg  "
              f"速度MAE {s['speed_mae']:.3f}m/s")
    print(f"# 書き出し完了: {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
