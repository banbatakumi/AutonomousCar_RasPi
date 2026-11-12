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

## 誤差が小さくても走れるとは限らない——ずれへの応答 dδ/dψ も見る

手本の線の上の絵だけで測った誤差は、線から外れたときに戻せるかを測れない
（Codevilla et al. 2018: オフラインの誤差は実走の良さとほとんど対応しない）。
v4 は val 誤差 0.5° でも実車では数mで線を外れた。そこで、絵を「車が向きを ψ
変えた絵」に作り替え（`yaw_aug.py` と同じ方法）、舵がどれだけ戻す向きに動くか
（`yaw_response`）を出す。**負で大きいほど戻す力が強い。** 手本の追従（pure pursuit・
先読み 0.3〜0.6m）は約 −0.8〜−1.5、v4 は約 −0.12 だった。

## 外乱で舵がどれだけ変わるか

舵の正解を変えない外乱（ブレ・ノイズ・色味・JPEG・明るさ。`photo_aug.py`）を掛けた絵と
元の絵で舵を比べる（`perturbation_response`）。**変化が小さいほど外乱に強い。** 特に
横ブレは「旋回の速さの手がかり」になりやすく、ブレで舵が増える・減るモデルは手本と違う
速度で走ると舵が合わない（2026-10-30、v6 は横ブレ 20px で舵が 1.4 倍）。切っている場面の
舵の比（外乱あり / なし）も出す。
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

__all__ = ["EVAL_COLUMNS", "SPLIT_TRAIN", "SPLIT_VAL", "SPLIT_UNUSED", "SPLIT_NEW",
           "split_of", "speed_trained", "load_train_config", "read_eval", "summarize",
           "worst_segments", "perturbations", "perturbation_response"]

EVAL_COLUMNS = ["file", "source_mcap", "cam", "t_capture_ns", "split",
                "steer_true", "steer_pred", "speed_true", "speed_pred"]
SPLIT_TRAIN = "train"
SPLIT_VAL = "val"
#: 除外区間・後退・静止など、学習に使っていないフレーム。予測は出すが誤差の集計には入れない
SPLIT_UNUSED = "unused"
#: 学習後に追加された記録（`train_config.json`の`train_sources`に無い）。学習にも検証にも
#: 使っていないので、「train」と誤表示せず別に数える（本当に見たことのないデータでの誤差）
SPLIT_NEW = "new"

NS = 1_000_000_000


def split_of(sample: S.Sample, exclusions: list[S.Exclusion], train_cfg: dict) -> str:
    if not S.usable(sample, exclusions):
        return SPLIT_UNUSED
    sources = train_cfg.get("train_sources")
    if sources is not None and sample.source_mcap not in sources:
        return SPLIT_NEW
    is_val = S.is_val(sample.source_mcap, sample.t_ns,
                      block_s=float(train_cfg.get("block_s", 5.0)),
                      val_ratio=float(train_cfg.get("val_ratio", 0.15)),
                      seed=int(train_cfg.get("seed", 0)))
    return SPLIT_VAL if is_val else SPLIT_TRAIN


def load_train_config(run_dir: Path) -> dict:
    """`<run>/train_config.json`。無い・読めないなら空の dict。"""
    try:
        return json.loads((run_dir / "train_config.json").read_text())
    except (OSError, ValueError):
        return {}


def speed_trained(train_cfg: dict) -> bool:
    """速度の出力を学習したモデルか。

    `--speed-weight 0`（既定）で学習したモデルの `speed_norm` は、最終層が
    初期値のままの学習されていない値——舵の特徴が写り込んで絵に応じて動くので
    それらしく見えるが、意味は無い。評価に出すと誤解のもとなので隠す。
    `speed_weight` が書かれていない古い設定は「学習した」とみなす（当時の既定は 0.5）。
    """
    return float(train_cfg.get("speed_weight", 0.5)) > 0


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
    """`{"val": {"n", "steer_mae_deg", "speed_mae"}, "train": {...}}`。学習後に追加された
    記録（`new`）があれば同じ形で`"new"`も入る。"""
    out = {}
    names = [SPLIT_VAL, SPLIT_TRAIN]
    if any(r["split"] == SPLIT_NEW for r in rows):
        names.append(SPLIT_NEW)
    for name in names:
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


def _infer(model, pair) -> tuple[float, float]:
    """`pair` は `(右の RGB, 左の RGB)`。"""
    import numpy as np

    rgb, rgb_left = pair
    return model.infer(np.ascontiguousarray(rgb), np.ascontiguousarray(rgb_left))


def yaw_response(model, pairs: list, fx_px, body, deg: float = 3.0) -> list[float]:
    """各組について dδ/dψ（車が左へ ψ 向いたときの舵の変化 / ψ、無次元）を返す。

    `pairs` は `(右の RGB, 左の RGB)` の列。`fx_px`・`body` は `(右, 左)` の組（1つの値なら両方に使う）。
    絵の全体を横にずらし（端は端の画素で延長）、車体は元の位置に戻してから
    `model.infer()` に通す。ずらしが切り落とし幅以内なら延長した端が窓の外に落ちるので、
    学習の拡張と同じ絵になる。同じ ψ を左右両方に掛ける。
    """
    import numpy as np
    from yaw_aug import yawed_window

    fxs = tuple(fx_px) if isinstance(fx_px, (tuple, list)) else (fx_px, fx_px)
    bodies = tuple(body) if isinstance(body, (tuple, list)) else (body, body)
    psi = math.radians(deg)

    def turned(rgb, i: int, sign: float):
        if rgb is None:
            return None
        shift = round(fxs[i] * psi)
        # 両側に shift ぶん端を延長してから窓をずらす＝全体をずらして端を延長したのと同じ
        padded = np.pad(rgb, ((0, 0), (shift, shift), (0, 0)), mode="edge")
        b = bodies[i]
        pb = None if b is None else np.pad(b, ((0, 0), (shift, shift)), mode="constant")
        return yawed_window(padded, shift, sign * psi, fxs[i], pb)

    out = []
    for rgb, rgb_left in pairs:
        d_left = _infer(model, (turned(rgb, 0, +1), turned(rgb_left, 1, +1)))[0] * model.max_steer
        d_right = _infer(model, (turned(rgb, 0, -1), turned(rgb_left, 1, -1)))[0] * model.max_steer
        out.append((d_left - d_right) / (2 * psi))
    return out


def perturbations() -> list[tuple[str, object]]:
    """`perturbation_response` で掛ける外乱（名前, 関数）。強さは `photo_aug` の既定の上限付近。"""
    import numpy as np
    import photo_aug as pa

    def blur(px):
        return lambda rgb: pa.horizontal_blur(rgb, round(rgb.shape[1] * px / 640))

    return [
        ("横ブレ12px", blur(12)),
        ("横ブレ24px", blur(24)),
        ("ノイズσ6", lambda rgb: pa.sensor_noise(rgb, 6.0, np.random.default_rng(0))),
        ("JPEG品質40", lambda rgb: pa.jpeg_roundtrip(rgb, 40)),
        ("暖色", lambda rgb: pa.color_shift(rgb, (1.08, 1.0, 0.92), 1.0)),
        ("寒色", lambda rgb: pa.color_shift(rgb, (0.92, 1.0, 1.08), 1.0)),
        ("暗い-25", lambda rgb: rgb.clip(25) - 25),
        ("明るい+25", lambda rgb: rgb.clip(0, 230) + 25),
    ]


def perturbation_response(model, pairs: list, turn_deg: float = 5.0) -> list[dict]:
    """外乱ごとに、舵の変化の平均 [deg] と、切っている場面の舵の比（外乱あり / なし）を返す。
    `pairs` は `(右の RGB, 左の RGB)` の列。同じ外乱を左右両方に掛ける。"""
    base = [_infer(model, p)[0] * model.max_steer for p in pairs]
    turning = [i for i, b in enumerate(base) if abs(b) > math.radians(turn_deg)]
    out = []
    for name, fn in perturbations():
        pert = [_infer(model, (fn(r), fn(l)))[0] * model.max_steer
                for r, l in pairs]
        diff = sum(abs(p - b) for p, b in zip(pert, base, strict=True)) / len(pairs)
        ratio = (sum(abs(pert[i]) for i in turning) / sum(abs(base[i]) for i in turning)
                 if turning else math.nan)
        out.append({"name": name, "mean_abs_diff_deg": math.degrees(diff),
                    "turn_ratio": ratio, "n_turn": len(turning)})
    return out


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

    train_cfg = load_train_config(args.run)
    if not train_cfg:
        print(f"# 警告: {args.run / 'train_config.json'} が無いので、"
              "既定の振り分けで val/train を決めます",
              file=sys.stderr)
    exclusions = S.load_exclusions(args.frames)
    n_all = len(samples)
    samples = [s for s in samples if S.has_left(s)]
    if len(samples) < n_all:
        print(f"# 左の画像が無い {n_all - len(samples)}件は評価しない", file=sys.stderr)
    if not samples:
        print(f"{args.frames} に左右の組がありません（extract_pairs.py で取り出し直す）", file=sys.stderr)
        return 2

    def load_pair(s):
        return S.load_rgb(s.path), S.load_rgb(S.left_path(s.path))

    out_path = args.out or args.run / "eval.csv"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(EVAL_COLUMNS)
        for i, s in enumerate(samples, 1):
            steer_n, speed_n = _infer(model, load_pair(s))
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
    with_speed = speed_trained(train_cfg)
    for name, label in ((SPLIT_VAL, "検証"), (SPLIT_TRAIN, "学習")):
        s = summary[name]
        print(f"# {label} {s['n']}件  操舵MAE {s['steer_mae_deg']:.2f}deg"
              + (f"  速度MAE {s['speed_mae']:.3f}m/s" if with_speed else ""))
    if not with_speed:
        print("# 速度は学習していないモデル（速度の重み 0）なので、速度の誤差は出しません")
    print(f"# 書き出し完了: {out_path}")

    # ── ずれへの応答（モジュールdocstring参照） ──
    from yaw_aug import BODY_MASK_LEFT_NAME, camera_fx_px, ensure_body_mask

    from raspi.core.vehicle import Vehicle

    val = [s for s in samples if split_of(s, exclusions, train_cfg) == SPLIT_VAL]
    pick = val[::max(1, len(val) // 60)][:60] or samples[::max(1, len(samples) // 60)][:60]
    if pick:
        pairs = [load_pair(s) for s in pick]
        vehicle = Vehicle.load()
        width = pairs[0][0].shape[1]
        # 止まっている間の絵は背景も動かないので、車体の推定には走行中の絵だけを使う
        moving = [s for s in samples if S.drop_reason(s) is None]
        fx_px = (camera_fx_px(vehicle, "front", width), camera_fx_px(vehicle, "left", width))
        body = (ensure_body_mask(args.frames, [s.path for s in moving]),
                ensure_body_mask(args.frames, [S.left_path(s.path) for s in moving],
                                 BODY_MASK_LEFT_NAME))
        resp = sorted(yaw_response(model, pairs, fx_px, body))
        med = resp[len(resp) // 2]
        print(f"# ずれへの応答 dδ/dψ: 中央値 {med:+.2f}（{len(resp)}枚。負で大きいほど戻す力が強い。"
              f"手本の追従は約 -0.8〜-1.5）")
        # ── 外乱で舵がどれだけ変わるか（モジュールdocstring参照） ──
        print("# 外乱による舵の変化（小さいほど強い。比は切っている場面の 外乱あり/なし、1.00 が理想）:")
        for r in perturbation_response(model, pairs):
            print(f"#   {r['name']:8s} 平均 {r['mean_abs_diff_deg']:4.1f}°  比 {r['turn_ratio']:.2f}"
                  f"（{r['n_turn']}枚）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
