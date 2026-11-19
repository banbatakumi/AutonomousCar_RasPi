"""ml_stereo_e2e/eval.py — 記録を、実車と同じ推論ノードと planner に通して、手本と比べる（開ループ）。

    .venv/bin/python -m ml_stereo_e2e.eval --model models/stereo_e2e/v1.onnx --data ml_stereo_e2e/runs/v1/data REC.mcap
    .venv/bin/python -m ml_stereo_e2e.eval --check-disp REC.mcap      # Pi の視差と Mac で作り直した視差を比べるだけ

**学習に使っていない記録で見ること。** 記録の左右の組ごとに、視差の帯を `extract.py` と同じ照合で作り、
`raspi/nodes/stereo_e2e_node.StereoE2ENode.infer_once`（ONNX・前処理とも実車と同じコード）→
`raspi/auto/stereo_e2e.StereoE2E.plan`（既定の MPC で経路を追う）まで通す。出す列は `ml_e2e/eval.py` と同じ
（走れる割合・舵の相関と RMS・速度の比・経路の誤差・dδ/dψ と理想比・推論の時間）。

推論の時刻は「撮影＋`_DISP_ARRIVE_S`」（Pi で撮影から `stereo/disp` まで中央値 約 45ms）。車は記録のとおりに動くので、
走れるかどうかは実走でしか分からない。

## `--check-disp`

記録に Pi の `/stereo/disp` がある（このモードを入れた後の記録）なら、同じ撮影時刻の左右の組（JPEG）を Mac で照合し直した値と
比べ、視差（ch0）の差の平均と、照合の出た割合（ch1）の差の平均を出す。学習（JPEG から作る）と実車（生の画像から作る）の
違いの大きさを見るため。
"""

from __future__ import annotations

import argparse
import base64
import io
import json
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))
sys.path.insert(0, str(_ROOT / "ml_cam_e2e"))

from yaw_aug import yawed_window  # noqa: E402

from ml_e2e import labels as L  # noqa: E402
from ml_e2e import records as R  # noqa: E402
from ml_e2e.eval import (_uncrop, _vs, add_yaw, ideal_steer, is_driving, label_index, load_labels,  # noqa: E402
                         new_result, print_header, print_row, score_step, yaw_window)
from ml_stereo_e2e.extract import disparity_for, make_processor, vehicle_for  # noqa: E402
from raspi.auto.registry import merged_params  # noqa: E402
from raspi.auto.stereo_e2e import StereoE2E  # noqa: E402
from raspi.core.stereo_e2e_preproc import (DISP_VERSION, crop_disp, disp_from_bytes, disp_input,  # noqa: E402
                                           disp_to_bytes, yaw_shift_disp)
from raspi.core.vehicle import Vehicle  # noqa: E402
from raspi.msgs import StereoDisp, VehicleState  # noqa: E402
from raspi.nodes.stereo_e2e_node import StereoE2ENode, StereoOnnx, load_model  # noqa: E402

__all__ = ["replay", "yaw_response", "check_disp", "main"]

NS = 1_000_000_000
#: 撮影から `stereo/disp` が届くまで [s]（Pi の実測の中央値）
_DISP_ARRIVE_S = 0.045


def model_settings(model: StereoOnnx) -> dict:
    """モデルの契約の照合の設定（`extract.make_processor` に渡す形）。"""
    cfg = json.loads(Path(model.path).with_suffix(".json").read_text())
    return {"algo": cfg["disp_algo"], "scale": cfg["disp_scale"], "num_disp": cfg["disp_num_disp"],
            "block": cfg["disp_block"], "uniqueness": cfg["disp_uniqueness"], "rows": cfg["disp_rows"],
            "disp_hw": cfg["disp_hw"]}


def _disp_msg(arr: np.ndarray, model: StereoOnnx, t: int, seq: int) -> StereoDisp:
    return StereoDisp(data=disp_to_bytes(arr), h=arr.shape[1], w=arr.shape[2], num_disp=model.disp_num_disp,
                      scale=model.disp_scale, rows=list(model.disp_rows), preproc_version=DISP_VERSION,
                      algo=model.disp_algo, block=model.disp_block, uniqueness=model.disp_uniqueness,
                      t_capture=t, seq=seq)


def replay(mcap: Path, node: StereoE2ENode, vehicle: Vehicle, *, params: dict[str, float],
           band: tuple[float, float] | None, labels: dict | None = None, yaw_every: int = 0) -> dict:
    """記録1本を推論ノードと planner に通す。数字の dict を返す。"""
    rec = R.load(mcap)
    model = node.model
    proc = make_processor(vehicle_for(mcap)[0], model_settings(model))
    planner = StereoE2E()
    label_idx, stride = label_index(labels, model.ds)
    out = new_result()
    vi = 0
    last_plan_t = 0
    for seq, (t, jpg, jpg_left) in enumerate(R.iter_image_pairs(mcap)):
        t_now = t + int(_DISP_ARRIVE_S * NS)
        while vi < len(rec.vs_t) and rec.vs_t[vi] <= t_now:
            planner.on_vehicle_state(_vs(rec, vi))
            vi += 1
        if vi == 0:
            continue
        vs = _vs(rec, vi - 1)
        ci = min(max(int(np.searchsorted(rec.cmd_t, t_now)), 0), len(rec.cmd_t) - 1)
        if not is_driving(rec, ci, vs):
            continue
        try:
            arr, _ = disparity_for(proc, jpg, jpg_left, model.disp_hw)
        except ValueError:
            out["reasons"]["照合できない"] += 1
            continue
        rgb = np.asarray(Image.open(io.BytesIO(jpg)).convert("RGB"))
        rgb_left = np.asarray(Image.open(io.BytesIO(jpg_left)).convert("RGB"))
        plan = node.infer_once(disp=_disp_msg(arr, model, t, seq), rgb=rgb, rgb_left=rgb_left, vs=vs,
                               now_ns=t_now, seq=seq)
        dt = (t_now - last_plan_t) / NS if last_plan_t else 0.0
        last_plan_t = t_now
        st = planner.plan(plan, vs, params, dt)
        if not score_step(out, plan, st, rec, ci, model, labels, label_idx.get(t), stride):
            continue

        if yaw_every and out["ready"] % yaw_every == 0:
            g = yaw_response(model, vehicle, rgb, rgb_left, arr, vs, params, band,
                             ref_xy=np.column_stack([plan.path_x, plan.path_y]))
            add_yaw(out, g)
    return out


def yaw_response(model: StereoOnnx, vehicle: Vehicle, rgb: np.ndarray, rgb_left: np.ndarray, disp: np.ndarray,
                 vs: VehicleState, params: dict[str, float], band: tuple[float, float], psi_deg: int = 4,
                 ref_xy: np.ndarray | None = None) -> tuple[float, float] | None:
    """車の向きを ±`psi_deg` ずらした入力での、経路を追う舵の変化 dδ/dψ [rad/rad] と、その理想の値。

    `ml_e2e/eval.yaw_response` と同じ測り方。点群の代わりに視差の帯を同じ角度だけ透視投影どおりに横にずらし
    （`stereo_e2e_preproc.yaw_shift_disp`。`band` は帯の列で測った焦点距離・主点）、学習と同じ窓を切る。
    """
    win_geom = yaw_window(vehicle, rgb, model.crop_x, psi_deg)
    if win_geom is None:
        return None
    fx, fx_left, margin = win_geom
    look = params["look_k"] * vs.speed + params["look_min"]
    steers, ideal = [], []
    for sgn in (-1, 1):
        psi = math.radians(sgn * psi_deg)
        win = _uncrop(yawed_window(rgb, margin, psi, fx, None), margin)
        win_left = _uncrop(yawed_window(rgb_left, margin, psi, fx_left, None), margin)
        o = model.infer(model.image_input(win, win_left), disp_input(crop_disp(yaw_shift_disp(disp, psi, *band), model.disp_crop_cols)),
                        model.state_input(vs.speed, vs.steer_actual, vs.yaw_rate))
        steers.append(L.pursuit_steer(o.path_xy, look, vehicle.wheelbase, vehicle.max_steer))
        if ref_xy is not None:
            ideal.append(ideal_steer(ref_xy, psi, look, vehicle))
    span = math.radians(2 * psi_deg)
    g_ideal = (ideal[1] - ideal[0]) / span if len(ideal) == 2 else float("nan")
    return (steers[1] - steers[0]) / span, g_ideal


def check_disp(mcap: Path, *, limit: int = 300) -> dict | None:
    """記録の Pi の `/stereo/disp` と、同じ組を Mac で照合し直した値の差。`/stereo/disp` が無ければ None。"""
    from mcap.reader import make_reader

    pi: dict[int, dict] = {}
    with open(mcap, "rb") as f:
        for _s, _ch, m in make_reader(f).iter_messages(topics=["/stereo/disp"]):
            pi[int(m.log_time)] = json.loads(m.data)
    if not pi:
        return None
    from raspi.core.stereo_match import MatchParams
    from tools.stereo_bench.data import load_stereo_info

    first = next(iter(pi.values()))
    # 照合の方式・窓は `/stereo/info` に残っている（無ければ今の既定）。`uniquenessRatio` は残っていないので今の既定
    info = load_stereo_info(mcap)
    m = MatchParams()
    settings = {"algo": info[-1]["algo"] if info else m.algo, "scale": first["scale"], "num_disp": first["num_disp"],
                "block": int(info[-1]["block"]) if info else m.block, "uniqueness": m.uniqueness,
                "rows": first["rows"], "disp_hw": [first["h"], first["w"]]}
    proc = make_processor(vehicle_for(mcap)[0], settings)
    d0, d1, n = [], [], 0
    for t, jpg, jpg_left in R.iter_image_pairs(mcap):
        msg = pi.get(int(t))
        if msg is None:
            continue
        a_pi = disp_from_bytes(base64.b64decode(msg["data"]), msg["h"], msg["w"])
        a_mac, _ = disparity_for(proc, jpg, jpg_left, (msg["h"], msg["w"]))
        both = (a_pi[1] > 0.5) & (a_mac[1] > 0.5)
        if both.any():
            d0.append(float(np.abs(a_pi[0] - a_mac[0])[both].mean() * msg["num_disp"]))
        d1.append(float(np.abs(a_pi[1] - a_mac[1]).mean()))
        n += 1
        if n >= limit:
            break
    return {"pairs": n, "disp_px": float(np.mean(d0)) if d0 else float("nan"),
            "valid": float(np.mean(d1)) if d1 else float("nan")}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mcap_files", nargs="+", type=Path)
    ap.add_argument("--model", type=Path, default=None)
    ap.add_argument("--data", type=Path, default=None, help="`extract.py` の出力（経路の誤差を出す）")
    ap.add_argument("--max-speed", type=float, default=5.0, help="planner の最高速度（既定 5 = 抑えない）")
    ap.add_argument("--yaw-every", type=int, default=10, help="何組に1回 dδ/dψ を測るか（0 で測らない）")
    ap.add_argument("--check-disp", action="store_true", help="Pi の視差と Mac で作り直した視差を比べるだけ")
    args = ap.parse_args()

    if args.check_disp:
        for p in args.mcap_files:
            r = check_disp(p)
            if r is None:
                print(f"{p.stem}: /stereo/disp が無い（このモードを入れる前の記録）")
            else:
                print(f"{p.stem}: {r['pairs']}組  視差の差 平均 {r['disp_px']:.2f}px  照合の出た割合の差 平均 {r['valid']:.3f}")
        return 0
    if args.model is None:
        print("--model が要る", file=sys.stderr)
        return 2

    vehicle = Vehicle.load()
    model = load_model(args.model)
    node = StereoE2ENode(model=model, model_name=args.model.stem)
    params = merged_params(StereoE2E.id, {"max_speed": args.max_speed})
    cfg = json.loads(args.model.with_suffix(".json").read_text())
    trained = set(cfg.get("train_records", []))
    band = (float(cfg["disp_col_f"]), float(cfg["disp_col_cx"])) if "disp_col_f" in cfg else None

    print_header("組")
    for p in args.mcap_files:
        r = replay(p, node, vehicle, params=params, band=band, labels=load_labels(args.data, p.stem),
                   yaw_every=args.yaw_every if band is not None else 0)
        print_row(p.stem, r, trained)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
