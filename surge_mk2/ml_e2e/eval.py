"""ml_e2e/eval.py — 記録を、実車と同じ推論ノードと planner に通して、手本と比べる。

    .venv/bin/python -m ml_e2e.eval --model models/e2e_fusion/v1.onnx ~/Downloads/20261108/surge_20261108_023840.mcap
    .venv/bin/python -m ml_e2e.eval --model models/e2e_fusion/v1.onnx --data ml_e2e/runs/v1/data <mcap...>

**学習に使っていない記録で見ること**（`train.py` の「検証」に出た記録。同じ記録で見ても実力は分からない）。

記録の `vehicle_state`・`scan`・画像を時刻順に流し、`raspi/nodes/e2e_fusion_node.E2EFusionNode.infer_once`
（ONNX・前処理とも実車と同じコード）→ `raspi/auto/e2e_fusion.E2EFusion.plan`（推測航法で経路をずらして追う）
まで通す。出すのは:

| 列 | 中身 |
|---|---|
| 走れる | `ready=True` だった割合（残りは理由ごとに数える） |
| 舵の相関・RMS | planner が出した舵と、記録の指令の舵（同じ時刻） |
| 速度の比 | planner の目標速度 ÷ 記録の指令の速度（中央値）。1 より小さければ手本より遅い |
| 経路の誤差 | `--data` を渡したとき。経路点とラベルの距離（最初の 2m／全体） |
| dδ/dψ・理想比 | 車の向きを ψ ずらした入力にしたとき、経路を追う舵がどれだけ戻るか [rad/rad]（負で大きいほど戻す）と、「モデルが経路を正しく −ψ 回した場合」の値に対する割合。1 に近いほど、ずれを入力から読み取れている |

**これは開ループの比較。** 車は記録のとおりに動き、モデルの舵で動いた先の絵は見ていない。走れるかどうかは
実走でしか分からない。ここで分かるのは、配線・時刻・契約が合っていることと、ずれを戻す向きの舵が出ていること。

`--no-image`・`--no-lidar` で片方の入力を伏せた評価ができる。
"""

from __future__ import annotations

import argparse
import io
import math
import sys
from collections import Counter
from pathlib import Path

import numpy as np
from PIL import Image

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))
sys.path.insert(0, str(_ROOT / "ml_cam_e2e"))

from yaw_aug import front_fx_px, yawed_window  # noqa: E402

from ml_e2e import labels as L  # noqa: E402
from ml_e2e import records as R  # noqa: E402
from raspi.auto.e2e_fusion import E2EFusion  # noqa: E402
from raspi.auto.registry import merged_params  # noqa: E402
from raspi.core.cam_e2e_preproc import crop_margin_px  # noqa: E402
from raspi.core.vehicle import Vehicle  # noqa: E402
from raspi.msgs import Scan, VehicleState  # noqa: E402
from raspi.nodes.e2e_fusion_node import E2EFusionNode, load_model  # noqa: E402

__all__ = ["replay", "yaw_response", "main"]

NS = 1_000_000_000
#: 点群の1周が届くのは、先頭セクタの時刻からこれだけ後 [s]（1周 0.1s ＋ 通信・組み立て）
_SCAN_ARRIVE_S = 0.11
#: 画像が撮影から届くまで [s]
_IMAGE_ARRIVE_S = 0.02


def _vs(rec: R.Recording, i: int) -> VehicleState:
    return VehicleState(t_capture=int(rec.vs_t[i]), speed=float(rec.vs_speed[i]),
                        yaw_rate=float(rec.vs_yaw_rate[i]), steer_actual=float(rec.vs_steer[i]),
                        stopped=bool(abs(rec.vs_speed[i]) < 0.02))


def _scan(rec: R.Recording, i: int, seq: int) -> Scan:
    sat = rec.scan_sat[i]
    return Scan(t_capture=int(rec.scan_t[i]), seq=seq, dist=rec.scan_dist[i].tolist(),
                sector_seen=rec.scan_seen[i].tolist(), saturated=sat.tolist() if sat.any() else None)


def replay(mcap: Path, node: E2EFusionNode, vehicle: Vehicle, *, params: dict[str, float],
           use_image: bool = True, use_lidar: bool = True, labels: dict | None = None,
           yaw_every: int = 0) -> dict:
    """記録1本を推論ノードと planner に通す。数字の dict を返す。"""
    rec = R.load(mcap)
    images = list(R.iter_images(mcap)) if use_image else []
    img_t = np.array([t for t, _ in images], dtype=np.int64)
    planner = E2EFusion()
    node._rows.clear()
    model = node.model
    label_idx = {int(t): i for i, t in enumerate(labels["t"])} if labels is not None else {}
    stride = max(1, int(round(model.ds / float(labels["ds"])))) if labels is not None else 1

    out = {"n": 0, "ready": 0, "reasons": Counter(), "steer": [], "steer_ref": [],
           "speed": [], "speed_ref": [], "near": [], "full": [], "b": [], "infer_ms": [], "dpsi": [], "dpsi_ratio": []}
    vi = 0
    last_plan_t = 0
    for si in range(len(rec.scan_t)):
        t_now = int(rec.scan_t[si]) + int(_SCAN_ARRIVE_S * NS)
        # その時刻までの車両状態を planner へ（推測航法）
        while vi < len(rec.vs_t) and rec.vs_t[vi] <= t_now:
            planner.on_vehicle_state(_vs(rec, vi))
            vi += 1
        if vi == 0:
            continue
        vs = _vs(rec, vi - 1)
        if use_lidar:
            node.push_scan(_scan(rec, si, si))
        ci = int(np.searchsorted(rec.cmd_t, t_now))
        ci = min(max(ci, 0), len(rec.cmd_t) - 1)
        driving = bool(rec.cmd_arm[ci]) and rec.cmd_mode[ci] in (1, 2) and vs.speed >= 0.15 \
            and not rec.cmd_torque_mode[ci]
        if not driving:
            continue                               # 止まっている・DISARM の周は比べない

        rgb, t_img = None, 0
        if use_image and len(img_t):
            j = int(np.searchsorted(img_t, t_now - int(_IMAGE_ARRIVE_S * NS), side="right")) - 1
            if j >= 0 and t_now - int(img_t[j]) < 0.2 * NS:
                t_img = int(img_t[j])
                rgb = np.asarray(Image.open(io.BytesIO(images[j][1])).convert("RGB"))
        plan = node.infer_once(rgb=rgb, t_image_ns=t_img, vs=vs, now_ns=t_now, seq=si)
        dt = (t_now - last_plan_t) / NS if last_plan_t else 0.0
        last_plan_t = t_now
        st = planner.plan(plan, vs, params, dt)
        out["n"] += 1
        if plan.ready:
            out["infer_ms"].append(plan.infer_ms)
        if not st.ready:
            out["reasons"][st.reason.split("（")[0]] += 1
            continue
        out["ready"] += 1
        out["steer"].append(st.target_steer)
        out["steer_ref"].append(float(rec.cmd_steer[ci]))
        if rec.cmd_mode[ci] == 2 and not rec.cmd_brake[ci] and rec.cmd_speed[ci] > 0.2:
            out["speed"].append(st.target_speed)
            out["speed_ref"].append(float(rec.cmd_speed[ci]))
        out["b"].append(float(np.mean(plan.path_b)))

        li = label_idx.get(t_img)
        if li is not None:
            sel = slice(stride - 1, stride * model.k, stride)
            ref = labels["path_xy"][li][sel]
            ok = labels["path_valid"][li][sel]
            d = np.hypot(np.asarray(plan.path_x) - ref[:, 0], np.asarray(plan.path_y) - ref[:, 1])
            near = max(1, int(2.0 / model.ds))
            if ok[:near].any():
                out["near"].append(float(d[:near][ok[:near]].mean()))
            if ok.any():
                out["full"].append(float(d[ok].mean()))

        if yaw_every and rgb is not None and out["ready"] % yaw_every == 0 and rec.scan_seen[si].all():
            g = yaw_response(node, vehicle, rgb, rec.scan_dist[max(si - model.history + 1, 0):si + 1], vs,
                             params, ref_xy=np.column_stack([plan.path_x, plan.path_y]))
            if g is not None and abs(g[1]) > 1e-3:
                out["dpsi"].append(g[0])
                out["dpsi_ratio"].append(g[0] / g[1])
    return out


def yaw_response(node: E2EFusionNode, vehicle: Vehicle, rgb: np.ndarray, dists: np.ndarray,
                 vs: VehicleState, params: dict[str, float], psi_deg: int = 4,
                 ref_xy: np.ndarray | None = None) -> tuple[float, float] | None:
    """車の向きを ±`psi_deg` ずらした入力での、経路を追う舵の変化 dδ/dψ [rad/rad]。

    画像は窓を横へずらし（`yaw_aug.yawed_window`、車体は動かさないマスク無し）、点群は角度をずらす。
    舵は経路点を Pure Pursuit で追ったもの（`labels.pursuit_steer`、planner と同じ注視距離）。
    戻り値は `(実際の値, 理想の値)`。理想は、ずらさない入力で出た経路 `ref_xy` をそのまま ∓ψ 回したときの値
    （注視距離が長いほど小さい。高速では −0.2 程度にしかならないので、実際の値だけでは良し悪しが分からない）。
    """
    model = node.model
    fx = front_fx_px(vehicle, rgb.shape[1])
    margin = crop_margin_px(rgb.shape[1], model.crop_x)
    if math.ceil(fx * math.radians(psi_deg)) > margin:
        return None
    from raspi.auto.base import scan_window
    from raspi.core.e2e_fusion_preproc import lidar_input, lidar_rows

    look = params["look_k"] * vs.speed + params["look_min"]
    steers = []
    ideal = []
    for sgn in (-1, 1):
        psi = sgn * psi_deg
        # 左右の余白が要るので、切り落とす前の絵から窓をずらして作り、モデルには切り落とし済みとして渡す
        win = yawed_window(rgb, margin, math.radians(psi), fx, None)
        image = model.image_input(_uncrop(win, margin))
        rows = []
        for dist in dists[::-1]:                      # 新しい順
            # 車が左へ ψ 向くと、物は −ψ の方向に見える: 新しい角 θ の値は元の角 θ+ψ の値
            sc = Scan(dist=np.roll(dist, -psi).tolist(), sector_seen=[True] * 12)
            w = scan_window(sc, model.fov_deg, model.max_range)
            rows.append(lidar_rows(w.dist, w.measured, model.max_range))
        lidar = lidar_input(rows, model.history)
        o = model.infer(image, lidar, model.state_input(vs.speed, vs.steer_actual, vs.yaw_rate))
        steers.append(L.pursuit_steer(o.path_xy, look, vehicle.wheelbase, vehicle.max_steer))
        if ref_xy is not None:
            c, sn = math.cos(-math.radians(psi)), math.sin(-math.radians(psi))
            rot = np.stack([c * ref_xy[:, 0] - sn * ref_xy[:, 1], sn * ref_xy[:, 0] + c * ref_xy[:, 1]], axis=1)
            ideal.append(L.pursuit_steer(rot, look, vehicle.wheelbase, vehicle.max_steer))
    span = math.radians(2 * psi_deg)
    g_ideal = (ideal[1] - ideal[0]) / span if len(ideal) == 2 else float("nan")
    return (steers[1] - steers[0]) / span, g_ideal


def _uncrop(win: np.ndarray, margin: int) -> np.ndarray:
    """切り落とし済みの窓の左右に `margin` の余白を足す（`image_input` がもう一度切り落とす分）。"""
    return np.pad(win, ((0, 0), (margin, margin), (0, 0)), mode="edge")


def _corr(a, b) -> float:
    a, b = np.asarray(a), np.asarray(b)
    return float(np.corrcoef(a, b)[0, 1]) if len(a) > 2 and a.std() > 0 and b.std() > 0 else float("nan")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mcap_files", nargs="+", type=Path)
    ap.add_argument("--model", type=Path, required=True)
    ap.add_argument("--data", type=Path, default=None, help="`extract.py` の出力（経路の誤差を出す）")
    ap.add_argument("--no-image", action="store_true")
    ap.add_argument("--no-lidar", action="store_true")
    ap.add_argument("--max-speed", type=float, default=5.0, help="planner の最高速度（既定 5 = 抑えない）")
    ap.add_argument("--yaw-every", type=int, default=10, help="何周に1回 dδ/dψ を測るか（0 で測らない）")
    args = ap.parse_args()

    import json

    vehicle = Vehicle.load()
    node = E2EFusionNode(model=load_model(args.model), model_name=args.model.stem, vehicle=vehicle)
    params = merged_params(E2EFusion.id, {"max_speed": args.max_speed})
    ds_data = None
    if args.data is not None:
        ds_data = float(json.loads((args.data / "dataset.json").read_text())["config"]["ds"])
    cfg = json.loads(args.model.with_suffix(".json").read_text())
    trained = set(cfg.get("train_records", []))

    print(f"{'記録':30} {'学習':4} {'周':>5} {'走れる':>6} {'舵相関':>6} {'舵RMS':>6} {'速度比':>6} "
          f"{'近く':>6} {'全体':>6} {'b':>5} {'dδ/dψ':>6} {'理想比':>6} {'推論':>6}")
    for p in args.mcap_files:
        labels = None
        if args.data is not None and (args.data / f"{p.stem}.npz").exists():
            d = np.load(args.data / f"{p.stem}.npz")
            labels = {"t": d["t"], "path_xy": d["path_xy"], "path_valid": d["path_valid"], "ds": ds_data}
        r = replay(p, node, vehicle, params=params, use_image=not args.no_image, use_lidar=not args.no_lidar,
                   labels=labels, yaw_every=args.yaw_every)

        def med(key):
            return float(np.median(r[key])) if r[key] else float("nan")

        def mean(key):
            return float(np.mean(r[key])) if r[key] else float("nan")

        s, ref = np.asarray(r["steer"]), np.asarray(r["steer_ref"])
        rms = float(np.degrees(np.sqrt(np.mean((s - ref) ** 2)))) if len(s) else float("nan")
        ratio = float(np.median(np.asarray(r["speed"]) / np.asarray(r["speed_ref"]))) if r["speed"] else float("nan")
        print(f"{p.stem:30} {'○' if p.stem in trained else '—':4} {r['n']:5d} "
              f"{100.0 * r['ready'] / max(1, r['n']):5.1f}% {_corr(s, ref):6.3f} {rms:5.1f}° "
              f"{ratio:6.2f} {mean('near'):5.2f}m {mean('full'):5.2f}m {mean('b'):5.2f} {med('dpsi'):6.2f} {med('dpsi_ratio'):6.2f} "
              f"{mean('infer_ms'):5.1f}ms", flush=True)
        for why, n in r["reasons"].most_common(4):
            print(f"    走れない: {why} ×{n}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
