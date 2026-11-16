"""ml_stereo_e2e/extract.py — 記録（`.mcap`）から学習用の標本を作る（左右の画像＋視差の帯＋車両状態 → 経路点）。

    .venv/bin/python -m ml_stereo_e2e.extract ~/Downloads/20261115/*.mcap --out ml_stereo_e2e/runs/v1/data

1. **経路のラベルと画像の組は `ml_e2e.extract` をそのまま使う**（`slam2d_route` の最適ライン、無ければその後に
   通った軌跡。LiDAR は記録のときの先生にだけ使う）。出力の並び（`frames/`・`<記録名>.npz`・`dataset.json`）も同じ
2. その組（JPEG）を Pi の `stereo_node` と同じ `StereoProcessor` で照合し直し、`stereo_e2e_preproc.disp_band_input`
   を通した視差の帯を npz の `disp`（`(N, 2, Hd, Wd)` float16）に足す。照合できなかった標本は `disp_ok=False`
   （学習で使わない）

校正値は**記録したときの** `vehicle.toml`（記録のメタデータ `provenance`）。無い古い記録は今の `vehicle.toml`。
学習が使う校正の値（視差の帯の焦点距離・主点、左右のカメラの fx、`max_steer`・`steer_link`）も、その校正から
求めて `dataset.json` の記録ごとの `calib_values` に残す。**学習は今の `vehicle.toml` を見ない**（記録の後で
校正を直しても、視差と画像の拡張・正規化が食い違わない。レビュー M10）。
照合の設定は既定で Pi の `stereo_node` と同じ（`MatchParams()`・行の帯 `DEFAULT_ROWS`）。設定は `dataset.json` の
`stereo` に書き、`train.py` がモデルの契約へ写す。Pi の `stereo/disp` がこれと違う設定なら推論しない。

**画像は JPEG（記録の品質 90）から照合する**ので、Pi が生の画像から作る視差とわずかに違う。違いの大きさは
`eval.py --check-disp`（記録に `/stereo/disp` がある記録で、同じ組の Pi の値と比べる）で見る。

「このモデル自身の走行」（`auto:stereo_e2e`）と、E2E 融合の走行（`auto:e2e_fusion`）は手本にしない（`--include-self` で使う）。
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # repo root

from ml_e2e import extract as E  # noqa: E402
from raspi.core.stereo_e2e_preproc import DEFAULT_DISP_HW, DISP_VERSION, band_geometry, disp_band_input  # noqa: E402
from raspi.core.stereo_match import MatchParams  # noqa: E402

__all__ = ["SELF_SOURCES", "stereo_settings", "make_processor", "disparity_for", "calib_values", "extract_recording",
           "main"]

#: 手本にしない（このモデル自身・LiDAR を使う E2E 融合の）走行の指令の `source`
SELF_SOURCES = ("auto:stereo_e2e", "auto:e2e_fusion")


def stereo_settings(match: MatchParams, rows: tuple[int, int], disp_hw: tuple[int, int]) -> dict:
    """`dataset.json` の `stereo`（照合と視差の帯の設定。モデルの契約に写す）。"""
    return {"algo": match.algo, "scale": match.scale, "num_disp": match.num_disp, "block": match.block,
            "uniqueness": match.uniqueness, "rows": [int(rows[0]), int(rows[1])],
            "disp_hw": [int(disp_hw[0]), int(disp_hw[1])], "disp_version": DISP_VERSION}


def vehicle_for(path: Path):
    """記録したときの `Vehicle`（メタデータ `provenance.vehicle_toml`）。無ければ今の `vehicle.toml`。`(Vehicle, 出どころ)`。"""
    import tomllib

    from raspi.core.vehicle import Vehicle
    from tools.stereo_bench.data import load_run_info

    toml = load_run_info(path).get("provenance", {}).get("vehicle_toml")
    if toml:
        return Vehicle._from_dict(tomllib.loads(toml), Path("<provenance>")), "記録"
    return Vehicle.load(), "今の vehicle.toml"


def make_processor(vehicle, settings: dict):
    """`dataset.json` の `stereo` の設定で、Pi と同じ `StereoProcessor` を作る。"""
    from raspi.nodes.stereo_node import StereoProcessor

    match = MatchParams(settings["algo"], float(settings["scale"]), int(settings["num_disp"]), int(settings["block"]),
                        int(settings["uniqueness"]))
    return StereoProcessor(vehicle, match=match, rows=tuple(settings["rows"]))


def disparity_for(proc, jpg_right: bytes, jpg_left: bytes, disp_hw) -> tuple[np.ndarray, float]:
    """JPEG の組 → `(視差の帯 (2, Hd, Wd) float32, 視差の出た割合)`。値は Pi と同じく float16 を通す。"""
    import cv2

    r = cv2.imdecode(np.frombuffer(jpg_right, np.uint8), cv2.IMREAD_COLOR)
    l_ = cv2.imdecode(np.frombuffer(jpg_left, np.uint8), cv2.IMREAD_COLOR)
    if r is None or l_ is None:
        raise ValueError("JPEG を読めない")
    _st, d = proc.process(r, l_, "BGR888")
    arr = disp_band_input(d, proc.match.num_disp, tuple(disp_hw))
    return arr.astype(np.float16).astype(np.float32), float(proc.last_valid_frac)


def calib_values(proc, vehicle, settings: dict, image_w: int) -> dict:
    """学習が使う校正の値（記録の校正から。`train.py` は今の `vehicle.toml` を見ずにこれを使う）。

    - `band_f`・`band_cx`: 視差の帯の列で測った焦点距離・主点（`stereo_e2e_preproc.band_geometry`。向きの拡張）
    - `fx_px`・`fx_px_left`: 保存した画像（幅 `image_w`）の左右のカメラの水平焦点距離（画像の向きの拡張）
    - `max_steer`・`steer_link`: 車両状態の正規化と、その舵角の意味（ステアのリンクの換算）
    """
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ml_cam_e2e"))
    from yaw_aug import camera_fx_px  # noqa: PLC0415

    f_b, cx_b = band_geometry(proc.rect.f, proc.rect.cx, float(settings["scale"]), int(settings["num_disp"]),
                              int(settings["disp_hw"][1]))
    return {"band_f": f_b, "band_cx": cx_b, "image_w": int(image_w),
            "fx_px": camera_fx_px(vehicle, "front", image_w), "fx_px_left": camera_fx_px(vehicle, "left", image_w),
            "max_steer": float(vehicle.max_steer), "steer_link": [float(x) for x in vehicle.steer_link]}


def extract_recording(path: Path, out_dir: Path, cfg: E.ExtractConfig, settings: dict, *,
                      rejects: Counter | None = None) -> dict:
    """記録1本を標本にして `<out_dir>/<記録名>.npz` に書く（`ml_e2e.extract` ＋視差）。内訳の dict を返す。"""
    rejects = rejects if rejects is not None else Counter()
    info = E.extract_recording(path, out_dir, cfg, rejects=rejects)
    if not info.get("samples"):
        return info
    vehicle, src = vehicle_for(path)
    info["calib"] = src
    try:
        proc = make_processor(vehicle, settings)
    except ValueError as e:
        info["skipped"] = f"ステレオの校正値が無い（{e}）"
        info["samples"] = 0
        (out_dir / f"{Path(path).stem}.npz").unlink(missing_ok=True)
        return info
    npz = out_dir / f"{Path(path).stem}.npz"
    d = dict(np.load(npz))
    n = len(d["t"])
    hw = tuple(settings["disp_hw"])
    disp = np.zeros((n, 2, *hw), dtype=np.float16)
    ok = np.zeros(n, dtype=bool)
    valid = []
    image_w = 0
    for i in range(n):
        try:
            arr, vf = disparity_for(proc, (out_dir / str(d["image"][i])).read_bytes(),
                                    (out_dir / str(d["image_left"][i])).read_bytes(), hw)
        except Exception as e:                                # noqa: BLE001
            rejects[f"照合できない（{type(e).__name__}）"] += 1
            continue
        disp[i] = arr
        ok[i] = True
        valid.append(vf)
        if not image_w:
            import cv2
            img = cv2.imdecode(np.frombuffer((out_dir / str(d["image"][i])).read_bytes(), np.uint8), cv2.IMREAD_COLOR)
            image_w = int(img.shape[1]) if img is not None else 0
    d["disp"] = disp
    d["disp_ok"] = ok
    np.savez_compressed(npz, **d)
    info["disp_ok"] = int(ok.sum())
    info["valid_frac"] = round(float(np.mean(valid)), 3) if valid else None
    if image_w:
        info["calib_values"] = calib_values(proc, vehicle, settings, image_w)
    return info


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mcap_files", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--k", type=int, default=40, help="経路点の数（既定 40）")
    ap.add_argument("--ds", type=float, default=0.25, help="経路点の間隔 [m]（既定 0.25）")
    ap.add_argument("--include-self", action="store_true", help="このモデル自身・E2E 融合の走行も手本に使う")
    ap.add_argument("--algo", choices=("bm", "sgbm"), default=MatchParams.algo)
    ap.add_argument("--scale", type=float, default=MatchParams.scale)
    ap.add_argument("--num-disp", type=int, default=MatchParams.num_disp)
    ap.add_argument("--block", type=int, default=MatchParams.block)
    ap.add_argument("--uniqueness", type=int, default=MatchParams.uniqueness)
    ap.add_argument("--rows", default=None, help="照合する平行化画像の行 V0:V1（既定は stereo_node と同じ）")
    ap.add_argument("--disp-hw", default=f"{DEFAULT_DISP_HW[0]}x{DEFAULT_DISP_HW[1]}", help="視差の帯を縮めた 高さx幅")
    args = ap.parse_args()

    from raspi.nodes.stereo_node import DEFAULT_ROWS

    rows = tuple(int(x) for x in args.rows.split(":")) if args.rows else DEFAULT_ROWS
    hw = tuple(int(x) for x in args.disp_hw.lower().split("x"))
    settings = stereo_settings(MatchParams(args.algo, args.scale, args.num_disp, args.block, args.uniqueness),
                               rows, hw)
    cfg = E.ExtractConfig(k=args.k, ds=args.ds, include_self=args.include_self, self_sources=SELF_SOURCES)
    args.out.mkdir(parents=True, exist_ok=True)
    meta_path = args.out / "dataset.json"
    meta = (json.loads(meta_path.read_text()) if meta_path.exists()
            else {"config": cfg.to_json(), "stereo": settings, "records": {}})
    if meta["config"] != cfg.to_json() or meta.get("stereo") != settings:
        print(f"{meta_path} の設定が今回と違います。別の --out へ抽出してください", file=sys.stderr)
        return 2

    rejects: Counter = Counter()
    total = 0
    for p in args.mcap_files:
        if not p.exists():
            print(f"# skip: {p}（見つからない）", file=sys.stderr)
            continue
        try:
            info = extract_recording(p, args.out, cfg, settings, rejects=rejects)
        except Exception as e:                              # noqa: BLE001
            print(f"# skip: {p.name}（読めない: {type(e).__name__}: {e}）。壊れていれば "
                  f"`python -m raspi.tools.mcap_repair {p} --inplace`", file=sys.stderr)
            continue
        meta["records"][p.stem] = info
        if info.get("skipped"):
            print(f"# {p.name}: 使わない（{info['skipped']}）", flush=True)
            continue
        total += info.get("disp_ok", 0)
        print(f"# {p.name}: {info.get('disp_ok', 0)}/{info['samples']}枚（ライン {info.get('line', 0)}・"
              f"軌跡 {info.get('future', 0)}／視差の出た割合 {info.get('valid_frac')}／校正は{info.get('calib')}）",
              flush=True)
        meta_path.write_text(json.dumps(meta, ensure_ascii=False, indent=1))
    meta_path.write_text(json.dumps(meta, ensure_ascii=False, indent=1))
    print(f"# 合計 {total}枚 → {args.out}")
    if rejects:
        print("# 使わなかったコマ: " + "・".join(f"{r} {n}枚" for r, n in rejects.most_common()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
