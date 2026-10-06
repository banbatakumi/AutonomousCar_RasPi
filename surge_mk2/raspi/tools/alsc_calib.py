"""色むら補正（ALSC）の表を、白い壁の RAW から作る。レンズを替えたときに使う。

    # Pi の上で。カメラを掴むので先に配信を止める
    sudo systemctl stop surge-camera
    .venv/bin/python -m raspi.tools.alsc_calib --cam 0 --rows 0,0.54 \
        --out config/cam_tuning/imx219_wide160.json
    sudo systemctl start surge-camera

    # 撮るだけ撮って Mac で作り直す
    .venv/bin/python -m raspi.tools.alsc_calib --cam 0 --save-raw /tmp/wall.npz
    python3 -m raspi.tools.alsc_calib --npz wall.npz --rows 0,0.54 --out config/cam_tuning/…json

撮り方: **白い壁（無地）にカメラを向け、壁が画面の中心と上の両隅まで入るようにする。**
明るさのむらは構わない（色の比しか見ない）が、**色の違う光が混ざらないこと**（窓の光と
電球など）。車体が写り込む行は `--rows 上,下`（画面の高さに対する割合）で外す。

作るもの: `rpi.alsc` の `calibrations_Cr`/`calibrations_Cb`（緑に対する赤・青の倍率、32×32）。
`raspi/core/cam_tuning.py` が標準のチューニングへ重ねる。使うレンズは `config/vehicle.toml` の
レンズのプロファイルの `isp_tuning` に書く。

**中心からの距離だけの関数として当てはめる**（r² の3次式）。理由は2つ:

- 車体が写り込んで壁が画面の一部しか占めなくても、全体の表が作れる
- 壁に当たる光の色の偏り（左右の傾き）をレンズのせいにして焼き込まない

当てはめは残差（`残差σ`）と半径の範囲を表示する。データの無い外側は、最も外の値で止める
（3次式を外へ伸ばすと暴れる。そこは多くがレンズの像の外）。

周辺減光（`luminance_lut`）は作らない——均一に照らした面が要り、白い壁では測れない。
光源の色温度ごとの表も1枚だけ（撮ったときの色温度）。残りは ALSC の適応補正に任せる。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

#: PiSP（Pi 5）の ALSC の格子。VC4（Pi 4 まで）は 16×12 で別物
GRID = 32
#: この割合より暗い画素は壁とみなさない（レンズの筒の影・暗い写り込み）。99% 点に対する比
MIN_BRIGHTNESS = 0.30
SATURATED = 60000                                             # 16bit に伸ばした RAW の飽和


def split_bayer(raw: np.ndarray, order: str, black: float):
    """16bit の Bayer 配列 → 半分の解像度の (R, G, B, 飽和マスク)。`order` は `BGGR` 等。"""
    planes = {(0, 0): raw[0::2, 0::2], (0, 1): raw[0::2, 1::2],
              (1, 0): raw[1::2, 0::2], (1, 1): raw[1::2, 1::2]}
    by_color: dict[str, list[np.ndarray]] = {"R": [], "G": [], "B": []}
    for (y, x), p in planes.items():
        by_color[order[y * 2 + x]].append(p.astype(np.float64))
    sat = np.maximum.reduce(list(planes.values())) > SATURATED
    r, g, b = (sum(by_color[c]) / len(by_color[c]) - black for c in "RGB")
    return r, g, b, sat


def _r2(h: int, w: int, aspect: float | None = None) -> np.ndarray:
    """各画素の、中心からの距離の2乗（隅で 1）。`aspect` は像の縦/横（省略時は h/w）。

    表の格子は 32×32 でも像は 4:3 なので、格子に対しては像の `aspect` を渡す。
    """
    aspect = h / w if aspect is None else aspect
    yy, xx = np.mgrid[0:h, 0:w]
    u = (xx + 0.5) / w - 0.5
    v = ((yy + 0.5) / h - 0.5) * aspect
    return (u * u + v * v) / (0.25 + 0.25 * aspect ** 2)


def _fit(r2: np.ndarray, ratio: np.ndarray) -> tuple[np.ndarray, float]:
    """`ratio ≈ c0 + c1·r² + c2·r⁴ + c3·r⁶`。外れ値（壁の汚れ・写り込み）を3回落とす。"""
    a = np.stack([np.ones_like(r2), r2, r2 ** 2, r2 ** 3], axis=1)
    c = np.linalg.lstsq(a, ratio, rcond=None)[0]
    for _ in range(3):
        res = ratio - a @ c
        keep = np.abs(res) <= 2.5 * res.std() + 1e-9          # +ε: 雑音の無いデータで全滅させない
        c = np.linalg.lstsq(a[keep], ratio[keep], rcond=None)[0]
    return c, float((ratio - a @ c)[keep].std())


def fit_tables(r: np.ndarray, g: np.ndarray, b: np.ndarray, sat: np.ndarray,
               rows: tuple[float, float] = (0.0, 1.0), grid: int = GRID) -> tuple[list, list, dict]:
    """白い壁の (R, G, B) から `calibrations_Cr`/`Cb` の表（`grid`×`grid`、最小値 1）を作る。"""
    h, w = g.shape
    yy = np.mgrid[0:h, 0:w][0]
    mask = (yy >= rows[0] * h) & (yy < rows[1] * h) & ~sat \
        & (g > MIN_BRIGHTNESS * np.percentile(g, 99)) & (r > 0) & (b > 0)
    if mask.sum() < 5000:
        raise ValueError(f"壁の画素が足りない（{int(mask.sum())}）。--rows と明るさを見直す")
    r2 = _r2(h, w)
    r2_max = float(np.percentile(r2[mask], 99.5))
    if r2[mask].min() > 0.02 or r2_max < 0.4:
        raise ValueError(f"壁が中心から周辺まで写っていない（r² {r2[mask].min():.2f}〜{r2_max:.2f}）")
    cell = np.minimum(_r2(grid, grid, aspect=h / w), r2_max)                 # 外側は最も外の値で止める
    tables, info = [], {"pixels": int(mask.sum()), "r2_max": round(r2_max, 3)}
    for name, ch in (("Cr", r), ("Cb", b)):
        c, sigma = _fit(r2[mask], g[mask] / ch[mask])
        t = c[0] + c[1] * cell + c[2] * cell ** 2 + c[3] * cell ** 3
        t = t / t.min()
        tables.append([round(float(v), 3) for v in t.ravel()])
        info[name] = {"residual_sigma": round(sigma / float(c[0]), 4),
                      "max_gain": round(float(t.max()), 3)}
    return tables[0], tables[1], info


def capture(idx: int, frames: int = 8) -> dict:
    """フル画角の RAW を `frames` 枚平均して返す（Pi の上だけ）。"""
    from picamera2 import Picamera2

    cam = Picamera2(idx)
    full = cam.camera_properties["PixelArraySize"]
    mode = min((m for m in cam.sensor_modes if tuple(m["crop_limits"][2:]) == tuple(full)),
               key=lambda m: m["size"][0])                     # フル画角で最も小さいモード
    cam.configure(cam.create_still_configuration(
        raw={"size": mode["size"], "format": str(mode["unpacked"])}, buffer_count=4))
    raw_cfg = cam.camera_configuration()["raw"]
    sensor = str(cam.camera_properties.get("Model", ""))
    cam.start()
    try:
        time.sleep(2.5)                                        # AE が落ち着くまで
        acc = None
        for _ in range(frames):
            req = cam.capture_request()
            a = req.make_array("raw").view(np.uint16).astype(np.float64)
            md = req.get_metadata()
            req.release()
            acc = a if acc is None else acc + a
    finally:
        cam.stop()
        cam.close()
    return {"raw": (acc / frames)[:, :raw_cfg["size"][0]].astype(np.float32),
            "order": str(raw_cfg["format"])[1:5],
            "black": float(md["SensorBlackLevels"][0]),
            "ct": int(md.get("ColourTemperature", 0)),
            "sensor": sensor}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cam", type=int, default=0, help="カメラ番号（0=前 1=後）")
    ap.add_argument("--npz", help="撮らずに、--save-raw で保存した RAW から作る")
    ap.add_argument("--save-raw", help="撮った RAW を保存する（.npz）")
    ap.add_argument("--rows", default="0,1", help="使う行の範囲（高さに対する割合。例 0,0.54）")
    ap.add_argument("--out", help="書き出す差分ファイル（config/cam_tuning/….json）")
    args = ap.parse_args()

    if args.npz:
        z = np.load(args.npz)
        shot = {k: (z[k] if k == "raw" else z[k].item()) for k in z.files}
    else:
        shot = capture(args.cam)
    if args.save_raw:
        np.savez_compressed(args.save_raw, **shot)
        print(f"# RAW を保存: {args.save_raw}")

    lo, hi = (float(v) for v in args.rows.split(","))
    r, g, b, sat = split_bayer(shot["raw"], shot["order"], shot["black"])
    cr, cb, info = fit_tables(r, g, b, sat, rows=(lo, hi))
    print(f"# {shot['sensor']} {shot['order']} 色温度 {shot['ct']}K  壁 {info['pixels']} 画素  "
          f"r² 〜{info['r2_max']}")
    for name in ("Cr", "Cb"):
        print(f"#   {name}: 最大倍率 {info[name]['max_gain']}  残差σ {info[name]['residual_sigma']:.1%}")
    if not args.out:
        return 0
    ct = shot["ct"] or 4000
    doc = {"_comment": "raspi/tools/alsc_calib.py が書いた。手で書き換えない",
           "sensor": shot["sensor"], "calibrated": time.strftime("%Y-%m-%d"), "fit": info,
           "rpi.alsc": {"calibrations_Cr": [{"ct": ct, "table": cr}],
                        "calibrations_Cb": [{"ct": ct, "table": cb}]}}
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(doc, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"# 書き出し: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
