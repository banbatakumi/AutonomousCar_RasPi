"""`.mcap` の左右画像から視差を作り直し、Foxglove の 3D パネルで見られる点群を足す（Mac 側）。

    .venv/bin/python -m tools.mcap_stereo_cloud logs/run.mcap              # logs/run_stereo.mcap
    .venv/bin/python -m tools.mcap_stereo_cloud logs/run.mcap --every 2 --stride 3

元の記録を丸ごと写し、`/viz/stereo/points`（`foxglove.PointCloud`・frame=`base_link`）を足した別ファイルを作る。
`/tf`（odom→base_link）も写るので、Foxglove で `odom` を表示フレームにすれば世界に並ぶ。

## 何をしているか

- 視差の生データは記録に無い（`/stereo/disp` は12×68のブロック平均、`/stereo/depth` は色付き JPEG）。
  記録された左右の JPEG の組（`/viz/image/front`・`/viz/image/left`）から、Pi の `stereo_node` と同じ
  平行化・照合（`StereoRectifier`・`MatchParams`）をここで回し直す。
- 校正値は**記録に入っていた `provenance.vehicle_toml`**（撮った時の値）を使う。無ければ `config/vehicle.toml`。
- 点は視差の出た画素だけ（左端の探索幅ぶんは落とす）。色は主カメラ（右）の輝度。
- **Pi の照合と完全には一致しない**: 入力が JPEG（品質 90）で、記録は `--image-hz`（既定 5Hz）の間引き。
  同期していない時期の画像（左右の差が 0.5ms 超）は組にならないので点群が出ない。
"""

from __future__ import annotations

import argparse
import base64
import json
import sys
from collections.abc import Iterator
from pathlib import Path

import numpy as np

__all__ = ["CLOUD_TOPIC", "add_stereo_cloud", "default_out_path", "pointcloud_message", "points_from_disparity"]

CLOUD_TOPIC = "/viz/stereo/points"
FRAME_ID = "base_link"

#: 1点 = x y z（float32）+ red green blue alpha（uint8）= 16 バイト。
#: Foxglove は `red/green/blue/alpha` を「RGBA（別フィールド）」として色に使う
_POINT_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                         ("red", "u1"), ("green", "u1"), ("blue", "u1"), ("alpha", "u1")])
_FIELDS = [{"name": n, "offset": o, "type": t} for n, o, t in (
    ("x", 0, 7), ("y", 4, 7), ("z", 8, 7), ("red", 12, 1), ("green", 13, 1), ("blue", 14, 1), ("alpha", 15, 1))]


def default_out_path(src: Path, out_dir: Path | None = None) -> Path:
    return (out_dir or src.parent) / f"{src.stem}_stereo.mcap"


def points_from_disparity(rect, d: np.ndarray, rr: np.ndarray, cam_pose, *, scale: float, skip_cols: int,
                          stride: int = 2, max_range: float = 10.0) -> np.ndarray:
    """視差 `d`（帯・無効は 0 以下）→ base_link の点と灰色の輝度。`_POINT_DTYPE` の配列。

    :param rr: 平行化した主カメラ（右）の輝度。点が主カメラの画素 `u - d` に当たるので、色はそこから取る
    :param stride: 縦横それぞれ何画素おきに使うか（2 で約 1/4 の点数）
    :param max_range: これより遠い点は落とす [m]（遠いほど視差が小さく誤差が大きい）
    """
    d = np.where(np.arange(d.shape[1])[None, :] < skip_cols, -1.0, d)
    sub = d[::stride, ::stride]
    vi, ui = np.nonzero(sub > 0)
    v, u = vi * stride, ui * stride
    dd = d[v, u]
    pts = rect.points_from_pixels(u, v, dd, scale=scale)
    keep = pts[:, 2] <= max_range
    pts, v, u, dd = pts[keep], v[keep], u[keep], dd[keep]
    base = rect.rect_to_base(pts, *cam_pose)
    h, w = rr.shape
    rv = np.clip(np.rint((v + 0.5) / scale - 0.5), 0, h - 1).astype(int)
    ru = np.clip(np.rint((u + 0.5) / scale - 0.5 - dd / scale), 0, w - 1).astype(int)
    out = np.empty(len(base), _POINT_DTYPE)
    out["x"], out["y"], out["z"] = base[:, 0], base[:, 1], base[:, 2]
    gray = rr[rv, ru]
    out["red"] = out["green"] = out["blue"] = gray
    out["alpha"] = 255
    return out


def pointcloud_message(points: np.ndarray, unix_ns: int) -> dict:
    """`foxglove.PointCloud`（JSON）。"""
    return {
        "timestamp": {"sec": unix_ns // 1_000_000_000, "nsec": unix_ns % 1_000_000_000},
        "frame_id": FRAME_ID,
        "pose": {"position": {"x": 0.0, "y": 0.0, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
        "point_stride": _POINT_DTYPE.itemsize,
        "fields": _FIELDS,
        "data": base64.b64encode(points.tobytes()).decode("ascii"),
    }


def _vehicle(src: Path, override: Path | None):
    """校正値: 指定 > 記録の `provenance.vehicle_toml` > `config/vehicle.toml`。"""
    from raspi.core.vehicle import Vehicle
    from tools.stereo_bench.data import load_run_info

    if override is not None:
        return Vehicle.load(override), f"指定 {override}"
    recorded = Vehicle.from_provenance(load_run_info(src))
    if recorded is not None:
        return recorded, "記録の provenance"
    return Vehicle.load(), "config/vehicle.toml（記録に校正値の控えが無い）"


def _clouds(frames, proc, *, stride: int, max_range: float) -> Iterator[tuple[int, dict]]:
    """組ごとに `(log_time, PointCloud)`。時刻順。"""
    from raspi.core.pixfmt import to_gray
    from raspi.core.stereo_match import disparity

    for fr in frames:
        r, l_ = fr.decode()
        if r is None or l_ is None:
            continue                                   # 壊れた1枚で全体を止めない
        rr, rl = proc.rect(to_gray(r, "BGR888"), to_gray(l_, "BGR888"))
        d = disparity(proc.match, proc.matcher, rl, rr)
        pts = points_from_disparity(proc.rect, d, rr, proc.cam_pose, scale=proc.match.scale,
                                    skip_cols=proc.match.num_disp, stride=stride, max_range=max_range)
        yield fr.t_ns + max(fr.dt_ns, 0), pointcloud_message(pts, fr.t_ns)


def add_stereo_cloud(src: Path, out: Path, *, every: int = 1, stride: int = 2, max_range: float = 10.0,
                     vehicle_toml: Path | None = None, progress=None) -> tuple[int, str]:
    """`src` を写して点群を足した `out` を作る。`(点群の数, 校正値の出所)`。"""
    from mcap.reader import make_reader
    from mcap.writer import Writer

    from raspi.nodes.stereo_node import StereoProcessor
    from raspi.rec.mcap_log import FOXGLOVE_POINTCLOUD_SCHEMA
    from tools.stereo_bench.data import load_frames

    vehicle, origin = _vehicle(src, vehicle_toml)
    proc = StereoProcessor(vehicle)
    frames = load_frames(src, stride=every)
    clouds = _clouds(frames, proc, stride=stride, max_range=max_range)
    nxt = next(clouds, None)
    n = 0

    with open(src, "rb") as fin, open(out, "wb") as fout:
        reader = make_reader(fin)
        w = Writer(fout)
        w.start()
        schemas: dict[int, int] = {}
        channels: dict[int, int] = {}
        cloud_ch = w.register_channel(CLOUD_TOPIC, "json", w.register_schema(
            "foxglove.PointCloud", "jsonschema", json.dumps(FOXGLOVE_POINTCLOUD_SCHEMA).encode()))
        seq = 0

        def write_cloud(t: int, msg: dict) -> None:
            nonlocal seq, n
            seq += 1
            n += 1
            w.add_message(cloud_ch, t, json.dumps(msg).encode(), t, seq)
            if progress is not None:
                progress(n / max(1, len(frames)))

        for schema, ch, m in reader.iter_messages(log_time_order=True):
            if ch.topic == CLOUD_TOPIC:
                continue                               # 作り直し。前回の点群は写さない
            while nxt is not None and nxt[0] <= m.log_time:
                write_cloud(*nxt)
                nxt = next(clouds, None)
            if ch.id not in channels:
                sid = 0
                if schema is not None:
                    if schema.id not in schemas:
                        schemas[schema.id] = w.register_schema(schema.name, schema.encoding, schema.data)
                    sid = schemas[schema.id]
                channels[ch.id] = w.register_channel(ch.topic, ch.message_encoding, sid, dict(ch.metadata))
            w.add_message(channels[ch.id], m.log_time, m.data, m.publish_time, m.sequence)
        while nxt is not None:
            write_cloud(*nxt)
            nxt = next(clouds, None)
        for rec in reader.iter_metadata():
            w.add_metadata(rec.name, dict(rec.metadata))
        w.finish()
    return n, origin


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=".mcap の左右画像から視差を作り直し、Foxglove 用の点群を足す")
    ap.add_argument("mcap", nargs="+", type=Path)
    ap.add_argument("--out-dir", type=Path, default=None, help="出力先（省略時は入力と同じフォルダ）")
    ap.add_argument("--every", type=int, default=1, help="左右の組を何組おきに使うか（既定 1 = 全部）")
    ap.add_argument("--stride", type=int, default=2, help="画素を縦横何画素おきに点にするか（既定 2）")
    ap.add_argument("--max-range", type=float, default=10.0, help="これより遠い点を落とす [m]（既定 10）")
    ap.add_argument("--vehicle", type=Path, default=None, help="校正値の vehicle.toml（省略時は記録の控え）")
    args = ap.parse_args(argv)
    if args.every < 1 or args.stride < 1:
        ap.error("--every と --stride は 1 以上")
    for src in args.mcap:
        out = default_out_path(src, args.out_dir)
        if out.resolve() == src.resolve():
            print(f"{src.name}: 出力先が入力と同じです", file=sys.stderr)
            return 1
        n, origin = add_stereo_cloud(src, out, every=args.every, stride=args.stride,
                                     max_range=args.max_range, vehicle_toml=args.vehicle)
        print(f"{out}  点群 {n} 枚（校正値: {origin}）")
        if n == 0:
            print(f"{src.name}: 左右の組がありません（同期していない・--image-hz 0 で記録した？）", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
