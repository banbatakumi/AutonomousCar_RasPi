"""地図データの永続化 — 占有格子(3値)・中心線・レーシングラインをファイルに保存する。

**`slam2d`を一切importしない。** `OccGrid`の再構築は`raspi/auto/_slam2d_nav.py`
（`raspi/`側がslam2dを使う変換をここに閉じ込める、というそのファイルの既存方針）
に置き、ここは純粋なファイルI/O層にする。

## 1件 = `<name>.npz`（配列） + `<name>.json`（メタデータ）のペア

`AutoPanel.tsx`が読む `.onnx` + `<name>.json`（備考）と同じ形。大きい配列は
npzに、一覧表示に要る軽い情報だけjsonに分けることで、`list_maps()` がnpzを
1つも展開せずに済む。

## 生の hits/misses ではなく3値（trinary）だけを保存する

`OccGrid` は凍結後、`hits`/`misses` の生カウントではなく `wall_mask()` /
`known_free_mask()` から導かれる3値の判定結果でしか読まれない
（`slam2d_raceline` の RACE 段は常に凍結済み）。3値さえあれば判定を完全に
再現できるので、生カウント配列を保存する意味が無い（読み込み側は
`raspi/auto/_slam2d_nav.py::occgrid_from_trinary()` 参照）。

## `allow_pickle=False` は必須

アップロード経由で任意のバイト列が `.npz` として渡ってくる。pickle 復元による
任意コード実行を避けるため、保存・読み込みの両方で明示的に禁止する。
"""

from __future__ import annotations

import json
import os
import time
import zipfile
from pathlib import Path
from typing import BinaryIO, NamedTuple

import numpy as np

__all__ = ["MAPS_DIR", "LoadedMap", "resolve_map_path", "list_maps",
           "save_map", "load_map", "delete_map", "validate_upload", "save_upload"]

MAPS_DIR = Path(__file__).resolve().parents[2] / "saved_maps"

_REQUIRED_KEYS = {"trinary", "resolution", "origin_x", "origin_y",
                  "centerline_xy", "raceline_xy", "raceline_v"}


class LoadedMap(NamedTuple):
    resolution: float
    origin_x: float
    origin_y: float
    trinary: np.ndarray          #: (height, width) uint8。0=未知 1=空き 2=占有
    centerline_xy: np.ndarray    #: (N, 2)
    raceline_xy: np.ndarray      #: (M, 2)
    raceline_v: np.ndarray       #: (M,)
    created_at: float


# ── パス解決 ──

def resolve_map_path(name: str) -> Path | None:
    """`saved_maps/` の外に出る名前（`../`・絶対パス等）を弾く。

    `raspi/nodes/telemetry_node.py::_resolve_log_path` と同じ禁則。
    """
    if not name or "/" in name or "\\" in name or name in (".", ".."):
        return None
    base = MAPS_DIR.resolve()
    target = (base / f"{name}.npz").resolve()
    if target.parent != base:
        return None
    return target


def _meta_path(npz_path: Path) -> Path:
    return npz_path.with_suffix(".json")


def _polyline_length(xy: np.ndarray) -> float:
    xy = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    if len(xy) < 2:
        return 0.0
    d = np.roll(xy, -1, axis=0) - xy
    return float(np.hypot(d[:, 0], d[:, 1]).sum())


def _write_meta(target: Path, loaded: LoadedMap) -> None:
    height, width = loaded.trinary.shape
    meta = {
        "created_at": loaded.created_at,
        "resolution": loaded.resolution,
        "width": int(width),
        "height": int(height),
        "raceline_points": int(len(loaded.raceline_xy)),
        "length_m": _polyline_length(loaded.raceline_xy),
    }
    _meta_path(target).write_text(json.dumps(meta))


# ── 一覧 ──

def list_maps() -> list[dict]:
    """軽いメタデータだけを返す。**npzは1つも展開しない**（`<name>.json`だけ読む）。"""
    if not MAPS_DIR.is_dir():
        return []
    out = []
    for p in sorted(MAPS_DIR.glob("*.npz")):
        try:
            meta = json.loads(_meta_path(p).read_text())
        except (OSError, ValueError):
            continue
        out.append({"name": p.stem, **meta})
    return out


# ── 保存 ──

def save_map(name: str, *, resolution: float, origin_x: float, origin_y: float,
            trinary: np.ndarray, centerline_xy: np.ndarray,
            raceline_xy: np.ndarray, raceline_v: np.ndarray,
            created_at: float | None = None) -> None:
    """今の地図（3値＋中心線＋レーシングライン）を`<name>.npz`＋`<name>.json`として書く。

    `raise ValueError`: 名前が不正（`resolve_map_path`参照）。
    """
    target = resolve_map_path(name)
    if target is None:
        raise ValueError(f"invalid map name: {name!r}")
    MAPS_DIR.mkdir(parents=True, exist_ok=True)
    created_at = time.time() if created_at is None else created_at

    loaded = LoadedMap(
        resolution=float(resolution), origin_x=float(origin_x), origin_y=float(origin_y),
        trinary=np.ascontiguousarray(trinary, dtype=np.uint8),
        centerline_xy=np.asarray(centerline_xy, dtype=np.float64).reshape(-1, 2),
        raceline_xy=np.asarray(raceline_xy, dtype=np.float64).reshape(-1, 2),
        raceline_v=np.asarray(raceline_v, dtype=np.float64).reshape(-1),
        created_at=float(created_at))

    tmp = target.with_name(target.name + ".tmp")
    # **ファイルオブジェクトを渡す。** `np.savez_compressed(path, ...)` は
    # 拡張子が`.npz`で終わっていないと自動で足してしまい、`<name>.npz.tmp`が
    # `<name>.npz.tmp.npz`として書かれて後続の`os.replace`が壊れる
    with open(tmp, "wb") as f:
        _write_npz(f, loaded)
    os.replace(tmp, target)      # 原子的に差し替える
    _write_meta(target, loaded)


def _write_npz(f: BinaryIO, m: LoadedMap) -> None:
    np.savez_compressed(
        f, trinary=m.trinary,
        resolution=np.float64(m.resolution), origin_x=np.float64(m.origin_x),
        origin_y=np.float64(m.origin_y), centerline_xy=m.centerline_xy,
        raceline_xy=m.raceline_xy, raceline_v=m.raceline_v,
        created_at=np.float64(m.created_at))


# ── 読み込み・検証 ──

def _load_npz(file) -> LoadedMap:
    """`raise ValueError`: 必須キー欠落・型不正。それ以外の壊れ方は
    `np.load`/`zipfile`側の例外（呼び出し側でまとめて潰す）。
    """
    with np.load(file, allow_pickle=False) as z:
        missing = _REQUIRED_KEYS - set(z.files)
        if missing:
            raise ValueError(f"missing keys: {sorted(missing)}")
        trinary = np.asarray(z["trinary"])
        if trinary.ndim != 2 or trinary.dtype != np.uint8:
            raise ValueError("trinary must be a 2D uint8 array")
        if not np.isin(trinary, (0, 1, 2)).all():
            raise ValueError("trinary must contain only 0/1/2")
        return LoadedMap(
            resolution=float(z["resolution"]), origin_x=float(z["origin_x"]),
            origin_y=float(z["origin_y"]), trinary=np.ascontiguousarray(trinary),
            centerline_xy=np.asarray(z["centerline_xy"], dtype=np.float64).reshape(-1, 2),
            raceline_xy=np.asarray(z["raceline_xy"], dtype=np.float64).reshape(-1, 2),
            raceline_v=np.asarray(z["raceline_v"], dtype=np.float64).reshape(-1),
            created_at=float(z["created_at"]) if "created_at" in z.files else 0.0)


def load_map(name: str) -> LoadedMap | None:
    """壊れている・存在しないなら`None`（呼び出し側は理由を出し分けなくてよい設計）。"""
    target = resolve_map_path(name)
    if target is None or not target.is_file():
        return None
    try:
        return _load_npz(target)
    except (OSError, ValueError, KeyError, zipfile.BadZipFile, EOFError):
        return None


def validate_upload(data: bytes) -> LoadedMap | None:
    """アップロードされたバイト列を検証だけする（書き込まない）。壊れていれば`None`。"""
    import io
    try:
        return _load_npz(io.BytesIO(data))
    except (OSError, ValueError, KeyError, zipfile.BadZipFile, EOFError):
        return None


def save_upload(name: str, data: bytes) -> LoadedMap | None:
    """検証してから`saved_maps/`へ書く。壊れている/名前が不正なら書かず`None`。"""
    loaded = validate_upload(data)
    if loaded is None:
        return None
    target = resolve_map_path(name)
    if target is None:
        return None
    MAPS_DIR.mkdir(parents=True, exist_ok=True)
    tmp = target.with_name(target.name + ".tmp")
    tmp.write_bytes(data)
    os.replace(tmp, target)
    _write_meta(target, loaded)
    return loaded


# ── 削除 ──

def delete_map(name: str) -> None:
    target = resolve_map_path(name)
    if target is None:
        return
    target.unlink(missing_ok=True)
    _meta_path(target).unlink(missing_ok=True)
