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

## 経路の設定はダウンロードの瞬間に npz へ同梱する（`export_map` / `import_upload`）

ディスク上は `<name>.npz` と `<name>.routes.json`（経由点・停止点・ミッション・地図作成の軌跡）を
**別々に**置く（経由点は地図を保存した後も GUI で編集されるので、npz に焼き込むと二重管理になる）。
ただし Mac⇔Pi の受け渡し（GUI の DL/UL）では1ファイルで運ぶ:

- DL（`export_map`）: その時点の `.routes.json` を npz の項目 `routes_json`（0次元の文字列配列、
  pickle 不要）として足した npz を返す
- UL（`import_upload`）: `routes_json` があれば `route_config.RouteConfig` で検証して `.routes.json` に書く。
  **無ければ同じ名前の古い `.routes.json` を消す**（別の地図に古い経由点が残らないように）。
  ディスクの npz には `routes_json` を残さない

以前は npz だけを運んでいたので、Pi に上げた地図には経由点も地図作成の軌跡も無く、自動経路は保存した
線をなぞるだけ・道路グラフの逆走禁止も効かなかった（2026-09-29）。`routes_json` の無い古い npz も読める。

## `allow_pickle=False` は必須

アップロード経由で任意のバイト列が `.npz` として渡ってくる。pickle 復元による
任意コード実行を避けるため、保存・読み込みの両方で明示的に禁止する。
"""

from __future__ import annotations

import json
import math
import os
import time
import zipfile
from pathlib import Path
from typing import BinaryIO, NamedTuple

import numpy as np

__all__ = ["MAPS_DIR", "LoadedMap", "resolve_map_path", "list_maps",
           "save_map", "load_map", "delete_map", "validate_upload", "save_upload",
           "import_upload", "export_map", "save_routes", "load_routes", "ROUTES_KEY",
           "polyline_length"]

MAPS_DIR = Path(__file__).resolve().parents[2] / "saved_maps"

#: DL/UL の npz に同梱する経路の設定（JSON 文字列）の項目名（モジュール docstring）
ROUTES_KEY = "routes_json"
#: 同梱できる経路の設定の大きさの上限（`telemetry_node` が GUI から受ける上限と同じ）
_MAX_ROUTES_CHARS = 1_000_000

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
    if name.endswith(".routes"):
        # `<名前>.routes` のメタデータ（`<名前>.routes.json`）は、地図 `<名前>` の経路の設定と
        # 同じファイルになり、保存しただけで相手の経由点を上書きする
        return None
    base = MAPS_DIR.resolve()
    target = (base / f"{name}.npz").resolve()
    if target.parent != base:
        return None
    return target


def _meta_path(npz_path: Path) -> Path:
    return npz_path.with_suffix(".json")


def _routes_path(npz_path: Path) -> Path:
    """経路の設定（`raspi/auto/route_config.py`）。`<name>.routes.json`。"""
    return npz_path.with_name(npz_path.stem + ".routes.json")


def polyline_length(xy: np.ndarray) -> float:
    """閉じた点列（周回）の1周の長さ [m]（`RaceLine.length` と同じ定義）。"""
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
        "length_m": polyline_length(loaded.raceline_xy),
    }
    # 途中で落ちても壊れた JSON を残さない（壊れると `list_maps` がその地図を黙って飛ばす）
    p = _meta_path(target)
    tmp = p.with_name(p.name + ".tmp")
    tmp.write_text(json.dumps(meta))
    os.replace(tmp, p)


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

    `raise ValueError`: 名前が不正（`resolve_map_path`参照）、または**経路の設定を持つ
    既存の地図への上書き**。経路の設定（経由点・停止点）は地図の座標で書かれているので、
    別の地図を同じ名前で書くと古い経由点がそのまま付いてくる。黙って消すと人の作業が
    失われるので、どちらもせずに断る（別名で保存するか、先に削除してもらう）。
    """
    target = resolve_map_path(name)
    if target is None:
        raise ValueError(f"invalid map name: {name!r}")
    if target.is_file() and _routes_path(target).is_file():
        raise ValueError(f"地図『{name}』には経路の設定がある。別の名前で保存するか、先に削除してください")
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


def _write_npz(f: BinaryIO, m: LoadedMap, *, routes: str | None = None) -> None:
    extra = {} if routes is None else {ROUTES_KEY: np.array(routes)}
    np.savez_compressed(
        f, trinary=m.trinary,
        resolution=np.float64(m.resolution), origin_x=np.float64(m.origin_x),
        origin_y=np.float64(m.origin_y), centerline_xy=m.centerline_xy,
        raceline_xy=m.raceline_xy, raceline_v=m.raceline_v,
        created_at=np.float64(m.created_at), **extra)


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
        m = LoadedMap(
            resolution=float(z["resolution"]), origin_x=float(z["origin_x"]),
            origin_y=float(z["origin_y"]), trinary=np.ascontiguousarray(trinary),
            centerline_xy=np.asarray(z["centerline_xy"], dtype=np.float64).reshape(-1, 2),
            raceline_xy=np.asarray(z["raceline_xy"], dtype=np.float64).reshape(-1, 2),
            raceline_v=np.asarray(z["raceline_v"], dtype=np.float64).reshape(-1),
            created_at=float(z["created_at"]) if "created_at" in z.files else 0.0)
    # 数値として使えない中身を走行まで持ち込まない（NaN の経路は RACE に入ってから毎周期
    # 例外になる）。**点の数はここでは見ない**——ラインが空の地図（経路を道路グラフから
    # 作り直す `slam2d_route` 用）は有効で、足りるかは読む側が決める
    if not (math.isfinite(m.resolution) and m.resolution > 0.0
            and math.isfinite(m.origin_x) and math.isfinite(m.origin_y)):
        raise ValueError("resolution/origin must be finite and resolution > 0")
    for key in ("centerline_xy", "raceline_xy", "raceline_v"):
        if not np.isfinite(getattr(m, key)).all():
            raise ValueError(f"{key} contains NaN/Inf")
    return m


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
    """検証してから`saved_maps/`へ書く。壊れている/名前が不正なら書かず`None`。

    同梱の経路の設定の扱いは `import_upload`（こちらはその戻り値の地図だけを返す）。
    """
    return import_upload(name, data)[0]


def _bundled_routes(data: bytes) -> str | None:
    """npz に同梱された経路の設定（JSON 文字列）。無ければ `None`。"""
    import io
    with np.load(io.BytesIO(data), allow_pickle=False) as z:
        if ROUTES_KEY not in z.files:
            return None
        arr = np.asarray(z[ROUTES_KEY])
        if arr.ndim != 0 or arr.dtype.kind != "U":
            raise ValueError("routes_json must be a 0-d string array")
        text = str(arr)
    if len(text) > _MAX_ROUTES_CHARS:
        raise ValueError("routes_json is too large")
    return text


def import_upload(name: str, data: bytes) -> tuple[LoadedMap | None, str]:
    """アップロードを検証して `saved_maps/` へ書く。`(地図, 誤り)`。失敗なら `(None, 理由)`。

    同梱の経路の設定（`ROUTES_KEY`）は検証して `<name>.routes.json` へ。**無ければ古い
    `<name>.routes.json` を消す**。経路の設定が壊れていれば地図ごと受け付けない
    （地図だけ入れて経由点が黙って消えるより、やり直してもらう方がよい）。
    """
    loaded = validate_upload(data)
    if loaded is None:
        return None, "壊れた地図ファイルです"
    target = resolve_map_path(name)
    if target is None:
        return None, "地図の名前が不正です"
    try:
        routes = _bundled_routes(data)
        if routes is not None:
            from .route_config import RouteConfig   # 遅延 import（純粋なファイル I/O 層に保つ）
            routes = RouteConfig.from_json(routes).to_json()
    except (OSError, ValueError, KeyError, zipfile.BadZipFile, EOFError) as e:
        return None, f"同梱の経路の設定が不正です（{e}）"
    MAPS_DIR.mkdir(parents=True, exist_ok=True)
    tmp = target.with_name(target.name + ".tmp")
    with open(tmp, "wb") as f:
        _write_npz(f, loaded)               # 同梱の経路の設定はディスクの npz に残さない
    os.replace(tmp, target)
    _write_meta(target, loaded)
    if routes is None:
        _routes_path(target).unlink(missing_ok=True)
    else:
        save_routes(name, routes)
    return loaded, ""


def export_map(name: str) -> bytes | None:
    """ダウンロード用の npz。今の `<name>.routes.json` を `ROUTES_KEY` として同梱する。

    読めなければ `None`。
    """
    import io
    loaded = load_map(name)
    if loaded is None:
        return None
    routes = load_routes(name)
    buf = io.BytesIO()
    _write_npz(buf, loaded, routes=routes or None)
    return buf.getvalue()


# ── 削除 ──

def delete_map(name: str) -> None:
    target = resolve_map_path(name)
    if target is None:
        return
    target.unlink(missing_ok=True)
    _meta_path(target).unlink(missing_ok=True)
    _routes_path(target).unlink(missing_ok=True)


# ── 経路の設定（経由点・停止点・ミッション） ──

def save_routes(name: str, text: str) -> bool:
    """経路の設定（JSON文字列、検証は`route_config.RouteConfig`側）を書く。

    地図が存在しない名前には書かない（孤立した設定ファイルを作らない）。
    """
    target = resolve_map_path(name)
    if target is None or not target.is_file():
        return False
    p = _routes_path(target)
    tmp = p.with_name(p.name + ".tmp")
    tmp.write_text(text)
    os.replace(tmp, p)
    return True


def load_routes(name: str) -> str:
    """経路の設定の JSON 文字列。無ければ空文字。"""
    target = resolve_map_path(name)
    if target is None:
        return ""
    try:
        return _routes_path(target).read_text()
    except OSError:
        return ""
