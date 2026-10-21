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

学習した横グリップ（`raspi/nav/grip.py`、`<name>.grip.json`）も同じ流儀で `grip_json` として同梱する。
こちらは壊れていても地図は受け付け、値だけ捨てる（無くても走れる。読み込む側が範囲を検査して丸める）。

`routes_json` が無いと、Pi に上げた地図には経由点も地図作成の軌跡も無く、自動経路は保存した
線をなぞるだけ・道路グラフの逆走禁止も効かない。`routes_json` の無い古い npz も読める。

## `allow_pickle=False` は必須

アップロード経由で任意のバイト列が `.npz` として渡ってくる。pickle 復元による
任意コード実行を避けるため、保存・読み込みの両方で明示的に禁止する。
"""

from __future__ import annotations

import json
import math
import os
import time
from pathlib import Path
from typing import BinaryIO, NamedTuple

import numpy as np

__all__ = ["MAPS_DIR", "LoadedMap", "resolve_map_path", "list_maps",
           "save_map", "load_map", "delete_map", "validate_upload", "save_upload",
           "import_upload", "export_map", "save_routes", "load_routes", "ROUTES_KEY",
           "save_grip", "load_grip", "clear_grip", "save_grip_vehicle", "load_grip_vehicle",
           "save_grip_surface", "load_grip_surface", "clear_grip_surface", "GRIP_KEY",
           "save_line_cache", "load_line_cache",
           "polyline_length"]

MAPS_DIR = Path(__file__).resolve().parents[2] / "saved_maps"

#: DL/UL の npz に同梱する経路の設定（JSON 文字列）の項目名（モジュール docstring）
ROUTES_KEY = "routes_json"
#: npz に同梱する学習した横グリップの項目名（`nav/grip.GripMap` の JSON）
GRIP_KEY = "grip_json"
#: 地図によらない学習の記録を置くファイル。地図ではないので `*.npz` の一覧に出ない。
#: 車に固有の値（`launch_ratio` = 横の限界 ÷ 発進の加速度）と、路面の記録（`surface`、`save_grip_surface`）
_GRIP_VEHICLE = "_grip_vehicle.json"
#: 路面の記録として受け付ける範囲 [m/s²]（`nav/grip.A_FLOOR`〜`A_CEIL`、摩擦円の比は 1〜`COUP_CEIL`）
_SURFACE_RANGE = {"lat": (1.0, 9.0), "dec": (0.5, 9.0), "coup": (1.0, 2.0)}
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


def _grip_path(npz_path: Path) -> Path:
    """学習した横グリップ（`raspi/nav/grip.py`）。`<name>.grip.json`。"""
    return npz_path.with_name(npz_path.stem + ".grip.json")


def _lines_path(npz_path: Path) -> Path:
    """仕上げた経路の写し（`save_line_cache`）。`<name>.lines.pkl`。"""
    return npz_path.with_name(npz_path.stem + ".lines.pkl")


def polyline_length(xy: np.ndarray) -> float:
    """閉じた点列（周回）の1周の長さ [m]（`RaceLine.length` と同じ定義）。"""
    xy = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    if len(xy) < 2:
        return 0.0
    d = np.roll(xy, -1, axis=0) - xy
    return float(np.hypot(d[:, 0], d[:, 1]).sum())


def _write_text_atomic(p: Path, text: str, *, sync: bool) -> None:
    """`p` へ原子的に書く。`sync` なら差し替える前にディスクまで落とす（`_write_npz_atomic`）。
    走行中に何度も書く小さなファイル（`.grip.json`）は `sync=False`（計画の周期を止めない）。"""
    tmp = p.with_name(p.name + ".tmp")
    with open(tmp, "w") as f:
        f.write(text)
        if sync:
            f.flush()
            os.fsync(f.fileno())
    os.replace(tmp, p)


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
    _write_text_atomic(p, json.dumps(meta, allow_nan=False), sync=True)


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
        # 学習値の有無は軽いファイルの有無だけで分かる（npz を開かない）。GUI は
        # 「学習値を消す」を押せるかどうかの表示に使う
        out.append({"name": p.stem, **meta, "has_grip": _grip_path(p).is_file()})
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
    # 学習した横グリップは地図の座標で持つ。別の地図を同じ名前で書くなら古い値は使えない
    # （経路の設定と違って人の作業ではないので、断らずに消す）
    _grip_path(target).unlink(missing_ok=True)
    _lines_path(target).unlink(missing_ok=True)
    created_at = time.time() if created_at is None else created_at

    loaded = LoadedMap(
        resolution=float(resolution), origin_x=float(origin_x), origin_y=float(origin_y),
        trinary=np.ascontiguousarray(trinary, dtype=np.uint8),
        centerline_xy=np.asarray(centerline_xy, dtype=np.float64).reshape(-1, 2),
        raceline_xy=np.asarray(raceline_xy, dtype=np.float64).reshape(-1, 2),
        raceline_v=np.asarray(raceline_v, dtype=np.float64).reshape(-1),
        created_at=float(created_at))
    _check_finite(loaded)       # 読み込み側が弾く地図を、保存の時点で断る（一覧に居座って消せなくなる）

    tmp = target.with_name(target.name + ".tmp")
    # **ファイルオブジェクトを渡す。** `np.savez_compressed(path, ...)` は
    # 拡張子が`.npz`で終わっていないと自動で足してしまい、`<name>.npz.tmp`が
    # `<name>.npz.tmp.npz`として書かれて後続の`os.replace`が壊れる
    _write_npz_atomic(tmp, target, loaded)
    _write_meta(target, loaded)


def _write_npz_atomic(tmp: Path, target: Path, m: LoadedMap) -> None:
    """`tmp` へ書いて**ディスクまで落としてから**（fsync）`target` へ原子的に差し替える。

    地図は数分かけて作るもので、作った直後に電源を切られることが多い（バッテリーを抜く）。
    fsync しないと、差し替えだけがディスクに残って中身の無いファイルになりうる。
    走行中に何度も書く小さなファイル（`.grip.json`）は fsync しない（計画の周期を止めない）。
    """
    with open(tmp, "wb") as f:
        _write_npz(f, m)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, target)


def _write_npz(f: BinaryIO, m: LoadedMap, *, routes: str | None = None,
               grip: str | None = None) -> None:
    extra = {} if routes is None else {ROUTES_KEY: np.array(routes)}
    if grip:
        extra[GRIP_KEY] = np.array(grip)
    np.savez_compressed(
        f, trinary=m.trinary,
        resolution=np.float64(m.resolution), origin_x=np.float64(m.origin_x),
        origin_y=np.float64(m.origin_y), centerline_xy=m.centerline_xy,
        raceline_xy=m.raceline_xy, raceline_v=m.raceline_v,
        created_at=np.float64(m.created_at), **extra)


# ── 読み込み・検証 ──

#: 地図の格子の上限（セル数）。実機の地図は 640×640 前後（0.025m、16m 四方）。圧縮後 32MB の
#: アップロードでも、deflate は同じ値の繰り返しを千倍以上に縮めるので、展開後は何 GB にもなりうる
#: （Pi が OOM で落ちる）。展開する前に配列の形から断る
MAX_MAP_CELLS = 16_000_000
#: 中心線・レーシングラインの点数の上限
MAX_MAP_POINTS = 200_000


def _check_npz_sizes(z) -> None:
    """`np.load` した npz の各配列を**展開する前に**、`.npy` のヘッダの形だけで大きさを見る。"""
    from numpy.lib import format as npy
    for key, limit in (("trinary", MAX_MAP_CELLS), ("centerline_xy", 2 * MAX_MAP_POINTS),
                       ("raceline_xy", 2 * MAX_MAP_POINTS), ("raceline_v", MAX_MAP_POINTS)):
        with z.zip.open(key + ".npy") as f:
            version = npy.read_magic(f)
            read_header = npy.read_array_header_1_0 if version == (1, 0) else npy.read_array_header_2_0
            shape = read_header(f)[0]
        if math.prod(shape) > limit:
            raise ValueError(f"{key} is too large: shape={shape}")


def _check_finite(m: LoadedMap) -> None:
    """数値として使えない中身を走行まで持ち込まない（NaN の経路は RACE に入ってから毎周期
    例外になる）。読み込み（`_load_npz`）と保存（`save_map`）が同じ条件を共有する。
    **点の数はここでは見ない**——ラインが空の地図（経路を道路グラフから作り直す
    `slam2d_route` 用）は有効で、足りるかは読む側が決める"""
    if not (math.isfinite(m.resolution) and m.resolution > 0.0
            and math.isfinite(m.origin_x) and math.isfinite(m.origin_y)):
        raise ValueError("resolution/origin must be finite and resolution > 0")
    for key in ("centerline_xy", "raceline_xy", "raceline_v"):
        if not np.isfinite(getattr(m, key)).all():
            raise ValueError(f"{key} contains NaN/Inf")


def _load_npz(file) -> LoadedMap:
    """`raise ValueError`: 必須キー欠落・型不正。それ以外の壊れ方は
    `np.load`/`zipfile`側の例外で、**種類は決まっていない**（圧縮データが壊れていれば `zlib.error`、
    0 次元でない数値なら `TypeError`）。呼び出し側は `except Exception` でまとめて潰す——
    取りこぼすと、地図を選んだだけで planner を呼んだノードまで落ちる。
    """
    with np.load(file, allow_pickle=False) as z:
        missing = _REQUIRED_KEYS - set(z.files)
        if missing:
            raise ValueError(f"missing keys: {sorted(missing)}")
        _check_npz_sizes(z)
        trinary = np.asarray(z["trinary"])
        if trinary.ndim != 2 or trinary.dtype != np.uint8:
            raise ValueError("trinary must be a 2D uint8 array")
        if trinary.size and int(trinary.max()) > 2:      # uint8 なので 0 未満は無い。一時配列も作らない
            raise ValueError("trinary must contain only 0/1/2")
        m = LoadedMap(
            resolution=float(z["resolution"]), origin_x=float(z["origin_x"]),
            origin_y=float(z["origin_y"]), trinary=np.ascontiguousarray(trinary),
            centerline_xy=np.asarray(z["centerline_xy"], dtype=np.float64).reshape(-1, 2),
            raceline_xy=np.asarray(z["raceline_xy"], dtype=np.float64).reshape(-1, 2),
            raceline_v=np.asarray(z["raceline_v"], dtype=np.float64).reshape(-1),
            created_at=float(z["created_at"]) if "created_at" in z.files else 0.0)
    _check_finite(m)
    return m


def load_map(name: str) -> LoadedMap | None:
    """壊れている・存在しないなら`None`（呼び出し側は理由を出し分けなくてよい設計）。"""
    target = resolve_map_path(name)
    if target is None or not target.is_file():
        return None
    try:
        return _load_npz(target)
    except Exception:                       # noqa: BLE001  壊れ方は問わない（下記）
        return None


def validate_upload(data: bytes) -> LoadedMap | None:
    """アップロードされたバイト列を検証だけする（書き込まない）。壊れていれば`None`。"""
    import io
    try:
        return _load_npz(io.BytesIO(data))
    except Exception:                       # noqa: BLE001  壊れ方は問わない（`_load_npz`）
        return None


def save_upload(name: str, data: bytes) -> LoadedMap | None:
    """検証してから`saved_maps/`へ書く。壊れている/名前が不正なら書かず`None`。

    同梱の経路の設定の扱いは `import_upload`（こちらはその戻り値の地図だけを返す）。
    """
    return import_upload(name, data)[0]


def _bundled_text(data: bytes, key: str) -> str | None:
    """npz に同梱された JSON 文字列（項目 `key`）。無ければ `None`。"""
    import io
    with np.load(io.BytesIO(data), allow_pickle=False) as z:
        if key not in z.files:
            return None
        arr = np.asarray(z[key])
        if arr.ndim != 0 or arr.dtype.kind != "U":
            raise ValueError(f"{key} must be a 0-d string array")
        text = str(arr)
    if len(text) > _MAX_ROUTES_CHARS:
        raise ValueError(f"{key} is too large")
    return text


def _bundled_routes(data: bytes) -> str | None:
    """npz に同梱された経路の設定（JSON 文字列）。無ければ `None`。"""
    return _bundled_text(data, ROUTES_KEY)


def _bundled_grip(data: bytes) -> str | None:
    """npz に同梱された横グリップを検査して書き直したもの。無い・壊れているなら `None`。"""
    try:
        text = _bundled_text(data, GRIP_KEY)
        if text is None:
            return None
        from ..nav.grip import GripMap             # 遅延 import（純粋なファイル I/O 層に保つ）
        return GripMap.from_json(text).to_json()
    except Exception:                               # noqa: BLE001  壊れた同梱は捨てる（`_load_npz`）
        return None


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
    except Exception as e:                  # noqa: BLE001  壊れ方は問わない（`_load_npz`）
        return None, f"同梱の経路の設定が不正です（{e}）"
    MAPS_DIR.mkdir(parents=True, exist_ok=True)
    tmp = target.with_name(target.name + ".tmp")
    _write_npz_atomic(tmp, target, loaded)  # 同梱の経路の設定はディスクの npz に残さない
    _write_meta(target, loaded)
    if routes is None:
        _routes_path(target).unlink(missing_ok=True)
    else:
        save_routes(name, routes)
    # 学習した横グリップも同じ扱い（同梱が無ければ古い値を消す）
    grip = _bundled_grip(data)
    if grip is None:
        _grip_path(target).unlink(missing_ok=True)
    else:
        save_grip(name, grip)
    # 仕上げた経路の写しは同梱しない。別の地図の写しが残らないよう消す
    _lines_path(target).unlink(missing_ok=True)
    return loaded, ""


def export_map(name: str) -> bytes | None:
    """ダウンロード用の npz。今の `<name>.routes.json` を `ROUTES_KEY` として、
    `<name>.grip.json` を `GRIP_KEY` として同梱する。

    読めなければ `None`。
    """
    import io
    loaded = load_map(name)
    if loaded is None:
        return None
    routes = load_routes(name)
    buf = io.BytesIO()
    _write_npz(buf, loaded, routes=routes or None, grip=load_grip(name) or None)
    return buf.getvalue()


# ── 削除 ──

def delete_map(name: str) -> None:
    target = resolve_map_path(name)
    if target is None:
        return
    target.unlink(missing_ok=True)
    _meta_path(target).unlink(missing_ok=True)
    _routes_path(target).unlink(missing_ok=True)
    _grip_path(target).unlink(missing_ok=True)
    _lines_path(target).unlink(missing_ok=True)


# ── 経路の設定（経由点・停止点・ミッション） ──

def save_routes(name: str, text: str) -> bool:
    """経路の設定（JSON文字列、検証は`route_config.RouteConfig`側）を書く。

    地図が存在しない名前には書かない（孤立した設定ファイルを作らない）。
    """
    target = resolve_map_path(name)
    if target is None or not target.is_file():
        return False
    # 人が置いた経由点・停止点。電源を切られても失わないよう fsync する（編集のたびで、周期の中ではない）
    _write_text_atomic(_routes_path(target), text, sync=True)
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


# ── 学習した横グリップ（`raspi/nav/grip.py`） ──

def save_grip(name: str, text: str) -> bool:
    """学習した横グリップ（JSON文字列）を書く。地図が存在しない名前には書かない。"""
    target = resolve_map_path(name)
    if target is None or not target.is_file():
        return False
    _write_text_atomic(_grip_path(target), text, sync=False)
    return True


def load_grip(name: str) -> str:
    """学習した横グリップの JSON 文字列。無ければ空文字（検査は `nav.grip.GripMap.from_json`）。"""
    target = resolve_map_path(name)
    if target is None:
        return ""
    try:
        return _grip_path(target).read_text()
    except OSError:
        return ""


def clear_grip(name: str) -> bool:
    """地図 `name` の学習値（`<name>.grip.json`）だけを消す。消したら `True`。

    **地図・経路の設定（`.routes.json`）・仕上げた経路の写し（`.lines.pkl`）は残す。**
    車に固有の `_grip_vehicle.json`（発進の加速度との比・路面の記録）も残す——あれは
    コースではなく車・床の値なので、地図を選び直しても同じ値を使う。

    走行中の planner は学習値を持ったままなので、GUI からの削除は
    `AutoCtrl.grip_clear_seq` でも planner に伝える（`slam2d_raceline.request_clear_grip`）。
    ここだけで消すと、planner が次の保存で書き戻す。
    """
    target = resolve_map_path(name)
    if target is None:
        return False
    p = _grip_path(target)
    existed = p.is_file()
    p.unlink(missing_ok=True)
    return existed


def _grip_vehicle_read() -> dict:
    try:
        raw = json.loads((MAPS_DIR / _GRIP_VEHICLE).read_text())
    except (OSError, ValueError):
        return {}
    return raw if isinstance(raw, dict) else {}


def _grip_vehicle_write(**items) -> None:
    """`_grip_vehicle.json` の項目を書き換える（他の項目は残す。`None` の項目は消す）。"""
    try:
        raw = _grip_vehicle_read()
        for k, v in items.items():
            if v is None:
                raw.pop(k, None)
            else:
                raw[k] = v
        MAPS_DIR.mkdir(parents=True, exist_ok=True)
        p = MAPS_DIR / _GRIP_VEHICLE
        tmp = p.with_name(p.name + ".tmp")
        tmp.write_text(json.dumps(raw))
        os.replace(tmp, p)
    except OSError:
        pass


def save_line_cache(name: str, key: str, data: bytes) -> None:
    """仕上げた経路（道路グラフ・レーシングライン）を地図の横へ書く。`key` は作った条件の指紋。

    仕上げ（見積もりラップの最小化）は Pi で 8 秒ほどかかり、その間は仕上げ前の線で走る（2026-10-20 の実機:
    1 周目の途中で乗り換えた）。同じ地図・同じ条件なら結果は同じなので、写しを置いて 1 周目から使う。
    ★ 中身は pickle。このプログラムが自分で書いたファイルだけを読む（アップロード・ダウンロードの npz には
    含めない）。地図の削除・上書きで一緒に消す。
    """
    target = resolve_map_path(name)
    if target is None or not target.is_file():
        return
    try:
        _write_bytes_atomic(_lines_path(target), json.dumps({"key": key}).encode() + b"\n" + data)
    except OSError:
        pass


def load_line_cache(name: str, key: str) -> bytes | None:
    """`save_line_cache` の中身。無い・条件 `key` が違うなら `None`。"""
    target = resolve_map_path(name)
    if target is None:
        return None
    try:
        raw = _lines_path(target).read_bytes()
        head, _, data = raw.partition(b"\n")
        if json.loads(head.decode()).get("key") != key:
            return None
    except (OSError, ValueError, UnicodeDecodeError, AttributeError):
        return None
    return data or None


def _write_bytes_atomic(p: Path, data: bytes) -> None:
    tmp = p.with_name(p.name + ".tmp")
    tmp.write_bytes(data)
    os.replace(tmp, p)


def save_grip_vehicle(launch_ratio: float) -> None:
    """車に固有の「横の限界 ÷ 発進の加速度」を書く（地図によらない。`vehicle.toml` には書かない）。"""
    _grip_vehicle_write(launch_ratio=round(float(launch_ratio), 4))


def load_grip_vehicle() -> float:
    """保存した比。無い・壊れている・範囲外（0.5〜4）なら 0。"""
    try:
        r = float(_grip_vehicle_read().get("launch_ratio", 0.0))
    except (ValueError, TypeError):
        return 0.0
    return r if math.isfinite(r) and 0.5 <= r <= 4.0 else 0.0


def save_grip_surface(lat: float, dec: float, coup: float, ceil: float = 0.0, *, t: float) -> None:
    """路面の記録（直近に学習して採用した横・減速の上限 [m/s²]・摩擦円の比・横の安全装置が働いた
    いちばん低い上限 `ceil` [m/s²]（0 = 無い））を書く。

    地図によらない。**新しい地図**（`.grip.json` が無い）の学習の出発点になる（`load_grip_surface`）。
    路面はコースではなく床で決まるので、同じ床の別のコースでも 1 周目から使える。
    :param t: 書いた時刻（UNIX 秒。表示用）
    """
    _grip_vehicle_write(surface={"lat": round(float(lat), 3), "dec": round(float(dec), 3),
                                 "coup": round(float(coup), 3), "ceil": round(float(ceil), 3),
                                 "t": round(float(t), 1)})


def load_grip_surface() -> dict[str, float] | None:
    """路面の記録 `{"lat", "dec", "coup", "ceil", "t"}`。無い・壊れている・範囲外なら `None`
    （`ceil` だけは範囲外なら 0 = 無い）。"""
    raw = _grip_vehicle_read().get("surface")
    if not isinstance(raw, dict):
        return None
    out: dict[str, float] = {}
    try:
        for k, (lo, hi) in _SURFACE_RANGE.items():
            v = float(raw.get(k, 1.0 if k == "coup" else 0.0))
            if not (math.isfinite(v) and lo <= v <= hi):
                return None
            out[k] = v
        c = float(raw.get("ceil", 0.0))
        lo, hi = _SURFACE_RANGE["lat"]
        out["ceil"] = c if math.isfinite(c) and lo <= c <= hi else 0.0
        t = float(raw.get("t", 0.0))
        out["t"] = t if math.isfinite(t) else 0.0
    except (ValueError, TypeError):
        return None
    return out


def clear_grip_surface() -> None:
    """路面の記録を消す（路面が変わった。次の新しい地図は同定値から学習する）。"""
    _grip_vehicle_write(surface=None)
