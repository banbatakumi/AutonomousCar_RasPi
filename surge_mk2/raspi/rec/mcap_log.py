"""MCAP 書き出し（`docs/architecture.md` §11）。

**`.sfl` が一次記録、MCAP は「読ませるための形」**という役割分担にしてある。

- `.sfl` … UART を流れた生バイトそのもの。CRC エラーや SEQ 逆行といった
  「異常の証拠」が残る。io_node の実時間ループが stdlib だけで書ける
- `.mcap` … 解釈済みのトピック。**Foxglove Studio でそのまま開ける。**
  外部ライブラリ（`mcap`）に依存するが、書くのは logger_node と
  オフラインのエクスポータだけなので、io_node の実時間パスは汚れない

## ワイヤ形式は JSON + jsonschema にする

内部バスは msgpack だが、**MCAP の中身は JSON にする。** Foxglove Studio が
MCAP で読めるのは protobuf / ros / flatbuffer / **json** であって、msgpack は
入っていない。せっかく MCAP にしても開けなければ意味がない。

スキーマは `msgspec.json.schema()` が msgspec 型から自動生成する。
**メッセージ型を足したときにスキーマを書き忘れる余地を無くす**ため、
手書きしない（Foxglove 用の可視化トピックだけは例外で、下に手書きしてある）。

## 時刻は UNIX epoch に直して書く

バス上の時刻は `CLOCK_MONOTONIC`（Pi の起動からの経過）なので、そのまま
`log_time` に入れると Foxglove が「1970年1月4日」と表示する。記録開始時点で
`CLOCK_MONOTONIC` と `CLOCK_REALTIME` を1組取っておき、その差で epoch に直す。

**メッセージの中身（`t_capture` / `t_pub`）は単調時刻のまま触らない。**
換算するのは MCAP のヘッダ側の時刻だけ。こうしておけば `.sfl` と突き合わせられる。
変換に使った1組は MCAP のメタデータ `surge` に残してあるので、後から逆算できる。

## Foxglove 用の可視化トピック

生トピック（`/scan` など）は数値の記録としては完全だが、Foxglove の 3D パネルは
`foxglove.*` のスキーマ名しか解釈しない。そこで**同じデータを別トピックにも書く**。

- `/viz/scan` … `foxglove.PointCloud`。**無効点（`dist=0`）は落とす。**
  0 を残すと原点に壁があるように描かれる（`msgs.convert` の ScanAssembler と同じ理由）
- `/viz/image/front` … `foxglove.CompressedImage`（JPEG）
"""

from __future__ import annotations

import base64
import math
import struct
import sys
import time
from pathlib import Path
from typing import Any

import msgspec

__all__ = [
    "McapLog",
    "json_schema",
    "default_mcap_path",
    "HAS_MCAP",
    "VIZ_SCAN_TOPIC",
    "VIZ_IMAGE_PREFIX",
    "LIDAR_FRAME_ID",
]

try:
    from mcap.writer import CompressionType, Writer
except ImportError:                         # pragma: no cover - 環境依存
    HAS_MCAP = False
else:
    HAS_MCAP = True

#: 可視化トピック。生トピックと名前が衝突しないよう `/viz/` にまとめる
VIZ_SCAN_TOPIC = "/viz/scan"
VIZ_IMAGE_PREFIX = "/viz/image/"

#: 座標系の名前。**車両座標**（x=前・y=左・反時計回りが正）。
#: LiDAR は車体に固定なので Phase 0 では車両座標と同一に扱う
LIDAR_FRAME_ID = "base_link"

_COMPRESSION = {"zstd": "ZSTD", "lz4": "LZ4", "none": "NONE"}

_encode = msgspec.json.encode


def default_mcap_path(directory: str | Path = "logs", prefix: str = "surge") -> Path:
    """`logs/surge_20260808_153012.mcap` を作る（`.sfl` と同じ命名規則）。"""
    d = Path(directory)
    d.mkdir(parents=True, exist_ok=True)
    return d / f"{prefix}_{time.strftime('%Y%m%d_%H%M%S', time.localtime())}.mcap"


def json_schema(cls: type) -> dict[str, Any]:
    """msgspec 型 → JSON Schema（`$ref` を展開した自己完結形）。

    `msgspec.json.schema()` は `{"$ref": "#/$defs/X", "$defs": {...}}` を返すが、
    **Foxglove は入れ子の `$ref` を解決しない。** 先頭の1段だけ展開して、
    トップレベルが `"type": "object"` になるようにする。
    """
    s = msgspec.json.schema(cls)
    ref = s.pop("$ref", None)
    if ref is None:
        return s
    defs = s.pop("$defs", {})
    name = ref.rsplit("/", 1)[-1]
    top = dict(defs.pop(name))
    if defs:
        # 残りは `#/$defs/...` で引かれるのでルートに置いたまま持ち回る
        top["$defs"] = defs
    top.update(s)
    return top


# ── Foxglove のスキーマ（ここだけ手書き） ────────────────────────────────

_TIME = {"type": "object",
         "properties": {"sec": {"type": "integer"}, "nsec": {"type": "integer"}}}
_VEC3 = {"type": "object", "properties": {k: {"type": "number"} for k in "xyz"}}
_QUAT = {"type": "object", "properties": {k: {"type": "number"} for k in "xyzw"}}
_POSE = {"type": "object",
         "properties": {"position": _VEC3, "orientation": _QUAT}}

#: `foxglove.PackedElementField.type` の FLOAT32
_FLOAT32 = 7

FOXGLOVE_POINTCLOUD_SCHEMA: dict[str, Any] = {
    "type": "object",
    "properties": {
        "timestamp": _TIME,
        "frame_id": {"type": "string"},
        "pose": _POSE,
        "point_stride": {"type": "integer"},
        "fields": {"type": "array", "items": {
            "type": "object",
            "properties": {"name": {"type": "string"},
                           "offset": {"type": "integer"},
                           "type": {"type": "integer"}}}},
        "data": {"type": "string", "contentEncoding": "base64"},
    },
}

FOXGLOVE_COMPRESSED_IMAGE_SCHEMA: dict[str, Any] = {
    "type": "object",
    "properties": {
        "timestamp": _TIME,
        "frame_id": {"type": "string"},
        "data": {"type": "string", "contentEncoding": "base64"},
        "format": {"type": "string"},
    },
}

#: 単位行列の姿勢。**点群は既に車両座標に直してある**ので変換は要らない
_IDENTITY_POSE = {"position": {"x": 0.0, "y": 0.0, "z": 0.0},
                  "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}

_XYZ_FIELDS = [{"name": "x", "offset": 0, "type": _FLOAT32},
               {"name": "y", "offset": 4, "type": _FLOAT32},
               {"name": "z", "offset": 8, "type": _FLOAT32}]

#: 添字（車両角[deg]）→ 単位ベクトル。毎周 720回の三角関数を回す意味がない
_DIR = [(math.cos(math.radians(d)), math.sin(math.radians(d))) for d in range(360)]


def _stamp(unix_ns: int) -> dict[str, int]:
    return {"sec": unix_ns // 1_000_000_000, "nsec": unix_ns % 1_000_000_000}


def pointcloud_from_scan(scan, unix_ns: int, frame_id: str = LIDAR_FRAME_ID) -> dict:
    """`Scan` → `foxglove.PointCloud`。**無効点は落とす。**

    `dist=0` は「そこに何も無い」ではなく「測れなかった」なので、
    残すと原点を囲む円状の壁として描かれる（`msgs.convert` の ScanAssembler 参照）。
    見えなかったセクタも同様に落ちる（`dist` が 0 のままのため）。
    """
    buf = bytearray()
    dist = scan.dist
    for deg in range(min(360, len(dist))):
        d = dist[deg]
        if d <= 0.0:
            continue
        cx, sy = _DIR[deg]
        buf += struct.pack("<3f", d * cx, d * sy, 0.0)
    return {
        "timestamp": _stamp(unix_ns),
        "frame_id": frame_id,
        "pose": _IDENTITY_POSE,
        "point_stride": 12,
        "fields": _XYZ_FIELDS,
        "data": base64.b64encode(bytes(buf)).decode("ascii"),
    }


def compressed_image(jpeg: bytes, unix_ns: int, frame_id: str) -> dict:
    """JPEG バイト列 → `foxglove.CompressedImage`。"""
    return {
        "timestamp": _stamp(unix_ns),
        "frame_id": frame_id,
        "data": base64.b64encode(jpeg).decode("ascii"),
        "format": "jpeg",
    }


class _CountingWriter:
    """書いたバイト数を数えるラッパ。**パイプへ流すために要る。**

    2つの役割がある。

    1. **`tell()` を提供する。** `mcap.writer.Writer` は索引に入れるオフセットを
       `stream.tell()` で取るが、パイプは seek できず `OSError: Illegal seek` で落ちる。
       MCAP は追記専用なので**書いたバイト数がそのままオフセット**であり、
       自前で数えれば正しい値を返せる
    2. 進捗表示の大きさ。パイプでは `stat()` が使えない
    """

    def __init__(self, raw) -> None:
        self._raw = raw
        self.count = 0

    def write(self, b) -> int:
        n = self._raw.write(b)
        self.count += len(b) if n is None else n
        return n

    def tell(self) -> int:
        return self.count

    def __getattr__(self, name):
        return getattr(self._raw, name)


# ── 本体 ────────────────────────────────────────────────────────────────

class McapLog:
    """トピックを MCAP に書く。チャネルは**初出のトピックで自動登録**する。

        with McapLog("run.mcap") as log:
            log.write("/vehicle_state", state, t_mono_ns=state.t_capture)

    :param path: 出力先。**`"-"` を渡すと標準出力へ流す**（下記）
    :param t0_mono_ns: 単調時刻→epoch 換算の基準（省略時は現在時刻を1組取る）。
        **`.sfl` から変換するときは必ずログのヘッダの値を渡すこと。**
        現在時刻で換算すると、記録した日時ではなく変換した日時になる
    :param t0_unix_ns: 同上の epoch 側
    :param compression: `zstd` / `lz4` / `none`
    :param metadata: MCAP のメタデータ `surge` に載せる情報（記録元・引数など）
    :raises RuntimeError: `mcap` が入っていない

    ## 標準出力に流せる（SD カードを削らないため）

    `path="-"` で `sys.stdout.buffer` に書く。**Pi 側で ssh 越しに実行すれば、
    記録は PC のディスクにだけ落ちて SD には1バイトも書かれない。**

        ssh surge-mk2 '~/surge_mk2/.venv/bin/python -m raspi.nodes.logger_node -o -' > run.mcap

    MCAP は追記型なのでパイプでもそのまま成立する。**途中で ssh が切れたら
    索引が書かれない**が、`tools/mcap_repair.py` で救える。
    """

    def __init__(self, path: str | Path, *, t0_mono_ns: int | None = None,
                 t0_unix_ns: int | None = None, compression: str = "zstd",
                 metadata: dict[str, Any] | None = None) -> None:
        if not HAS_MCAP:
            raise RuntimeError(
                "mcap が入っていません。`pip install mcap` してください "
                "（.sfl の記録・再生には不要で、MCAP 書き出しのときだけ要ります）")

        self.t0_mono_ns = t0_mono_ns if t0_mono_ns is not None else time.monotonic_ns()
        self.t0_unix_ns = t0_unix_ns if t0_unix_ns is not None else time.time_ns()

        name = _COMPRESSION.get(compression)
        if name is None:
            raise ValueError(f"未知の圧縮方式: {compression!r}（zstd/lz4/none）")

        self.to_stdout = str(path) == "-"
        raw_f = None
        if self.to_stdout:
            self.path = None
            # **バイト列を書くので `sys.stdout` ではなく `.buffer`。**
            # テキスト層を通すと改行が変換されてファイルが壊れる
            self._f = _CountingWriter(sys.stdout.buffer)
        else:
            self.path = Path(path)
            self.path.parent.mkdir(parents=True, exist_ok=True)
            raw_f = open(self.path, "wb")
            self._f = _CountingWriter(raw_f)
        try:
            self._w = Writer(self._f, compression=getattr(CompressionType, name))
            self._w.start(profile="", library="surge-mk2")
        except Exception:
            # ここで失敗すると呼び出し側は `self` を持てない（`close()` を
            # 呼べない）ので、開いた fd はここで自分で閉じる
            if raw_f is not None:
                raw_f.close()
            raise
        self._schemas: dict[str, int] = {}
        self._channels: dict[str, int] = {}
        self._seq: dict[str, int] = {}
        self._closed = False

        #: トピックごとの書き込み数。終了時の要約に使う
        self.counts: dict[str, int] = {}
        self.written = 0

        meta = {"t0_mono_ns": str(self.t0_mono_ns), "t0_unix_ns": str(self.t0_unix_ns),
                "t0_local": time.strftime("%Y-%m-%d %H:%M:%S",
                                          time.localtime(self.t0_unix_ns / 1e9))}
        for k, v in (metadata or {}).items():
            meta[k] = str(v)
        self._w.add_metadata("surge", meta)

    # ── 時刻 ──

    def to_unix_ns(self, t_mono_ns: int) -> int:
        """`CLOCK_MONOTONIC` [ns] → UNIX epoch [ns]。"""
        return t_mono_ns - self.t0_mono_ns + self.t0_unix_ns

    # ── チャネル登録 ──

    def _schema_id(self, name: str, schema: dict) -> int:
        sid = self._schemas.get(name)
        if sid is None:
            sid = self._schemas[name] = self._w.register_schema(
                name=name, encoding="jsonschema", data=_encode(schema))
        return sid

    def channel(self, topic: str, schema_name: str, schema: dict) -> int:
        """トピックのチャネル ID を得る（未登録なら作る）。"""
        cid = self._channels.get(topic)
        if cid is None:
            cid = self._channels[topic] = self._w.register_channel(
                topic=topic, message_encoding="json",
                schema_id=self._schema_id(schema_name, schema))
        return cid

    # ── 書き込み ──

    def write(self, topic: str, msg, *, t_mono_ns: int | None = None,
              schema_name: str | None = None) -> None:
        """msgspec / dataclass のメッセージを1件書く。

        :param t_mono_ns: この観測の時刻（単調）。省略時は `t_capture` → `t_pub` の順に見る。
            **`t_capture` を優先するのは「センサが測った時刻」で並べたいため**
            （publish 時刻で並べると UART の往復ぶんずれる）
        :param schema_name: スキーマ名。省略時は型名
        """
        self._check_open()
        if t_mono_ns is None:
            t_mono_ns = getattr(msg, "t_capture", 0) or getattr(msg, "t_pub", 0) \
                or time.monotonic_ns()
        cid = self._channels.get(topic)
        if cid is None:
            # **スキーマを組むのは初出のトピックのときだけ。** 毎回作ると
            # 50Hz × トピック数ぶん `msgspec.json.schema()` を回すことになる
            cls = type(msg)
            cid = self.channel(topic, schema_name or cls.__name__, json_schema(cls))
        self._write(cid, topic, _encode(msg), t_mono_ns, getattr(msg, "seq", 0))

    def write_dict(self, topic: str, obj: dict, schema_name: str, schema: dict,
                   t_mono_ns: int) -> None:
        """スキーマを明示して dict を1件書く（Foxglove 用トピック・イベント）。"""
        self._check_open()
        cid = self._channels.get(topic)
        if cid is None:
            cid = self.channel(topic, schema_name, schema)
        self._write(cid, topic, _encode(obj), t_mono_ns, 0)

    def _check_open(self) -> None:
        """**チャネル登録より先に見る。** 閉じた Writer に触ると分かりにくく壊れる。"""
        if self._closed:
            raise ValueError("クローズ済みの McapLog に書き込もうとしました")

    def _write(self, cid: int, topic: str, data: bytes,
               t_mono_ns: int, seq: int) -> None:
        t = self.to_unix_ns(t_mono_ns)
        if not seq:
            seq = self._seq.get(topic, 0) + 1
        self._seq[topic] = seq
        self._w.add_message(channel_id=cid, log_time=t, publish_time=t,
                            data=data, sequence=seq)
        self.counts[topic] = self.counts.get(topic, 0) + 1
        self.written += 1

    # ── Foxglove 用 ──

    def write_viz_scan(self, scan, *, t_mono_ns: int | None = None,
                       topic: str = VIZ_SCAN_TOPIC) -> None:
        t = t_mono_ns if t_mono_ns is not None else (scan.t_capture or scan.t_pub)
        self.write_dict(topic, pointcloud_from_scan(scan, self.to_unix_ns(t)),
                        "foxglove.PointCloud", FOXGLOVE_POINTCLOUD_SCHEMA, t)

    def write_viz_image(self, jpeg: bytes, cam: str, t_mono_ns: int) -> None:
        self.write_dict(VIZ_IMAGE_PREFIX + cam,
                        compressed_image(jpeg, self.to_unix_ns(t_mono_ns), cam),
                        "foxglove.CompressedImage", FOXGLOVE_COMPRESSED_IMAGE_SCHEMA,
                        t_mono_ns)

    # ── 後始末 ──

    def close(self) -> None:
        """要約セクションを書いて閉じる。

        **`finish()` を呼ばないとインデックスが書かれず、Foxglove がシークできない。**
        （読めなくはないが、全部走査することになる）
        """
        if self._closed:
            return
        self._closed = True
        try:
            self._w.finish()
            self._f.flush()
        finally:
            # 標準出力は**閉じない**（呼び出し側の道具が続けて使うことがある）
            if not self.to_stdout:
                self._f.close()

    @property
    def size_bytes(self) -> int:
        """書いたバイト数。パイプでも数えられるよう自前で数えている。"""
        return self._f.count

    @property
    def size_text(self) -> str:
        """人間に見せる大きさ。**短い記録で `0.0MB` と出ない**ようにする。"""
        n = self.size_bytes
        return f"{n / 1e6:.1f}MB" if n >= 1e6 else f"{n / 1e3:.0f}KB"

    def __enter__(self) -> "McapLog":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
