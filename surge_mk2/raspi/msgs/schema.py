"""GUI に流すメッセージの「版」。**手で数える版番号を持たない。**

`/ws/telemetry` は msgpack のバイナリをそのまま投げ、GUI は `as Snapshot` で
型を名乗らせているだけなので、**Pi 側がフィールドを消しても GUI は気づかず
`undefined` を読む**（2026-08-21 のレビュー 🟠3）。境界で1つだけ札を見て、
食い違っていたら「古い画面を信じて走る」前に人間へ知らせる。

## なぜ手書きの版番号にしないか

`protocol.toml` の `protocol_version` は STM32 のファームと歩調を合わせる必要が
あるので人間が上げる。こちらは**同じリポジトリの中だけの話**なので、上げ忘れが
そのまま「版は合っているのに中身が違う」という一番たちの悪い状態になる。

だからフィールド名と型そのものから決定的に導く。`types.py` を触れば必ず変わり、
触っていなければ絶対に変わらない。`config/gen_msgs.py` が同じ値を
`gui/src/generated/msgs.ts` に焼き込み、`--check` が両者の一致を見る。

## 何を数えるか

**GUI が読む型だけ**（`MIRRORED`）。`DriveCmd` のような GUI → Pi 方向のものや、
バスの中だけで完結する型（`ImageRef`・`Heartbeat`）は入れない。
入れると、GUI に無関係な変更のたびに版が変わって狼少年になる。
"""

from __future__ import annotations

import hashlib

import msgspec.inspect as mi

from . import types

__all__ = ["MIRRORED", "schema_version"]

#: GUI が写しを持っている型。**`config/gen_msgs.py` が生成する対象と同じ集合。**
#: ここに足したら GUI 側にも型が生えるので、`gen_msgs.py` の `EXPORTS` も直すこと
MIRRORED: tuple[type, ...] = (
    types.VehicleState,
    types.Scan,
    types.LinkDiag,
    types.AutoState,
    types.AutoMap,
    types.LineScan,
    types.TargetTrack,
)


def _field_sig(t) -> str:
    """`msgspec.inspect` の型 → 比較用の文字列。

    `repr()` をそのまま使わないのは、msgspec が上げたときに制約フィールド
    （`gt=None` 等）の表現が変われば版だけが動いてしまうため。
    **名前と構造だけ**を見る。
    """
    if isinstance(t, mi.UnionType):
        return "|".join(_field_sig(x) for x in t.types)
    if isinstance(t, (mi.ListType, mi.SetType, mi.FrozenSetType)):
        return f"list[{_field_sig(t.item_type)}]"
    if isinstance(t, mi.DictType):
        return f"dict[{_field_sig(t.key_type)},{_field_sig(t.value_type)}]"
    return type(t).__name__


def schema_version() -> int:
    """`MIRRORED` のフィールド名と型から決まる 32bit の札。

    **順序も見る。** msgpack は map なので順序が変わっても壊れないが、
    順序が変わるのはたいてい構造を触ったときなので、拾えるなら拾っておく。
    """
    h = hashlib.sha256()
    for info in mi.multi_type_info(list(MIRRORED)):
        h.update(info.cls.__name__.encode())
        for f in info.fields:
            h.update(b"\x00")
            h.update(f.encode_name.encode())
            h.update(b"\x01")
            h.update(_field_sig(f.type).encode())
    return int.from_bytes(h.digest()[:4], "big")
