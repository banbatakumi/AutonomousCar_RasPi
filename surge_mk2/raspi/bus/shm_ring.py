"""画像用の共有メモリリングバッファ。

**なぜ ZMQ に画像を流さないか**（`docs/architecture.md` §6.2）:
640x480x3 を 30Hz で msgpack 経由にすると **28 MB/s のメモリコピー**が発生する
（実測でも 28 MB/s だった）。代わりに:

    camera_node     → 共有メモリの N枚リングに書く
    ZMQ には        → {name, slot, seq, t_capture_ns, ...} の数十バイトだけ流す
    perception_node → その参照から numpy view を作る（**コピーゼロ**）

## 構造

    ┌ ヘッダ 64B ─────────────────────────────────────┐
    │ magic / version / n_slots / slot_bytes           │
    │ width / height / stride / format / write_seq     │
    ├ スロット表 32B × n_slots ────────────────────────┤
    │ seq(seqlock) / t_capture_ns / frame_id / nbytes  │
    ├ データ領域 slot_bytes × n_slots ─────────────────┤
    │ …画素…                                          │
    └──────────────────────────────────────────────────┘

`write_seq` は累積フレーム数。書き込み先は `(write_seq) % n_slots` で決まるので、
**読み手は「最新」を一意に特定できる**。

## seqlock — 読んでいる最中の上書きを検出する

リングを 8枚取れば、書き手が同じスロットに戻るまで 8フレーム（30fps なら 266ms）ある。
実用上は上書きは起きないが、**起きたことを検出できない**のが怖いので seqlock を入れる。

    書き手: seq += 1（奇数＝書き込み中） → データを書く → seq += 1（偶数＝安定）
    読み手: seq を読む（奇数なら再試行） → データを読む → seq を読み直す
            → 値が変わっていたら**読んでいる間に上書きされた**ので破棄

**注意（正直に書く）**: Python には明示的なメモリバリアが無く、この順序は
CPython の実装と x86/ARM のストア順序に依存している。厳密な保証ではない。
実効的な守りは「リング段数を十分取ること」で、seqlock はその**検出手段**である。
`still_valid()` が False を返したら、そのフレームは捨てて次を待つこと。

## 使い方

```python
# 書き手
ring = FrameRing.create("surge_cam0", 640, 480, "RGB888", n_slots=8)
ring.write(frame_bytes, t_capture_ns=ts, frame_id=n)

# 読み手（別プロセス）
ring = FrameRing.attach("surge_cam0")
ref = ring.latest()
if ref is not None:
    arr = ref.as_array()      # コピーゼロの numpy view
    ...                       # 使う
    if not ref.still_valid():
        pass                  # 読んでいる間に上書きされた → 捨てる
```
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from multiprocessing import shared_memory
from typing import NamedTuple

__all__ = ["FrameRing", "FrameRef", "FrameDesc", "MAGIC", "FORMAT_VERSION"]

MAGIC = b"SURGERNG"
FORMAT_VERSION = 1

HEADER_SIZE = 64
SLOT_META_SIZE = 32
_ALIGN = 64

_S_HEADER = struct.Struct("<8sHH5I8sQ16x")
_S_WRITE_SEQ = struct.Struct("<Q")
_WRITE_SEQ_OFF = 40
_S_SLOT = struct.Struct("<4Q")          # seq, t_capture_ns, frame_id, nbytes

#: 1画素あたりのバイト数。生の画素だけを扱う（圧縮形式は対象外）
_BYTES_PER_PIXEL = {"RGB888": 3, "BGR888": 3, "XRGB8888": 4, "XBGR8888": 4,
                    "RGB565": 2, "GREY": 1}


def _open_shm(name: str, create: bool = False, size: int = 0):
    """`track` 引数は Python 3.13 以降にしか無いので吸収する。

    追跡を切るのは、**読み手プロセスが終了するときに segment を消してしまう**
    のを防ぐため（resource_tracker は attach しただけの segment も掃除しにくる）。
    寿命は書き手の `unlink()` で明示的に管理する。
    """
    try:
        return shared_memory.SharedMemory(name=name, create=create, size=size,
                                          track=False)
    except TypeError:
        return shared_memory.SharedMemory(name=name, create=create, size=size)


class FrameDesc(NamedTuple):
    """スロット1枚の説明。**これがバスに流れる想定**（画素は流さない）。"""

    name: str            #: 共有メモリの名前
    slot: int
    seq: int             #: このフレームの通し番号（= write_seq）
    t_capture_ns: int    #: Pi CLOCK_MONOTONIC。カメラの SensorTimestamp をそのまま入れる
    frame_id: int        #: 撮像側のフレーム番号（picamera2 の FrameCount）
    width: int
    height: int
    fmt: str
    stride: int
    nbytes: int


@dataclass(slots=True)
class FrameRef:
    """スロットへのゼロコピー参照。

    **使い終わったら `still_valid()` を確認すること。** False なら
    読んでいる間に書き手が同じスロットを上書きしている（内容が混ざっている）。
    """

    ring: "FrameRing"
    desc: FrameDesc
    _seq_at_read: int

    @property
    def buffer(self) -> memoryview:
        """画素のメモリビュー（コピーなし）。"""
        return self.ring._slot_data(self.desc.slot)[: self.desc.nbytes]

    def as_array(self):
        """numpy view を作る（コピーなし）。numpy が無ければ ImportError。"""
        import numpy as np

        d = self.desc
        bpp = _BYTES_PER_PIXEL.get(d.fmt)
        if bpp is None:
            raise ValueError(f"画素形式 {d.fmt} の配列化には対応していません")
        arr = np.frombuffer(self.buffer, dtype=np.uint8)
        if d.stride == d.width * bpp:
            return arr.reshape(d.height, d.width, bpp) if bpp > 1 else \
                arr.reshape(d.height, d.width)
        # ストライドに詰め物がある場合は行ごとに切り出す
        arr = arr.reshape(d.height, d.stride)
        return arr[:, : d.width * bpp].reshape(d.height, d.width, bpp)

    def copy(self) -> bytes:
        """安全なコピーを取る。取った後に `still_valid()` を確認すること。"""
        return bytes(self.buffer)

    def still_valid(self) -> bool:
        """読んでいる間に上書きされていないか（seqlock の後段）。"""
        seq, _, _, _ = self.ring._read_slot_meta(self.desc.slot)
        return seq == self._seq_at_read and seq % 2 == 0


class FrameRing:
    """画像 N枚のリング。**書き手は1つだけ**（複数書き手は想定していない）。"""

    def __init__(self, shm, owner: bool) -> None:
        self._shm = shm
        self._owner = owner
        self._closed = False
        #: close() が参照残りで失敗したか。**握り潰さず呼び出し側に見せる**
        self.close_blocked = False

        raw = bytes(shm.buf[:HEADER_SIZE])
        (magic, ver, hdr, n_slots, slot_bytes, width, height, stride,
         fmt, _wseq) = _S_HEADER.unpack(raw)
        if magic != MAGIC:
            raise ValueError(f"共有メモリ {shm.name} はリングではありません")
        if ver > FORMAT_VERSION:
            raise ValueError(f"未対応のリング版 {ver} > {FORMAT_VERSION}")

        self.name = shm.name
        self.n_slots = n_slots
        self.slot_bytes = slot_bytes
        self.width = width
        self.height = height
        self.stride = stride
        self.fmt = fmt.decode("ascii").rstrip("\x00 ")
        self._meta_off = hdr
        self._data_off = _align(hdr + n_slots * SLOT_META_SIZE)

    # ── 生成・接続 ──

    @classmethod
    def create(cls, name: str, width: int, height: int, fmt: str = "RGB888",
               *, n_slots: int = 8, stride: int | None = None) -> "FrameRing":
        """新しいリングを作る。**同名が残っていたら消してから作り直す。**

        前回異常終了して `/dev/shm` に残骸があっても起動できるようにするため。
        """
        bpp = _BYTES_PER_PIXEL.get(fmt)
        if bpp is None:
            raise ValueError(f"未知の画素形式: {fmt}")
        if n_slots < 2:
            raise ValueError("n_slots は 2 以上（1枚だと必ず読み書きが衝突する）")
        stride = stride or width * bpp
        slot_bytes = _align(stride * height)
        total = _align(HEADER_SIZE + n_slots * SLOT_META_SIZE) + n_slots * slot_bytes

        try:
            stale = _open_shm(name)
            stale.close()
            stale.unlink()
        except FileNotFoundError:
            pass

        shm = _open_shm(name, create=True, size=total)
        _S_HEADER.pack_into(shm.buf, 0, MAGIC, FORMAT_VERSION, HEADER_SIZE,
                            n_slots, slot_bytes, width, height, stride,
                            fmt.encode("ascii")[:8].ljust(8, b"\x00"), 0)
        meta_off = HEADER_SIZE
        for i in range(n_slots):
            _S_SLOT.pack_into(shm.buf, meta_off + i * SLOT_META_SIZE, 0, 0, 0, 0)
        return cls(shm, owner=True)

    @classmethod
    def attach(cls, name: str) -> "FrameRing":
        """既存のリングに読み手として接続する。"""
        return cls(_open_shm(name), owner=False)

    # ── 内部 ──

    def _slot_meta_off(self, slot: int) -> int:
        return self._meta_off + slot * SLOT_META_SIZE

    def _read_slot_meta(self, slot: int) -> tuple[int, int, int, int]:
        return _S_SLOT.unpack_from(self._shm.buf, self._slot_meta_off(slot))

    def _slot_data(self, slot: int) -> memoryview:
        off = self._data_off + slot * self.slot_bytes
        return self._shm.buf[off: off + self.slot_bytes]

    @property
    def write_seq(self) -> int:
        """これまでに書き込まれたフレーム数。"""
        return _S_WRITE_SEQ.unpack_from(self._shm.buf, _WRITE_SEQ_OFF)[0]

    @property
    def nbytes(self) -> int:
        """共有メモリ全体の大きさ。"""
        return self._shm.size

    # ── 書き込み ──

    def write(self, data, t_capture_ns: int, frame_id: int = 0) -> FrameDesc:
        """1フレーム書き込む。`data` はバッファプロトコルを持つもの。

        カメラの DMA バッファは自前の共有メモリではないので、**ここで1回だけ
        memcpy が入る**。これは避けられない（28 MB/s なので負荷としては軽い）。
        """
        if self._owner is False:
            raise RuntimeError("読み手として接続したリングには書き込めません")
        view = memoryview(data).cast("B")
        n = view.nbytes
        if n > self.slot_bytes:
            raise ValueError(f"フレームが大きすぎます: {n} > {self.slot_bytes}")

        seq = self.write_seq
        slot = seq % self.n_slots
        off = self._slot_meta_off(slot)
        cur = _S_SLOT.unpack_from(self._shm.buf, off)[0]

        # seqlock: 奇数にしてから書き、書き終わってから偶数に戻す
        _S_SLOT.pack_into(self._shm.buf, off, cur + 1, t_capture_ns, frame_id, n)
        self._slot_data(slot)[:n] = view
        _S_SLOT.pack_into(self._shm.buf, off, cur + 2, t_capture_ns, frame_id, n)

        # 最後に write_seq を進める。ここまでは読み手にこのフレームは見えない
        _S_WRITE_SEQ.pack_into(self._shm.buf, _WRITE_SEQ_OFF, seq + 1)
        return FrameDesc(self.name, slot, seq + 1, t_capture_ns, frame_id,
                         self.width, self.height, self.fmt, self.stride, n)

    # ── 読み出し ──

    def latest(self) -> FrameRef | None:
        """最新フレームへのゼロコピー参照。まだ何も無ければ None。"""
        seq = self.write_seq
        if seq == 0:
            return None
        slot = (seq - 1) % self.n_slots
        lock, t_cap, fid, nbytes = self._read_slot_meta(slot)
        if lock % 2 == 1:
            return None                      # 書き込み中
        desc = FrameDesc(self.name, slot, seq, t_cap, fid,
                         self.width, self.height, self.fmt, self.stride, nbytes)
        return FrameRef(self, desc, lock)

    def latest_copy(self, retries: int = 8) -> tuple[FrameDesc, bytes] | None:
        """最新フレームを**安全にコピー**して返す。上書きされたら再試行する。

        ゼロコピーの参照を扱いたくない場面（記録・送信）用。
        """
        for _ in range(retries):
            ref = self.latest()
            if ref is None:
                return None
            data = ref.copy()
            if ref.still_valid():
                return ref.desc, data
        return None

    def read(self, desc: FrameDesc) -> FrameRef | None:
        """バスから受け取った説明を使って該当スロットを参照する。

        **説明が指すフレームがまだ生きているか**（上書きされていないか）を確かめる。
        """
        if desc.slot >= self.n_slots:
            return None
        behind = self.write_seq - desc.seq
        if behind < 0 or behind >= self.n_slots:
            return None                      # 一周してしまった
        lock, t_cap, fid, nbytes = self._read_slot_meta(desc.slot)
        if lock % 2 == 1 or fid != desc.frame_id:
            return None
        return FrameRef(self, desc._replace(nbytes=nbytes, t_capture_ns=t_cap), lock)

    # ── 後始末 ──

    def close(self) -> None:
        """共有メモリを閉じる。

        **閉じる前に `FrameRef` と numpy 配列を手放すこと。**
        Python のバッファプロトコル上、共有メモリを参照している view が
        1つでも生きていると閉じられない（`BufferError`）。
        黙って握り潰すと、後で `SharedMemory.__del__` が
        「Exception ignored」を吐いて原因が分からなくなるので、
        **握り潰さずに `close_blocked` を立てて呼び出し側に見せる。**
        """
        if self._closed:
            return
        try:
            self._shm.close()
        except BufferError:
            self.close_blocked = True
            return                     # _closed は立てない（後で閉じ直せる）
        except Exception:
            pass
        self._closed = True

    def unlink(self) -> None:
        """共有メモリを消す。**作った側だけが呼ぶ。**"""
        self.close()
        if self._owner:
            try:
                self._shm.unlink()
            except FileNotFoundError:
                pass

    def __enter__(self) -> "FrameRing":
        return self

    def __exit__(self, *exc) -> None:
        self.unlink() if self._owner else self.close()

    def __repr__(self) -> str:
        return (f"<FrameRing {self.name} {self.width}x{self.height} {self.fmt} "
                f"slots={self.n_slots} seq={self.write_seq}>")


def _align(n: int, to: int = _ALIGN) -> int:
    return (n + to - 1) // to * to
