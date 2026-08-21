"""共有メモリの画像 → JPEG。**telemetry_node と logger_node が共有する。**

同じことを2箇所に書くと、片方だけ色が入れ替わる（BGR/RGB）という直しにくい
バグになる。エンコーダの選択も、共有メモリの読み方も、ここ1箇所に置く。

## エンコーダの選択

`simplejpeg` は picamera2 が依存しているので **Pi には既に入っている。**
Mac で試すときは Pillow に落ちる（カメラが無いので速度は要らない）。

## 読み方は「最新スロットを読んで、読み終えてから検証する」

`FrameRing` は seqlock なので、読んでいる最中に書き手が同じスロットを
上書きしうる。`still_valid()` が False なら**混ざった画像**なので捨てる。
1フレーム落とすのは、色が半分ずれた画像を記録するより安い。

## 掴んだ共有メモリが「作り直された」ことを検出する

`camera_node` を再起動すると `/dev/shm/surge_cam0` は unlink されて作り直されるが、
**既に attach しているプロセスは古い方を掴んだまま**になる（Linux では unlink
されてもマッピングは生きる）。放っておくと**同じ画像を延々と記録し続ける**という、
エラーも出ない一番たちの悪い壊れ方をする。

そこで `ImageRef.ring_seq`（書き手が進めている番号）と、こちらが掴んでいるリングの
`write_seq` を比べ、開きすぎていたら attach し直す。

**同じことが `RemoveIPC` でも起きる**（2026-08-08 に実際に踏んだ）。
systemd-logind の既定 `RemoveIPC=yes` は、そのユーザーの最後のセッションが
終わると uid 所有の共有メモリを全消しするため、`User=pi` の systemd サービスが
使っている `/dev/shm/surge_cam*` まで消える。Pi 側は
`/etc/systemd/logind.conf.d/10-surge-removeipc.conf` で `RemoveIPC=no` にしてある。
"""

from __future__ import annotations

import time

from .cleanup import quiet_close

__all__ = ["make_encoder", "RingJpeg", "STALE_GAP"]

#: `ImageRef.ring_seq` と手元の `write_seq` がこれ以上開いていたら、
#: 掴んでいるのは作り直される前の共有メモリだと判断して attach し直す。
#: 30Hz なら約2秒ぶん。**遅れているだけの購読者を誤検出しない程度に大きく取る**
STALE_GAP = 64


def make_encoder(quality: int):
    """使える JPEG エンコーダを探す。`(encode(arr, colorspace) -> bytes, 実装名)`。

    見つからなければ `(None, None)`。**呼び出し側で「カメラ機能なし」に
    落とせるようにする**ため、例外にはしない。
    """
    try:
        import simplejpeg

        def enc(arr, colorspace):
            return simplejpeg.encode_jpeg(arr, quality=quality, colorspace=colorspace)
        return enc, "simplejpeg"
    except ImportError:
        pass
    try:
        from io import BytesIO

        from PIL import Image

        def enc(arr, colorspace):
            if colorspace == "BGR":
                arr = arr[:, :, ::-1]
            buf = BytesIO()
            Image.fromarray(arr).save(buf, "JPEG", quality=quality)
            return buf.getvalue()
        return enc, "pillow"
    except ImportError:
        return None, None


class RingJpeg:
    """共有メモリリングを名前で開いて、最新フレームを JPEG にする。

        jp = RingJpeg(quality=70)
        if jp.ok:
            got = jp.encode_latest("surge_cam0")     # (jpeg, t_capture_ns) | None

    リングは名前ごとに1回だけ attach してキャッシュする（開き直しは高くつく）。

    :param quality: JPEG 品質
    """

    def __init__(self, quality: int = 70) -> None:
        self.encode, self.impl = make_encoder(quality)
        self._rings: dict[str, object] = {}
        #: 上書きに当たって捨てたフレーム数。診断に使う
        self.torn = 0
        self.errors = 0
        #: 共有メモリが作り直されて attach し直した回数
        self.reattached = 0
        #: 直近の失敗理由。**数だけ数えても原因は分からない**ので文字列で残す
        #: （`errors=448` だけ見えても「なぜ」が分からず時間を溶かす）
        self.last_error: str = ""
        #: 実際にエンコードした枚数と、それに使った CPU 時間 [s]。
        #:
        #: **telemetry_node と logger_node が同じフレームを別々にエンコードしている**
        #: （2026-08-21 のレビュー 🟡7）。素朴に共通化すると
        #: 「誰も見ていない間も camera_node が焼き続ける」形になって idle の CPU が
        #: むしろ増えるので、**まず本当にボトルネックかを測れるようにした。**
        #: `process_time` を使うのは、待ちを含む wall clock では
        #: 「エンコードが重い」と「バスが詰まっている」が区別できないため
        self.encoded = 0
        self.encode_cpu_s = 0.0

    @property
    def cpu_per_frame_ms(self) -> float:
        """1枚あたりのエンコード CPU 時間 [ms]。**0 枚なら 0。**

        カメラ2台ぶんの実測値がここに出る。`image_hz` を掛ければ、
        二重エンコードで何%の CPU を無駄にしているかが直接わかる。
        """
        return (self.encode_cpu_s / self.encoded * 1000.0) if self.encoded else 0.0

    def stats(self) -> dict:
        """診断・終了統計に出す形。**捨てた数と「なぜ」を必ず並べる。**"""
        return {
            "impl": self.impl,
            "encoded": self.encoded,
            "cpu_per_frame_ms": round(self.cpu_per_frame_ms, 2),
            "cpu_total_s": round(self.encode_cpu_s, 2),
            "torn": self.torn,
            "errors": self.errors,
            "reattached": self.reattached,
            "last_error": self.last_error,
        }

    @property
    def ok(self) -> bool:
        return self.encode is not None

    def encode_latest(self, shm_name: str,
                      expect_seq: int | None = None) -> tuple[bytes, int] | None:
        """`(JPEG, t_capture_ns)`。読めなければ None。**例外は投げない。**

        記録や配信のために毎周叩かれるので、1枚失敗しただけでノードを
        落とさない。理由が要るときは `errors` / `torn` / `last_error` を見る。

        :param expect_seq: 書き手が今どこまで進んでいるか（`ImageRef.ring_seq`）。
            渡すと**共有メモリが作り直されたことを検出**して attach し直す
        """
        if self.encode is None:
            return None
        try:
            from ..bus import FrameRing

            ring = self._rings.get(shm_name)
            # **差の絶対値で見る。** 作り直された共有メモリは `write_seq` が 0 から
            # 数え直すので、書き手の方が「小さい」形でズレる。片側だけ見ると
            # 一番ありがちな再起動のケースを取りこぼす
            if (ring is not None and expect_seq is not None
                    and abs(expect_seq - ring.write_seq) > STALE_GAP):
                # 掴んでいるのは作り直される前の共有メモリ。
                # **放置すると同じ画像を延々と記録し続ける**
                self._drop(shm_name)
                self.reattached += 1
                ring = None
            if ring is None:
                ring = self._rings[shm_name] = FrameRing.attach(shm_name)
            frame = ring.latest()
            if frame is None:
                return None
            arr = frame.as_array()
            # `fmt` は**メモリ上の実際のバイト順**。名前を信じて入れ替えない
            colorspace = "BGR" if frame.desc.fmt.startswith("BGR") else "RGB"
            t0 = time.process_time()
            jpg = self.encode(arr, colorspace)
            self.encode_cpu_s += time.process_time() - t0
            self.encoded += 1
            t = frame.desc.t_capture_ns
            del arr
            if not frame.still_valid():
                self.torn += 1
                return None
            return jpg, t
        except Exception as e:
            self.errors += 1
            self.last_error = f"{type(e).__name__}: {e}"
            # 掴み直せば直る類（共有メモリの作り直し・消失）があるので捨てておく
            self._drop(shm_name)
            return None

    def _drop(self, shm_name: str) -> None:
        ring = self._rings.pop(shm_name, None)
        if ring is not None:
            with quiet_close(f"FrameRing {shm_name}"):
                ring.close()

    def close(self) -> None:
        for name in list(self._rings):
            self._drop(name)
