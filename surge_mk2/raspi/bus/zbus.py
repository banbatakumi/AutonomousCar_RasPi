"""内部バス — ZeroMQ PUB/SUB の薄いラッパ（`docs/architecture.md` §6.2）。

輸送層は ZeroMQ に任せ、**その上のメッセージ規約だけを自作する**。
画像は共有メモリ（`shm_ring`）なので、ここを通るのは数百バイト以下のものだけ。

## ワイヤ形式は「1フレーム」にする

    b"vehicle_state" + b"\\x00" + msgpack(payload)

素直に書くなら multipart（`[topic, payload]`）だが、**ZMQ_CONFLATE は
multipart メッセージに対応していない。** 制御系に CONFLATE は必須なので、
トピックとペイロードを1フレームに詰める形にした。

SUB の購読は**先頭バイト列の前方一致**なので、これでフィルタもそのまま効く。
区切りに NUL を入れているのは、`scan` の購読が `scan_debug` にまで一致するのを
防ぐため（トピック名に NUL は現れない）。

## 購読は「トピック1本につきソケット1本」

**CONFLATE はソケット単位であってトピック単位ではない。** 1本の SUB に
`vehicle_state` と `scan` を両方購読させて CONFLATE を掛けると、
片方が来るたびにもう片方が捨てられ、**LiDAR が来るたびに車両状態が消える**。
しかも見た目は動いているので気づきにくい。

トピックごとにソケットを分ければ、この事故が構造的に起きない。
ソケットは軽い（数個〜十数個）ので、素直さを取る。

## 配送ポリシーは購読側が決める

同じトピックを、購読者ごとに違うポリシーで受けられる（§6.2）。

| ポリシー | 用途 | 設定 |
|---|---|---|
| `LATEST` | 制御・GUI 表示 | `CONFLATE=1`（キュー長1・上書き） |
| `RELIABLE` | ロガー・記録 | `RCVHWM` 大きめ |

制御系が「2秒前の点群で舵を切る」のを防ぐのが `LATEST` の目的。

## slow joiner

PUB は接続が確立するまでの間、送ったものを捨てる。50Hz の連続ストリームなら
数十msの取りこぼしなので実害は無いが、**「起動して1発だけ送って終わる」用途では
届かない。** `Publisher.wait_ready()` で待てるようにしてある。
"""

from __future__ import annotations

import os
import threading
import time
from pathlib import Path

import msgspec
import zmq

from ..msgs.types import MsgBase, type_for_topic

__all__ = ["Publisher", "Subscriber", "Policy", "LATEST", "RELIABLE",
           "endpoint_for_node", "endpoints_for_topic", "TOPIC_OWNER",
           "SEP", "encode", "decode"]

SEP = b"\x00"

#: 環境変数で差し替えられる。テストは `tmpdir` を指してプロセス間干渉を避ける
ENV_DIR = "SURGE_BUS_DIR"
#: `tcp://` に切り替える（別マシンから覗きたいとき。既定は ipc）
ENV_TCP = "SURGE_BUS_TCP"

_DEFAULT_DIR = "/tmp/surge-bus"

#: ノードごとに1本の endpoint を bind する。**トピックごとではない。**
#: PUB を1プロセス1本にしておくと、購読側は接続先が固定されて配線が単純になる
_NODE_TCP_PORT = {"io": 5570, "camera": 5571, "control": 5572, "planning": 5573,
                  "cam_perception": 5574, "line_perception": 5575, "test": 5579}

#: トピック → どのノードが publish するか
#:
#: **`cmd` の発行元は `control`（telemetry_node）だけ。** planning_node は
#: `auto/cmd` に出し、engage されている間だけ telemetry_node が `cmd` へ中継する。
#: 2つのノードが同じ `cmd` に publish すると、購読側からは区別できないまま
#: 50Hz で交互に上書きし合い、「勝手にハンドルが戻る」という再現困難な症状になる
#: （`docs/architecture.md` §7.4 / `telemetry_node` の `_cmd_pump`）。
#:
#: ★ **完全一致で先に引けるキーを持たせること。** 前方一致フォールバック
#: （`endpoints_for_topic()` 参照）は辞書の並び順に依存する上、
#: `"scan/cam"` が `"scan"` の前方一致に誤って拾われ `cam_perception` ではなく
#: `io` に解決される、という事故が実際に起きた（2026-08-23、`scan/cam` が
#: 常に空のまま `ftg_cam` が動かなかった）。新しいトピックを足すときは、
#: 既存キーの前方一致に誤って拾われないか必ず確認すること
TOPIC_OWNER: dict[str, str] = {
    "vehicle_state": "io",
    "scan": "io",
    "diag/link": "io",
    "cmd": "control",
    "log/ctrl": "control",
    "auto/ctrl": "control",
    "ui/event": "control",
    "cam/config": "control",
    "cam/model": "control",
    "e2e/model": "control",
    "auto/cmd": "planning",
    "auto/state": "planning",
    "auto/map": "planning",
    "image/": "camera",
    "image/front": "camera",
    "image/rear": "camera",
    "scan/cam": "cam_perception",
    "line/cam": "line_perception",
}


def bus_dir() -> Path:
    d = Path(os.environ.get(ENV_DIR, _DEFAULT_DIR))
    d.mkdir(parents=True, exist_ok=True)
    return d


def endpoint_for_node(node: str) -> str:
    """ノード名 → bind/connect する endpoint。"""
    if os.environ.get(ENV_TCP):
        port = _NODE_TCP_PORT.get(node)
        if port is None:
            raise KeyError(f"tcp モードでは未知のノード: {node!r}")
        return f"tcp://127.0.0.1:{port}"
    return f"ipc://{bus_dir() / (node + '.ipc')}"


def endpoints_for_topic(topic: str) -> list[str]:
    """トピック → 接続すべき endpoint の一覧。

    `hb/` は全ノードが publish するので全 endpoint に繋ぐ。
    """
    if topic.startswith("hb/") or topic == "hb/":
        return [endpoint_for_node(n) for n in
                ("io", "camera", "control", "planning",
                 "cam_perception", "line_perception")]
    owner = TOPIC_OWNER.get(topic)
    if owner is None:
        # 前方一致（"image/" のような接頭辞購読）
        for pref, n in TOPIC_OWNER.items():
            if topic.startswith(pref):
                owner = n
                break
    if owner is None:
        raise KeyError(f"publish 元が不明なトピック: {topic!r}")
    return [endpoint_for_node(owner)]


# ── ワイヤ形式 ────────────────────────────────────────────────────────

_encoder = msgspec.msgpack.Encoder()
_decoders: dict[type, msgspec.msgpack.Decoder] = {}


def encode(topic: str, msg: MsgBase) -> bytes:
    return topic.encode() + SEP + _encoder.encode(msg)


def decode(raw: bytes) -> tuple[str, MsgBase]:
    """1フレーム → `(topic, msg)`。

    :raises ValueError: 区切りが見つからない（別プロトコルが混ざっている）
    """
    i = raw.find(SEP)
    if i < 0:
        raise ValueError("区切り(NUL)が無いフレーム。バスに別の形式が流れている")
    topic = raw[:i].decode()
    cls = type_for_topic(topic)
    dec = _decoders.get(cls)
    if dec is None:
        dec = _decoders[cls] = msgspec.msgpack.Decoder(cls)
    return topic, dec.decode(raw[i + 1:])


# ── ポリシー ──────────────────────────────────────────────────────────

class Policy(msgspec.Struct, frozen=True):
    """購読ポリシー。

    :param conflate: 最新1件だけ持つ（古いものは捨てる）。制御・表示はこちら
    :param hwm: 受信キューの上限。`conflate=False` のときだけ効く
    """

    conflate: bool = True
    hwm: int = 1000


#: 制御・GUI 表示用。**古いデータは捨てて最新だけ**
LATEST = Policy(conflate=True)
#: ロガー・記録用。**1フレームも落としたくない**
RELIABLE = Policy(conflate=False, hwm=100_000)


# ── Publisher ────────────────────────────────────────────────────────

class Publisher:
    """1ノードにつき1本。`t_pub` と `seq` はここで押す。

    :param node: `endpoint_for_node()` に渡すノード名
    :param hwm: 送信キューの上限。**溢れたら捨てる**（`ZMQ_XPUB_NODROP` は使わない）。
        購読者が遅いせいで publish 側の制御ループが止まる方が危険なため
    :param thread_safe: 複数スレッドから `send()` するなら True。
        **ZeroMQ のソケットはスレッドセーフではない。** camera_node のように
        カメラ1台につき1スレッドで回す構成では必ず立てること
    """

    def __init__(self, node: str, *, endpoint: str | None = None,
                 hwm: int = 1000, ctx: zmq.Context | None = None,
                 thread_safe: bool = False) -> None:
        self.node = node
        self.endpoint = endpoint or endpoint_for_node(node)
        self._own_ctx = ctx is None
        self.ctx = ctx or zmq.Context.instance()
        self.sock = self.ctx.socket(zmq.PUB)
        self.sock.setsockopt(zmq.SNDHWM, hwm)
        self.sock.setsockopt(zmq.LINGER, 0)
        self.sock.bind(self.endpoint)
        self._seq: dict[str, int] = {}
        self._lock = threading.Lock() if thread_safe else None
        self.sent = 0

    def send(self, topic: str, msg: MsgBase) -> MsgBase:
        """`t_pub` / `seq` を押して送る。渡した msg をそのまま返す（記録に使えるように）。"""
        if self._lock is not None:
            with self._lock:
                return self._send(topic, msg)
        return self._send(topic, msg)

    def _send(self, topic: str, msg: MsgBase) -> MsgBase:
        seq = self._seq.get(topic, 0) + 1
        self._seq[topic] = seq
        msg.seq = seq
        msg.t_pub = time.monotonic_ns()
        if msg.t_capture == 0:
            msg.t_capture = msg.t_pub
        try:
            self.sock.send(encode(topic, msg), zmq.NOBLOCK)
            self.sent += 1
        except zmq.Again:
            pass          # HWM 到達。制御ループを止めないため黙って捨てる
        return msg

    def wait_ready(self, delay_s: float = 0.2) -> None:
        """slow joiner の待ち。**単発 publish の前だけ使う。**

        ZeroMQ には「購読者が繋がったか」を PUB から知る手軽な API が無いので、
        素直に待つ。連続ストリームでは不要。
        """
        time.sleep(delay_s)

    def close(self) -> None:
        self.sock.close()
        if self._own_ctx:
            pass          # Context.instance() は共有物なので term しない


# ── Subscriber ───────────────────────────────────────────────────────

class Subscriber:
    """トピックごとに SUB ソケットを1本持つ。

        sub = Subscriber({"vehicle_state": LATEST, "scan": LATEST})
        for topic, msg in sub.poll(timeout_ms=20):
            ...

    :param topics: `{トピック名: Policy}`。リストを渡すと全部 `LATEST` になる。
        末尾が `/` のトピックは**前方一致購読**（`"image/"` で両カメラ）
    """

    def __init__(self, topics, *, ctx: zmq.Context | None = None) -> None:
        if not isinstance(topics, dict):
            topics = {t: LATEST for t in topics}
        self.ctx = ctx or zmq.Context.instance()
        self.poller = zmq.Poller()
        self.socks: dict[str, zmq.Socket] = {}
        #: トピックごとの最新値。`poll()` が更新する
        self.latest: dict[str, MsgBase] = {}
        self.received = 0

        try:
            self._open(topics)
        except Exception:
            self.close()          # 途中で失敗したソケットを残さない
            raise

    def _open(self, topics: dict) -> None:
        for topic, pol in topics.items():
            s = self.ctx.socket(zmq.SUB)
            # CONFLATE は connect より先に設定しないと効かない
            if pol.conflate:
                s.setsockopt(zmq.CONFLATE, 1)
            else:
                s.setsockopt(zmq.RCVHWM, pol.hwm)
            s.setsockopt(zmq.LINGER, 0)
            # 末尾 "/" は接頭辞購読、それ以外は NUL まで含めた完全一致
            prefix = topic.encode() if topic.endswith("/") else topic.encode() + SEP
            s.setsockopt(zmq.SUBSCRIBE, prefix)
            self.socks[topic] = s
            self.poller.register(s, zmq.POLLIN)
            for ep in endpoints_for_topic(topic):
                s.connect(ep)

    def poll(self, timeout_ms: int = 0) -> list[tuple[str, MsgBase]]:
        """届いているものを全部返す。**`LATEST` のトピックは各1件まで。**"""
        out: list[tuple[str, MsgBase]] = []
        ready = dict(self.poller.poll(timeout_ms))
        for topic, s in self.socks.items():
            if s not in ready:
                continue
            while True:
                try:
                    raw = s.recv(zmq.NOBLOCK)
                except zmq.Again:
                    break
                t, msg = decode(raw)
                self.latest[t] = msg
                self.received += 1
                out.append((t, msg))
        return out

    def recv_one(self, timeout_ms: int = 1000) -> tuple[str, MsgBase] | None:
        """1件だけ待って返す。テストと簡易ツール向け。"""
        got = self.poll(timeout_ms)
        return got[0] if got else None

    def close(self) -> None:
        for s in self.socks.values():
            self.poller.unregister(s)
            s.close()
        self.socks.clear()

    def __enter__(self) -> "Subscriber":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
