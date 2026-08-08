"""replay_node — 記録した `.sfl` を実時間で流し、io_node の代わりをする。

    python3 -m raspi.nodes.replay_node logs/run.sfl              # 等速再生
    python3 -m raspi.nodes.replay_node logs/run.sfl --speed 4    # 4倍速
    python3 -m raspi.nodes.replay_node logs/run.sfl --speed 0    # 最速（待たない）
    python3 -m raspi.nodes.replay_node logs/run.sfl --start 12 --end 18
    python3 -m raspi.nodes.replay_node logs/run.sfl --verify     # 記録と突き合わせる

**旧シミュレータを削除した今、これが事実上のシミュレータ代わり**
（`docs/architecture.md` §11）。実車・STM32・シリアルが無くても、知覚・経路生成・
GUI を本物のセンサノイズ入りのデータで開発できる。pyserial にも依存しない。

## 実機と同じ結果になるようにしてあること

- 受信の解釈・時刻同期・health 判定は **io_node と同じ `LinkTracker`** を使う。
  2つ書けば必ずズレるので、共有している
- **ログ時刻の座標系のまま動かす。** 「今」は壁時計ではなく再生カーソル。
  こうすると health の遷移も時刻同期の推定値も記録時と同じ値になり、
  **バグ再現が確定的になる**（何度流しても同じ結果）。壁時計は待ち時間の計算にしか使わない
- **PING の T1 は TX レコードから復元する。** PONG(RX) と突き合わせるので、
  時刻同期は記録時とまったく同じ4タイムスタンプで動く
- **フレームが無い区間でもカーソルを進めて health を更新する。**
  リンクが完全に切れた区間は「レコードが1つも無い」ので、
  次のフレームまで待つと FAULT を見逃す

## 復元できないもの

CRC エラー・長さエラー・再同期バイト数は**捨てたフレーム**なので RX レコードに
存在しない。これらは 1Hz の `linkstats` イベントから復元する（そのために記録している）。
再生側で数え直しているのは、フレーム数と種別だけ。
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.core.link_tracker import (  # noqa: E402
    LinkTracker,
    format_status,
    write_status,
)
from raspi.proto import packets  # noqa: E402
from raspi.proto.framing import RxStats  # noqa: E402
from raspi.rec import FrameLogReader, Kind  # noqa: E402

__all__ = ["ReplayNode"]

NS = 1_000_000_000

#: カーソルを進める最大の刻み。ここが荒いと、フレームの無い区間で
#: health の遷移する時刻がずれる（100ms 判定に対して十分細かくする）
_STEP_NS = 10 * 1_000_000

_STATUS_WALL_S = 0.5


class ReplayNode:
    """`.sfl` を読んで `LinkTracker` に流す。

    :param speed: 再生倍率。**0 なら待たずに最速**（テスト・一括処理向け）
    :param start_s: ここまではカーソルを進めずに早送りする（状態は温めておく）
    :param end_s: ここで打ち切る
    :param on_telemetry: `(Telemetry, t_pi_ns)`
    :param on_frame: 受信フレームごとに `(t_ns, type, seq, msg)`
    :param on_tx: 記録された送信フレームごとに `(t_ns, type, seq, msg)`。
        **再生では何も送信しない**が、何が指令されていたかは下流に見せる
    :param on_event: META 以外の JSON イベントごとに `(t_ns, dict)`
    """

    def __init__(self, path: str | Path, *, speed: float = 1.0,
                 start_s: float = 0.0, end_s: float | None = None,
                 on_telemetry=None, on_frame=None, on_tx=None, on_event=None) -> None:
        self.path = Path(path)
        self.speed = speed
        self.start_s = start_s
        self.end_s = end_s
        self.on_tx = on_tx
        self.on_event = on_event

        self.tracker = LinkTracker(on_telemetry=on_telemetry, on_frame=on_frame)
        self.state = self.tracker.state
        self.sync = self.tracker.sync

        #: linkstats イベントから復元した受信統計（再生側では数え直せない）
        self.recorded_stats = RxStats()
        #: ログに残っていた最後の linkstats（時刻同期の推定値の比較に使う）
        self.recorded_sync: dict | None = None
        self.meta: dict = {}
        self.tx_counts: dict[int, int] = {}
        self.rx_records = 0
        #: 再生中に観測した health 遷移 `(t_rel_s, from, to)`
        self.health_changes: list[tuple[float, str, str]] = []
        #: ログに書かれていた health 遷移（記録時のもの）
        self.logged_health: list[tuple[float, str, str]] = []

        self._running = False
        self._cursor_ns = 0
        self._t0_ns = 0
        self._wall0 = 0.0
        self._next_status_wall = 0.0
        self._paced = False       # start_s に到達したか

    # ── 時刻 ──

    def rel_s(self, t_ns: int) -> float:
        return (t_ns - self._t0_ns) / 1e9

    def _wall_deadline(self, t_ns: int) -> float:
        return self._wall0 + (t_ns - self._t0_ns - self.start_s * 1e9) / 1e9 / self.speed

    def _advance_to(self, t_ns: int, status_cb) -> None:
        """カーソルを t_ns まで進める。刻みごとに health と表示を更新する。

        フレームの無い区間でもここを通るので、リンク断が FAULT として見える。
        """
        while self._running and self._cursor_ns < t_ns:
            nxt = min(t_ns, self._cursor_ns + _STEP_NS)
            if self._paced and self.speed > 0:
                # 待ちは常に「開始時刻からの絶対締切」に対して計算する。
                # 差分を積み上げると sleep の誤差が溜まって再生が遅れていく
                d = self._wall_deadline(nxt) - time.monotonic()
                if d > 0:
                    time.sleep(d)
            self._cursor_ns = nxt
            self._tick(status_cb)

    def _tick(self, status_cb) -> None:
        changed = self.tracker.update_health(self._cursor_ns)
        if changed is not None:
            prev = self.health_changes[-1][2] if self.health_changes else "INIT"
            self.health_changes.append((self.rel_s(self._cursor_ns), prev, changed))
        if status_cb:
            w = time.monotonic()
            if w >= self._next_status_wall:
                status_cb(self)
                self._next_status_wall = w + _STATUS_WALL_S

    # ── 本体 ──

    def run(self, status_cb=None) -> None:
        self._running = True
        with FrameLogReader(self.path) as reader:
            self.meta = reader.meta
            self._t0_ns = reader.header.t0_mono_ns
            self._cursor_ns = self._t0_ns
            self._wall0 = time.monotonic()

            for rec in reader:
                if not self._running:
                    break
                rel = self.rel_s(rec.t_ns)
                if self.end_s is not None and rel > self.end_s:
                    break

                if not self._paced and rel >= self.start_s:
                    # 早送り終わり。ここから実時間で流す
                    self._paced = True
                    self._wall0 = time.monotonic()
                    self._cursor_ns = max(self._cursor_ns, rec.t_ns)

                self._advance_to(rec.t_ns, status_cb)
                self._handle(rec)

            self.truncated = reader.truncated

        # 最後のフレームから end_s（または記録終端）までも進めて health を確定させる
        if self._running and self.end_s is not None:
            self._advance_to(self._t0_ns + int(self.end_s * 1e9), status_cb)
        self._running = False

    def _handle(self, rec) -> None:
        if rec.kind == Kind.RX:
            self.rx_records += 1
            self.tracker.feed(rec.t_ns, rec.type, rec.seq, rec.payload)
        elif rec.kind == Kind.TX:
            self.tx_counts[rec.type] = self.tx_counts.get(rec.type, 0) + 1
            msg = rec.decode()
            if isinstance(msg, packets.Ping):
                # 記録された送信時刻がそのまま T1。PONG と突き合わせれば
                # 時刻同期は記録時と同じ入力で動く
                self.tracker.note_ping_sent(msg.ping_id, rec.t_ns)
            if self.on_tx:
                self.on_tx(rec.t_ns, rec.type, rec.seq, msg)
        elif rec.kind == Kind.EVENT:
            self._handle_event(rec)

    def _handle_event(self, rec) -> None:
        body = rec.json()
        name = body.get("name")
        if name == "linkstats":
            for k, v in body.get("rx", {}).items():
                if hasattr(self.recorded_stats, k):
                    setattr(self.recorded_stats, k, v)
            self.recorded_sync = body.get("sync")
        elif name == "health":
            self.logged_health.append((self.rel_s(rec.t_ns),
                                       body.get("from", "?"), body.get("to", "?")))
        if self.on_event:
            self.on_event(rec.t_ns, body)

    def stop(self) -> None:
        self._running = False


# ── 記録との突き合わせ ────────────────────────────────────────────────

def verify(node: ReplayNode) -> bool:
    """再生結果がログの記録内容と辻褄が合うかを見る。

    合わなければ**ログに足りない情報がある**ということで、記録側の設計の問題。
    再生器の問題として片付けないこと。
    """
    ok = True
    print("\n=== 記録との突き合わせ ===")

    logged = node.recorded_stats.frame_ok
    if logged:
        match = node.rx_records == logged
        ok &= match
        print(f"{'✓' if match else '✗'} 受信フレーム数: 再生 {node.rx_records} / "
              f"記録 frame_ok {logged}"
              + ("" if match else "  ← io_node が解析したのに記録できていないフレームがある"))
    else:
        print("— linkstats イベントが無いので受信数を照合できない")

    if node.recorded_sync:
        rec_drift = node.recorded_sync.get("drift_ppm")
        now_drift = node.sync.drift_ppm
        if rec_drift is None or now_drift is None:
            print(f"— ドリフト推定を照合できない（記録 {rec_drift} / 再生 {now_drift}）")
        else:
            match = abs(rec_drift - now_drift) < 1.0
            ok &= match
            print(f"{'✓' if match else '✗'} クロックドリフト: 再生 {now_drift:+.2f}ppm / "
                  f"記録 {rec_drift:+.2f}ppm"
                  + ("" if match else "  ← PING/PONG の記録が不足している"))
        rec_n = node.recorded_sync.get("n")
        match = rec_n == node.sync.n_samples
        ok &= match
        print(f"{'✓' if match else '✗'} 時刻同期サンプル数: 再生 {node.sync.n_samples} / "
              f"記録 {rec_n}")

    if node.logged_health:
        replayed = [(f, t) for _, f, t in node.health_changes]
        recorded = [(f, t) for _, f, t in node.logged_health]
        match = replayed == recorded
        ok &= match
        print(f"{'✓' if match else '✗'} health 遷移: 再生 {replayed} / 記録 {recorded}")

    print("=> " + ("一致。ログだけで実機の挙動を再現できている。"
                   if ok else "**不一致。記録している情報が足りない。**"))
    return ok


def _status_line(node: ReplayNode) -> None:
    write_status(f"{node.rel_s(node._cursor_ns):7.2f}s "
                 + format_status(node.state, node.sync, node.recorded_stats))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", type=Path)
    ap.add_argument("--speed", type=float, default=1.0,
                    help="再生倍率。0 で最速（既定 1.0）")
    ap.add_argument("--start", type=float, default=0.0, help="開始位置 [s]")
    ap.add_argument("--end", type=float, default=None, help="終了位置 [s]")
    ap.add_argument("--quiet", action="store_true", help="ライブ表示なし")
    ap.add_argument("--verify", action="store_true", help="記録内容と突き合わせる")
    args = ap.parse_args()

    if not args.path.exists():
        print(f"ファイルがありません: {args.path}", file=sys.stderr)
        return 2

    node = ReplayNode(args.path, speed=args.speed,
                      start_s=args.start, end_s=args.end)
    signal.signal(signal.SIGINT, lambda *_: node.stop())

    print(f"# replay {args.path} speed={'最速' if args.speed == 0 else f'x{args.speed}'}")
    if args.start or args.end is not None:
        print(f"# 区間 {args.start}s .. {args.end if args.end is not None else '終端'}s")

    t_wall = time.monotonic()
    node.run(status_cb=None if args.quiet else _status_line)
    t_wall = time.monotonic() - t_wall

    print(f"\n\n=== 再生結果 ({t_wall:.2f}s で {node.rel_s(node._cursor_ns):.2f}s 分) ===")
    if node.meta:
        print("記録元: " + " ".join(f"{k}={v}" for k, v in node.meta.items()
                                    if k in ("t0_local", "port", "baud", "node")))
    rx = " ".join(f"{packets.BY_TYPE[t].NAME}={n}"
                  for t, n in sorted(node.state.counts.items()) if t in packets.BY_TYPE)
    tx = " ".join(f"{packets.BY_TYPE[t].NAME}={n}"
                  for t, n in sorted(node.tx_counts.items()) if t in packets.BY_TYPE)
    print(f"RX {node.rx_records}: {rx}")
    print(f"TX: {tx}")
    print(f"health={node.state.health} 遷移={node.health_changes}")
    print(f"time sync: n={node.sync.n_samples} offset={node.sync.offset_ns} ns "
          f"drift={node.sync.drift_ppm} ppm")
    if node.state.lidar_sectors:
        print(f"lidar sectors seen: {len(node.state.lidar_sectors)}/12")
    if getattr(node, "truncated", False):
        print("!! ログの末尾が切れている（記録中に落ちた可能性）")

    if args.verify:
        return 0 if verify(node) else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
