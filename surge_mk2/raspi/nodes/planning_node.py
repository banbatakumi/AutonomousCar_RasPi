"""planning_node — 点群から走行指令を作る（`docs/architecture.md` §6.1 / §8）。

    .venv/bin/python -m raspi.nodes.planning_node
    .venv/bin/python -m raspi.nodes.planning_node --mode ftg   # 起動時のモードを指定
    .venv/bin/python -m raspi.nodes.planning_node --quiet

- **購読**: `vehicle_state`・`auto/ctrl`（どのモードで走るかの意思）・
  登録されている全 planner の `input_topic`（既定 `scan`。カメラ系 planner は
  `scan/cam` を使う。`input_topics()` が和集合を作る）
- **発行**: `auto/cmd`（走行指令）・`auto/state`（判断の根拠）・
  `auto/map`（地図と経路。**変わったときだけ**）・`hb/planning`

アルゴリズムそのものは `raspi/auto/` にあり、このノードは配線だけを持つ。

## `cmd` ではなく `auto/cmd` に出す

**`cmd` の発行元は telemetry_node だけ**にしてある。ここが直接 `cmd` に出すと、
telemetry_node も 50Hz で `cmd`（操縦者不在なら DISARM）を流しているので、
購読側からは区別できないまま交互に上書きし合う。「勝手にハンドルが戻る」
という再現困難な症状になり、しかも**止める側が負けることがある**。

代わりに、engage されている間だけ telemetry_node が `auto/cmd` を `cmd` へ
中継する（`telemetry_node._cmd_pump`）。人間のデッドマン（GUI が 50Hz で
送り続ける・150ms 途絶で DISARM）は自律走行中もそのまま生きる。

`architecture.md` §6.1 の図は「planning → io 直行」と書いているが、それは
**遅延段数の話**。中継しても増えるのは telemetry_node の 50Hz 周期ぶん
（最悪 20ms）で、指令の生成元が 1 つに定まる価値の方が大きいと判断した。

## engage していなくても計画は回す

`auto/ctrl.engaged` が False でも planner は動かし続け、`auto/state` を出す。
**走らせる前に「今なら何をするか」を GUI で見られる**ようにするため。
ラジコンで手動走行しながら、FTG がどのギャップを選ぶかを眺められる。

このとき `auto/cmd` は `mode=DISARM` / `arm=False` / 速度 0 で出る。
中継側でも engage を確認しているので**二重に閉じている**。

## 計画は入力が来たときだけ

`scan`（LiDAR）は 10Hz、`auto/cmd` の発行は 50Hz。同じ周で 5 回計算しても
答えは変わらないので、**新しい周が来たときだけ** planner を回して結果を
保持する（どのトピックが「入力」かは `Planner.input_topic` が決める。
カメラ系 planner なら `scan/cam` のケイデンスがこれに当たる）。
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.auto import PLANNERS, make_planner, merged_params  # noqa: E402
from raspi.bus import LATEST, Publisher, Subscriber  # noqa: E402
from raspi.core.cleanup import quiet_close  # noqa: E402
from raspi.msgs import AutoCtrl, AutoState, DriveCmd, Heartbeat as HbMsg  # noqa: E402
from raspi.msgs.types import (  # noqa: E402
    TOPIC_AUTO_CMD,
    TOPIC_AUTO_CTRL,
    TOPIC_AUTO_MAP,
    TOPIC_AUTO_STATE,
    TOPIC_E2E_MODEL,
    TOPIC_HB_PREFIX,
    TOPIC_VEHICLE_STATE,
)

__all__ = ["PlanningNode", "input_topics"]

NS = 1_000_000_000
#: `auto/cmd` の発行レート。**telemetry_node の `CMD_PUB_HZ` と揃える**
CMD_HZ = 50
#: `auto/state` の発行レート。点群が 10Hz なのでそれ以上出しても同じ絵になる
STATE_HZ = 10
HB_HZ = 10


def input_topics() -> set[str]:
    """登録されている全 planner が使う `input_topic` の和集合。

    **`Subscriber` は購読するトピック集合をコンストラクタ時に固定する**
    （後から `add_topic()` する口が無い）ので、GUI でモードを切り替えた瞬間に
    そのトピックが未購読、という事態を避けるには起動時に全部読んでおく必要がある。
    LiDAR 系 planner しか無かった今までは `scan` 1つで足りていたが、
    カメラ系 planner（`follow_the_gap_cam.py`）が初めてそれ以外を使う
    （`raspi/auto/base.py` の `Planner.input_topic`）。
    """
    return {cls.input_topic for cls in PLANNERS.values()}


class PlanningNode:
    """`docs/architecture.md` §8 のうち、まだ「行動判断」だけを持つ骨。

    自己位置推定・地図・経路生成は Phase 3 で足す。**足すときもこのノードの
    外側（`raspi/auto/`）に置き、ここは配線のまま保つ。**
    """

    def __init__(self, *, pub: Publisher, sub: Subscriber,
                 mode: str = "", quiet: bool = False) -> None:
        self.pub = pub
        self.sub = sub
        self.quiet = quiet

        #: 現在の意思（`auto/ctrl` の写し）。**telemetry_node が真値を持つ**
        self.ctrl = AutoCtrl(mode=mode, engaged=False)
        self.planner = make_planner(mode)
        self._params: dict[str, float] = merged_params(mode, {})

        #: 最後に出した判断。`auto/cmd` は 50Hz でこれを繰り返す
        self.state = AutoState()
        self._last_scan_seq = -1
        self._last_map_seq = -1
        self._last_plan_ns = 0
        self._plan_dt = 0.0
        self._plans = 0
        self._plan_hz = 0.0
        self._hz_t0 = 0
        self._hz_n = 0

        self._running = False

    # ── 意思の受け取り ──

    def _apply_ctrl(self, c: AutoCtrl) -> None:
        """`auto/ctrl` を反映する。**モードが変わったら planner を作り直す。**

        パラメータだけの変更で作り直すと、スライダを動かすたびに舵の平滑化が
        リセットされて走りがカクつく。`reset()` はモードが変わったときだけ。
        """
        changed = c.mode != self.ctrl.mode
        # disengage も状態を捨てる契機にする。次に engage したときに、
        # **前回の舵の続きから動き出さない**ようにするため
        released = self.ctrl.engaged and not c.engaged
        # 「地図を確定」「地図を削除」は**回数が増えたときだけ**効かせる
        # （`AutoCtrl.freeze_seq` / `clear_seq`）
        freeze = c.freeze_seq > self.ctrl.freeze_seq and not changed
        clear = c.clear_seq > self.ctrl.clear_seq and not changed
        self.ctrl = c
        if clear and self.planner is not None:
            self.planner.request_clear()
            self._last_map_seq = -1
            if not self.quiet:
                print("# 地図を削除（GUI）", flush=True)
        if freeze and self.planner is not None:
            self.planner.request_freeze()
            if not self.quiet:
                print("# 地図を確定（GUI）", flush=True)
        if changed:
            self.planner = make_planner(c.mode)
            self.state = AutoState(mode=c.mode)
            self._last_scan_seq = -1
            self._last_map_seq = -1
            if not self.quiet:
                who = self.planner.name if self.planner else "（なし）"
                print(f"# モード → {who}", flush=True)
            # 作り直した直後に、既に届いている `e2e/model` の意思があれば
            # 即座に反映する（次のポンプまで待たせない）
            m = self.sub.latest.get(TOPIC_E2E_MODEL)
            if m is not None:
                self._apply_e2e_model(m)
        if (changed or released) and self.planner is not None:
            self.planner.reset()
        self._params = merged_params(c.mode, c.params)

    def _apply_e2e_model(self, m) -> None:
        """`e2e/model`（GUIが選んだモデル名）を、対応できる planner にだけ伝える。

        `raspi/auto/` は「新しい planner を足してもこのファイルは触らない」設計
        なので、`E2ELidar` をここで import して特別扱いしない。**`reload_if_changed`
        という名前のメソッドを持っていれば使う**ダックタイピングで済ませる
        （今のところ実装しているのは `E2ELidar` だけ）。
        """
        reload_fn = getattr(self.planner, "reload_if_changed", None)
        if reload_fn is not None:
            reload_fn(m.name)

    # ── 計画 ──

    def _replan(self, now_ns: int) -> None:
        if self.planner is None:
            return
        scan = self.sub.latest.get(self.planner.input_topic)
        if scan is None:
            return
        if scan.seq == self._last_scan_seq:
            return                          # 同じ周。計算しても答えは変わらない
        self._last_scan_seq = scan.seq

        dt = (now_ns - self._last_plan_ns) / NS if self._last_plan_ns else 0.0
        self._last_plan_ns = now_ns

        vs = self.sub.latest.get(TOPIC_VEHICLE_STATE)
        # **`plan()` は engaged を知らない**（`Planner` の共通契約に含めていない
        # ——全 planner に影響するので変えない）。時間で進むステップ列を持つ
        # システム同定 planner（`raspi/auto/sysid_*.py`）だけがこれを必要とする
        # ので、`_apply_e2e_model` の `reload_if_changed` と同じダックタイピング
        # で「持っていれば呼ぶ」形にする（2026-08-31、試験開始前に勝手に進む・
        # 中止しても止まらないという不具合の修正）
        set_engaged_fn = getattr(self.planner, "set_engaged", None)
        if set_engaged_fn is not None:
            set_engaged_fn(self.ctrl.engaged)
        st = self.planner.plan(scan, vs, self._params, dt)
        st.engaged = self.ctrl.engaged
        st.scan_age_ms = (now_ns - scan.t_capture) / 1e6
        self._plans += 1
        self._hz_n += 1
        if self._hz_t0 == 0:
            self._hz_t0 = now_ns
        elif now_ns - self._hz_t0 >= NS:
            self._plan_hz = self._hz_n * NS / (now_ns - self._hz_t0)
            self._hz_t0 = now_ns
            self._hz_n = 0
        st.plan_hz = self._plan_hz
        self.state = st

    def _current(self, now_ns: int) -> AutoState:
        """今この瞬間に有効な判断。**古い点群で走らせない。**

        `_replan` は新しい周が来たときしか動かないので、点群が途絶えても
        直前の判断が残り続ける。ここで鮮度を見て制動に読み替える。
        """
        st = self.state
        if self.planner is None:
            return AutoState(mode=self.ctrl.mode, engaged=self.ctrl.engaged,
                             reason="モードが選ばれていない")
        scan = self.sub.latest.get(self.planner.input_topic)
        if scan is None:
            return AutoState(mode=self.ctrl.mode, planner=self.planner.name,
                             engaged=self.ctrl.engaged, reason="点群がまだ届いていない")
        age = now_ns - scan.t_pub
        if age > self.planner.stale_ms * 1_000_000:
            return AutoState(mode=self.ctrl.mode, planner=self.planner.name,
                             engaged=self.ctrl.engaged,
                             scan_age_ms=age / 1e6, plan_hz=self._plan_hz,
                             reason=f"点群が {age / 1e6:.0f}ms 古い")
        st.engaged = self.ctrl.engaged
        return st

    def _cmd_from(self, st: AutoState) -> DriveCmd:
        """判断 → 走行指令。

        **`ready=False` は必ず制動になる。** planner が「分からない」と言った
        ときに惰行させると、分からないまま壁まで転がる。

        `arm` と `brake_torque` はここでは決めない（`brake_torque=0` は
        「未指定 ＝ STM32 の最大制動」の意味になる）。**中継側が GUI の値を
        使う**ので、planning_node が握るのは速度・舵・制動の有無だけ。
        """
        if not self.ctrl.engaged:
            # engage していないときの指令は「見せるためだけ」の値。DISARM で出す
            return DriveCmd(mode=0, arm=False, source="planning:idle")
        if not st.ready or st.brake:
            return DriveCmd(mode=2, arm=True, brake=True,
                            target_speed=0.0, target_steer=st.target_steer,
                            source=f"planning:{self.ctrl.mode}")
        return DriveCmd(mode=2, arm=True,
                        target_speed=st.target_speed, target_steer=st.target_steer,
                        source=f"planning:{self.ctrl.mode}")

    def _publish_map(self) -> None:
        """地図と経路を `auto/map` へ。**`map_seq` が変わったときだけ。**

        地図は 400×400 あって `auto/state`（10Hz）には載せられないので別トピック。
        planner が変わっていないものを返し続けてよい約束（`Planner.snapshot`）なので、
        ここでは版だけを見る。`scan` を同じ周で2回送らない `telemetry_node._snapshot`
        と同じ流儀。
        """
        if self.planner is None:
            return
        m = self.planner.snapshot()
        if m is None or m.map_seq == self._last_map_seq:
            return
        self._last_map_seq = m.map_seq
        self.pub.send(TOPIC_AUTO_MAP, m)

    # ── ループ ──

    def run(self, duration_s: float | None = None) -> None:
        self._running = True
        t_end = time.monotonic() + duration_s if duration_s else None
        cmd_period = NS // CMD_HZ
        next_cmd = time.monotonic_ns()
        next_state = next_cmd
        next_hb = next_cmd

        while self._running:
            if t_end and time.monotonic() >= t_end:
                break
            # **2ms でブロックする。** 0 にすると CPU を 1 コア食い潰す
            for topic, msg in self.sub.poll(2):
                if topic == TOPIC_AUTO_CTRL:
                    self._apply_ctrl(msg)
                elif topic == TOPIC_E2E_MODEL:
                    self._apply_e2e_model(msg)

            now = time.monotonic_ns()
            self._replan(now)

            if now >= next_cmd:
                next_cmd += cmd_period
                if now - next_cmd > cmd_period * 5:
                    next_cmd = now + cmd_period      # 大きく遅れたら追いつきをやめる
                st = self._current(now)
                self.pub.send(TOPIC_AUTO_CMD, self._cmd_from(st))
                if now >= next_state:
                    next_state = now + NS // STATE_HZ
                    self.pub.send(TOPIC_AUTO_STATE, st)
                    self._publish_map()

            if now >= next_hb:
                next_hb = now + NS // HB_HZ
                self.pub.send(TOPIC_HB_PREFIX + "planning",
                              HbMsg(node="planning",
                                    detail=f"{self.ctrl.mode or '-'}"
                                           f"{' engaged' if self.ctrl.engaged else ''}"))

    def stop(self) -> None:
        self._running = False

    def close(self) -> None:
        # 終了時に一発「止まれ」を置いてから消える。中継側が engage を見ている
        # ので実害は無いが、**最後の値が「走れ」で残らない**方が読みやすい
        # 届かなくても実害は無い（中継側が engage を見ている）。**送れたら送る**
        with quiet_close("終了時の auto/cmd 停止指令"):
            self.pub.send(TOPIC_AUTO_CMD, DriveCmd(mode=0, source="planning:shutdown"))
            self.pub.send(TOPIC_AUTO_STATE, AutoState(reason="planning_node 終了"))
        self.sub.close()
        self.pub.close()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", default="",
                    help=f"起動時のモード（{'/'.join(PLANNERS) or 'なし'}）。"
                         f"通常は GUI から選ぶので指定は要らない")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    if args.mode and args.mode not in PLANNERS:
        print(f"知らないモード: {args.mode!r}（候補 {', '.join(PLANNERS)}）",
              file=sys.stderr)
        return 2

    pub = Publisher("planning")
    topics = {TOPIC_VEHICLE_STATE: LATEST, TOPIC_AUTO_CTRL: LATEST, TOPIC_E2E_MODEL: LATEST}
    topics.update({t: LATEST for t in input_topics()})
    sub = Subscriber(topics)
    node = PlanningNode(pub=pub, sub=sub, mode=args.mode, quiet=args.quiet)

    if not args.quiet:
        print(f"# planning_node  publish {pub.endpoint}")
        print(f"# モード: {', '.join(f'{k}({v.name})' for k, v in PLANNERS.items())}")
        print(f"# 起動時 {args.mode or '（なし。GUI の自動運転タブで選ぶ）'}")
        print("#\n# `auto/cmd` は engage されている間だけ telemetry_node が"
              " `cmd` へ中継する。")
        print("# **このノード単独では車は動かない**（ARM も人間が GUI で保持する）\n")

    def _shutdown(*_):
        node.stop()

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, _shutdown)

    try:
        node.run()
    finally:
        node.close()
        if not args.quiet:
            print(f"\n=== 終了時の統計 ===\n計画 {node._plans} 回 "
                  f"（実測 {node._plan_hz:.1f}Hz）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
