"""planner をシムの真値と突き合わせて評価する。**バスも GUI もプロセスも使わない。**

    .venv/bin/python -m sim.bench --course normal
    .venv/bin/python -m sim.bench --course course1 --mode slam2d_raceline --time 240
    .venv/bin/python -m sim.bench --mode ftg --set explore_speed=0.6
    .venv/bin/python -m sim.bench --course toyota2 --mode slam2d_route \
        --routes sim/routes/toyota2.json --switch 200:B
    .venv/bin/python -m sim.bench --course toyota2 --map toyota2 --mode slam2d_route \
        --time 120 --trace /tmp/t.csv          # 保存済み地図から走る（地図作成を飛ばす）

## ラップタイム（真値のゲート）

RACE で最初に走り出した真値の位置に、進行方向と直交するゲート線を置き、それを
前向きに跨いだ時刻（100Hz で補間）でラップを計る。**プランナーの周回カウンタには
頼らない**（planner の数え方は経路の最寄り点の添字で、時刻の分解能が 0.1s しか無い）。1周目はスタンディング
スタートを含むので、比較には2周目以降（フライング）を使う。

`io_node` と同じことを 1 プロセスの中でやる:

    SimLink.poll() → フレーム → ScanAssembler / StateBuilder → planner.plan()
                  → command_from_cmd() → SimLink.send()

**実機と同じ変換コードを通す**（`BusBridge`＝`ScanAssembler` の鏡像反転・`StateBuilder`、
`LinkTracker`＋`TimeSync` による STM32 時刻→Pi 時刻の換算、`command_from_cmd` のクランプ）ので、
ここで走れば `sim.run` でも走る。PING を 5Hz で送って時刻同期を育てる（収束するまでは実機と同じく
受信時刻で代用する）。

`--map` の走行は、地図を**真のコースへ重ねて**（`slam_bench.align_map`）自己位置の誤差と
経路からの真の横偏差を出す（planner の自己申告ではない）。

## なぜ別に用意するのか

`sim.run` は 4 プロセスを立ち上げてブラウザで engage する。**自己位置の誤差が
何 cm かを知りたいだけのときに重すぎる。** しかもシムの真値は UDP の私設
チャンネルにしか出ないので、GUI を開かないと見られない。

ここは真値を直に読めるので、**SLAM の推定と真値を毎周期突き合わせられる**。
実機では絶対に手に入らない数字なので、これが座標系と精度を確定する場所になる。

## `--allow-arm` に相当するものは無い

シム専用の入口で、動かす対象が numpy の中の車だけなので ARM を人間に握らせる
意味が無い（`sim/run.py` が `--allow-arm` を既定で付けているのと同じ理由）。
**このファイルを実機のコードから import してはいけない。**

## 限界

- **壁時計で回る**: 計画・指令の周期は `time.monotonic_ns()` で刻むので、ホストの負荷で結果が
  変わる（決定論的でない）。`advance_to` は 1 回 0.25s で頭打ちなので、それ以上ホストが止まると
  シムの時間が遅れる。指令の間隔が仮想 STM32 の途絶（100ms）を超えた走行は「捨てる走行」の
  印が付く（`Result.report`）。
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np  # noqa: E402

from raspi.auto.registry import PLANNERS, make_planner, merged_params  # noqa: E402
from raspi.core.bus_bridge import BusBridge  # noqa: E402
from raspi.core.link_tracker import LinkTracker  # noqa: E402
from raspi.msgs import DriveCmd, command_from_cmd  # noqa: E402
from raspi.msgs.types import TOPIC_SCAN, TOPIC_VEHICLE_STATE  # noqa: E402
from raspi.proto.generated import packets  # noqa: E402

from .course import list_courses  # noqa: E402
from .link import create_sim_link  # noqa: E402
from .params import REAL_VIBRATION  # noqa: E402
from .stm32 import COMMAND_TIMEOUT_NS  # noqa: E402

#: 仮想 STM32 が COMMAND 途絶で車を止める間隔 [ms]
_DEADMAN_MS = COMMAND_TIMEOUT_NS / 1e6

__all__ = ["Bench", "Result"]

NS = 1_000_000_000
CMD_HZ = 100
#: PING の間隔（`io_node` の `PING_HZ`）
PING_PERIOD_NS = NS // 5


class _Capture:
    """`BusBridge` の出力（バスの代わり）。最後に出た車両状態と、組み上がった点群を受ける。"""

    def __init__(self) -> None:
        self.vs = None
        self.scan = None

    def send(self, topic, obj) -> None:
        if topic == TOPIC_VEHICLE_STATE:
            self.vs = obj
        elif topic == TOPIC_SCAN:
            self.scan = obj


class Result:
    """1回の走行の結果。**数字は全部ここに集める。**"""

    def __init__(self) -> None:
        self.plans = 0
        self.plan_ms: list[float] = []
        self.pose_err: list[float] = []        #: SLAM と真値の**移動量**の差 [m]
        self.yaw_err: list[float] = []         #: 同 向き [deg]
        self.cross: list[float] = []           #: 経路からの横偏差 [m]（RACE 段のみ。planner の自己申告）
        #: 真の位置から経路までの距離 [m]（`--map` の RACE 段。地図を真のコースへ重ねて測る）
        self.cross_true: list[float] = []
        self.collisions = 0
        self.laps = 0
        self.phase = ""
        self.reason = ""
        self.lap_times: list[float] = []
        self.distance = 0.0
        #: EXPLORE段が終わった瞬間（地図構築完了時点）の自己位置・向き誤差。
        #: RACE段の誤差は「凍結地図に対する追跡精度」でしかなく、地図自体が
        #: 歪んでいるかどうかはこの数字でしか直接見えない
        #: （`/Users/banbatakumi/.claude/plans/slam2d-slam-slam2d-imu-slam-slam2d-imu-nifty-aurora.md`
        #: フェーズ1参照）
        self.explore_end_pose_err: float | None = None
        self.explore_end_yaw_err: float | None = None
        #: 経路選択走行（`slam2d_route`）の検証。周回ごとに真値で通った道路グラフの
        #: エッジ列（真のコースから作ったグラフの番号）と、その周に走っていた経路
        self.lap_edges: list[tuple[str, list[int]]] = []
        #: 停止点に止まったときの、真値の位置と停止点の距離 [m]。**自己位置の誤差を含む**
        self.stop_err: float | None = None
        #: 同じく推定位置（map フレーム）と停止点の距離 [m]。**止める制御の精度**
        #: （実機では停止点を SLAM の地図の上で置くので、こちらが「狙った所に止まれたか」）
        self.stop_err_est: float | None = None
        self.notes: list[str] = []
        #: 真値のゲートで計ったラップ [s]（先頭はスタンディングスタートを含む）
        self.gate_laps: list[float] = []
        self.race_vmax = 0.0                   #: RACE 中の真値の最高速度 [m/s]
        self.race_brake_s = 0.0                #: RACE 中に制動を出していた時間 [s]
        self.race_collisions = 0               #: RACE 中の衝突
        #: RACE 中の車体外形と真の壁との最小距離 [m]（周回ごと・全体）。衝突の手前の余裕
        self.race_min_clear = math.inf
        self.lap_min_clear: list[float] = []
        self.ideal_lap: float | None = None    #: 速度プロファイルどおりの1周 [s]
        self.path_len: float | None = None
        # RACE 中の真値（100Hz）。**「車の実力を使えているか」を見る数字**
        self.race_n = 0
        self.race_steer_sat = 0                #: 舵の指令が上限に張り付いていたサンプル数
        self.race_a_lat: list[float] = []      #: 横加速度の絶対値 [m/s²]
        self.race_accel: list[float] = []      #: 加速を要求している間（指令 > 車速+0.3）の実加速 [m/s²]
        self.race_decel: list[float] = []      #: 減速を要求している間（指令 < 車速-0.3）の実減速 [m/s²]
        self.race_over: list[float] = []       #: 経路の速度に対する出過ぎ [m/s]（正 = 速すぎ）
        # 障害物の回避（planner の内部状態を読む）
        self.avoid_starts = 0                  #: 横へ避ける経路を引いた回数
        self.avoid_elastic = 0                 #: そのうち、窓の中の線を引き直して作った回数
        self.avoid_ms: list[float] = []        #: その計算時間 [ms]
        self.detour_starts = 0                 #: Hybrid A* の迂回に入った回数
        self.detour_s = 0.0                    #: 迂回していた時間 [s]
        self.stop_s = 0.0                      #: RACE 中に止まっていた時間 [s]（車速 < 0.05）
        # 横グリップの学習（`raspi/nav/grip.py`。planner の内部状態と真値を突き合わせる）
        self.grip_on = False
        #: 前兆ごとの記録: RACE 開始からの時刻・学習側の値・真値の超過率（0 = 限界より下＝誤検出）・
        #: その後 1.5 秒の横偏差の最大と壁との余裕の最小
        self.grip_events: list[dict] = []
        self.grip_conv_t: float | None = None  #: 収束した時刻（RACE 開始から）[s]
        self.grip_conv_laps: int | None = None #: そのとき走り終えていた周の数
        self.grip_level = 0.0                  #: 終わったときの全体の値 [m/s²]
        self.grip_bad = 0.0
        self.grip_state = ""
        self.grip_trials = (0, 0)              #: (試行, 通った数)
        self.grip_launch: float | None = None  #: 発進の加速度の見積もり [m/s²]
        self.grip_ratio = 0.0
        self.grip_true = 0.0                   #: 終わったときの真値の横の限界 [m/s²]
        #: 限界より下で走っている間の、前兆の量（`dev − base`）の最大と p99.9（しきい値の余裕）
        self.grip_excess: list[float] = []
        self.grip_levels: list[tuple[float, float]] = []   #: (RACE 開始からの時刻, 全体の値) の移り変わり
        #: 加速・減速の学習（終わりの値・真値・出なかった窓・移り変わり）
        self.grip_long: dict = {}
        self.cmd_gap_ms = 0.0                  #: 指令を送る間隔の最大 [ms]
        self.rearms = 0                        #: 仮想 STM32 が ARM を断り、外して入れ直した回数
        self.slow_ms: dict[str, float] = {}

    def report(self) -> str:
        def stat(v, unit, scale=1.0):
            if not v:
                return "—"
            a = [x * scale for x in v]
            return (f"平均 {statistics.fmean(a):.2f}{unit} "
                    f"中央 {statistics.median(a):.2f}{unit} "
                    f"最大 {max(a):.2f}{unit}")

        gap = (f"  指令の間隔 最大 {self.cmd_gap_ms:.0f}ms"
               + (f"（★{_DEADMAN_MS:.0f}ms を超えた。デッドマンで止まった走行。捨てて走らせ直す）"
                  if self.cmd_gap_ms > _DEADMAN_MS else ""))
        if self.cmd_gap_ms > 100.0:
            gap += "  各部の最大 " + "・".join(f"{k} {v:.0f}ms" for k, v in self.slow_ms.items())
        if self.rearms:
            gap += f"  ARM の入れ直し {self.rearms} 回（ホストが詰まって COMMAND が途切れた）"
        out = [
            f"計画       {self.plans} 回  1周期 {stat(self.plan_ms, 'ms')}{gap}",
            f"自己位置   {stat(self.pose_err, 'cm', 100)}",
            f"向き       {stat(self.yaw_err, '°')}",
            f"横偏差     {stat([abs(c) for c in self.cross], 'cm', 100)}（自己申告）",
            *([f"真の横偏差 {stat(self.cross_true, 'cm', 100)}（地図を真のコースへ重ねた）"]
              if self.cross_true else []),
            f"周回       {self.laps} 周  走行 {self.distance:.1f}m  衝突 {self.collisions}",
        ]
        if self.explore_end_pose_err is not None:
            out.append(
                f"EXPLORE終了時点  自己位置 {self.explore_end_pose_err * 100:.1f}cm  "
                f"向き {self.explore_end_yaw_err:.1f}°")
        if self.lap_times:
            out.append(
                "ラップ     " + " / ".join(f"{t:.1f}s" for t in self.lap_times))
        for i, (key, edges) in enumerate(self.lap_edges):
            out.append(f"周{i + 1:<2} 経路{key or '-'}  通ったエッジ {'→'.join(map(str, edges))}")
        if self.stop_err is not None:
            out.append(f"停止誤差   真値 {self.stop_err * 100:.1f}cm（自己位置の誤差を含む）・"
                       f"推定 {self.stop_err_est * 100:.1f}cm（止める制御の精度）")
        if self.gate_laps:
            fly = self.gate_laps[1:]
            best = f"最良 {min(fly):.2f}s 平均 {statistics.fmean(fly):.2f}s" if fly else "—"
            out.append("真値ラップ " + " / ".join(f"{t:.2f}s" for t in self.gate_laps)
                       + f"  （2周目以降 {best}）")
        if self.ideal_lap is not None:
            out.append(f"理想ラップ {self.ideal_lap:.2f}s（プロファイルどおり・"
                       f"1周 {self.path_len:.1f}m）")
        out.append(f"RACE       最高速 {self.race_vmax:.2f}m/s  制動 {self.race_brake_s:.1f}s  "
                   f"衝突 {self.race_collisions}  車体と壁の最小距離 "
                   f"{self.race_min_clear * 100:.1f}cm")
        if self.race_n:
            def mean(v):
                return f"{statistics.fmean(v):.2f}" if v else "—"
            p99 = (f"{float(np.percentile(self.race_a_lat, 99)):.2f}" if self.race_a_lat else "—")
            over = (f"平均 {statistics.fmean(self.race_over):+.2f} "
                    f"最大 {max(self.race_over):+.2f}m/s" if self.race_over else "—")
            pms = (f"{float(np.percentile(self.plan_ms, 99)):.1f}" if self.plan_ms else "—")
            out.append(f"車の使い方 舵の張り付き {100.0 * self.race_steer_sat / self.race_n:.1f}%  "
                       f"横G p99 {p99}m/s²  要求中の加速 {mean(self.race_accel)}・"
                       f"減速 {mean(self.race_decel)}m/s²  経路速度からの出過ぎ {over}  "
                       f"停止 {self.stop_s:.1f}s  計画 p99 {pms}ms")
        if self.avoid_starts or self.detour_starts:
            ms = (f"計算 平均 {statistics.fmean(self.avoid_ms):.1f}ms 最大 {max(self.avoid_ms):.1f}ms"
                  if self.avoid_ms else "")
            out.append(f"回避       止まらず避けた {self.avoid_starts} 回（うち引き直し {self.avoid_elastic}） {ms}  "
                       f"迂回(Hybrid A*) {self.detour_starts} 回・{self.detour_s:.1f}s")
        if self.grip_on:
            ev = self.grip_events
            # 頭打ち（区間を抜けてから分かる前兆）は、その瞬間の真値と突き合わせられないので別に数える
            flat = sum(1 for e in ev if e["kind"] == "頭打ち")
            false = sum(1 for e in ev if e["over"] <= 0.0 and e["kind"] != "頭打ち")
            conv = (f"収束 {self.grip_conv_t:.1f}s・{self.grip_conv_laps}周" if self.grip_conv_t is not None
                    else "収束せず")
            out.append(f"横G学習    全体 {self.grip_level:.2f}m/s²（真値 {self.grip_true:.2f}・"
                       f"前兆の値 {self.grip_bad:.2f}・{self.grip_state}）  {conv}  "
                       f"試行 {self.grip_trials[0]}（通過 {self.grip_trials[1]}）  "
                       f"前兆 {len(ev)} 回（うち頭打ち {flat}・誤検出 {false}）")
            if self.grip_conv_t is not None and self.gate_laps:
                k = self.grip_conv_laps or 0
                after = self.gate_laps[max(k, 1):]
                if after:
                    ref = statistics.fmean(after)
                    lost = sum(t - ref for t in self.gate_laps[1:max(k, 1)])
                    out.append(f"           収束後のラップ 平均 {ref:.2f}s（{len(after)} 周）・"
                               f"収束までに失った時間 {lost:+.2f}s（2周目から）")
            if ev:
                out.append("           前兆のあと 1.5s: 横偏差 最大 "
                           f"{max(e['cross'] for e in ev) * 100:.1f}cm・壁との余裕 最小 "
                           f"{min(e['clear'] for e in ev) * 100:.1f}cm・真値の超過 最大 "
                           f"{max(e['over'] for e in ev) * 100:.1f}%")
                for e in ev[:40]:
                    out.append(f"             {e['t']:6.1f}s {e['kind']} 値 {e['b']:.2f}（上限 "
                               f"{e['applied']:.2f}{'・想定外' if e['surprise'] else ''}）真値の超過 "
                               f"{e['over'] * 100:+.1f}% → 横偏差 {e['cross'] * 100:.1f}cm・"
                               f"余裕 {e['clear'] * 100:.1f}cm")
            if self.grip_long:
                lg = self.grip_long
                out.append(f"           加速 {lg['acc']:.2f}m/s²（真値 {lg['acc_true']:.2f}）・減速 "
                           f"{lg['dec']:.2f}m/s²（真値 {lg['dec_true']:.2f}）  判断 {lg['windows']} 回・"
                           f"下げた {len(lg['events'])} 回"
                           + "".join(f"\n             {e['t']:6.1f}s {e['kind']} {e['ask']:.2f} → "
                                     f"{e['got']:.2f}（計画との差 {e['lag']:+.2f}m/s）"
                                     for e in lg["events"][:12]))
                if lg["trail"]:
                    out.append("           加速/減速の移り変わり " + " ".join(
                        f"{t:.0f}s:{a:.2f}/{d:.2f}" for t, a, d in lg["trail"][:40]))
            if self.grip_launch is not None:
                out.append(f"           発進の加速度 {self.grip_launch:.2f}m/s²・比 {self.grip_ratio:.2f}")
            if self.grip_excess:
                ex = np.asarray(self.grip_excess)
                out.append(f"           限界より下での前兆の量 最大 {ex.max():.3f}・p99.9 "
                           f"{float(np.percentile(ex, 99.9)):.3f}（しきい値 "
                           f"{self.grip_thr:.3f}）")
            if self.grip_levels:
                out.append("           値の移り変わり " + " ".join(
                    f"{t:.0f}s:{lv:.2f}" for t, lv in self.grip_levels[:60]))
        if self.lap_min_clear:
            out.append("周ごとの最小距離 " + " / ".join(f"{c * 100:.1f}cm"
                                                      for c in self.lap_min_clear))
        out.extend(self.notes)
        out.append(f"最後の状態 [{self.phase}] {self.reason}")
        return "\n".join(out)


class Bench:
    def __init__(self, course: str | Path, mode: str, params: dict[str, float], *,
                 max_speed: float = 3.0, max_steer: float = 0.524,
                 model: str | None = None, quiet: bool = False, seed: int = 0,
                 routes: dict | None = None, events: list[tuple[float, str, str]] | None = None,
                 auto_race: bool = True, map_name: str | None = None,
                 trace: Path | None = None,
                 locate_hint: tuple[float, float] | None = None,
                 obstacles: list[tuple[float, float, float]] | None = None,
                 vehicle: dict[str, float] | None = None,
                 control: dict[str, float] | None = None,
                 surface: dict | None = None, assume: dict[str, float] | None = None,
                 grip_file: Path | None = None, grip_cfg: dict[str, float] | None = None,
                 sim_params: dict[str, float] | None = None,
                 launch_ratio: float = 0.0,
                 grip_prior: tuple[float, ...] | None = None,
                 state_dir: Path | None = None) -> None:
        """:param model: `reload_if_changed(name)`を持つplanner（今のところ`e2e_lidar`
            だけ）へ渡すモデル名。`raspi/nodes/planning_node.py`の`_apply_e2e_model()`
            と同じダックタイピング（`E2ELidar`をここでimportして特別扱いしない）。
            **これが無いと`--mode e2e_lidar`はモデル未ロードのまま何も動かない**
            （GUI/`e2e/model`トピック経由でしかモデルを選べないので、
            `sim.bench`単体で検証するために必要）
        """
        # 地図を作る planner は BUILD の終わりに地図を自動保存する。**実機の
        # `saved_maps/` を bench の地図で埋めない**よう、保存先を一時ディレクトリへ
        import tempfile

        import shutil

        from raspi.auto import mapstore
        self._real_maps_dir = mapstore.MAPS_DIR
        self._tmp_maps_dir = mapstore.MAPS_DIR = Path(tempfile.mkdtemp(prefix="bench_maps_"))
        #: `--map`: 保存済み地図の**写し**を一時ディレクトリへ置いて読む（経路の設定を
        #: 保存し直しても実機の `saved_maps/` を書き換えない）
        self._map_name = map_name
        if map_name:
            src = self._real_maps_dir / f"{map_name}.npz"
            if not src.is_file():
                raise SystemExit(f"地図が無い: {src}")
            for suf in (".npz", ".json", ".routes.json"):
                f = self._real_maps_dir / f"{map_name}{suf}"
                if f.is_file():
                    shutil.copy(f, mapstore.MAPS_DIR / f.name)
            # 学習した横グリップは、実機の `saved_maps/` のものは使わず `--grip` で渡したものだけ読む
            # （bench の路面は実機と別物）
            if grip_file is not None:
                shutil.copy(grip_file, mapstore.MAPS_DIR / f"{map_name}.grip.json")
            # `--state`: 前の bench が残した地図の学習値・仕上げた経路の写しを置く（2 回目の走行の再現）
            self._state_dir = state_dir
            if state_dir is not None:
                for suf in (".grip.json", ".lines.pkl"):
                    f = state_dir / f"{map_name}{suf}"
                    if f.is_file():
                        shutil.copy(f, mapstore.MAPS_DIR / f.name)
        if launch_ratio > 0.0:
            # `--launch-ratio`: 車に固有の「横の限界 ÷ 発進の加速度」を知っている状態から始める
            mapstore.save_grip_vehicle(launch_ratio)
        if grip_prior is not None:
            # `--grip-prior`: 別の地図で学習した路面の記録がある状態から始める（新しい地図の出発点）
            import time as _time
            mapstore.save_grip_surface(*grip_prior, t=_time.time())
        # `--vehicle`: **真値の車だけ**諸元を変える（planner は `config/vehicle.toml` のまま）。
        # 同定値が外れていたとき（グリップが低い・舵が届かない等）に走りが壊れないかを見る
        spec = None
        if vehicle:
            from dataclasses import fields, replace

            from .vehicle import VehicleSpec
            known = {f.name for f in fields(VehicleSpec)}
            bad = sorted(set(vehicle) - known)
            if bad:
                raise SystemExit(f"--vehicle: 知らない諸元 {', '.join(bad)}")
            spec = replace(VehicleSpec.load(), **vehicle)
        params_sim = None
        if sim_params:
            from dataclasses import fields as _fields, replace as _replace

            from .params import SimParams
            bad = sorted(set(sim_params) - {f.name for f in _fields(SimParams)})
            if bad:
                raise SystemExit(f"--sim: 知らない項目 {', '.join(bad)}")
            params_sim = _replace(SimParams.load(), **sim_params)
        self.link = create_sim_link(course, seed=seed, with_channel=False, spec=spec,
                                    params=params_sim)
        # `--mu`・`--surface`・`--mu-at`: 真値の路面を場所と時刻で変える（`sim/surface.py`）
        self.surface = None
        if surface:
            from .surface import Surface
            sf = self.surface = Surface(self.link.sim.spec.mu)
            if surface.get("base"):
                sf.set_base(*surface["base"])
            for d in surface.get("discs", []):
                sf.add_disc(*d)
            for c in surface.get("changes", []):
                sf.add_change(*c)
            self.link.sim.surface = sf
        # 実機では io_node が `[control]` を CONFIG_SET で STM32 へ入れる。bench は io_node を
        # 通さないので、車速PIの加速度フィードフォワード（★v0.21）だけ直に入れる
        from raspi.core.control_params import load_control_params
        ctl = {**load_control_params(), **(control or {})}
        self.sim = self.link.sim
        # 車に効かないもの（TC・TV の係数など）も値を持たせる。launch_exit_torque_nm・速度 FF は車へ、
        # TC・ABS の有効/無効は旗の出し方に効く
        from raspi.core.control_params import CONTROL_PARAM_IDS, ENABLE_KEYS
        self.sim.vehicle.set_speed_ff(0.0, 0.0)
        for key, val in ctl.items():
            pid = CONTROL_PARAM_IDS.get(key, ENABLE_KEYS.get(key))
            if pid is not None:
                self.sim.apply_config(pid, float(val))
        if obstacles:
            # 障害物はシムの世界（LiDAR と衝突判定が見る格子）にだけ刻む。`--map` と
            # 併用すると凍結地図には無い物が経路の上に居る状況になる
            from .track import stamp_discs
            c = self.sim.course
            stamp_discs(c.grid, c.origin, c.resolution, list(obstacles))
            c._padded_grid = None
        self.planner = make_planner(mode)
        if self.planner is None:
            raise SystemExit(f"知らないモード: {mode!r}（候補 {', '.join(PLANNERS)}）")
        if model:
            reload_fn = getattr(self.planner, "reload_if_changed", None)
            if reload_fn is None:
                raise SystemExit(f"{mode!r} はモデル選択に対応していない"
                                 f"（reload_if_changedを持たない）")
            reload_fn(model)
        if assume:
            # `--assume`: **planner が信じている車**の諸元だけ変える（真値の車はそのまま）。
            # 「真値を知っている固定設定」の基準を作るのに使う
            from dataclasses import replace as _rep
            try:
                self.planner.vehicle = _rep(self.planner.vehicle, **assume)
            except TypeError as e:
                raise SystemExit(f"--assume: {e}")
        learner = getattr(self.planner, "_grip", None)
        if grip_cfg and learner is not None:
            for k, val in grip_cfg.items():
                if not hasattr(learner.cfg, k):
                    raise SystemExit(f"--grip-set: 知らない項目 {k}")
                setattr(learner.cfg, k, type(getattr(learner.cfg, k))(val))
            learner.cfg.fixed_step = "step_up" in grip_cfg
        self._grip_n = 0
        self._grip_open: list[dict] = []
        self._grip_slip: list[tuple[float, float]] = []    # 直近の (時刻, 真値の超過率)
        self._grip_lv = -1.0
        self._long_n = 0
        self._last_cmd_ns = 0
        self._grip_t0: float | None = None
        self._race_t0: float | None = None
        self.mode = mode
        self.params = merged_params(mode, params, hidden=True)
        self.max_speed = max_speed
        self.max_steer = max_steer
        self.quiet = quiet

        # `io_node` と同じ受信経路（時刻同期つき）。バスの代わりに `_Capture` が結果を受ける
        self._cap = _Capture()
        self.bridge = BusBridge(self._cap, clock=time.monotonic_ns)
        self.tracker = LinkTracker(on_telemetry=self.bridge.on_telemetry,
                                   on_frame=self.bridge.on_frame,
                                   on_time_reset=self._on_time_reset)
        self.bridge.to_pi_ns = self.tracker.sync.to_pi_ns
        self._ping_seq = 0
        self._next_ping_ns = 0
        self.vs = None
        self.res = Result()

        # 真値の起点。**SLAM の原点は「最初の姿勢」**なので、絶対座標ではなく
        # 起点からの移動量どうしを比べる（`test_nav.py` と同じ考え方）
        v = self.sim.vehicle
        self._origin = (v.x, v.y, v.yaw)
        #: `--map` の地図の原点の世界姿勢（RACE に入ってから `_align_map` が決める）
        self._map_origin: tuple[float, float, float] | None = None
        self._last_lap_t = 0.0
        self._prev_laps = 0
        self._prev_phase = ""

        #: DONE（地図作成が終わって人の「走行」を待つ段）で、自動保存した地図を
        #: 読み込ませて走り出させる（GUI の「レーシングライン走行」を押すのと同じ）
        self.auto_race = auto_race
        #: `(時刻, 種類, 値)`。種類は "route"（`request_route`）/"signal"（`request_signal`）
        self.events = sorted(events or [])
        self._truth_log: list[tuple[str, int, str, float, float]] = []
        self._routes_world = routes
        if routes is not None:
            if getattr(self.planner, "request_routes", None) is None:
                raise SystemExit(f"{mode!r} は経路の設定（--routes）に対応していない")
            if not map_name:
                self.planner.request_routes(json.dumps(self._to_map_frame(routes)))
        #: `--map` の読み込みは最初の `plan()` の後に行う（`slam2d_route` は
        #: `plan()` で受けたパラメータで経路を作るため）
        self._load_pending = bool(map_name)
        self._locate_hint = locate_hint

        # 真値のゲート（モジュール docstring）
        self._gate: tuple[float, float, float, float] | None = None
        self._gate_prev: tuple[float, float, float] | None = None   # (t, 沿う距離, 横)
        self._gate_last_t: float | None = None
        self._gate_odom = 0.0
        self._race_col0: int | None = None
        self._trace_f = open(trace, "w") if trace else None
        if self._trace_f:
            self._trace_f.write("t,phase,x,y,yaw,v,cmd_v,brake,prof_v,idx,cross,clear,launch,"
                                "steer_cmd,steer,a_lat,a_x,kappa,slip,g_level,g_excess,g_cap,g_gate\n")
            # 加速・減速の学習（`nav/grip.LongLearner`）へ渡した値と、その瞬間の真値（`<trace>.long.csv`）
            lg = getattr(getattr(self.planner, "_grip", None), "long", None)
            if lg is not None:
                self._long_f = open(str(trace) + ".long.csv", "w")
                self._long_f.write("t,v,a_plan,gate,v_plan,v_true,ax_true,grip_long,u_y,"
                                   "cmd_v,accel,decel\n")
                inner = lg.observe

                def observe(t, v, a_plan, gate, v_plan=None, *rest, _in=inner, _lg=lg, **kw):
                    tv, sp = self.sim.vehicle, self.sim.spec
                    u = tv.speed ** 2 * abs(tv._curv_req) / max(sp.mu * tv.grip_lat * 9.81, 1e-6)
                    self._long_f.write(
                        f"{t:.4f},{v:.4f},{a_plan:.4f},{gate},"
                        f"{float('nan') if v_plan is None else v_plan:.4f},{tv.speed:.4f},"
                        f"{tv.accel_x:.4f},{tv.grip_long:.4f},{u:.4f},{tv.cmd.target_speed:.4f},"
                        f"{_lg.map.accel:.4f},{_lg.map.decel:.4f}\n")
                    return _in(t, v, a_plan, gate, v_plan, *rest, **kw)
                lg.observe = observe
        # 車体外形の輪郭と、真の壁からの距離場（`_clearance`）
        from scipy.ndimage import distance_transform_edt

        from .course import _outline
        c = self.sim.course
        # `distance_transform_edt` はセル中心どうしの距離なので、壁の面までは約半セル短い。
        # 半セル引く（負にはしない）
        self._edt = np.maximum(
            distance_transform_edt(~c.grid.astype(bool)) * c.resolution - c.resolution / 2, 0.0)
        self._outline = _outline(np.asarray(self.sim.spec.footprint, dtype=np.float64),
                                 c.resolution / 2)
        self._lap_clear = math.inf
        self._prev_avoid = None
        self._prev_detour = False
        self._prev_reason_head = ""
        #: 障害物まわりの判断の移り変わり（`--obs-log` で出す）
        self._obs_log: list[str] = []
        self._feed_vs = getattr(self.planner, "on_vehicle_state", None)
        self._tick_fn = getattr(self.planner, "tick", None)

    def _on_time_reset(self, rx_ns: int) -> None:
        self.bridge.state_builder.reset()
        self.bridge.scans.reset()

    def _send_ping(self, now_ns: int) -> None:
        self._ping_seq = (self._ping_seq + 1) & 0xFFFFFFFF
        t1 = self.link.send(packets.Ping(ping_id=self._ping_seq))
        self.tracker.note_ping_sent(self._ping_seq, t1)
        self._next_ping_ns = now_ns + PING_PERIOD_NS

    # ── 座標 ──

    def _to_map(self, x: float, y: float, yaw: float | None = None, origin=None):
        """シムの世界座標 → SLAM の map フレーム（起点が原点、起点の向きが +x）。
        `origin` は map の原点の世界姿勢（既定は bench の起点。`--map` の地図は別）。"""
        ox, oy, oyaw = origin or self._origin
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        dx, dy = x - ox, y - oy
        mx, my = dx * c - dy * s, dx * s + dy * c
        if yaw is None:
            return mx, my, None
        return mx, my, (yaw - oyaw + math.pi) % (2 * math.pi) - math.pi

    def _to_map_frame(self, cfg: dict) -> dict:
        """`--routes` の JSON（シムの世界座標で書く）を map フレームへ直す。"""
        out = json.loads(json.dumps(cfg))
        for k, pts in (out.get("groups") or {}).items():
            conv = []
            for p in pts:
                mx, my, myaw = self._to_map(p[0], p[1], p[2] if len(p) > 2 else None)
                conv.append([mx, my] + ([] if myaw is None else [myaw]))
            out["groups"][k] = conv
        out["avoid"] = [list(self._to_map(q[0], q[1])[:2]) for q in (out.get("avoid") or [])]
        for name, st in (out.get("stops") or {}).items():
            mx, my, myaw = self._to_map(st["x"], st["y"], st.get("yaw"))
            st["x"], st["y"], st["yaw"] = mx, my, myaw
        return out

    # ── 真値 ──

    def _truth_rel(self) -> tuple[float, float, float]:
        """map フレームでの真の姿勢。`--map` なら、地図を真のコースへ重ねた原点（`_align_map`）基準。"""
        v = self.sim.vehicle
        ox, oy, oyaw = self._map_origin or self._origin
        dx, dy = v.x - ox, v.y - oy
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        return (dx * c - dy * s, dx * s + dy * c,
                (v.yaw - oyaw + math.pi) % (2 * math.pi) - math.pi)

    # ── 走行 ──

    def run(self, duration_s: float) -> Result:
        t0 = time.monotonic()
        next_cmd = time.monotonic_ns()
        cmd_period = NS // CMD_HZ
        cmd = DriveCmd(mode=0)
        last_plan_ns = 0
        next_tick = next_cmd

        slow = self.res.slow_ms                  # ループの各部の最大 [ms]（どこで詰まったかを見る）

        def took(key: str, t_start: float) -> None:
            if self._gate is not None:
                slow[key] = max(slow.get(key, 0.0), (time.perf_counter() - t_start) * 1000)

        rearm = False
        # 仮想 STM32 は起動直後、ARM を一度外すまで ARM を受け付けない。最初の指令を ARM にすると
        # 断られて入れ直しになる（実機では人が ARM を入れる前に DISARM が流れている）ので、先に DISARM を送る
        self.link.send(command_from_cmd(DriveCmd(mode=0), allow_arm=True,
                                        max_speed=self.max_speed, max_steer=self.max_steer))
        while time.monotonic() - t0 < duration_s:
            t_poll = time.perf_counter()
            frames = self.link.poll()
            took("シム", t_poll)
            for f in frames:
                self._cap.scan = None
                msg = self.tracker.feed(f.rx_ns, f.type, f.seq, f.payload)
                if isinstance(msg, packets.Telemetry):
                    self.vs = self._cap.vs
                    # `planning_node` と同じく、届いた順に全部渡す（点ごとの脱スキューと、
                    # 点群の合間の姿勢の前進に使う。渡さないと SLAM は
                    # 「1周のあいだ twist は一定」の近似になる）
                    if self._feed_vs is not None:
                        t_vs = time.perf_counter()
                        self._feed_vs(self.vs)
                        took("車両状態", t_vs)
                    continue
                if isinstance(msg, packets.Log) and msg.message.startswith(b"ARM ignored"):
                    # ホストが詰まって COMMAND が 100ms 途切れると、仮想 STM32 はファームと同じく
                    # ARM を一度外すまで受け付けない。実機では io_node が GUI の ARM を下ろさせ、
                    # 人が入れ直す。ベンチは人の代わりに1回だけ外して入れ直す
                    rearm = True
                    continue
                if not isinstance(msg, (packets.LidarSector, packets.LidarSectorI,
                                        packets.LidarSectorC)):
                    continue
                scan = self._cap.scan
                if scan is None:
                    continue

                now = time.monotonic_ns()
                dt = (now - last_plan_ns) / NS if last_plan_ns else 0.0
                last_plan_ns = now
                t_plan = time.perf_counter()
                st = self.planner.plan(scan, self.vs, self.params, dt)
                self.res.plan_ms.append((time.perf_counter() - t_plan) * 1000)
                self.res.plans += 1
                took("計画", t_plan)
                t_now = time.monotonic() - t0
                t_rec = time.perf_counter()
                self._record(st, t_now)
                self._step_events(st, t_now)
                took("記録", t_rec)

                # `planning_node._cmd_from` と同じ変換に、実機で `telemetry_node._merge_auto` が
                # 載せる GUI のレート制限（`[safety] cmd_*_limit`）を足す。★ 載せないと
                # `steer_rate_limit=0` をファームが「最大舵角/秒」（0.52rad/s、実機の 7.0 の
                # 1/13）と読み、切り返しに舵が間に合わず経路から外れた
                sp = self.sim.spec
                accel = sp.cmd_accel_limit_m_s2
                if st.accel_limit > 0:
                    accel = min(accel, st.accel_limit) if accel > 0 else st.accel_limit
                if not st.ready or st.brake:
                    cmd = DriveCmd(mode=2, arm=True, brake=True,
                                   target_steer=st.target_steer,
                                   steer_rate_limit=sp.cmd_steer_rate_limit_rad_s,
                                   brake_torque=st.brake_torque if st.ready else 0.0)
                else:
                    cmd = DriveCmd(mode=2, arm=True, target_speed=st.target_speed,
                                   target_steer=st.target_steer, accel_limit=accel,
                                   steer_rate_limit=sp.cmd_steer_rate_limit_rad_s,
                                   launch=st.launch)

            now = time.monotonic_ns()
            if now >= self._next_ping_ns:
                self._send_ping(now)
            if now >= next_tick:
                # 点群の合間の追従（`planning_node._tick` と同じ。50Hz）
                next_tick = now + NS // 50
                st_last = getattr(self, "_last_st", None)
                if (self._tick_fn is not None and st_last is not None and st_last.ready
                        and not cmd.brake and cmd.mode == 2):
                    t_tick = time.perf_counter()
                    out = self._tick_fn(self.vs)
                    took("追従", t_tick)
                    if out is not None:
                        steer, speed, ramp = out
                        if steer is not None:
                            cmd.target_steer = float(steer)
                        if speed is not None:
                            cmd.target_speed = float(speed)
                            lim = self.sim.spec.cmd_accel_limit_m_s2
                            cmd.accel_limit = (min(lim, ramp) if lim > 0 else ramp) if ramp > 0 \
                                else lim
            if now >= next_cmd:
                # 指令の間隔（ホストが詰まって `cmd_deadman_ms` を超えると、仮想 STM32 が車を止める。
                # その走行は捨てる——走りの問題ではない）
                # （走っている間だけ数える。地図の読み込み・ラインの計算で止まっている間は関係ない）
                if self._last_cmd_ns and self._gate is not None and abs(self.sim.vehicle.speed) > 0.3:
                    self.res.cmd_gap_ms = max(self.res.cmd_gap_ms, (now - self._last_cmd_ns) / 1e6)
                self._last_cmd_ns = now
                next_cmd = now + cmd_period
                out_cmd = cmd
                if rearm:
                    out_cmd = DriveCmd(mode=0)       # ARM を一度外す（次の周期から元の指令に戻る）
                    rearm = False
                    self.res.rearms += 1
                self.link.send(command_from_cmd(
                    out_cmd, allow_arm=True,
                    max_speed=self.max_speed, max_steer=self.max_steer))
                t_smp = time.perf_counter()
                self._sample(time.monotonic() - t0, cmd, cmd_period / NS)
                took("集計", t_smp)

        self.res.collisions = self.sim.vehicle.collisions
        if self._race_col0 is not None:
            self.res.race_collisions = self.sim.vehicle.collisions - self._race_col0
        self.res.distance = float(self.sim.vehicle.odom_front[0])
        path = getattr(self.planner, "path", None)
        if path is not None and getattr(path, "closed", False) and len(path) >= 3:
            ds = np.hypot(*(np.roll(path.xy, -1, axis=0) - path.xy).T)
            v = np.maximum(path.v, 1e-3)
            self.res.ideal_lap = float(np.sum(2.0 * ds / (v + np.roll(v, -1))))
            self.res.path_len = float(path.length)
        if self._trace_f:
            self._trace_f.close()
            if path is not None and len(path) >= 3:
                # 終わったときに追っていた経路（`<trace>.path.csv`）
                from raspi.nav.raceline import speed_kappa
                kap = speed_kappa(path.xy, path.closed)
                with open(self._trace_f.name + ".path.csv", "w") as f:
                    f.write("idx,x,y,v,kappa\n")
                    for i in range(len(path)):
                        f.write(f"{i},{path.xy[i, 0]:.4f},{path.xy[i, 1]:.4f},{path.v[i]:.4f},"
                                f"{kap[i]:.5f}\n")
        if getattr(self, "_long_f", None):
            self._long_f.close()
        return self.res

    def _sample(self, t: float, cmd: DriveCmd, dt: float) -> None:
        """指令周期（100Hz）で真値を見る: ゲートのラップ・最高速・制動・記録。"""
        st = getattr(self, "_last_st", None)
        if st is None or st.phase not in ("RACE", "DETOUR"):
            return
        v = self.sim.vehicle
        if self._gate is None:
            if not st.ready:
                return
            self._gate = (v.x, v.y, math.cos(v.yaw), math.sin(v.yaw))
            self.res.notes.append(f"RACE 開始 地図座標 ({st.pose_x:.2f}, {st.pose_y:.2f})"
                                  f"（--locate-hint に使える）")
            self._gate_last_t = t
            self._gate_odom = float(v.odom_front[0])
            self._race_col0 = v.collisions
            self._race_t0 = t
        self._sample_grip(t, st, v)
        gx, gy, tx, ty = self._gate
        along = (v.x - gx) * tx + (v.y - gy) * ty
        lat = -(v.x - gx) * ty + (v.y - gy) * tx
        prev = self._gate_prev
        self._gate_prev = (t, along, lat)
        clear = self._clearance(v.x, v.y, v.yaw)
        self.res.race_min_clear = min(self.res.race_min_clear, clear)
        self._lap_clear = min(self._lap_clear, clear)
        if (prev is not None and prev[1] < 0.0 <= along and abs(lat) < 1.0
                and float(v.odom_front[0]) - self._gate_odom > 3.0):
            tc = prev[0] + (t - prev[0]) * (-prev[1]) / max(along - prev[1], 1e-9)
            self.res.gate_laps.append(tc - self._gate_last_t)
            self.res.lap_min_clear.append(self._lap_clear)
            self._lap_clear = math.inf
            self._gate_last_t = tc
            self._gate_odom = float(v.odom_front[0])
            if not self.quiet:
                print(f"  {t:5.1f}s 真値ラップ {self.res.gate_laps[-1]:.2f}s", flush=True)
        self.res.race_vmax = max(self.res.race_vmax, float(v.speed))
        if cmd.brake:
            self.res.race_brake_s += dt
        r = self.res
        r.race_n += 1
        if abs(cmd.target_steer) >= 0.98 * self.sim.spec.max_steer:
            r.race_steer_sat += 1
        r.race_a_lat.append(abs(float(v.accel_lateral)))
        if not cmd.brake:
            if cmd.target_speed > v.speed + 0.3:
                r.race_accel.append(float(v.accel_x))
            elif cmd.target_speed < v.speed - 0.3:
                r.race_decel.append(-float(v.accel_x))
        if abs(v.speed) < 0.05:
            r.stop_s += dt
        pl = self.planner
        path_now = getattr(pl, "_avoid_path", None) or getattr(pl, "path", None)
        hint = getattr(pl, "_hint", -1)
        if path_now is not None and 0 <= hint < len(path_now):
            r.race_over.append(float(v.speed) - float(path_now.v[hint]))
        av, dt_on = getattr(pl, "_avoid", None), getattr(pl, "_detour", None) is not None
        if av is not None and av is not self._prev_avoid and getattr(av, "obstacle", None) is not None:
            r.avoid_starts += 1
            r.avoid_elastic += getattr(pl, "_avoid_how", "") == "引き直し"
            r.avoid_ms.append(float(getattr(pl, "_avoid_ms", 0.0)))
        self._prev_avoid = av
        if dt_on:
            r.detour_s += dt
            if not self._prev_detour:
                r.detour_starts += 1
        self._prev_detour = dt_on
        if self._trace_f:
            path = getattr(self.planner, "path", None)
            i = getattr(self.planner, "_hint", -1)
            pv = float(path.v[i]) if path is not None and 0 <= i < len(path) else float("nan")
            self._trace_f.write(
                f"{t:.3f},{st.phase},{v.x:.4f},{v.y:.4f},{v.yaw:.4f},{v.speed:.4f},"
                f"{cmd.target_speed:.3f},{int(cmd.brake)},{pv:.3f},{i},{st.cross_track:.4f},"
                f"{clear:.4f},{int(v.launching)},{cmd.target_steer:.4f},{v.steer_actual:.4f},"
                f"{v.accel_lateral:.3f},{v.accel_x:.3f},"
                f"{(v.yaw_rate / v.speed if abs(v.speed) > 0.05 else 0.0):.4f},"
                f"{v.slip_frac:.4f},{self._trace_grip()}\n")

    def _trace_grip(self) -> str:
        g = getattr(self.planner, "_grip", None)
        if g is None or not getattr(self.planner, "_grip_on", False):
            return "0,0,0,"
        return (f"{g.level_at(*self.planner.slam.pose[:2]):.3f},{g.last_excess:.4f},"
                f"{g.cap if g.cap is not None else 0.0:.3f},{self.planner._grip_gate}")

    def _sample_grip(self, t: float, st, v) -> None:
        """横グリップの学習の状態を真値と突き合わせる（100Hz）。"""
        pl = self.planner
        g = getattr(pl, "_grip", None)
        if g is None or not getattr(pl, "_grip_on", False):
            return
        r = self.res
        r.grip_on = True
        r.grip_thr = g.cfg.dev_thr
        tr = t - (self._race_t0 or t)
        if self._grip_t0 is None:
            # 学習側の時刻（車両状態の時刻）と bench の時刻のずれ
            self._grip_t0 = (self.vs.t_capture / 1e9 - tr) if self.vs is not None else 0.0
        slip = float(v.slip_frac)
        self._grip_slip.append((t, slip))
        while self._grip_slip and self._grip_slip[0][0] < t - 0.4:
            self._grip_slip.pop(0)
        if slip <= 0.0 and not pl._grip_gate and g.last_excess != 0.0 and g.cap is None:
            r.grip_excess.append(g.last_excess)
        clear = self._clearance(v.x, v.y, v.yaw)
        for e in self._grip_open:
            e["cross"] = max(e["cross"], abs(st.cross_track))
            e["clear"] = min(e["clear"], clear)
            e["over"] = max(e["over"], slip)
        self._grip_open = [e for e in self._grip_open if t - e["_t"] < 1.5]
        while self._grip_n < len(g.events):
            src = g.events[self._grip_n]
            self._grip_n += 1
            e = {"t": tr, "_t": t, "kind": src["kind"], "b": src["b"],
                 "applied": src["applied"], "surprise": src["surprise"],
                 "over": max(s for _, s in self._grip_slip), "cross": abs(st.cross_track),
                 "clear": clear}
            r.grip_events.append(e)
            self._grip_open.append(e)
            if not self.quiet:
                print(f"  {t:5.1f}s ★横Gの前兆 {src['kind']} 値 {src['b']:.2f}（上限 "
                      f"{src['applied']:.2f}）真値の超過 {e['over'] * 100:+.1f}%", flush=True)
        if g.converged and r.grip_conv_t is None:
            r.grip_conv_t, r.grip_conv_laps = tr, len(r.gate_laps)
        elif not g.converged and r.grip_conv_t is not None and not g.down_only:
            r.grip_conv_t = r.grip_conv_laps = None      # 収束が解けた（路面が変わった等）
        if abs(g.map.level - self._grip_lv) > 1e-6:
            self._grip_lv = g.map.level
            r.grip_levels.append((tr, g.map.level))
        r.grip_level, r.grip_bad = g.map.level, g.map.bad
        r.grip_state = g.state(pl._grip_gate)
        r.grip_trials = (g.trials, g.passes)
        r.grip_launch, r.grip_ratio = g.launch.value, g.launch_ratio
        r.grip_true = float(v.spec.mu * v.grip_lat * 9.81)
        lg, sp = g.long, v.spec
        ev = [{**e, "t": e["t"] - self._grip_t0} for e in lg.events]
        trail = r.grip_long.get("trail", [])
        if not trail or abs(trail[-1][1] - g.map.accel) + abs(trail[-1][2] - g.map.decel) > 1e-6:
            trail.append((tr, g.map.accel, g.map.decel))
        # 真値: 路面で決まる上限と、目標車速を動かす傾きの上限の小さい方
        acc_true = min(sp.drive_accel_m_s2 * v.grip_long - sp.rolling_resistance,
                       sp.cmd_accel_limit_m_s2)
        dec_true = min(sp.speed_decel_m_s2 * v.grip_long + sp.rolling_resistance,
                       sp.cmd_accel_limit_m_s2)
        r.grip_long = {"acc": g.map.accel, "dec": g.map.decel, "windows": lg.windows, "events": ev,
                       "trail": trail, "acc_true": acc_true, "dec_true": dec_true,
                       "done": (True, lg.dec_done)}

    def _clearance(self, x: float, y: float, yaw: float) -> float:
        """車体外形（輪郭）と真の壁との最小距離 [m]。0 = 接触。"""
        c = self.sim.course
        co, si = math.cos(yaw), math.sin(yaw)
        b = self._outline
        wx = x + b[:, 0] * co - b[:, 1] * si
        wy = y + b[:, 0] * si + b[:, 1] * co
        cols = np.clip(((wx - c.origin[0]) / c.resolution).astype(np.int32), 0,
                       self._edt.shape[1] - 1)
        rows = np.clip(((wy - c.origin[1]) / c.resolution).astype(np.int32), 0,
                       self._edt.shape[0] - 1)
        return float(self._edt[rows, cols].min())

    def _step_events(self, st, t: float) -> None:
        if self._load_pending:
            self._load_pending = False
            if self._routes_world is not None:
                # 経路の設定は地図と一緒に読ませる（読み込み後に渡すと、道路グラフが
                # できる前なので経由点のグループが作られない）
                from raspi.auto import mapstore
                cfg = self._to_map_frame(self._routes_world)
                # 地図作成の軌跡（自動経路のスタートと逆走禁止の元）は地図側のものを残す
                # （GUI の保存 `request_routes` と同じ。消すと自動経路が保存した線をなぞるだけになる）
                try:
                    old = json.loads(mapstore.load_routes(self._map_name) or "{}")
                except ValueError:
                    old = {}
                if "explore_traj" in old and "explore_traj" not in cfg:
                    cfg["explore_traj"] = old["explore_traj"]
                mapstore.save_routes(self._map_name, json.dumps(cfg))
            self.planner.request_load(self._map_name)
            err = getattr(self.planner, "_load_error", "")
            if err:
                raise SystemExit(err)
            if self._locate_hint is not None:
                self.planner.request_locate_hint(*self._locate_hint)
            if not self.quiet:
                print(f"  {t:5.1f}s 保存済み地図『{self._map_name}』を読み込んだ", flush=True)
            return
        if (self.auto_race and st.phase == "DONE"
                and getattr(self.planner, "_saved_map_name", "")):
            name = self.planner._saved_map_name
            if not self.quiet:
                print(f"  {t:5.1f}s 地図『{name}』で走行を開始する（bench が押す）", flush=True)
            self.planner.request_load(name)
        while self.events and self.events[0][0] <= t:
            _, kind, value = self.events.pop(0)
            fn = getattr(self.planner, "request_route" if kind == "route" else "request_signal",
                         None)
            if fn is not None:
                fn(value, "bench")
                self.res.notes.append(f"{t:5.1f}s {kind} → {value}")
                if not self.quiet:
                    print(f"  {t:5.1f}s 切替要求 {kind}={value}", flush=True)

    def _record(self, st, t: float) -> None:
        self._last_st = st
        if st.reason != self._prev_reason_head and ("障害物" in st.reason or "迂回" in st.reason[:4]):
            if len(self._obs_log) < 400:
                self._obs_log.append(f"{t:6.1f}s v={self.sim.vehicle.speed:.2f} {st.reason[:150]}")
        self._prev_reason_head = st.reason
        v = self.sim.vehicle
        self._truth_log.append((st.phase, st.laps, getattr(st, "route_active", ""), v.x, v.y))
        if self._map_name and self._map_origin is None and st.phase in ("RACE", "DETOUR") and st.ready:
            self._align_map(st)
        if not self._map_name or self._map_origin is not None:
            # 保存済み地図の map フレームは bench の起点と無関係なので、`--map` は地図を真のコースへ
            # 重ねてから比べる（RACE に入る前は比べない）
            tx, ty, tyaw = self._truth_rel()
            self.res.pose_err.append(math.hypot(st.pose_x - tx, st.pose_y - ty))
            d = (st.pose_yaw - tyaw + math.pi) % (2 * math.pi) - math.pi
            self.res.yaw_err.append(abs(math.degrees(d)))
            if self._map_name and st.phase in ("RACE", "DETOUR"):
                dist = self._path_distance(tx, ty)
                if dist is not None:
                    self.res.cross_true.append(dist)
        if st.phase == "RACE":
            self.res.cross.append(st.cross_track)
        if st.laps > self._prev_laps:
            self.res.lap_times.append(t - self._last_lap_t)
            self._last_lap_t = t
            self._prev_laps = st.laps

        # EXPLORE段が終わった瞬間（地図構築完了、直後の姿勢）の誤差を1回だけ残す。
        # RACE段の誤差は凍結地図に対する追跡精度でしかないため、
        # 「地図構築中にどれだけドリフトしたか」はここでしか直接見えない
        if self._prev_phase == "EXPLORE" and st.phase != "EXPLORE" and self.res.pose_err:
            self.res.explore_end_pose_err = self.res.pose_err[-1]
            self.res.explore_end_yaw_err = self.res.yaw_err[-1]
        self._prev_phase = st.phase

        self.res.laps = st.laps
        self.res.phase = st.phase
        self.res.reason = st.reason

        if not self.quiet and self.res.plans % 20 == 0:
            err = (f"誤差 {self.res.pose_err[-1] * 100:5.1f}cm {self.res.yaw_err[-1]:4.1f}°"
                   if self.res.pose_err else f"速度 {v.speed:4.2f}m/s")
            print(f"  {t:5.1f}s [{st.phase or '-'}] 真値({v.x:5.2f},{v.y:5.2f}) {err}  "
                  f"一致度 {st.match_score:.2f}  {st.reason}", flush=True)

    def _align_map(self, st) -> None:
        """`--map` の地図を真のコースへ剛体で重ね、地図の原点の世界姿勢を決める。

        初期値は「いまの真の姿勢と planner の自己位置が一致する原点」。そこから壁の重なりで
        詰める（`slam_bench.align_map`）ので、自己位置の誤差は壁の重ね合わせには入らない。"""
        v = self.sim.vehicle
        oyaw = v.yaw - st.pose_yaw
        c, s = math.cos(oyaw), math.sin(oyaw)
        guess = (v.x - (st.pose_x * c - st.pose_y * s), v.y - (st.pose_x * s + st.pose_y * c), oyaw)
        slam = getattr(self.planner, "slam", None)
        if slam is None:
            self._map_origin = guess
            return
        from .slam_bench import align_map
        self._map_origin = align_map(slam.grid, self.sim.course, guess)
        d = math.hypot(self._map_origin[0] - guess[0], self._map_origin[1] - guess[1])
        self.res.notes.append(f"地図を真のコースへ重ねた（初期値から {d * 100:.0f}cm・"
                              f"{math.degrees(self._map_origin[2] - guess[2]):+.1f}°）")

    def _path_distance(self, x: float, y: float) -> float | None:
        """map フレームの点 `(x, y)` から、いま追っている経路（折れ線）までの距離 [m]。"""
        path = getattr(self.planner, "path", None)
        if path is None or len(path) < 2:
            return None
        xy = np.asarray(path.xy, dtype=np.float64)
        a = xy
        b = np.roll(xy, -1, axis=0) if getattr(path, "closed", False) else xy[1:]
        if not getattr(path, "closed", False):
            a = xy[:-1]
        ab = b - a
        l2 = np.maximum((ab ** 2).sum(axis=1), 1e-12)
        t = np.clip(((np.array([x, y]) - a) * ab).sum(axis=1) / l2, 0.0, 1.0)
        proj = a + ab * t[:, None]
        return float(np.sqrt(((proj - np.array([x, y])) ** 2).sum(axis=1)).min())

    def map_quality(self) -> str:
        """出来上がった地図を**真のコースと重ねて**採点する。

        姿勢の誤差だけを見ていると「地図がどれくらい歪んでいるか」が分からない。
        実機の GUI で崩れて見えた地図が、数字の上では悪く見えないこともある。

        - **正確さ**: 地図に立てた壁のうち、真の壁の近くにあるものの割合
        - **網羅**: 走った付近の真の壁のうち、地図に立ったものの割合

        SLAM の原点は「最初の姿勢」なので、真のコースを起点の車体座標へ移してから比べる。
        """
        planner = getattr(self.planner, "slam", None)
        if planner is None:
            return "—"
        g = planner.grid
        wall = g.wall_mask()
        if not wall.any():
            return "地図が空"

        course = self.sim.course
        ox, oy, oyaw = self._origin
        c, s = math.cos(-oyaw), math.sin(-oyaw)

        # 地図の壁セル → 世界座標（シムの座標系）
        rows, cols = np.nonzero(wall)
        mx, my = g.to_world(cols, rows)
        wx = ox + mx * math.cos(oyaw) - my * math.sin(oyaw)
        wy = oy + mx * math.sin(oyaw) + my * math.cos(oyaw)

        # 真の壁からの距離。**壁を 1 セル膨らませた版と比べる**（量子化のぶん）
        tol = 2
        occ = course.grid
        col = ((wx - course.origin[0]) / course.resolution).astype(np.int32)
        row = ((wy - course.origin[1]) / course.resolution).astype(np.int32)
        ok = ((col >= tol) & (col < occ.shape[1] - tol)
              & (row >= tol) & (row < occ.shape[0] - tol))
        near = np.zeros(col.size, dtype=bool)
        for dr in range(-tol, tol + 1):
            for dc in range(-tol, tol + 1):
                near[ok] |= occ[row[ok] + dr, col[ok] + dc]
        good = float(near.sum()) / max(1, near.size)
        return (f"壁 {int(wall.sum())} セル / 正確さ {good * 100:.0f}% "
                f"（真の壁から {tol * course.resolution * 100:.0f}cm 以内）")

    def route_report(self) -> None:
        """走行中（RACE）に真値で通った道路グラフのエッジを周回ごとに数える。

        グラフは**真のコース**から作る（SLAM の地図とは別物。番号の意味は
        `raspi/nav/roadgraph.py` を真値に当てたもの）。停止点に止まったなら
        停止誤差も出す。
        """
        from raspi.core.vehicle import Vehicle
        from raspi.nav.roadgraph import build_graph, trinary_from_bool

        course = self.sim.course
        veh = Vehicle.load()
        try:
            g = build_graph(trinary_from_bool(course.grid), resolution=course.resolution,
                            origin=course.origin, keep=veh.half_width + 0.02,
                            seed=self._origin[:2])
        except ValueError as e:
            self.res.notes.append(f"真値の道路グラフを作れない: {e}")
            return
        h, w = g.label.shape
        laps: dict[int, tuple[str, list[int]]] = {}
        for phase, lap, key, x, y in self._truth_log:
            if phase != "RACE":
                continue
            c, r = g.to_cell(x, y)
            if not (0 <= c < w and 0 <= r < h):
                continue
            e = int(g.label[r, c])
            if e < 0:
                continue
            k, seq = laps.setdefault(lap, (key, []))
            if not seq or seq[-1] != e:
                seq.append(e)
            laps[lap] = (k or key, seq)
        self.res.lap_edges = [laps[k] for k in sorted(laps)]
        if self._routes_world is not None and self.res.phase in ("STOPPED", "PARK"):
            then = (self._routes_world.get("mission") or {}).get("then", "")
            stop = (self._routes_world.get("stops") or {}).get(then)
            if stop is not None:
                v = self.sim.vehicle
                self.res.stop_err = math.hypot(v.x - stop["x"], v.y - stop["y"])
                mx, my, _ = self._to_map(stop["x"], stop["y"])
                st = self._last_st
                self.res.stop_err_est = math.hypot(st.pose_x - mx, st.pose_y - my)

    def save_grip(self, dst: Path) -> str:
        """学習した横グリップ（一時ディレクトリの `<地図>.grip.json`）を `dst` へ写す（`--save-grip`）。"""
        import shutil

        from raspi.auto import mapstore
        save = getattr(self.planner, "_grip_save", None)
        if save is not None:
            save()
        name = getattr(self.planner, "_grip_name", "")
        src = mapstore.MAPS_DIR / f"{name}.grip.json"
        if not name or not src.is_file():
            return "学習した横グリップが無いので保存しない"
        shutil.copy(src, dst)
        return f"学習した横グリップを {dst} に保存した"

    def save_built_map(self, name: str) -> str:
        """bench が作った地図を実機の `saved_maps/` へ `name` で写す（`--save-map`）。"""
        import shutil

        from raspi.auto import mapstore
        src = getattr(self.planner, "_saved_map_name", "")
        if not src:
            return "地図ができていないので保存しない"
        self._real_maps_dir.mkdir(parents=True, exist_ok=True)
        for suf in (".npz", ".json", ".routes.json"):
            f = mapstore.MAPS_DIR / f"{src}{suf}"
            if f.is_file():
                shutil.copy(f, self._real_maps_dir / f"{name}{suf}")
        return f"地図を {self._real_maps_dir / name}.npz に保存した"

    def close(self) -> None:
        close = getattr(self.planner, "close", None)
        if close is not None:
            close()
        self.link.close()
        # 書き換えた保存先を戻し、一時ディレクトリを消す
        import shutil

        from raspi.auto import mapstore
        mapstore.MAPS_DIR = self._real_maps_dir
        shutil.rmtree(self._tmp_maps_dir, ignore_errors=True)


def _error_preset(name: str) -> dict[str, float]:
    """`--errors` の車速・ジャイロの誤差を `SimParams` の欄に直す。値は `slam_bench.SensorModel`
    の preset から読む（二重に持たない）。"""
    if name == "none":
        return {}
    from .slam_bench import SensorModel
    m = SensorModel.preset(name)
    return {"speed_scale_err": m.speed_scale - 1.0, "gyro_scale_err": m.gyro_scale - 1.0,
            "gyro_bias_dps": m.gyro_bias_dps, "gyro_noise_dps": m.gyro_noise_dps}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--course", default="normal", help="コース名 or パス")
    ap.add_argument("--mode", default="slam2d_raceline", choices=list(PLANNERS))
    ap.add_argument("--time", type=float, default=180.0, help="走らせる秒数")
    ap.add_argument("--seed", type=int, default=0, help="シムの乱数シード（LiDARノイズ等）")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                    help="planner のパラメータを上書き（複数可）")
    # 実機の io_node は STM32 の LIMITS（5m/s）で切る。3.0 のままだと `v_max` を 3 より上げても効かない
    ap.add_argument("--max-speed", type=float, default=5.0)
    ap.add_argument("--max-steer", type=float, default=0.524)
    ap.add_argument("--model", default=None,
                    help="reload_if_changedに対応するplanner（e2e_lidar）へ渡すモデル名"
                         "（例: v2 → models/e2e_lidar/v2.onnx）。他のモードでは無視される")
    ap.add_argument("--routes", default=None,
                    help="経路の設定 JSON（`raspi/auto/route_config.py` の形、**シムの世界座標**で書く）。"
                         "slam2d_route 用")
    ap.add_argument("--switch", action="append", default=[], metavar="T:GROUP",
                    help="時刻 T [s] に経路グループを切り替える（複数可）")
    ap.add_argument("--signal", action="append", default=[], metavar="T:VALUE",
                    help="時刻 T [s] に信号認識の値を送る（`signal_map` で読み替え、複数可）")
    ap.add_argument("--no-auto-race", action="store_true",
                    help="地図作成が終わっても走り出さない（既定は bench が「走行」を押す）")
    ap.add_argument("--map", default=None,
                    help="saved_maps/ の地図名。地図作成を飛ばし、読み込んで LOCATE から走る")
    ap.add_argument("--save-map", default=None, metavar="NAME",
                    help="bench が作った地図を saved_maps/NAME として残す")
    ap.add_argument("--locate-hint", default=None, metavar="X,Y",
                    help="--map のとき LOCATE に渡すおおよその位置（地図座標）。対称なコースで速く確実にする")
    ap.add_argument("--trace", default=None, help="RACE 中の速度などを CSV で書く（100Hz）")
    ap.add_argument("--obstacle", action="append", default=[], metavar="X,Y,R",
                    help="シムの世界座標に円柱の障害物を置く（繰り返し可。地図には入れないなら --map と併用）")
    ap.add_argument("--vehicle", action="append", default=[], metavar="KEY=VALUE",
                    help="真値の車だけ諸元を上書き（`sim.vehicle.VehicleSpec` の欄名。例 mu=0.37、複数可）。"
                         "planner は config/vehicle.toml のまま")
    ap.add_argument("--control", action="append", default=[], metavar="KEY=VALUE",
                    help="**シムの STM32 だけ** `[control]` の値を上書き（launch_exit_torque_nm・"
                         "speed_ff_*・tc_enable などは仮想 STM32 が持つ）。planner は config/vehicle.toml のまま")
    ap.add_argument("--mu", default=None, metavar="MU[:MU_LONG]",
                    help="真値の路面の摩擦係数（一様）。`--vehicle mu=` と違い、加速の上限も同じ割合で変える。"
                         "MU_LONG を付けると前後だけ別の値（前後と横で摩擦が違う路面）")
    ap.add_argument("--surface", action="append", default=[], metavar="X,Y,R,MU[,MU_LONG]",
                    help="シムの世界座標 (X, Y) の半径 R の円の中だけ摩擦を変える（繰り返し可）")
    ap.add_argument("--mu-at", action="append", default=[], metavar="T:MU[:MU_LONG]",
                    help="bench の時刻 T [s] から一様の摩擦を変える（繰り返し可）")
    ap.add_argument("--assume", action="append", default=[], metavar="KEY=VALUE",
                    help="**planner が信じている車**の諸元だけ上書き（`raspi.core.vehicle.Vehicle` の欄名。"
                         "例 mu=0.55）。真値の車はそのまま")
    ap.add_argument("--grip", default=None, metavar="FILE",
                    help="学習した横グリップの JSON を --map の地図の横へ置いてから走る（保存値からの再開）")
    ap.add_argument("--save-grip", default=None, metavar="FILE",
                    help="走り終えたときの学習した横グリップを FILE へ書く")
    ap.add_argument("--launch-ratio", type=float, default=0.0,
                    help="横の限界 ÷ 発進の加速度（前の走行で収束したときに保存される値）を与えて始める。"
                         "発進の加速度から横グリップの初期値を出す経路の確認用")
    ap.add_argument("--grip-prior", default=None, metavar="LAT:DEC[:COUP[:CEIL]]",
                    help="路面の記録（別の地図で学習した横・減速 [m/s²]・摩擦円の比・安全装置が働いた横の上限）を置いてから走る。"
                         "新しい地図の学習はここから始まる（`mapstore.save_grip_surface`）")
    ap.add_argument("--state", default=None, metavar="DIR",
                    help="--map の地図の学習値（.grip.json）と仕上げた経路の写し（.lines.pkl）を DIR から読み、"
                         "走り終えたら DIR へ書く（同じ地図の 2 回目の走行を再現する）")
    ap.add_argument("--grip-per-lap", action="store_true",
                    help="横グリップを周の境目でだけ上げる（収束の速さを比べる基準版）")
    ap.add_argument("--grip-set", action="append", default=[], metavar="KEY=VALUE",
                    help="学習の定数を上書き（`raspi.nav.grip.GripConfig` の欄名。上げ幅の実測用）")
    ap.add_argument("--sim", action="append", default=[], metavar="KEY=VALUE",
                    help="センサ誤差などシムの設定を上書き（`sim.params.SimParams` の欄名）")
    ap.add_argument("--real-noise", action="store_true",
                    help="実機で測った走行中の揺れ（ヨーレート・車速・実舵角）をテレメトリに足す"
                         "（`sim.params.REAL_VIBRATION`。`--sim` の指定が優先）")
    ap.add_argument("--errors", default="none", choices=["none", "real", "harsh"],
                    help="閉ループのセンサ誤差（`sim.slam_bench` の real・harsh と同じ車速・ジャイロの"
                         "倍率とバイアス）。既定 none = 今まで通り。`--sim` の指定が優先")
    ap.add_argument("--obs-log", action="store_true",
                    help="障害物まわりの判断（reason の移り変わり）を時刻つきで出す")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    course = Path(args.course)
    if not course.exists():
        found = [p for p in list_courses() if p.stem == args.course]
        if not found:
            names = ", ".join(sorted({p.stem for p in list_courses()}))
            print(f"コースが無い: {args.course}（候補 {names}）", file=sys.stderr)
            return 2
        course = found[0]

    params: dict[str, float] = {}
    for kv in args.set:
        k, _, v = kv.partition("=")
        params[k.strip()] = float(v)

    routes = json.loads(Path(args.routes).read_text()) if args.routes else None
    events: list[tuple[float, str, str]] = []
    for kind, items in (("route", args.switch), ("signal", args.signal)):
        for it in items:
            t, _, v = it.partition(":")
            events.append((float(t), kind, v))

    def pairs(items):
        return {k.strip(): float(v) for k, _, v in (it.partition("=") for it in items)}

    def nums(text, sep):
        return tuple(float(x) for x in text.split(sep))

    surface: dict = {}
    if args.mu:
        surface["base"] = nums(args.mu, ":")
    if args.surface:
        surface["discs"] = [nums(d, ",") for d in args.surface]
    if args.mu_at:
        surface["changes"] = [nums(c, ":") for c in args.mu_at]
    grip_cfg = pairs(args.grip_set)
    if args.grip_per_lap:
        grip_cfg["per_lap"] = 1.0

    b = Bench(course, args.mode, params, max_speed=args.max_speed,
              max_steer=args.max_steer, model=args.model, quiet=args.quiet, seed=args.seed,
              routes=routes, events=events, auto_race=not args.no_auto_race,
              map_name=args.map, trace=Path(args.trace) if args.trace else None,
              locate_hint=(tuple(float(v) for v in args.locate_hint.split(","))
                           if args.locate_hint else None),
              obstacles=[tuple(float(v) for v in o.split(",")) for o in args.obstacle],
              vehicle={k.strip(): float(v) for k, _, v in
                       (kv.partition("=") for kv in args.vehicle)},
              control={k.strip(): float(v) for k, _, v in
                       (kv.partition("=") for kv in args.control)},
              surface=surface or None, assume=pairs(args.assume),
              grip_file=Path(args.grip) if args.grip else None, grip_cfg=grip_cfg,
              sim_params={**(REAL_VIBRATION if args.real_noise else {}),
                          **_error_preset(args.errors), **pairs(args.sim)},
              launch_ratio=args.launch_ratio,
              state_dir=Path(args.state) if args.state else None,
              grip_prior=(tuple(float(x) for x in (args.grip_prior.split(":") + ["1.0", "0"][
                                    max(0, len(args.grip_prior.split(":")) - 2):])[:4])
                          if args.grip_prior else None))
    print(f"# {course.name} / {args.mode} / {args.time:.0f}s / seed={args.seed}")
    try:
        res = b.run(args.time)
    except KeyboardInterrupt:
        res = b.res
    quality = b.map_quality()
    b.route_report()
    if args.save_map:
        res.notes.append(b.save_built_map(args.save_map))
    if args.save_grip:
        res.notes.append(b.save_grip(Path(args.save_grip)))
    from raspi.auto import mapstore as _ms
    save = getattr(b.planner, "_grip_save", None)
    if save is not None:
        save()
    if args.state and args.map:
        import shutil as _sh
        _st = Path(args.state)
        _st.mkdir(parents=True, exist_ok=True)
        for suf in (".grip.json", ".lines.pkl"):
            f = _ms.MAPS_DIR / f"{args.map}{suf}"
            if f.is_file():
                _sh.copy(f, _st / f.name)
        res.notes.append(f"地図の学習値・経路の写しを {_st} に書いた")
    surf = _ms.load_grip_surface()
    if surf is not None:
        res.notes.append(f"路面の記録: 横 {surf['lat']:.2f}・減速 {surf['dec']:.2f}・比 {surf['coup']:.2f}"
                         f"・安全装置 {surf['ceil']:.2f}"
                         f"（--grip-prior {surf['lat']}:{surf['dec']}:{surf['coup']}:{surf['ceil']}）")
    b.close()
    if args.obs_log:
        print("\n=== 障害物まわりの判断 ===")
        print("\n".join(b._obs_log))
    print("\n=== 結果 ===")
    print(res.report())
    print("地図       " + quality)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
