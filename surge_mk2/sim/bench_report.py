"""`sim.bench` の 1 回の走行の結果と、その表示（`Result.report`）。

`sim/bench.py` から分けた（数字を集める箱と表示の書式は、走らせ方と独立に読めるようにする）。
"""

from __future__ import annotations

import math
import statistics

import numpy as np

from .stm32 import COMMAND_TIMEOUT_NS

#: 仮想 STM32 が COMMAND 途絶で車を止める間隔 [ms]
_DEADMAN_MS = COMMAND_TIMEOUT_NS / 1e6

__all__ = ["Result"]


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
        # 動く障害物（`--mover`）の追い抜き（真値）
        self.mover_passes = 0                  #: 他車を後ろから抜いた回数（車の座標で前→後ろへ移った）
        #: 他車との接触のうち、他車の方が速く近づいていたもの（避けない他車に当てられた）
        self.mover_hit_behind = 0
        self.mover_hit_front = 0               #: 同じく車の方が速く近づいていたもの（こちらが当てた）
        self.mover_min_clear = math.inf        #: 車体外形と他車の最小距離 [m]
        #: 他車が 3m 以内に居る間に、車速が元の経路の速度より 0.3m/s 以上遅かった時間 [s]（抑えられた時間）
        self.mover_slow_s = 0.0
        # 横グリップの学習（`raspi/nav/grip.py`。planner の内部状態と真値を突き合わせる）
        self.grip_on = False
        #: 前兆ごとの記録: RACE 開始からの時刻・学習側の値・真値の超過率（0 = 限界より下＝誤検出）・
        #: その後 1.5 秒の横偏差の最大と壁との余裕の最小
        self.grip_events: list[dict] = []
        self.grip_conv_t: float | None = None  #: 収束した時刻（RACE 開始から）[s]
        self.grip_conv_laps: int | None = None #: そのとき走り終えていた周の数
        self.grip_level = 0.0                  #: 終わったときの横の上限 [m/s²]
        self.grip_ceil = 0.0                   #: 安全装置が働いた上限（`GripMap.ceil`）[m/s²]
        self.grip_state = ""
        self.grip_steps = (0, 0)               #: タイムで決めた段の (判断, 採用)
        self.grip_true = 0.0                   #: 終わったときの真値の横の限界 [m/s²]
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
        if self.mover_min_clear < math.inf:
            out.append(f"他車       抜いた {self.mover_passes} 回  接触 当てた {self.mover_hit_front}・当てられた {self.mover_hit_behind}  "
                       f"最小距離 {self.mover_min_clear * 100:.1f}cm  "
                       f"近くで経路の速度より遅かった {self.mover_slow_s:.1f}s")
        if self.avoid_starts or self.detour_starts:
            ms = (f"計算 平均 {statistics.fmean(self.avoid_ms):.1f}ms "
                  f"p99 {float(np.percentile(self.avoid_ms, 99)):.1f}ms 最大 {max(self.avoid_ms):.1f}ms"
                  f"（{len(self.avoid_ms)} 回）" if self.avoid_ms else "")
            out.append(f"回避       止まらず避けた {self.avoid_starts} 回（うち引き直し {self.avoid_elastic}） {ms}  "
                       f"迂回(Hybrid A*) {self.detour_starts} 回・{self.detour_s:.1f}s")
        if self.grip_on:
            ev = self.grip_events
            # 横偏差の増え（周を終えてから分かる）は、その瞬間の真値と突き合わせられないので誤検出に数えない
            false = sum(1 for e in ev if e["over"] <= 0.0 and e["kind"] != "横偏差の増え")
            conv = (f"収束 {self.grip_conv_t:.1f}s・{self.grip_conv_laps}周" if self.grip_conv_t is not None
                    else "収束せず")
            out.append(f"横G学習    横 {self.grip_level:.2f}m/s²（真値 {self.grip_true:.2f}・"
                       f"天井 {self.grip_ceil:.2f}・{self.grip_state}）  {conv}  "
                       f"段の判断 {self.grip_steps[0]}（採用 {self.grip_steps[1]}）  "
                       f"安全装置 {len(ev)} 回（うち誤検出 {false}）")
            if self.grip_conv_t is not None and self.gate_laps:
                k = self.grip_conv_laps or 0
                after = self.gate_laps[max(k, 1):]
                if after:
                    ref = statistics.fmean(after)
                    lost = sum(t - ref for t in self.gate_laps[1:max(k, 1)])
                    out.append(f"           収束後のラップ 平均 {ref:.2f}s（{len(after)} 周）・"
                               f"収束までに失った時間 {lost:+.2f}s（2周目から）")
            if ev:
                out.append("           安全装置のあと 1.5s: 横偏差 最大 "
                           f"{max(e['cross'] for e in ev) * 100:.1f}cm・壁との余裕 最小 "
                           f"{min(e['clear'] for e in ev) * 100:.1f}cm・真値の超過 最大 "
                           f"{max(e['over'] for e in ev) * 100:.1f}%")
                for e in ev[:40]:
                    out.append(f"             {e['t']:6.1f}s {e['kind']} 横加速度 {e['a']:.2f}（上限 "
                               f"{e['applied']:.2f}）真値の超過 "
                               f"{e['over'] * 100:+.1f}% → 横偏差 {e['cross'] * 100:.1f}cm・"
                               f"余裕 {e['clear'] * 100:.1f}cm")
            if self.grip_long:
                lg = self.grip_long
                out.append(f"           加速 {lg['acc']:.2f}m/s²（真値 {lg['acc_true']:.2f}）・減速 "
                           f"{lg['dec']:.2f}m/s²（真値 {lg['dec_true']:.2f}）  判断 {lg['windows']} 回・"
                           f"下げた {len(lg['events'])} 回"
                           + (f"  包絡（横の使い具合 0/0.5/1）加速 {'/'.join(f'{x:.2f}' for x in lg['acc_env'])}"
                              f"・減速 {'/'.join(f'{x:.2f}' for x in lg['dec_env'])}" if lg.get("acc_env") else "")
                           + "".join(f"\n             {e['t']:6.1f}s {e['kind']} u={e.get('u', 0.0):.1f} "
                                     f"{e['ask']:.2f} → {e['got']:.2f}"
                                     + (f"（計画との差 {e['lag']:+.2f}m/s）" if "lag" in e else
                                        f"（出た÷求めた {e['ratio']:.2f}）" if "ratio" in e else "")
                                     for e in lg["events"][:12]))
                if lg["trail"]:
                    out.append("           加速/減速の移り変わり " + " ".join(
                        f"{t:.0f}s:{a:.2f}/{d:.2f}" for t, a, d in lg["trail"][:40]))
            if self.grip_levels:
                out.append("           値の移り変わり " + " ".join(
                    f"{t:.0f}s:{lv:.2f}" for t, lv in self.grip_levels[:60]))
        if self.lap_min_clear:
            out.append("周ごとの最小距離 " + " / ".join(f"{c * 100:.1f}cm"
                                                      for c in self.lap_min_clear))
        out.extend(self.notes)
        out.append(f"最後の状態 [{self.phase}] {self.reason}")
        return "\n".join(out)
