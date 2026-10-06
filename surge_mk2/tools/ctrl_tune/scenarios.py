"""制御を評価する場面（指令の時系列）と、その結果の物差し。

1つの場面 = 指令の区間の並び（`Seg`）＋どの制御を有効にするか＋初速。車両モデル（`Plant`）と
ファーム（`fw.Firmware`）に通して、`Result`（時系列）から物差しを計算する。

## 物差し

- **効率**: その場面で「一定のトルクを掛け続ける」やり方の最良（`oracle()`。スリップを見ずに
  ちょうどグリップの限界のトルクを知っていた場合）に対する、進んだ距離・止まった距離の比。
  路面μが途中で変わらない場面では、これが制御の上限の目安になる
- **滑っていた割合**: トルクを掛けている間に、スリップ率の絶対値が 0.3 を超えていた時間の割合
- **横グリップの残り**: 前後に滑るほど横力が出なくなる（摩擦円）。1/√(1+(κ/0.1)²) の平均。
  直線の場面でも、同じ滑り方で旋回していたらどれだけ横に踏ん張れたかの目安になる
- **トルクの暴れ**: 指令トルクの変化の速さの RMS [N·m/s]（振動・発熱・音）
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace

import numpy as np

from .fw import (FLAG_ABS, FLAG_LIFT, FLAG_TC, FLAG_TV, IN, MODE_BRAKE, MODE_DISARM, MODE_SPEED,
                 MODE_TORQUE, OUT, Firmware, HostConfig)
from .plant import Plant

__all__ = ["Seg", "Scenario", "Result", "run", "oracle", "SLIP", "ALL", "DT"]

DT = 0.0005          # ファームの制御周期
SUBSTEPS = 20
MAX_TORQUE_NM = 0.15


@dataclass(frozen=True)
class Seg:
    """指令の1区間。"""

    duration_s: float
    mode: int = MODE_SPEED
    value: float = 0.0          # 目標車速 / 1輪のトルク / 1輪の制動トルク
    accel_limit: float = 0.0
    steer: float = 0.0
    mu_left: float = 1.0
    mu_right: float = 1.0
    test_moment: float = float("nan")
    front_scale: float = 1.0


@dataclass(frozen=True)
class Scenario:
    key: str
    label: str
    segs: tuple[Seg, ...]
    initial_speed: float = 0.0
    tc: bool = True
    abs: bool = True
    lift: bool = True
    tv: bool = True
    #: 物差しを取る区間の番号（その区間の始まりから終わりまで）
    measure: int = 0
    #: "accel"（進んだ距離が長いほど良い）/ "brake"（止まるまでが短いほど良い）
    kind: str = "accel"

    def inputs(self) -> np.ndarray:
        rows = []
        for s in self.segs:
            n = int(round(s.duration_s / DT))
            row = np.zeros(len(IN))
            row[[IN["mode"], IN["value"], IN["accel_limit"], IN["steer"], IN["mu_left"], IN["mu_right"],
                 IN["test_moment"], IN["front_scale"]]] = (s.mode, s.value, s.accel_limit, s.steer,
                                                          s.mu_left, s.mu_right, s.test_moment,
                                                          s.front_scale)
            rows.append(np.tile(row, (n, 1)))
        return np.vstack(rows)

    def window(self) -> slice:
        n = [int(round(s.duration_s / DT)) for s in self.segs]
        a = sum(n[:self.measure])
        return slice(a, a + n[self.measure])


@dataclass
class Result:
    scenario: Scenario
    out: np.ndarray
    metrics: dict[str, float] = field(default_factory=dict)

    def col(self, name: str) -> np.ndarray:
        return self.out[:, OUT[name]]


def _metrics(sc: Scenario, out: np.ndarray, slip_target: float = 0.1) -> dict[str, float]:
    w = sc.window()
    o = out[w]
    c = lambda name: o[:, OUT[name]]            # noqa: E731
    kappa = np.maximum(np.abs(c("kappa_left")), np.abs(c("kappa_right")))
    m: dict[str, float] = {}
    flags = c("flags").astype(int)
    for name, bit in (("tc", FLAG_TC), ("abs", FLAG_ABS), ("tv", FLAG_TV), ("lift", FLAG_LIFT)):
        m[f"{name}_active"] = float(np.mean((flags & bit) != 0))
    if sc.kind == "brake":
        # 止まる = 0.3m/s を下回る（それより下は MD の制動が抜けていくので制御の差が出ない）
        below = np.nonzero(c("speed") < 0.3)[0]
        end = int(below[0]) if len(below) else len(o) - 1
        m["distance"] = float(c("distance")[end] - c("distance")[0])
        m["stopped"] = float(len(below) > 0)
        kappa = kappa[:end + 1]
        o_end = end + 1
    else:
        m["distance"] = float(c("distance")[-1] - c("distance")[0])
        m["speed_end"] = float(c("speed")[-1])
        o_end = len(o)
    m["slipping"] = float(np.mean(kappa > 0.3))
    # 目標のスリップ率からの行き過ぎ（目標に対する比の時間平均。0 = 一度も目標を超えていない、
    # 1 = 平均して目標の2倍で滑っていた）。「目標より上に居座る」ゲインを見分ける物差し
    m["slip_over"] = float(np.mean(np.maximum(0.0, kappa - slip_target)) / slip_target)
    m["slip_mean"] = float(np.mean(kappa))
    m["slip_peak"] = float(np.max(kappa))
    m["lateral_keep"] = float(np.mean(1.0 / np.sqrt(1.0 + (kappa / 0.1) ** 2)))
    cmd = (c("cmd_left")[:o_end] + c("cmd_right")[:o_end]) * 0.5
    m["chatter"] = float(np.sqrt(np.mean(np.diff(cmd) ** 2)) / DT) if o_end > 2 else 0.0
    m["wheel_peak"] = float(np.max(np.maximum(np.abs(c("wheel_left")), np.abs(c("wheel_right")))))
    yaw = c("yaw_rate")
    m["yaw_peak"] = float(np.max(np.abs(yaw)))
    m["heading"] = float(abs(np.sum(yaw) * DT))
    err = c("yaw_target") - yaw
    m["yaw_err_rms"] = float(np.sqrt(np.mean(err ** 2)))
    m["torque_diff_rms"] = float(np.sqrt(np.mean((c("cmd_right") - c("cmd_left")) ** 2)))
    return m


def run(fw: Firmware, plant: Plant, sc: Scenario, params: dict[str, float] | None = None) -> Result:
    cfg = HostConfig(dt_s=DT, substeps=SUBSTEPS, tc_enabled=int(sc.tc), abs_enabled=int(sc.abs),
                     wheel_lift_guard_enabled=int(sc.lift), tv_enabled=int(sc.tv), imu_ready=1,
                     initial_speed_m_s=sc.initial_speed)
    out = fw.run(plant.to_c(), cfg, sc.inputs(), params)
    # その場面を受け持つ制御の目標（制動モードは ABS、それ以外は TC）
    key = "abs_slip_target" if sc.segs[sc.measure].mode == MODE_BRAKE else "tc_slip_target"
    target = (params or {}).get(key, fw.params[key].default if key in fw.params else 0.1)
    return Result(sc, out, _metrics(sc, out, target))


def oracle(fw: Firmware, plant: Plant, sc: Scenario) -> float:
    """制御なしで一定のトルクを掛け続けたときの最良の距離（効率の分母）。

    物差しを取る区間のトルクだけを 0.01〜0.15N·m で振る。駆動は長いほど、制動は短いほど良い。
    """
    best = None
    quiet = replace_plant_noise(plant)
    for torque in np.linspace(0.01, MAX_TORQUE_NM, 29):
        segs = list(sc.segs)
        s = segs[sc.measure]
        mode = MODE_BRAKE if sc.kind == "brake" else MODE_TORQUE
        segs[sc.measure] = Seg(s.duration_s, mode, float(torque), 0.0, s.steer, s.mu_left, s.mu_right,
                               s.test_moment, s.front_scale)
        free = Scenario(sc.key, sc.label, tuple(segs), sc.initial_speed, tc=False, abs=False, lift=False,
                        tv=False, measure=sc.measure, kind=sc.kind)
        r = run(fw, quiet, free)
        d = r.metrics["distance"]
        if sc.kind == "brake":
            if r.metrics["stopped"] and (best is None or d < best):
                best = d
        elif best is None or d > best:
            best = d
    return float(best) if best is not None else float("nan")


def replace_plant_noise(plant: Plant) -> Plant:
    return replace(plant, front_noise_rad_s=0.0, rear_noise_rad_s=0.0, gyro_noise_rad_s=0.0)


_FULL = MAX_TORQUE_NM
_COAST = Seg(0.05, MODE_DISARM)

#: TC・ABS・片輪浮き対策の場面
SLIP: tuple[Scenario, ...] = (
    Scenario("launch", "停止から全開（トルク指令）", (Seg(1.0, MODE_TORQUE, _FULL),), 0.0),
    Scenario("launch_pi", "停止から車速指令 3m/s（ランプ3.0）", (Seg(1.0, MODE_SPEED, 3.0, 3.0),), 0.0),
    Scenario("roll", "0.5m/s から全開", (Seg(0.8, MODE_TORQUE, _FULL),), 0.5),
    Scenario("accel_mu_drop", "全開の途中で路面μが0.4倍（0.2s）",
             (Seg(0.25, MODE_TORQUE, _FULL), Seg(0.2, MODE_TORQUE, _FULL, mu_left=0.4, mu_right=0.4),
              Seg(0.4, MODE_TORQUE, _FULL)), 0.5, measure=1),
    Scenario("accel_split", "全開・左だけ路面μが0.4倍",
             (Seg(0.8, MODE_TORQUE, _FULL, mu_left=0.4),), 0.5),
    Scenario("lift", "停止から全開・左後輪が浮いている",
             (Seg(0.6, MODE_TORQUE, _FULL, mu_left=0.03),), 0.0),
    Scenario("corner_exit", "旋回（舵0.3rad・1.2m/s）から全開",
             (Seg(0.5, MODE_SPEED, 1.2, 3.0, steer=0.3), Seg(0.7, MODE_TORQUE, _FULL, steer=0.3)), 1.2,
             measure=1),
    Scenario("brake", "2.5m/s から最大制動", (Seg(0.1, MODE_SPEED, 2.5, 3.0), Seg(1.5, MODE_BRAKE, _FULL)),
             2.5, measure=1, kind="brake"),
    Scenario("brake_soft", "2.5m/s から制動 0.10N·m",
             (Seg(0.1, MODE_SPEED, 2.5, 3.0), Seg(1.5, MODE_BRAKE, 0.10)), 2.5, measure=1, kind="brake"),
    Scenario("brake_mu_drop", "最大制動の途中で路面μが0.4倍（0.2s）",
             (Seg(0.1, MODE_SPEED, 2.5, 3.0), Seg(0.2, MODE_BRAKE, _FULL),
              Seg(0.2, MODE_BRAKE, _FULL, mu_left=0.4, mu_right=0.4), Seg(1.5, MODE_BRAKE, _FULL)),
             2.5, measure=3, kind="brake"),
    Scenario("brake_split", "最大制動・左だけ路面μが0.4倍",
             (Seg(0.1, MODE_SPEED, 2.5, 3.0), Seg(2.0, MODE_BRAKE, _FULL, mu_left=0.4)), 2.5, measure=1,
             kind="brake"),
    Scenario("decel_pi", "2.5m/s から車速指令0（ランプ3.0）",
             (Seg(0.1, MODE_SPEED, 2.5, 3.0), Seg(1.5, MODE_SPEED, 0.0, 3.0)), 2.5, measure=1,
             kind="brake"),
    Scenario("decel_pi_corner", "旋回しながら車速指令0（ランプ3.0）",
             (Seg(0.3, MODE_SPEED, 1.5, 3.0, steer=0.25), Seg(1.0, MODE_SPEED, 0.0, 3.0, steer=0.25)), 1.5,
             measure=1, kind="brake"),
    Scenario("cruise", "1.5m/s の定速（介入しないこと）", (Seg(1.0, MODE_SPEED, 1.5, 3.0),), 1.5),
    Scenario("cruise_turn", "1.2m/s・舵0.35rad の定常旋回（介入しないこと）",
             (Seg(1.0, MODE_SPEED, 1.2, 3.0, steer=0.35),), 1.2),
)

ALL = SLIP
