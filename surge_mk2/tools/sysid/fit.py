"""mcapログ → システム同定パラメータ。

GUIの「システム同定」タブが録ったmcapには `raspi/nodes/logger_node.py` の
`DEFAULT_TOPICS` がそのまま入っている：`/cmd`（`DriveCmd`。telemetry_node が
STM32 へ流した指令）・`/vehicle_state`（`VehicleState`。50Hz）・`/scan`（`Scan`。10Hz）。

## 方針: シムのモデルそのものを当てはめる（2026-09-24、全面的に作り直し）

以前は「10%到達時刻をむだ時間とする」「IMUの|accel_x|の95パーセンタイル」の
ような、モデルとは別の特徴量から値を読んでいた。これだと**読んだ値をシムに入れても
実機の応答を再現する保証が無い**（実際、加速度はIMUのノイズ・傾き・バイアスで
STM32の上限3.0m/s²を超える3.73m/s²と読まれていた）。今は：

- 舵: `sim/vehicle.py` の `SteerServo`（むだ時間→定常ゲイン→1次遅れ→角速度上限→
  ヒステリシス）に実機の `steer_cmd_echo` を入れ、出てくる舵角が実機の `steer_actual` に
  最も近くなるパラメータを探す（**出力誤差法**）
- 速度・加減速: 同じく `SpeedController`（ファームの目標ランプ→PI→加速度の上限→車体）に
  実機の `/cmd` を入れて、オドメトリ由来の車速に合わせる（制動の減速度だけは速度の傾き）
- グリップ: 各速度段の定常旋回の横加速度の頭打ち

## 第三者検証を受けた変更（2026-09-26）

モデルと違う形の真値をベンチに入れると、黙って誤った値を返す箇所があった。いずれも直した:
速度は1次遅れ→ファームの PI（時定数が下限に張り付いていた）、有効自由度を残差の自己相関から
（`n_eff`。決め打ちだと存在しないガタ・ヒステリシスを採った）、前輪エンコーダの周期誤差の
推定と補正（`correct_encoder_ripple`。ガタと不感帯を取り違えた）、ヨーの2次遅れ・サーボの
定常ゲインをモデルに追加、残差をノイズの床と比べて「モデルの形から外れている」と警告する。
- 舵の効き・中立ずれ・アンダーステア勾配: 1つの試験では決まらないので、ステア試験
  （走行中の小舵角・低速の階段）・前後運動試験（直線の定速区間）・旋回試験（限界より下の区間）の
  観測をまとめて `VehicleSpec.curvature()` に当てはめる（`fit_geometry()`）

こうしておけば、ここで出た値を `vehicle.toml` に入れたシムは、同じ指令に対して
実機と同じ応答を返す（残差は `FitResult.notes` に出す）。検証は `tools/sysid/bench.py`
（真値の分かっているシムで各試験を閉ループで回し、ここで復元できるかを見る）。

## 時刻はメッセージの中の単調時刻を使う

`/vehicle_state` は `t_capture`（STM32の時刻をPi時刻へ換算済み）、`/cmd` は `t_pub`、
`/scan` は最後の点を測り終えた時刻（`sector_t_ns` ＋ `sector_dur_us` の最大。`sector_t_ns` は
セクタ先頭点の時刻で、2026-10-09 から STM32 の時刻を換算した値。それ以前の記録は受信時刻）。mcapヘッダの `log_time` は
epochへ換算した値なので混ぜない。

## `steer_cmd_echo` の意味（2026-09-24に判明）

実機のエコーは受理した指令そのものではなく、COMMAND の `steer_rate_limit`
（GUIが毎回載せる 7.0rad/s）でランプさせた**参照舵角**だった（0.524radのステップに
約75ms＝0.524/7.0）。以前はこのランプを「エコーの平滑化」とみなして、ランプの
遅れをシムに入れていなかった（シムの舵が実機より速かった）。今は `sim/vehicle.py`
がこのランプを再現し、ここではランプ後の参照→実舵角だけを当てはめる。
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np
from mcap.reader import make_reader
from scipy.optimize import least_squares, minimize, minimize_scalar
from scipy.signal import cont2discrete, lfilter

from raspi.auto._sysid_common import (REAR_SLIP_LIMIT, corner_limit, corner_windows, first_saturated,
                                      md_down, rear_slip)
from raspi.auto.sysid_latency import LATENCY_CODE_MOD, LATENCY_CODE_STEP_RAD
from sim.vehicle import (DriveInput, SpeedController, SteerLink, SteerServo, VehicleSpec, backlash,
                         brake_speed_factor, next_speed)

__all__ = [
    "Sample", "Cmd", "ScanStamp", "Log", "FitResult",
    "load_log", "load_samples",
    "fit_steer", "fit_speed", "fit_accel", "fit_corner", "fit_geometry", "fit_latency",
    "sensor_check", "odom_speed",
]

GRAVITY_MPS2 = 9.81
NS = 1e9
#: ジャイロの測定範囲 [rad/s]（ファームは MPU6050 を ±250°/s で使う、`mpu6050.c` GYRO_CONFIG=0）。
#: これに張り付いたサンプルは本当のヨーレートではないので、曲率・横加速度の材料から外す
GYRO_RANGE_RAD_S = math.radians(250.0)
_GYRO_SAT = 0.97 * GYRO_RANGE_RAD_S


# ── ログ ──────────────────────────────────────────────────────────────────

@dataclass
class Sample:
    """`/vehicle_state` 1件と、その時刻に効いていた `/cmd`（直前の1件）。"""

    t: float                # [s] ログ先頭からの経過
    target_speed: float     # [m/s]（/cmd）
    target_steer: float     # [rad]（/cmd）
    brake: bool             # （/cmd）
    #: [m/s] **前輪オドメトリをゼロ位相で微分した車速**（車体中心線方向に射影）。解析はすべてこれを使う。
    #: STM32 の `speed` は 2kHz で計算した値にローパス（時定数約10ms）を掛けて 50Hz に間引いたもので、
    #: 遅れがあるうえ、25Hz より上のノイズが間引きで折り返して混ざる。`odom_dist` はローパス前の
    #: 角度を積算した位置なので、記録を録り終えてから前後のサンプルで微分すれば遅れも折り返しも無い
    #: （`_speed_from_odom`）。`speed` の遅れ（`vehicle.toml` の `speed_filter_s`）にも依存しない
    speed: float
    steer_actual: float     # [rad]
    steer_cmd_echo: float   # [rad]（STM32がランプさせた参照舵角）
    yaw_rate: float         # [rad/s]
    accel_x: float          # [m/s²]（IMU。解析には使わない。表示用）
    tc_active: bool
    accel_limit: float = 0.0        # [m/s²]（/cmd）
    steer_rate_limit: float = 0.0   # [rad/s]（/cmd）
    brake_torque: float = 0.0       # [N·m]（/cmd）
    odom: float = 0.0               # [m] 前輪累積走行距離の平均（射影なし。周期誤差の補正後）
    speed_filtered: float = 0.0     # [m/s] STM32 が送った `speed` そのもの（ローパス後。自己点検用）
    odom_l: float = 0.0             # [m] 左前輪の累積走行距離（記録の値）
    odom_r: float = 0.0             # [m] 右前輪
    wheel_speed_rear: float = 0.0   # [m/s] 後輪モータの周速の平均（前輪と独立なセンサ。表示・点検用）
    abs_active: bool = False        # ABS が制動を削っている（★v0.15。それより前の記録は常に False）


@dataclass
class Cmd:
    """`/cmd` 1件（telemetry_node が publish した時刻）。"""

    t: float
    target_speed: float
    target_steer: float
    brake: bool
    accel_limit: float = 0.0
    steer_rate_limit: float = 0.0
    brake_torque: float = 0.0


@dataclass
class ScanStamp:
    """`/scan` 1件の時刻。`t_done` は最後の点を測り終えた時刻（Pi時刻）。"""

    t_done: float
    t_pub: float
    seq: int


@dataclass
class Log:
    samples: list[Sample]
    cmds: list[Cmd]
    scans: list[ScanStamp] = field(default_factory=list)
    #: 記録を読むときに分かったセンサの所見（前輪エンコーダの周期誤差の補正など）
    sensor_notes: list[str] = field(default_factory=list)
    #: 記録したときの STM32 のファームの ID（`/diag/link` の `fw_id`。無い記録は None）
    fw_id: int | None = None
    #: 記録したときに STM32 に入っていたステアのリンクの換算 `(steer_link_gain, steer_link_cubic)`
    #: （`/diag/link` の `control_params`。無い記録＝プロトコル v0.17 以前は換算なし＝`(1, 0)`）。
    #: 記録の舵角（`steer_actual`・指令）はこの換算を通した後の路面舵角
    steer_link: tuple[float, float] = (1.0, 0.0)
    #: モータドライバが無応答だった区間 `(始まり [s], 終わり [s], 名前)`（`md_dropouts`）。その間、
    #: STM32 はその車輪の速度・電流を最後の値のまま送り、止まった MD は駆動も制動もしない
    md_dropouts: list[tuple[float, float, str]] = field(default_factory=list)


@dataclass
class FitResult:
    """`values` は `config/vehicle.toml` `[dynamics]` のキー → 値。`notes` は人が読む所見。"""

    values: dict[str, float]
    notes: list[str] = field(default_factory=list)
    #: 値をそのまま使うべきでない警告（★「残差がノイズの何倍」＝記録がシムのモデルの形から外れている）。
    #: 同じ文は `notes` にも入る。GUI はこれがある試験の項目を既定で適用しない（2026-09-27、以前は
    #: 所見の文字列でしか分からず、★が出ても全項目が既定でチェックされていた）
    warnings: list[str] = field(default_factory=list)


def _cmd_from_json(t: float, o: dict) -> Cmd:
    return Cmd(t=t,
               target_speed=float(o.get("target_speed", 0.0)),
               target_steer=float(o.get("target_steer", 0.0)),
               brake=bool(o.get("brake", False)),
               accel_limit=float(o.get("accel_limit", 0.0)),
               steer_rate_limit=float(o.get("steer_rate_limit", 0.0)),
               brake_torque=float(o.get("brake_torque", 0.0)))


def md_dropouts(vs_msgs: list[tuple[float, dict]]) -> list[tuple[float, float, str]]:
    """モータドライバが無応答だった区間 `(始まり, 終わり, 名前)`（`vs_msgs` と同じ時間軸、時刻順）。
    判定はプランナーの `SensorGuard` と同じ（`_sysid_common.md_down`）。"""
    out: list[tuple[float, float, str]] = []
    open_: dict[str, float] = {}
    t = 0.0
    for t, vs in vs_msgs:
        down = md_down(vs.get("md_status") or [])
        for name in down:
            open_.setdefault(name, t)
        for name in [n for n in open_ if n not in down]:
            out.append((open_.pop(name), t, name))
    out.extend((t0, t, name) for name, t0 in open_.items())
    return sorted(out)


def build_log(vs_msgs: list[tuple[float, dict]], cmds: list[Cmd],
              scans: list[ScanStamp]) -> Log:
    """時刻 [s]（同じ時間軸）付きのメッセージ列 → `Log`。`bench.py` も使う。"""
    vs_msgs = sorted(vs_msgs, key=lambda x: x[0])
    cmds = sorted(cmds, key=lambda c: c.t)
    scans = sorted(scans, key=lambda s: s.t_done)
    if not vs_msgs or not cmds:
        raise ValueError("mcapに /cmd または /vehicle_state が含まれていません"
                         "（システム同定タブで録ったログか確認してください）")
    t0 = vs_msgs[0][0]
    cmd_t = np.array([c.t for c in cmds])
    samples: list[Sample] = []
    for t, vs in vs_msgs:
        i = int(np.searchsorted(cmd_t, t, side="right")) - 1
        c = cmds[i] if i >= 0 else Cmd(t=t, target_speed=0.0, target_steer=0.0, brake=False)
        odom = vs.get("odom_dist", [0.0, 0.0])
        ws = vs.get("wheel_speed", [0.0, 0.0, 0.0, 0.0])
        samples.append(Sample(
            t=t - t0,
            target_speed=c.target_speed, target_steer=c.target_steer, brake=c.brake,
            speed=float(vs.get("speed", 0.0)),
            steer_actual=float(vs.get("steer_actual", 0.0)),
            steer_cmd_echo=float(vs.get("steer_cmd_echo", 0.0)),
            yaw_rate=float(vs.get("yaw_rate", 0.0)),
            accel_x=float(vs.get("accel", [0.0, 0.0, 0.0])[0]),
            tc_active=bool(vs.get("tc_active", False)),
            accel_limit=c.accel_limit, steer_rate_limit=c.steer_rate_limit,
            brake_torque=c.brake_torque,
            odom=(float(odom[0]) + float(odom[1])) / 2.0,
            odom_l=float(odom[0]), odom_r=float(odom[1]),
            speed_filtered=float(vs.get("speed", 0.0)),
            wheel_speed_rear=(float(ws[2]) + float(ws[3])) / 2.0 if len(ws) >= 4 else 0.0,
            abs_active=bool(vs.get("abs_active", False)),
        ))
    sensor_notes = correct_encoder_ripple(samples)
    _speed_from_odom(samples)
    return Log(samples=samples,
               cmds=[replace(c, t=c.t - t0) for c in cmds],
               scans=[ScanStamp(s.t_done - t0, s.t_pub - t0, s.seq) for s in scans],
               sensor_notes=sensor_notes,
               md_dropouts=[(a - t0, b - t0, name) for a, b, name in md_dropouts(vs_msgs)])


# ── 統計 ──────────────────────────────────────────────────────────────────

def n_eff(resid: np.ndarray) -> float:
    """残差の系列の有効な独立サンプル数 `n / τ_int`（積分自己相関時間、2026-09-26）。

    尤度比の検定は残差が独立だと仮定する。以前は「全サンプル独立」（`fit_steer`）や
    「50Hzの5サンプルで1自由度」（`fit_geometry`）と決め打ちしていたが、モデルで表せない
    ゆっくりした誤差（エンコーダの周期誤差・ヨーの振動）が残差に乗ると隣り合う残差が
    強く相関し、自由度を水増しして**存在しない要素（ガタ・ヒステリシス）を採用した**。
    ここでは残差そのものの自己相関から見積もる（ρ_k が 0.05 を下回った所で打ち切り）。
    5点の移動平均を通した白色ノイズなら τ_int=5 で、以前の決め打ちと一致する。"""
    r = np.asarray(resid, dtype=float)
    n = len(r)
    if n < 10:
        return float(max(n, 1))
    x = r - r.mean()
    v = float(x @ x)
    if v <= 1e-30:
        return float(n)
    tau = 1.0
    for k in range(1, min(n // 4, 400)):
        rho = float(x[:-k] @ x[k:]) / v
        if rho < 0.05:
            break
        tau += 2.0 * rho
    return n / tau


def _lr_significant(n: float, err_small: float, err_big: float) -> bool:
    """要素を足したモデル（平均二乗誤差 `err_small`）が、足さないモデル（`err_big`）より
    ノイズでは説明できないほど良いか（尤度比、`_CAP_LR_THRESHOLD`）。"""
    return n * math.log(max(err_big, 1e-15) / max(err_small, 1e-15)) > _CAP_LR_THRESHOLD


# ── 前輪エンコーダの周期誤差（2026-09-26） ─────────────────────────────────

#: ファームが累積回転角を距離に直すときの車輪半径（`DRIVE_FRONT_WHEEL_RADIUS_M`）
_FRONT_WHEEL_RADIUS_M = 0.030
#: 推定する高調波（1回転に k 回）
_RIPPLE_HARMONICS = (1, 2, 3)
#: 補正するのは振幅がこれより大きく、尤度比で有意なときだけ [rad]
_RIPPLE_MIN_RAD = math.radians(0.3)


def _ripple_segments(samples: list[Sample]) -> list[np.ndarray]:
    """周期誤差の推定に使う区間: 指令が変わらず・制動なし・同じ向きに 0.15m/s 以上で走り続け、
    車輪が1.5回転以上回る区間（頭の0.2sは捨てる）。"""
    out = []
    circ = 2 * math.pi * _FRONT_WHEEL_RADIUS_M
    cur: list[int] = []

    def flush():
        if len(cur) >= 10:
            idx = np.array(cur)
            t = np.array([samples[i].t for i in idx])
            idx = idx[t >= t[0] + 0.2]
            if len(idx) >= 10 and abs(samples[idx[-1]].odom - samples[idx[0]].odom) >= 1.5 * circ:
                out.append(idx)

    for i, x in enumerate(samples):
        ok = (not x.brake and abs(x.speed_filtered) > 0.15 and cur
              and abs(x.target_speed - samples[cur[-1]].target_speed) < 1e-4
              and abs(x.target_steer - samples[cur[-1]].target_steer) < 1e-4
              and (x.speed_filtered > 0) == (samples[cur[-1]].speed_filtered > 0))
        if ok:
            cur.append(i)
            continue
        flush()
        cur = [i] if (not x.brake and abs(x.speed_filtered) > 0.15) else []
    flush()
    return out


def _ripple_design(theta: np.ndarray) -> np.ndarray:
    cols = []
    for k in _RIPPLE_HARMONICS:
        cols += [np.sin(k * theta), np.cos(k * theta)]
    return np.stack(cols, axis=1)


def encoder_ripple(samples: list[Sample], side: str) -> tuple[np.ndarray, float, float]:
    """片輪の角度の周期誤差 `e(θ) = Σ a_k sin kθ + b_k cos kθ` の係数・振幅 [rad]・尤度比の統計量。

    アナログの絶対角エンコーダは1回転の中で読みが非線形（1回転に1〜数回のうねり）なので、
    読んだ角度 θ_m = θ + e(θ) を微分した速度は車輪の回転数でうねる。これは白色ノイズと違い
    何サンプルも続くので、曲率（ヨーレート÷速度）に乗ってガタ・不感帯を取り違えさせた
    （検証: ±3.4°の誤差で不感帯0.6°→0・ガタ0.8°→1.15°）。定速の区間ごとに「3次式＋共通の
    高調波」を θ_m に当てはめて推定する（θ ≈ θ_m で評価。誤差の2次の項は無視）。"""
    segs = _ripple_segments(samples)
    if not segs:
        return np.zeros(2 * len(_RIPPLE_HARMONICS)), 0.0, 0.0
    attr = "odom_l" if side == "l" else "odom_r"
    blocks, ys, harm = [], [], []
    for idx in segs:
        t = np.array([samples[i].t for i in idx])
        th = np.array([getattr(samples[i], attr) for i in idx]) / _FRONT_WHEEL_RADIUS_M
        tt = (t - t.mean()) / max(np.ptp(t), 1e-3)
        blocks.append(np.stack([np.ones_like(tt), tt, tt ** 2, tt ** 3], axis=1))
        ys.append(th)
        harm.append(_ripple_design(th))
    n_poly = sum(b.shape[1] for b in blocks)
    n = sum(len(y) for y in ys)
    X0 = np.zeros((n, n_poly))
    r0 = c0 = 0
    for b in blocks:
        X0[r0:r0 + len(b), c0:c0 + b.shape[1]] = b
        r0 += len(b)
        c0 += b.shape[1]
    H = np.concatenate(harm)
    y = np.concatenate(ys)
    # 高調波の符号: θ_m = poly + e(θ)  → 真の角度 = θ_m − e
    X1 = np.concatenate([X0, H], axis=1)
    p0, *_ = np.linalg.lstsq(X0, y, rcond=None)
    p1, *_ = np.linalg.lstsq(X1, y, rcond=None)
    res0 = y - X0 @ p0
    res1 = y - X1 @ p1
    coef = p1[n_poly:]
    th_grid = np.linspace(0, 2 * math.pi, 360, endpoint=False)
    amp = float(np.max(np.abs(_ripple_design(th_grid) @ coef)))
    stat = n_eff(res1) * math.log(max(float(np.mean(res0 ** 2)), 1e-18)
                                  / max(float(np.mean(res1 ** 2)), 1e-18))
    return coef, amp, stat


def correct_encoder_ripple(samples: list[Sample]) -> list[str]:
    """左右の前輪で周期誤差を推定し、有意なら `odom_l/odom_r/odom` を補正する。所見を返す。"""
    notes = []
    if len(samples) < 50:
        return notes
    fixed = {}
    for side, name in (("l", "左"), ("r", "右")):
        coef, amp, stat = encoder_ripple(samples, side)
        if amp == 0.0 and stat == 0.0:
            notes.append("前輪エンコーダの周期誤差: 定速の区間が無く推定できず")
            return notes
        use = amp >= _RIPPLE_MIN_RAD and stat > _CAP_LR_THRESHOLD
        notes.append(f"前輪エンコーダの周期誤差（{name}）: 振幅 {math.degrees(amp):.2f}°"
                     + ("→ 補正した" if use else "（有意でない・小さいので補正せず）"))
        if use:
            fixed[side] = coef
    for side, coef in fixed.items():
        attr = "odom_l" if side == "l" else "odom_r"
        th = np.array([getattr(x, attr) for x in samples]) / _FRONT_WHEEL_RADIUS_M
        th_true = th - _ripple_design(th) @ coef
        for x, v in zip(samples, th_true * _FRONT_WHEEL_RADIUS_M):
            setattr(x, attr, float(v))
    if fixed:
        for x in samples:
            x.odom = 0.5 * (x.odom_l + x.odom_r)
    return notes


def _median_dt(t: np.ndarray) -> float:
    """記録の刻み [s]（中央値。TELEMETRY は 100Hz、2026-09-25 までの記録は 50Hz）。"""
    return float(np.median(np.diff(t))) if len(t) > 1 else 0.01


#: オドメトリを微分する窓の半幅 [s]（50Hzで前後2サンプル・計5点、100Hzで前後4サンプル・計9点）
_ODOM_DIFF_HALF_S = 0.045


def odom_speed(samples: list[Sample]) -> np.ndarray:
    """前輪オドメトリ（左右平均）をゼロ位相で微分し、舵角で車体中心線方向へ射影した車速 [m/s]。

    各サンプルの前後 `_ODOM_DIFF_HALF_S` 以内の点に直線を当てはめた傾き（左右対称な窓なので
    遅れが出ない。等間隔なら2次の項も打ち消し合うので、加速中でも中心の傾きは正しい）。
    時刻は実際の値を使うので、取りこぼしで間隔が空いても崩れない。

    `odom_dist` は 0.1mm 単位（STM32 内部は 1mrad≈0.03mm で積算し端数も繰り越す）なので、
    20ms で1カウントも進まないのは 5mm/s 未満のときだけ。50Hz・5点の当てはめで量子化の誤差は
    約0.5mm/s（`docs/development.md` §4.5）。窓は**時間**で決める（TELEMETRY は 2026-09-26 に
    50→100Hz。サンプル数で決めると頻度で窓の幅が変わる）。
    """
    t = np.array([x.t for x in samples])
    od = np.array([x.odom for x in samples])
    d = np.array([x.steer_actual for x in samples])
    return _odom_slope(t, od) * np.cos(d)


def _odom_slope(t: np.ndarray, od: np.ndarray) -> np.ndarray:
    """`odom_speed` の微分（射影の前）。各時刻の前後 `_ODOM_DIFF_HALF_S` 以内の点に当てはめた直線の傾き。
    `_simulate_speed` もシムの位置に同じ操作を掛ける（実測と同じ鈍り方にそろえる）。"""
    n = len(t)
    # 各サンプルについて前後 k サンプルずつを並べ、窓（±_ODOM_DIFF_HALF_S）の中の点だけで当てはめる
    idx = np.arange(n)
    k = max(2, int(math.ceil(_ODOM_DIFF_HALF_S / _median_dt(t))))
    J = np.clip(idx[:, None] + np.arange(-k, k + 1)[None, :], 0, n - 1)
    T = t[J] - t[:, None]
    ok = np.abs(T) <= _ODOM_DIFF_HALF_S
    X = od[J]
    cnt = ok.sum(axis=1)
    tm = np.where(ok, T, 0.0).sum(axis=1) / np.maximum(cnt, 1)
    xm = np.where(ok, X, 0.0).sum(axis=1) / np.maximum(cnt, 1)
    dt_ = np.where(ok, T - tm[:, None], 0.0)
    num = (dt_ * np.where(ok, X - xm[:, None], 0.0)).sum(axis=1)
    den = (dt_ * dt_).sum(axis=1)
    return np.where(den > 1e-12, num / np.maximum(den, 1e-12), 0.0)


def _speed_from_odom(samples: list[Sample]) -> None:
    """`speed` をオドメトリ由来の車速（`odom_speed`）に置き換える。元の値は `speed_filtered`。
    オドメトリが全く動いていない記録（センサが無い等）は置き換えない。"""
    for x in samples:
        x.speed_filtered = x.speed
    if len(samples) < 3 or max(x.odom for x in samples) - min(x.odom for x in samples) < 1e-4:
        return
    for x, vi in zip(samples, odom_speed(samples)):
        x.speed = float(vi)


def load_log(path: str | Path) -> Log:
    """mcap → `Log`。時刻はメッセージの中の単調時刻（モジュールdocstring参照）。"""
    vs_raw: list[tuple[int, int, dict]] = []
    cmd_raw: list[tuple[int, int, dict]] = []
    scan_raw: list[tuple[int, int, dict]] = []
    fw_id = None
    link: dict[str, float] = {}
    with open(path, "rb") as f:
        reader = make_reader(f)
        for _schema, channel, message in reader.iter_messages(
                topics=["/cmd", "/vehicle_state", "/scan", "/diag/link"]):
            obj = json.loads(message.data)
            if channel.topic == "/diag/link":
                fw_id = fw_id if fw_id is not None else obj.get("fw_id")
                # STM32 が答えた値（接続直後は空。最後に見えた値を採る）
                link.update({k: float(v) for k, v in (obj.get("control_params") or {}).items()
                             if k in ("steer_link_gain", "steer_link_cubic")})
            elif channel.topic == "/vehicle_state":
                vs_raw.append((int(obj.get("t_capture", 0)), message.log_time, obj))
            elif channel.topic == "/cmd":
                cmd_raw.append((int(obj.get("t_pub", 0)), message.log_time, obj))
            else:
                # スキャン完了 = 最後のセクタの先頭点の時刻 ＋ そのセクタの所要時間
                durs = obj.get("sector_dur_us") or []
                t_done = max((int(x) + (int(durs[i]) * 1000 if i < len(durs) else 0)
                              for i, x in enumerate(obj.get("sector_t_ns", [])) if x), default=0)
                scan_raw.append((t_done, message.log_time, obj))

    # 単調時刻が欠けたログ（古い形式）はヘッダの log_time で揃える（混ぜない）
    use_mono = all(m for m, _, _ in vs_raw + cmd_raw)
    pick = (lambda m, lt: m) if use_mono else (lambda m, lt: lt)
    vs_msgs = [(pick(m, lt) / NS, o) for m, lt, o in vs_raw]
    cmds = [_cmd_from_json(pick(m, lt) / NS, o) for m, lt, o in cmd_raw]
    scans = []
    if use_mono:
        for t_done, _lt, o in scan_raw:
            if t_done:
                scans.append(ScanStamp(t_done / NS, int(o.get("t_pub", 0)) / NS,
                                       int(o.get("seq", 0))))
    log = build_log(vs_msgs, cmds, scans)
    log.fw_id = fw_id
    log.steer_link = (link.get("steer_link_gain", 1.0), link.get("steer_link_cubic", 0.0))
    return log


def load_samples(path: str | Path) -> list[Sample]:
    return load_log(path).samples


def _arr(samples: list[Sample], attr: str) -> np.ndarray:
    return np.array([getattr(s, attr) for s in samples], dtype=float)


def _gyro_bias(samples: list[Sample]) -> tuple[float, int]:
    """止まっている区間（速度≈0・指令0）の `yaw_rate` の中央値＝ジャイロのバイアス。"""
    still = [s.yaw_rate for s in samples
             if abs(s.speed) < 0.01 and abs(s.target_speed) < 1e-6 and not s.brake] or \
        [s.yaw_rate for s in samples if abs(s.speed) < 0.01 and abs(s.target_speed) < 1e-6]
    if len(still) < 20:
        return 0.0, len(still)
    return float(np.median(still)), len(still)


# ── ① ステア ─────────────────────────────────────────────────────────────

#: エコーの量子化（0.0001rad）より十分大きい「動いた」の閾値
_ECHO_EPS = 3e-4
#: 物理上限を採る尤度比の閾値（自由度1のχ²で p≈0.002）
_CAP_LR_THRESHOLD = 10.0
#: 残差がノイズの床のこの倍を超えたら「モデルの形から外れている」と警告する
_RESID_WARN_RATIO = 2.0


def _reconstruct_echo(t: np.ndarray, echo: np.ndarray,
                      rate_hint: float) -> tuple[np.ndarray, np.ndarray]:
    """50Hzのエコーから、ランプの始点・終点を補間した折れ線 `(bp_t, bp_v)` を作る。

    ランプの途中のサンプルが2点以上あれば直線を当てはめ、前後の平坦部と交わる点を
    始点・終点にする（サンプル間隔より細かく決まる）。1点以下なら COMMAND の
    `steer_rate_limit` の傾きを使い、それも無ければ（瞬時のステップ）サンプル間の中点。
    """
    n = len(t)
    flat_edge = np.abs(np.diff(echo)) <= _ECHO_EPS          # i と i+1 が同じ値か
    in_plateau = np.zeros(n, dtype=bool)
    in_plateau[:-1] |= flat_edge
    in_plateau[1:] |= flat_edge
    plateaus: list[tuple[int, int]] = []                     # (first, last)
    i = 0
    while i < n:
        if in_plateau[i]:
            j = i
            while j + 1 < n and flat_edge[j]:
                j += 1
            plateaus.append((i, j))
            i = j + 1
        else:
            i += 1
    if not plateaus:
        return t.copy(), echo.copy()

    bp_t = [t[0]]
    bp_v = [float(np.median(echo[plateaus[0][0]:plateaus[0][1] + 1]))]
    for (a0, a1), (b0, b1) in zip(plateaus, plateaus[1:]):
        va = float(np.median(echo[a0:a1 + 1]))
        vb = float(np.median(echo[b0:b1 + 1]))
        delta = vb - va
        ramp = list(range(a1 + 1, b0))
        ta, tb = float(t[a1]), float(t[b0])
        slope = 0.0
        if len(ramp) >= 2:
            slope = float(np.polyfit(t[ramp], echo[ramp], 1)[0])
        elif rate_hint > 0:
            slope = math.copysign(rate_hint, delta)
        if abs(slope) > 1e-6 and slope * delta > 0:
            if ramp:
                t_on = float(t[ramp[0]]) - (float(echo[ramp[0]]) - va) / slope
            else:
                dur = abs(delta / slope)
                t_on = 0.5 * (ta + tb) - 0.5 * dur
            t_on = min(max(t_on, ta), float(t[ramp[0]]) if ramp else tb)
            t_end = min(max(t_on + delta / slope, float(t[ramp[-1]]) if ramp else t_on), tb)
        else:
            t_on = t_end = 0.5 * (ta + tb)
        bp_t += [t_on, max(t_end, t_on + 1e-6)]
        bp_v += [va, vb]
    bp_t.append(max(float(t[-1]), bp_t[-1] + 1e-6))
    bp_v.append(bp_v[-1])
    return np.array(bp_t), np.array(bp_v)


def _servo_response(bp_t: np.ndarray, bp_v: np.ndarray, t_eval: np.ndarray, y0: float,
                    dead: float, tau: float, cap: float, dt: float = 0.002,
                    hyst: float = 0.0, gain: float = 1.0) -> np.ndarray:
    """`SteerServo` と同じ式（中点でむだ時間ぶん遡った参照 → ×定常ゲイン → 1次遅れ → 上限 →
    ヒステリシス）の応答。

    むだ時間は参照を連続時間でずらして与える（`SteerServo`の履歴引きと同じ意味で、
    `dead`について連続になるので最適化しやすい）。上限なしなら `lfilter` で速く解く。
    """
    t0 = float(t_eval[0])
    k = np.arange(int(math.ceil((float(t_eval[-1]) - t0) / dt)) + 1)
    u = np.interp(t0 + (k + 0.5) * dt - dead, bp_t, bp_v) * gain
    if cap <= 1e-6:
        a = math.exp(-dt / tau) if tau > 1e-6 else 0.0
        y = lfilter([1.0 - a], [1.0, -a], u, zi=[a * y0])[0]
    else:
        y = np.empty_like(u)
        yk = y0
        for i, uk in enumerate(u.tolist()):
            yk = SteerServo.lag_step(yk, uk, dt, tau, cap)
            y[i] = yk
    if hyst > 0.0:
        out = np.empty_like(y)
        a = y0
        for i, z in enumerate(y.tolist()):
            a = backlash(a, z, hyst)
            out[i] = a
        y = out
    return np.interp(t_eval, t0 + (k + 1) * dt, y)


#: 走行中とみなす速度 [m/s]（ステア試験の低速部0.2m/sと走行部1.0m/sを分ける）
_ROLLING_M_S = 0.5
#: ステア試験の走行部（1.0m/s）と階段部（0.3〜0.6m/s。2026-09-26 までの記録は掃引部）を指令速度で分ける境目 [m/s]
_ZIG_SPEED_M_S = 0.8
#: これより大きい舵の指令だけをジグザグのステップとみなす（直線へ戻す補正舵
#: `sysid_steer.LANE_KEEP_MAX_RAD`=2°を除く）
_ZIG_MIN_RAD = math.radians(3.0)


def _fit_dead_tau(sse, starts, cap: float) -> tuple[float, float, float]:
    """`sse(dead, tau, cap)` を最小にする (むだ時間, 時定数, 誤差)。log(tau) で探す。"""
    best = None
    for d0, tau0 in starts:
        r = minimize(lambda x: sse(min(max(x[0], 0.0), 0.2), math.exp(x[1]), cap),
                     x0=[d0, math.log(tau0)], method="Nelder-Mead",
                     options={"xatol": 1e-4, "fatol": 1e-10, "maxiter": 400})
        if best is None or r.fun < best.fun:
            best = r
    return min(max(float(best.x[0]), 0.0), 0.2), math.exp(float(best.x[1])), float(best.fun)


def _steer_noise(s: list[Sample]) -> float:
    """止まっていて参照舵角も動いていない間の `steer_actual` のノイズの標準偏差 [rad]（残差の床）。

    隣り合うサンプルの差の標準偏差 /√2 で測る（止まっている舵角が区間ごとに違っても、
    ゆっくり落ち着いていく途中でも、ノイズだけが残る）。"""
    d = [s[i].steer_actual - s[i - 1].steer_actual for i in range(2, len(s))
         if all(abs(x.speed) < 0.01 and abs(x.target_speed) < 1e-6 for x in (s[i], s[i - 1]))
         and abs(s[i].steer_cmd_echo - s[i - 2].steer_cmd_echo) <= _ECHO_EPS]
    if len(d) < 20:
        return 0.0
    return float(np.std(d)) / math.sqrt(2.0)


#: 止まる位置の遅れを直接読む階段の段差の上限 [rad]（これより大きい段差は行き過ぎてから止まる）
_STAIR_SMALL_STEP_RAD = math.radians(3.5)


def _stair_hysteresis(s: list[Sample]) -> tuple[float, int] | None:
    """ステア試験の階段部（段を保持、`sysid_steer`）の**小さな段差**から、サーボの止まる位置の幅
    （`steer_servo_hysteresis_rad`、`sim/vehicle.py` の `SteerServo` のヒステリシス）を直接読む。

    段ごとに前進区間の後半の実舵角と参照（エコー）の差を、段が動いた向きで符号を揃えて「手前に
    止まった量」にし、その中央値の2倍を幅とする。実機（2026-09-27）のサーボは大きな段差では目標を
    行き過ぎてから止まり（15°→17.9°→13.7°）、1次遅れ＋ヒステリシスのモデルに全体を当てはめると
    幅を 1.6° と読んだが、小さな段差では同じ0°指令で上り−1.34°・下り+1.48°（幅約2.8°）に止まる。
    学習中の車の舵はゆっくり動くので、効くのは小さな段差の方。返り値 (幅, 使った段の数)、段が
    4つ未満なら None。"""
    runs = _phase_runs(s, lambda x: 0.0 < x.target_speed < _ZIG_SPEED_M_S and not x.brake)
    lags = []
    prev_level = None
    for run in runs:
        # 指令の舵角が一定の区切り
        pieces, cur = [], [run[0]]
        for i in run[1:]:
            if abs(s[i].target_steer - s[cur[-1]].target_steer) > 1e-4:
                pieces.append(cur)
                cur = []
            cur.append(i)
        pieces.append(cur)
        for piece in pieces:
            level = s[piece[0]].target_steer
            fwd = [i for i in piece if s[i].speed > 0.1]
            if prev_level is not None and 1e-4 < abs(level - prev_level) <= _STAIR_SMALL_STEP_RAD and len(fwd) >= 10:
                tail = fwd[len(fwd) // 2:]
                a = float(np.median([s[i].steer_actual for i in tail]))
                e = float(np.median([s[i].steer_cmd_echo for i in tail]))
                lags.append((e - a) * math.copysign(1.0, level - prev_level))
            prev_level = level
    if len(lags) < 4:
        return None
    return max(0.0, 2.0 * float(np.median(lags))), len(lags)


def _servo_overshoot(s: list[Sample]) -> tuple[float, int] | None:
    """5°以上の段差（エコーのランプが止まった時点から0.3s）で、実舵角が目標を行き過ぎた量の中央値
    [rad] と段の数。シムのサーボ（1次遅れ）には行き過ぎが無いので、★の所見でその大きさを知らせる。"""
    t = np.array([x.t for x in s])
    e = np.array([x.steer_cmd_echo for x in s])
    a = np.array([x.steer_actual for x in s])
    moving = np.abs(np.diff(e)) > _ECHO_EPS
    over = []
    k = 1
    while k < len(e):
        if moving[k - 1]:
            j = k
            while j < len(moving) and moving[j]:
                j += 1
            d = e[j] - e[k - 1]
            if abs(d) >= math.radians(5.0):
                m = (t >= t[j]) & (t <= t[j] + 0.3)
                if np.any(m):
                    over.append(max(0.0, float(np.max((a[m] - e[j]) * math.copysign(1.0, d)))))
            k = j + 1
        else:
            k += 1
    return (float(np.median(over)), len(over)) if len(over) >= 4 else None


def fit_steer(log: Log) -> FitResult:
    """ステア試験 → `dead_time_s`・`tau_steer_s`・`steer_rate_limit_rad_s`・
    `steer_servo_hysteresis_rad`・`steer_servo_gain`。

    - `steer_rate_limit_rad_s` はサーボ出力側の物理上限。参照（エコー）自体が COMMAND の
      レート制限でランプするので、それより速い上限は観測できない——上限を入れても残差が
      ノイズ以上に縮まらなければ 0（制限なし）を返す。大振幅のステップが要る
    - サーボの定常ゲイン（位置ループの定常偏差）・ヒステリシス・物理上限は、それぞれ
      入れると残差が有意に縮むときだけ採る（尤度比。自由度は残差の自己相関から、`n_eff`）。
      定常ゲインを持たないと、定常偏差がヒステリシスとむだ時間の偏りに化けた（2026-09-26）
    - むだ時間・時定数は、**走行中（`_ROLLING_M_S`以上）のステップ**が4回以上あれば
      そこだけで当てはめ直した値を返す（学習中の車は走りながら舵を切るため）。低速部の
      値は所見に並べる——大きく違えばサーボの応答が速度（負荷）に依存している
    - 残差が静止中の舵角のノイズより大きく上回れば、モデルの形から外れていると警告する
    """
    s = [x for x in log.samples]
    if len(s) < 50:
        raise ValueError("記録が短すぎます（mcapが正しいか確認）")
    t = _arr(s, "t")
    echo = _arr(s, "steer_cmd_echo")
    actual = _arr(s, "steer_actual")
    speed = _arr(s, "speed")
    if np.ptp(echo) < math.radians(3.0):
        raise ValueError("舵角のステップ入力が検出できませんでした（mcapが正しいか確認）")
    if np.ptp(actual) < 0.3 * np.ptp(echo):
        raise ValueError("steer_actualがほとんど動いていません（配線・ARMを確認）")

    rate_cmd = float(np.median([x.steer_rate_limit for x in s]))
    bp_t, bp_v = _reconstruct_echo(t, echo, rate_cmd)
    y0 = float(actual[0])

    # ステップごとの窓（始点の少し前から0.45s）を、始点の速度で走行中／低速に分ける。
    # 3°未満の変化（直線へ戻す補正舵など）はステップとして数えない
    onsets = [bp_t[i] for i in range(1, len(bp_t) - 1, 2)
              if abs(bp_v[i + 1] - bp_v[i]) >= _ZIG_MIN_RAD]
    rolling = np.zeros(len(t), dtype=bool)
    creep = np.zeros(len(t), dtype=bool)
    n_roll = n_creep = 0
    for on in onsets:
        w = (t >= on - 0.05) & (t <= on + 0.45)
        if abs(float(np.interp(on, t, speed))) >= _ROLLING_M_S:
            rolling |= w
            n_roll += 1
        else:
            creep |= w
            n_creep += 1

    hyst = 0.0
    sg = 1.0
    cap = 0.0

    def resp(dead_: float, tau_: float, cap_: float, h: float, g: float) -> np.ndarray:
        return _servo_response(bp_t, bp_v, t, y0, dead_, tau_, cap_, hyst=h, gain=g) - actual

    def make_sse(mask: np.ndarray | None):
        def sse(dead: float, tau: float, cap_: float) -> float:
            r = resp(dead, tau, cap_, hyst, sg)
            if mask is not None:
                r = r[mask]
            return float(np.mean(r * r))
        return sse

    starts = ((0.01, 0.05), (0.03, 0.1), (0.005, 0.2))
    sse_all = make_sse(None)
    # ① 全体・上限なしで (むだ時間, 時定数)
    dead, tau, err = _fit_dead_tau(sse_all, starts, 0.0)
    ref_slope = float(np.max(np.abs(np.diff(bp_v) / np.maximum(np.diff(bp_t), 1e-6))))

    def nm(f, x0):
        return minimize(f, x0=x0, method="Nelder-Mead",
                        options={"xatol": 1e-4, "fatol": 1e-12, "maxiter": 500})

    # ② サーボの定常ゲイン（位置ループの定常偏差: 実舵角が参照の一定割合の手前で止まる）
    def fit_gain() -> None:
        nonlocal dead, tau, sg, err
        sg = 1.0
        err = sse_all(dead, tau, cap)
        r = nm(lambda x: float(np.mean(resp(min(max(x[0], 0.0), 0.2), math.exp(x[1]), cap,
                                                hyst, x[2]) ** 2)),
               [dead, math.log(tau), 1.0])
        g_ = float(r.x[2])
        r1 = resp(min(max(float(r.x[0]), 0.0), 0.2), math.exp(float(r.x[1])), cap, hyst, g_)
        if _lr_significant(n_eff(r1), float(r.fun), err):
            dead = min(max(float(r.x[0]), 0.0), 0.2)
            tau = math.exp(float(r.x[1]))
            sg = g_
            err = float(r.fun)

    # ③ サーボのヒステリシス（止まる位置が、近づいた側に幅の半分だけ手前になる）。
    #    階段・掃引（同じ舵角に左右から近づく）が無いと、1次遅れの定常偏差と区別できない
    stair = _stair_hysteresis(s)

    def fit_hyst() -> None:
        nonlocal dead, tau, hyst, err
        if stair is not None:
            # 階段の小さな段差から直接読んだ幅で固定し、むだ時間・時定数だけ当てはめ直す
            hyst = stair[0]
            r = nm(lambda x: float(np.mean(resp(min(max(x[0], 0.0), 0.2), math.exp(x[1]), cap,
                                                    hyst, sg) ** 2)), [dead, math.log(max(tau, 1e-4))])
            dead, tau = min(max(float(r.x[0]), 0.0), 0.2), math.exp(float(r.x[1]))
            err = float(r.fun)
            return
        hyst = 0.0
        err = sse_all(dead, tau, cap)
        hs = np.radians(np.geomspace(0.05, 4.0, 14))
        errs_h = [float(np.mean(resp(dead, tau, cap, h, sg) ** 2)) for h in hs]
        kh = int(np.argmin(errs_h))
        r = nm(lambda x: float(np.mean(resp(min(max(x[0], 0.0), 0.2), math.exp(x[1]), cap,
                                                math.exp(x[2]), sg) ** 2)),
               [dead, math.log(tau), math.log(hs[kh])])
        d_, t_, h_ = min(max(float(r.x[0]), 0.0), 0.2), math.exp(float(r.x[1])), math.exp(float(r.x[2]))
        if _lr_significant(n_eff(resp(d_, t_, cap, h_, sg)), float(r.fun), err):
            dead, tau, hyst, err = d_, t_, h_, float(r.fun)

    # 定常ゲインとヒステリシスは互いに補い合うので2周（片方を入れた後にもう片方を見直す）
    for _ in range(2):
        fit_gain()
        fit_hyst()

    # ④ 物理上限（参照のランプより遅い上限だけが観測できる）。上限が効くのは大きな
    #    ステップ（±30°）の直後の短い間だけなので、**その区間だけを取り出し**、上限なし・
    #    ありの両方で (むだ時間, 時定数) も当てはめ直して尤度比で比べる。全体で当てはめた
    #    時定数を固定して上限だけ試すと、時定数がすでに上限の分を吸っていて見落とした
    #    （ベンチのランダムな真値で、上限4.3rad/s・時定数0.11sを見落とした）
    big = [bp_t[i] for i in range(1, len(bp_t) - 1, 2)
           if abs(bp_v[i + 1] - bp_v[i]) >= math.radians(20.0)]
    wins = []
    for on in big:
        i0 = int(np.searchsorted(t, on - 0.05))
        i1 = int(np.searchsorted(t, on + 0.5))
        if i1 - i0 >= 10:
            wins.append((i0, i1))
    if ref_slope > 1.0 and len(wins) >= 2:
        def res_w(dead_: float, tau_: float, cap_: float) -> np.ndarray:
            return np.concatenate([
                _servo_response(bp_t, bp_v, t[i0:i1], float(actual[i0]), dead_, tau_, cap_,
                                hyst=hyst, gain=sg) - actual[i0:i1] for i0, i1 in wins])

        def sse_w(dead_: float, tau_: float, cap_: float) -> float:
            r_ = res_w(dead_, tau_, cap_)
            return float(np.mean(r_ * r_))

        d0, t0_, e_nocap = _fit_dead_tau(sse_w, ((dead, tau),), 0.0)
        caps = np.geomspace(1.0, min(ref_slope, 50.0), 12)
        best_c = None
        for c in caps[caps < 0.9 * ref_slope]:
            d_, t_, e_ = _fit_dead_tau(sse_w, ((dead, tau),), float(c))
            if best_c is None or e_ < best_c[0]:
                best_c = (e_, float(c), d_, t_)
        if best_c is not None and _lr_significant(n_eff(res_w(best_c[2], best_c[3], best_c[1])),
                                                  best_c[0], e_nocap):
            r = nm(lambda x: sse_w(min(max(x[0], 0.0), 0.2), math.exp(x[1]), math.exp(x[2])),
                   [best_c[2], math.log(best_c[3]), math.log(best_c[1])])
            cap = math.exp(float(r.x[2]))
            if cap < 0.9 * ref_slope:
                err = sse_all(dead, tau, cap)
            else:
                cap = 0.0

    if cap > 0.0:
        # 上限が見つかったら、定常ゲイン・ヒステリシスを上限込みで推定し直す（先に上限なしで
        # 決めた値は、上限の分の食い違いを抱えたまま決まっている。ベンチで0.56°を0.33°と読んだ）
        fit_gain()
        fit_hyst()

    sigma = _steer_noise(s)
    rms = math.sqrt(err)
    notes = [f"残差RMS {math.degrees(rms):.2f}°（舵の振れ幅 {math.degrees(np.ptp(echo)):.0f}°"
             + (f"、静止中の舵角のノイズ {math.degrees(sigma):.2f}°）" if sigma > 0 else "）"),
             (f"サーボのヒステリシス（止まる位置の幅） {math.degrees(hyst):.2f}°"
              + (f"（階段の小さな段差 {stair[1]} 段から直接）" if stair is not None else ""))
             if hyst > 0 else "サーボのヒステリシスは検出されず（0）",
             f"サーボの定常ゲイン {sg:.3f}" if sg != 1.0 else "サーボの定常偏差は検出されず（ゲイン1）"]
    warnings: list[str] = []
    if sigma > 0 and rms > _RESID_WARN_RATIO * sigma + math.radians(0.05):
        ov = _servo_overshoot(s)
        notes.append(f"★残差が舵角のノイズの {rms / sigma:.1f} 倍。サーボの応答がシムのモデル"
                     "（むだ時間→定常ゲイン→1次遅れ→上限→ヒステリシス）の形から外れている。"
                     + (f"5°以上の段差で目標を平均 {math.degrees(ov[0]):.1f}° 行き過ぎている（{ov[1]} 段。"
                        "シムのサーボに行き過ぎは無い）。" if ov is not None and ov[0] > 3 * sigma else "")
                     + "値をそのまま使わず、応答の形（行き過ぎ・速度依存など）を確かめること")
        warnings = [notes[-1]]
    # ⑤ 走行中・低速それぞれで当てはめ直す（上限・ヒステリシス・定常ゲインは②〜④の値で固定）
    parts = {}
    for name, mask, n in (("走行中", rolling, n_roll), ("低速", creep, n_creep)):
        if n >= 4:
            parts[name] = _fit_dead_tau(make_sse(mask), ((dead, tau),), cap)
            d_, t_, e_ = parts[name]
            notes.append(f"{name}（{n}回）: むだ時間 {d_ * 1000:.1f}ms・時定数 {t_ * 1000:.1f}ms・"
                         f"残差 {math.degrees(math.sqrt(e_)):.2f}°")
    if "走行中" in parts:
        dead, tau, _ = parts["走行中"]
        notes.append("返す値は走行中のステップから（学習中の車は走りながら舵を切る）")
    else:
        notes.append("走行中のステップが無い（roll_speed=0？）。低速も含めた全体の値を返す")
    if cap == 0.0:
        notes.append(f"サーボの物理上限は観測できず（参照のランプ {ref_slope:.1f}rad/s より速い）"
                     "→ 0（制限なし）。シムでは COMMAND のレート制限が効く")
    if rate_cmd > 0:
        notes.append(f"エコーのランプ {ref_slope:.2f}rad/s（COMMAND の steer_rate_limit {rate_cmd:.2f}rad/s）"
                     "——一致していればシムの参照ランプの再現は正しい")
    return FitResult({"dead_time_s": dead, "tau_steer_s": tau,
                      "steer_rate_limit_rad_s": cap, "steer_servo_hysteresis_rad": hyst,
                      "steer_servo_gain": sg}, notes, warnings)


# ── ② 前後運動（速度制御の応答。2026-09-26 までは速度応答試験） ───────────────

#: 速度の出力誤差の刻み [s]（ファームは2kHz。PI の遅れに対して十分細かい）
_SPEED_SIM_DT = 0.004


@dataclass
class _SpeedRun:
    """前進の指令で始まる区間（制動・後退指令を含まない）と、その頭の制御器の状態。"""

    idx: list[int]
    t: np.ndarray
    v: np.ndarray
    ref0: float
    target0: float
    #: シミュレーションを始める時刻と速度（区間の1つ前のサンプル＝まだ前の指令が効いている時点。
    #: 区間の最初のサンプルから始めると、指令が変わってからそのサンプルまでの最大20msぶん、
    #: 制御器がすでに動き出しているのを取りこぼした）
    t_start: float = 0.0
    v_start: float = 0.0
    #: 区間の前後に微分の窓2つ分を足したサンプルの時刻・オドメトリ・舵角と、その中での区間の
    #: 1つ前（`t_start`）・最後のサンプルの位置。`_simulate_speed` が実測と同じ微分を掛けるのに使う
    #: （None なら微分を掛けず、シムの速度をそのまま返す）
    win_t: np.ndarray | None = None
    win_x: np.ndarray | None = None
    win_d: np.ndarray | None = None
    k_start: int = 0
    k_end: int = 0


def _speed_runs(s: list[Sample], min_s: float = 0.5) -> list[_SpeedRun]:
    """制動も後退の指令も無く、前進の指令で始まる区間。直前の指令（後退の戻り・静止）が
    定常だったとみなして、制御器の目標のランプの位置を引き継ぐ。"""
    t_all = _arr(s, "t")
    od_all = _arr(s, "odom")
    d_all = _arr(s, "steer_actual")
    out = []
    for r in _phase_runs(s, lambda x: not x.brake and x.target_speed >= 0.0):
        k = next((j for j, i in enumerate(r) if s[i].target_speed > 1e-6), None)
        if k is None:
            continue
        r = r[k:]
        if s[r[-1]].t - s[r[0]].t < min_s:
            continue
        prev = s[r[0] - 1] if r[0] > 0 else s[r[0]]
        v0 = prev.speed
        ref0 = prev.target_speed if abs(v0 - prev.target_speed) < 0.15 else v0
        i_prev = r[0] - 1 if r[0] > 0 else r[0]
        a = int(np.searchsorted(t_all, t_all[i_prev] - 2 * _ODOM_DIFF_HALF_S - 1e-9))
        b = int(np.searchsorted(t_all, t_all[r[-1]] + 2 * _ODOM_DIFF_HALF_S + 1e-9, side="right"))
        out.append(_SpeedRun(r, _arr([s[i] for i in r], "t"), _arr([s[i] for i in r], "speed"),
                             ref0, prev.target_speed, prev.t, v0,
                             t_all[a:b], od_all[a:b], d_all[a:b], i_prev - a, r[-1] - a))
    return out


def _simulate_speed(spec: VehicleSpec, cmds: list[Cmd], delay: float, run: _SpeedRun,
                    dt: float = _SPEED_SIM_DT) -> np.ndarray:
    """`SpeedController`（ファームの速度制御）に `/cmd` を `delay` だけ遅らせて入れた応答を、
    **実測の速度と同じ操作**（オドメトリの窓の直線当てはめ、`odom_speed`）を通して区間の時刻で返す。

    実測の速度は前後 `_ODOM_DIFF_HALF_S` の窓で微分したものなので、加速度が急に変わる所では
    窓の幅だけ鈍る。区間の末尾（全開加速→制動）では窓の後ろ半分が制動に掛かり、区間の最後の
    サンプルで 5cm/s ほど低く読む。シムの速度をそのまま比べるとこの差を「最高速付近で加速度が
    落ちる」が説明してしまい、上限の無い真値で約2.3m/sの最高速を返した（2026-09-26 の第三者検証、
    8シード中3例）。そこで区間の中はシムの速度を積分した位置、区間の外は実測のオドメトリの
    増分をつないだ位置に、実測と同じ窓の当てはめを掛けてから比べる。"""
    if run.win_t is None:
        return _simulate_speed_raw(spec, cmds, delay, run, run.t, dt)
    t, od, d = run.win_t, run.win_x, run.win_d
    ks, ke = run.k_start, run.k_end
    fine = t[ks] + np.arange(1, int(math.ceil((t[ke] - t[ks]) / dt)) + 1) * dt
    v = _simulate_speed_raw(spec, cmds, delay, run, fine, dt)
    tt = np.concatenate([[t[ks]], fine])
    # 車速（車体中心線方向）→ 前輪の走行距離の進み（実測の射影 cos の逆）
    rate = np.concatenate([[run.v_start], v]) / np.maximum(np.cos(np.interp(tt, t, d)), 0.2)
    pos = np.concatenate([[0.0], np.cumsum(0.5 * (rate[1:] + rate[:-1]) * np.diff(tt))])
    x = od.astype(float).copy()
    x[ks + 1:ke + 1] = od[ks] + np.interp(t[ks + 1:ke + 1], tt, pos)
    x[ke + 1:] = x[ke] + (od[ke + 1:] - od[ke])
    return (_odom_slope(t, x) * np.cos(d))[ke + 1 - len(run.t):ke + 1]


def _simulate_speed_raw(spec: VehicleSpec, cmds: list[Cmd], delay: float, run: _SpeedRun,
                        t_eval: np.ndarray, dt: float = _SPEED_SIM_DT) -> np.ndarray:
    """`_simulate_speed` のシムの部分（微分を掛ける前の速度を `t_eval` で）。
    区間の頭の状態: 速度は実測、目標のランプは直前の指令、積分は転がり抵抗と釣り合う値。"""
    t0, t1 = float(run.t_start), float(t_eval[-1])
    cmd_t = [c.t + delay for c in cmds]
    inputs = [DriveInput(armed=True, brake=c.brake, target_speed=c.target_speed,
                         accel_limit=c.accel_limit, brake_torque=c.brake_torque) for c in cmds]
    ctl = SpeedController(spec)
    v = float(run.v_start)
    i0 = 0.0
    if abs(run.ref0) >= SpeedController.STANDSTILL_M_S and spec.speed_ki > 1e-9 and spec.speed_plant_gain > 1e-9:
        i0 = math.copysign(spec.rolling_resistance / spec.speed_plant_gain / spec.speed_ki, run.ref0)
    ctl.reset(v=v, ref=run.ref0, integral=i0)
    idle = DriveInput(armed=True, target_speed=run.target0)
    n = int(math.ceil((t1 - t0) / dt)) + 1
    out = np.empty(n)
    j = int(np.searchsorted(cmd_t, t0, side="right")) - 1
    for k in range(n):
        tk = t0 + k * dt
        t_end = tk + dt
        # 指令の切り替えは刻みの途中でも正確な時刻で入れる（刻みに丸めると応答が `delay` に
        # ついて階段状になり、最小二乗の勾配が0になって遅れが初期値から動かなかった）
        while j + 1 < len(cmd_t) and cmd_t[j + 1] < t_end:
            if cmd_t[j + 1] > tk:
                h = cmd_t[j + 1] - tk
                v = ctl.step(inputs[j] if j >= 0 else idle, v, h)
                tk += h
            j += 1
        v = ctl.step(inputs[j] if j >= 0 else idle, v, t_end - tk)
        out[k] = v
    return np.interp(t_eval, t0 + (np.arange(n) + 1) * dt, out)


def _speed_noise(s: list[Sample]) -> float:
    """静止中（指令0・止まっている）のオドメトリ由来の速度の標準偏差 [m/s]（残差の床）。"""
    x = np.array([x.speed for x in s if abs(x.target_speed) < 1e-6 and abs(x.speed_filtered) < 0.03
                  and not x.brake])
    return float(np.std(x)) if len(x) >= 20 else 0.0


def _speed_noise_moving(s: list[Sample]) -> float:
    """走っている間（前進・指令が0.4s以上変わらない区間の後半）の、オドメトリ由来の速度の
    ばらつき [m/s]（区間ごとに直線を引いた残りの標準偏差の中央値）。前輪エンコーダのうねりや
    振動で静止中より大きい（実機で静止中0.2cm/s に対し 0.7〜2.7cm/s）。"""
    keep = _steady_tail_mask(s, lambda x: not x.brake and x.target_speed > 0.2, 0.4)
    idx = np.where(keep)[0]
    out = []
    for g in np.split(idx, np.where(np.diff(idx) > 1)[0] + 1):
        if len(g) >= 10:
            t = np.array([s[i].t for i in g])
            v = np.array([s[i].speed for i in g])
            out.append(float(np.std(v - np.polyval(np.polyfit(t, v, 1), t))))
    return float(np.median(out)) if out else 0.0


def _speed_floor(s: list[Sample]) -> tuple[float, str]:
    """残差と比べる速度のノイズ [m/s] と説明。静止中と走っている間の大きい方（2026-09-27。静止中
    だけを基準にしていたので、実機の記録で残差がノイズ程度でも★「7倍」と出た）。"""
    still, moving = _speed_noise(s), _speed_noise_moving(s)
    return max(still, moving), f"速度のノイズ 静止中 {still * 100:.1f}・走行中 {moving * 100:.1f}cm/s"


def _require_pi(base: VehicleSpec) -> None:
    if base.speed_kp <= 1e-9 or base.speed_ki <= 1e-9:
        raise ValueError("vehicle.toml の [dynamics] に speed_kp/speed_ki（STM32 の速度PIの定数）が"
                         "ありません。ファームの DRIVE_SPEED_KP/KI を書いてから解析してください")


#: `least_squares` に渡す「上限なし」の代わりの値 [m/s²]（これより大きい上限は効かない）
_NO_CAP = 50.0


def _fit_speed_model(logs: Log | list[Log], base: VehicleSpec,
                     free: dict[str, tuple[float, float, float]],
                     fixed: dict[str, float] | None = None, starts: list[dict[str, float]] | None = None):
    """`SpeedController` の出力誤差。`free` は キー → (初期値, 下限, 上限)（`delay` は遅れ [s]、
    全記録で共通）。`starts` は初期値の上書きの候補（最も残差の小さい解を採る）。
    返り値 `(値, 平均二乗誤差, 残差)`。"""
    logs = logs if isinstance(logs, list) else [logs]
    all_sets = [(lg.cmds, _speed_runs(lg.samples)) for lg in logs]
    if not any(runs for _, runs in all_sets):
        raise ValueError("前進の速度指令の区間が見つかりません（mcapが正しいか確認）")
    # TC の介入・空転を含む区間は外す（タイヤのグリップで加速が決まっていて、速度制御のモデルの形から
    # 外れる。含めると外れた分を車体の利得・転がり抵抗が吸収し、TC を写した真値で 22→17.6 と壊れた）。
    # 全部が外れる記録では外さない（残差の★で知らせる）
    sets = [(cmds, [r for r in runs if not _traction_event(lg.samples, r.idx)])
            for (cmds, runs), lg in zip(all_sets, logs)]
    if not any(runs for _, runs in sets):
        sets = all_sets
    names = list(free)
    fx = dict(fixed or {})

    def spec_of(x):
        p = dict(fx)
        p.update(zip(names, (float(v) for v in x)))
        delay = p.pop("delay", 0.0)
        return replace(base, **p), delay

    def resid(x):
        sp, delay = spec_of(x)
        return np.concatenate([_simulate_speed(sp, cmds, delay, r) - r.v
                               for cmds, runs in sets for r in runs])

    lo = [free[n][1] for n in names]
    hi = [free[n][2] for n in names]
    r = None
    for st in (starts or [{}]):
        x0 = [min(max(st.get(n, free[n][0]), lo[i]), hi[i]) for i, n in enumerate(names)]
        ri = least_squares(resid, x0=x0, bounds=(lo, hi), diff_step=1e-3)
        if r is None or ri.cost < r.cost:
            r = ri
    p = dict(fx)
    p.update(zip(names, (float(v) for v in r.x)))
    return p, float(np.mean(r.fun ** 2)), r.fun


def fit_speed(log: Log, base: VehicleSpec | None = None) -> FitResult:
    """速度の記録（直線）→ 車体の側の `speed_plant_gain`・`rolling_resistance`。

    2026-09-26 までの速度応答試験の記録用に残してある。今の前後運動試験は `fit_accel()` が
    加減速の上限と一緒に当てはめる（`analyze` はそちらを使う）。

    ファームの速度制御（目標のランプ→PI→トルク上限、`SpeedController`）はファームの定数
    （`speed_kp` 等）で決まっているので、未知なのは「トルク → 加速度」の利得と転がり抵抗だけ。
    それを前進の区間ごとに実測速度（オドメトリ由来）へ当てはめる。加速度の上限と
    `/cmd`→STM32 の遅れも一緒に動かす（上限は加減速試験で、遅れは遅延試験で決める）。

    以前は1次遅れ（`tau_speed_s`）を当てはめていた。ファームの形（ランプ→2次応答）の真値に
    対しては時定数が探索範囲の下限に張り付き、残差を見ても判定が無いので気づけなかった
    （2026-09-26 の第三者検証）。今は残差を静止中のノイズと比べて警告する。
    """
    base = base or VehicleSpec.load()
    _require_pi(base)
    s = log.samples
    targets = {round(x.target_speed, 3) for x in s if x.target_speed > 1e-6 and not x.brake}
    if len(targets) < 2:
        raise ValueError("速度の段が1種類しかありません（v_low と v_high を変えて録り直す）")
    g0 = base.speed_plant_gain if base.speed_plant_gain > 0 else 1.0 / (base.wheel_radius * base.mass)
    # 加速・減速の上限は、効いていれば 0.4→1.2m/s の段でも頭打ちになる。値は加減速試験で
    # 決めるが、ここで動かさないと利得と転がり抵抗が上限の代わりに歪む（上限が「なし」の
    # 初期値に張り付くと勾配が無く動かない＝ランダムな真値の検証で見つかった）。効く・効かない
    # の両方から探して、残差の小さい方を採る
    p, err, res = _fit_speed_model(log, base, {
        "speed_plant_gain": (g0, 1.0, 500.0),
        "rolling_resistance": (min(max(base.rolling_resistance, 0.0), 2.0), 0.0, 3.0),
        "drive_accel_m_s2": (2.5, 0.5, _NO_CAP),
        "speed_decel_m_s2": (2.5, 0.5, _NO_CAP),
        "delay": (0.01, 0.0, 0.15),
    }, {"drive_fade_speed_m_s": 0.0, "drive_top_speed_m_s": 0.0},
        starts=[{}, {"drive_accel_m_s2": 1.8, "speed_decel_m_s2": 1.8},
                {"drive_accel_m_s2": 8.0, "speed_decel_m_s2": 8.0}])
    rms = math.sqrt(err)
    floor, floor_txt = _speed_floor(s)
    notes = [f"残差RMS {rms * 100:.1f}cm/s（{floor_txt}、"
             f"{len(_speed_runs(s))} 区間、{min(targets):.2f}〜{max(targets):.2f}m/s）",
             f"車体の利得 {p['speed_plant_gain']:.1f}(m/s²)/(N·m)（名目 "
             f"{1.0 / (base.wheel_radius * base.mass):.1f}）・転がり抵抗 {p['rolling_resistance']:.2f}m/s²・"
             f"/cmd→STM32 の遅れ {p['delay'] * 1000:.0f}ms"]
    warnings: list[str] = []
    if floor > 0 and rms > _RESID_WARN_RATIO * floor + 0.01:
        notes.append(f"★残差が速度のノイズの {rms / floor:.1f} 倍。車速の応答がシムの速度制御"
                     "（ファームの目標ランプ→PI→トルク上限）の形から外れている。ファームの"
                     "DRIVE_SPEED_KP/KI・DRIVE_MAX_ACCEL と vehicle.toml の speed_kp/speed_ki/"
                     "speed_ramp_max_m_s2 が一致しているか確かめること")
        warnings = [notes[-1]]
    return FitResult({"speed_plant_gain": p["speed_plant_gain"],
                      "rolling_resistance": p["rolling_resistance"]}, notes, warnings)


def _lpf(t: np.ndarray, x: np.ndarray, tau: float, order: int) -> np.ndarray:
    """1次遅れ `order` 段（サンプル間隔は実際の時刻から。取りこぼしがあっても崩れない）。

    入力はサンプル間を直線でつなぐ（1次ホールドの厳密な離散化、`_lpf_weights` と同じ）。
    0次ホールドだと半サンプル早く応答し、その分を時定数が吸った（80ms を 89ms と読んだ）。"""
    y = x.copy()
    if tau <= 1e-6:
        return y
    dt = np.diff(t).clip(1e-4, None)
    e = np.exp(-dt / tau)
    r = (1.0 - e) * tau / dt
    for _ in range(order):
        out = np.empty_like(y)
        out[0] = y[0]
        for i in range(1, len(y)):
            out[i] = e[i - 1] * out[i - 1] + (1.0 - r[i - 1]) * y[i] + (r[i - 1] - e[i - 1]) * y[i - 1]
        y = out
    return y


def sensor_check(log: Log, base: VehicleSpec | None = None) -> FitResult:
    """速度センサの自己点検（値は返さない。所見だけ）。

    - 静止中の `speed` とオドメトリ由来の速度のノイズ
    - オドメトリ由来の速度（同定に使う値）に `vehicle.toml` のローパス（ファームの定数）を
      掛けたものが、記録の `speed` と合うか。合わなければ、ファームのローパスが `vehicle.toml` と
      違う（＝シムの観測の遅れ、学習の入力が実機と食い違う）
    ファームの ADC 平均化・ローパスの変更（2026-09-25）で、ノイズがどれだけ残ったかは
    実機で未確認なので、最初の記録で必ず見る。
    """
    base = base or VehicleSpec.load()
    s = log.samples
    # 前輪エンコーダの周期誤差（`correct_encoder_ripple`。記録を読むときに補正済み）
    notes: list[str] = list(log.sensor_notes)
    t = _arr(s, "t")
    vf = _arr(s, "speed_filtered")
    vo = _arr(s, "speed")                         # オドメトリ由来（`_speed_from_odom`）
    still = np.array([abs(x.target_speed) < 1e-6 and abs(x.speed_filtered) < 0.03 for x in s])
    if np.count_nonzero(still) >= 20:
        notes.append(f"静止中のノイズ: speed {np.std(vf[still]) * 1000:.1f}mm/s・"
                     f"オドメトリ由来 {np.std(vo[still]) * 1000:.1f}mm/s（標準偏差）")
    if float(np.ptp(vo)) < 0.3:
        notes.append("オドメトリが動いておらず、speed のローパスの照合はできず")
        return FitResult({}, notes)
    tau, order = base.speed_filter_s, max(1, int(round(base.speed_filter_order)))
    moving = np.abs(vf) > 0.2

    def rms(tau_: float, order_: int) -> float:
        r = (_lpf(t, vo, tau_, order_) - vf)[moving]
        return float(np.sqrt(np.mean(r * r)))

    e_cfg = rms(tau, order)
    grid = np.arange(0.0, 0.2001, 0.0025)
    e_best, tau_best = min((rms(x, 1), x) for x in grid)
    notes.append(f"speed とオドメトリの照合: vehicle.toml のローパス（{tau * 1000:.1f}ms×{order}段）で"
                 f"残差 {e_cfg * 100:.1f}cm/s、記録に最も合う1段のローパスは {tau_best * 1000:.1f}ms"
                 f"（残差 {e_best * 100:.1f}cm/s）")
    if e_cfg > 1.5 * e_best + 0.005 and abs(tau_best - tau) > 0.01:
        notes.append("★vehicle.toml の speed_filter_s がファームの実際のローパスと合っていない可能性。"
                     "シムの観測（学習の入力）の遅れが実機とずれるので、ファームの DRIVE_LPF_K_FRONT を"
                     "確認して直すこと（同定の値はオドメトリから求めているので影響しない）")
    return FitResult({}, notes)


# ── ② 前後運動（加減速の上限） ─────────────────────────────────────────────

#: 傾きを読む窓の長さ [s] と、各フェーズの頭から捨てる時間 [s]（むだ時間・立ち上がり）
_SLOPE_WINDOW_S = 0.2
_PHASE_SKIP_S = 0.1


def _phase_runs(samples: list[Sample], pred) -> list[list[int]]:
    """`pred(sample)` が連続して真になる区間（添字のリスト）。"""
    runs, cur = [], []
    for i, x in enumerate(samples):
        if pred(x):
            cur.append(i)
        elif cur:
            runs.append(cur)
            cur = []
    if cur:
        runs.append(cur)
    return runs


def _window_slopes(samples: list[Sample], run: list[int]) -> list[tuple[float, float, int]]:
    """区間を窓に切って `(平均速度, 傾き, 窓の先頭添字)` を返す。頭 `_PHASE_SKIP_S` は捨てる。"""
    if not run:
        return []
    t = np.array([samples[i].t for i in run])
    v = np.array([samples[i].speed for i in run])
    keep = t >= t[0] + _PHASE_SKIP_S
    t, v = t[keep], v[keep]
    idx = [i for i, k in zip(run, keep) if k]
    out = []
    j = 0
    while j < len(t):
        m = (t >= t[j]) & (t < t[j] + _SLOPE_WINDOW_S)
        if np.count_nonzero(m) >= 5 and t[m][-1] - t[m][0] >= 0.6 * _SLOPE_WINDOW_S:
            out.append((float(np.mean(v[m])), float(np.polyfit(t[m], v[m], 1)[0]), idx[j]))
        j += max(1, np.count_nonzero(m) // 2)
    return out


#: 空転とみなす後輪と前輪の速度差（`sysid_accel` のプランナーと同じ形）と、続く時間 [s]。
#: プランナーは0.1s続いたら制動するので、記録の中で「制動なしで空転している」のはそれより短い
#: （空転の真値で0.09s）。ここは所見を出すだけなので半分にする
_SPIN_TOL_M_S = 0.2
_SPIN_TOL_FRAC = 0.3
_SPIN_PERSIST_S = 0.05


def _spinning(x: Sample) -> bool:
    """後輪が前進方向に回っていて、前輪より大きく速い（TC で抑えきれない空転）。前進を条件に
    するのは、加速の段の頭（まだ後退中）で後輪の値が無い記録（後輪0）を空転と取り違えないため
    （ベンチの mcap で 0.8m/s² の段を空転と読んだ、2026-09-27）。"""
    return x.wheel_speed_rear > 0.1 and \
        x.wheel_speed_rear - x.speed > _SPIN_TOL_M_S + _SPIN_TOL_FRAC * abs(x.speed)


def _slipping(x: Sample) -> bool:
    """加速中（制動なし・前進の指令）に後輪が滑っている（滑り率が `REAR_SLIP_LIMIT` を超える）か、
    抑えきれない空転。プランナー（`sysid_accel`）と同じ判定。TC の旗は見ない（2026-09-27、実機で
    TC の旗が30〜50msだけ立ったのを「介入」と数えて上の段を飛ばした）。"""
    if x.brake or x.target_speed <= 0.0 or x.wheel_speed_rear <= 0.1:
        return False
    return rear_slip(x.speed_filtered, x.wheel_speed_rear) > REAR_SLIP_LIMIT or _spinning(x)


def _slip_runs(s: list[Sample]) -> list[list[int]]:
    """後輪が滑っている状態が `_SPIN_PERSIST_S` 以上続いた区間（一瞬の揺れは数えない）。"""
    return [r for r in _phase_runs(s, _slipping) if s[r[-1]].t - s[r[0]].t >= _SPIN_PERSIST_S]


def _traction_event(s: list[Sample], idx: list[int]) -> bool:
    """区間に、加速中に後輪が滑った（`_slip_runs`）所があるか。"""
    ids = set(idx)
    return any(i in ids for r in _slip_runs(s) for i in r)


def _accel_level(x: Sample) -> float:
    """そのサンプルの段の指令の加速度（`accel_limit`。0＝全開は3.0とみなす）。"""
    return x.accel_limit if 0.0 < x.accel_limit < 3.0 else 3.0


def _slip_limited_accel(s: list[Sample]) -> tuple[float, float] | None:
    """加速中に後輪が滑っていた区間の車の加速度 [m/s²] と、それが出た段の指令の加速度。滑りながら
    （TC が効きながら）この車が実際に出せた加速度（グリップで決まる）。0.15s 以上の区間が無ければ None。

    段ごとに速度の傾きの中央値を取り、**いちばん大きい段**を返す。TC はスリップ率を目標（0.2）に
    保つので、限界より下の段でも滑り率は `REAR_SLIP_LIMIT` を超える。その段の加速度は指令どおり
    （限界より低い）で、全段をまとめた中央値だと限界を低く読む（2026-10-10、全段を走るようにした）。"""
    levels: dict[float, list[float]] = {}
    for run in _slip_runs(s):
        t = np.array([s[i].t for i in run])
        if len(run) >= 5 and t[-1] - t[0] >= 0.15:
            levels.setdefault(_accel_level(s[run[0]]), []).append(
                float(np.polyfit(t, [s[i].speed for i in run], 1)[0]))
    if not levels:
        return None
    a, lv = max((float(np.median(v)), k) for k, v in levels.items())
    return a, lv


def _slip_accel(s: list[Sample]) -> float | None:
    """後輪が滑った最初の区間の、指令の加速度（`_accel_level`）。無ければ None。"""
    runs = _slip_runs(s)
    return _accel_level(s[runs[0][0]]) if runs else None


def _top_accel_level(s: list[Sample]) -> float:
    """記録の中でいちばん上の段の指令の加速度（前進の加速の指令が出ているサンプルから）。"""
    return max((_accel_level(x) for x in s if not x.brake and x.target_speed > 0.0), default=0.0)


#: STM32 の最大制動トルク [N·m]（`brake_torque` 0 ＝未指定のとき。`DRIVE_MAX_BRAKE_TORQUE_NM`）
_MAX_BRAKE_TORQUE_NM = 0.13


def _brake_nm(x: Sample) -> float:
    """そのサンプルで実際に掛かる制動トルク [N·m]。未指定（0）は最大、最大より上の指令は STM32 が
    最大に丸める（前後運動試験の最後の段は 0.15 を指令する＝「最大」の意味）。丸めずに読むと、止まった
    後の保持（未指定）が別の強さ 0.13 として混ざり、0.15 の段の傾きも実際より小さく読む。"""
    return min(x.brake_torque, _MAX_BRAKE_TORQUE_NM) if x.brake_torque > 0 else _MAX_BRAKE_TORQUE_NM


def _fit_brake(levels: dict[float, float], rr: float, base: VehicleSpec) -> tuple[dict[str, float], list[str]]:
    """強さが1通りしか無い記録（2026-09-26 まで。GUI の制動トルクで録った）: その減速度を上限として
    返すだけ（傾きは決まらない）。強さが複数ある記録は `_fit_brake_runs`。"""
    nm, d = sorted(levels.items())[-1]
    notes = [f"制動の減速度 {d:.2f}m/s²（制動トルク {nm:.3f}N·m の1通り。強さによる違いは測れていない）"]
    torque_decel = nm * base.drive_ratio / (base.wheel_radius * base.mass)
    if nm < _MAX_BRAKE_TORQUE_NM - 1e-6 and d >= 0.9 * torque_decel:
        notes.append(f"制動は制動トルク {nm:.3f}N·m で頭打ちの可能性（トルクからの逆算"
                     f" {torque_decel:.2f}m/s²）。制動トルクを最大にして録ると上限が測れる")
    return {"brake_decel_m_s2": d}, notes


@dataclass
class _BrakeRun:
    """制動1回ぶん: 制動の指令が効き始めた時刻からの経過時間・前輪の走行距離・その時の速さ。"""

    nm: float
    t: np.ndarray
    x: np.ndarray
    v0: float
    #: 後輪が前輪よりはっきり遅い（ロック）サンプルがあったか（後輪の周速の記録がある場合）
    locked: bool
    #: ABS が制動を削ったサンプルがあったか（★v0.15）。ABS は後輪をロック手前に保つので、
    #: 減速度はグリップの上限で頭打ちになる——ロックと同じく頭打ちがある証拠
    abs_cut: bool = False


#: 後輪がロックしたとみなす、後輪の周速が前輪の6割未満の状態の継続時間 [s]。ABS（★v0.15）は
#: 滑り始めを検知してから制動を抜くので、1〜2サンプル（10〜20ms）だけ6割を切る。これをロックと
#: 数えると、ABS が効いている記録すべてに「ロックした」の★が出た（2026-09-27 の実機の記録）
_LOCK_PERSIST_S = 0.03


def _rear_locked(run: list[Sample]) -> bool:
    """制動1回ぶんのサンプルで、後輪の周速が前輪の6割未満の状態が `_LOCK_PERSIST_S` 以上続いたか
    （後輪の周速の記録がある場合。車速0.3m/s以下は見ない）。"""
    if not any(abs(x.wheel_speed_rear) > 1e-6 for x in run):
        return False
    since = None
    for x in run:
        if x.speed > 0.3 and x.wheel_speed_rear / x.speed < 0.6:
            since = x.t if since is None else since
            if x.t - since >= _LOCK_PERSIST_S:
                return True
        else:
            since = None
    return False


def _brake_runs(log: Log) -> list[_BrakeRun]:
    s = log.samples
    out = []
    cmd_t = np.array([c.t for c in log.cmds])
    for run in _phase_runs(s, lambda x: x.brake and x.speed > 0.02):
        if run[0] == 0:
            continue
        nm = _brake_nm(s[run[0]])
        # 制動の指令が出た時刻（/cmd）。効き始めるまでの遅れは `_fit_brake_runs` が一緒に推定する
        k = int(np.searchsorted(cmd_t, s[run[0] - 1].t, side="right"))
        while k < len(log.cmds) and not log.cmds[k].brake:
            k += 1
        if k >= len(log.cmds):
            continue
        t0 = log.cmds[k].t
        idx = [i for i in range(run[0] - 1, run[-1] + 1) if s[i].t >= t0] or run
        tt = np.array([s[i].t for i in idx]) - t0
        j = max(0, run[0] - 1)
        # 距離は t0 の時点に内挿した値から
        x0 = float(np.interp(t0, [s[j].t, s[j + 1].t], [s[j].odom, s[j + 1].odom]))
        v0 = float(np.interp(t0, [s[j].t, s[j + 1].t], [s[j].speed, s[j + 1].speed]))
        xx = np.array([s[i].odom for i in idx]) - x0
        locked = _rear_locked([s[i] for i in run])
        abs_cut = any(s[i].abs_active for i in run)
        out.append(_BrakeRun(nm, tt, xx, v0, locked, abs_cut))
    return out


def _sim_brake(spec: VehicleSpec, run: _BrakeRun, lag: float = 0.0, dt: float = 0.001) -> np.ndarray:
    """`sim/vehicle.py` の `next_speed`（制動）で `run` の制動を回した、各時刻の走行距離。
    `lag` [s] は指令から制動が効き始めるまで（それまでは速さを保つ）。"""
    di = DriveInput(armed=True, brake=True, brake_torque=run.nm)
    v, x, tt = run.v0, 0.0, 0.0
    out = np.empty(len(run.t))
    for k, te in enumerate(run.t):
        while tt < te - 1e-12:
            h = min(dt, te - tt)
            v2 = v if tt < lag else next_speed(spec, di, v, h)
            x += 0.5 * (v + v2) * h
            v, tt = v2, tt + h
        out[k] = x
    return out


def _fit_brake_runs(log: Log, base: VehicleSpec, rr: float) -> tuple[dict[str, float], list[str]]:
    """前後運動試験の制動（段ごとに強さを変える、2026-09-27〜）→ 傾き（`brake_decel_per_nm`）と
    後輪がロックしたときの頭打ち（`brake_decel_m_s2`、0＝頭打ちなし）。

    **制動中の走行距離（位置）**に `sim/vehicle.py` の制動のモデル（傾き×強さ×MD の境界層＋転がり
    抵抗、頭打ち）をそのまま当てはめる。速度の傾きで読むと、MD の境界層（約0.6m/sより遅いと
    弱まる）と止まる直前の微分の鈍りが混ざり、実機の記録で強さ→減速度が直線にも頭打ちにも
    ならなかった。位置なら微分しないので止まる直前も使える。頭打ちは、入れる・入れないを尤度比で
    比べる（ロックが見えない記録でも決まる）。"""
    runs = _brake_runs(log)
    spec0 = replace(base, rolling_resistance=rr)
    n = len(runs)

    def resid_with(g, cap, lag, dv):
        # 各回の初速も一緒に合わせる: 記録の速度（オドメトリを±45msで微分）は制動が始まる瞬間には
        # 窓が制動にかかって低めに出る（ベンチで頭打ちを 3.9→3.7 と読んだ）。指令から効き始める
        # までの遅れ `lag` は全回で共通
        sp = replace(spec0, brake_decel_per_nm=g, brake_decel_m_s2=cap)
        return np.concatenate([_sim_brake(sp, replace(r, v0=r.v0 + d), lag) - r.x for r, d in zip(runs, dv)])

    g0 = base.drive_ratio / (base.wheel_radius * base.mass)
    # 初速の読み違いは数cm/s（制動の減速度×微分の窓の半幅の数分の1）。広く許すと、ロックで
    # 減速が弱まった分まで初速で説明してしまった（実機の記録）
    lo_dv, hi_dv = [-0.08] * n, [0.08] * n
    r1 = least_squares(lambda p: resid_with(p[0], 0.0, p[1], p[2:]), [g0, 0.01] + [0.0] * n,
                       bounds=([1.0, 0.0] + lo_dv, [500.0, 0.08] + hi_dv), diff_step=1e-3)
    best = None
    # 頭打ちの初期値は低い方（1.0・1.75）からも、傾きは名目からも試す。頭打ちなしの当てはめ `r1` は、
    # 強さのほとんどが頭打ちしている記録では傾きを小さくして辻褄を合わせる（2026-10-10 の実機:
    # グリップが低く 0.08N·m 以上がすべて約1.7m/s² → 傾き 11、名目 33）。その傾きから始めると
    # 最大の強さでも減速度が 2.5 に届かず、頭打ちが効かないまま動かなかった（頭打ち 0 と読んだ）
    for g_start in (float(r1.x[0]), g0):
        for cap0 in (1.0, 1.75, 2.5, 4.0, 6.0):
            r2 = least_squares(lambda p: resid_with(p[0], p[1], p[2], p[3:]),
                               [g_start, cap0] + list(r1.x[1:]),
                               bounds=([1.0, 0.3, 0.0] + lo_dv, [500.0, 20.0, 0.08] + hi_dv), diff_step=1e-3)
            if best is None or r2.cost < best.cost:
                best = r2
    e1, e2 = float(np.mean(r1.fun ** 2)), float(np.mean(best.fun ** 2))
    g, cap = float(best.x[0]), float(best.x[1])
    peak = max(g * r.nm * brake_speed_factor(spec0, r.v0 + d) + rr for r, d in zip(runs, best.x[3:]))
    locked = sorted({round(r.nm, 3) for r in runs if r.locked})
    abs_cut = sorted({round(r.nm, 3) for r in runs if r.abs_cut})
    # 後輪のロック・ABS の介入が記録に見えていれば頭打ちあり（見えない記録＝後輪の周速が無い・
    # ロックしない強さだけ、では尤度比で決める）
    use_cap = cap < 0.97 * peak and (bool(locked) or bool(abs_cut)
                                     or _lr_significant(n_eff(best.fun), e2, e1))
    lag = float(best.x[2])
    if not use_cap:
        g, cap, e, lag = float(r1.x[0]), 0.0, e1, float(r1.x[1])
    else:
        e = e2
    levels = sorted({round(r.nm, 3) for r in runs})
    notes = [f"制動: 傾き {g:.1f}(m/s²)/(N·m)（名目 {g0:.1f}、×MD の境界層 tanh(車速÷"
             f"{base.brake_boundary_speed_m_s:.2f}m/s)）"
             + (f"、グリップで {cap:.2f}m/s² に頭打ち（{'ABS' if abs_cut else '後輪のロック'}）"
                if use_cap else "、最大の強さまで頭打ちなし")
             + f"。強さ {'・'.join(f'{x:.2f}' for x in levels)}N·m、指令から効き始めるまで {lag * 1000:.0f}ms、"
             f"走行距離の残差RMS {math.sqrt(e) * 1000:.1f}mm"]
    if abs_cut:
        notes.append(f"制動トルク {'・'.join(f'{x:.2f}' for x in abs_cut)}N·m で ABS が制動を削った"
                     "（頭打ちはロックではなく ABS が保ったグリップの上限）")
    if locked:
        notes.append(f"★制動トルク {'・'.join(f'{x:.2f}' for x in locked)}N·m で後輪がロックした（後輪の周速が前輪の"
                     "6割未満）。ABS が無い・切っている・フォールバックした記録なら、最大制動"
                     "（フェイルセーフ・E-Stop）でもロックする")
    return {"brake_decel_per_nm": g, "brake_decel_m_s2": cap}, notes


def _spin_accel(s: list[Sample]) -> float | None:
    """加速中（制動なし・前進の指令）に後輪が前輪より大きく速い状態（TC で抑えきれない空転）が
    続いた最初の区間の、指令の加速度（`accel_limit`。0＝全開は3.0とみなす）。無ければ None。"""
    since = None
    for x in s:
        on = not x.brake and x.target_speed > 0.0 and _spinning(x)
        if not on:
            since = None
            continue
        since = x.t if since is None else since
        if x.t - since >= _SPIN_PERSIST_S:
            return x.accel_limit if 0.0 < x.accel_limit < 3.0 else 3.0
    return None


def fit_accel(log: Log, base: VehicleSpec | None = None, speed_log: Log | None = None) -> FitResult:
    """前後運動試験 → 車体の利得・転がり抵抗（`speed_plant_gain`・`rolling_resistance`）・駆動加速度の
    上限とその速度依存（`drive_accel_m_s2`・`drive_fade_speed_m_s`・`drive_top_speed_m_s`）・
    速度指令での減速の上限（`speed_decel_m_s2`）・制動の減速度。

    2026-09-27 から前後運動試験（`sysid_accel`）は加速度を段階的に上げる1つの試験で、速度応答
    試験（`sysid_speed`）は無くなった。車体の利得・転がり抵抗もこの記録だけで当てはめる。

    全開加速（目標 v_max+1.0）でも、ファームは目標を `speed_ramp_max_m_s2`（3.0m/s²）で
    ランプさせるので、車の加速度はまずランプで決まる。**モータ・TC の上限がランプより低い
    ときだけ**その上限が見える。そこで `SpeedController`（`base` の利得・転がり抵抗）に記録の
    指令を入れ、上限を入れる・入れないを尤度比で比べる（物理上限の舵と同じ考え方）。
    観測できなかった上限は 0（＝ランプとPIの出力上限だけ）を返す。

    `brake` の減速度は速度制御を通らない（ファームは PI を迂回して制動トルクを掛ける）ので、
    制動区間の速度の傾きから読む。

    `speed_log`（2026-09-26 までの速度応答試験の記録）を渡すと、両方の記録を合わせて当てはめる
    （加減速の上限と利得は互いに補い合うので、別々に決めると片方の誤差がもう片方に移った）。
    """
    base = base or VehicleSpec.load()
    _require_pi(base)
    s = log.samples
    notes: list[str] = []
    values: dict[str, float] = {}

    # 制動: 制動トルクごとに減速度（速度の傾き）をまとめる。当てはめは車体の利得・転がり抵抗が
    # 決まってから（下の `_fit_brake`）
    brake_levels: dict[float, list[float]] = {}
    for run in _phase_runs(s, lambda x: x.brake and x.speed > 0.1):
        nm = _brake_nm(s[run[0]])
        brake_levels.setdefault(round(nm, 4), []).extend(
            -a for vm, a, _ in _window_slopes(s, run) if vm > 0.15)
    brake_levels = {nm: float(np.median(d)) for nm, d in brake_levels.items() if len(d) >= 1}
    if not brake_levels:
        notes.append("ブレーキの区間が無い（奇数回目のサイクルが走っていない）")

    spin = _spin_accel(s)
    slip_a = _slip_accel(s)
    v_top = max((x.speed for x in s), default=0.0)
    if v_top < 1.0:
        raise ValueError("全開加速の区間が短すぎます（1.0m/s に届いていない。run_length_m を確認）")
    delay0 = (0.01, 0.0, 0.15)
    # 全部入り（加速の上限とその速度依存・減速の上限）で当てはめてから、1つずつ外して
    # 有意に悪化しないものは外す（後退選択）。1つずつ足す順だと、入っていない方の上限の
    # 食い違いを抱えたまま比べることになり、減速の上限2.5m/s²を見逃した（ベンチ）
    full = {"drive_accel_m_s2": (2.5, 0.3, _NO_CAP),
            "drive_fade_speed_m_s": (max(0.3, 0.6 * v_top), 0.0, 10.0),
            "drive_top_speed_m_s": (max(2.0, 2.0 * v_top), 0.5, 20.0),
            "speed_decel_m_s2": (2.5, 0.3, _NO_CAP), "delay": delay0}
    logs = [log] + ([speed_log] if speed_log is not None else [])
    full["speed_plant_gain"] = (base.speed_plant_gain if base.speed_plant_gain > 0
                                else 1.0 / (base.wheel_radius * base.mass), 1.0, 500.0)
    full["rolling_resistance"] = (min(max(base.rolling_resistance, 0.0), 2.0), 0.0, 3.0)
    fixed: dict[str, float] = {}
    pb, eb, rb = _fit_speed_model(logs, base, full, fixed,
                                  starts=[{}, {"drive_accel_m_s2": 4.0, "speed_decel_m_s2": 4.0}])
    use = {"fade": True, "dn": True, "up": True}
    drops = (("fade", ("drive_fade_speed_m_s", "drive_top_speed_m_s")),
             ("dn", ("speed_decel_m_s2",)),
             ("up", ("drive_accel_m_s2", "drive_fade_speed_m_s", "drive_top_speed_m_s")))
    for name, keys in drops:
        if not use[name]:
            continue
        free2 = {k: (pb[k], *full[k][1:]) if k in pb else v for k, v in full.items()
                 if k not in keys and k not in fixed}
        fixed2 = {**fixed, **{k: 0.0 for k in keys}}
        p2, e2, r2 = _fit_speed_model(logs, base, free2, fixed2)
        if not _lr_significant(n_eff(rb), eb, e2):
            use[name] = False
            if name == "up":
                use["fade"] = False
            fixed, pb, eb, rb = fixed2, p2, e2, r2
    # 上限に張り付いた＝効いていない
    if pb.get("drive_accel_m_s2", 0.0) >= 0.95 * _NO_CAP:
        use["up"] = use["fade"] = False
    if pb.get("speed_decel_m_s2", 0.0) >= 0.95 * _NO_CAP:
        use["dn"] = False
    use_up, use_dn = use["up"], use["dn"]
    # グリップで決まる加速の上限（TC の介入・空転）。速度制御の当てはめからはその区間を外してあるので
    # ここで決め、利得・転がり抵抗をその上限のもとで当てはめ直す（外した区間の手前でもグリップで
    # 頭打ちになっていることがあり、上限なしのまま当てはめると利得が 22→19.9 と歪んだ）
    traction_notes: list[str] = []
    a_grip = None
    if spin is not None:
        a_grip = spin
        traction_notes.append(
            f"★加速中に TC で抑えきれない後輪の空転を検出（段の加速度 {spin:.1f}m/s²。プランナーはそこで"
            "止めて上の段を走らない）。空転の直後に制動するのでデータが短く、実際の上限は空転しなかった段の"
            f"加速度から {spin:.1f}m/s² の間——加速の上限は上から抑えて {spin:.1f}m/s² にした。シムは空転を"
            "表現しない。TC の定数（DRIVE_TC_*）も確かめる")
    elif slip_a is not None:
        sl = _slip_limited_accel(s)
        a_sl, lv = sl if sl is not None else (None, 0.0)
        a_grip = a_sl if a_sl is not None and a_sl > 0.1 else None
        traction_notes.append(
            f"加速中に後輪が滑った（{slip_a:.1f}m/s² の段から）。"
            + (f"滑っていた間の加速度は{'全開' if lv >= 3.0 else f' {lv:.1f}m/s² '}の段の {a_sl:.2f}m/s² が最大で、"
               "これを加速の上限にした（グリップで決まる）。"
               if a_grip is not None else "滑っていた時間が短く、その加速度は読めなかった。")
            + "滑った区間は速度制御（車体の利得・転がり抵抗）の当てはめから外した")
        if a_grip is not None and lv < 3.0 and lv >= _top_accel_level(s) and a_grip >= 0.9 * lv:
            # 段の指令より上の加速度は出ないので、指令どおりに加速できていれば限界はもっと上かもしれない
            traction_notes.append(
                f"★この加速度は段の指令（{lv:.1f}m/s²）とほぼ同じで、それより上の段を走っていない。"
                "グリップの限界ではなく指令どおりに加速しただけかもしれない（限界はこれ以上、としか"
                "言えない）。全段を走った記録で測り直すこと（2026-10-10 より前のプランナーは、滑った段で"
                "打ち切っていた）")
    if a_grip is not None and (not use_up or a_grip < pb["drive_accel_m_s2"]):
        use_up = use["up"] = True
        fixed3 = {**fixed, "drive_accel_m_s2": a_grip}
        if not use["fade"]:
            fixed3.update(drive_fade_speed_m_s=0.0, drive_top_speed_m_s=0.0)
        free3 = {k: (pb[k], *full[k][1:]) if k in pb else v for k, v in full.items() if k not in fixed3}
        pb, eb, rb = _fit_speed_model(logs, base, free3, fixed3)
    if use_up:
        if use["fade"] and pb["drive_top_speed_m_s"] > pb["drive_fade_speed_m_s"] + 0.05 \
                and pb["drive_fade_speed_m_s"] < v_top:
            notes.append(f"{pb['drive_fade_speed_m_s']:.2f}m/s を超えると加速度の上限が落ち、"
                         f"{pb['drive_top_speed_m_s']:.2f}m/s で0になる（{v_top:.2f}m/s まで実測、"
                         "それより上は外挿）")
        else:
            use["fade"] = False
            notes.append("速度による加速度の上限の低下は見られない（測った範囲で一定）")
        if a_grip is not None and pb["drive_accel_m_s2"] == a_grip:
            pass                                 # 所見は下（TC・空転）
        elif pb["drive_accel_m_s2"] < base.speed_ramp_max_m_s2:
            notes.append(f"駆動加速度の上限 {pb['drive_accel_m_s2']:.2f}m/s²（ファームの目標ランプ "
                         f"{base.speed_ramp_max_m_s2:.1f}m/s² より低いので見えた）")
        else:
            notes.append(f"駆動加速度の上限 {pb['drive_accel_m_s2']:.2f}m/s²（ファームの目標ランプより高く、"
                         "PIが目標に追いつく瞬間だけ効く。値の確かさは低いが挙動への影響も小さい）")
    else:
        notes.append(f"駆動加速度の上限は観測できず（ファームの目標ランプ {base.speed_ramp_max_m_s2:.1f}m/s²"
                     " とPIの出力上限で加速が決まっていた）→ 0（それ以外の上限なし）")
    if use_dn:
        notes.append(f"速度指令0での減速の上限 {pb['speed_decel_m_s2']:.2f}m/s²")
    else:
        notes.append("速度指令0での減速の上限は観測できず（ランプとPIで決まっていた）→ 0")
    floors = [_speed_floor(lg.samples) for lg in logs]
    floor = float(np.median([f for f, _ in floors]))
    rms = math.sqrt(eb)
    notes.append(f"残差RMS {rms * 100:.1f}cm/s（{floors[0][1]}）")
    warnings: list[str] = []
    if floor > 0 and rms > _RESID_WARN_RATIO * floor + 0.01:
        notes.append(f"★残差が速度のノイズの {rms / floor:.1f} 倍。加減速がシムの速度制御の形から"
                     "外れている（ファームの速度PIの定数が vehicle.toml と合っているか、TCの介入が無いか確かめる）")
        warnings = [notes[-1]]
    notes += traction_notes
    warnings = warnings + [n for n in traction_notes if n.startswith("★")]
    if len(brake_levels) >= 2:
        vb, nb = _fit_brake_runs(log, base, float(pb["rolling_resistance"]))
        values.update(vb)
        notes += nb
    elif brake_levels:
        vb, nb = _fit_brake(brake_levels, float(pb["rolling_resistance"]), base)
        values.update(vb)
        notes += nb
    values["drive_accel_m_s2"] = float(pb["drive_accel_m_s2"]) if use_up else 0.0
    values["drive_fade_speed_m_s"] = float(pb["drive_fade_speed_m_s"]) if use["fade"] else 0.0
    values["drive_top_speed_m_s"] = float(pb["drive_top_speed_m_s"]) if use["fade"] else 0.0
    values["speed_decel_m_s2"] = float(pb["speed_decel_m_s2"]) if use_dn else 0.0
    if speed_log is not None and warnings:
        # 加減速がモデルから外れた記録（例: 空転で加速が頭打ち）と合わせると、外れた分を利得と
        # 転がり抵抗が吸収する（第三者検証で 21.4→17.8・0.28→0.14、真値22・0.30）。
        # 速度応答試験だけで決めた値を残す（返さなければ analyze が上書きしない）
        notes.append(f"★のため、合わせて当てはめ直した車体の利得 {pb['speed_plant_gain']:.1f}・転がり抵抗 "
                     f"{pb['rolling_resistance']:.2f}m/s² は採らない（速度応答試験の値のまま）")
    else:
        # 1つの記録だけのとき（今の前後運動試験）は★でも返す（他に材料が無い）。GUI は★の出た
        # 試験の項目を既定で適用しない（`analyze.Analysis.warned`）
        values["speed_plant_gain"] = float(pb["speed_plant_gain"])
        values["rolling_resistance"] = float(pb["rolling_resistance"])
        notes.append(f"車体の利得 {pb['speed_plant_gain']:.1f}(m/s²)/(N·m)（名目 "
                     f"{1.0 / (base.wheel_radius * base.mass):.1f}）・転がり抵抗 "
                     f"{pb['rolling_resistance']:.2f}m/s²・/cmd→STM32 の遅れ {pb['delay'] * 1000:.0f}ms"
                     + ("（速度応答試験の記録と合わせて）" if speed_log is not None else ""))
    return FitResult(values, notes, warnings)


# ── ③ 旋回グリップ ───────────────────────────────────────────────────────

@dataclass
class _Stage:
    v: float          # 速度の中央値 [m/s]
    a_lat: float      # 横加速度の中央値 [m/s²]（符号なし）
    delta: float      # 実舵角の中央値 [rad]（符号つき）
    kappa: float      # 曲率 [1/m]（符号つき）
    target_speed: float
    #: この区間のサンプルの添字
    idx: list[int] = field(default_factory=list)
    #: 後輪の滑り率の中央値（`_sysid_common.rear_slip`。後輪の周速の記録が無ければ 0）
    slip: float = 0.0


#: 旋回試験の走り出し（速度が目標に追いつくまで）を捨てる長さ [s]（プランナーの `warmup_s` と同じ）
_CORNER_WARMUP_S = 1.5


def _corner_stages(samples: list[Sample], bias: float) -> list[_Stage]:
    """旋回している区間（舵を切って前進の指令）を `CORNER_WINDOW_S` ごとの区間にまとめる。

    プランナー（`sysid_corner`）が打ち切りの判断に使うのと**同じ区間の切り方**
    （`_sysid_common.corner_windows`）。2026-09-27 に速度を段ではなく連続して上げる形にしたので、
    段ごとの後半ではなく一定の長さの区間で見る。"""
    out: list[_Stage] = []
    for run in _phase_runs(samples, lambda x: abs(x.target_steer) > 0.05 and x.target_speed > 0.05):
        t_run0 = samples[run[0]].t
        run = [i for i in run if samples[i].t - t_run0 >= _CORNER_WARMUP_S
               and abs(samples[i].yaw_rate) < _GYRO_SAT and samples[i].speed > 0.05]
        if len(run) < 5:
            continue
        t = [samples[i].t for i in run]
        v = [samples[i].speed for i in run]
        w = [samples[i].yaw_rate - bias for i in run]
        a = [abs(vi * wi) for vi, wi in zip(v, w)]
        # 後輪の滑り率（プランナーと同じく STM32 の speed と比べる。後輪の周速の記録が無ければ 0）
        has_rear = any(abs(samples[i].wheel_speed_rear) > 1e-6 for i in run)
        sl = [rear_slip(samples[i].speed_filtered, samples[i].wheel_speed_rear) if has_rear else 0.0
              for i in run]
        for (vm, am, i0, i1), (_, slm, _, _) in zip(corner_windows(t, v, a), corner_windows(t, v, sl)):
            idx = run[i0:i1]
            aa = float(np.median([v[k] * w[k] for k in range(i0, i1)]))
            d = float(np.median([samples[i].steer_actual for i in idx]))
            out.append(_Stage(vm, am, d, aa / (vm * vm), float(np.median(
                [samples[i].target_speed for i in idx])), idx, slm))
    return out


#: 旋回試験でここまで走って頭打ちが無ければ、mu を下限として返す（学習の最高速度＋余裕）
_CORNER_ENOUGH_M_S = 2.2


def _corner_split(log: Log) -> tuple[list[_Stage], int | None, list[str], str]:
    """旋回の区間と、限界に達した区間の番号（無ければ None）と所見と限界の理由。限界は横加速度が
    頭打ち・後輪が流れた（曲率が跳ねた）の早い方（`_sysid_common.corner_limit`、プランナーと同じ判定）。"""
    s = log.samples
    bias, n_still = _gyro_bias(s)
    notes = []
    if n_still < 20:
        notes.append("停止区間が短く、ジャイロのバイアスを補正していない")
    stages = _corner_stages(s, bias)
    first, why = corner_limit([(st.v, st.a_lat) for st in stages])
    return stages, first, notes, why


def fit_corner(log: Log, base: VehicleSpec | None = None) -> FitResult:
    """旋回グリップ試験 → `mu`。

    旋回の区間（`_corner_stages`）を、`corner_limit()`（プランナーが打ち切りに使うのと同じ判定）で
    限界に達した区間と、その手前の余裕のある区間に分け、限界までに出た最大の横加速度 / g。余裕の
    ある区間は舵の効き・アンダーステア勾配の材料として `fit_geometry()` が使う。

    限界は車体が滑ったこと——**横加速度が頭打ち**か**後輪が流れた**（曲率が跳ねた）の早い方。後輪の
    空転（滑り率）では区切らない（2026-10-08、内輪の空転だけで限界と読んで `mu` が低く出た）。シムは
    横の限界を前輪側の頭打ち（`mu`）でしか表現しないので、後輪が先に流れるときは所見で知らせる
    （限界付近の挙動は再現されない）。
    """
    stages, first_sat, notes, why = _corner_split(log)
    n_sat = sum(1 for x in log.samples if abs(x.yaw_rate) >= _GYRO_SAT)
    if n_sat:
        notes.append(f"★ジャイロが測定範囲（±{math.degrees(GYRO_RANGE_RAD_S):.0f}°/s）に張り付いた"
                     f"サンプルが {n_sat} 個（横加速度の材料から外した）。滑り出す前に張り付くと、"
                     "限界の判定が遅れる")
    if len(stages) < 3:
        raise ValueError("旋回の区間が3つ未満です（記録が短すぎる）")
    if first_sat is None:
        v_top = max(st.v for st in stages)
        if v_top < _CORNER_ENOUGH_M_S:
            raise ValueError(
                "グリップの頭打ちに達していないようです（速度を上げても横加速度が伸び続けている）。"
                f"{v_top:.2f}m/s で止まっている——周囲のクリアランスで止まっていないか確認し、"
                "実際に滑り出すところまで録り直してください")
        # 学習の速度域の上まで走って頭打ちが無い＝その範囲ではグリップは効かない。
        # 観測した最大の横加速度を mu の下限として返す（シムはその範囲で頭打ちにならない）
        a_max = max(st.a_lat for st in stages)
        notes.append(f"★{v_top:.2f}m/s（舵30°）まで滑り出さなかった。mu は観測した最大の横加速度 "
                     f"{a_max:.2f}m/s² からの下限（実際はこれ以上）。学習の速度域（〜2.0m/s）では"
                     "グリップの頭打ちは起きないので、シムの挙動には影響しない")
        return FitResult({"mu": a_max / GRAVITY_MPS2}, notes)
    # 限界までに出た最大の横加速度（区間の中央値なので一瞬の揺れには引っ張られない）。プランナーは
    # 限界に達したらすぐ止めるので、限界の後の区間はほとんど無い
    upto = stages[:first_sat + 1]
    a_sat = max(st.a_lat for st in upto)
    lim = stages[first_sat]
    notes.append(f"限界 {a_sat:.2f}m/s²（{why}、{lim.v:.2f}m/s・後輪の滑り率 {lim.slip:.2f}）")
    early = [st for st in stages[:max(1, first_sat - 3)]]
    k_ref = float(np.median([abs(st.kappa) for st in early])) if early else abs(lim.kappa)
    if why == "後輪が流れた" or any(abs(st.kappa) > 1.08 * k_ref for st in stages[first_sat - 1:first_sat + 1]):
        notes.append("限界では後輪が先に流れた（後輪駆動のオーバーステア寄り）。シムは横の限界を前輪側の"
                     "頭打ち（mu）で表すので、mu は限界の横加速度として合うが、限界を超えたときの挙動"
                     "（後輪が流れる）は再現されない")
    return FitResult({"mu": a_sat / GRAVITY_MPS2}, notes)


# ── 舵の効き・中立ずれ・アンダーステア・リンクのガタ（3試験をまとめて） ────────

#: 曲率の観測に使う最低速度 [m/s]（ヨーレートのノイズ÷速度が曲率のノイズになる）
_GEO_MIN_V = 0.2


@dataclass
class _GeoSeries:
    """1つの記録の、曲率の観測に使うサンプル。リンクのガタとヨーの遅れは履歴を持つので、
    記録の頭からの全系列を持っておき、観測点の値だけを比べる。"""

    source: str
    t: np.ndarray           # 記録全体の時刻
    steer: np.ndarray       # 記録全体の steer_actual（平滑化済み）
    v_full: np.ndarray      # 記録全体の車速（オドメトリ由来）
    idx: np.ndarray         # 観測に使うサンプルの添字
    v: np.ndarray           # idx での速度
    kappa: np.ndarray       # idx での曲率（ジャイロのバイアスを除いた yaw/speed）
    #: 観測点の、元の記録でのサンプル番号（`idx` は間引いた系列の中の位置）
    src: np.ndarray | None = None
    #: 舵角の平滑化に使った点数（間引く前の刻みで。ノイズの見積もり用。1＝平滑化なし）
    n_smooth: int = 1
    #: 遅れの重みの添字の表（`_lag_w1`・`_lag_w2` が最初に使うときに作る）
    j1: np.ndarray | None = None
    j2: np.ndarray | None = None


#: `fit_geometry` に渡す系列の刻み [s]。100Hz の記録は 50Hz 相当に間引く（2026-09-26）。舵の効き・
#: ガタ・ヨーの遅れは0.1s以上の現象で、50Hz で精度を確かめてある。間引かないと当てはめの残差の
#: 計算（全サンプル×遅れの重み）が2万回以上回り、解析1回が約2.5分かかった。舵角の平滑化と
#: オドメトリの速度は間引く前の全サンプルで作ってから間引く
_GEO_DT = 0.02


def _series(log: Log, source: str, keep, min_v: float = _GEO_MIN_V,
            smooth: bool = True) -> _GeoSeries | None:
    s = log.samples
    bias, _ = _gyro_bias(s)
    t_full = _arr(s, "t")
    steer_full = _arr(s, "steer_actual")
    if smooth:
        steer_full = _smooth(steer_full, t_full, _steer_noise(s))
    stride = max(1, int(round(_GEO_DT / _median_dt(t_full))))
    sel = np.arange(0, len(s), stride)
    # 観測点は間引いた系列の中の位置で持つ
    pos = np.array([p for p, i in enumerate(sel) if keep(i) and s[i].speed > min_v
                    and abs(s[i].yaw_rate) < _GYRO_SAT], dtype=int)
    if len(pos) < 10:
        return None
    v = np.array([s[sel[p]].speed for p in pos])
    w = np.array([s[sel[p]].yaw_rate for p in pos]) - bias
    return _GeoSeries(source, t_full[sel], steer_full[sel], _arr(s, "speed")[sel], pos, v, w / v,
                      src=sel[pos], n_smooth=_smooth_n(t_full) if smooth else 1)


#: 1次遅れの重みを持つ過去の長さ [s]（サンプル数は記録の刻みから決める）
_LAG_S = 0.6


def _lag_index(t: np.ndarray, at: np.ndarray) -> np.ndarray:
    """`_lpf_weights` の添字の表（遅れの値によらず、記録と観測点だけで決まる。int32）。"""
    K = max(10, int(math.ceil(_LAG_S / _median_dt(t))))
    return np.clip(at[:, None] - np.arange(K)[None, :], 0, len(t) - 1).astype(np.int32)


def _lpf_weights(t: np.ndarray, at: np.ndarray, tau: np.ndarray,
                 Jc: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray]:
    """時定数がサンプルごとに変わる1次遅れの、添字 `at` での出力を「過去 `_LAG_S` 秒ぶんのサンプルの
    重み付き和」にした `(W, J)`。

    入力はサンプル間を**直線でつなぐ**（1次ホールドの厳密な離散化）。サンプル値がその直前の
    区間ずっと続いたとみなす（0次ホールド）と、連続に変化する実際の信号より半サンプル（10ms）
    早く応答し、その分を当てはめが余計な遅れで埋めた（IMU 10ms を 25ms と読んだ）。
        y_i = e·y_{i-1} + (1-r)·x_i + (r-e)·x_{i-1}、 e = exp(-dt/τ)、 r = (1-e)·τ/dt
    重みを先に作っておけば、当てはめの各評価は `(W * x[J]).sum(1)` だけで済む。

    重みは単精度で返す（和が1に正規化された重みなので精度は足りる）。添字の表 `Jc` は遅れの
    値によらないので、呼び出し側が1度作って渡せる（`_lag_index`）。100Hz の記録では1組が数MB
    になり、遅れの格子探索で溜め込むと1回の解析で8GBを超えた（2026-09-26）。
    """
    n = len(t)
    dt = np.diff(t, prepend=t[0] - 0.02).clip(1e-3, 0.1)
    with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
        e = np.where(tau > 1e-6, np.exp(-dt / np.maximum(tau, 1e-9)), 0.0)
        r = np.where(tau > 1e-6, (1.0 - e) * tau / dt, 0.0)
    r = np.where(np.isfinite(r), r, 1.0)          # τ→∞（止まっている間の緩和長）: 出力は保持
    b0, b1 = 1.0 - r, r - e
    if Jc is None:
        Jc = _lag_index(t, at)
    K = Jc.shape[1]
    valid = (at[:, None] - np.arange(K)[None, :]) >= 0
    E = e[Jc]                                      # E[:, k] = e_{i-k}
    PA = np.concatenate([np.ones((len(at), 1)), np.cumprod(E, axis=1)[:, :-1]], axis=1)  # Π_{m<k} e_{i-m}
    W = b0[Jc] * PA
    # x_{i-k} は b1_{i-k+1} の項からも入る（k≥1）
    W[:, 1:] += b1[Jc[:, :-1]] * PA[:, :-1]
    W = np.where(valid, W, 0.0)
    W /= np.where(np.abs(W.sum(axis=1, keepdims=True)) > 1e-12, W.sum(axis=1, keepdims=True), 1.0)
    return W.astype(np.float32), Jc


def _lag_weights(g: _GeoSeries, tau_s: float, relax_m: float):
    """ヨーの遅れの2段: ①曲率にタイヤの緩和長（走行距離での1次遅れ、`sim/vehicle.py` と同じく
    速度を掛ける**前**）→ ②ヨーレートに IMU のローパス（時間での1次遅れ）。

    緩和長を「ヨーレートに掛かる時間の遅れ」として1段にまとめると、低速部のように速度の
    符号が0.5sごとに反転する区間でシムと大きく食い違い、緩和長を IMU の遅れと読み違えた。
    返り値は `(W1, J1, W2, J2)`（①は全サンプル、②は観測点）。"""
    return _lag_w1(g, relax_m) + _lag_w2(g, tau_s)


def _lag_w1(g: _GeoSeries, relax_m: float) -> tuple[np.ndarray, np.ndarray]:
    """①タイヤの緩和長の重み（全サンプル）。緩和長だけで決まる。"""
    n = len(g.t)
    if g.j1 is None:
        g.j1 = _lag_index(g.t, np.arange(n))
    with np.errstate(divide="ignore"):
        tau1 = relax_m / np.abs(g.v_full) if relax_m > 1e-6 else np.zeros(n)
    tau1 = np.where(np.isfinite(tau1), tau1, 1e9)
    return _lpf_weights(g.t, np.arange(n), tau1, g.j1)


def _lag_w2(g: _GeoSeries, tau_s: float) -> tuple[np.ndarray, np.ndarray]:
    """②IMU のローパスの重み（観測点だけ）。IMU の遅れだけで決まる。"""
    if g.j2 is None:
        g.j2 = _lag_index(g.t, g.idx)
    return _lpf_weights(g.t, g.idx, np.full(len(g.t), tau_s), g.j2)


class _LRU:
    """最近使ったものだけを `maxsize` 個まで持つ入れ物（`fit_geometry` の途中結果用）。"""

    def __init__(self, maxsize: int) -> None:
        from collections import OrderedDict
        self._d: OrderedDict = OrderedDict()
        self.maxsize = maxsize

    def get(self, key, make):
        if key in self._d:
            self._d.move_to_end(key)
            return self._d[key]
        v = make()
        self._d[key] = v
        if len(self._d) > self.maxsize:
            self._d.popitem(last=False)
        return v


def _smooth_n(t: np.ndarray) -> int:
    """`_smooth` の窓の点数（奇数、約0.1s。50Hzで5点・100Hzで11点）。"""
    return 2 * max(1, int(round(0.05 / _median_dt(t)))) + 1


def _smooth(x: np.ndarray, t: np.ndarray, noise: float = 0.0) -> np.ndarray:
    """中心の移動平均（約0.1s、`_smooth_n`。傾きが一定の区間では遅れない）。

    リンクのガタのモデルに**測った舵角のノイズをそのまま通すと、ガタ（遊び）がノイズを
    吸収して残差が縮むので、無いはずのガタが検出される**（ベンチでノイズ3倍にすると
    ガタ0の真値に対して0.4〜0.8°が出た）。実際の舵角はサーボで滑らかに動くので、先に
    ノイズを落としてから通す。

    ただし**舵のステップの角（折れ曲がり）では均さない**（窓の半幅で取った2階差分から見た
    傾きの変化が `_SMOOTH_KINK_SLOPE` を超える所は生の値）。移動平均は傾きが一定の所では遅れないが、角を丸めて前後0.05sに
    ずらす。振動的なヨー応答（曲率の2次遅れ）があると、その丸めた角が振動として続き、
    3次の項が偏った（2026-09-26）。

    角のしきい値は、舵角のノイズ `noise`（静止中の標準偏差、`_steer_noise`）の2階差分の
    `_SMOOTH_KINK_NOISE_K` 倍を下回らない（2026-09-27）。100Hz ではしきい値（2階差分1.25°）が
    ノイズ3倍（0.26°）の2階差分のばらつき（約0.63°）の2倍しかなく、サンプルの約4割がノイズで
    「角」とみなされて均されず、ガタを 0.8°→1.6° と読んだ。本物のステップの角（7rad/s の
    ランプの折れ目で約20°）は桁違いに大きいので、しきい値を上げても角は見落とさない。"""
    n = _smooth_n(t)
    if len(x) < n:
        return x
    k = np.ones(n) / n
    y = np.convolve(x, k, mode="same")
    h = n // 2
    y[:h] = x[:h]
    y[-h:] = x[-h:]
    kink = np.zeros(len(x), dtype=bool)
    d2 = np.abs(x[2 * h:] - 2 * x[h:-h] + x[:-2 * h])
    kink[h:-h] = d2 > max(_SMOOTH_KINK_SLOPE * h * _median_dt(t), _SMOOTH_KINK_NOISE_K * math.sqrt(6.0) * noise)
    # 角の前後 h サンプル（移動平均の窓が角にかかる範囲）も生の値にする
    kink = np.convolve(kink.astype(float), np.ones(n), mode="same") > 0
    return np.where(kink, x, y)


#: `_smooth` が「角」とみなす傾きの変化 [rad/s]（2階差分 ÷ 窓の半幅の時間）。50Hz・半幅2点で
#: 2階差分1.0°に当たる。舵のステップ（7rad/sのランプ）の角は桁違いに大きい。掃引（2026-09-26 まで）の折り返し
#: （0.2rad/s→保持）は均す側に残す（生の値だとノイズがガタに吸われた）
_SMOOTH_KINK_SLOPE = math.radians(1.0) / 0.04
#: `_smooth` の角のしきい値の下限（舵角のノイズの2階差分の標準偏差 √6·σ の何倍か）
_SMOOTH_KINK_NOISE_K = 5.0


def _steady_tail_mask(s: list[Sample], pred, min_s: float) -> np.ndarray:
    """`pred`が真で指令（速度・舵）が `min_s` 以上変わらない区間の、後半のサンプル。"""
    keep = np.zeros(len(s), dtype=bool)
    for run in _phase_runs(s, pred):
        pieces, cur = [], [run[0]]
        for i in run[1:]:
            if abs(s[i].target_speed - s[cur[-1]].target_speed) > 1e-4 or \
                    abs(s[i].target_steer - s[cur[-1]].target_steer) > 1e-4:
                pieces.append(cur)
                cur = []
            cur.append(i)
        pieces.append(cur)
        for piece in pieces:
            if s[piece[-1]].t - s[piece[0]].t >= min_s:
                keep[piece[len(piece) // 2:]] = True
    return keep


def _geo_straight(log: Log) -> _GeoSeries | None:
    """前後運動試験（2026-09-26 までは速度応答試験）の定速区間（舵0°・前進）→ 中立ずれの材料。

    指令が0.4s以上変わらない区間の後半だけ（加速中は左右モータのトルク差でもヨーが出る）。
    後退はタイヤのセルフアライニングの向きが逆でリンクのガタの寄り方が変わりうるので使わない。"""
    s = log.samples
    keep = _steady_tail_mask(s, lambda x: not x.brake and abs(x.target_steer) < 1e-6
                             and x.target_speed > 0.2, 0.4)
    return _series(log, "直線", lambda i: keep[i])


def _geo_steer(log: Log) -> list[_GeoSeries]:
    """ステア試験 → 走行中の小舵角（ジグザグ）と、低速の全域（段を保持する階段・大振幅ステップ。
    2026-09-26 までの記録は掃引）。

    舵が速く動いているサンプル（|dδ/dt|≥0.5rad/s）は除く（車の向きの追従の遅れが混ざる）。
    前進のサンプルだけ。"""
    s = log.samples
    t = _arr(s, "t")
    dd = np.abs(np.gradient(_arr(s, "steer_actual"), t))
    out = []
    zig = _series(log, "走行中の小舵角", lambda i: s[i].target_speed >= _ZIG_SPEED_M_S
                  and abs(s[i].target_steer) > _ZIG_MIN_RAD and not s[i].brake and dd[i] < 0.3)
    # 階段部（0.3m/s・中立付近だけ0.6m/s。2026-09-26 までの記録は掃引部）。走行部（1.0m/s）とは指令速度で分ける
    low = _series(log, "低速の全域", lambda i: 0.0 < s[i].target_speed < _ZIG_SPEED_M_S
                  and not s[i].brake and dd[i] < 0.5)
    return [x for x in (zig, low) if x is not None]


def _geo_steer_dynamic(log: Log) -> list[_GeoSeries]:
    """ステア試験の**過渡**（舵のステップの直後を含む全サンプル）→ ヨーの遅れの材料。

    走行中（1.0m/s）のジグザグと低速（0.2m/s）の大振幅ステップの両方を使う。同じ緩和長
    でも時間の遅れが速度に反比例するので、2つの速度があれば IMU のローパス（時間で一定）
    と緩和長（距離で一定）を分けられる。"""
    s = log.samples
    # 舵角は平滑化しない（移動平均はステップの角を鈍らせ、それを遅れと取り違える。ここでは
    # ガタのモデルに通さないので、ノイズがガタに吸収される心配も無い）
    zig = _series(log, "走行中の過渡", lambda i: s[i].target_speed >= _ROLLING_M_S and not s[i].brake,
                  smooth=False)
    low = _series(log, "低速の過渡", lambda i: 0.0 < s[i].target_speed < _ROLLING_M_S
                  and not s[i].brake, min_v=0.12, smooth=False)
    return [x for x in (zig, low) if x is not None]


def _geo_corner(log: Log) -> _GeoSeries | None:
    """旋回グリップ試験の、限界より十分下（頭打ちの8割未満）の区間 → 速度依存の材料。"""
    stages, first_sat, _, _ = _corner_split(log)
    if first_sat is not None:
        a_sat = float(np.median([st.a_lat for st in stages[first_sat:]]))
        ok = [st for st in stages[:first_sat] if st.a_lat < 0.8 * a_sat]
    else:
        ok = stages
    keep = {i for st in ok for i in st.idx}
    return _series(log, "旋回", lambda i: i in keep)


_YAW2_DT = 0.01


def _yaw2_filter(t: np.ndarray, wn: float, zeta: float):
    """曲率の2次遅れ（振動的なヨー応答）`ω²/(s²+2ζωs+ω²)` を等間隔（`_YAW2_DT`）の格子で1次ホールド離散化した
    `lfilter` の係数 `(b, a)`。`wn<=0` なら None（遅れなし）。格子は `_apply_yaw2` が作る。

    記録は取りこぼしで刻みが不揃いなので、入力を等間隔の格子へ直線補間してから掛け、
    記録の時刻へ補間して戻す（`_apply_yaw2`）。一定の刻みとみなして掛けると、取りこぼしの
    後の振動の位相が20msずれ、振動的なヨー応答の真値で3次の項が偏った（2026-09-26）。"""
    if wn <= 1e-6:
        return None
    num, den, _ = cont2discrete(([wn * wn], [1.0, 2.0 * zeta * wn, wn * wn]), _YAW2_DT, method="foh")
    return np.ravel(num), np.ravel(den)


def _apply_yaw2(t: np.ndarray, x: np.ndarray, f2) -> np.ndarray:
    b, a = f2
    grid = np.arange(float(t[0]), float(t[-1]) + _YAW2_DT, _YAW2_DT)
    return np.interp(t, grid, lfilter(b, a, np.interp(grid, t, x)))


def _geo_noise_floor(logs: dict[str, "Log"], series: list, L: float) -> float:
    """センサのノイズだけで生じる `fit_geometry` の残差の RMS の見積もり [rad]（舵角換算）。

    曲率の観測 = ヨーレート÷速度 なので、静止中のヨーレートのノイズ σ_w は舵角換算で
    `σ_w·L/v`、重み（速度に比例）を掛けると `σ_w·L/v̄`。舵角は n 点で均すので `σ_s/√n`。"""
    ws, ss = [], []
    for lg in logs.values():
        s = lg.samples
        still = [x.yaw_rate for x in s if abs(x.speed) < 0.01 and abs(x.target_speed) < 1e-6]
        if len(still) >= 20:
            ws.append(float(np.std(still)))
        sn = _steer_noise(s)
        if sn > 0:
            ss.append(sn)
    v_bar = float(np.mean(np.concatenate([g.v for g in series])))
    still = 0.0
    if ws:
        sw = float(np.median(ws))
        sst = float(np.median(ss)) if ss else 0.0
        n_sm = min(g.n_smooth for g in series)
        still = math.sqrt((sw * L / max(v_bar, 1e-3)) ** 2 + sst * sst / n_sm)
    return max(still, _geo_moving_noise(series, L, v_bar))


def _geo_moving_noise(series: list, L: float, v_bar: float) -> float:
    """走っている間の曲率のばらつき（舵角換算、残差と同じ速度の重み）[rad]。舵角が変わらない
    観測点の並び（0.2°以上動いたら区切る）の後半で、直線を引いた残りの二乗平均の中央値。

    曲率 = ヨーレート÷速度 のノイズは走っている間の方がずっと大きい（振動・ジャイロ・前輪の
    オドメトリ）。実機の記録で一定の舵で前進している間のばらつきは舵角換算で0.5〜1.0°あり、
    静止中の値（0.02°）だけを基準にしていたので、残差がノイズ程度でも★「40倍」と出た（2026-09-27）。"""
    out = []
    for g in series:
        pos = np.asarray(g.idx)
        d = g.steer[pos]
        cut = np.where((np.diff(pos) > 1) | (np.abs(np.diff(d)) > math.radians(0.2)))[0] + 1
        for c in np.split(np.arange(len(pos)), cut):
            if len(c) < 10:
                continue
            c = c[len(c) // 2:]
            eq = np.arctan(g.kappa[c] * L)
            w = g.v[c] / max(v_bar, 1e-3)
            x = np.arange(len(c), dtype=float)
            out.append(float(np.sqrt(np.mean((w * (eq - np.polyval(np.polyfit(x, eq, 1), x))) ** 2))))
    return float(np.median(out)) if len(out) >= 3 else 0.0


def _link_series(steer: np.ndarray, hyst: float, dead: float) -> np.ndarray:
    """`SteerLink` を記録の頭から通したタイヤ角の系列（シムと同じ実装）。"""
    link = SteerLink(hyst, dead)
    return np.array([link.step(x) for x in steer.tolist()])


def fit_geometry(logs: dict[str, Log], base: VehicleSpec | None = None) -> FitResult:
    """舵の効き（`steer_gain`・`steer_gain_cubic`）・中立ずれ・アンダーステア勾配・リンクの
    ガタと不感帯を、使える試験の観測をまとめて1度に当てはめる。

    `logs` のキーは `"accel"`（前後運動試験の定速区間→中立ずれ。2026-09-26 までの記録は
    `"speed"`＝速度応答試験）・`"steer"`（走行中の小舵角と低速の全域の階段→効き・非線形性・
    ガタ・不感帯）・`"corner"`（限界より下の区間→速度依存）。
    モデルは `sim/vehicle.py` と同じ：タイヤ角 d = `SteerLink`（ガタ→不感帯）(steer_actual)、
    曲率 = tan(gain·d + cubic·d³ + offset) / (L + K·v²)。

    - 残差は**舵角の単位**で測る（観測した曲率を、その速度で出すのに要る舵角
      `atan(κ·(L + K·v²))` に直して比べる）。曲率のままだと直進と大舵角で重みが桁違い
    - ガタ・不感帯は幅を格子で探し（履歴を持つので微分で追えない）、その他は各格子点で
      最小二乗。ガタ・不感帯・3次の項・アンダーステア勾配は、入れると残差が「ノイズでは
      説明できないほど」縮むときだけ採る（尤度比、`_CAP_LR_THRESHOLD`）。使わない自由度を
      シムに入れない
    - 観測で決まらない量は `base` の値のまま（中立ずれ: 舵0°付近か左右両方の観測、
      効き: |δ|>3°、3次の項: |δ|の幅15°以上、勾配: |δ|>3°で v² の幅0.3以上）
    """
    base = base or VehicleSpec.load()
    series: list[_GeoSeries] = []
    for key in ("speed", "accel"):
        if key in logs:
            series += [x for x in (_geo_straight(logs[key]),) if x is not None]
    if "steer" in logs:
        series += _geo_steer(logs["steer"])
    if "corner" in logs:
        series += [x for x in (_geo_corner(logs["corner"]),) if x is not None]
    if not series:
        raise ValueError("舵の効きを読める区間がありません（前後運動・ステア・旋回の記録を確認）")

    v_all = np.concatenate([g.v for g in series])
    d_all = np.concatenate([g.steer[g.idx] for g in series])
    kap_all = np.concatenate([g.kappa for g in series])
    big = np.abs(d_all) > math.radians(3.0)
    has_gain = int(np.count_nonzero(big)) >= 20
    has_off = bool(np.count_nonzero(np.abs(d_all) < math.radians(1.0)) >= 20) or (
        bool(np.any(d_all[big] > 0)) and bool(np.any(d_all[big] < 0)))
    has_cubic = has_gain and float(np.ptp(np.abs(d_all[big]))) > math.radians(15.0)
    has_k = has_gain and float(np.ptp(v_all[big] ** 2)) > 0.3
    # ガタ・不感帯は、同じ舵角に左右両方から近づく記録（階段・掃引）があるときだけ
    has_link = any(g.source == "低速の全域" for g in series)
    L = base.wheelbase

    #: 当てはめない量の値。観測で決まらないものは base のまま、外したものは 0
    fixed = {"steer_gain": base.steer_gain, "steer_gain_cubic": base.steer_gain_cubic,
             "steer_offset_rad": base.steer_offset_rad, "understeer_gradient": base.understeer_gradient}
    lo = {"steer_gain": 0.5, "steer_gain_cubic": -2.0, "steer_offset_rad": -0.1,
          "understeer_gradient": -0.05}
    hi = {"steer_gain": 1.5, "steer_gain_cubic": 2.0, "steer_offset_rad": 0.1,
          "understeer_gradient": 0.3}

    # 途中結果は上限つきで持つ（100Hz の記録で溜め込むと1回の解析で8GBを超えた、2026-09-26）。
    # 緩和長の格子（17通り）・ガタの格子が回っている間は捨てずに済む大きさ
    link_cache = _LRU(64)
    w1_cache = _LRU(24)
    w2_cache = _LRU(24)
    yaw2_cache = _LRU(64)
    #: 直前に解けたパラメータ（次の最小二乗の初期値。格子の隣の点はほぼ同じ解になる）
    warm: dict[str, float] = {}

    def prepare(sers: list[_GeoSeries], hyst: float, dead: float, lag: tuple[float, float, float, float]):
        """リンクを通したタイヤ角の全系列と、ヨーの遅れの重み（同じ組み合わせは使い回す）。"""
        out = []
        for g in sers:
            d_full = link_cache.get((id(g), round(hyst, 6), round(dead, 6)),
                                    lambda g=g: _link_series(g.steer, hyst, dead))
            w1 = w1_cache.get((id(g), round(lag[1], 6)), lambda g=g: _lag_w1(g, lag[1]))
            w2 = w2_cache.get((id(g), round(lag[0], 6)), lambda g=g: _lag_w2(g, lag[0]))
            f2 = yaw2_cache.get((id(g), round(lag[2], 4), round(lag[3], 4)),
                                lambda g=g: _yaw2_filter(g.t, lag[2], lag[3]))
            out.append((g, d_full, w1 + w2 + (f2,)))
        return out

    def resid_fn(prep, names: list[str]):
        v_o = np.concatenate([g.v for g, *_ in prep])
        k_o = np.concatenate([g.kappa for g, *_ in prep])
        w_o = v_o / float(np.mean(v_o))
        # 曲率 = ヨーレート/速度 なので、ヨーレートのノイズは舵角換算で速度に反比例して効く。
        # 重みを速度に比例させる（等しい重みだと低速のノイズに引っ張られ、不感帯とガタを取り違えた）

        def unpack(x: np.ndarray) -> dict[str, float]:
            p = dict(fixed)
            p.update(zip(names, (float(xi) for xi in x)))
            return p

        def resid(x: np.ndarray) -> np.ndarray:
            p = unpack(x)
            preds = []
            for g, d_full, (W1, J1, W2, J2, f2) in prep:
                dl = p["steer_gain"] * d_full + p["steer_gain_cubic"] * d_full ** 3 + p["steer_offset_rad"]
                kap = np.tan(dl) / (L + p["understeer_gradient"] * g.v_full ** 2)
                kap = (W1 * kap[J1]).sum(axis=1)          # タイヤの緩和長（曲率に、距離で）
                if f2 is not None:                        # 振動的なヨー応答（曲率に、時間で）
                    kap = _apply_yaw2(g.t, kap, f2)
                yaw = g.v_full * kap
                preds.append((W2 * yaw[J2]).sum(axis=1) / g.v)   # IMU のローパス（ヨーレートに、時間で）
            k_p = np.concatenate(preds)
            ll = L + p["understeer_gradient"] * v_o * v_o
            return w_o * (np.arctan(k_o * ll) - np.arctan(k_p * ll))
        return resid, unpack

    def solve(names: list[str], hyst: float, dead: float, lag: tuple[float, float, float, float],
              sers: list[_GeoSeries] | None = None):
        prep = prepare(sers if sers is not None else series, hyst, dead, lag)
        resid, unpack = resid_fn(prep, names)
        x0 = [min(max(warm.get(n, fixed[n]), lo[n]), hi[n]) for n in names]
        r = least_squares(resid, x0=x0, bounds=([lo[n] for n in names], [hi[n] for n in names]))
        p = unpack(r.x)
        warm.update({n: p[n] for n in names})
        return p, float(np.mean(r.fun ** 2)), r.fun

    names = [n for n, ok in (("steer_gain", has_gain), ("steer_gain_cubic", has_cubic),
                             ("steer_offset_rad", has_off), ("understeer_gradient", has_k)) if ok]
    if not names:
        raise ValueError("舵の効き・中立ずれのどちらも決められる観測がありません")

    #: (IMUのローパス [s], タイヤの緩和長 [m], ヨーの2次遅れの固有角振動数 [rad/s], 減衰比)
    lag = (0.0, 0.0, 0.0, 1.0)

    def search(names: list[str], use_h: bool, use_d: bool):
        """ガタ・不感帯の幅を格子で探す。座標ごとに「全域を0.2°刻み → 最良点の周り±0.2°を
        0.05°刻み」を3周回す。ガタと不感帯は互いに補い合うので、粗い格子の周りだけを細かく
        探すと途中の値（真値0.8°に対し0.5°）で抜け出せなくなった。"""
        coarse = np.radians(np.arange(0.0, 3.01, 0.2))
        fine = np.radians(np.arange(-0.2, 0.201, 0.05))
        h = dd = 0.0
        p, e, res = solve(names, h, dd, lag)
        for _ in range(3 if (use_h and use_d) else 1):
            for axis, use in (("h", use_h), ("d", use_d)):
                if not use:
                    continue
                for grid in (coarse, None):
                    cur = h if axis == "h" else dd
                    xs = coarse if grid is not None else (cur + fine)
                    xs = xs[xs >= 0.0]
                    cands = [(solve(names, *((x, dd) if axis == "h" else (h, x)), lag), x) for x in xs]
                    (p, e, res), cur = min(cands, key=lambda c: c[0][1])
                    if axis == "h":
                        h = cur
                    else:
                        dd = cur
        return p, e, h, dd, res

    # ヨーの遅れ（IMUのローパス・タイヤの緩和長）は、舵のステップ直後の過渡から読む。
    # 遅れを無視すると、上げ下げで曲率がずれて見え、ガタに化ける（ゆっくりした掃引で起きた）
    # （ベンチでヨーレートの遅れ30msがガタ+0.5°になった）。一方、遅れを先に単独で読むと
    # ガタ・不感帯の静的なずれが遅れに混ざる（IMU 20ms を 10ms、緩和長3cmを IMU 40ms と
    # 読んだ）。そこで「遅れを固定してガタ等を探す」と「それを固定して遅れを探す」を交互に回す
    dyn = _geo_steer_dynamic(logs["steer"]) if "steer" in logs else []
    notes_lag = "ステア試験の記録が無く、ヨーの遅れは測れず（0とした）"
    use_h = use_d = has_link

    def estimate_lag(p_cur: dict[str, float], h_: float, d_: float) -> tuple[float, float, float, float]:
        bak = dict(fixed)
        fixed.update({k: p_cur[k] for k in fixed})
        preps: dict = {}

        def dyn_resid(lag_: tuple[float, float, float, float]) -> np.ndarray:
            resid, _ = resid_fn(prepare(dyn, h_, d_, lag_), [])
            return resid(np.array([]))

        def sse_dyn(*lag_: float) -> float:
            # 残差の列は持たず平均二乗誤差だけを覚える（格子は千通りを超える）
            key = tuple(round(x, 4) for x in lag_)
            if key not in preps:
                r = dyn_resid(lag_)
                preps[key] = float(np.mean(r * r))
            return preps[key]

        taus = np.arange(0.0, 0.061, 0.005)
        relaxes = np.arange(0.0, 0.081, 0.005)
        e_best, tau_b, rel_b = min((sse_dyn(a_, b_, 0.0, 1.0), a_, b_) for a_ in taus for b_ in relaxes)
        wn_b, z_b = 0.0, 1.0
        # ヨーの2次遅れ（振動的な応答）。1次の遅れで表せない行き過ぎがあるときだけ入る。
        # これが無いと、振動的なヨー応答（ζ=0.4）で舵の効きが13%ずれ、ガタと不感帯が入れ替わった
        e2_best = None
        for wn in (8.0, 12.0, 16.0, 20.0, 25.0, 32.0, 40.0):
            for z in (0.2, 0.3, 0.4, 0.5, 0.7):
                for a_ in np.arange(0.0, 0.061, 0.01):
                    for b_ in (0.0, 0.02, 0.04, 0.06):
                        e_ = sse_dyn(a_, b_, wn, z)
                        if e2_best is None or e_ < e2_best[0]:
                            e2_best = (e_, a_, b_, wn, z)
        if e2_best is not None and _lr_significant(n_eff(dyn_resid(e2_best[1:])), e2_best[0], e_best):
            e_best, tau_b, rel_b, wn_b, z_b = e2_best
        # 格子の最良点から連続値で詰める（格子の粗さの食い違いを舵の効き・3次の項が吸った）
        x0 = [tau_b, rel_b] + ([wn_b, z_b] if wn_b > 0 else [])

        def f(x):
            x = [max(0.0, x[0]), max(0.0, x[1])] + ([max(3.0, x[2]), min(max(0.1, x[3]), 1.5)]
                                                     if len(x) > 2 else [0.0, 1.0])
            return sse_dyn(*x)

        r_ = minimize(f, x0=x0, method="Nelder-Mead",
                      options={"xatol": 2e-4, "fatol": 1e-12, "maxiter": 150,
                               "initial_simplex": None})
        if r_.fun < e_best:
            xs = list(r_.x)
            tau_b, rel_b = max(0.0, xs[0]), max(0.0, xs[1])
            if wn_b > 0:
                wn_b, z_b = max(3.0, xs[2]), min(max(0.1, xs[3]), 1.5)
            e_best = float(r_.fun)
        # 片方ずつ 0 にして、有意に悪化しないなら 0（使わない遅れをシムに入れない）
        n_dyn = n_eff(dyn_resid((tau_b, rel_b, wn_b, z_b)))
        if not _lr_significant(n_dyn, e_best, sse_dyn(0.0, rel_b, wn_b, z_b)):
            tau_b = 0.0
        if not _lr_significant(n_dyn, sse_dyn(tau_b, rel_b, wn_b, z_b), sse_dyn(tau_b, 0.0, wn_b, z_b)):
            rel_b = 0.0
        fixed.clear()
        fixed.update(bak)
        return float(tau_b), float(rel_b), float(wn_b), float(z_b)

    def joint_refine(p_cur: dict[str, float], h_: float, d_: float,
                     lag_: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
        """舵の効きとヨーの遅れを、全系列（定常＋過渡）で同時に詰め直す。交互に回すだけだと、
        遅れの4つ（IMU・緩和長・ω・ζ）が互いに入れ替わった解に止まり、それを舵の効きと3次の
        項が補った（ランダムな真値の検証で効き6%・3次の項+0.2、2026-09-26）。遅れの重みは
        値ごとに作り直す（キャッシュすると連続値の探索で膨れる）。"""
        act = [i for i in (0, 1) if lag_[i] > 0]
        use2 = lag_[2] > 0
        sers = series + dyn
        links = [_link_series(g.steer, h_, d_) for g in sers]

        def unpack(x):
            k = len(names)
            lg = list(lag_)
            for i in act:
                lg[i] = max(0.0, float(x[k]))
                k += 1
            if use2:
                lg[2], lg[3] = max(3.0, float(x[k])), min(max(0.1, float(x[k + 1])), 1.5)
            return x[:len(names)], tuple(lg)

        def resid(x):
            xn, lg = unpack(x)
            prep = [(g, dl, _lag_weights(g, lg[0], lg[1]) + (_yaw2_filter(g.t, lg[2], lg[3]),))
                    for g, dl in zip(sers, links)]
            return resid_fn(prep, names)[0](np.asarray(xn))

        x0 = [p_cur[n] for n in names] + [lag_[i] for i in act] + ([lag_[2], lag_[3]] if use2 else [])
        lo_ = [lo[n] for n in names] + [0.0] * len(act) + ([3.0, 0.1] if use2 else [])
        hi_ = [hi[n] for n in names] + [0.2] * len(act) + ([80.0, 1.5] if use2 else [])
        x0 = [min(max(v, a_ + 1e-9), b_ - 1e-9) for v, a_, b_ in zip(x0, lo_, hi_)]
        r0 = resid(np.asarray(x0))
        r = least_squares(resid, x0=x0, bounds=(lo_, hi_), diff_step=1e-3, max_nfev=60)
        if float(np.mean(r.fun ** 2)) >= float(np.mean(r0 ** 2)):
            return lag_
        warm.update({n: float(v) for n, v in zip(names, r.x[:len(names)])})
        return unpack(r.x)[1]

    p, err, h, dd, res = search(names, use_h, use_d)
    if len(dyn) == 2 and has_gain:
        # 交互に回して、遅れがほとんど変わらなくなるまで（連続値で詰めるので「完全に同じ」には
        # ならない。3周で打ち切ると、振動的なヨー応答の真値で3次の項が収束しきらなかった）
        for _ in range(5):
            new_lag = estimate_lag(p, h, dd)
            if all(abs(a_ - b_) <= tol for a_, b_, tol in zip(new_lag, lag, (5e-4, 5e-4, 0.2, 0.01))):
                break
            lag = new_lag
            p, err, h, dd, res = search(names, use_h, use_d)
        lag = joint_refine(p, h, dd, lag)
        p, err, h, dd, res = search(names, use_h, use_d)
        notes_lag = (f"ヨーの遅れ: IMUのローパス {lag[0] * 1000:.0f}ms・タイヤの緩和長 {lag[1] * 100:.1f}cm"
                     + (f"・2次遅れ ω={lag[2]:.0f}rad/s ζ={lag[3]:.2f}" if lag[2] > 0 else "")
                     + "（走行中1.0m/sと低速0.2m/sの過渡から）")

    # 採否: 1つずつ外してみて、残差が有意に悪化しないものは外す（0 に戻す）
    dropped = []
    for opt in ("steer_link_deadband_rad", "steer_link_hysteresis_rad", "steer_gain_cubic",
                "understeer_gradient"):
        if opt == "steer_link_hysteresis_rad" and use_h:
            p2, e2, h2, d2, r2 = search(names, False, use_d)
        elif opt == "steer_link_deadband_rad" and use_d:
            p2, e2, h2, d2, r2 = search(names, use_h, False)
        elif opt in names:
            names2 = [n for n in names if n != opt]
            keep_fixed = fixed[opt]
            fixed[opt] = 0.0
            p2, e2, h2, d2, r2 = search(names2, use_h, use_d)
            fixed[opt] = keep_fixed
        else:
            continue
        if not _lr_significant(n_eff(res), err, e2):
            dropped.append(opt)
            p, err, h, dd, res = p2, e2, h2, d2, r2
            if opt == "steer_link_hysteresis_rad":
                use_h = False
            elif opt == "steer_link_deadband_rad":
                use_d = False
            else:
                names = [n for n in names if n != opt]
                fixed[opt] = 0.0

    vals = {n: p[n] for n in names}
    if has_cubic and "steer_gain_cubic" not in vals:
        vals["steer_gain_cubic"] = 0.0
    if has_k and "understeer_gradient" not in vals:
        vals["understeer_gradient"] = 0.0
    if has_link:
        vals["steer_link_hysteresis_rad"] = h if use_h else 0.0
        vals["steer_link_deadband_rad"] = dd if use_d else 0.0
    if dyn:
        vals["yaw_rate_filter_s"], vals["yaw_relaxation_m"] = lag[0], lag[1]
        vals["yaw_natural_freq_rad_s"], vals["yaw_damping"] = lag[2], lag[3]

    rms = math.degrees(math.sqrt(err))
    by_src = {g.source: len(g.idx) for g in series}
    floor = _geo_noise_floor(logs, series, L)
    notes = [f"観測 {len(v_all)} サンプル（" + "・".join(f"{k} {n}" for k, n in by_src.items())
             + f"）、残差 {rms:.2f}°（舵角換算。ノイズだけなら約 {math.degrees(floor):.2f}°）", notes_lag]
    warnings: list[str] = []
    if floor > 0 and math.radians(rms) > _RESID_WARN_RATIO * floor + math.radians(0.05):
        notes.append(f"★残差がノイズだけのときの {math.radians(rms) / floor:.1f} 倍。舵→曲率→ヨーレートが"
                     "シムのモデルの形から外れている（前輪エンコーダの周期誤差・ヨー応答の形・サーボの"
                     "負荷依存など）。ガタ・不感帯・3次の項は特に信用しないこと")
        warnings = [notes[-1]]
    n_sat = sum(1 for lg in logs.values() for x in lg.samples if abs(x.yaw_rate) >= _GYRO_SAT)
    if n_sat:
        notes.append(f"ジャイロが測定範囲（±{math.degrees(GYRO_RANGE_RAD_S):.0f}°/s）に張り付いたサンプル"
                     f" {n_sat} 個を材料から外した")
    if has_link:
        notes.append(f"リンクのガタ {math.degrees(vals['steer_link_hysteresis_rad']):.2f}°・"
                     f"不感帯 {math.degrees(vals['steer_link_deadband_rad']):.2f}°"
                     "（0 は検出されず。ノイズに埋もれるほど小さい可能性もある——残差の大きさと比べる）")
    else:
        notes.append("階段の記録が無く、リンクのガタ・不感帯は測れず（ステア試験を最後まで録る）")
    for n in dropped:
        if n in ("steer_gain_cubic", "understeer_gradient"):
            notes.append(f"{n} は入れても残差がほとんど縮まないので 0")
    for n, ok in (("steer_gain", has_gain), ("steer_offset_rad", has_off),
                  ("steer_gain_cubic", has_cubic), ("understeer_gradient", has_k)):
        if not ok:
            notes.append(f"{n} は観測が足りず今の値のまま")
    return FitResult(vals, notes, warnings)


# ── ④ 遅延 ───────────────────────────────────────────────────────────────

#: TELEMETRY の `t_us` がスナップショットの時刻になったファームの ID（MainF446RE_V3 の
#: `RAS_FIRMWARE_ID`、2026-09-27）。それより前は送信時刻で、中身は最大1周期古かった
FW_ID_TELEMETRY_SNAPSHOT_TIME = 0x4D463304


def fit_latency(log: Log) -> FitResult:
    """スキャン完了 → STM32 がその指令を受理するまで（`control_latency_s`）。

    `/cmd` の舵角が大きく変わった点ごとに：
    - 反応したスキャン: 舵角に刻んだスキャン番号（`sysid_latency`）で特定する。
      刻みが読めなければ「その `/cmd` より前に publish された最新のスキャン」
    - 受理: その後 `steer_cmd_echo` が動き始めた時刻。エコーは `steer_rate_limit` で
      ランプするので、最初に動いたサンプルの変化量から始点を逆算する
    """
    s = log.samples
    if not log.scans:
        raise ValueError("mcapに /scan が含まれていません（遅延試験のログか確認）")
    t = _arr(s, "t")
    echo = _arr(s, "steer_cmd_echo")
    scan_pub = np.array([sc.t_pub for sc in log.scans])

    cmds = log.cmds
    # 舵の「反転」だけを見る（試験の始まり・終わりの 0°との行き来は、刻んだ番号の
    # 読み取りを狂わせるので除く）
    trans = [i for i in range(1, len(cmds))
             if abs(cmds[i].target_steer - cmds[i - 1].target_steer) > 0.02
             and abs(cmds[i].target_steer) > 0.01 and abs(cmds[i - 1].target_steer) > 0.01]
    if len(trans) < 10:
        raise ValueError("舵の反転がほとんどありません（遅延試験のログか確認）")
    mags = np.array([abs(cmds[i].target_steer) for i in trans])
    base_amp = float(np.min(mags))
    codes = (mags - base_amp) / LATENCY_CODE_STEP_RAD
    coded = bool(np.all(np.abs(codes - np.round(codes)) < 0.15)) and np.ptp(codes) >= 1
    # 刻みの0（=振幅そのもの）は最小の振れ幅。反転が十分あれば8通りすべて現れる
    total, pi_part, link = [], [], []
    for n_tr, i in enumerate(trans):
        c = cmds[i]
        j = int(np.searchsorted(scan_pub, c.t, side="right")) - 1
        if coded:
            want = int(round(codes[n_tr])) % LATENCY_CODE_MOD
            while j >= 0 and log.scans[j].seq % LATENCY_CODE_MOD != want:
                j -= 1
        if j < 0:
            continue
        t_done = log.scans[j].t_done
        k = int(np.searchsorted(t, c.t, side="right")) - 1
        if k < 2 or abs(echo[k] - echo[k - 1]) > _ECHO_EPS:
            continue                                   # 直前のランプが終わっていない
        e0 = echo[k]
        m = k + 1
        while m < len(t) and t[m] < c.t + 0.3 and abs(echo[m] - e0) <= _ECHO_EPS:
            m += 1
        if m >= len(t) or t[m] >= c.t + 0.3:
            continue
        moved = abs(echo[m] - e0)
        remaining = abs(c.target_steer - e0)
        if c.steer_rate_limit > 0 and moved < remaining - 1e-3:
            onset = max(t[m] - moved / c.steer_rate_limit, t[m - 1])
        else:
            onset = 0.5 * (t[m - 1] + t[m])
        total.append(onset - t_done)
        pi_part.append(c.t - t_done)
        link.append(onset - c.t)

    if len(total) < 10:
        raise ValueError("遅延を読める反転が10回未満でした（記録が短いか、エコーが動いていない）")
    tot = np.array(total)
    notes = [f"{len(tot)} 回: 中央値 {np.median(tot) * 1000:.1f}ms・p90 {np.percentile(tot, 90) * 1000:.1f}ms",
             f"内訳（中央値）: スキャン完了→/cmd {np.median(pi_part) * 1000:.1f}ms、"
             f"/cmd→STM32受理 {np.median(link) * 1000:.1f}ms",
             "E2Eモデルの推論時間はこの上に乗る（E2E走行のログを渡すと Pi 側の区間で見られる）。"
             "最終点の計測から最終セクタの受信まで（数ms）は含まない"]
    if not coded:
        notes.append("舵角にスキャン番号が刻まれていない（遅延試験以外のログ）。直前のスキャンに反応したとみなした")
    warnings = []
    if log.fw_id is not None and log.fw_id < FW_ID_TELEMETRY_SNAPSHOT_TIME:
        notes.append(f"★記録したファーム（fw_id 0x{log.fw_id:08X}）は TELEMETRY の t_us が送信時刻で、中身は"
                     "それより最大1周期（100Hz で10ms）古い。遅延がその分ずれ、起動ごとに変わる。"
                     f"0x{FW_ID_TELEMETRY_SNAPSHOT_TIME:08X} 以降のファームで録り直すこと")
        warnings = [notes[-1]]
    return FitResult({"control_latency_s": float(np.median(tot))}, notes, warnings)
