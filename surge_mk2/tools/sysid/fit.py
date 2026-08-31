"""mcapログ → システム同定パラメータのフィッティング。

GUIの「システム同定」タブが録ったmcapは `raspi/nodes/logger_node.py` の
`DEFAULT_TOPICS` のまま（`--topics` は指定していない）なので、`/cmd`
（`raspi/msgs/types.DriveCmd`。**実際にSTM32へ送られた指令**）と
`/vehicle_state`（`VehicleState`）が必ず含まれている。この2トピックだけを
読めば3つの試験すべてが解析できる——`/cmd` は「AutoState（plannerの意図）」
ではなく「安全機構（deadman・ARM）を経た後の実際の指令」なので、こちらの方が
実車の応答と正しく対応する。

各試験の解析関数は `dict[str, float]` を返す（キーは `config/vehicle.toml`
`[dynamics]` のキー名と一致させてある。`tools/sysid/toml_update.py` がそのまま
書き込める形）。
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from mcap.reader import make_reader

__all__ = ["Sample", "load_samples", "fit_steer", "fit_speed", "fit_corner"]

GRAVITY_MPS2 = 9.81


@dataclass
class Sample:
    t: float               # [s] 先頭サンプルからの経過
    target_speed: float     # [m/s]（/cmd）
    target_steer: float     # [rad]（/cmd）
    speed: float             # [m/s]（/vehicle_state）
    steer_actual: float      # [rad]（/vehicle_state）
    yaw_rate: float          # [rad/s]（/vehicle_state）


def load_samples(path: str | Path) -> list[Sample]:
    """`/cmd`と`/vehicle_state`を時刻で対応づけて1本の時系列にする。

    どちらも50Hz程度で独立に出ているので、`/vehicle_state`の各サンプルに
    最も時刻が近い`/cmd`を割り当てる（最近傍探索。ズレは最悪でも半周期＝
    10ms程度で、時定数の推定に対しては無視できる）。
    """
    cmd_msgs: list[tuple[int, dict]] = []
    vs_msgs: list[tuple[int, dict]] = []
    with open(path, "rb") as f:
        reader = make_reader(f)
        for _schema, channel, message in reader.iter_messages(topics=["/cmd", "/vehicle_state"]):
            obj = json.loads(message.data)
            if channel.topic == "/cmd":
                cmd_msgs.append((message.log_time, obj))
            else:
                vs_msgs.append((message.log_time, obj))

    if not cmd_msgs or not vs_msgs:
        raise ValueError(
            "mcapに /cmd または /vehicle_state が含まれていません"
            "（システム同定タブで録ったログか確認してください）")

    cmd_msgs.sort(key=lambda x: x[0])
    vs_msgs.sort(key=lambda x: x[0])
    cmd_ts = np.array([t for t, _ in cmd_msgs], dtype=np.int64)

    t0 = vs_msgs[0][0]
    samples: list[Sample] = []
    for t_ns, vs in vs_msgs:
        idx = int(np.searchsorted(cmd_ts, t_ns))
        idx = min(idx, len(cmd_msgs) - 1)
        if idx > 0 and abs(cmd_ts[idx - 1] - t_ns) < abs(cmd_ts[idx] - t_ns):
            idx -= 1
        cmd = cmd_msgs[idx][1]
        samples.append(Sample(
            t=(t_ns - t0) / 1e9,
            target_speed=float(cmd.get("target_speed", 0.0)),
            target_steer=float(cmd.get("target_steer", 0.0)),
            speed=float(vs.get("speed", 0.0)),
            steer_actual=float(vs.get("steer_actual", 0.0)),
            yaw_rate=float(vs.get("yaw_rate", 0.0)),
        ))
    return samples


# ── 共通: ステップ検出と一次遅れ+むだ時間フィット ──────────────────────────

def _find_steps(t: np.ndarray, target: np.ndarray, threshold: float) -> list[tuple[int, int]]:
    """`target`が`threshold`以上変化した区間の`(start_idx, end_idx)`一覧。

    `end_idx`は次のステップの開始（無ければ配列末尾）。ノイズ（1サンプルだけの
    ブレ）は`threshold`未満として無視する——3つのplannerはいずれもステップを
    十分な保持時間（`hold_s`）だけ保持するので、実際の切り替わりは
    `threshold`を大きく超える
    """
    steps: list[tuple[int, int]] = []
    start = 0
    for i in range(1, len(target)):
        if abs(target[i] - target[start]) > threshold:
            steps.append((start, i))
            start = i
    steps.append((start, len(target)))
    return steps


def _fit_first_order_with_delay(t: np.ndarray, y: np.ndarray) -> tuple[float, float]:
    """`y`を0→1に正規化した1本の応答から `(dead_time_s, tau_s)` を推定する。

    :param t: ステップ開始からの経過時間 [s]（`t[0] == 0`）
    :param y: `(actual - a0) / (target - a0)`。理想的には 0→1 の単調な一次遅れ応答

    - `dead_time_s`: `y`が最初に10%へ達する時刻
    - `tau_s`: `dead_time_s`以降の区間を `ln(1-y)` の線形回帰（傾き=-1/tau）で推定

    ## 振幅が大きいステップだとレート制限で頭打ちになる区間がある

    ステアの試験は振幅（`amplitude_deg`）が大きいほど`steer_rate_limit_rad_s`
    （サーボの物理的な最大角速度）に頭打ちされる時間が長くなる。**純粋な
    一次遅れなら変化率はステップ直後が最大でそこから単調に減っていくが、
    レート制限中は変化率がほぼ一定のまま推移する**——この「一定のまま続く
    区間」をプラトーとして検出し、そこが終わった後（自由に減衰し始めた
    区間）だけを`tau`の回帰に使う。こうしておくと振幅を変えても
    `tau_steer_s`の推定値が大きくブレない（`steer_rate_limit_rad_s`自体は
    このプラトーの実測値から`fit_steer`側で別途求める）。
    """
    # 10%到達点をむだ時間とする（度数の少ない初期区間はノイズに弱いので線形補間）
    above = np.where(y >= 0.1)[0]
    if len(above) == 0:
        return 0.0, 0.0
    i = int(above[0])
    if i == 0:
        dead_time = 0.0
    else:
        # y[i-1] < 0.1 <= y[i] の間を線形補間
        frac = (0.1 - y[i - 1]) / max(y[i] - y[i - 1], 1e-9)
        dead_time = t[i - 1] + frac * (t[i] - t[i - 1])

    regression_start = _skip_rate_limited_plateau(t, y)
    regression_start = max(regression_start, dead_time)

    mask = (t >= regression_start) & (y > 0.02) & (y < 0.95)
    if np.count_nonzero(mask) < 3:
        return dead_time, 0.0
    tau_t = t[mask] - regression_start
    ln1my = np.log(1.0 - y[mask])
    # ln(1-y) = -(t-regression_start)/tau  →  傾きから tau を求める
    slope, _intercept = np.polyfit(tau_t, ln1my, 1)
    tau = -1.0 / slope if slope < -1e-9 else 0.0
    return dead_time, max(tau, 0.0)


def _skip_rate_limited_plateau(t: np.ndarray, y: np.ndarray) -> float:
    """変化率がほぼ一定のまま続く先頭区間（レート制限のプラトー）の終了時刻。

    プラトーが無ければ`t[0]`を返す（何も除外しない）。3サンプル未満の
    「たまたま最初が一番速かった」程度は誤検出を避けるため無視する
    （1サンプルだけ最大なのは純粋な一次遅れでも起きる自然な形）。
    """
    if len(t) < 5:
        return float(t[0]) if len(t) else 0.0
    rates = np.diff(y) / np.maximum(np.diff(t), 1e-9)
    rate_max = np.max(np.abs(rates))
    if rate_max < 1e-9:
        return float(t[0])
    near_peak = np.abs(rates) >= 0.9 * rate_max
    plateau_len = 0
    for v in near_peak:
        if not v:
            break
        plateau_len += 1
    if plateau_len < 3:
        return float(t[0])
    return float(t[plateau_len])


def _step_responses(samples: list[Sample], target_attr: str, actual_attr: str,
                    step_threshold: float, min_hold_s: float) -> list[tuple[np.ndarray, np.ndarray, float, np.ndarray]]:
    """各ステップの `(t_rel, y_normalized, target_delta, actual_raw)` を返す。

    `actual_raw`（正規化前の実測値そのもの）は最大変化率（レート上限の推定）に使う。
    立ち上がり直後から次のステップの手前までを1本として切り出す。
    """
    t = np.array([s.t for s in samples])
    target = np.array([getattr(s, target_attr) for s in samples])
    actual = np.array([getattr(s, actual_attr) for s in samples])

    steps = _find_steps(t, target, step_threshold)
    out = []
    for start, end in steps:
        if start == 0:
            continue                       # 直前の定常値が無い最初の区間は使わない
        if t[end - 1] - t[start] < min_hold_s:
            continue
        a0 = actual[start - 1]             # ステップ直前の定常値
        tgt = target[start]
        delta = tgt - a0
        if abs(delta) < step_threshold:
            continue
        seg_t = t[start:end] - t[start]
        seg_y = (actual[start:end] - a0) / delta
        out.append((seg_t, seg_y, delta, actual[start:end]))
    return out


def fit_steer(samples: list[Sample]) -> dict[str, float]:
    """ステア試験のログ → `tau_steer_s`・`dead_time_s`・`steer_rate_limit_rad_s`。

    複数ステップ（`SysIdSteer`は既定4往復）の推定値を中央値で束ねる
    （中央値は外れ値1本に引きずられにくい）。`steer_rate_limit_rad_s`だけは
    「観測された中で最も速く動いた瞬間」を採る——物理的な床を測りたいので、
    平均ではなく最大が正しい
    """
    resp = _step_responses(samples, "target_steer", "steer_actual",
                           step_threshold=math.radians(2.0), min_hold_s=0.3)
    if not resp:
        raise ValueError("ステップ入力が検出できませんでした（mcapが正しいか確認）")

    dead_times, taus, rates = [], [], []
    for seg_t, seg_y, _delta, seg_actual in resp:
        dt, tau = _fit_first_order_with_delay(seg_t, seg_y)
        dead_times.append(dt)
        taus.append(tau)
        if len(seg_t) > 1:
            rate = np.max(np.abs(np.diff(seg_actual) / np.diff(seg_t)))
            rates.append(float(rate))

    return {
        "dead_time_s": float(np.median(dead_times)),
        "tau_steer_s": float(np.median([t for t in taus if t > 0]) if any(t > 0 for t in taus) else 0.0),
        "steer_rate_limit_rad_s": float(np.max(rates)) if rates else 0.0,
    }


def fit_speed(samples: list[Sample]) -> dict[str, float]:
    """加速試験のログ → `tau_speed_s`。

    `SysIdSpeed`は前進⇄後退を交互にステップするので、前進側・後退側どちらの
    遷移も使う——速度指令モードの一次遅れ（`sim/vehicle.py`の`_next_speed()`）
    は符号に関わらず対称な式で、ブレーキ（`brake=True`）は一度も使わないので
    後退側だけ違う挙動になる理由が無い。
    """
    resp = _step_responses(samples, "target_speed", "speed",
                           step_threshold=0.05, min_hold_s=0.3)
    taus = []
    for seg_t, seg_y, _delta, _seg_actual in resp:
        _dt, tau = _fit_first_order_with_delay(seg_t, seg_y)
        if tau > 0:
            taus.append(tau)
    if not taus:
        raise ValueError("加速ステップが検出できませんでした（mcapが正しいか確認）")
    return {"tau_speed_s": float(np.median(taus))}


def fit_corner(samples: list[Sample]) -> dict[str, float]:
    """旋回グリップ試験のログ → `mu`。

    横方向グリップ限界に達すると実測の向心加速度 `speed*yaw_rate` は
    `mu*g` に頭打ちになる（`sim/vehicle.py` の `step()` と同じ関係）。
    段階的に速度を上げるこの試験は、その頭打ちを実際に踏むことを狙っている
    ので、**記録全体で観測された向心加速度の上位側（95パーセンタイル）**を
    `mu*g`の実測値として使う（単発のノイズに頭打ちの判定を引きずられない
    ように、最大値ではなく上位パーセンタイルにしてある）。

    ## 頭打ちに実際に達していない記録を検出する

    `SysIdCorner`のパラメータ既定値は安全側（低速）に振ってあるので、
    バンビが速度を十分上げないまま止めると、**頭打ちに一度も達しないまま**
    「そのとき出せた最大の向心加速度」を`mu*g`と誤認しかねない
    （検証: 低速のみのログでは`mu`が実際より小さく出ることをシムで確認済み）。

    頭打ちに達していなければ曲率 `yaw_rate/speed` は速度に依らずほぼ一定
    （`tan(steer)/wheelbase`）のはずなので、**速度が低い段と高い段で曲率が
    どれだけ落ちたか**を見て判定する。最高速の段でも曲率が最低速の段の
    90%以上残っているなら「頭打ちを踏めていない」とみなしてエラーにする
    （wheelbaseが要らない判定式にしてある——`yaw_rate/speed`同士の比較なので
    幾何パラメータに依存しない）。
    """
    a_lat = np.array([abs(s.speed * s.yaw_rate) for s in samples if abs(s.speed) > 0.05])
    if len(a_lat) < 5:
        raise ValueError("有効な旋回区間が短すぎます（速度がほぼ0のまま記録されていないか確認）")

    _check_corner_saturated(samples)

    a_lat_est = float(np.percentile(a_lat, 95))
    return {"mu": a_lat_est / GRAVITY_MPS2}


def _check_corner_saturated(samples: list[Sample]) -> None:
    """速度の段ごとの曲率を比べ、頭打ちに達したらしいかを確認する。

    `SysIdCorner`は`target_speed`を段階的に上げる（`_find_steps`で段の境界を
    検出できる）。各段の後半（前半は速度が定常に落ち着く途中なので外す）で
    曲率の中央値を取り、最低速の段と最高速の段を比べる。

    ## 段の検出しきい値は`v_step`より十分小さくすること

    `SysIdCorner`の`v_start`/`v_max`/`stages`次第では1段あたりの速度刻みが
    かなり細かくなりうる（例: `v_start=0.9, v_max=1.0, stages=15`なら約
    0.007m/s）。しきい値が粗すぎると複数の段を1本の区間として誤って束ねて
    しまい、**段が2本未満しか検出できず、この安全確認自体が意味を失う**。
    `DriveCmd.target_speed`の量子化幅（0.001m/s）より十分大きく、かつ現実的な
    最小`v_step`より小さい0.01m/sをしきい値にしてある。

    :raises ValueError: 曲率がほとんど落ちていない（頭打ちに達していない）、
        または段が2本未満しか検出できず判定できない
    """
    t = np.array([s.t for s in samples])
    target_speed = np.array([s.target_speed for s in samples])
    speed = np.array([s.speed for s in samples])
    yaw_rate = np.array([s.yaw_rate for s in samples])

    stages = _find_steps(t, target_speed, threshold=0.01)
    stage_curvatures: list[tuple[float, float]] = []   # (段の目標速度, 曲率中央値)
    for start, end in stages:
        mid = start + (end - start) // 2               # 後半だけを使う
        seg_speed = speed[mid:end]
        seg_yaw = yaw_rate[mid:end]
        mask = seg_speed > 0.05
        if np.count_nonzero(mask) < 3:
            continue
        curvature = np.median(seg_yaw[mask] / seg_speed[mask])
        stage_curvatures.append((float(target_speed[start]), float(abs(curvature))))

    if len(stage_curvatures) < 2:
        # **黙って判定を諦めない。** ここで素通りさせると、頭打ちに達したか
        # 一度も確認しないまま`fit_corner`がmuを返してしまう
        raise ValueError(
            "速度の段が2本以上検出できず、グリップの頭打ちに達したか確認できません"
            "（記録が短すぎるか、v_start/v_max/stagesの組み合わせで刻みが細かすぎる"
            "可能性があります）。録り直すか、v_stepが広がる設定に変えてください")

    stage_curvatures.sort(key=lambda x: x[0])
    low_curvature = stage_curvatures[0][1]
    high_curvature = stage_curvatures[-1][1]
    if low_curvature < 1e-6:
        return
    if high_curvature / low_curvature > 0.9:
        raise ValueError(
            "グリップの頭打ちに達していないようです（最高速でも曲率が"
            f"{high_curvature / low_curvature * 100:.0f}%残っている）。"
            "GUIのシステム同定タブでv_max（最終段の目標速度）を上げ、"
            "実際に滑り出すところまで速度を上げて録り直してください")
