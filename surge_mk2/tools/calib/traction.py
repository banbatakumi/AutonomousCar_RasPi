"""校正 8: TC・ABS の確かめ（`raspi/auto/calib_traction_check.py` の記録）→ 合否と `[limits]`。
設計は `docs/calibration.md` §4.8。

区間ごと（車速 0.6m/s 以上を `_CHUNK_S` に区切る）に、STM32 のスリップ率（駆動・車速指令の減速は左右の平均、
制動はロックしかけている側）と車体の加速度（走行距離の 2 次式）を出す。

- 合否: 車速 `_TARGET_V` 以上でのスリップ率の平均が目標の ±`SLIP_TOL`。区間の中の揺れが `SLIP_STD_MAX` を超えたら、
  その揺れが一つの周波数に集まっているか（`PEAK_SHARE_MAX`）を見る。集まっていれば TC・ABS の発振で不合格、
  広く散っていれば滑っているタイヤの力のむら（2026-10-19 の実機では古いゲインでも新しいゲインでも 0.1〜0.17、
  6〜30Hz に散る。TC の働かない定速では 0.012）なので★だけ
  区間をまたいだ違いは振動ではなく車速による傾き（制動は MD の制動トルクが低速で抜け、スリップ率が下がる）なので、
  参考に出すだけ
- 上限: 全開の発進の加速度・車速指令で目一杯の減速の減速度の中央値を `[limits]` へ
  （slam2d の加速・減速の上限。走行中の学習の初期値・学習しないときの値）
- 予測との比較: 記録の指令を、同定した車（`config/vehicle.toml`）のファームのシムに入れ直した加速度（★のときは
  校正 7 の曲線が実車と合っていない）
"""

from __future__ import annotations

import numpy as np

from raspi.auto._calib_common import phase_parts

from .encoder import _stm_time
from .log import Log
from .replay import car_from_toml, replay
from .result import Result
from .speed import body_distance

__all__ = ["analyze", "SLIP_TOL", "SLIP_STD_MAX", "LIMIT_USE"]

SLIP_TOL = 0.04
SLIP_STD_MAX = 0.04
#: 揺れのパワーのうち一番強い周波数（隣の 1 本を含む）が占める割合の上限。超えたら発振とみなす
PEAK_SHARE_MAX = 0.5
#: 実測の中央値に掛ける割合（[limits] は上限そのもの。使う割合は slam2d が `FIXED_*_USE` で掛ける）
LIMIT_USE = 1.0
_CHUNK_S = 0.12
_MIN_V = 0.6
_TARGET_V = 1.0
#: 区間の始まりから定常の評価に使わない時間 [s]（制動を掛けた瞬間の一瞬のロック・発進の空転は別に報告する）
_ONSET_S = 0.12


def _onsets(log: Log, kind: str, slip: np.ndarray, select_low: bool) -> list[tuple[float, float]]:
    """区間の始まりの行き過ぎ: (スリップ率の大きさの最大, 目標の 2 倍を超えていた時間 [s]) の並び。"""
    out = []
    for _, i0, i1 in log.segments(lambda p, k=kind: (phase_parts(p) or ("",))[0] == k):
        j1 = min(i1, i0 + int(0.3 / 0.01))
        sl, sr = slip[i0:j1, 0], slip[i0:j1, 1]
        s = np.abs(np.minimum(sl, sr) if select_low else 0.5 * (sl + sr))
        out.append((float(np.max(s)) if len(s) else 0.0, s))
    return out


def _chunks(log: Log, kind: str, t: np.ndarray, x: np.ndarray, v: np.ndarray):
    out = []
    for _, i0, i1 in log.segments(lambda p, k=kind: (phase_parts(p) or ("",))[0] == k):
        idx = np.arange(i0, i1)
        idx = idx[(np.abs(v[idx]) >= _MIN_V) & (t[idx] - t[i0] >= _ONSET_S)]
        if len(idx) < 8:
            continue
        n = max(1, int((t[idx[-1]] - t[idx[0]]) / _CHUNK_S))
        for part in np.array_split(idx, n):
            if len(part) >= 6:
                cf = np.polyfit(t[part] - t[part[0]], x[part], 2)
                out.append((part, 2 * cf[0]))
    return out


def peak_share(log: Log, kind: str, series_of, v: np.ndarray, t: np.ndarray) -> tuple[float, float]:
    """区間ごとのスリップ率（1 次の傾きを引く）のパワースペクトルを足し、一番強い周波数とその隣が占める割合と
    その周波数 [Hz] を返す。発振なら一つの周波数に集まり、タイヤの力のむらなら広く散る。"""
    n = 32
    acc = None
    for _, i0, i1 in log.segments(lambda p, k=kind: (phase_parts(p) or ("",))[0] == k):
        idx = np.arange(i0, i1)
        idx = idx[(np.abs(v[idx]) >= _MIN_V) & (t[idx] - t[i0] >= _ONSET_S)]
        for j in range(0, len(idx) - n + 1, n):
            x = series_of(idx[j:j + n])
            x = x - np.polyval(np.polyfit(np.arange(n), x, 1), np.arange(n))
            P = np.abs(np.fft.rfft(x * np.hanning(n))) ** 2
            acc = P if acc is None else acc + P
    if acc is None or acc[1:].sum() <= 0:
        return 0.0, 0.0
    P = acc[1:]
    k = int(np.argmax(P))
    share = float(P[max(0, k - 1):k + 2].sum() - min(P[max(0, k - 1)], P[min(len(P) - 1, k + 1)])) / float(P.sum())
    return share, float(np.fft.rfftfreq(n, 0.01)[k + 1])


def analyze(log: Log, car=None) -> Result:
    res = Result("校正 8: TC・ABS の確かめ")
    if log.aborted:
        res.warn(f"試験は途中で中止された（{log.aborted}）")
    t = _stm_time(log)
    x = body_distance(log)
    v = np.gradient(x, t)
    slip = log.vs["tc_slip"]
    tc_t = log.control_params.get("tc_slip_target")
    abs_t = log.control_params.get("abs_slip_target")
    if tc_t is None or abs_t is None:
        res.warn("記録に STM32 の調整値（/diag/link）が無い。目標スリップ率は toml の値を使う")
        from . import toml_io
        f = toml_io.read_flat()
        tc_t = f.get("control.tc_slip_target", 0.1)
        abs_t = f.get("control.abs_slip_target", 0.15)
    res.info(f"記録したときの目標スリップ率: TC {tc_t:.3f}・ABS {abs_t:.3f}、ゲイン TC kp "
             f"{log.control_params.get('tc_kp_nm_per_m_s', float('nan')):.3f}・ki {log.control_params.get('tc_ki_nm_per_m', float('nan')):.2f}"
             f"、ABS kp {log.control_params.get('abs_kp_nm_per_m_s', float('nan')):.3f}・ki {log.control_params.get('abs_ki_nm_per_m', float('nan')):.2f}")
    measured: dict[str, float] = {}
    pred = None
    try:
        pred = replay(log, car or car_from_toml(), latency=0.012)
    except Exception as e:                      # noqa: BLE001  予測が作れなくても合否は出す
        res.info(f"予測を作れなかった（{e}）")
    for kind, name, target, select_low in (("drive", "全開の発進（TC）", tc_t, False),
                                            ("decel", "車速指令の減速（TC）", tc_t, False),
                                            ("brake", "最大の制動（ABS）", abs_t, True)):
        ch = _chunks(log, kind, t, x, v)
        if len(ch) < 3:
            res.fail(f"{name}: 区間が足りない（{len(ch)} 個。`calib_traction_check` の記録か確かめる）")
            continue
        lam, acc, inner, vv = [], [], [], []
        for part, a in ch:
            sl, sr = slip[part, 0], slip[part, 1]
            series = np.abs(np.minimum(sl, sr) if select_low else 0.5 * (sl + sr))
            lam.append(float(np.mean(series)))
            inner.append(float(np.std(series)))
            acc.append(abs(a))
            vv.append(float(np.mean(np.abs(v[part]))))
        lam, acc, vv = np.array(lam), np.array(acc), np.array(vv)
        fast = vv >= _TARGET_V
        mean = float(np.mean(lam[fast])) if fast.any() else float(np.mean(lam))
        std = float(np.max(inner))
        if fast.any() and (~fast).any():
            res.info(f"{name}: スリップ率は {_TARGET_V:.0f}m/s 以上で {np.mean(lam[fast]):.3f}・未満で {np.mean(lam[~fast]):.3f}"
                     "（車速による傾き。振動ではない）")
        a_med = float(np.median(acc))
        measured[kind] = a_med
        txt = f"{name}: スリップ率 {mean:.3f}（区間の中の揺れ 最大 {std:.3f}、目標 {target:.3f}）、{'加速度' if kind == 'drive' else '減速度'}の中央値 {a_med:.2f}m/s²"
        if pred is not None:
            pa = [abs(float(np.polyfit(t[p] - t[p[0]], pred["distance"][p], 2)[0]) * 2) for p, _ in ch]
            txt += f"（予測 {np.median(pa):.2f}m/s²）"
        res.info(txt)
        if abs(mean - target) > SLIP_TOL:
            # 試験のトルクで目標まで滑らない（タイヤが強い）なら外れていても問題ない: 制限が働いていない
            lim = log.vs["tc_limit_nm"].min(axis=1) if kind != "brake" else log.vs["abs_limit_nm"]
            idxs = np.concatenate([p for p, _ in ch])
            active = float(np.mean(lim[idxs] < 0.13 - 1e-3))
            if mean < target and active < 0.3:
                res.warn(f"{name}: スリップ率が目標まで上がらず、制限が働いたのは {active:.0%} だけ（トルクの上限で決まっている）。"
                         "この試験では TC・ABS の効きを確かめられていない（グリップの強い路面？）")
            else:
                res.fail(f"{name}: スリップ率の平均 {mean:.3f} が目標 {target:.3f} から {abs(mean - target):.3f} 外れる（目安 ±{SLIP_TOL}）")
        else:
            res.ok(f"{name}: スリップ率が目標に保たれている（差 {abs(mean - target):.3f}）")
        if std > SLIP_STD_MAX:
            def series_of(part, sl=select_low):
                a, b = slip[part, 0], slip[part, 1]
                return np.abs(np.minimum(a, b) if sl else 0.5 * (a + b))
            share, f_pk = peak_share(log, kind, series_of, v, t)
            if share > PEAK_SHARE_MAX:
                res.fail(f"{name}: 区間の中のスリップ率の揺れ {std:.3f}、その {share:.0%} が {f_pk:.0f}Hz に集まる"
                         f"（目安 {PEAK_SHARE_MAX:.0%} 以下。TC・ABS が発振している）")
            else:
                res.warn(f"{name}: 区間の中のスリップ率の揺れ {std:.3f} は大きいが、周波数は散っている（一番強い {f_pk:.0f}Hz で"
                         f" {share:.0%}）。発振ではなく、滑っているタイヤの力のむら")
        else:
            res.ok(f"{name}: 振動していない（区間の中の揺れ 最大 {std:.3f}）")
        ons = _onsets(log, kind, slip, select_low)
        if ons:
            peak = max(o[0] for o in ons)
            over = max(float(np.sum(o[1] > 2 * target)) * 0.01 for o in ons)
            res.info(f"{name}: 始まりの行き過ぎ スリップ率 最大 {peak:.2f}・目標の 2 倍を超えた時間 最長 {over * 1000:.0f}ms")
        tq = log.vs["torque_cmd"].sum(axis=1)
        for part, _ in ch[:1]:
            res.plot(f"{name}: スリップ率（横軸: 時刻 [s]）", "左", log.t[part], slip[part, 0])
            res.plot(f"{name}: スリップ率（横軸: 時刻 [s]）", "右", log.t[part], slip[part, 1])
        del tq
    if "drive" in measured:
        res.values["limits.a_accel_m_s2"] = round(LIMIT_USE * measured["drive"], 3)
        res.notes["limits.a_accel_m_s2"] = "全開の発進（TC）の実測の中央値"
    if "brake" in measured:
        res.values["limits.abs_brake_decel_m_s2"] = round(measured["brake"], 3)
        res.notes["limits.abs_brake_decel_m_s2"] = ("最大の制動（ABS）の実測の中央値。io_node が STM32 へ送り、自動停止の"
                                                    "停止距離と DISARM 後の制動の上限に使う")
    if "decel" in measured:
        res.values["limits.a_brake_m_s2"] = round(LIMIT_USE * measured["decel"], 3)
        res.notes["limits.a_brake_m_s2"] = "車速指令で目一杯の減速（TC）の実測の中央値"
    return res
