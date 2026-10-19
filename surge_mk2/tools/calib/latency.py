"""校正 4: 遅延（`raspi/auto/calib_latency.py` の記録 → `[dynamics] control_latency_s`）。設計は `docs/calibration.md` §4.4。

`/cmd` の舵角が反転した点ごとに:
- 反応したスキャン: 舵角に刻んだスキャンの番号で特定する（刻みが読めなければ、その `/cmd` より前で最新のスキャン）
- 受理: その後 STM32 の `steer_cmd_echo` が動き始めた時刻。エコーは指令の舵速度の上限でランプするので、最初に動いた
  サンプルの変化量から動き始めを逆算する

遅れ = 受理 − スキャンの最後の点を測り終えた時刻。中央値を `control_latency_s` にする。内訳（スキャン完了 → `/cmd`、
`/cmd` → 受理）も出す。時刻はどちらも Pi の時計（`steer_cmd_echo` の時刻は STM32 の時刻を時刻同期で換算したもの）。
"""

from __future__ import annotations

import numpy as np

from raspi.auto.calib_latency import CODE_MOD, CODE_STEP_RAD

from .log import Log
from .result import Result

__all__ = ["analyze"]

_ECHO_EPS = 5e-4


def analyze(log: Log) -> Result:
    res = Result("校正 4: 遅延（スキャン完了 → STM32 受理）")
    if not log.scans:
        res.fail("記録に /scan が無い（遅延の試験の記録か確かめる）")
        return res
    t = log.t
    echo = log.vs["steer_cmd_echo"]
    sc_seq = np.array([s[0] for s in log.scans])
    sc_done = np.array([s[1] for s in log.scans])
    sc_pub = np.array([s[2] for s in log.scans])
    cs = log.cmd["target_steer"]
    rate = log.cmd["steer_rate_limit"]
    trans = [i for i in range(1, len(cs)) if abs(cs[i] - cs[i - 1]) > 0.02 and abs(cs[i]) > 0.01 and abs(cs[i - 1]) > 0.01]
    if len(trans) < 10:
        res.fail(f"舵の反転が {len(trans)} 回しかない（遅延の試験の記録か確かめる）")
        return res
    mags = np.array([abs(cs[i]) for i in trans])
    base = float(np.min(mags))
    codes = (mags - base) / CODE_STEP_RAD
    coded = bool(np.all(np.abs(codes - np.round(codes)) < 0.15)) and np.ptp(codes) >= 1
    total, pi_part, link = [], [], []
    for n_tr, i in enumerate(trans):
        tc = log.cmd_t[i]
        j = int(np.searchsorted(sc_pub, tc, side="right")) - 1
        if coded:
            # 刻み = 反応したスキャンの番号 mod 8（最小の振れ幅が刻み 0。反転が十分あれば 8 通り全部が出る）
            want = int(round(codes[n_tr])) % CODE_MOD
            while j >= 0 and sc_seq[j] % CODE_MOD != want:
                j -= 1
        if j < 0:
            continue
        k = int(np.searchsorted(t, tc, side="right")) - 1
        if k < 2 or abs(echo[k] - echo[k - 1]) > _ECHO_EPS:
            continue                                   # 直前のランプが終わっていない
        e0 = echo[k]
        m = k + 1
        while m < len(t) and t[m] < tc + 0.3 and abs(echo[m] - e0) <= _ECHO_EPS:
            m += 1
        if m >= len(t) or t[m] >= tc + 0.3:
            continue
        moved = abs(echo[m] - e0)
        remaining = abs(cs[i] - e0)
        if rate[i] > 0 and moved < remaining - 1e-3:
            onset = max(t[m] - moved / rate[i], t[m - 1])
        else:
            onset = 0.5 * (t[m - 1] + t[m])
        total.append(onset - sc_done[j])
        pi_part.append(tc - sc_done[j])
        link.append(onset - tc)
    if len(total) < 10:
        res.fail(f"遅延を読める反転が {len(total)} 回しかない（記録が短いか、エコーが動いていない）")
        return res
    tot = np.array(total)
    med = float(np.median(tot))
    res.info(f"{len(tot)} 回: 中央値 {med * 1000:.1f}ms・p90 {np.percentile(tot, 90) * 1000:.1f}ms・"
             f"ばらつき（四分位の幅）{(np.percentile(tot, 75) - np.percentile(tot, 25)) * 1000:.1f}ms")
    res.info(f"内訳（中央値）: スキャン完了 → /cmd {np.median(pi_part) * 1000:.1f}ms、/cmd → STM32 受理 {np.median(link) * 1000:.1f}ms")
    if not coded:
        res.warn("舵角にスキャン番号が刻まれていない（遅延の試験以外の記録？）。直前のスキャンに反応したとみなした")
    if med < 0 or med > 0.15:
        res.warn(f"遅れ {med * 1000:.0f}ms がありそうな範囲（0〜150ms）の外。時刻同期を確かめる")
    else:
        res.ok("遅れはありそうな範囲（0〜150ms）")
    res.values["dynamics.control_latency_s"] = round(med, 4)
    res.plot("遅れ [ms]（横軸: 反転の番号）", "遅れ", np.arange(len(tot)), tot * 1000)
    return res
