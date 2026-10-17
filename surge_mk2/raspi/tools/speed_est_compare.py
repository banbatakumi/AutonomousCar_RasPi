"""実機の記録で、車速の推定2通り（前輪の微分+LPF と カルマンフィルタ）を比べる。

    python3 -m raspi.tools.speed_est_compare ~/Downloads/surge_20261020_101500.mcap
    python3 -m raspi.tools.speed_est_compare run.mcap --tmin 30 --tmax 120   # 区間を絞る
    python3 -m raspi.tools.speed_est_compare run.mcap --export run.npz       # 小さな記録に抜く

## 何のためか

車速は走行距離と加速度計のカルマンフィルタ（`STM32 src/control/speed_kf.h`）で、プロトコル v0.25 で前輪の微分+LPF は廃止した。
- **v0.25 以降の記録**: `speed` が KF だけ。静止ノイズ・前輪の基準との差・`kf_gated`・加速度バイアス・テレメトリ間隔を見る
- **v0.24 の記録**: `speed_lpf`（従来の LPF）と `speed_kf` が両方あり、**同じ走行の中で並べて比べられる**（KF 導入時の比較に使った）

## 基準（真値の代わり）

車の真の速度は実機では測れない。**前輪の走行距離 `odom_dist`（累積。微分も LPF も通っていない）を
零位相で平滑して微分**したものを基準にする。零位相なので遅れが無い（推定器の遅れがそのまま見える）。
ただし:

- 基準も前輪から作るので、**前輪がロック・スリップすると基準も狂う**（推定器の誤差は測れない）
- 100Hz・0.1mm 刻みの距離から作るので、雑音の床は数 mm/s。**これより小さい差は見えない**
- STM32 の時計が約 0.35% 速いこと（`docs/development.md` の通信遅延の節）は、推定器の車速にも
  基準にも同じ倍率で出るので、**走行距離の積分の比で倍率を先に推定して取り除く**（倍率そのものも出す）

## 見るもの

- 静止中の雑音（推定器の標準偏差）
- 走行中の誤差（RMSE と最大）を、加減速の強い区間・定速の区間に分けて
- 遅れ（基準に最も合うずらし量）
- KF の `kf_gated`（測定を弱めた回数）と `accel_bias_kf` の動き
- `speed_source` が切り替わっていれば、区間ごとの TC の介入（A/B）
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

__all__ = ["analyze", "format_report", "load_mcap", "reference_speed"]

#: 零位相平滑の幅（サンプル数。100Hz なら 1.5 = 15ms）。小さいほど加速度の山が残り、大きいほど雑音が減る
REF_SIGMA_SAMPLES = 1.5
#: 静止とみなす基準速度 [m/s]（これ未満が 1 秒以上続いた区間）
STILL_SPEED = 0.02
STILL_MIN_S = 1.0
#: 加減速の強い区間・定速の区間 [m/s²]
HIGH_ACCEL = 1.5
STEADY_ACCEL = 0.3
STEADY_MIN_SPEED = 0.5

_FIELDS = ("speed", "speed_kf", "speed_lpf", "accel_kf", "accel_bias_kf", "kf_gated", "speed_source",
           "steer_actual", "odom_dist", "wheel_speed", "tc_slip", "tc_limit_nm", "t_stm_us", "tc_active",
           "accel", "armed")


def load_mcap(path: str | Path, tmin: float | None = None, tmax: float | None = None) -> dict[str, np.ndarray]:
    """`/vehicle_state` を配列にする。時刻は STM32 の時計（`t_stm_us`）を巻き戻りなしの秒にしたもの。"""
    from mcap.reader import make_reader

    rows = []
    with open(path, "rb") as f:
        for _, ch, m in make_reader(f).iter_messages(topics=["/vehicle_state"]):
            rows.append(json.loads(m.data))
    if not rows:
        raise SystemExit("記録に /vehicle_state がありません")
    if "accel_kf" not in rows[0]:
        raise SystemExit("この記録はプロトコル v0.23 以前（KF の診断が無い）。v0.24 以降のファームで取り直してください")
    d: dict[str, np.ndarray] = {}
    for k in _FIELDS:
        d[k] = np.array([r.get(k, 0) for r in rows], dtype=float)
    if "speed_kf" not in rows[0]:         # v0.25 以降: speed が KF の車速（LPF は廃止）
        d["speed_kf"] = d["speed"].copy()
        d.pop("speed_lpf")
    # t_stm_us は u32 で 71 分で巻き戻る
    t = d["t_stm_us"].copy()
    wraps = np.cumsum(np.r_[0, np.diff(t) < -2**31])
    d["t"] = (t + wraps * 2**32) * 1e-6
    d["t"] -= d["t"][0]
    d["t_host"] = np.array([r.get("t_capture", 0) for r in rows], dtype=float) * 1e-9
    keep = np.ones(len(d["t"]), bool)
    if tmin is not None:
        keep &= d["t"] >= tmin
    if tmax is not None:
        keep &= d["t"] <= tmax
    return {k: v[keep] for k, v in d.items()}


def _gauss_smooth(x: np.ndarray, sigma: float) -> np.ndarray:
    n = int(6 * sigma) | 1
    k = np.exp(-0.5 * ((np.arange(n) - n // 2) / sigma) ** 2)
    k /= k.sum()
    pad = n // 2
    xp = np.r_[np.full(pad, x[0]), x, np.full(pad, x[-1])]
    return np.convolve(xp, k, "valid")


def reference_speed(odom_dist: np.ndarray, steer: np.ndarray, t: np.ndarray,
                    sigma_samples: float = REF_SIGMA_SAMPLES) -> np.ndarray:
    """前輪の走行距離から、遅れの無い基準速度を作る。

    `odom_dist` は [FL, FR] の累積で射影なし。左右平均の増分に cos(舵角) を掛けて車体前後方向へ直して
    積算し、零位相で平滑して微分する。
    """
    d = 0.5 * (odom_dist[:, 0] + odom_dist[:, 1])
    inc = np.diff(d) * np.cos(steer[1:])
    s = np.r_[0.0, np.cumsum(inc)]
    s_sm = _gauss_smooth(s, sigma_samples)
    return np.gradient(s_sm, t)


def _stats(e: np.ndarray) -> tuple[float, float]:
    if len(e) == 0:
        return float("nan"), float("nan")
    return float(np.sqrt(np.mean(e**2))), float(np.max(np.abs(e)))


def _best_shift(est: np.ndarray, ref: np.ndarray, mask: np.ndarray, max_shift: int = 8) -> int:
    """est を何サンプル前へずらすと ref に最も合うか（正 = est が遅れている）。"""
    best, best_e = 0, float("inf")
    for L in range(-max_shift, max_shift + 1):
        e = np.mean((np.roll(est, -L) - ref)[mask] ** 2)
        if e < best_e:
            best, best_e = L, e
    return best


def _runs(mask: np.ndarray) -> list[tuple[int, int]]:
    """真の区間 [始め, 終わり) の一覧。"""
    idx = np.flatnonzero(np.diff(np.r_[0, mask.astype(int), 0]))
    return list(zip(idx[0::2], idx[1::2]))


def analyze(d: dict[str, np.ndarray]) -> dict:
    """`load_mcap` の配列から指標を出す。"""
    t = d["t"]
    n = len(t)
    dt = float(np.median(np.diff(t))) if n > 1 else 0.01
    ref_raw = reference_speed(d["odom_dist"], d["steer_actual"], t)
    r: dict = {"n": n, "duration_s": float(t[-1] - t[0]), "dt_ms": dt * 1e3}
    # STM32 の制御ループの余裕: ループが間に合わないとテレメトリ（100Hz）の間隔が乱れる。KF を足した後に見る
    gaps = np.diff(t) * 1e3
    r["tlm_interval_ms"] = {"p50": float(np.percentile(gaps, 50)), "p99": float(np.percentile(gaps, 99)),
                            "max": float(gaps.max()), "late_over_15ms": int((gaps > 15.0).sum())}

    # 比べる推定器: v0.24 の記録は LPF と KF、v0.25 以降は KF だけ
    est = {"LPF": d["speed_lpf"]} if "speed_lpf" in d else {}
    est["KF"] = d["speed_kf"]
    base = d["speed_lpf"] if "speed_lpf" in d else d["speed_kf"]
    # 倍率（時計・半径の誤差。推定器と基準に共通）: **走行距離の積分の比**で求める。遅れのある推定器は
    # 加減速で速度の比がずれるが、面積は変わらない（速度の比の中央値だと、減速中心の記録で遅れが倍率に
    # 混ざり、基準が狂う）。基準を従来のLPFの尺度に合わせ、KF の倍率は別に出す（KF だけ違えば誤差に出る）
    # 記録の端 0.5s は外す（基準の平滑・推定器の立ち上がりが端で合わない。記録が走行中から始まる場合に効く）
    # **走行中のサンプルだけ**で積分する: 静止中の雑音の絶対値（LPF 2mm/s なら 44 秒の静止で 0.1m）が
    # 面積に足されて、短い走行の記録では倍率が 2% 以上狂う（実機の記録で確認）
    inner = np.zeros(n, bool)
    inner[max(5, int(0.5 / dt)): max(6, n - int(0.5 / dt))] = True
    inner &= np.abs(ref_raw) > 0.05
    area_ref = float(np.sum(np.abs(ref_raw[inner])))
    scale = float(np.sum(np.abs(base[inner])) / area_ref) if area_ref > 1e-6 else 1.0
    r["scale_lpf_over_ref"] = scale          # v0.25 以降は KF の倍率
    r["scale_kf_over_ref"] = float(np.sum(np.abs(d["speed_kf"][inner])) / area_ref) if area_ref > 1e-6 else 1.0
    ref = ref_raw * scale     # 基準を推定器の尺度に合わせる（共通の倍率を取り除く）

    r["ref_peak_mps"] = float(np.max(np.abs(ref)))

    # 区間の分類
    a_ref = np.gradient(_gauss_smooth(ref, 3.0), t)
    edge = np.zeros(n, bool)
    edge[: max(5, int(0.3 / dt))] = True
    edge[-max(5, int(0.3 / dt)):] = True
    still_raw = np.abs(ref) < STILL_SPEED
    still = np.zeros(n, bool)
    for a, b in _runs(still_raw):
        if (b - a) * dt >= STILL_MIN_S:
            still[a + int(0.2 / dt): b - int(0.1 / dt)] = True
    moving_m = (np.abs(ref) >= STILL_SPEED) & ~edge
    high = moving_m & (np.abs(a_ref) > HIGH_ACCEL)
    steady = moving_m & (np.abs(a_ref) < STEADY_ACCEL) & (np.abs(ref) > STEADY_MIN_SPEED)
    r["still_s"] = float(still.sum() * dt)
    r["moving_s"] = float(moving_m.sum() * dt)
    r["high_accel_s"] = float(high.sum() * dt)
    r["steady_s"] = float(steady.sum() * dt)

    for name, x in est.items():
        e = x - ref
        out = {
            "still_std_mmps": 1e3 * float(np.std(x[still])) if still.sum() > 10 else float("nan"),
            "all_rmse_mmps": 1e3 * _stats(e[moving_m])[0],
            "all_peak_mmps": 1e3 * _stats(e[moving_m])[1],
            "high_rmse_mmps": 1e3 * _stats(e[high])[0],
            "high_peak_mmps": 1e3 * _stats(e[high])[1],
            "steady_rmse_mmps": 1e3 * _stats(e[steady])[0],
            "lag_ms": _best_shift(x, ref, moving_m) * dt * 1e3,
        }
        r[name] = out

    # KF の状態
    gated = d["kf_gated"]
    dg = np.diff(gated) % 65536                       # 16bit で巻き戻る
    r["kf_gated_total"] = int(dg.sum())
    r["kf_gated_per_min"] = float(dg.sum() / max(t[-1] - t[0], 1e-9) * 60.0)
    tc = d["tc_active"] > 0
    r["kf_gated_while_tc_per_min"] = (float(dg[tc[1:]].sum() / max(tc.sum() * dt, 1e-9) * 60.0)
                                      if tc.sum() > 10 else float("nan"))
    b = d["accel_bias_kf"]
    r["accel_bias"] = {"start": float(b[:20].mean()), "end": float(b[-20:].mean()),
                       "min": float(b.min()), "max": float(b.max()),
                       "moving_std": float(np.std(b[moving_m])) if moving_m.sum() > 20 else float("nan")}

    # speed_source ごとの TC（切り替えがあれば A/B）
    src = d["speed_source"].astype(int)
    r["by_source"] = {}
    for s in sorted(set(src.tolist())):
        m = src == s
        if m.sum() < 50:
            continue
        runs = _runs(tc & m)
        slip = np.abs(d["tc_slip"][m & ~tc]).max(axis=1) if (m & ~tc).any() else np.array([0.0])
        mv = m & moving_m
        r["by_source"][s] = {
            "seconds": float(m.sum() * dt),
            "tc_active_ratio": float((tc & m).sum() / m.sum()),
            "tc_events_per_min": float(len(runs) / max(m.sum() * dt, 1e-9) * 60.0),
            "tc_event_mean_ms": float(np.mean([(b - a) * dt * 1e3 for a, b in runs])) if runs else 0.0,
            "slip_abs_p95_when_not_tc": float(np.percentile(slip, 95)),
            "speed_rmse_vs_ref_mmps": 1e3 * _stats((d["speed"] - ref)[mv])[0] if mv.sum() > 20 else float("nan"),
        }
    return r


def format_report(r: dict) -> str:
    L = []
    L.append(f"記録 {r['duration_s']:.1f}s ({r['n']} 点, {r['dt_ms']:.1f}ms 刻み)  基準速度の最大 {r['ref_peak_mps']:.2f} m/s")
    L.append(f"  静止 {r['still_s']:.1f}s / 走行 {r['moving_s']:.1f}s（うち加減速の強い区間 {r['high_accel_s']:.1f}s、定速 {r['steady_s']:.1f}s）")
    if "LPF" in r:
        L.append(f"  基準との倍率: LPF {r['scale_lpf_over_ref']:.4f}  KF {r['scale_kf_over_ref']:.4f}"
                 "  （STM32 の時計の誤差。共通の倍率は取り除いて比べている。LPF と KF の差が大きければ別の原因）")
    else:
        L.append(f"  基準との倍率: KF {r['scale_kf_over_ref']:.4f}  （1.000 からのずれ＝前輪の半径・STM32 の時計の誤差。"
                 "v0.25 の `dt` 修正の後は 1.000±0.003 のはず）")
    g = r["tlm_interval_ms"]
    L.append(f"  STM32 のテレメトリ間隔 [ms]: 中央値 {g['p50']:.2f} / 99% {g['p99']:.2f} / 最大 {g['max']:.1f} / 15ms 超 {g['late_over_15ms']} 回"
             "（制御ループが間に合わないと乱れる）")
    L.append("")
    L.append(f"{'':8s}{'静止std':>9s}{'走行RMSE':>10s}{'最大':>8s}{'強加減速RMSE':>13s}{'最大':>8s}{'定速RMSE':>10s}{'遅れ':>8s}   [mm/s, ms]")
    for name in [n for n in ("LPF", "KF") if n in r]:
        s = r[name]
        L.append(f"{name:8s}{s['still_std_mmps']:9.2f}{s['all_rmse_mmps']:10.1f}{s['all_peak_mmps']:8.0f}"
                 f"{s['high_rmse_mmps']:13.1f}{s['high_peak_mmps']:8.0f}{s['steady_rmse_mmps']:10.1f}{s['lag_ms']:8.1f}")
    L.append("")
    b = r["accel_bias"]
    L.append(f"KF: 測定を弱めた回数 {r['kf_gated_total']}（{r['kf_gated_per_min']:.1f}/分、TC 中 {r['kf_gated_while_tc_per_min']:.1f}/分）"
             f"  加速度バイアス {b['start']:.2f}→{b['end']:.2f}（範囲 {b['min']:.2f}〜{b['max']:.2f}、走行中の標準偏差 {b['moving_std']:.3f}）")
    if r["by_source"]:
        L.append("")
        L.append("制御に使った車速（speed_source）ごと:")
        L.append(f"  {'source':>6s}{'秒':>8s}{'TC介入率':>10s}{'介入/分':>9s}{'平均ms':>8s}{'非介入の|slip|95%':>18s}{'speedのRMSE':>13s}")
        for s, v in r["by_source"].items():
            L.append(f"  {('LPF' if s == 0 else 'KF'):>6s}{v['seconds']:8.1f}{v['tc_active_ratio']:10.3f}{v['tc_events_per_min']:9.1f}"
                     f"{v['tc_event_mean_ms']:8.1f}{v['slip_abs_p95_when_not_tc']:18.3f}{v['speed_rmse_vs_ref_mmps']:13.1f}")
        if len(r["by_source"]) > 1:
            L.append("  （走る場面が違うと比べられない。同じコース・同じ速度設定の周回で見ること）")
    return "\n".join(L)


def _main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("path", help=".mcap か、--export で抜いた .npz")
    ap.add_argument("--tmin", type=float, help="解析する区間の始め [s]（STM32 の時計）")
    ap.add_argument("--tmax", type=float, help="同 終わり [s]")
    ap.add_argument("--export", help="読み出した配列を .npz に書く")
    a = ap.parse_args(argv)
    if a.path.endswith(".npz"):
        z = np.load(a.path)
        d = {k: z[k] for k in z.files}
        keep = np.ones(len(d["t"]), bool)
        if a.tmin is not None:
            keep &= d["t"] >= a.tmin
        if a.tmax is not None:
            keep &= d["t"] <= a.tmax
        d = {k: v[keep] for k, v in d.items()}
    else:
        d = load_mcap(a.path, a.tmin, a.tmax)
    if a.export:
        np.savez_compressed(a.export, **d)
        print(f"書き出し: {a.export}")
    print(format_report(analyze(d)))
    return 0


if __name__ == "__main__":
    sys.exit(_main(sys.argv[1:]))
