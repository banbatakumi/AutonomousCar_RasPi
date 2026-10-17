"""車速カルマンフィルタ（STM32 `src/control/speed_kf.h`）の調整値が、実機の記録に対して妥当かを検定する。

    python3 -m raspi.tools.kf_consistency ~/Downloads/surge_20261020_101500.mcap
    python3 -m raspi.tools.kf_consistency run.mcap --tmin 5 --tmax 60

## 何を見るか（真値は要らない）

フィルタは測定のたびに「残差 ÷ 自分が主張する標準偏差」z を作る（STM32 が集計して TELEMETRY v0.26 で送る）。
調整値（測定ノイズ SIGMA_S・SIGMA_A、プロセスノイズ JERK_PSD・BIAS_WALK_PSD）が現実と合っていれば、z は
**平均 0・分散 1 の白色雑音**になる。これは真値の無い実機でも検定できる（カルマンフィルタの一貫性検定）。

- **NIS = 平均 z²**: 1 より大きければ現実の誤差が主張より大きい（測定ノイズ・プロセスノイズが小さすぎる）、
  小さければ大きく見積もりすぎ（保守的）。
- **自己相関 = 平均 z_k z_{k-1}**、**相関係数 ρ = 自己相関 ÷ NIS**: ρ が 0 でなければ残差が時間的に相関している。
  白色雑音の仮定が破れている＝**NIS だけで調整してはいけない**。前輪エンコーダの周期誤差（角度の関数）は
  ρ≈1（残差のほぼ全部が相関）になる。自己相関そのものは NIS が小さいと小さく見えるので、判定は ρ で行う。

走行距離（2kHz）と加速度（IMU の 200Hz）を別々に、区間（静止・低速・中速・高速・強い加減速）ごとに出す。
**NIS の信頼区間は、10ms ごとの窓の平均のばらつきから出す**（窓の中の相関は織り込む。窓をまたぐ相関は
見えないので、区間が短いと過信する）。

## 読み方の原則

1. ρ が大きい（信頼区間が 0 を外れ、かつ 0.5 超）なら、その測定の調整値は NIS では決められない。
   モデルに無い系統誤差があり、測定ノイズを膨らませて逃がしている（保守的で NIS<1 になる）状態。
2. 自己相関が小さいときだけ、NIS の大小は「R・Q のどちらか（または両方）を増やす／減らす」向きを示す。
   **大きさまでは言えない**: 残差の分散 S は「状態の不確かさ HPHᵀ」と「測定ノイズ R」の和で、σ だけを
   √NIS 倍にしても NIS は 1 にならない（シムで確かめた: SIGMA_A を 0.6→0.11 にして NIS 0.034→0.099）。
   値の決め直しは、ここで測った NIS・自己相関をシムの雑音モデルに合わせ込み、真値のあるシムで
   誤差を最小にする（`tools/ctrl_tune` の掃引、`docs/development.md` §4.8）。
3. 静止の区間は、センサ自身の白色雑音だけを見ている（振動・周期誤差は出ない）。
4. 静止の NIS が走行中よりずっと小さいのは普通（走行中は振動・周期誤差・加速度の変化が足される）。
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np

__all__ = ["analyze", "format_report", "load_mcap", "rows_to_arrays", "firmware_constants"]

#: 走行距離・加速度の測定が 1 窓（TELEMETRY 1 周期）に入る回数の目安 = 窓の長さ ÷ これ
DIST_PERIOD_S = 0.0005
ACCEL_PERIOD_S = 0.005
#: 残差が「強く相関している」と言える相関係数 ρ（信頼区間が 0 を外れることに加えてこれを超えること）。
#: 測定ノイズを大きく取りすぎると、動きへの追従遅れが残差に残って ρ は 0.3 前後まで出る（シムで確認）が、
#: 前輪の周期誤差があれば ρ≈1 になる。0.5 はその中間
RHO_STRONG = 0.5
#: NIS がこの範囲に信頼区間ごと入れば「整合」
NIS_OK = (0.7, 1.4)
#: 静止の判定
STILL_SPEED = 0.02
STILL_MIN_S = 1.0

_FIELDS = ("speed", "accel_kf", "accel_bias_kf", "kf_gated", "t_stm_us", "steer_actual",
           "kf_z_rms_dist", "kf_z_rms_accel", "kf_rho_dist", "kf_rho_accel")


def firmware_constants(fw_dir: Path | None = None) -> dict[str, float]:
    """ファームの `speed_kf.h` の既定値（SIGMA_S・SIGMA_A など）。読めなければ空。"""
    if fw_dir is None:
        from tools.ctrl_tune.fw import FW_DIR
        fw_dir = FW_DIR
    path = fw_dir / "src" / "control" / "speed_kf.h"
    if not path.exists():
        return {}
    text = path.read_text(encoding="utf-8")
    out = {}
    for name in ("SIGMA_S_M", "SIGMA_A_M_S2", "JERK_PSD", "BIAS_WALK_PSD", "GATE_SIGMA"):
        m = re.search(rf"#define\s+SPEED_KF_{name}\s+([0-9.eE+-]+)f?", text)
        if m:
            out[name] = float(m.group(1))
    return out


def rows_to_arrays(rows: list[dict]) -> dict[str, np.ndarray]:
    """`/vehicle_state` の行（JSON を読んだ dict）を配列にする。時刻 `t` は STM32 の時計を巻き戻りなしの秒にしたもの。"""
    if not rows:
        raise SystemExit("記録に /vehicle_state がありません")
    if "kf_z_rms_dist" not in rows[0]:
        raise SystemExit("この記録は一貫性の診断（プロトコル v0.26）が無い。v0.26 のファームで取り直してください")
    d = {k: np.array([r.get(k, 0) for r in rows], dtype=float) for k in _FIELDS}
    # STM32 は √NIS と相関係数で送る（NIS は静止で 1e-3 まで小さく、そのまま量子化すると潰れる）。窓ごとの NIS・自己相関に戻す
    for ch in ("dist", "accel"):
        nis = d[f"kf_z_rms_{ch}"] ** 2
        d[f"kf_nis_{ch}"] = nis
        d[f"kf_ac_{ch}"] = d[f"kf_rho_{ch}"] * nis
    t = d["t_stm_us"].copy()
    wraps = np.cumsum(np.r_[0, np.diff(t) < -2**31])        # u32 の us は 71 分で巻き戻る
    d["t"] = (t + wraps * 2**32) * 1e-6
    d["t"] -= d["t"][0]
    return d


def load_mcap(path: str | Path, tmin: float | None = None, tmax: float | None = None) -> dict[str, np.ndarray]:
    """`/vehicle_state` を配列にする。時刻 `t` は STM32 の時計を巻き戻りなしの秒にしたもの。"""
    from mcap.reader import make_reader

    rows = []
    with open(path, "rb") as f:
        for _, _ch, m in make_reader(f).iter_messages(topics=["/vehicle_state"]):
            rows.append(json.loads(m.data))
    d = rows_to_arrays(rows)
    keep = np.ones(len(d["t"]), bool)
    if tmin is not None:
        keep &= d["t"] >= tmin
    if tmax is not None:
        keep &= d["t"] <= tmax
    return {k: v[keep] for k, v in d.items()}


def _runs(mask: np.ndarray) -> list[tuple[int, int]]:
    idx = np.flatnonzero(np.diff(np.r_[0, mask.astype(int), 0]))
    return list(zip(idx[0::2], idx[1::2]))


def _weighted_mean(x: np.ndarray, w: np.ndarray) -> tuple[float, float, int]:
    """重み付き平均・標準誤差（窓の平均のばらつきから）・窓の数。"""
    n = len(x)
    if n < 2 or w.sum() <= 0:
        return float("nan"), float("nan"), n
    mean = float(np.sum(w * x) / np.sum(w))
    # 重みが一定に近いので、窓の平均の標準偏差 / √窓数 を標準誤差とする
    se = float(np.std(x, ddof=1) / np.sqrt(n))
    return mean, se, n


def _channel(nis: np.ndarray, ac: np.ndarray, valid: np.ndarray, w: np.ndarray) -> dict | None:
    """1 つの測定（走行距離か加速度）の、ある区間での統計と判定。"""
    m = valid
    if m.sum() < 30:
        return None
    nm, nse, n = _weighted_mean(nis[m], w[m])
    am, ase, _ = _weighted_mean(ac[m], w[m])
    out = {"windows": int(n), "nis": nm, "nis_ci": 1.96 * nse, "ac": am, "ac_ci": 1.96 * ase}
    rho = am / nm if nm > 0 else float("nan")
    rho_ci = 1.96 * ase / nm if nm > 0 else float("nan")
    out["rho"], out["rho_ci"] = rho, rho_ci
    correlated = (abs(rho) - rho_ci > 0) and abs(rho) > RHO_STRONG
    out["correlated"] = bool(correlated)
    if nm + 1.96 * nse < NIS_OK[0]:
        out["nis_verdict"] = "保守的（誤差を大きく見積もりすぎ）"
    elif nm - 1.96 * nse > NIS_OK[1]:
        out["nis_verdict"] = "過小見積もり（現実の誤差が主張より大きい）"
    else:
        out["nis_verdict"] = "整合"
    return out


def analyze(d: dict[str, np.ndarray], sigma_s: float | None = None, sigma_a: float | None = None) -> dict:
    """`load_mcap` の配列から区間ごとの一貫性の指標を出す。"""
    t = d["t"]
    n = len(t)
    if n < 50:
        raise ValueError("記録が短すぎる（50 点未満）")
    dt = np.diff(t, prepend=t[0] - float(np.median(np.diff(t))))
    dt = np.clip(dt, 0.0, 0.05)
    v = np.abs(d["speed"])
    acc = np.abs(d["accel_kf"])

    still = np.zeros(n, bool)
    for a, b in _runs(v < STILL_SPEED):
        if (t[b - 1] - t[a]) >= STILL_MIN_S:
            still[a + 10: b] = True
    moving = v >= STILL_SPEED
    segs = {
        "静止": still,
        "低速 0.02〜0.5m/s": moving & (v < 0.5),
        "中速 0.5〜1.5m/s": (v >= 0.5) & (v < 1.5),
        "高速 1.5m/s〜": v >= 1.5,
        "強い加減速 |a|>1.5m/s²": moving & (acc > 1.5),
        "全走行": moving,
    }
    res: dict = {"duration_s": float(t[-1] - t[0]), "n": n, "segments": {}, "sigma_s": sigma_s, "sigma_a": sigma_a}
    for name, mask in segs.items():
        seg = {"seconds": float(dt[mask].sum())}
        sel = mask & (d["kf_nis_dist"] > 0)
        seg["dist"] = _channel(d["kf_nis_dist"], d["kf_ac_dist"], sel,
                               dt / DIST_PERIOD_S)
        sel = mask & (d["kf_nis_accel"] > 0)
        seg["accel"] = _channel(d["kf_nis_accel"], d["kf_ac_accel"], sel, dt / ACCEL_PERIOD_S)
        res["segments"][name] = seg
    g = np.diff(d["kf_gated"]) % 65536                  # 16bit で巻き戻る
    res["gated_total"] = int(g.sum())
    res["gated_per_min"] = float(g.sum() / max(t[-1] - t[0], 1e-9) * 60.0)
    res["accel_bias_range"] = (float(d["accel_bias_kf"].min()), float(d["accel_bias_kf"].max()))
    return res


def _fmt(c: dict | None) -> list[str]:
    if c is None:
        return ["  （データ不足）"]
    s = (f"  NIS {c['nis']:.3g} ±{c['nis_ci']:.2g}   自己相関 {c['ac']:+.2g} ±{c['ac_ci']:.2g}  ρ {c['rho']:+.2f}   "
         f"[{c['windows']} 窓]  → {c['nis_verdict']}")
    out = [s]
    if c["correlated"]:
        out.append("    ★残差が時間的に相関している（白色雑音の仮定の破れ）。NIS だけでは調整できない"
                   "——周期誤差などの系統誤差を補正してから測定ノイズを決める")
    elif c["nis"] < NIS_OK[0]:
        out.append("    白色で NIS<1: 測定ノイズか状態の不確かさ（プロセスノイズ）が過大。性能を落としてでも保守的にしている状態")
    elif c["nis"] > NIS_OK[1]:
        out.append("    白色で NIS>1: 測定ノイズかプロセスノイズが過小（現実の誤差・変化に追従できていない）")
    return out


def format_report(r: dict) -> str:
    L = [f"記録 {r['duration_s']:.1f}s（{r['n']} 点）"]
    if r["sigma_s"]:
        L.append(f"  今の調整値: SIGMA_S {r['sigma_s'] * 1e3:.3g} mm / SIGMA_A {r['sigma_a']:.3g} m/s²"
                 if r["sigma_a"] else f"  今の調整値: SIGMA_S {r['sigma_s'] * 1e3:.3g} mm")
    L.append(f"  測定を弱めた回数 {r['gated_total']}（{r['gated_per_min']:.1f}/分）  "
             f"加速度バイアスの範囲 {r['accel_bias_range'][0]:.2f}〜{r['accel_bias_range'][1]:.2f} m/s²")
    for name, seg in r["segments"].items():
        if seg["seconds"] < 1.0:
            continue
        L.append("")
        L.append(f"■ {name}（{seg['seconds']:.1f}s）")
        L.append(" 走行距離（前輪エンコーダ）:")
        L += _fmt(seg["dist"])
        L.append(" 加速度（MPU6050）:")
        L += _fmt(seg["accel"])
    return "\n".join(L)


def _main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("path", help=".mcap")
    ap.add_argument("--tmin", type=float, help="解析する区間の始め [s]（STM32 の時計）")
    ap.add_argument("--tmax", type=float, help="同 終わり [s]")
    ap.add_argument("--sigma-s", type=float, help="走行距離の測定ノイズ [m]（省略はファームの speed_kf.h）")
    ap.add_argument("--sigma-a", type=float, help="加速度の測定ノイズ [m/s²]（同）")
    a = ap.parse_args(argv)
    fw = firmware_constants()
    d = load_mcap(a.path, a.tmin, a.tmax)
    print(format_report(analyze(d, a.sigma_s or fw.get("SIGMA_S_M"), a.sigma_a or fw.get("SIGMA_A_M_S2"))))
    return 0


if __name__ == "__main__":
    sys.exit(_main(sys.argv[1:]))
