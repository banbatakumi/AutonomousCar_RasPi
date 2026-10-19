"""手で押した実機の**生データ**（前輪の 2kHz の角速度＋IMU の 200Hz の加速度）に、いまの `speed_kf.c` をそのまま通し、
実機の一貫性の診断（NIS・相関係数 ρ）を出す。

    PYTHONPATH=. python3 -m tools.fwsim.kf_raw_replay ~/kf_study_2026-10-16/push3.bin
    PYTHONPATH=. python3 -m tools.fwsim.kf_raw_replay push3.bin --define SPEED_KF_SIGMA_S_M=0.0003f   # 調整値を変えて比べる

## 何のためか

TELEMETRY（100Hz）には 2kHz の生の測定が載らないので、記録から KF の入力は再現できない（`raspi/tools/kf_consistency.py` は
STM32 が集計した診断を読む）。この道具は、2026-10-16 の手押しで STM32 の RAM に 4 秒ためた生データ
（デバッグ用ファーム `debug/kf-raw-capture`、SWD で読み出し。形式は下の `load`）を使い、**調整値を変えたときの
NIS・ρ を実機のデータで再計算できる**。KF の C は `drive.c` と同じ呼び方（`Predict` → 距離 → 加速度）で動かす。

注意: 手で押したデータなので、**駆動電源の振動・路面の凹凸は含まない**（走行中の加速度の雑音は実機で 0.6m/s² 前後、
手押しは 0.2〜0.3）。前輪の絶対角は無いので周期誤差の位相は見えない。
"""

from __future__ import annotations

import argparse
import ctypes as C
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.tools.kf_consistency import analyze, firmware_constants, format_report  # noqa: E402
from tools.fwsim.fw import FW_DIR  # noqa: E402

R = 0.030                          # 前輪の半径 [m]（drive.h の DRIVE_FRONT_WHEEL_RADIUS_M）
DIR_L, DIR_R = -1.0, +1.0          # drive.h の DRIVE_FRONT_LEFT_DIR / RIGHT_DIR
WINDOW = 20                        # 診断を 10ms（= 20 ティック）の窓に畳む（TELEMETRY と同じ）


def load(path: str | Path) -> dict:
    """デバッグ用ファームの 4 秒キャプチャ（`g_kf_capture`）を読む。ティックごとの前輪角速度と IMU の生加速度。"""
    b = Path(path).read_bytes()
    _cmd, _status, cpu, nt, ni = struct.unpack_from("<5I", b, 0)
    ticks = np.frombuffer(b, dtype=np.dtype([("cyc", "<u4"), ("wl", "<i2"), ("wr", "<i2")]), count=8000, offset=28)[:nt]
    imu_t = np.dtype([("cyc", "<u4")] + [(k, "<f4") for k in ("ax", "ay", "az", "gz", "ax_lpf", "lin_ax", "pitch", "v_pid", "v_tc")])
    imus = np.frombuffer(b, dtype=imu_t, count=800, offset=28 + 8 * 8000)[:ni]
    t = (ticks["cyc"] - ticks["cyc"][0]).astype(np.float64) / (cpu * 1e6)
    ti = (imus["cyc"] - ticks["cyc"][0]).astype(np.int64).astype(np.float64) / (cpu * 1e6)
    return {"t": t, "wl": ticks["wl"] * 0.05, "wr": ticks["wr"] * 0.05, "ti": ti, "ax": imus["ax"].astype(float)}


class _KfConfig(C.Structure):
    """`speed_kf.h` の `SpeedKfConfig`（並びも同じ）。"""
    _fields_ = [("sigma_s_m", C.c_float), ("sigma_a_m_s2", C.c_float), ("sigma_a_per_m_s", C.c_float),
                ("jerk_psd", C.c_float), ("bias_walk_psd", C.c_float)]


class _Kf(C.Structure):
    _fields_ = [("cfg", _KfConfig), ("x", C.c_float * 4), ("p", C.c_float * 16), ("s_meas", C.c_float), ("initialized", C.c_bool),
                ("resync", C.c_bool), ("gated_count", C.c_uint32), ("diag", C.c_ubyte * 128)]


def build(defines: tuple[str, ...] = ()) -> C.CDLL:
    out = Path(tempfile.mkdtemp()) / "libspeedkf.so"
    subprocess.run(["cc", "-O2", "-shared", "-fPIC", f"-I{FW_DIR / 'src/control'}", str(FW_DIR / "src/control/speed_kf.c"),
                    *[f"-D{d}" for d in defines], "-o", str(out), "-lm"], check=True)
    lib = C.CDLL(str(out))
    p = C.POINTER(_Kf)
    for name, args in (("SpeedKf_Init", [p]), ("SpeedKf_Predict", [p, C.c_float]), ("SpeedKf_UpdateDistance", [p, C.c_float]),
                       ("SpeedKf_UpdateAccel", [p, C.c_float]),
                       ("SpeedKf_TakeDiag", [p, C.c_int, C.POINTER(C.c_float), C.POINTER(C.c_float)])):
        getattr(lib, name).argtypes = args
    return lib


def replay(lib: C.CDLL, d: dict) -> dict:
    """生データを KF に通し、`kf_consistency.analyze` に渡せる 10ms 窓の配列を返す。"""
    t = d["t"]
    dt = np.r_[5e-4, np.diff(t)]
    w = 0.5 * (d["wl"] * DIR_L + d["wr"] * DIR_R)                  # 前輪の角速度の左右平均 [rad/s]
    gap = w * dt * R                                                # 走行距離の増分 [m]（舵角 0）
    kf = _Kf()
    lib.SpeedKf_Init(C.byref(kf))
    j, n_imu = 0, len(d["ti"])
    rows = []
    for i in range(len(t)):
        lib.SpeedKf_Predict(C.byref(kf), float(dt[i]))
        lib.SpeedKf_UpdateDistance(C.byref(kf), float(gap[i]))
        while j < n_imu and d["ti"][j] <= t[i]:
            lib.SpeedKf_UpdateAccel(C.byref(kf), float(d["ax"][j]))
            j += 1
        if (i + 1) % WINDOW == 0:
            diag = []
            for ch in (0, 1):
                nis, ac = C.c_float(), C.c_float()
                lib.SpeedKf_TakeDiag(C.byref(kf), ch, C.byref(nis), C.byref(ac))
                diag += [nis.value, ac.value]
            rows.append((t[i], kf.x[1], kf.x[2], kf.gated_count, *diag))
    a = np.array(rows)
    return {"t": a[:, 0] - a[0, 0], "speed": np.abs(a[:, 1]), "accel_kf": a[:, 2], "accel_bias_kf": np.zeros(len(a)),
            "kf_gated": a[:, 3], "kf_nis_dist": a[:, 4], "kf_ac_dist": a[:, 5], "kf_nis_accel": a[:, 6], "kf_ac_accel": a[:, 7]}


def _main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("path", nargs="+", help="デバッグ用ファームのキャプチャ（push*.bin / static*.bin）")
    ap.add_argument("--define", action="append", default=[], help="調整値の上書き（例 SPEED_KF_SIGMA_S_M=0.0003f）")
    a = ap.parse_args(argv)
    lib = build(tuple(a.define))
    fw = firmware_constants()
    for path in a.path:
        r = replay(lib, load(path))
        print(f"===== {path}（最高 {r['speed'].max():.2f}m/s、|a| 最大 {np.abs(r['accel_kf']).max():.1f}m/s²）")
        print(format_report(analyze(r, fw.get("SIGMA_S_M"), fw.get("SIGMA_A_M_S2"))))
    return 0


if __name__ == "__main__":
    sys.exit(_main(sys.argv[1:]))
