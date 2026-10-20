"""車両の校正（設計は `docs/calibration.md`）。記録（mcap）→ 解析 → `config/vehicle.toml` へ書く値。

    .venv/bin/python -m tools.calib 記録.mcap [記録2.mcap …] [--apply]
    .venv/bin/python -m tools.calib.gui            # GUI（launcher の「車両の校正」）

どの解析を使うかは記録の中の planner の id（`/auto/state` の `mode`）で決める。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

__all__ = ["CALIBRATIONS", "Calibration", "analyze_file", "calibration_for"]


@dataclass(frozen=True)
class Calibration:
    order: str            #: 表示の番号（"1"・"5 確かめる" など）
    title: str
    planners: tuple[str, ...]   #: この解析が読める planner の id（旧 planner の記録も読めるものは並べる）
    analyze: Callable
    #: 先に適用しておくべき校正（表示用）
    needs: str = ""
    #: **録る前に** STM32 に入っていなければならない `[control]`・`[dynamics]` のキー（記録の `/diag/link` と今の toml を
    #: 突き合わせる。`prereq_check`）
    record_keys: tuple[str, ...] = ()


def _enc(log):
    from . import encoder
    return encoder.analyze(log)


def _steer(log):
    from . import steer
    return steer.analyze_static(log)


def _steer_dyn(log):
    from . import steer
    return steer.analyze_dynamic(log)


def _latency(log):
    from . import latency
    return latency.analyze(log)


def _speed(log):
    from . import speed
    return speed.analyze(log)


def _speed_check(log):
    from . import speed_check
    return speed_check.analyze(log)


def _wheel(log):
    from . import wheel
    return wheel.analyze(log)


def _tyre(log):
    from . import tyre
    return tyre.analyze(log)


def _tc_gain(log):
    from . import tc_gain
    return tc_gain.analyze(log)


def _traction(log):
    from . import traction
    return traction.analyze(log)


#: エンコーダの校正（校正 1）。ほかの試験の車速・走行距離・スリップ率はすべてこの上に乗る
_ENC = tuple(f"control.enc_{s}_{k}" for s in ("left", "right") for k in ("lo", "hi", "h1s", "h1c", "h2s", "h2c"))
_SPEED = ("dynamics.speed_kp", "dynamics.speed_ki", "control.speed_ff_nm_per_m_s2", "control.speed_ff_rr_nm")
_SLIP_TARGETS = ("control.tc_slip_target", "control.abs_slip_target")
_TRACTION = _SLIP_TARGETS + ("control.tc_kp_nm_per_m_s", "control.tc_ki_nm_per_m",
                             "control.abs_kp_nm_per_m_s", "control.abs_ki_nm_per_m")

CALIBRATIONS: tuple[Calibration, ...] = (
    Calibration("1", "前輪エンコーダ", ("calib_encoder", "sysid_encoder"), _enc),
    Calibration("2", "ステア（舵角→曲率）", ("calib_steer", "sysid_steer"), _steer, "1", _ENC),
    Calibration("3", "ステアの遅れ", ("calib_steer_dyn",), _steer_dyn, "1・2", _ENC),
    Calibration("4", "遅延", ("calib_latency", "sysid_latency"), _latency),
    Calibration("5", "車速制御（測る）", ("calib_speed", "sysid_speed_pi"), _speed, "1", _ENC),
    Calibration("5", "車速制御（確かめる）", ("calib_speed_check",), _speed_check, "5（測る）を適用", _ENC + _SPEED),
    Calibration("6", "後輪の空転（台上）", ("calib_wheel", "sysid_wheel"), _wheel),
    Calibration("7", "タイヤ→TC・ABS", ("calib_tyre",), _tyre, "1 を適用して録る・5・6 を適用して解析", _ENC),
    Calibration("8", "TC・ABS のゲイン", ("calib_tc_gain",), _tc_gain, "5・6・7 を適用", _ENC + _SLIP_TARGETS),
    Calibration("9", "TC・ABS の確かめ", ("calib_traction_check",), _traction, "7・8 を適用", _ENC + _TRACTION),
)


def prereq_check(cal: Calibration, log, res, toml_path=None) -> None:
    """録ったときに STM32 に入っていた値（`/diag/link`）が今の toml と同じかを確かめ、違えば★を足す。

    ほかの校正を適用する前に録った記録（例: エンコーダの校正を入れ直す前）は、その上に乗った値が古い。"""
    if not cal.record_keys:
        return
    from . import toml_io
    cur = toml_io.read_flat(toml_path or toml_io.DEFAULT_TOML)
    cp = log.control_params
    if not cp:
        res.warn("記録に STM32 の調整値（/diag/link）が無く、録ったときの校正を確かめられない")
        return
    diff = []
    for k in cal.record_keys:
        name = k.split(".", 1)[1]
        if name not in cp or k not in cur:
            continue
        if abs(float(cp[name]) - cur[k]) > 1e-5 * max(1.0, abs(cur[k])):
            diff.append(f"{name}（記録 {float(cp[name]):.6g}・今 {cur[k]:.6g}）")
    if diff:
        enc = any(d.startswith("enc_") for d in diff)
        what = "エンコーダの校正（校正 1）" if enc else "確かめる値"
        res.warn(f"録ったときの STM32 の{what}が今の vehicle.toml と違う: {'、'.join(diff[:3])}"
                 f"{' ほか' if len(diff) > 3 else ''}。適用して `--restart-io` してから録り直す")
    else:
        res.ok("録ったときの STM32 の値は今の vehicle.toml と同じ（前の校正を入れてから録った記録）")


def calibration_for(mode: str) -> Calibration | None:
    for c in CALIBRATIONS:
        if mode in c.planners:
            return c
    return None


def analyze_file(path: str):
    """記録を読み、対応する校正の解析を返す `(Calibration, Result)`。分からない記録は ValueError。"""
    from .log import load_mcap

    log = load_mcap(path)
    cal = calibration_for(log.mode)
    if cal is None:
        raise ValueError(f"校正の試験の記録ではない（planner: {log.mode or '不明'}）")
    res = cal.analyze(log)
    prereq_check(cal, log, res)
    return cal, res
