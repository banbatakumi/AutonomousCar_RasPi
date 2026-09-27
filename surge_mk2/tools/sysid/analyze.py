"""システム同定の解析の手順（どの試験の記録から何を、どの順で求めるか）。

`tools/sysid/gui.py`（画面）と `tools/sysid/tests/`（mcap→解析→`vehicle.toml`→シムの
再現までの通し確認）が同じ手順を使うよう、画面から切り離してここに置く。
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from pathlib import Path

from sim.vehicle import VehicleSpec

from . import fit

__all__ = ["TESTS", "GEOMETRY", "Analysis", "analyze"]

#: (内部キー, 表示名, この記録から求めるパラメータ)。**並び順が解析の順序**——
#: 後ろの試験は前の試験の結果を使う。舵の効き・中立ずれ・アンダーステア勾配・リンクのガタ・
#: ヨーの遅れは1つの試験では決まらないので、最後にステア・前後運動・旋回の記録をまとめて
#: 当てはめる（`GEOMETRY`）。2026-09-27 に速度応答試験と加減速試験を前後運動試験（加速度を
#: 段階的に上げる）1つにまとめた
TESTS: tuple[tuple[str, str, tuple[str, ...]], ...] = (
    ("steer", "① ステア試験", ("tau_steer_s", "dead_time_s", "steer_rate_limit_rad_s",
                              "steer_servo_hysteresis_rad", "steer_servo_gain")),
    ("accel", "② 前後運動試験", ("speed_plant_gain", "rolling_resistance",
                               "drive_accel_m_s2", "drive_fade_speed_m_s", "drive_top_speed_m_s",
                               "brake_decel_m_s2", "brake_decel_per_nm", "speed_decel_m_s2")),
    ("corner", "③ 旋回グリップ試験", ("mu",)),
    ("latency", "④ 遅延試験", ("control_latency_s",)),
)
GEOMETRY = ("舵の効き（①②③をまとめて）",
            ("steer_gain", "steer_gain_cubic", "steer_offset_rad", "understeer_gradient",
             "steer_link_hysteresis_rad", "steer_link_deadband_rad",
             "yaw_rate_filter_s", "yaw_relaxation_m", "yaw_natural_freq_rad_s", "yaw_damping"),
            ("steer", "accel", "corner"))

_FNS = {
    "steer": lambda lg, b: fit.fit_steer(lg),
    "accel": lambda lg, b: fit.fit_accel(lg, b),
    "corner": fit.fit_corner,
    "latency": lambda lg, b: fit.fit_latency(lg),
}


@dataclass
class Analysis:
    #: `[dynamics]` のキー → 求めた値
    results: dict[str, float] = field(default_factory=dict)
    #: 表示名 → 所見
    notes: dict[str, list[str]] = field(default_factory=dict)
    #: 解析できなかった試験（表示名つきのエラー文）
    errors: list[str] = field(default_factory=list)
    #: ★（記録がシムのモデルの形から外れている、`FitResult.warnings`）の出た試験で求めたキー
    #: → その試験の表示名。GUI はこれを既定で適用しない
    warned: dict[str, str] = field(default_factory=dict)


def analyze(sources: dict[str, str | Path | fit.Log], base: VehicleSpec) -> Analysis:
    """`sources` は試験キー（`TESTS` の内部キー）→ mcap のパス（または読み込み済みの `Log`）。

    `base` は今の `vehicle.toml`。前の試験で求めた値で順に上書きしながら次の試験を解析する。
    """
    out = Analysis()
    logs: dict[str, fit.Log] = {}
    for key, label, params in TESTS:
        src = sources.get(key)
        if src is None:
            continue
        try:
            logs[key] = src if isinstance(src, fit.Log) else fit.load_log(src)
            r = _FNS[key](logs[key], base)
        except Exception as e:  # noqa: BLE001 - 画面にそのまま出す
            out.errors.append(f"{label}: {e}")
            continue
        vals = {k: v for k, v in r.values.items() if k in params}
        out.results.update(vals)
        out.notes[label] = r.notes
        for k in vals:
            if r.warnings:
                out.warned[k] = label
            else:
                out.warned.pop(k, None)       # 後の試験が★なしで求め直した
        base = replace(base, **vals)

    # 速度センサの自己点検（速度が大きく変わる記録で。値は変えない）と、前輪エンコーダの
    # 周期誤差（各記録を読むときに推定・補正した結果）
    if "accel" in logs:
        out.notes["速度センサの自己点検"] = fit.sensor_check(logs["accel"], base).notes
    for key, label, _ in TESTS:
        if key in logs and key != "accel" and logs[key].sensor_notes:
            out.notes.setdefault(label, [])
            out.notes[label] = out.notes[label] + logs[key].sensor_notes

    geo_label, geo_params, geo_src = GEOMETRY
    geo_logs = {k: logs[k] for k in geo_src if k in logs}
    if geo_logs:
        try:
            r = fit.fit_geometry(geo_logs, base)
            vals = {k: v for k, v in r.values.items() if k in geo_params}
            out.results.update(vals)
            out.notes[geo_label] = r.notes
            if r.warnings:
                out.warned.update({k: geo_label for k in vals})
        except Exception as e:  # noqa: BLE001
            out.errors.append(f"{geo_label}: {e}")
    return out
