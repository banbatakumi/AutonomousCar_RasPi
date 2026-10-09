"""システム同定の解析の手順（どの試験の記録から何を、どの順で求めるか）。

`tools/sysid/gui.py`（画面）と `tools/sysid/tests/`（mcap→解析→`vehicle.toml`→シムの
再現までの通し確認）が同じ手順を使うよう、画面から切り離してここに置く。
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np

from raspi.core.control_params import STEER_LINK_X_MAX_RAD, steer_link_road
from sim.vehicle import VehicleSpec

from . import fit, toml_update

__all__ = ["TESTS", "GEOMETRY", "LINK_KEYS", "Analysis", "analyze", "fold_steer_link"]

#: 舵の効きを畳み込んだ結果のキー（`[control]` に書く。必ず2つ一緒に適用する）
LINK_KEYS = toml_update.LINK_KEYS

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


def _md_dropout_text(drops: list[tuple[float, float, str]]) -> str:
    names = "・".join(dict.fromkeys(n for _, _, n in drops))
    total = sum(b - a for a, b, _ in drops)
    return (f"★{names}のモータドライバが試験中に無応答になった（{drops[0][0]:.1f}s から、計 {total:.1f}s）。"
            "この記録は使えない——駆動バッテリーを充電し、MD の電源を確かめて録り直してください")


def fold_steer_link(link: tuple[float, float], gain: float, cubic: float) -> tuple[float, float]:
    """記録したときのリンクの換算 `link` に、その記録から求めた舵の効き（`gain`・`cubic`）を重ねて、
    新しいリンクの換算 `(steer_link_gain, steer_link_cubic)` を返す。

    記録の舵角は `d = link(x)`（x = モータ角×リンク比）、同定が見つけた実際の向きは
    `gain·d + cubic·d³`。合わせた `x → 実際の向き` を可動範囲で `g·x + c·x³` に当てはめ直す
    （3次式どうしの合成は9次になるので最小二乗。`link` が換算なしなら `(gain, cubic)` そのもの）。
    STM32 がこれを使えば、報告する舵角は実際の向きになり、舵の効きは 1 と 0 に戻る。
    """
    x = np.linspace(-STEER_LINK_X_MAX_RAD, STEER_LINK_X_MAX_RAD, 241)
    d = steer_link_road(x, *link)
    y = gain * d + cubic * d ** 3
    (g, c), *_ = np.linalg.lstsq(np.column_stack([x, x ** 3]), y, rcond=None)
    return float(g), float(c)


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


def analyze(sources: dict[str, str | Path | fit.Log], base: VehicleSpec,
            fold_link: bool = True) -> Analysis:
    """`sources` は試験キー（`TESTS` の内部キー）→ mcap のパス（または読み込み済みの `Log`）。

    `base` は今の `vehicle.toml`。前の試験で求めた値で順に上書きしながら次の試験を解析する。

    `fold_link`: 舵の効き（`steer_gain`・`steer_gain_cubic`）を、STM32 のリンクの換算
    （`LINK_KEYS`）へ畳み込んで返す（★プロトコル v0.18。`fold_steer_link`）。結果には `steer_gain`・
    `steer_gain_cubic` の代わりに `LINK_KEYS` が入り、`toml_update.apply_dynamics` がそれを `[control]` に
    書いて `[dynamics]` の2つを 1 と 0 に戻す。False は当てはめの結果そのまま（解析の検証用）。
    """
    out = Analysis()
    logs: dict[str, fit.Log] = {}
    for key, label, params in TESTS:
        src = sources.get(key)
        if src is None:
            continue
        try:
            lg = src if isinstance(src, fit.Log) else fit.load_log(src)
            if lg.md_dropouts:
                # 止まった MD は駆動も制動もせず、車輪の値は古いまま固まる。その記録から読んだ値は
                # 車の特性ではない（2026-10-08、右後輪の MD が落ちた記録で制動の頭打ちが 2.75→1.91）
                raise ValueError(_md_dropout_text(lg.md_dropouts))
            logs[key] = lg
            r = _FNS[key](lg, base)
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
            notes = list(r.notes)
            if fold_link and ("steer_gain" in vals or "steer_gain_cubic" in vals):
                links = {lg.steer_link for lg in geo_logs.values()}
                if len(links) > 1:
                    raise ValueError("記録によって STM32 のステアのリンクの換算が違います（" + "、".join(
                        f"gain {g:.4f}・cubic {c:.4f}" for g, c in sorted(links))
                        + "）。同じ設定で録った記録だけを選んでください")
                link = links.pop()
                gain = vals.pop("steer_gain", base.steer_gain)
                cubic = vals.pop("steer_gain_cubic", base.steer_gain_cubic)
                g, c = fold_steer_link(link, gain, cubic)
                vals.update(zip(LINK_KEYS, (g, c)))
                x = STEER_LINK_X_MAX_RAD
                notes.append(
                    f"舵の効き（倍率 {gain:.4f}・3次 {cubic:+.4f}）を STM32 のリンクの換算へ畳み込んだ: "
                    f"記録時 gain {link[0]:.4f}・cubic {link[1]:+.4f} → gain {g:.4f}・cubic {c:+.4f}"
                    f"（最大舵角 {np.degrees(steer_link_road(x, *link)):.2f}° → "
                    f"{np.degrees(steer_link_road(x, g, c)):.2f}°）。適用すると [dynamics] の steer_gain・"
                    "steer_gain_cubic は 1 と 0、max_steer はこの最大舵角になる")
            out.results.update(vals)
            out.notes[geo_label] = notes
            if r.warnings:
                out.warned.update({k: geo_label for k in vals})
        except Exception as e:  # noqa: BLE001
            out.errors.append(f"{geo_label}: {e}")
    return out
