"""`raspi/tools/run_summary.py` の検出と比較。実機の記録で確かめた値（衝突 |ax|≥8 など）を固定する。"""

from __future__ import annotations

import json
from pathlib import Path

from mcap.writer import Writer

from raspi.tools import run_summary as rs

T0 = 1_000_000_000


def _write(path: Path, vs: list[dict], auto: list[dict], ctrl: list[dict] | None = None,
           link: list[dict] | None = None) -> None:
    with open(path, "wb") as f:
        w = Writer(f)
        w.start()
        schema = w.register_schema(name="x", encoding="jsonschema", data=b"{}")
        chans = {}

        def put(topic: str, msgs: list[dict]) -> None:
            if not msgs:
                return
            cid = chans.setdefault(topic, w.register_channel(topic=topic, message_encoding="json",
                                                             schema_id=schema))
            for m in msgs:
                w.add_message(cid, log_time=m["t_capture"], publish_time=m["t_capture"],
                              data=json.dumps(m).encode())

        w.add_metadata("surge", {"t0_mono_ns": str(T0), "t0_local": "2026-10-27 10:00:00"})
        put("/vehicle_state", vs)
        put("/auto/state", auto)
        put("/auto/ctrl", ctrl or [])
        put("/diag/link", link or [])
        w.finish()


def _vs(t: float, ax: float = 0.0, v: float = 2.0, drive_v: float = 10.0, **kw) -> dict:
    return {"t_capture": T0 + int(t * 1e9), "speed": v, "accel": [ax, 0.0, 9.8],
            "batt_voltage": [drive_v, 9.5], "batt_current": [1.0, 0.5], "temp": [30, None],
            "faults": [], "drive_power_on": True, **kw}


def _auto(t: float, laps: int, phase: str = "RACE", events: float = 0.0, cross: float = 0.02) -> dict:
    return {"t_capture": T0 + int(t * 1e9), "phase": phase, "laps": laps, "cross_track": cross,
            "match_score": 1.0, "grip": {"lat": 5.0, "acc0": 2.9, "dec0": 3.0, "events": events}}


def _run(tmp_path: Path, name: str, ax_at: dict[float, float] | None = None,
         lap_period: float = 6.0) -> dict:
    ax_at = ax_at or {}
    vs = [_vs(i * 0.1, ax=ax_at.get(round(i * 0.1, 1), 0.0)) for i in range(0, 400)]
    auto = [_auto(i * 0.1, int(i * 0.1 // lap_period)) for i in range(0, 400)]
    p = tmp_path / f"{name}.mcap"
    _write(p, vs, auto, ctrl=[{"t_capture": T0, "mode": "slam2d_route", "race_map": "m",
                               "params": {"v_max": 4.0}}])
    return rs.summarize(p)


def test_lap_times_exclude_out_lap(tmp_path):
    s = _run(tmp_path, "a")
    lp = s["laps"]
    assert lp["count"] == 5            # 0→1 の通過は出走周で数えない
    assert abs(lp["times"]["mean"] - 6.0) < 0.15


def test_crash_threshold(tmp_path):
    clean = _run(tmp_path, "clean", {1.0: -5.8})          # 正常な制動の最大
    assert clean["vehicle"]["crashes"] == []
    crash = _run(tmp_path, "crash", {20.0: -15.6, 20.1: -12.0, 8.0: -6.4})
    kinds = [c["kind"] for c in crash["vehicle"]["crashes"]]
    assert kinds == ["接触の疑い", "衝突"]                    # 近い 2 サンプルは 1 事象
    assert any("衝突" in x for x in crash["findings"])


def test_precursor_counts_increments(tmp_path):
    auto = [_auto(i * 0.1, i // 60, events=float(min(i // 100, 3))) for i in range(300)]
    p = tmp_path / "p.mcap"
    _write(p, [_vs(i * 0.1) for i in range(300)], auto)
    assert rs.summarize(p)["laps"]["precursors"] == 2     # 0→1→2（i<300 なので 3 には届かない）


def test_drive_voltage_ignores_power_on_transient(tmp_path):
    vs = [_vs(i * 0.01, drive_v=3.0 if i < 100 else 9.0) for i in range(400)]   # 入って 1s は 3V
    p = tmp_path / "v.mcap"
    _write(p, vs, [_auto(0.0, 0)])
    assert rs.summarize(p)["vehicle"]["drive_v"]["min"] == 9.0


def test_missing_topics_are_none_not_error(tmp_path):
    p = tmp_path / "m.mcap"
    _write(p, [_vs(0.0), _vs(0.1)], [])
    s = rs.summarize(p)
    assert "/auto/state" in s["missing"] and s["laps"]["count"] == 0
    assert "不明" in rs.render_markdown(s) or "記録なし" in rs.render_markdown(s)


def test_compare_and_find_previous(tmp_path):
    a = _run(tmp_path, "run1", lap_period=7.0)
    b = _run(tmp_path, "run2", lap_period=6.0)
    a["start_sort"], b["start_sort"] = "2026-10-26", "2026-10-27"
    d = tmp_path / "out"
    rs.save(a, rs.render_markdown(a), d)
    assert rs.find_previous(d, b) == d / "run1_summary.json"
    assert rs.find_previous(d, a) is None                  # 自分より前が無い
    cmp = rs.compare(b, rs.load_summary(d / "run1_summary.json"))
    row = next(r for r in cmp["rows"] if r["label"] == "平均ラップ [s]")
    assert row["verdict"] == "改善" and row["delta"] < 0
    assert "前回との比較" in rs.render_markdown(b, cmp)
