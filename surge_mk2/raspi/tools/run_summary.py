"""実機の走行記録（.mcap）を 1 枚のサマリにまとめる。前回の走行との差分も出せる。

    python3 -m raspi.tools.run_summary ~/Downloads/surge_20261027_101210.mcap
    python3 -m raspi.tools.run_summary run.mcap --out-dir ~/runs            # 結果を保存する
    python3 -m raspi.tools.run_summary run.mcap --prev ~/runs/old_summary.json

GUI は `tools/run_summary_gui.py`（launcher の「走行サマリ」）。

## 何のためか

走行のたびに PROGRESS.md へ手で書いていた集計（周回タイム・電圧・衝突・学習値・前兆）を、
毎回同じ観点で出す。**自動化の目的は、観点がぶれないことと、前回との差が一目で分かること。**

## 入力が読むトピック

`/vehicle_state`（100Hz）・`/auto/state`・`/auto/ctrl`・`/diag/link`。画像と点群は読まない
（154MB の記録でも 1 秒前後）。**無いトピックの項目は None（表示は「不明」）**にして、
古い記録でも落とさない。

## 検出の根拠（実機の記録で確かめた値）

- **衝突**: 縦加速度 |ax| ≥ 8 m/s²。壁に当たった 20261026_104056 は最大 15.6、正常な走行は
  どの記録も最大 5.8（ABS の制動が 3.6 前後、コーナーの横加速度は ax に出ない）。
  6〜8 は「接触の疑い」。**横加速度では見ない**（コーナリングだけで |ay| が 8 を超える）。
- **前兆の回数**: `grip.events` は区間の中で積み上がる値なので、増えた分を足す（ゼロへ戻るのは区間の切れ目）。
- **周回タイム**: `auto/state.laps` が RACE の間に 1 増えた時刻の差。最初の 1 周は出走（発進を含む）
  なので別に出し、最良・平均には入れない。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Callable

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

SCHEMA = 1

#: 縦加速度の大きさ [m/s²]。根拠は上の docstring
CRASH_AX = 8.0
TOUCH_AX = 6.0
#: 同じ事象とみなす間隔 [s]
EVENT_GAP_S = 0.5
#: 横偏差・自己位置の「要確認」のしきい値
CROSS_WARN_M = 0.25
#: 駆動電池が低いとみなす電圧 [V]（8C Ni-MH。2026-10-26 は 7.75V で警告が立った）
DRIVE_LOW_V = 7.75
#: これ未満の電圧は読み取り値として扱わない [V]
VALID_V = 3.0
#: 駆動電源が入ってから電圧を信用するまでの時間 [s]
POWER_SETTLE_S = 2.0

#: 比較表に出す設定（auto/ctrl.params）。変わったものは別に全部出す
KEY_PARAMS = ("v_max", "line_margin", "kappa_use", "grip_learn", "grip_init", "grip_max",
              "ctrl_mode", "speed_ff", "speed_fb", "launch")

TOPICS = ("/vehicle_state", "/auto/state", "/auto/ctrl", "/diag/link")


# ── 読み込み ──

def _read(path: Path, progress: Callable[[float], None] | None) -> dict[str, Any]:
    from mcap.reader import make_reader

    with open(path, "rb") as f:
        reader = make_reader(f)
        meta: dict[str, str] = {}
        for md in reader.iter_metadata():
            if md.name == "surge":
                meta = dict(md.metadata)
        summ = reader.get_summary()
        t_start = summ.statistics.message_start_time if summ and summ.statistics else 0
        t_end = summ.statistics.message_end_time if summ and summ.statistics else 0
        t0 = int(meta["t0_mono_ns"]) if "t0_mono_ns" in meta else None
        out: dict[str, list] = {t: [] for t in TOPICS}
        span = max(1, t_end - t_start)
        n = 0
        for _schema, ch, msg in reader.iter_messages(topics=list(TOPICS)):
            out[ch.topic].append(json.loads(msg.data))
            n += 1
            if progress is not None and n % 2000 == 0:
                progress(min(1.0, (msg.log_time - t_start) / span))
    return {"meta": meta, "t0": t0, "msgs": out, "unix_span_s": (t_end - t_start) / 1e9,
            "unix_start_ns": t_start}


def _t(d: dict, t0: int | None) -> float:
    """記録の最初（logger_node の起動）からの秒。メタデータが無い記録は単調時刻のまま。"""
    return (d["t_capture"] - (t0 or 0)) / 1e9


def _stats(a: list[float] | np.ndarray) -> dict[str, float] | None:
    if len(a) == 0:
        return None
    x = np.asarray(a, dtype=float)
    return {"min": float(x.min()), "max": float(x.max()), "mean": float(x.mean()),
            "std": float(x.std()), "p95": float(np.percentile(x, 95))}


def _cluster(times: list[float], values: list[float]) -> list[tuple[float, float]]:
    """近い時刻の事象を 1 つにまとめる（時刻は最初、値は絶対値の最大）。"""
    out: list[list[float]] = []
    for t, v in zip(times, values):
        if out and t - out[-1][2] <= EVENT_GAP_S:
            out[-1][1] = max(out[-1][1], abs(v))
            out[-1][2] = t
        else:
            out.append([t, abs(v), t])
    return [(a, b) for a, b, _ in out]


# ── 集計 ──

def _phases(auto: list[dict], t0) -> list[dict]:
    """phase の連続区間。区間ごとの長さ。"""
    segs: list[dict] = []
    for d in auto:
        ph = d.get("phase") or ""
        t = _t(d, t0)
        if segs and segs[-1]["phase"] == ph:
            segs[-1]["t_end"] = t
        else:
            segs.append({"phase": ph, "t_start": t, "t_end": t})
    return segs


def _grip_env(g: dict) -> dict:
    """横の上限と前後の包絡。**古い記録は包絡が無く `acc`・`dec` 1 つだけ**なので u=0 の点として読む。"""
    out = {k: g.get(k) for k in ("lat", "acc0", "acc1", "acc2", "dec0", "dec1", "dec2")}
    if out["acc0"] is None:
        out["acc0"] = g.get("acc")
    if out["dec0"] is None:
        out["dec0"] = g.get("dec")
    return out


def _laps(auto: list[dict], t0) -> dict[str, Any]:
    """RACE の区間ごとに、周回の通過時刻から周回タイムと、周ごとの状態を出す。"""
    runs: list[dict] = []
    cur: dict | None = None
    prev_laps = None
    prev_events = 0.0
    for d in auto:
        t = _t(d, t0)
        race = d.get("phase") == "RACE"
        if not race:
            cur = None
            prev_laps = None
            continue
        if cur is None:
            cur = {"t_start": t, "laps": [], "precursors": 0, "cross": [], "_last": None}
            runs.append(cur)
            prev_laps = d.get("laps", 0)
            prev_events = 0.0
        g = d.get("grip") or {}
        ev = g.get("events")
        if ev is not None:
            if ev > prev_events:
                cur["precursors"] += int(round(ev - prev_events))
            prev_events = ev
        if d.get("cross_track") is not None:
            cur["cross"].append(abs(d["cross_track"]))
        laps = d.get("laps", 0)
        if laps > (prev_laps or 0):
            cur["laps"].append({"n": laps, "t": t, "grip": _grip_env(g)})
            prev_laps = laps
        cur["t_end"] = t
    result_runs = []
    all_times: list[float] = []
    for r in runs:
        laps = r["laps"]
        rows = []
        for i, lp in enumerate(laps):
            if i == 0:
                rows.append({"n": lp["n"], "t_cross": lp["t"], "time": None, "grip": lp["grip"]})
                continue
            tm = lp["t"] - laps[i - 1]["t"]
            rows.append({"n": lp["n"], "t_cross": lp["t"], "time": tm, "grip": lp["grip"]})
            all_times.append(tm)
        times = [x["time"] for x in rows if x["time"] is not None]
        result_runs.append({
            "t_start": r["t_start"], "t_end": r.get("t_end", r["t_start"]),
            "out_lap_s": (laps[0]["t"] - r["t_start"]) if laps else None,
            "laps": rows, "times": _stats(times), "precursors": r["precursors"],
            "cross_max": max(r["cross"]) if r["cross"] else None,
        })
    return {"runs": result_runs, "times": _stats(all_times), "count": len(all_times)}


def _vstat(t: np.ndarray, v: np.ndarray, race: np.ndarray) -> dict[str, Any]:
    return {"start": float(v[0]), "min": float(v.min()), "end": float(v[-1]),
            "t_min": float(t[v.argmin()]),
            "min_race": float(v[race].min()) if race.any() else None}


def _vehicle(vs: list[dict], t0, auto_race: list[tuple[float, float]]) -> dict[str, Any]:
    if not vs:
        return {}
    t = np.array([_t(d, t0) for d in vs])
    ax = np.array([d["accel"][0] for d in vs])
    speed = np.array([d["speed"] for d in vs])
    bv = np.array([d["batt_voltage"] for d in vs])
    # 駆動側は、駆動電源が入った直後は電池に繋がり切っておらず低い値が数百 ms 出る（20261026 は 3V 台）。
    # 入ってから POWER_SETTLE_S 経った後だけ見る。フィールドの無い記録は VALID_V 以上のものだけ見る
    on = np.array([bool(d.get("drive_power_on", True)) for d in vs])
    since = np.full(len(vs), np.inf)
    t_on = None
    for i in range(len(vs)):
        if on[i]:
            if t_on is None:
                t_on = t[i]
            since[i] = t[i] - t_on
        else:
            t_on = None
    on = on & (since >= POWER_SETTLE_S) & (bv[:, 0] >= VALID_V)
    bc = np.array([d["batt_current"] for d in vs])
    # 温度は未接続のセンサが None で来る。無いサンプルは 0 として最大には効かせない
    temp = np.array([max((x for x in (d.get("temp") or []) if x is not None), default=0)
                     for d in vs], dtype=float)

    def in_race(tt: np.ndarray) -> np.ndarray:
        m = np.zeros(len(tt), dtype=bool)
        for a, b in auto_race:
            m |= (tt >= a) & (tt <= b)
        return m

    race = in_race(t)
    big = np.nonzero(np.abs(ax) >= TOUCH_AX)[0]
    ev = _cluster(list(t[big]), list(ax[big]))
    crashes = [{"t": a, "ax": b, "kind": "衝突" if b >= CRASH_AX else "接触の疑い"} for a, b in ev]

    faults: dict[str, dict] = {}
    for d, tt in zip(vs, t):
        for f in d.get("faults") or []:
            e = faults.setdefault(f, {"count": 0, "t_first": float(tt)})
            e["count"] += 1
            e["t_last"] = float(tt)

    def edges(key: str) -> list[float]:
        out, prev = [], False
        for d, tt in zip(vs, t):
            cur = bool(d.get(key))
            if cur and not prev:
                out.append(float(tt))
            prev = cur
        return out

    dt = np.diff(t) if len(t) > 1 else np.array([0.0])
    return {
        "samples": len(vs),
        "rate_hz": float(1.0 / np.median(dt)) if len(dt) and np.median(dt) > 0 else None,
        "gap_max_s": float(dt.max()) if len(dt) else None,
        "crashes": crashes,
        "ax_max_abs": float(np.abs(ax).max()),
        "speed_max": float(speed.max()),
        "speed_race": _stats(speed[race]) if race.any() else None,
        "drive_v": (_vstat(t[on], bv[on, 0], race[on]) if on.any() else None),
        "signal_v": _vstat(t, bv[:, 1], race),
        "drive_a_max": float(bc[:, 0].max()),
        "signal_a_max": float(bc[:, 1].max()),
        "temp_max": float(temp.max()),
        "faults": faults,
        "estop_edges": edges("estop_active"),
        "uart_timeout_edges": edges("uart_timeout"),
        "auto_stop_edges": edges("auto_stop_active"),
        "abs_edges": len(edges("abs_active")),
        "tc_edges": len(edges("tc_active")),
        "armed_edges": len(edges("armed")),
        "t_first": float(t[0]), "t_last": float(t[-1]),
    }


def _locate(auto: list[dict], min_score: float, t0) -> dict[str, Any]:
    sc = [(_t(d, t0), d["match_score"]) for d in auto
          if d.get("phase") == "RACE" and d.get("match_score") is not None]
    if not sc:
        return {}
    low, prev = 0, False
    for _, s in sc:
        cur = s < min_score
        low += int(cur and not prev)
        prev = cur
    arr = np.array([s for _, s in sc])
    return {"score_min": float(arr.min()), "score_mean": float(arr.mean()),
            "low_fraction": float((arr < min_score).mean()), "lost_count": low,
            "min_score": min_score}


def _learning(auto: list[dict]) -> dict[str, Any]:
    gs = [d["grip"] for d in auto if d.get("grip")]
    if not gs:
        return {}
    first, last = _grip_env(gs[0]), _grip_env(gs[-1])
    return {"first": first, "last": last,
            "lat_max": max((g.get("lat") or 0.0) for g in gs),
            "reason_last": next((d.get("reason") for d in reversed(auto) if d.get("reason")), None)}


def _link(link: list[dict]) -> dict[str, Any]:
    if not link:
        return {}
    last = link[-1]
    return {"health_last": last.get("health"),
            "health_bad": sum(1 for d in link if d.get("health") != "OK"),
            "rx": last.get("rx"), "md_rx_error": last.get("md_rx_error"),
            "cmd_stale_count": sum(1 for d in link if d.get("cmd_stale")),
            "arm_inhibited_count": sum(1 for d in link if d.get("arm_inhibited")),
            "control_params": link[0].get("control_params"),
            "tc": link[0].get("tc_enabled"), "tv": link[0].get("tv_enabled"),
            "abs": link[0].get("abs_enabled")}


def summarize(path: str | Path, *, progress: Callable[[float], None] | None = None) -> dict[str, Any]:
    path = Path(path)
    raw = _read(path, progress)
    t0, m = raw["t0"], raw["msgs"]
    auto, vs = m["/auto/state"], m["/vehicle_state"]
    ctrl, link = m["/auto/ctrl"], m["/diag/link"]

    params = (ctrl[0].get("params") if ctrl else None) or {}
    phases = _phases(auto, t0)
    race_spans = [(p["t_start"], p["t_end"]) for p in phases if p["phase"] == "RACE"]
    min_score = float(params.get("min_score", 0.35))
    local_start = raw["meta"].get("t0_local")

    s: dict[str, Any] = {
        "schema": SCHEMA,
        "file": path.name,
        "size_mb": round(path.stat().st_size / 1e6, 1),
        "start_local": local_start,
        "start_sort": local_start or path.stem,
        "duration_s": raw["unix_span_s"],
        "mode": ctrl[0].get("mode") if ctrl else (auto[0].get("mode") if auto else None),
        "map": next((d.get("race_map") for d in ctrl if d.get("race_map")), None),
        "params": params,
        "params_changes": sum(1 for a, b in zip(ctrl, ctrl[1:]) if a.get("params") != b.get("params")),
        "phases": phases,
        "laps": _laps(auto, t0),
        "vehicle": _vehicle(vs, t0, race_spans),
        "locate": _locate(auto, min_score, t0),
        "learning": _learning(auto),
        "link": _link(link),
        "missing": [t for t in TOPICS if not m[t]],
    }
    lp = s["laps"]
    lp["cross_max"] = max((r["cross_max"] for r in lp["runs"] if r["cross_max"] is not None),
                          default=None)
    lp["precursors"] = sum(r["precursors"] for r in lp["runs"])
    s["findings"] = _findings(s)
    return s


def _findings(s: dict) -> list[str]:
    """「要確認」の一覧。**重い順**。"""
    out: list[str] = []
    v = s["vehicle"]
    for c in v.get("crashes", []):
        mark = "🔴" if c["kind"] == "衝突" else "🟡"
        out.append(f"{mark} {c['kind']}: {c['t']:.1f}s・縦加速度 {c['ax']:.1f}m/s²")
    for name, e in (v.get("faults") or {}).items():
        out.append(f"🟡 フォールト {name}: {e['count']}サンプル（{e['t_first']:.1f}s〜{e['t_last']:.1f}s）")
    if v.get("estop_edges"):
        out.append(f"🔴 E-Stop {len(v['estop_edges'])}回（{', '.join(f'{x:.1f}s' for x in v['estop_edges'][:5])}）")
    if v.get("uart_timeout_edges"):
        out.append(f"🟡 UART タイムアウト {len(v['uart_timeout_edges'])}回")
    dv = v.get("drive_v")
    if dv and dv["min"] < DRIVE_LOW_V:
        out.append(f"🔴 駆動電圧が {dv['min']:.2f}V まで低下（{dv['t_min']:.1f}s）")
    lo = s["locate"]
    if lo and lo["lost_count"]:
        out.append(f"🟡 RACE 中に自己位置の一致度が {lo['min_score']} を割った: {lo['lost_count']}回"
                   f"（最小 {lo['score_min']:.2f}）")
    cm = s["laps"].get("cross_max")
    if cm is not None and cm > CROSS_WARN_M:
        out.append(f"🟡 横偏差が最大 {cm * 100:.0f}cm")
    gap = v.get("gap_max_s")
    if gap is not None and gap > 0.2:
        out.append(f"🟡 車両状態の欠落: 最大 {gap * 1000:.0f}ms")
    lk = s["link"]
    if lk.get("rx") and (lk["rx"].get("crc_error") or lk["rx"].get("packet_loss")):
        out.append(f"🟡 通信エラー: CRC {lk['rx'].get('crc_error')}・欠落 {lk['rx'].get('packet_loss')}")
    if lk.get("health_bad"):
        out.append(f"🟡 リンクの健全性が OK でない: {lk['health_bad']}サンプル")
    if s["missing"]:
        out.append(f"ℹ️ 無いトピック: {', '.join(s['missing'])}（その項目は不明）")
    if not out:
        out.append("✅ 要確認の事象なし")
    return out


# ── 比較 ──

def _get(d: dict, path: str):
    for k in path.split("."):
        if d is None:
            return None
        d = d.get(k) if isinstance(d, dict) else None
    return d


#: (表示名, JSON のパス, 小さいほど良いか None=良し悪しなし, 書式)
COMPARE_ROWS: list[tuple[str, str, bool | None, str]] = [
    ("周回数（最初の1周を除く）", "laps.count", None, "{:.0f}"),
    ("最良ラップ [s]", "laps.times.min", True, "{:.2f}"),
    ("平均ラップ [s]", "laps.times.mean", True, "{:.2f}"),
    ("ラップのばらつき(σ) [s]", "laps.times.std", True, "{:.3f}"),
    ("前兆の回数", "laps.precursors", True, "{:.0f}"),
    ("横偏差の最大 [m]", "laps.cross_max", True, "{:.3f}"),
    ("最高速度 [m/s]", "vehicle.speed_max", None, "{:.2f}"),
    ("RACE 中の平均速度 [m/s]", "vehicle.speed_race.mean", False, "{:.2f}"),
    ("縦加速度の最大 [m/s²]", "vehicle.ax_max_abs", True, "{:.1f}"),
    ("駆動電圧の最小 [V]", "vehicle.drive_v.min", False, "{:.2f}"),
    ("制御電圧の最小 [V]", "vehicle.signal_v.min", False, "{:.2f}"),
    ("駆動電流の最大 [A]", "vehicle.drive_a_max", True, "{:.2f}"),
    ("温度の最大 [℃]", "vehicle.temp_max", True, "{:.0f}"),
    ("一致度の最小", "locate.score_min", False, "{:.2f}"),
    ("見失い回数", "locate.lost_count", True, "{:.0f}"),
    ("学習: 横 [m/s²]（終了時）", "learning.last.lat", None, "{:.2f}"),
    ("学習: 加速 u=0（終了時）", "learning.last.acc0", None, "{:.2f}"),
    ("学習: 減速 u=0（終了時）", "learning.last.dec0", None, "{:.2f}"),
]


def compare(cur: dict, prev: dict) -> dict[str, Any]:
    rows = []
    for label, path, lower_better, fmt in COMPARE_ROWS:
        a, b = _get(prev, path), _get(cur, path)
        if a is None and b is None:
            continue
        delta = (b - a) if a is not None and b is not None else None
        verdict = ""
        if delta is not None and lower_better is not None and abs(delta) > 1e-9:
            better = (delta < 0) == lower_better
            verdict = "改善" if better else "悪化"
        rows.append({"label": label, "prev": a, "cur": b, "delta": delta, "fmt": fmt,
                     "verdict": verdict})
    pa, pb = prev.get("params") or {}, cur.get("params") or {}
    changed = {k: (pa.get(k), pb.get(k)) for k in sorted(set(pa) | set(pb)) if pa.get(k) != pb.get(k)}
    ca = _get(prev, "link.control_params") or {}
    cb = _get(cur, "link.control_params") or {}
    fw = {k: (ca.get(k), cb.get(k)) for k in sorted(set(ca) | set(cb))
          if not _close(ca.get(k), cb.get(k))}
    return {"prev_file": prev.get("file"), "rows": rows, "params": changed, "fw_params": fw,
            "map_changed": prev.get("map") != cur.get("map")}


def _close(a, b) -> bool:
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return math.isclose(a, b, rel_tol=1e-6, abs_tol=1e-9)
    return a == b


def find_previous(directory: Path, cur: dict) -> Path | None:
    """保存先の `*_summary.json` のうち、`cur` より前に始まった走行で最も新しいもの。"""
    best: tuple[str, Path] | None = None
    for p in directory.glob("*_summary.json"):
        try:
            d = json.loads(p.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        key = d.get("start_sort") or ""
        if d.get("file") == cur["file"] or key >= cur["start_sort"]:
            continue
        if best is None or key > best[0]:
            best = (key, p)
    return best[1] if best else None


# ── 出力 ──

def _f(x, fmt="{:.2f}", none="不明") -> str:
    return none if x is None else fmt.format(x)


def render_markdown(s: dict, cmp: dict | None = None) -> str:
    v, lp, lo, lr, lk = s["vehicle"], s["laps"], s["locate"], s["learning"], s["link"]
    L: list[str] = []
    L.append(f"# 走行サマリ {s['file']}")
    L.append("")
    L.append(f"- 開始 {s.get('start_local') or '不明'}・長さ {_f(s['duration_s'], '{:.0f}')}s・"
             f"{s['size_mb']}MB")
    L.append(f"- planner `{s.get('mode') or '不明'}`・地図 `{s.get('map') or '不明'}`"
             f"・走行中の設定変更 {s['params_changes']}回")
    ph = " → ".join(f"{p['phase'] or '(なし)'} {p['t_end'] - p['t_start']:.0f}s" for p in s["phases"]
                    if p["t_end"] - p["t_start"] >= 1.0)
    L.append(f"- 段階: {ph or '不明'}")
    L.append("")
    L.append("## 要確認")
    L += [f"- {x}" for x in s["findings"]]
    L.append("")

    L.append("## 周回")
    if lp["count"]:
        t = lp["times"]
        L.append(f"- {lp['count']}周（最初の1周を除く）・最良 **{t['min']:.2f}s**・平均 {t['mean']:.2f}s・"
                 f"σ {t['std']:.3f}s・最悪 {t['max']:.2f}s")
    else:
        L.append("- 周回の記録なし")
    for i, r in enumerate(lp["runs"]):
        L.append("")
        L.append(f"### RACE {i + 1}（{r['t_start']:.0f}s〜{r['t_end']:.0f}s）"
                 f"・前兆 {r['precursors']}回・横偏差の最大 {_f(r['cross_max'], '{:.3f}')}m"
                 f"・出走 {_f(r['out_lap_s'], '{:.2f}')}s")
        if r["laps"]:
            L.append("")
            L.append("| 周 | ラップ[s] | 横 | 加速 u0/u.5/u1 | 減速 u0/u.5/u1 |")
            L.append("|---:|---:|---:|---|---|")
            for row in r["laps"]:
                g = row["grip"]
                L.append(f"| {row['n']} | {_f(row['time'])} | {_f(g.get('lat'))} | "
                         f"{_f(g.get('acc0'))}/{_f(g.get('acc1'))}/{_f(g.get('acc2'))} | "
                         f"{_f(g.get('dec0'))}/{_f(g.get('dec1'))}/{_f(g.get('dec2'))} |")
    L.append("")

    L.append("## 車両")
    if v:
        sr = v.get("speed_race")
        L.append(f"- 車両状態 {v['samples']}件（{_f(v['rate_hz'], '{:.0f}')}Hz・最大の欠落 "
                 f"{_f((v['gap_max_s'] or 0) * 1000, '{:.0f}')}ms）")
        L.append(f"- 速度: 最高 {v['speed_max']:.2f}m/s・RACE 平均 {_f(sr and sr['mean'])}m/s")
        d, g = v["drive_v"], v["signal_v"]
        if d:
            L.append(f"- 駆動電圧（駆動電源が入っている間）: 開始 {d['start']:.2f}V → 最小 {d['min']:.2f}V"
                     f"（{d['t_min']:.0f}s）→ 終了 {d['end']:.2f}V・電流の最大 {v['drive_a_max']:.2f}A")
        else:
            L.append("- 駆動電圧: 駆動電源が入った記録なし")
        L.append(f"- 制御電圧: 開始 {g['start']:.2f}V → 最小 {g['min']:.2f}V・電流の最大 {v['signal_a_max']:.2f}A")
        L.append(f"- 温度の最大 {v['temp_max']:.0f}℃・縦加速度の最大 {v['ax_max_abs']:.1f}m/s²・"
                 f"ABS 作動 {v['abs_edges']}回・TC 作動 {v['tc_edges']}回・ARM の立ち上がり {v['armed_edges']}回")
    else:
        L.append("- 車両状態の記録なし")
    L.append("")

    L.append("## 自己位置・学習・通信")
    if lo:
        L.append(f"- 自己位置（RACE）: 一致度 最小 {lo['score_min']:.2f}・平均 {lo['score_mean']:.2f}・"
                 f"見失い {lo['lost_count']}回（しきい値 {lo['min_score']}）")
    if lr:
        f_, l_ = lr["first"], lr["last"]
        L.append(f"- 学習（開始 → 終了）: 横 {_f(f_['lat'])} → {_f(l_['lat'])}・"
                 f"加速 {_f(f_['acc0'])} → {_f(l_['acc0'])}・減速 {_f(f_['dec0'])} → {_f(l_['dec0'])}"
                 f"・横の最大 {_f(lr['lat_max'])}")
        if lr.get("reason_last"):
            L.append(f"- 最後の状態表示: {lr['reason_last']}")
    if lk:
        rx = lk.get("rx") or {}
        L.append(f"- 通信: health {lk['health_last']}・CRC {rx.get('crc_error')}・欠落 {rx.get('packet_loss')}・"
                 f"再同期 {rx.get('resync_bytes')}・MD の受信エラー {lk.get('md_rx_error')}"
                 f"・TC {lk.get('tc')}・TV {lk.get('tv')}・ABS {lk.get('abs')}")
    L.append("")

    L.append("## 主な設定")
    p = s.get("params") or {}
    L.append(" ・ ".join(f"{k}={p[k]}" for k in KEY_PARAMS if k in p) or "不明")
    L.append("")

    if cmp:
        L.append(f"## 前回との比較（前回: {cmp['prev_file']}）")
        if cmp["map_changed"]:
            L.append("- ⚠️ 地図が違う: 周回タイムは単純に比べられない")
        L.append("")
        L.append("| 項目 | 前回 | 今回 | 差 | |")
        L.append("|---|---:|---:|---:|---|")
        for r in cmp["rows"]:
            fmt = r["fmt"]
            L.append(f"| {r['label']} | {_f(r['prev'], fmt)} | {_f(r['cur'], fmt)} | "
                     f"{_f(r['delta'], '{:+.3g}', '–')} | {r['verdict']} |")
        if cmp["params"]:
            L.append("")
            L.append("変更された設定（auto/ctrl）: " + "、".join(
                f"{k} {a}→{b}" for k, (a, b) in cmp["params"].items()))
        if cmp["fw_params"]:
            L.append("")
            L.append("変更されたファーム側の制御値: " + "、".join(
                f"{k} {_f(a, '{:.4g}')}→{_f(b, '{:.4g}')}" for k, (a, b) in cmp["fw_params"].items()))
        L.append("")
    return "\n".join(L)


def save(s: dict, md: str, out_dir: Path) -> tuple[Path, Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = Path(s["file"]).stem
    jp, mp = out_dir / f"{stem}_summary.json", out_dir / f"{stem}_summary.md"
    jp.write_text(json.dumps(s, ensure_ascii=False, indent=1), encoding="utf-8")
    mp.write_text(md, encoding="utf-8")
    return jp, mp


def load_summary(path: Path) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def run(src: Path, *, out_dir: Path | None, prev: Path | None = None,
        progress: Callable[[float], None] | None = None) -> tuple[dict, str, list[Path]]:
    """記録 → (サマリ, Markdown, 保存したファイル)。`prev` が無ければ `out_dir` から探す。"""
    s = summarize(src, progress=progress)
    prev_s = None
    if prev is not None:
        prev_s = load_summary(prev)
    elif out_dir is not None and out_dir.is_dir():
        found = find_previous(out_dir, s)
        prev_s = load_summary(found) if found else None
    md = render_markdown(s, compare(s, prev_s) if prev_s else None)
    saved = list(save(s, md, out_dir)) if out_dir is not None else []
    return s, md, saved


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mcap", type=Path)
    ap.add_argument("--out-dir", type=Path, default=None, help="保存先（無ければ保存しない）")
    ap.add_argument("--prev", type=Path, default=None,
                    help="比較元の *_summary.json（無ければ --out-dir から直前の走行を探す）")
    a = ap.parse_args()
    _, md, saved = run(a.mcap, out_dir=a.out_dir, prev=a.prev)
    print(md)
    for p in saved:
        print(f"保存: {p}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
