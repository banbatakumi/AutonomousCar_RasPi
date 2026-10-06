"""制御ロジックの比較ベンチ — 2つのファームを同じ車両モデル・同じ場面で走らせて並べる。

    .venv/bin/python -m tools.ctrl_tune.bench                 # コミット済み(HEAD) と 作業ツリー
    .venv/bin/python -m tools.ctrl_tune.bench --ref v0.15     # 比べる相手のコミット・タグ
    .venv/bin/python -m tools.ctrl_tune.bench --variants      # 車ごとの内訳も出す

作業ツリー側のパラメータは `config/vehicle.toml` の `[control]`（無ければファームの既定値）。
比べる相手が調整パラメータを持たない古いファームなら、その定数のまま走る。

ロジックを変えるときは、**変える前にここで今の値を控え、変えた後にもう一度回して**比べること
（`tools/sysid/bench.py` と同じ位置づけ: 検証器を先に持つ）。

## 読み方

`scenarios.py` の物差し。各欄は「あり得る車」（`plant.variants()`）での平均で、括弧は最悪の車。
効率は一定トルクの最良に対する比（駆動は進んだ距離、制動は止まった距離の逆比）。
"""

from __future__ import annotations

import argparse

import numpy as np

from . import scenarios as S
from .fw import Firmware
from .plant import Plant, variants
from .tuning import load_params

__all__ = ["compare", "main"]

#: 効率を出す場面（一定トルクの最良が定義できる＝路面が途中で変わっても同じ条件で比べられる）
_ORACLE = frozenset({"launch", "roll", "accel_mu_drop", "corner_exit", "brake", "brake_soft",
                     "brake_mu_drop"})


def compare(fws: dict[str, tuple[Firmware, dict[str, float] | None]], plant: Plant,
            scenarios: tuple[S.Scenario, ...] = S.SLIP, seeds: int = 1):
    """ファームの名前 → 場面 → 車 → 物差し。"""
    out: dict[str, dict[str, dict[str, dict[str, float]]]] = {name: {} for name in fws}
    vs = variants(plant, seeds)
    any_fw = next(iter(fws.values()))[0]
    for sc in scenarios:
        orc = {vn: S.oracle(any_fw, v, sc) for vn, v in vs} if sc.key in _ORACLE else {}
        for name, (fw, params) in fws.items():
            row = {}
            for vn, v in vs:
                m = S.run(fw, v, sc, params).metrics
                if vn in orc and m["distance"] > 0:
                    m["efficiency"] = orc[vn] / m["distance"] if sc.kind == "brake" else m["distance"] / orc[vn]
                row[vn] = m
            out[name][sc.key] = row
    return out


def _cell(rows: dict[str, dict[str, float]], key: str, worst_high: bool) -> str:
    vals = [m[key] for m in rows.values() if key in m]
    if not vals:
        return "      -      "
    worst = max(vals) if worst_high else min(vals)
    return f"{np.mean(vals):5.2f} ({worst:5.2f})"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--ref", default="HEAD", help="比べる相手のコミット（既定 HEAD）")
    ap.add_argument("--seeds", type=int, default=1)
    ap.add_argument("--variants", action="store_true", help="車ごとの内訳も出す")
    args = ap.parse_args(argv)

    plant = Plant.load()
    new = Firmware()
    fws = {f"{args.ref}": (Firmware(args.ref), None), "作業ツリー": (new, load_params(new))}
    scs = S.SLIP
    res = compare(fws, plant, scs, args.seeds)
    cols = (("効率", "efficiency", False), ("滑っていた割合", "slipping", True),
            ("横グリップの残り", "lateral_keep", False), ("トルクの暴れ", "chatter", True),
            ("後輪の最高周速", "wheel_peak", True), ("ヨーレート偏差", "yaw_err_rms", True))
    print("各欄: 車の組での平均（最悪の車）\n")
    for sc in scs:
        print(f"■ {sc.label}  [{sc.key}]")
        print("  " + " " * 12 + "".join(f"{c[0]:>14s}" for c in cols))
        for name in fws:
            print(f"  {name:12s}" + "".join(" " + _cell(res[name][sc.key], c[1], c[2]) for c in cols))
        if args.variants:
            for vn in next(iter(res.values()))[sc.key]:
                for name in fws:
                    m = res[name][sc.key][vn]
                    print(f"      {vn:16s} {name:10s} 距離 {m['distance']:.3f}m 効率 "
                          f"{m.get('efficiency', float('nan')):.2f} 滑り {m['slipping']:.2f} "
                          f"ピーク {m['slip_peak']:.2f} 介入 TC{m['tc_active']:.2f} ABS{m['abs_active']:.2f}")
        print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
