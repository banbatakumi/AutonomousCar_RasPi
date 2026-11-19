"""`sim.bench --trace` の CSV の記録（RACE 中の 100Hz の真値・指令・学習の値）。

`sim/bench.py` から分けた。書く列と書式は分ける前と同じ:

- `<trace>`: RACE 中の 100Hz の行（`TraceWriter.row`）
- `<trace>.long.csv`: 加速・減速の学習（`nav/grip.LongLearner`）へ渡した値と、その瞬間の真値
- `<trace>.path.csv`: 終わったときに追っていた経路（`TraceWriter.close`）
"""

from __future__ import annotations

from pathlib import Path

__all__ = ["TraceWriter"]

_HEADER = ("t,phase,x,y,yaw,v,cmd_v,brake,prof_v,idx,cross,clear,launch,"
           "steer_cmd,steer,a_lat,a_x,kappa,slip,g_level,g_cross_gain,g_cap,g_gate\n")
_LONG_HEADER = ("t,v,a_plan,gate,v_plan,v_true,ax_true,grip_long,u_y,"
                "cmd_v,accel,decel\n")


def trace_grip(planner) -> str:
    """横グリップの学習の値（trace の最後の4列）。学習が無い・切ってあるなら 0。"""
    g = getattr(planner, "_grip", None)
    if g is None or not getattr(planner, "_grip_on", False):
        return "0,0,0,"
    return (f"{g.map.level:.3f},{g.last_cross_gain:.4f},"
            f"{g.cap if g.cap is not None else 0.0:.3f},{planner._grip_gate}")


class TraceWriter:
    """`--trace` のファイルを開いて持つ。`sim` は真値の読み出し元（`SimLink.sim`）。"""

    def __init__(self, path: Path, planner, sim) -> None:
        self.planner = planner
        self.sim = sim
        self._f = open(path, "w")
        self._f.write(_HEADER)
        self._long_f = None
        # 加速・減速の学習（`nav/grip.LongLearner`）へ渡した値と、その瞬間の真値（`<trace>.long.csv`）
        lg = getattr(getattr(planner, "_grip", None), "long", None)
        if lg is not None:
            self._long_f = open(str(path) + ".long.csv", "w")
            self._long_f.write(_LONG_HEADER)
            inner = lg.observe

            def observe(t, v, a_plan, gate, v_plan=None, *rest, _in=inner, _lg=lg, **kw):
                tv, sp = self.sim.vehicle, self.sim.spec
                u = tv.speed ** 2 * abs(tv._curv_req) / max(sp.mu * tv.grip_lat * 9.81, 1e-6)
                self._long_f.write(
                    f"{t:.4f},{v:.4f},{a_plan:.4f},{gate},"
                    f"{float('nan') if v_plan is None else v_plan:.4f},{tv.speed:.4f},"
                    f"{tv.accel_x:.4f},{tv.grip_long:.4f},{u:.4f},{tv.cmd.target_speed:.4f},"
                    f"{_lg.map.accel:.4f},{_lg.map.decel:.4f}\n")
                return _in(t, v, a_plan, gate, v_plan, *rest, **kw)
            lg.observe = observe

    @property
    def name(self) -> str:
        return self._f.name

    def row(self, t: float, st, v, cmd, clear: float) -> None:
        """RACE 中の 1 行（100Hz）。`v` は真値の車、`clear` は車体と真の壁の最小距離。"""
        path = getattr(self.planner, "path", None)
        i = getattr(self.planner, "_hint", -1)
        pv = float(path.v[i]) if path is not None and 0 <= i < len(path) else float("nan")
        self._f.write(
            f"{t:.3f},{st.phase},{v.x:.4f},{v.y:.4f},{v.yaw:.4f},{v.speed:.4f},"
            f"{cmd.target_speed:.3f},{int(cmd.brake)},{pv:.3f},{i},{st.cross_track:.4f},"
            f"{clear:.4f},{int(v.launching)},{cmd.target_steer:.4f},{v.steer_actual:.4f},"
            f"{v.accel_lateral:.3f},{v.accel_x:.3f},"
            f"{(v.yaw_rate / v.speed if abs(v.speed) > 0.05 else 0.0):.4f},"
            f"{v.slip_frac:.4f},{trace_grip(self.planner)}\n")

    def close(self) -> None:
        """閉じ、終わったときに追っていた経路を `<trace>.path.csv` へ書く。"""
        self._f.close()
        path = getattr(self.planner, "path", None)
        if path is not None and len(path) >= 3:
            from raspi.nav.raceline import speed_kappa
            kap = speed_kappa(path.xy, path.closed)
            with open(self._f.name + ".path.csv", "w") as f:
                f.write("idx,x,y,v,kappa\n")
                for i in range(len(path)):
                    f.write(f"{i},{path.xy[i, 0]:.4f},{path.xy[i, 1]:.4f},{path.v[i]:.4f},"
                            f"{kap[i]:.5f}\n")
        if self._long_f is not None:
            self._long_f.close()
