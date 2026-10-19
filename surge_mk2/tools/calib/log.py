"""校正の試験の記録（`Log`）。実機の mcap から読むか、シム（`tools/calib/simcar.py`）が作る。

解析が見るのはここにある列だけ（実機とシムで同じ形）。時刻はすべて Pi の単調時刻 [s] を、
最初の TELEMETRY を 0 にしてそろえたもの。

- TELEMETRY（100Hz、`/vehicle_state`）: `t` と `vs[名前]`（numpy の配列。`wheel_speed` は n×4 など）
- 指令（`/cmd`、io_node へ渡った値）: `cmd_t` と `cmd[名前]`
- planner の状態（`/auto/state`、10Hz）: `phase_t`・`phases`（planner が試験の区間に付けた名前）・`reasons`
- 記録したときに STM32 に入っていた調整値（`/diag/link` の `control_params`）: `control_params`
- スキャン（`/scan`）の番号と時刻: `scans`（遅延試験だけが使う）
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

__all__ = ["Log", "load_mcap", "VS_FIELDS", "CMD_FIELDS"]

NS = 1e9

#: TELEMETRY の列（`VehicleState` のフィールド名）と、1サンプルの要素数
VS_FIELDS: dict[str, int] = {
    "speed": 1, "yaw_rate": 1, "steer_actual": 1, "steer_cmd_echo": 1,
    "wheel_speed": 4, "odom_dist": 2, "accel": 3, "torque_cmd": 2, "torque_req": 2,
    "tc_slip": 2, "tc_limit_nm": 2, "abs_limit_nm": 1, "accel_kf": 1, "front_angle": 2,
    "md_status": 3, "flags": 1, "armed": 1, "t_stm_us": 1,
    "tc_active": 1, "abs_active": 1,
}
#: 指令の列（`DriveCmd` のフィールド名）
CMD_FIELDS = ("target_speed", "target_steer", "accel_limit", "steer_rate_limit", "brake", "brake_torque",
              "torque_mode", "target_torque", "arm")


@dataclass
class Log:
    t: np.ndarray
    vs: dict[str, np.ndarray]
    cmd_t: np.ndarray
    cmd: dict[str, np.ndarray]
    cmd_overrides: list[dict]
    phase_t: np.ndarray
    phases: list[str]
    reasons: list[str]
    mode: str = ""
    control_params: dict[str, float] = field(default_factory=dict)
    #: (seq, 最後の点を測り終えた時刻 [s], t_pub [s])
    scans: list[tuple[int, float, float]] = field(default_factory=list)
    source: str = ""

    # ── 時刻の対応 ────────────────────────────────────────────────────────

    def phase_of_samples(self) -> list[str]:
        """各 TELEMETRY サンプルの時点で planner が名乗っていた区間の名前（それより前は ""）。"""
        idx = np.searchsorted(self.phase_t, self.t, side="right") - 1
        return [self.phases[i] if i >= 0 else "" for i in idx]

    def segments(self, pred) -> list[tuple[str, int, int]]:
        """区間の名前が `pred(名前)` を満たす、連続した TELEMETRY の範囲 `(名前, 先頭, 末尾+1)` の並び。"""
        names = self.phase_of_samples()
        out: list[tuple[str, int, int]] = []
        i = 0
        n = len(names)
        while i < n:
            if not pred(names[i]):
                i += 1
                continue
            j = i
            while j < n and names[j] == names[i]:
                j += 1
            out.append((names[i], i, j))
            i = j
        return out

    def segments_by_reason(self, pred) -> list[tuple[str, int, int]]:
        """`segments` の、planner の理由の文で切る版（区間の名前を持たない旧 planner の記録用）。"""
        saved = self.phases
        self.phases = [f"{r}#{i}" if pred(r) else "" for i, r in enumerate(self.reasons)]
        try:
            # 同じ理由が続く間を 1 区間にする（#番号は隣どうしで違うので、理由の文でまとめ直す）
            names = self.phase_of_samples()
        finally:
            self.phases = saved
        out: list[tuple[str, int, int]] = []
        i = 0
        while i < len(names):
            if not names[i]:
                i += 1
                continue
            key = names[i].rsplit("#", 1)[0]
            j = i
            while j < len(names) and names[j] and names[j].rsplit("#", 1)[0] == key:
                j += 1
            out.append((key, i, j))
            i = j
        return out

    def cmd_at(self, name: str, t: np.ndarray | float) -> np.ndarray:
        """時刻 `t` に効いていた指令の値（その時刻以前で最新の `/cmd`）。"""
        idx = np.searchsorted(self.cmd_t, np.atleast_1d(t), side="right") - 1
        col = self.cmd[name]
        out = np.where(idx >= 0, col[np.clip(idx, 0, None)], 0.0)
        return out

    def first_reason(self, prefix: str) -> str | None:
        for r in self.reasons:
            if r.startswith(prefix):
                return r
        return None

    @property
    def finished(self) -> bool:
        """planner が「完了」と言ったか。"""
        return any(r.startswith("完了") for r in self.reasons)

    @property
    def aborted(self) -> str | None:
        return self.first_reason("中止")


def _vs_rows_to_arrays(rows: list[dict]) -> dict[str, np.ndarray]:
    out: dict[str, np.ndarray] = {}
    for name, width in VS_FIELDS.items():
        if width == 1:
            out[name] = np.array([float(r.get(name) or 0.0) for r in rows])
        else:
            vals = []
            for r in rows:
                v = r.get(name) or [0.0] * width
                v = [float(x) if x is not None else 0.0 for x in v][:width]
                vals.append(v + [0.0] * (width - len(v)))
            out[name] = np.array(vals, dtype=float).reshape(len(rows), width)
    return out


def build(vs_rows: list[tuple[float, dict]], cmd_rows: list[tuple[float, dict]],
          state_rows: list[tuple[float, dict]], control_params: dict[str, float] | None = None,
          scans: list[tuple[int, float, float]] | None = None, source: str = "") -> Log:
    """（時刻 [s], 中身）の並びから `Log` を作る。時刻は同じ時計なら原点はどこでもよい。"""
    vs_rows = sorted(vs_rows, key=lambda x: x[0])
    cmd_rows = sorted(cmd_rows, key=lambda x: x[0])
    state_rows = sorted(state_rows, key=lambda x: x[0])
    if not vs_rows:
        raise ValueError("TELEMETRY（/vehicle_state）がありません")
    t0 = vs_rows[0][0]
    t = np.array([r[0] - t0 for r in vs_rows])
    vs = _vs_rows_to_arrays([r[1] for r in vs_rows])
    cmd_t = np.array([r[0] - t0 for r in cmd_rows])
    cmd = {k: np.array([float(r[1].get(k) or 0.0) for r in cmd_rows]) for k in CMD_FIELDS}
    overrides = [dict(r[1].get("fw_overrides") or {}) for r in cmd_rows]
    phase_t = np.array([r[0] - t0 for r in state_rows])
    phases = [str(r[1].get("phase") or "") for r in state_rows]
    reasons = [str(r[1].get("reason") or "") for r in state_rows]
    modes = [str(r[1].get("mode") or "") for r in state_rows if r[1].get("engaged")]
    mode = max(set(modes), key=modes.count) if modes else ""
    sc = [(seq, tc - t0, tp - t0) for seq, tc, tp in (scans or [])]
    return Log(t=t, vs=vs, cmd_t=cmd_t, cmd=cmd, cmd_overrides=overrides, phase_t=phase_t, phases=phases,
               reasons=reasons, mode=mode, control_params=dict(control_params or {}), scans=sc, source=source)


def load_mcap(path: str | Path) -> Log:
    """GUI の「システム同定」タブで録った mcap → `Log`。"""
    from mcap.reader import make_reader

    vs_rows: list[tuple[float, dict]] = []
    cmd_rows: list[tuple[float, dict]] = []
    state_rows: list[tuple[float, dict]] = []
    scans: list[tuple[int, float, float]] = []
    params: dict[str, float] = {}
    topics = ["/vehicle_state", "/cmd", "/auto/state", "/diag/link", "/scan"]
    with open(path, "rb") as f:
        for _schema, channel, message in make_reader(f).iter_messages(topics=topics):
            obj = json.loads(message.data)
            topic = channel.topic
            if topic == "/vehicle_state":
                vs_rows.append(((obj.get("t_capture") or message.log_time) / NS, obj))
            elif topic == "/cmd":
                cmd_rows.append(((obj.get("t_pub") or message.log_time) / NS, obj))
            elif topic == "/auto/state":
                state_rows.append(((obj.get("t_pub") or message.log_time) / NS, obj))
            elif topic == "/diag/link":
                cp = obj.get("control_params") or {}
                # 記録の途中で入れ替わることは普通ない（試験中の一時的な上書きは fw_overrides で見える）。
                # 最初に「入った」と言っている値を採る
                if cp and not params and obj.get("control_params_status") == "ok":
                    params = {k: float(v) for k, v in cp.items()}
            elif topic == "/scan":
                # 最後の点を測り終えた時刻 = 各セクタの先頭の時刻 ＋ そのセクタの所要時間 の最大
                durs = obj.get("sector_dur_us") or []
                t_done = max((int(x) + (int(durs[i]) * 1000 if i < len(durs) else 0)
                              for i, x in enumerate(obj.get("sector_t_ns") or []) if x), default=0)
                scans.append((int(obj.get("seq") or 0), (t_done or obj.get("t_capture") or 0) / NS,
                              (obj.get("t_pub") or message.log_time) / NS))
    return build(vs_rows, cmd_rows, state_rows, params, scans, source=str(path))
