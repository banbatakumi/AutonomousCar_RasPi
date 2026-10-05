"""足回りの制御（TC・ABS・片輪浮き対策・TV）の調整パラメータを STM32 へ入れ、入ったことを確かめる。

## 値の出どころ

`config/vehicle.toml` の `[control]` が唯一の定義（キーは STM32 側 `control_params.h` のフィールド名）。
TV の規範に使う3つは `[dynamics]` の同定結果から作る（以前はファームの定数へ手で写していて、
同定し直すたびに食い違った）:

- `tv_steer_gain`・`tv_steer_gain_cubic` ← `[dynamics]` の `steer_gain`・`steer_gain_cubic`
- `tv_max_lateral_accel_m_s2` ← `mu`×g×`[control] tv_lateral_accel_ratio`

## 入ったことの確かめ方（`ControlParamSync`）

STM32 は `CONFIG_SET` に必ず `CONFIG_ACK`（`param_id`・**実際に入った値** `applied`・結果）を返す。
STM32 は値を Flash に保存しないので、電源を入れ直すと既定値に戻る。そこで:

1. 接続したら全項目を `CONFIG_SET` で送り、`CONFIG_ACK.applied` が送った値と一致するまで再送する
2. 一致した後も、1項目ずつ順番に `CONFIG_GET` で読み戻し続ける。食い違い（STM32 だけが
   再起動して既定値に戻った等）を見つけたらその項目を送り直す
3. STM32 の再起動を検出したら（`reset()`）最初からやり直す

状態は `status`（`pending`/`ok`/`mismatch`/`unsupported`）と `applied`（STM32 が答えた値）で
`LinkDiag` に出る。`unsupported` は STM32 が `UNKNOWN_ID` を返した＝ファームが古い。

## 一時的な上書き（`set_overrides()`）

同定の試験（`raspi/auto/sysid_*.py`）は TC・ABS を切ったり、ヨーモーメントを入れたりする。
planner が `AutoState.fw_overrides` に書いた値を、指令（`DriveCmd.fw_overrides`）が届いている間だけ
入れ、届かなくなったら元の値へ戻す（試験が中断・異常終了しても TC を切ったままにならない）。
"""

from __future__ import annotations

import math
import tomllib
from pathlib import Path
from typing import Callable

from ..proto.generated import packets

__all__ = ["CONTROL_PARAM_IDS", "OVERRIDABLE", "load_control_params", "ControlParamSync",
           "DEFAULT_TOML"]

DEFAULT_TOML = Path(__file__).resolve().parents[2] / "config" / "vehicle.toml"
GRAVITY_M_S2 = 9.80665

#: `[control]` のキー → `param_id`（`protocol.toml` の `[params]`。名前は大文字にしたもの）
CONTROL_PARAM_IDS: dict[str, int] = {
    name.lower(): getattr(packets.Param, name) for name in (
        "SLIP_SPEED_FLOOR_M_S", "TC_SLIP_TARGET", "TC_KP_NM_PER_M_S", "TC_KI_NM_PER_M",
        "TC_MIN_TORQUE_NM", "WHEEL_LIFT_DIFF_THRESHOLD_M_S",
        "ABS_SLIP_TARGET", "ABS_KP_NM_PER_M_S", "ABS_KI_NM_PER_M",
        "TV_KP_NM_PER_RAD_S", "TV_KI_NM_PER_RAD", "TV_DEADBAND_RAD_S", "TV_MAX_YAW_MOMENT_NM",
        "TV_MAX_LATERAL_ACCEL_M_S2", "TV_STEER_GAIN", "TV_STEER_GAIN_CUBIC", "TV_STABILITY_FACTOR",
        "TV_REF_LAG_S", "TV_TEST_MOMENT_NM")}

#: 一時的に上書きできる項目（調整パラメータ全部＋機能の ON/OFF）→ `param_id`
OVERRIDABLE: dict[str, int] = {
    **CONTROL_PARAM_IDS,
    "tc_enable": packets.Param.TC_ENABLE,
    "tv_enable": packets.Param.TV_ENABLE,
    "wheel_lift_guard_enable": packets.Param.WHEEL_LIFT_GUARD_ENABLE,
    "abs_enable": packets.Param.ABS_ENABLE,
}
_ENABLE_IDS = frozenset(OVERRIDABLE[k] for k in ("tc_enable", "tv_enable", "wheel_lift_guard_enable",
                                                 "abs_enable"))

#: 送った値と `applied` の一致の許容（STM32 は float32 で持つ）
_REL_TOL = 1e-5
_ABS_TOL = 1e-7


def load_control_params(toml_path: str | Path = DEFAULT_TOML) -> dict[str, float]:
    """`vehicle.toml` → STM32 へ送る値（キーは `CONTROL_PARAM_IDS` の名前）。

    `[control]` が無ければ空（＝何も送らず、STM32 の既定値のまま走る）。
    :raises ValueError: `[control]` に知らないキー・数でない値がある
    """
    with open(toml_path, "rb") as f:
        d = tomllib.load(f)
    control = d.get("control")
    if control is None:
        return {}
    dyn = d.get("dynamics", {})
    out: dict[str, float] = {}
    ratio = 0.0
    for key, value in control.items():
        if isinstance(value, dict):          # [control.plant] など（Pi は使わない）
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"[control] {key} が数ではありません: {value!r}")
        if key == "tv_lateral_accel_ratio":
            ratio = float(value)
        elif key in CONTROL_PARAM_IDS and key != "tv_test_moment_nm":
            out[key] = float(value)
        else:
            raise ValueError(f"[control] に知らないキーがあります: {key}")
    # [dynamics] の同定結果から作る（[control] に直接書いてあればそちらを使う）
    if "steer_gain" in dyn:
        out.setdefault("tv_steer_gain", float(dyn["steer_gain"]))
    if "steer_gain_cubic" in dyn:
        out.setdefault("tv_steer_gain_cubic", float(dyn["steer_gain_cubic"]))
    if ratio > 0.0 and dyn.get("mu", 0.0) > 0.0:
        out.setdefault("tv_max_lateral_accel_m_s2", float(dyn["mu"]) * GRAVITY_M_S2 * ratio)
    return out


def _same(a: float, b: float) -> bool:
    return abs(a - b) <= max(_ABS_TOL, _REL_TOL * max(abs(a), abs(b)))


class ControlParamSync:
    """`wanted`（名前 → 値）を STM32 に入れて保つ。`send` は `packets.ConfigSet`/`ConfigGet` を送る関数。

    `tick()` を毎ループ、`on_ack()` を `CONFIG_ACK` ごとに呼ぶ。時刻は単調時刻 [ns]。
    """

    RETRY_NS = 300_000_000          #: 応答が無いときの再送間隔
    VERIFY_NS = 500_000_000         #: 一致した後の読み戻しの間隔（1項目ずつ順番に）
    BURST = 4                       #: 1回の `tick()` で送る数の上限（UART を埋めない）

    def __init__(self, wanted: dict[str, float], send: Callable[[object], None]) -> None:
        unknown = set(wanted) - set(CONTROL_PARAM_IDS)
        if unknown:
            raise ValueError(f"知らない調整パラメータ: {sorted(unknown)}")
        self._send = send
        self._base: dict[int, float] = {CONTROL_PARAM_IDS[k]: float(v) for k, v in wanted.items()}
        self._names: dict[int, str] = {v: k for k, v in OVERRIDABLE.items()}
        self._overrides: dict[int, float] = {}
        #: 上書きを外すときに戻す値（ON/OFF は上書きの直前に STM32 が答えていた値）
        self._restore: dict[int, float] = {}
        self._applied: dict[int, float] = {}
        self._confirmed: set[int] = set()
        self._unsupported: set[int] = set()
        self._clamped: set[int] = set()
        self._last_tx: dict[int, int] = {}
        self._next_verify_ns = 0
        self._verify_i = 0
        #: 一致した後の読み戻しで食い違いを見つけた回数（STM32 だけが再起動した等）
        self.drift_count = 0

    # ── 外から呼ぶもの ──

    def reset(self) -> None:
        """STM32 が再起動した。全項目を送り直す（ON/OFF の戻し先も忘れる——既定値に戻っている）。"""
        self._applied.clear()
        self._confirmed.clear()
        self._unsupported.clear()
        self._clamped.clear()
        self._last_tx.clear()

    def set_overrides(self, overrides: dict[str, float]) -> None:
        """一時的な上書きを差し替える（空にすれば元へ戻す）。知らない名前・数でない値は無視する。"""
        new: dict[int, float] = {}
        for k, v in overrides.items():
            pid = OVERRIDABLE.get(k)
            if pid is None or isinstance(v, bool) or not isinstance(v, (int, float)) \
                    or not math.isfinite(v):
                continue
            new[pid] = float(v)
        if new == self._overrides:
            return
        for pid in new:
            if pid not in self._overrides and pid not in self._base and pid in self._applied:
                self._restore[pid] = self._applied[pid]
        for pid in set(self._overrides) - set(new):
            if pid not in self._base and pid not in self._restore:
                # 戻す先が分からない（上書きの前に STM32 の値を聞けていなかった）: ON/OFF は有効、
                # ほかは 0（`tv_test_moment_nm`）へ戻す
                self._restore[pid] = 1.0 if pid in _ENABLE_IDS else 0.0
        changed = {pid for pid in set(new) | set(self._overrides)
                   if new.get(pid) != self._overrides.get(pid)}
        self._overrides = new
        for pid in changed:
            self._confirmed.discard(pid)
            self._last_tx.pop(pid, None)       # すぐ送る

    def on_ack(self, msg: packets.ConfigAck) -> None:
        pid = msg.param_id
        want = self._target().get(pid)
        if msg.result == packets.ConfigResult.UNKNOWN_ID:
            if want is not None:
                self._unsupported.add(pid)
            return
        self._applied[pid] = float(msg.applied)
        if want is None:
            return
        if msg.result == packets.ConfigResult.OUT_OF_RANGE:
            self._clamped.add(pid)             # STM32 が範囲へ丸めた。再送しても同じなので止める
            return
        if _same(float(msg.applied), want):
            self._confirmed.add(pid)
            self._clamped.discard(pid)
            if pid in self._restore and pid not in self._overrides:
                del self._restore[pid]         # 戻し終えた
        elif pid in self._confirmed:
            self._confirmed.discard(pid)       # 読み戻しで食い違い → 次の tick で送り直す
            self._last_tx.pop(pid, None)
            self.drift_count += 1

    def tick(self, now_ns: int) -> None:
        sent = 0
        target = self._target()
        for pid, value in target.items():
            if sent >= self.BURST:
                return
            if pid in self._confirmed or pid in self._unsupported or pid in self._clamped:
                continue
            if now_ns - self._last_tx.get(pid, -self.RETRY_NS) < self.RETRY_NS:
                continue
            self._send(packets.ConfigSet(param_id=pid, value=value))
            self._last_tx[pid] = now_ns
            sent += 1
        # 上書きの前の値を知らない ON/OFF は、上書きを送る前に聞いておきたいが、順序を待つより
        # 「分からなければ有効へ戻す」方が単純で安全（`set_overrides`）
        if sent == 0 and target and now_ns >= self._next_verify_ns:
            ids = sorted(self._confirmed)
            if ids:
                self._verify_i %= len(ids)
                self._send(packets.ConfigGet(param_id=ids[self._verify_i]))
                self._verify_i += 1
            self._next_verify_ns = now_ns + self.VERIFY_NS

    # ── 状態 ──

    def _target(self) -> dict[int, float]:
        return {**self._base, **self._restore, **self._overrides}

    @property
    def status(self) -> str:
        """`none`（送るものが無い）/`pending`/`ok`/`mismatch`（範囲外で丸められた）/`unsupported`。"""
        target = self._target()
        if not target:
            return "none"
        if self._unsupported & set(target):
            return "unsupported"
        if self._clamped & set(target):
            return "mismatch"
        return "ok" if set(target) <= self._confirmed else "pending"

    @property
    def applied(self) -> dict[str, float]:
        """STM32 が答えた値（名前 → 値）。まだ答えの無い項目は入らない。"""
        return {self._names[pid]: v for pid, v in sorted(self._applied.items()) if pid in self._names}

    @property
    def problems(self) -> list[str]:
        """人が読む食い違いの一覧（`unsupported`・`mismatch` のとき）。"""
        target = self._target()
        out = [f"{self._names[pid]}: STM32 が対応していません（ファームが古い）"
               for pid in sorted(self._unsupported & set(target))]
        out += [f"{self._names[pid]}: {target[pid]:g} を送りましたが {self._applied.get(pid, float('nan')):g} "
                "に丸められました（範囲外）" for pid in sorted(self._clamped & set(target))]
        return out

    @property
    def overriding(self) -> bool:
        return bool(self._overrides)
