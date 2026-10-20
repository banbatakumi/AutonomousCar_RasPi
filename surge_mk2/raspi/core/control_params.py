"""足回りの制御（TC・ABS・片輪浮き対策・TV）の調整パラメータを STM32 へ入れ、入ったことを確かめる。

## 値の出どころ

`config/vehicle.toml` の `[control]` が唯一の定義（キーは STM32 側 `control_params.h` のフィールド名）。
同定用の `tv_test_moment_nm` だけは `[control]` に書けない（一時的な上書きでだけ入れる）。
車速PIのゲイン `speed_kp`・`speed_ki` だけは `[dynamics]` に書く（同定・シム・Pi の計算と共通の値）。
自動停止・DISARM 後の制動の見積もり（`brake_decel_m_s2`・`md_torque_lag_s`、v0.29）は `[control]` に書かず、
校正の結果（`DERIVED`）から作る。
**STM32 は値を持たない**（起動直後は仮置き）ので、全項目が揃っていなければ `load_control_params` は
エラーにし、io_node は ARM を通さない。

`steer_link_gain`・`steer_link_cubic`（ステアのリンクの換算、モータ角→路面舵角）もここを通る。
調整値ではなく校正の結果で、`tools/calib`（校正 2、`docs/calibration.md`）が `[control]` に書く。STM32 に入っていないと
舵角の指令・報告・`max_steer` の意味が食い違うので、`control_params_status` が `ok` でない間は走らせないこと。

## 入ったことの確かめ方（`ControlParamSync`）

STM32 は `CONFIG_SET` に必ず `CONFIG_ACK`（`param_id`・**実際に入った値** `applied`・結果）を返す。
STM32 は値を Flash に保存しないので、電源を入れ直すと既定値に戻る。そこで:

1. 接続したら全項目を `CONFIG_SET` で送り、`CONFIG_ACK.applied` が送った値と一致するまで再送する
2. 一致した後も、1項目ずつ順番に `CONFIG_GET` で読み戻し続ける。食い違いを1つでも見つけたら
   STM32 だけが再起動して既定値に戻ったと見て、全項目を送り直す（`drift_count` が増える。
   io_node はこれを見て、入り直すまで ARM を通さない）
3. STM32 の再起動を検出したら（`reset()`）最初からやり直す

状態は `status`（`pending`/`ok`/`mismatch`/`unsupported`）と `applied`（STM32 が答えた値）で
`LinkDiag` に出る。`unsupported` は STM32 が `UNKNOWN_ID` を返した＝ファームが古い。

## 一時的な上書き（`set_overrides()`）

校正の試験（`raspi/auto/calib_wheel.py`・`calib_tyre.py` は TC・ABS を切る）が使う。
planner が `AutoState.fw_overrides` に書いた値を、指令（`DriveCmd.fw_overrides`）が届いている間だけ
入れ、届かなくなったら元の値へ戻す（試験が中断・異常終了しても TC を切ったままにならない）。
"""

from __future__ import annotations

import math
import tomllib
from pathlib import Path
from typing import Callable

from ..proto.generated import packets

__all__ = ["CONTROL_PARAM_IDS", "DERIVED", "ENABLE_KEYS", "FROM_DYNAMICS", "disarm_brake_timeout_s", "REQUIRED", "OVERRIDABLE", "load_control_params", "ControlParamSync",
           "DEFAULT_TOML", "STEER_LINK_X_MAX_RAD", "steer_link_road", "steer_link_x", "steer_relink"]

DEFAULT_TOML = Path(__file__).resolve().parents[2] / "config" / "vehicle.toml"

#: `[control]` のキー → `param_id`（`protocol.toml` の `[params]`。名前は大文字にしたもの）
CONTROL_PARAM_IDS: dict[str, int] = {
    name.lower(): getattr(packets.Param, name) for name in (
        "SLIP_SPEED_FLOOR_M_S", "TC_SLIP_TARGET", "TC_KP_NM_PER_M_S", "TC_KI_NM_PER_M",
        "TC_MIN_TORQUE_NM", "WHEEL_LIFT_DIFF_THRESHOLD_M_S",
        "ABS_SLIP_TARGET", "ABS_KP_NM_PER_M_S", "ABS_KI_NM_PER_M",
        "TV_LOAD_GAIN_S2_PER_M", "TV_MAX_RATIO", "TV_TEST_MOMENT_NM",
        "STEER_LINK_GAIN", "STEER_LINK_CUBIC", "LAUNCH_EXIT_TORQUE_NM",
        "SPEED_FF_NM_PER_M_S2", "SPEED_FF_RR_NM", "SPEED_KP", "SPEED_KI",
        # ★v0.27: 前輪エンコーダの校正・車速のカルマンフィルタ
        *(f"ENC_{side}_{k}" for side in ("LEFT", "RIGHT") for k in ("LO", "HI", "H1S", "H1C", "H2S", "H2C")),
        "KF_SIGMA_S_M", "KF_SIGMA_A_M_S2", "KF_SIGMA_A_PER_M_S", "KF_JERK_PSD", "KF_BIAS_WALK_PSD",
        "KF_LEVER_ARM_M",
        # ★v0.28: ジャイロ z 軸の感度の補正
        "IMU_GYRO_Z_SCALE",
        # ★v0.29: 自動停止・DISARM 後の制動の見積もり（`DERIVED` から作る）
        "BRAKE_DECEL_M_S2", "MD_TORQUE_LAG_S")}

#: `[control]` ではなく `[dynamics]` から読む項目（同定・シム・Pi の計算と共通の値。2か所に持たない）
FROM_DYNAMICS = ("speed_kp", "speed_ki")

#: `[control]` に書かず、ほかの節の校正の結果から作る項目（同じ値を2か所に持たない）。値は (説明, 作り方)
DERIVED: dict[str, tuple[str, Callable[[dict], float]]] = {
    "brake_decel_m_s2": ("[limits] abs_brake_decel_m_s2（校正 9）",
                         lambda d: d["limits"]["abs_brake_decel_m_s2"]),
    "md_torque_lag_s": ("[control.plant] md_delay_s + md_tau_s（校正 6）",
                        lambda d: d["control"]["plant"]["md_delay_s"] + d["control"]["plant"]["md_tau_s"]),
}

#: STM32 の最高速の歯止め（`drive.h` の DRIVE_MAX_SPEED_M_S）と、止まり切るはずの時間に掛ける余裕
#: （`vehicle.h` の VEHICLE_STOP_TIME_MARGIN）。ファームの定数の写し（`tools/fwsim/tests/test_fw_constants.py` が突き合わせる）
STM32_MAX_SPEED_M_S = 5.0
STOP_TIME_MARGIN = 1.5


def disarm_brake_timeout_s(brake_decel_m_s2: float) -> float:
    """走行中に ARM が外れたとき、停車を待って制動を続ける時間の上限 [s]（STM32 の vehicle.c と同じ式）。
    最高速から `brake_decel_m_s2` で止まり切る時間 × 余裕。"""
    return STM32_MAX_SPEED_M_S / brake_decel_m_s2 * STOP_TIME_MARGIN


#: `load_control_params` が揃っていることを求める数値項目（STM32 は値を持たず、上位が送るまで仮置き）
REQUIRED = tuple(k for k in CONTROL_PARAM_IDS if k != "tv_test_moment_nm")

#: `[control]` に書ける機能の ON/OFF（0/1 か true/false）。STM32 の既定は**無効**で、ここに書いた値を
#: 接続のたびに送って有効にする（書かなければ無効のまま走る）。GUI の「走行アシスト」で切り替えたら
#: `ControlParamSync.set_wanted()` で送る値も差し替える（そうしないと読み戻しが元へ戻してしまう）
ENABLE_KEYS: dict[str, int] = {
    "tc_enable": packets.Param.TC_ENABLE,
    "tv_enable": packets.Param.TV_ENABLE,
    "wheel_lift_guard_enable": packets.Param.WHEEL_LIFT_GUARD_ENABLE,
    "abs_enable": packets.Param.ABS_ENABLE,
}

#: ステアのリンクの換算（`steer_link_gain`・`steer_link_cubic`）の入力 x = モータ角×リンク比 の
#: 可動範囲 [rad]。STM32 側 `steering.h` の STEERING_MAX_ANGLE_RAD × STEERING_LINKAGE_RATIO
STEER_LINK_X_MAX_RAD = math.radians(30.0)


def steer_link_road(x_rad: float, gain: float, cubic: float) -> float:
    """リンクの換算（STM32 の `steering.c` と同じ式）: x = モータ角×リンク比 → 路面舵角 [rad]。"""
    return gain * x_rad + cubic * x_rad ** 3


def steer_link_x(road_rad: float, gain: float, cubic: float) -> float:
    """`steer_link_road` の逆（路面舵角 → x）。可動範囲の外は端で止める（STM32 と同じニュートン法）。"""
    x = road_rad
    for _ in range(6):
        slope = max(gain + 3.0 * cubic * x * x, 0.3)
        x -= (steer_link_road(x, gain, cubic) - road_rad) / slope
        x = max(-STEER_LINK_X_MAX_RAD, min(STEER_LINK_X_MAX_RAD, x))
    return x


def steer_relink(angle_rad: float, link_from: tuple[float, float], link_to: tuple[float, float]) -> float:
    """リンクの換算 `link_from` のもとでの舵角を、同じモータ角になる `link_to` のもとでの舵角へ直す。

    舵角の数値の意味は STM32 に入っている換算で変わる（v0.17 以前は換算なし＝`(1, 0)` で、数値は
    「モータ角×0.5」だった）。別の換算のもとで学習・記録された舵角を今の車で同じ切れ角にするのに使う。
    """
    if tuple(link_from) == tuple(link_to):
        return angle_rad
    return steer_link_road(steer_link_x(angle_rad, *link_from), *link_to)

#: 一時的に上書きできる項目（調整パラメータ全部＋機能の ON/OFF）→ `param_id`
OVERRIDABLE: dict[str, int] = {**CONTROL_PARAM_IDS, **ENABLE_KEYS}
_ENABLE_IDS = frozenset(ENABLE_KEYS.values())

#: 送った値と `applied` の一致の許容（STM32 は float32 で持つ）
_REL_TOL = 1e-5
_ABS_TOL = 1e-7
#: `CONFIG_SET.value`（float32）で表せる最大。これを超える値はパケットに詰められない
_F32_MAX = 3.4028234663852886e38


def _is_number(value) -> bool:
    """`CONFIG_SET` で送れる数か（bool・非数・無限大・float32 に入らない大きさを除く）。"""
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value) and abs(value) <= _F32_MAX)


def load_control_params(toml_path: str | Path = DEFAULT_TOML, strict: bool = True) -> dict[str, float]:
    """`vehicle.toml` → STM32 へ送る値（キーは `CONTROL_PARAM_IDS`・`ENABLE_KEYS` の名前）。

    `strict`（既定）: STM32 は値を持たないので、`REQUIRED` の項目が1つでも欠けていればエラー。
    `[control]` が無い場合も同じ。`tc_enable` などの ON/OFF は書かなければ送らない（STM32 の既定＝無効）。
    :raises ValueError: 知らないキー・数でない値・（strict のとき）足りない項目
    """
    with open(toml_path, "rb") as f:
        d = tomllib.load(f)
    control = d.get("control")
    out: dict[str, float] = {}
    for key, value in (control or {}).items():
        if isinstance(value, dict):          # [control.plant] など（Pi は使わない）
            continue
        if key in ENABLE_KEYS:
            if isinstance(value, bool) or value in (0, 1):
                out[key] = 1.0 if value else 0.0
                continue
            raise ValueError(f"[control] {key} は 0/1 か true/false です: {value!r}")
        if not _is_number(value):
            raise ValueError(f"[control] {key} が数ではありません: {value!r}")
        if key in FROM_DYNAMICS:
            raise ValueError(f"[control] {key} は [dynamics] に書いてください（そちらが唯一の定義）")
        if key in DERIVED:
            raise ValueError(f"[control] {key} は書かない（{DERIVED[key][0]} から作る）")
        if key in CONTROL_PARAM_IDS and key != "tv_test_moment_nm":
            out[key] = float(value)
        else:
            raise ValueError(f"[control] に知らないキーがあります: {key}")
    dyn = d.get("dynamics") or {}
    for key in FROM_DYNAMICS:
        if key in dyn:
            v = dyn[key]
            if not _is_number(v):
                raise ValueError(f"[dynamics] {key} が数ではありません: {v!r}")
            out[key] = float(v)
    for key, (src, make) in DERIVED.items():
        try:
            v = make(d)
        except (KeyError, TypeError):
            continue                          # 足りなければ下の strict で知らせる
        if not _is_number(v):
            raise ValueError(f"{src} が数ではありません: {v!r}")
        out[key] = float(v)
    if strict:
        missing = [k for k in REQUIRED if k not in out]
        if missing:
            raise ValueError("STM32 は調整パラメータの値を持たないので vehicle.toml に全項目が要ります。"
                             f"足りない項目: {', '.join(missing)}"
                             "（speed_kp・speed_ki は [dynamics]、"
                             + "・".join(f"{k} は {v[0]}" for k, v in DERIVED.items()) + "、ほかは [control]）")
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
        unknown = set(wanted) - set(CONTROL_PARAM_IDS) - set(ENABLE_KEYS)
        if unknown:
            raise ValueError(f"知らない調整パラメータ: {sorted(unknown)}")
        self._send = send
        self._base: dict[int, float] = {OVERRIDABLE[k]: float(v) for k, v in wanted.items()}
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
        #: 一致した後の読み戻しで食い違いを見つけた回数（STM32 だけが再起動した等）。送る値を
        #: こちらが変えたとき（`set_wanted`・`set_overrides`）には増えない
        self.drift_count = 0

    # ── 外から呼ぶもの ──

    def reset(self) -> None:
        """STM32 が再起動した。全項目を送り直す（ON/OFF の戻し先も忘れる——既定値に戻っている）。"""
        self._applied.clear()
        self._confirmed.clear()
        self._unsupported.clear()
        self._clamped.clear()
        self._last_tx.clear()
        self._restore.clear()

    def set_wanted(self, name: str, value: float) -> None:
        """送る値（`[control]` 由来）を差し替える。GUI の ON/OFF が変えた機能が、読み戻しで元へ戻らないように。

        `[control]` に無い項目は何もしない（GUI は STM32 へ直接送り、ここは関与しない）。
        """
        pid = OVERRIDABLE.get(name)
        if pid is None or pid not in self._base or self._base[pid] == float(value):
            return
        self._base[pid] = float(value)
        self._resend(pid)

    def set_overrides(self, overrides: dict[str, float]) -> None:
        """一時的な上書きを差し替える（空にすれば元へ戻す）。知らない名前・数でない値は無視する。"""
        new: dict[int, float] = {}
        for k, v in overrides.items():
            pid = OVERRIDABLE.get(k)
            if pid is None or not _is_number(v):
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
            self._resend(pid)

    def _resend(self, pid: int) -> None:
        """送る値が変わった項目を次の `tick()` で送る。前の値が範囲外で丸められていた記憶も消す
        （消さないと、丸められた上書きを外しても元の値を送らず、STM32 に丸めた値が残る）。"""
        self._confirmed.discard(pid)
        self._clamped.discard(pid)
        self._last_tx.pop(pid, None)

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
            # 一致を確かめた後の読み戻しで食い違った = こちらが送っていないのに STM32 の値が変わった
            # （STM32 だけが再起動して既定値に戻った）。ほかの項目も戻っているはずなので、1項目ずつの
            # 読み戻しが一巡する（全項目で十数秒）のを待たず、全部を次の tick から送り直す
            self._confirmed.clear()
            self._clamped.clear()
            self._last_tx.clear()
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
