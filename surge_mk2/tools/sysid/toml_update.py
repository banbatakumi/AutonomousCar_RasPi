"""解析結果を `config/vehicle.toml` の `[dynamics]` へ書き戻す。

ステアのリンクの換算（`steer_link_gain`・`steer_link_cubic`、`analyze.LINK_KEYS`）だけは
STM32 へ送る値なので `[control]` に書き、合わせて `[dynamics]` の `steer_gain`・`steer_gain_cubic` を
1 と 0 に、最上位の `max_steer` を換算後の最大舵角にする（3つがそろって初めて意味が合う。
`docs/development.md` のシステム同定の節）。

**該当する数値だけを書き換える。** コメント・書式は `tomlkit`（ラウンドトリップ
対応のTOMLパーサ）でそのまま保持する。ヘッダのプロース説明文・`★未実測` の
注記・`measured` フラグは意図的に触らない——`rolling_resistance` と
`encoder_ticks_per_rev` が引き続き未実測のため `measured=true` に自動ではできず、
手書きの説明文をツールで書き換えると文意が壊れるリスクの方が大きいため。
呼び出し側（`gui.py`）が適用後にコメント見直しを促す。
"""

from __future__ import annotations

import math
import os
import tempfile
from pathlib import Path

import tomlkit

from raspi.core.control_params import STEER_LINK_X_MAX_RAD, steer_link_road

__all__ = ["ALLOWED_KEYS", "LINK_KEYS", "FF_KEYS", "LAUNCH_EXIT_KEY", "speed_ff_values",
           "launch_exit_value", "apply_dynamics"]

#: `[control]` に書くステアのリンクの換算。**必ず2つ一緒に**渡す
LINK_KEYS = ("steer_link_gain", "steer_link_cubic")
#: `[control]` の車速PIの加速度フィードフォワード（STM32 へ送る値、後輪2輪の合計トルク）
FF_KEYS = ("speed_ff_nm_per_m_s2", "speed_ff_rr_nm")


def speed_ff_values(speed_plant_gain: float, rolling_resistance: float) -> dict[str, float]:
    """`[dynamics]` の車体の利得 [(m/s²)/N·m] と転がり抵抗 [m/s²] → `FF_KEYS` の値。

    加速度 a を出すのに要るトルクは (a + 転がり抵抗) ÷ 利得。STM32 は
    `speed_ff_nm_per_m_s2`×目標車速の傾き ＋ `speed_ff_rr_nm`×進む向き を車速PIの出力に足す。
    """
    return {FF_KEYS[0]: round(1.0 / speed_plant_gain, 6),
            FF_KEYS[1]: round(max(rolling_resistance, 0.0) / speed_plant_gain, 6)}

#: `[control]` のローンチコントロールを終えるときの積分の初期値（1輪あたり）
LAUNCH_EXIT_KEY = "launch_exit_torque_nm"


def launch_exit_value(speed_plant_gain: float, rolling_resistance: float, drive_ratio: float) -> float:
    """`[dynamics]` の車体の利得・転がり抵抗・駆動輪数 → `LAUNCH_EXIT_KEY` の値 [N·m、1輪あたり]。

    発進を終えて車速PIへ戻すとき、積分を「その車速を保つのに要るトルク」から始める。それは
    転がり抵抗を打ち消す後輪の合計トルク（転がり抵抗 ÷ 利得）を駆動輪で割ったもの。
    """
    return round(max(rolling_resistance, 0.0) / speed_plant_gain / max(drive_ratio, 1.0), 6)


#: STM32 が `CONFIG_SET` で受け付ける範囲（範囲外は丸められる。`steer_link_*` は 0x0081/0x0082）。
#: 丸められた値を toml に残すと Pi が送る値と STM32 の値、`max_steer` の三者が食い違うので、書く前に弾く
LINK_RANGES = {"steer_link_gain": (0.5, 1.5), "steer_link_cubic": (-1.5, 1.5)}

#: このツールが書き込んでよいキー（[dynamics]直下のみ）。
#: `drive_ratio` は対象外（書こうとしたら`ValueError`にして誤って上書きしないようにする）。
#: `speed_filter_s`/`_order`・`speed_kp`/`speed_ki`/`speed_torque_max_nm` は
#: ファームの定数（測るものではない）なので対象外。`tau_speed_s`（1次遅れ）は測らない（速度はファームの PI、`sim/vehicle.py` の `SpeedController`）
ALLOWED_KEYS = {"tau_steer_s", "dead_time_s", "steer_rate_limit_rad_s", "mu",
                "drive_accel_m_s2", "brake_decel_m_s2",
                "drive_fade_speed_m_s", "drive_top_speed_m_s", "speed_decel_m_s2",
                "steer_offset_rad", "steer_gain", "understeer_gradient", "control_latency_s",
                # 舵のヒステリシス・ガタ・不感帯・非線形性
                "steer_gain_cubic", "steer_servo_hysteresis_rad",
                "steer_link_hysteresis_rad", "steer_link_deadband_rad",
                # ヨーの遅れ: IMU のローパスとタイヤの緩和長
                "yaw_rate_filter_s", "yaw_relaxation_m",
                # 速度PI・ヨーの2次遅れ・サーボの定常ゲイン
                "speed_plant_gain", "rolling_resistance", "yaw_natural_freq_rad_s", "yaw_damping",
                "steer_servo_gain",
                # ブレーキの強さ→減速度
                "brake_decel_per_nm"}


def apply_dynamics(toml_path: str | Path, values: dict[str, float]) -> list[str]:
    """`values`（キー→新しい値）を`[dynamics]`に書き込み、実際に変更したキーを返す。

    `LINK_KEYS` が入っていれば `[control]` に書き、`steer_gain`=1・`steer_gain_cubic`=0・`max_steer` も
    合わせて書く（モジュール冒頭）。

    既存値とほぼ同じ（誤差1e-9未満）ならスキップする——毎回書き込むと
    差分の無いコミットが生まれ、実際に変えた値が埋もれる。

    :raises ValueError: `ALLOWED_KEYS`にないキーが渡された、または`[dynamics]`
        テーブルが無い
    """
    values = dict(values)
    for key, value in values.items():
        if not math.isfinite(float(value)):
            raise ValueError(f"{key} が有限の数ではありません（{value}）。書き込みを中止します")
    link = {k: values.pop(k) for k in LINK_KEYS if k in values}
    unknown = set(values) - ALLOWED_KEYS
    if unknown:
        raise ValueError(f"[dynamics]に書き込めないキーです: {sorted(unknown)}")
    if link and len(link) != len(LINK_KEYS):
        raise ValueError("steer_link_gain と steer_link_cubic は必ず両方とも適用してください"
                         "（片方だけでは舵角の換算が別物になります）")
    if link and ("steer_gain" in values or "steer_gain_cubic" in values):
        raise ValueError("steer_link_* と steer_gain・steer_gain_cubic は同時に書けません"
                         "（リンクの換算へ畳み込んだ後の舵の効きは 1 と 0）")

    for key, (lo, hi) in LINK_RANGES.items():
        if key in link and not lo <= float(link[key]) <= hi:
            raise ValueError(f"{key}={link[key]:.4g} は STM32 の受け付ける範囲 {lo}〜{hi} の外です。"
                             "書き込みを中止します（試験をやり直してください）")

    path = Path(toml_path)
    doc = tomlkit.parse(path.read_text(encoding="utf-8"))
    if "dynamics" not in doc:
        raise ValueError(f"{path} に [dynamics] テーブルがありません")
    dynamics = doc["dynamics"]

    def put(table, key: str, value: float) -> None:
        old = table.get(key)
        if old is not None and abs(float(old) - float(value)) < 1e-9:
            return
        table[key] = float(value)
        changed.append(key)

    changed: list[str] = []
    for key, value in values.items():
        put(dynamics, key, value)
    if link:
        if "control" not in doc:
            raise ValueError(f"{path} に [control] テーブルがありません")
        for key in LINK_KEYS:
            put(doc["control"], key, link[key])
        put(dynamics, "steer_gain", 1.0)
        put(dynamics, "steer_gain_cubic", 0.0)
        # 書かれる値（put は float にするだけだが、比較は書式に依らない）から計算する
        put(doc, "max_steer", steer_link_road(STEER_LINK_X_MAX_RAD, *(float(link[k]) for k in LINK_KEYS)))
    # 車速PIの加速度フィードフォワード（★v0.21）は車体の利得と転がり抵抗から決まる。`[control]` に
    # 有効な値（> 0）が書いてあれば、同定し直した値に合わせて書き直す（0・未記入なら無効のまま）
    control = doc.get("control")
    gain = float(dynamics.get("speed_plant_gain", 0.0) or 0.0)
    rr = float(dynamics.get("rolling_resistance", 0.0) or 0.0)
    if control is not None and gain > 1e-6 and float(control.get(FF_KEYS[0], 0.0) or 0.0) > 0.0:
        ff = speed_ff_values(gain, rr)
        for key in FF_KEYS:
            put(control, key, ff[key])
    # ローンチコントロールを終えるときの積分の初期値も同じ2つ（と駆動輪数）で決まる。同じく有効なときだけ
    if control is not None and gain > 1e-6 and float(control.get(LAUNCH_EXIT_KEY, 0.0) or 0.0) > 0.0:
        put(control, LAUNCH_EXIT_KEY,
            launch_exit_value(gain, rr, float(dynamics.get("drive_ratio", 2.0) or 2.0)))

    if changed:
        _atomic_write(path, tomlkit.dumps(doc))
    return changed


def _atomic_write(path: Path, text: str) -> None:
    """同じフォルダの一時ファイルへ書いてから置き換える（途中で止まっても元のファイルは壊れない）。"""
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=path.name + ".", suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(text)
            f.flush()
            os.fsync(f.fileno())
        os.chmod(tmp, path.stat().st_mode & 0o777)
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise
