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

from pathlib import Path

import tomlkit

from raspi.core.control_params import STEER_LINK_X_MAX_RAD, steer_link_road

__all__ = ["ALLOWED_KEYS", "LINK_KEYS", "apply_dynamics"]

#: `[control]` に書くステアのリンクの換算。**必ず2つ一緒に**渡す
LINK_KEYS = ("steer_link_gain", "steer_link_cubic")

#: このツールが書き込んでよいキー（[dynamics]直下のみ）。
#: `drive_ratio` は対象外（書こうとしたら`ValueError`にして誤って上書きしないようにする）。
#: `speed_filter_s`/`_order`・`speed_kp`/`speed_ki`/`speed_torque_max_nm`/`speed_ramp_max_m_s2` は
#: ファームの定数（測るものではない）なので対象外。`tau_speed_s`（旧モデルの1次遅れ）は
#: 2026-09-26 から測らない（速度はファームの PI、`sim/vehicle.py` の `SpeedController`）
ALLOWED_KEYS = {"tau_steer_s", "dead_time_s", "steer_rate_limit_rad_s", "mu",
                "drive_accel_m_s2", "brake_decel_m_s2",
                # 2026-09-24 追加（`tools/sysid/fit.py` の作り直し）
                "drive_fade_speed_m_s", "drive_top_speed_m_s", "speed_decel_m_s2",
                "steer_offset_rad", "steer_gain", "understeer_gradient", "control_latency_s",
                # 2026-09-25 追加（舵のヒステリシス・ガタ・不感帯・非線形性）
                "steer_gain_cubic", "steer_servo_hysteresis_rad",
                "steer_link_hysteresis_rad", "steer_link_deadband_rad",
                # 2026-09-25 追加（ヨーの遅れ: IMU のローパスとタイヤの緩和長）
                "yaw_rate_filter_s", "yaw_relaxation_m",
                # 2026-09-26 追加（第三者検証: 速度PI・ヨーの2次遅れ・サーボの定常ゲイン）
                "speed_plant_gain", "rolling_resistance", "yaw_natural_freq_rad_s", "yaw_damping",
                "steer_servo_gain",
                # 2026-09-27 追加（ブレーキの強さ→減速度）
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
        put(doc, "max_steer", steer_link_road(STEER_LINK_X_MAX_RAD, *(link[k] for k in LINK_KEYS)))

    if changed:
        path.write_text(tomlkit.dumps(doc), encoding="utf-8")
    return changed
