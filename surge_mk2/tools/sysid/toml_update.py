"""解析結果を `config/vehicle.toml` の `[dynamics]` へ書き戻す。

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

__all__ = ["ALLOWED_KEYS", "apply_dynamics"]

#: このツールが書き込んでよいキー（[dynamics]直下のみ）。
#: `rolling_resistance`・`drive_ratio` は対象外（`docs`参照。書こうとしたら
#: `ValueError`にして誤って上書きしないようにする）
ALLOWED_KEYS = {"tau_steer_s", "dead_time_s", "steer_rate_limit_rad_s", "tau_speed_s", "mu",
                "drive_accel_m_s2", "brake_decel_m_s2"}


def apply_dynamics(toml_path: str | Path, values: dict[str, float]) -> list[str]:
    """`values`（キー→新しい値）を`[dynamics]`に書き込み、実際に変更したキーを返す。

    既存値とほぼ同じ（誤差1e-9未満）ならスキップする——毎回書き込むと
    差分の無いコミットが生まれ、実際に変えた値が埋もれる。

    :raises ValueError: `ALLOWED_KEYS`にないキーが渡された、または`[dynamics]`
        テーブルが無い
    """
    unknown = set(values) - ALLOWED_KEYS
    if unknown:
        raise ValueError(f"[dynamics]に書き込めないキーです: {sorted(unknown)}")

    path = Path(toml_path)
    doc = tomlkit.parse(path.read_text(encoding="utf-8"))
    if "dynamics" not in doc:
        raise ValueError(f"{path} に [dynamics] テーブルがありません")
    dynamics = doc["dynamics"]

    changed: list[str] = []
    for key, value in values.items():
        old = dynamics.get(key)
        if old is not None and abs(float(old) - float(value)) < 1e-9:
            continue
        dynamics[key] = float(value)
        changed.append(key)

    if changed:
        path.write_text(tomlkit.dumps(doc), encoding="utf-8")
    return changed
