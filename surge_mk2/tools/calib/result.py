"""解析の結果（`Result`）。どの校正も同じ形で返し、GUI・コマンドはこれだけを表示する。

- `values`: `config/vehicle.toml` に書く値。キーは `"<表>.<キー>"`（例 `"control.enc_left_lo"`、
  `"dynamics.speed_kp"`、`"limits.a_lat_m_s2"`、最上位は `"max_steer"`）
- `findings`: 所見。`level` は `ok`（確かめて問題なし）・`info`（参考）・`warn`（★値は出るが確かめること）・
  `fail`（この記録の値は使えない）
- `verdict`: 所見の最も悪いもの（`fail` があれば「不合格」、`warn` があれば「要確認」、なければ「合格」）
- `series`: GUI に描く時系列・散布図（名前 → (x, y の組の並び)）。描かなくても値は使える
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

__all__ = ["Finding", "Result", "LEVELS"]

LEVELS = ("ok", "info", "warn", "fail")
_MARK = {"ok": "✓", "info": "・", "warn": "★", "fail": "✗"}


@dataclass
class Finding:
    level: str
    text: str


@dataclass
class Result:
    test: str
    values: dict[str, float] = field(default_factory=dict)
    findings: list[Finding] = field(default_factory=list)
    #: 名前 → [(凡例, x, y), …]
    series: dict[str, list[tuple[str, np.ndarray, np.ndarray]]] = field(default_factory=dict)
    #: 値ごとの説明（GUI の一覧に出す。キーは `values` と同じ）
    notes: dict[str, str] = field(default_factory=dict)

    def add(self, level: str, text: str) -> None:
        assert level in LEVELS, level
        self.findings.append(Finding(level, text))

    def ok(self, text: str) -> None:
        self.add("ok", text)

    def info(self, text: str) -> None:
        self.add("info", text)

    def warn(self, text: str) -> None:
        self.add("warn", text)

    def fail(self, text: str) -> None:
        self.add("fail", text)

    def plot(self, name: str, label: str, x, y) -> None:
        self.series.setdefault(name, []).append((label, np.asarray(x, dtype=float), np.asarray(y, dtype=float)))

    @property
    def verdict(self) -> str:
        levels = {f.level for f in self.findings}
        if "fail" in levels:
            return "不合格"
        if "warn" in levels:
            return "要確認"
        return "合格"

    @property
    def usable(self) -> bool:
        """値を適用してよいか（不合格でない）。"""
        return self.verdict != "不合格"

    def report(self) -> str:
        lines = [f"■ {self.test}: {self.verdict}"]
        lines += [f"  {_MARK[f.level]} {f.text}" for f in self.findings]
        if self.values:
            lines.append("  書き込む値:")
            for k, v in self.values.items():
                note = self.notes.get(k)
                lines.append(f"    {k} = {v:.6g}" + (f"   # {note}" if note else ""))
        return "\n".join(lines)
