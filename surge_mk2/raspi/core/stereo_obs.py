"""`scan/stereo`（擬似 `Scan`）→ STM32 へ送る `STEREO_OBS`（自動停止の入力）。

判定は STM32 がする（`docs/uart_protocol.md` §5.6.4）。ここは点を間引いて渡すだけで、停止の判断も距離のしきい値も持たない。

- **「見つからない」は送らない**: 失敗・空き待ちの `Scan`（`t_capture=0`、または視野のセクタが無い）は None にする。
  STM32 は届かなければ LiDAR・超音波だけで判定に戻る。「何も見えていない」を「障害物なし」として送らない
- 照合できた周期は、点が 0 個でも送る（STM32 が「ステレオは生きている」と分かるように）
- 点は 1° の方位ごとの距離（base_link 原点）。2° の箱ごとに最も近い点を残し、多ければ近い順に `MAX_POINTS` 点
"""

from __future__ import annotations

import math

from raspi.proto import packets

__all__ = ["MAX_POINTS", "BIN_DEG", "MAX_AGE_MS", "scan_to_obs"]

MAX_POINTS = 40
BIN_DEG = 2
#: これより古い `Scan` は送らない（STM32 側の最大の古さ 300ms より手前。古い点は予想進路を延ばして補うが、限度がある）
MAX_AGE_MS = 250


def scan_to_obs(scan, now_ns: int) -> packets.StereoObs | None:
    if scan.t_capture <= 0 or not any(scan.sector_seen):
        return None
    age_ms = (now_ns - scan.t_capture) / 1e6
    if age_ms < 0:
        age_ms = 0.0
    if age_ms > MAX_AGE_MS:
        return None
    best: dict[int, tuple[float, float, float]] = {}
    for deg, d in enumerate(scan.dist):
        if not (d > 0.0) or not math.isfinite(d):
            continue
        b = deg // BIN_DEG
        if b not in best or d < best[b][0]:
            a = math.radians(deg)
            best[b] = (d, d * math.cos(a), d * math.sin(a))
    pts = sorted(best.values())[:MAX_POINTS]
    xy = [0] * (2 * MAX_POINTS)
    for i, (_, x, y) in enumerate(pts):
        xy[2 * i] = max(-32768, min(32767, round(x * 1000)))
        xy[2 * i + 1] = max(-32768, min(32767, round(y * 1000)))
    return packets.StereoObs(age_ms=min(255, round(age_ms)), count=len(pts), xy=xy)
