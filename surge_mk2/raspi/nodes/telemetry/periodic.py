"""「現在の意思」を周期的に流し直す処理を、1本のループで回す。

telemetry_node は、受け手（camera_node・planning_node・各推論ノード・io_node）が再起動しても、
取りこぼしても、GUI で何も操作しないまま最大1周期で最新の意思へ戻るよう、意思を繰り返し流す
（変更の瞬間は別に即座に1発送っている。こちらは保険）。以前は種類ごとに `while: sleep; send` の
タスクを1本ずつ持っていたので、それを `[(周期, 関数)]` の表にした。

**ここに入れるのは軽い同期処理だけ。** 待ちのある処理（`nmcli`・`vcgencmd` のサブプロセス、
カメラの JPEG）を入れると他の再送まで遅れるので、それらは別タスクのまま。
関数が例外を投げたらループごと終わる（`TelemetryServer._on_pump_done` がサーバを止め、
systemd が上げ直す。1本だけ黙って止まったままにしない）。
"""

from __future__ import annotations

import asyncio
from typing import Callable, Sequence

__all__ = ["run_periodic"]


async def run_periodic(jobs: Sequence[tuple[float, Callable[[], None]]],
                       running: Callable[[], bool]) -> None:
    """`jobs` の各関数を、それぞれの周期 [s] で呼ぶ。最初の呼び出しは1周期後。

    遅れた（イベントループが塞がれた）ときは、溜まったぶんをまとめて呼ばずに次の周期から数え直す。
    """
    loop = asyncio.get_running_loop()
    start = loop.time()
    due = [start + period for period, _ in jobs]
    while running():
        await asyncio.sleep(max(0.0, min(due) - loop.time()))
        if not running():
            return
        now = loop.time()
        for i, (period, fn) in enumerate(jobs):
            if due[i] <= now:
                fn()
                due[i] += period
                if due[i] <= now:
                    due[i] = now + period
