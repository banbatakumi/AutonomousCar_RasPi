"""ml_lidar/sim_support.py — `env.py`・`watch.py`が共通で使う、LiDARシムの小さな配線。

どちらも`sim.lidar.VirtualLidar`＋`raspi.msgs.convert.ScanAssembler`という同じ
中間案（UART/STM32バイトフレーミング層は経由しない）を土台にしているため、
そこだけ切り出してある。行動→物理量の変換や報酬計算はそれぞれ別物なので、
ここには置かない。
"""

from __future__ import annotations

from typing import Callable

__all__ = ["stm_us", "pump_lidar_until_scan"]


def stm_us(t_ns: int) -> int:
    """`VirtualLidar.poll()`が要求する時刻変換。時刻同期・クロックドリフトは
    UART層の関心事なので、ここでは単純な ns→us 切り捨てにする（中間案）。
    """
    return t_ns // 1000


def pump_lidar_until_scan(step: Callable[[], object | None], physics_substep: float,
                          max_pump_s: float = 0.25) -> None:
    """`reset()`直後、車体は動かさずスキャンが1周ぶん組み上がるまで`step()`を呼び続ける。

    :param step: 時計を1サブステップ進めてLiDARをポーリングし、組み上がった
        スキャン（無ければ`None`）を返す呼び出し側のクロージャ
    """
    max_iters = int(max_pump_s / physics_substep) + 1
    for _ in range(max_iters):
        if step() is not None:
            return
    raise RuntimeError("LiDARスキャンが1周ぶんも組み上がらなかった（実装バグの疑い）")
