"""シミュレータのパラメータ。**実機に対応物が無い量だけをここに置く。**

車両そのものの諸元（ホイールベース・操舵の時定数など）は `config/vehicle.toml` にある。
こちらは「シミュレータがどれくらい意地悪をするか」の設定で、実車には存在しない概念。

すべて `sim.gui` から走行中に変更できる（`TUNABLES` がそのメタデータ）。
"""

from __future__ import annotations

from dataclasses import dataclass, fields
from typing import NamedTuple

__all__ = ["SimParams", "TUNABLES", "Tunable"]


@dataclass
class SimParams:
    """既定値は「実機で観測された程度の意地悪さ」を狙ってある。"""

    # ── 仮想 LD06 ──
    #: 距離ノイズ σ = noise_sigma_m + noise_rel × 距離 [m]
    lidar_noise_sigma_m: float = 0.010
    lidar_noise_rel: float = 0.005
    #: 点単位のドロップ率。黒い物体・鋭い入射角で実機は返ってこない
    lidar_drop_rate: float = 0.02
    #: **セクタ単位**の脱落率。`ScanAssembler` の `sector_seen=False` 経路を踏ませるために要る。
    #: 点ドロップだけだと「欠測セクタのハッチ表示」が一生確認できない
    lidar_sector_drop_rate: float = 0.005
    #: セクタ生成から Pi に届くまでの伝送遅延 [ms]（UART の線上時間とは別）
    lidar_delay_ms: float = 5.0
    lidar_delay_jitter_ms: float = 2.0
    lidar_max_range_m: float = 12.0
    lidar_min_range_m: float = 0.02

    # ── 超音波 ──
    us_noise_sigma_m: float = 0.005
    us_max_range_m: float = 4.0

    # ── リンク ──
    #: フレームのバイトを化けさせる確率（1バイトあたり）。CRC エラーの表示確認用。
    #: 既定 0 ＝ 完全な回線。0 でないと「CRC エラー 0 が正常」の判定ができなくなる
    uart_bit_error_rate: float = 0.0
    #: 仮想 STM32 の時計のドリフト [ppm]。既定値は実車の実測値（PROGRESS.md）。
    #: 0 にすると時刻同期コードが仕事をしているか分からなくなる
    stm_clock_ppm: float = 3378.0

    def copy(self) -> "SimParams":
        return SimParams(**{f.name: getattr(self, f.name) for f in fields(self)})


class Tunable(NamedTuple):
    """`sim.gui` の調整パネル1行ぶん。"""

    key: str
    group: str
    label: str
    lo: float
    hi: float
    step: float
    fmt: str


#: 調整パネルに出す項目と範囲。ここに無い項目は起動時固定
TUNABLES: tuple[Tunable, ...] = (
    Tunable("lidar_noise_sigma_m", "LIDAR", "noise σ", 0.0, 0.20, 0.005, "{:.3f} m"),
    Tunable("lidar_noise_rel", "LIDAR", "noise 比例", 0.0, 0.05, 0.001, "{:.3f}"),
    Tunable("lidar_drop_rate", "LIDAR", "drop", 0.0, 0.50, 0.01, "{:.0%}"),
    Tunable("lidar_sector_drop_rate", "LIDAR", "sector drop", 0.0, 0.30, 0.005, "{:.1%}"),
    Tunable("lidar_delay_ms", "LIDAR", "delay", 0.0, 100.0, 1.0, "{:.0f} ms"),
    Tunable("lidar_delay_jitter_ms", "LIDAR", "jitter", 0.0, 50.0, 1.0, "{:.0f} ms"),
    Tunable("lidar_max_range_m", "LIDAR", "max range", 1.0, 12.0, 0.5, "{:.1f} m"),
    Tunable("uart_bit_error_rate", "LINK", "bit error", 0.0, 1e-3, 1e-5, "{:.1e}"),
    Tunable("stm_clock_ppm", "LINK", "clock drift", -20000, 20000, 100, "{:.0f} ppm"),
)
