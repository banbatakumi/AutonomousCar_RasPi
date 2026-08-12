"""仮想 LD06 — レイキャスト・距離ノイズ・欠損・遅延。

実機の LD06 は STM32 に直結され、STM32 が 1°ビニングまで済ませて `LIDAR_SECTOR` を
UART で送る（`docs/architecture.md` §4）。**Pi 側に LD06 の生パースは無い**ので、
シミュレータもセクタパケットを作るところから始める。

## セクタ単位（120Hz）で作る理由

1周 360点を一度に作ると、走査中に車体が動いている効果（モーションスミア）が消える。
実機の LD06 は 10回転/秒なので、1周する間に 3m/s の車は 30cm 進む。
セクタごとにその時刻の姿勢でレイを撃てば、この歪みが自然に入る。

## 出力はセンサ座標系（＝わざと鏡像にする）

実機の LD06 は裏向きに実装されていて、`ScanAssembler` が
`車両角 = (360 − センサ角) % 360` で戻している（`raspi/msgs/convert.py:281-298`）。

**シミュレータは反転した状態で出さなければならない。** 素直に車両座標で出すと、
反転処理が壊れていても画面が正しく見えてしまい、バグを永久に検出できない。

## 欠損は2種類ある

- **点単位**（黒い物体・鋭い入射角）→ `dist = 0`
- **セクタ単位**（パケットが落ちる）→ **パケットごと出さない**

後者を入れないと `ScanAssembler` の `sector_seen=False` 経路（GUI の欠測ハッチ表示）が
一度も踏まれない。「表示を作ったが一度も出たことがない」を避けるために要る。
"""

from __future__ import annotations

import math

import numpy as np

from raspi.proto import packets

from .params import SimParams
from .vehicle import VehicleSpec

__all__ = ["VirtualLidar", "SECTORS", "POINTS_PER_SECTOR", "SCAN_HZ"]

NS = 1_000_000_000

SECTORS = 12
POINTS_PER_SECTOR = 30
SCAN_HZ = 10                                  #: 回転数 [rev/s]
SECTOR_HZ = SCAN_HZ * SECTORS                 #: 120 Hz
SECTOR_PERIOD_NS = NS // SECTOR_HZ
ROT_SPEED_DPS = 360 * SCAN_HZ                 #: 3600 deg/s

#: セクタ s の点 i が向く**車両角** [deg]。センサ角 (s*30+i) の鏡像。
#: `convert.py` の `_DEG` と同じ表を、こちらは「作る側」として持つ
_VEHICLE_DEG = np.array(
    [[(360 - s * 30 - i) % 360 for i in range(POINTS_PER_SECTOR)] for s in range(SECTORS)],
    dtype=np.float64)
_VEHICLE_RAD = np.deg2rad(_VEHICLE_DEG)


class VirtualLidar:
    """:param course: 走行中に差し替えられる（`VirtualStm32.set_course`）"""

    def __init__(self, course, spec: VehicleSpec, params: SimParams,
                 seed: int = 0) -> None:
        self.course = course
        self.spec = spec
        self.params = params
        self.rng = np.random.default_rng(seed)
        self.mount = spec.sensor_pose("lidar")

        self._next_ns = 0
        self._sector = 0
        #: 出さなかったセクタの数。`lidar_sectors_lost` と突き合わせる用
        self.dropped_sectors = 0
        #: **車両角**で並べた直近の観測 [m]。0 = 無効。シミュレータGUI が描くためだけの物で、
        #: UART には流れない（Pi 側はセクタパケットから `ScanAssembler` が組み立てる）
        self.last_scan = np.zeros(360)

    def poll(self, t_ns: int, vehicle, stm_us) -> list[tuple[int, packets.LidarSector]]:
        """`t_ns` までに走査が終わったセクタを返す。

        戻り値の `gen_ns` は **Pi に届いてよい時刻**（伝送遅延を足したもの）。
        送信キューに積むのは呼び出し側（`VirtualStm32._emit`）の仕事。
        """
        if self._next_ns == 0:
            self._next_ns = t_ns + SECTOR_PERIOD_NS
            return []

        out: list[tuple[int, packets.LidarSector]] = []
        # 一度に作りすぎない。長く止まっていた後に何百セクタも吐くと帯域が破綻する
        for _ in range(SECTORS * 2):
            if t_ns < self._next_ns:
                break
            t_end = self._next_ns
            idx = self._sector
            self._sector = (self._sector + 1) % SECTORS
            self._next_ns += SECTOR_PERIOD_NS

            if self.rng.random() < self.params.lidar_sector_drop_rate:
                self.dropped_sectors += 1
                continue                      # ★ パケットごと出さない = sector_seen False

            pkt = self._sector_packet(idx, t_end, vehicle, stm_us)
            out.append((t_end + self._delay_ns(), pkt))
        return out

    # ── 1セクタ分 ──

    def _sector_packet(self, idx: int, t_end_ns: int, vehicle,
                       stm_us) -> packets.LidarSector:
        p = self.params
        # センサ位置（車体座標 → 世界座標）
        mx, my, myaw = self.mount
        c, s = math.cos(vehicle.yaw), math.sin(vehicle.yaw)
        ox = vehicle.x + mx * c - my * s
        oy = vehicle.y + mx * s + my * c

        # **車両角で撃つ。並びはセンサ順のまま**（＝配列の添字が鏡像になっている）
        angles = vehicle.yaw + myaw + _VEHICLE_RAD[idx]
        d = self.course.raycast(ox, oy, angles, p.lidar_max_range_m)

        valid = d > 0
        if p.lidar_noise_sigma_m > 0 or p.lidar_noise_rel > 0:
            sigma = p.lidar_noise_sigma_m + p.lidar_noise_rel * d
            d = d + self.rng.normal(0.0, 1.0, d.shape) * sigma
        # 近すぎ・遠すぎ・欠損はすべて 0（プロトコルの「無効」）
        valid &= (d >= p.lidar_min_range_m) & (d <= p.lidar_max_range_m)
        if p.lidar_drop_rate > 0:
            valid &= self.rng.random(d.shape) >= p.lidar_drop_rate
        mm = np.where(valid, np.clip(d * 1000.0, 0, 65535), 0).astype(np.int64)
        self.last_scan[_VEHICLE_DEG[idx].astype(np.int32)] = mm / 1000.0

        t_start_us = stm_us(t_end_ns - SECTOR_PERIOD_NS)
        return packets.LidarSector(
            sector_idx=idx,
            t_start_us=t_start_us,
            duration_us=SECTOR_PERIOD_NS // 1000,
            rot_speed_dps=int(ROT_SPEED_DPS + self.rng.normal(0, 8)),
            dist=[int(v) for v in mm],
        )

    def _delay_ns(self) -> int:
        p = self.params
        ms = p.lidar_delay_ms
        if p.lidar_delay_jitter_ms > 0:
            ms += abs(self.rng.normal(0.0, p.lidar_delay_jitter_ms))
        return int(max(0.0, ms) * 1e6)
