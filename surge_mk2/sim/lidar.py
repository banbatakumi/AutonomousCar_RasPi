"""仮想 LD06 — レイキャスト・距離ノイズ・欠損・遅延。

実機の LD06 は STM32 に直結され、STM32 が 1°ビニングまで済ませて `LIDAR_SECTOR` を
UART で送る（`docs/architecture.md` §4）。**Pi 側に LD06 の生パースは無い**ので、
シミュレータもセクタパケットを作るところから始める。

## セクタ単位（120Hz）で作る理由

1周 360点を一度に作ると、走査中に車体が動いている効果（モーションスミア）が消える。
実機の LD06 は 10回転/秒なので、1周する間に 3m/s の車は 30cm 進む。
セクタごとにその時刻の姿勢でレイを撃てば、この歪みが自然に入る。

**セクタの中でも点ごとに時刻が違う**（1セクタ 30点・約 8.3ms、3m/s なら 2.5cm 進む）。
実機の点 i は `t_start + duration * i / 29`（`raspi/nav/deskew.py` の `point_times_ns` と同じ式）
に測られるので、点ごとに「セクタ始まりと終わりの姿勢の補間」から撃つ。補間する添字 i は
**センサ順**（鏡像を戻す前）で、`t_start` が先頭の点（i=0）、末尾の点（i=29）が `t_end`。

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
        #: 直前のセクタ終端の姿勢 (x, y, yaw)。次のセクタの始まりの姿勢（点ごとの補間に使う）。
        #: yaw は巻き戻さず連続量で持つ
        self._prev_pose: tuple[float, float, float] | None = None
        #: 出さなかったセクタの数。`lidar_sectors_lost` と突き合わせる用
        self.dropped_sectors = 0
        #: **車両角**で並べた直近の観測 [m]。0 = 無効。シミュレータGUI が描くためだけの物で、
        #: UART には流れない（Pi 側はセクタパケットから `ScanAssembler` が組み立てる）
        self.last_scan = np.zeros(360)
        #: 直近にセクタを作った時刻 [ns]（0 = まだ無い）。仮想 STM32 の自動停止が「LiDAR が生きているか」に使う
        self.last_emit_ns = 0

    def power_off(self) -> None:
        """無給電にする（回転が止まり、点群は古くなる）。次の `poll` から回転を始め直す。"""
        self._next_ns = 0
        self._prev_pose = None
        self.last_emit_ns = 0
        self.last_scan[:] = 0.0

    def poll(self, t_ns: int, vehicle, stm_us) -> list[tuple[int, packets.LidarSector]]:
        """`t_ns` までに走査が終わったセクタを返す。

        戻り値の `gen_ns` は **Pi に届いてよい時刻**（伝送遅延を足したもの）。
        送信キューに積むのは呼び出し側（`VirtualStm32._emit`）の仕事。
        """
        if self._next_ns == 0:
            self._next_ns = t_ns + SECTOR_PERIOD_NS
            self._prev_pose = (vehicle.x, vehicle.y, vehicle.yaw)
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
            # 落とすセクタでも姿勢の履歴は進める（次のセクタの始まりの姿勢になる）
            pose0 = self._prev_pose
            self._prev_pose = (vehicle.x, vehicle.y, vehicle.yaw)

            if self.rng.random() < self.params.lidar_sector_drop_rate:
                self.dropped_sectors += 1
                continue                      # ★ パケットごと出さない = sector_seen False

            pkt = self._sector_packet(idx, t_end, vehicle, stm_us, pose0)
            self.last_emit_ns = t_end
            out.append((t_end + self._delay_ns(), pkt))
        return out

    # ── 1セクタ分 ──

    def _sector_packet(self, idx: int, t_end_ns: int, vehicle, stm_us,
                       pose0=None) -> packets.LidarSector:
        p = self.params
        mx, my, myaw = self.mount
        x1, y1, yaw1 = vehicle.x, vehicle.y, vehicle.yaw
        if pose0 is None:
            pose0 = (x1, y1, yaw1)
        x0, y0, yaw0 = pose0
        # 点 i（センサ順）の時刻は t_start + dur * i / 29。始まり（i=0）と終わり（i=29）の
        # 姿勢を線形補間する。yaw は差を [-π, π) に畳んでから補間する
        f = np.arange(POINTS_PER_SECTOR) / (POINTS_PER_SECTOR - 1)
        dyaw = (yaw1 - yaw0 + math.pi) % (2 * math.pi) - math.pi
        px = x0 + (x1 - x0) * f
        py = y0 + (y1 - y0) * f
        pyaw = yaw0 + dyaw * f
        c, s = np.cos(pyaw), np.sin(pyaw)
        # センサ位置（車体座標 → 世界座標）
        ox = px + mx * c - my * s
        oy = py + mx * s + my * c

        # **車両角で撃つ。並びはセンサ順のまま**（＝配列の添字が鏡像になっている）
        angles = pyaw + myaw + _VEHICLE_RAD[idx]
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
            dist=mm.tolist(),
        )

    def _delay_ns(self) -> int:
        p = self.params
        ms = p.lidar_delay_ms
        if p.lidar_delay_jitter_ms > 0:
            ms += abs(self.rng.normal(0.0, p.lidar_delay_jitter_ms))
        return int(max(0.0, ms) * 1e6)
