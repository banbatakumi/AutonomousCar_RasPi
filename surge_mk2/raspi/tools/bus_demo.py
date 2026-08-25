"""bus_demo — 車体も Pi も無い状態でバスにそれらしいデータを流す（`architecture.md` §10.6）。

    .venv/bin/python -m raspi.tools.bus_demo
    .venv/bin/python -m raspi.tools.bus_demo --duration 60 --faults

`io_node` と同じ endpoint に bind して `vehicle_state` / `scan` / `diag/link` を出す。
**下流（telemetry_node・GUI）から見て io_node と区別がつかない**のが狙い。

`cmd` を購読して舵と速度が反応するので、**GUI の操縦入力が効いていることを
実車なしで確認できる。** ここが閉じていないと「GUI で舵を切ったつもりが
どこかで落ちている」を実車の前で初めて気づくことになる。

## これはシミュレータではない

意図的に**運動学だけ**（自転車モデル + 矩形の部屋のレイキャスト）にしてある。
タイヤモデルもスリップも路面もない。**ここを育てて物理シミュレータにしないこと。**
本物のセンサノイズが要るなら `replay_node --bus` で実機ログを流す方が正しい
（`docs/architecture.md` §11）。用途は「GUI の配管と見た目を確認すること」だけ。
"""

from __future__ import annotations

import argparse
import math
import random
import signal
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.bus import LATEST, Publisher, Subscriber  # noqa: E402
from raspi.msgs import LinkDiag, Scan, VehicleState  # noqa: E402
from raspi.proto.generated.packets import PROTOCOL_VERSION  # noqa: E402
from raspi.msgs.types import (  # noqa: E402
    TOPIC_CMD,
    TOPIC_DIAG_LINK,
    TOPIC_SCAN,
    TOPIC_VEHICLE_STATE,
)
from raspi.proto import packets  # noqa: E402

NS = 1_000_000_000
STATE_HZ = 50
SCAN_HZ = 10
DIAG_HZ = 10

WHEELBASE = 0.23          # m（実測確定、2026-08-20。`architecture.md` §15-#4）
ROOM = (6.0, 4.0)         # m  部屋の内寸
#: ステアリングの1次遅れ時定数。**実測待ちの仮値**（Phase 1 で測る）
STEER_TAU_S = 0.12
SPEED_TAU_S = 0.35


class DemoVehicle:
    """自転車モデル。部屋の中をぐるぐるする。"""

    def __init__(self) -> None:
        self.x, self.y, self.th = ROOM[0] / 2, ROOM[1] / 2, 0.0
        self.v = 0.0
        self.steer = 0.0
        self.odom = [0.0, 0.0]        # FL, FR 累積 [m]
        self.t = 0.0

    def step(self, dt: float, target_v: float, target_steer: float) -> None:
        # 1次遅れ。**アクチュエータは即座には効かない**ことを GUI 上で見えるようにする
        self.steer += (target_steer - self.steer) * min(1.0, dt / STEER_TAU_S)
        self.v += (target_v - self.v) * min(1.0, dt / SPEED_TAU_S)

        self.th += self.v / WHEELBASE * math.tan(self.steer) * dt
        self.x += self.v * math.cos(self.th) * dt
        self.y += self.v * math.sin(self.th) * dt
        # 壁に入ったら跳ね返す（ここは見た目のためだけ。物理ではない）
        if not (0.3 < self.x < ROOM[0] - 0.3) or not (0.3 < self.y < ROOM[1] - 0.3):
            self.th += math.pi / 2
            self.x = min(max(self.x, 0.3), ROOM[0] - 0.3)
            self.y = min(max(self.y, 0.3), ROOM[1] - 0.3)

        # 前輪の走行距離は車体中心線距離の 1/cos(δ) 倍。**GUI 側の射影の検算になる**
        d = self.v * dt / max(1e-6, math.cos(self.steer))
        self.odom[0] += d
        self.odom[1] += d
        self.t += dt

    def raycast(self) -> list[float]:
        """部屋の壁までの距離を1°刻みで。0 は無効（欠測を混ぜる）。"""
        out = []
        for deg in range(360):
            a = self.th + math.radians(deg)
            ca, sa = math.cos(a), math.sin(a)
            best = 12.0
            for lim, comp, d in ((0.0, self.x, ca), (ROOM[0], self.x, ca),
                                 (0.0, self.y, sa), (ROOM[1], self.y, sa)):
                if abs(d) < 1e-9:
                    continue
                t = (lim - comp) / d
                if 0 < t < best:
                    best = t
            # LD06 は 1.5cm 程度のばらつき。たまに欠測する
            out.append(0.0 if random.random() < 0.02
                       else max(0.02, best + random.gauss(0, 0.015)))
        return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=None, help="秒で自動終了")
    ap.add_argument("--auto", action="store_true",
                    help="cmd が来なくても勝手に走る（配線確認用）")
    ap.add_argument("--faults", action="store_true",
                    help="20秒ごとに低電圧・過熱・E-Stop を演出する（GUI の異常表示の確認）")
    ap.add_argument("--allow-arm", action="store_true",
                    help="armed フラグを立てて返す（io_node の --allow-arm を模す）")
    args = ap.parse_args()

    pub = Publisher("io")
    sub = Subscriber({TOPIC_CMD: LATEST})
    car = DemoVehicle()
    print(f"# bus_demo → {pub.endpoint}  vehicle_state {STATE_HZ}Hz / "
          f"scan {SCAN_HZ}Hz / diag {DIAG_HZ}Hz")
    print("# cmd を購読して舵と速度が反応する。Ctrl-C で停止\n")

    running = [True]
    signal.signal(signal.SIGINT, lambda *_: running.__setitem__(0, False))

    t0 = time.monotonic()
    next_state = next_scan = next_diag = time.monotonic()
    last = t0
    cmd = None
    cmd_ns = 0
    n_state = n_scan = 0

    while running[0]:
        now = time.monotonic()
        if args.duration and now - t0 >= args.duration:
            break
        for topic, msg in sub.poll(2):
            if topic == TOPIC_CMD:
                cmd, cmd_ns = msg, time.monotonic_ns()

        fresh = cmd is not None and (time.monotonic_ns() - cmd_ns) < 150_000_000
        if fresh and cmd.mode != 0:
            tv, ts = cmd.target_speed, cmd.target_steer
        elif args.auto:
            tv, ts = 0.6, 0.35 * math.sin((now - t0) * 0.5)
        else:
            tv, ts = 0.0, 0.0

        dt = now - last
        last = now
        car.step(dt, tv, ts)

        el = now - t0
        estop = args.faults and 20 <= el % 60 < 24
        undervolt = args.faults and 40 <= el % 60 < 50

        if now >= next_state:
            n_state += 1
            flags = packets.FLG_IMU_OK | packets.FLG_LIDAR_OK \
                | packets.FLG_STEER_CENTER_VALID
            if fresh and cmd.arm and args.allow_arm and not estop:
                flags |= packets.FLG_ARMED | (cmd.mode & packets.FLG_MODE_MASK)
            if estop:
                flags |= packets.FLG_ESTOP_ACTIVE
            if undervolt:
                flags |= packets.FLG_FAULT_DRIVE_UNDERVOLTAGE

            ws = car.v / max(1e-6, math.cos(car.steer))
            pub.send(TOPIC_VEHICLE_STATE, VehicleState(
                t_capture=time.monotonic_ns(),
                speed=car.v, yaw_rate=car.v / WHEELBASE * math.tan(car.steer),
                steer_actual=car.steer, steer_cmd_echo=ts,
                wheel_speed=[ws, ws, car.v, car.v],
                odom_dist=list(car.odom),
                accel=[0.0, 0.0, 9.81], pitch=0.0, roll=0.0,
                motor_current=[abs(car.v) * 1.2, abs(car.v) * 1.2, abs(ts) * 0.8],
                torque_cmd=[abs(car.v) * 0.02, abs(car.v) * 0.02],
                temp=[32 + int(el) % 20, 33, 55 if args.faults and el % 60 > 50 else 30,
                      46],
                batt_voltage=[8.2 if undervolt else 11.4, 11.1],
                batt_current=[abs(car.v) * 2.0, 0.9],
                us_front=1.2 + 0.3 * math.sin(el), us_rear=None,
                md_status=[0x31, 0x31, 0x31], flags=flags,
                cmd_seq_echo=n_state & 0xFF,
                mode=cmd.mode if (fresh and cmd) else 0,
                armed=bool(flags & packets.FLG_ARMED),
                estop_active=estop, imu_ok=True, lidar_ok=True,
                steer_center_valid=True,
                faults=["drive_undervoltage"] if undervolt else [],
                odom_center=car.odom[0] * math.cos(car.steer),
                slip_front=[0.0, 0.0], slip_rear=[0.0, 0.0],
                stopped=abs(car.v) < 0.05,
            ))
            next_state = now + 1.0 / STATE_HZ

        if now >= next_scan:
            n_scan += 1
            t_ns = time.monotonic_ns()
            # セクタを1つ落としてみせる。GUI が「欠測」を描き分けられるかの確認
            seen = [True] * 12
            if n_scan % 7 == 0:
                seen[(n_scan // 7) % 12] = False
            dist = car.raycast()
            for i, ok in enumerate(seen):
                if not ok:
                    dist[i * 30:(i + 1) * 30] = [0.0] * 30
            pub.send(TOPIC_SCAN, Scan(
                t_capture=t_ns, dist=dist,
                sector_t_ns=[t_ns + i * 8_300_000 if seen[i] else 0 for i in range(12)],
                sector_dur_us=[8300 if s else 0 for s in seen],
                sector_seen=seen, rot_speed_dps=3594.0))
            next_scan = now + 1.0 / SCAN_HZ

        if now >= next_diag:
            pub.send(TOPIC_DIAG_LINK, LinkDiag(
                t_capture=time.monotonic_ns(),
                health="FAULT" if estop else "OK",
                estop_active=estop, drive_power_locked=False,
                arm_inhibited=not args.allow_arm,
                cmd_source=cmd.source if cmd else "", cmd_stale=not fresh,
                rx={"frame_ok": n_state * 3, "crc_error": 0, "packet_loss": 0,
                    "reordered": 0, "duplicate": 0, "resync_bytes": 0,
                    "len_error": 0, "unknown_type": 0, "wrong_direction": 0},
                stm_rx={"rx_frame_ok": n_state * 2, "rx_crc_error": 0,
                        "rx_len_error": 0, "rx_unknown_type": 0, "tx_drop": 0,
                        "md_rx_count": n_state * 6, "md_rx_error": 0},
                counts={"TELEMETRY": n_state, "LIDAR_SECTOR": n_scan * 12},
                sync_offset_ns=-1_234_567, sync_delay_ns=180_000,
                sync_drift_ppm=-12.4, sync_n=50,
                cmd_rtt_ms=11.0 + random.gauss(0, 1.5),
                protocol_version=PROTOCOL_VERSION, fw_id=0xDEADBEEF, protocol_match=True,
                hb_alive=True, hb_max_late_ms=0.46, hb_stalls=0,
                lidar_scans=n_scan, lidar_sectors_lost=n_scan // 7))
            next_diag = now + 1.0 / DIAG_HZ

        time.sleep(0.002)

    print(f"\n=== 終了 === vehicle_state={n_state} scan={n_scan} "
          f"publish={pub.sent}")
    sub.close()
    pub.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
