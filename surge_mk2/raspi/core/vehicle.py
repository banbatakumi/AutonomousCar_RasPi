"""`config/vehicle.toml` の読み手（`docs/architecture.md` §5.3）。

**全ノードがここだけを見る。** 同じ数字を2箇所に書くと必ず片方が古くなる、が §5.3 の
言い分なので、Pi 側でも読む口をひとつに絞っておく。

`sim/vehicle.py` にも同じファイルを読む `VehicleSpec` があるが、あちらは**シミュレータの
物理モデルが要る量**（時定数・転がり抵抗・駆動輪数）を持つ。こちらは**制御と知覚が要る量**
（ホイールベース・車体外形・センサ取付）だけ。`raspi/` は `sim/` を import できない
（実機に `sim/` を配らない、`raspi/nodes/io_node.py:447-453`）ので、共有はしていない。

## ★ 値のほとんどは未実測

`measured = false` の間は `wheelbase` も `wheel_radius` も暫定値（§15 未決事項 #2〜#4）。
**`measured` を見て「この数字を信じてよいか」を呼び出し側が判断できる**ようにしてある。
経路追従の精度はここが実測されるまで原理的に出ない。
"""

from __future__ import annotations

import tomllib
from dataclasses import dataclass, field
from pathlib import Path

__all__ = ["Vehicle", "DEFAULT_PATH"]

DEFAULT_PATH = Path(__file__).resolve().parents[2] / "config" / "vehicle.toml"


@dataclass(frozen=True, slots=True)
class Vehicle:
    """車両諸元のうち、制御と知覚が使うぶん。すべて SI・base_link 基準。"""

    #: 諸元が実測済みか。**False の間は経路追従の精度を云々しても意味が無い**
    measured: bool = False
    wheelbase: float = 0.25                #: L [m]
    track: float = 0.19                    #: トレッド [m]
    max_steer: float = 1.047               #: 最大路面舵角 [rad]
    wheel_radius: float = 0.033            #: [m]
    #: 車体外形ポリゴン [m]。base_link 基準
    footprint: tuple[tuple[float, float], ...] = ()
    #: LiDAR の取付位置 [m]。**base_link（後輪車軸中心）より前に出ている**
    lidar_x: float = 0.0
    lidar_y: float = 0.0
    lidar_yaw: float = 0.0                 #: [rad]
    #: 操舵のむだ時間と1次遅れ [s]。Pure Pursuit の遅延補償の既定値になる
    dead_time_s: float = 0.030
    tau_steer_s: float = 0.12
    _path: Path | None = field(default=None, compare=False)

    # ── 派生量 ──

    @property
    def half_width(self) -> float:
        """車体半幅 [m]。**外形ポリゴンの |y| の最大値**（トレッドではない）。

        レーシングラインの余裕はタイヤ間隔ではなく**外形**で決まる。
        トレッドで計算するとバンパーの張り出しぶん壁に寄る。
        """
        if self.footprint:
            return max(abs(y) for _, y in self.footprint)
        return self.track / 2.0

    @property
    def front_overhang(self) -> float:
        """base_link から前端までの距離 [m]。停止距離の判定に使う。"""
        if self.footprint:
            return max(x for x, _ in self.footprint)
        return self.wheelbase

    @classmethod
    def load(cls, path: str | Path | None = None) -> "Vehicle":
        """読めなければ**既定値で返す**（例外にしない）。

        諸元が読めないことは「走ってはいけない」ではなく「暫定値で走る」。
        暫定値であることは `measured=False` が伝えるので、判断は呼び出し側に任せる。
        """
        p = Path(path) if path else DEFAULT_PATH
        try:
            with open(p, "rb") as fp:
                d = tomllib.load(fp)
        except (OSError, tomllib.TOMLDecodeError):
            return cls()

        dyn = d.get("dynamics", {})
        lidar = d.get("sensors", {}).get("lidar", {})
        fp_raw = d.get("footprint", []) or []
        return cls(
            measured=bool(d.get("measured", False)),
            wheelbase=float(d.get("wheelbase", 0.25)),
            track=float(d.get("track", 0.19)),
            max_steer=float(d.get("max_steer", 1.047)),
            wheel_radius=float(d.get("wheel_radius", 0.033)),
            footprint=tuple((float(v[0]), float(v[1])) for v in fp_raw if len(v) >= 2),
            lidar_x=float(lidar.get("x", 0.0)),
            lidar_y=float(lidar.get("y", 0.0)),
            lidar_yaw=float(lidar.get("yaw", 0.0)),
            dead_time_s=float(dyn.get("dead_time_s", 0.030)),
            tau_steer_s=float(dyn.get("tau_steer_s", 0.12)),
            _path=p,
        )
