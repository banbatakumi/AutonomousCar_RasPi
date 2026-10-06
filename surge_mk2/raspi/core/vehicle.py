"""`config/vehicle.toml` の読み手（`docs/architecture.md` §5.3）。

**全ノードがここだけを見る。** 同じ数字を2箇所に書くと必ず片方が古くなる、が §5.3 の
言い分なので、Pi 側でも読む口をひとつに絞っておく。

`sim/vehicle.py` にも同じファイルを読む `VehicleSpec` があるが、あちらは**シミュレータの
物理モデルが要る量**（時定数・転がり抵抗・駆動輪数）を持つ。こちらは**制御と知覚が要る量**
（ホイールベース・車体外形・センサ取付）だけ。`raspi/` は `sim/` を import できない
（実機に `sim/` を配らない、`raspi/nodes/io_node.py:447-453`）ので、共有はしていない。

## ★ 実測が残っているのはアクチュエータの動特性だけ

`wheelbase`・`track`・ステアのリンク比（`max_steer` に反映済み）・`wheel_radius`・
車体外形・センサ取付は 2026-08-20 に実測確定済み（§15 #2〜#4。後輪はダイレクト
ドライブでギア比の概念が無いため #3 は車輪半径だけで確定）。残るは `[dynamics]`
（操舵のむだ時間・1次遅れなど）だけ。
**`measured` を見て「この数字を信じてよいか」を呼び出し側が判断できる**ようにしてある。

## カメラのレンズは「プロファイル」を1行で選ぶ（2026-10-03）

純正レンズ（≒66°）と IMX219 160° 広角を付け替えられるように、レンズで変わる値
（`hfov`・`bottom_crop`・`undistort_hfov`・魚眼の校正値 `fisheye`）は
`[sensors.cam_front.lenses.<名前>]` に分けて持ち、`[sensors.cam_front]` の
`lens = "<名前>"` で選ぶ。取付位置・姿勢（x/y/z/pitch/yaw）はレンズに依らないので
カメラの表に残す。解決は `resolve_lens()` の1か所だけ（`config/generate.py` も使う）。
"""

from __future__ import annotations

import math
import tomllib
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

from .camera_model import FisheyeCalib

__all__ = ["Vehicle", "DEFAULT_PATH", "resolve_lens", "LENS_NAME_RE"]

DEFAULT_PATH = Path(__file__).resolve().parents[2] / "config" / "vehicle.toml"

#: レンズ名に使える文字。📷 のファイル名（`surge_front_<lens>_640x360_…png`）に
#: 入るので `_` は使わせない（校正ツールが名前を区切れなくなる）
LENS_NAME_RE = re.compile(r"^[A-Za-z0-9-]+$")


def resolve_lens(cam: dict, *, strict: bool = False) -> dict:
    """カメラの表（`[sensors.cam_front]` 等）から、選ばれたレンズの値を重ねた dict を作る。

    `lens` が無ければ表をそのまま返す（レンズのプロファイルを使わない古い書き方も読める）。
    `lens` が `lenses` に無いときは、`strict` なら `ValueError`（生成器・校正ツール用。
    打ち間違いを黙って通さない）、そうでなければ警告してプロファイル無しで続ける
    （ノード用。諸元が読めなくても「暫定値で走る」という `Vehicle.load` の方針に合わせる）。
    返り値の `lens` には選ばれた名前（無ければ ""）が入る。
    """
    base = {k: v for k, v in cam.items() if k not in ("lens", "lenses")}
    name = cam.get("lens")
    if not name:
        base["lens"] = ""
        return base
    lenses = cam.get("lenses") or {}
    if name not in lenses:
        msg = (f"vehicle.toml: lens = \"{name}\" のプロファイルが無い"
               f"（あるのは {sorted(lenses)}）")
        if strict:
            raise ValueError(msg)
        print(f"!! {msg}。レンズ固有の値は既定値で動く", file=sys.stderr)
        base["lens"] = str(name)
        return base
    base.update(lenses[name])
    base["lens"] = str(name)
    return base


@dataclass(frozen=True, slots=True)
class Vehicle:
    """車両諸元のうち、制御と知覚が使うぶん。すべて SI・base_link 基準。"""

    #: 諸元が実測済みか。**False の間は経路追従の精度を云々しても意味が無い**
    measured: bool = False
    wheelbase: float = 0.23                #: L [m]（実測確定）
    track: float = 0.155                   #: トレッド [m]（実測確定）
    max_steer: float = 0.524               #: 最大路面舵角 [rad] = 30°（リンク比 0.5 実測確定）
    wheel_radius: float = 0.03             #: [m]（実測確定）
    #: 車体外形ポリゴン [m]。base_link 基準
    footprint: tuple[tuple[float, float], ...] = ()
    #: LiDAR の取付位置 [m]。**base_link（後輪車軸中心）より前に出ている**
    lidar_x: float = 0.0
    lidar_y: float = 0.0
    lidar_yaw: float = 0.0                 #: [rad]
    #: カメラの下端カット率（ボンネット等の映り込み除去。`camera_node.py` の ISP
    #: ScalerCrop が実際の読み出しに反映する）。GUI の進路ガイドの主点補正にも使う
    cam_front_bottom_crop: float = 0.0
    cam_rear_bottom_crop: float = 0.0
    #: 付いているレンズのプロファイル名（`[sensors.cam_*] lens`）。プロファイルを
    #: 使わない書き方なら ""。📷 のファイル名と校正ツールの書き込み先に使う
    cam_front_lens: str = ""
    cam_rear_lens: str = ""
    #: レンズ用の ISP チューニングの差分（`config/cam_tuning/` のファイル名、プロファイルの
    #: `isp_tuning`）。"" ＝ センサー標準のまま。`raspi/core/cam_tuning.py` が標準へ重ねる
    cam_front_isp_tuning: str = ""
    cam_rear_isp_tuning: str = ""
    #: カメラの取付位置・姿勢・画角。**IPM（`raspi/nav/ipm.py`）と `CameraView.tsx` の
    #: 進路ガイドが同じ値を見る。** `pitch` は下向きが正（`vehicle.toml` の注記どおり）で、
    #: 取り付け角度そのもの。走行中の車体姿勢ぶんは `vs.pitch`（IMU実測）で別途補正する
    cam_front_x: float = 0.097
    cam_front_y: float = 0.0
    cam_front_z: float = 0.09
    cam_front_pitch: float = 0.0           #: [rad] 下向きが正
    cam_front_yaw: float = 0.0             #: [rad]
    cam_front_hfov: float = 1.152          #: [rad]
    cam_rear_x: float = 0.044
    cam_rear_y: float = 0.0
    cam_rear_z: float = 0.09
    cam_rear_pitch: float = 0.0
    cam_rear_yaw: float = 3.14159265
    cam_rear_hfov: float = 1.152
    #: 魚眼レンズの校正値（選んだレンズの `fisheye`、`tools/cam_calib/` が書く）。
    #: **None ＝ 未校正**で、IPM・進路ガイドは従来どおり `hfov` のピンホールで近似する
    cam_front_fisheye: FisheyeCalib | None = None
    cam_rear_fisheye: FisheyeCalib | None = None
    #: 補正映像（魚眼 → 仮想ピンホール、`telemetry_node`）の水平画角 [rad]。
    #: 大きいほど広く写るが端が引き伸ばされる（ピンホールは 180° に届かない）
    cam_front_undistort_hfov: float = 1.92
    cam_rear_undistort_hfov: float = 1.92
    #: 操舵のむだ時間と1次遅れ [s]。Pure Pursuit の遅延補償の既定値になる
    dead_time_s: float = 0.030
    tau_steer_s: float = 0.12
    #: 舵の効き（`[dynamics]`、システム同定の値）。実舵角 = `steer_servo_gain`×指令、
    #: 実効舵角 = `steer_gain`·d + `steer_gain_cubic`·d³ + `steer_offset_rad`（d = 実舵角）。
    #: **`kappa_max` がここから最小旋回半径を出す**（`sim/vehicle.py` の曲率と同じ式）
    steer_servo_gain: float = 1.0
    steer_gain: float = 1.0
    steer_gain_cubic: float = 0.0
    steer_offset_rad: float = 0.0
    #: 指令が途絶してから DISARM に落とすまで [ms]（`docs/architecture.md` §9.4）。
    #: **telemetry_node と io_node が独立に持つが、値はここ 1 つ**
    cmd_deadman_ms: float = 150.0
    #: 自律走行中だけ、GUI からの `cmd` の途絶をここまで待つ [ms]（`vehicle.toml` の説明を参照）
    auto_link_grace_ms: float = 5000.0
    #: `auto/cmd` がこれだけ古ければ中継せず制動に読み替える [ms]
    auto_cmd_stale_ms: float = 200.0
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

    @property
    def rear_overhang(self) -> float:
        """base_link から後端までの距離 [m]。後退時の停止距離判定に使う。

        `front_overhang` と対称（`park_to_point.py` の後退時の安全距離計算）。
        """
        if self.footprint:
            return -min(x for x, _ in self.footprint)
        return 0.0

    @property
    def kappa_max(self) -> float:
        """車の曲がれる限界の曲率 [1/m]（最小旋回半径の逆数）。

        `tan(max_steer)/wheelbase` ではなく、**同定した舵の効きを通す**: 指令 `max_steer` で
        届く実舵角 → 実効舵角 → 曲率。オフセットは左右で不利な側に取る。幾何だけだと 40cm、
        同定値では 46cm で、40cm のつもりで引いた経路は曲がり切れなかった（2026-10-06 の実車）。
        """
        d = self.steer_servo_gain * self.max_steer
        delta = self.steer_gain * d + self.steer_gain_cubic * d ** 3 - abs(self.steer_offset_rad)
        return math.tan(max(delta, 1e-3)) / self.wheelbase

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
        safety = d.get("safety", {})
        sensors = d.get("sensors", {})
        lidar = sensors.get("lidar", {})
        cam_front = resolve_lens(sensors.get("cam_front", {}))
        cam_rear = resolve_lens(sensors.get("cam_rear", {}))
        fp_raw = d.get("footprint", []) or []
        return cls(
            measured=bool(d.get("measured", False)),
            wheelbase=float(d.get("wheelbase", 0.23)),
            track=float(d.get("track", 0.155)),
            max_steer=float(d.get("max_steer", 0.524)),
            wheel_radius=float(d.get("wheel_radius", 0.03)),
            footprint=tuple((float(v[0]), float(v[1])) for v in fp_raw if len(v) >= 2),
            lidar_x=float(lidar.get("x", 0.0)),
            lidar_y=float(lidar.get("y", 0.0)),
            lidar_yaw=float(lidar.get("yaw", 0.0)),
            cam_front_bottom_crop=float(cam_front.get("bottom_crop", 0.0)),
            cam_rear_bottom_crop=float(cam_rear.get("bottom_crop", 0.0)),
            cam_front_lens=cam_front["lens"],
            cam_rear_lens=cam_rear["lens"],
            cam_front_isp_tuning=str(cam_front.get("isp_tuning", "")),
            cam_rear_isp_tuning=str(cam_rear.get("isp_tuning", "")),
            cam_front_x=float(cam_front.get("x", 0.097)),
            cam_front_y=float(cam_front.get("y", 0.0)),
            cam_front_z=float(cam_front.get("z", 0.09)),
            cam_front_pitch=float(cam_front.get("pitch", 0.0)),
            cam_front_yaw=float(cam_front.get("yaw", 0.0)),
            cam_front_hfov=float(cam_front.get("hfov", 1.152)),
            cam_rear_x=float(cam_rear.get("x", 0.044)),
            cam_rear_y=float(cam_rear.get("y", 0.0)),
            cam_rear_z=float(cam_rear.get("z", 0.09)),
            cam_rear_pitch=float(cam_rear.get("pitch", 0.0)),
            cam_rear_yaw=float(cam_rear.get("yaw", 3.14159265)),
            cam_rear_hfov=float(cam_rear.get("hfov", 1.152)),
            cam_front_fisheye=FisheyeCalib.from_dict(cam_front.get("fisheye") or {}),
            cam_rear_fisheye=FisheyeCalib.from_dict(cam_rear.get("fisheye") or {}),
            cam_front_undistort_hfov=float(cam_front.get("undistort_hfov", 1.92)),
            cam_rear_undistort_hfov=float(cam_rear.get("undistort_hfov", 1.92)),
            dead_time_s=float(dyn.get("dead_time_s", 0.030)),
            tau_steer_s=float(dyn.get("tau_steer_s", 0.12)),
            steer_servo_gain=float(dyn.get("steer_servo_gain", 1.0)),
            steer_gain=float(dyn.get("steer_gain", 1.0)),
            steer_gain_cubic=float(dyn.get("steer_gain_cubic", 0.0)),
            steer_offset_rad=float(dyn.get("steer_offset_rad", 0.0)),
            cmd_deadman_ms=float(safety.get("cmd_deadman_ms", 150.0)),
            auto_link_grace_ms=float(safety.get("auto_link_grace_ms", 5000.0)),
            auto_cmd_stale_ms=float(safety.get("auto_cmd_stale_ms", 200.0)),
            _path=p,
        )
