"""`config/vehicle.toml` の読み手（`docs/architecture.md` §5.3）。

**全ノードがここだけを見る。** 同じ数字を2箇所に書くと必ず片方が古くなる、が §5.3 の
言い分なので、Pi 側でも読む口をひとつに絞っておく。

`sim/vehicle.py` にも同じファイルを読む `VehicleSpec` があるが、あちらは**シミュレータの
物理モデルが要る量**（時定数・転がり抵抗・駆動輪数）を持つ。こちらは**制御と知覚が要る量**
（ホイールベース・車体外形・センサ取付）だけ。`raspi/` は `sim/` を import できない
（実機に `sim/` を配らない、`raspi/nodes/io_node.py:447-453`）ので、共有はしていない。

## ★ 実測が残っているのはアクチュエータの動特性だけ

`wheelbase`・`track`・ステアのリンク比（`max_steer` に反映済み）・`wheel_radius`・
車体外形・センサ取付は実測確定済み（後輪はダイレクト
ドライブでギア比の概念が無く、車輪半径だけで確定）。残るは `[dynamics]`
（操舵のむだ時間・1次遅れなど）だけ。
**`measured` を見て「この数字を信じてよいか」を呼び出し側が判断できる**ようにしてある。

## カメラのレンズは「プロファイル」を1行で選ぶ

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
GRAVITY = 9.81

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



def _ramp_max(dyn: dict, safety: dict) -> float:
    """目標速度のランプの上限 [m/s²]。`[safety] cmd_accel_limit_m_s2`（v0.22〜、Pi が送る上限）。

    旧キー `[dynamics] speed_ramp_max_m_s2`（STM32 の固定上限だった頃の値）が残っている toml は
    そちらを読む。どちらも無ければ 3.0（STM32 が `accel_limit=0` のときに使う既定の傾き）。
    """
    if "speed_ramp_max_m_s2" in dyn:
        return float(dyn["speed_ramp_max_m_s2"])
    v = float(safety.get("cmd_accel_limit_m_s2", 0.0))
    return v if v > 0.0 else 3.0


#: `cmd_deadman_ms` として受け付ける範囲 [ms]。下は指令の周期（50Hz = 20ms）、上は「途絶してから
#: 止め始めるまで」として意味のある長さ
_DEADMAN_MS_RANGE = (20.0, 1000.0)


def _deadman_ms(value) -> float:
    """`[safety] cmd_deadman_ms`。範囲の外・数でない値は既定の 150ms にする。

    0 や負では指令が常に途絶扱いになって走れず、桁を間違えた大きな値（150000 など）では
    デッドマンが実質無くなる。どちらも黙って採らない。
    """
    v = float(value)
    lo, hi = _DEADMAN_MS_RANGE
    if not (lo <= v <= hi):          # NaN もここで落ちる
        print(f"!! vehicle.toml: cmd_deadman_ms = {value!r} は範囲外（{lo:g}〜{hi:g}ms）。"
              "150ms で動く", file=sys.stderr)
        return 150.0
    return v


@dataclass(frozen=True, slots=True)
class Vehicle:
    """車両諸元のうち、制御と知覚が使うぶん。すべて SI・base_link 基準。"""

    #: 諸元が実測済みか。**False の間は経路追従の精度を云々しても意味が無い**
    measured: bool = False
    wheelbase: float = 0.23                #: L [m]（実測確定）
    track: float = 0.155                   #: トレッド [m]（実測確定）
    #: 最大路面舵角 [rad]。**正は `vehicle.toml` の `max_steer`**（★v0.18 から、同定したリンクの
    #: 換算を通した値で現在 0.4636 = 26.6°。STM32 の `LIMITS.max_steer_rad` と同じ）。
    #: この既定 0.524 は toml にキーが無いときの値で、換算なし（モータ角 ±60° × リンク比 0.5 = 30°）
    max_steer: float = 0.524
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
    #: STM32 のステアのリンクの換算 `(steer_link_gain, steer_link_cubic)`（`[control]`、★プロトコル v0.18）。
    #: 路面舵角 = gain·x + cubic·x³、x = モータ角×0.5。**舵角の数値の意味はこれで決まる**——
    #: 別の換算のもとで学習したモデルの舵角は `control_params.steer_relink` で直す
    steer_link: tuple[float, float] = (1.0, 0.0)
    # ── 走りの限界（`[limits]`。**経路の速度と減速開始点はここから決める**。走行中の学習の初期値）──
    #: 横・加速・減速の上限 [m/s²]（正味。`docs/calibration.md` §4.7・§4.8 の校正 7・8 が書く）。0 = 未校正
    lim_lat_m_s2: float = 0.0
    lim_accel_m_s2: float = 0.0
    lim_brake_m_s2: float = 0.0
    rolling_resistance: float = 0.0        #: 転がり抵抗による減速度 [m/s²]
    #: STM32 の速度PI（ファームの定数）と車体の利得 [(m/s²)/(N·m)]。**速度指令の前倒しに使う**
    speed_kp: float = 0.0
    speed_ki: float = 0.0
    speed_plant_gain: float = 0.0
    #: 目標速度をならす傾きの上限 [m/s²]。**`[safety] cmd_accel_limit_m_s2`**（Pi が COMMAND に載せる
    #: 上限。v0.22 で STM32 の固定上限 3.0 が無くなり、これが実効の上限になった）
    speed_ramp_max_m_s2: float = 3.0
    #: 後輪の静的な荷重の割合と、前後の荷重移動 h/L（重心高さ ÷ ホイールベース）。`[control.plant]` の
    #: `rear_lateral_share`・`load_transfer`（校正 7）。slam2d の車軸ごとの摩擦（`grip_shape` = 1）が使う
    rear_load_frac: float = 0.59
    load_transfer_hl: float = 0.133
    #: planner が計画に使う**加速**の天井 [m/s²]（`[safety] plan_accel_limit_m_s2`。無ければ
    #: `speed_ramp_max_m_s2`）。減速の天井はランプの上限のまま——ランプの上限は加減速で対称なので、
    #: 減速だけを上げるときに加速の計画まで上がらないように分けてある（2026-10-20。加速は TC が
    #: 絞っていて 3.0 も出ていない。減速の天井を上げるのは後輪のロックを確かめてから、vehicle.toml）
    plan_accel_limit_m_s2: float = 0.0
    #: LiDAR の周の終わり → STM32 が COMMAND を受理するまで [s]
    control_latency_s: float = 0.0
    #: 曲率の2次遅れ（ヨー応答）の固有角振動数 [rad/s] と減衰比。0 = 無し
    yaw_natural_freq_rad_s: float = 0.0
    yaw_damping: float = 1.0
    #: COMMAND に載る舵のレート制限 [rad/s]（`[safety]`。実機の舵は常にこの角速度でランプする）
    cmd_steer_rate_limit_rad_s: float = 0.0
    #: STM32 の車速PIの加速度フィードフォワードが有効か（`[control]` の `speed_ff_nm_per_m_s2` > 0、
    #: ★v0.21）。有効なら速度指令は「プロファイルの速度＋その傾き」を渡すだけにする（`nav/speedctl.py`）
    speed_ff_firmware: bool = False
    #: そのフィードフォワードが目標車速の傾きに掛けるローパスの時定数 [s]（`[dynamics] speed_ff_filter_s`、
    #: ファームの定数 DRIVE_SPEED_FF_TAU_S）。Pi はこの時定数ぶん先の速度を送る
    speed_ff_filter_s: float = 0.02
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
        同定値では 46cm で、40cm のつもりで引いた経路は曲がり切れなかった（実車）。
        """
        d = self.steer_servo_gain * self.max_steer
        delta = self.steer_gain * d + self.steer_gain_cubic * d ** 3 - abs(self.steer_offset_rad)
        return math.tan(max(delta, 1e-3)) / self.wheelbase

    # ── 走りの限界（同定値から。未同定の値は控えめな既定で埋める）──

    @property
    def a_lat_max(self) -> float:
        """横加速度の限界 [m/s²]（`[limits] a_lat_m_s2`。校正 7 がタイヤの μ から見積もる）。未校正なら 3.5。"""
        return self.lim_lat_m_s2 if self.lim_lat_m_s2 > 1e-6 else 3.5

    def a_accel_max(self, v: float = 0.0) -> float:
        """直線を加速できる上限 [m/s²]（正味。`[limits] a_accel_m_s2`。校正 9 が TC を入れた全開の発進で測る）。

        目標速度は `speed_ramp_max_m_s2`（Pi が送る `accel_limit` の上限）でならすので、それより上は出ない。
        計画の加速の天井（`plan_accel_max`）でも頭打ちにする。
        `v` は互換のため（車速による低下は測っていない。2m の直線で出せるのは約 2m/s まで）。"""
        a = self.lim_accel_m_s2 if self.lim_accel_m_s2 > 1e-6 else 2.5
        return max(0.05, min(a, self.plan_accel_max))

    @property
    def plan_accel_max(self) -> float:
        """planner が計画に使う加速の天井 [m/s²]（走行中の学習の加速の出発点と天井も）。
        `[safety] plan_accel_limit_m_s2`、ランプの上限より上にはしない。"""
        p = self.plan_accel_limit_m_s2
        return min(p, self.speed_ramp_max_m_s2) if p > 0.0 else self.speed_ramp_max_m_s2

    @property
    def a_decel_max(self) -> float:
        """速度指令を下げたときの減速度の上限 [m/s²]（正味。`[limits] a_brake_m_s2`。校正 9 が車速指令で
        目一杯減速して測る。`brake`＝ABS は使わない）。"""
        a = self.lim_brake_m_s2 if self.lim_brake_m_s2 > 1e-6 else 2.5
        return max(0.05, min(a, self.speed_ramp_max_m_s2))

    @property
    def speed_loop_tau_s(self) -> float:
        """速度PIの比例項だけで見た時定数 [s]（= 1/(車体の利得×kp)）。

        加速度 `a` を出すには目標を車速より `a`×この値だけ先へ置く必要がある——**目標速度を
        そのまま送ると、車速はこの時間だけ遅れる**（実車の「約0.2s 遅れる」）。
        未同定なら 0（前倒ししない）。"""
        g = self.speed_plant_gain * self.speed_kp
        return 1.0 / g if g > 1e-6 else 0.0

    @property
    def speed_loop_reset_s(self) -> float:
        """速度PIの積分が比例項を引き継ぐ時定数 [s]（= kp/ki）。0 = 積分なし。"""
        return self.speed_kp / self.speed_ki if self.speed_ki > 1e-9 and self.speed_kp > 1e-9 else 0.0

    @property
    def steer_delay_s(self) -> float:
        """舵の指令が曲率に効くまでの遅れ [s]: 舵のむだ時間＋1次遅れ＋ヨーの2次遅れ（2ζ/ωn）。

        **指令を送ってからの遅れだけ**（姿勢の古さ・通信は含まない。それは呼び出し側が足す）。"""
        yaw = (2.0 * self.yaw_damping / self.yaw_natural_freq_rad_s
               if self.yaw_natural_freq_rad_s > 1e-6 else 0.0)
        return self.dead_time_s + self.tau_steer_s + yaw

    @property
    def steer_map(self):
        """曲率 ⇄ 指令舵角の変換に使う舵の効き（`nav.purepursuit.SteerMap`）。"""
        from ..nav.purepursuit import SteerMap
        return SteerMap(gain=self.steer_gain, cubic=self.steer_gain_cubic,
                        servo_gain=self.steer_servo_gain, offset=self.steer_offset_rad)

    @classmethod
    def load(cls, path: str | Path | None = None) -> "Vehicle":
        """読めなければ**既定値で返す**（例外にしない）。

        諸元が読めないことは「走ってはいけない」ではなく「暫定値で走る」。
        暫定値であることは `measured=False` が伝えるので、判断は呼び出し側に任せる。
        値の型が違う（数のはずが文字列、表のはずが数）ときも同じ扱いにする——io_node は
        import の時点でこれを読むので、例外にすると起動できずに再起動を繰り返す。
        """
        p = Path(path) if path else DEFAULT_PATH
        try:
            with open(p, "rb") as fp:
                d = tomllib.load(fp)
        except (OSError, tomllib.TOMLDecodeError):
            return cls()
        try:
            return cls._from_dict(d, p)
        except (TypeError, ValueError, KeyError, AttributeError, IndexError) as e:
            print(f"!! {p} の値を読めない（{type(e).__name__}: {e}）。既定値で動く", file=sys.stderr)
            return cls()

    @classmethod
    def _from_dict(cls, d: dict, p: Path) -> "Vehicle":

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
            steer_link=(float(d.get("control", {}).get("steer_link_gain", 1.0)),
                        float(d.get("control", {}).get("steer_link_cubic", 0.0))),
            lim_lat_m_s2=float(d.get("limits", {}).get("a_lat_m_s2", 0.0)),
            lim_accel_m_s2=float(d.get("limits", {}).get("a_accel_m_s2", 0.0)),
            lim_brake_m_s2=float(d.get("limits", {}).get("a_brake_m_s2", 0.0)),
            rear_load_frac=float(d.get("control", {}).get("plant", {}).get("rear_lateral_share", 0.59)),
            load_transfer_hl=float(d.get("control", {}).get("plant", {}).get("load_transfer", 0.133)),
            rolling_resistance=float(dyn.get("rolling_resistance", 0.0)),
            speed_kp=float(dyn.get("speed_kp", 0.0)),
            speed_ki=float(dyn.get("speed_ki", 0.0)),
            speed_plant_gain=float(dyn.get("speed_plant_gain", 0.0)),
            speed_ramp_max_m_s2=_ramp_max(dyn, safety),
            plan_accel_limit_m_s2=float(safety.get("plan_accel_limit_m_s2", 0.0)),
            control_latency_s=float(dyn.get("control_latency_s", 0.0)),
            yaw_natural_freq_rad_s=float(dyn.get("yaw_natural_freq_rad_s", 0.0)),
            yaw_damping=float(dyn.get("yaw_damping", 1.0)),
            cmd_steer_rate_limit_rad_s=float(safety.get("cmd_steer_rate_limit_rad_s", 0.0)),
            speed_ff_firmware=float(d.get("control", {}).get("speed_ff_nm_per_m_s2", 0.0)) > 0.0,
            speed_ff_filter_s=float(dyn.get("speed_ff_filter_s", 0.02)),
            cmd_deadman_ms=_deadman_ms(safety.get("cmd_deadman_ms", 150.0)),
            auto_link_grace_ms=float(safety.get("auto_link_grace_ms", 5000.0)),
            auto_cmd_stale_ms=float(safety.get("auto_cmd_stale_ms", 200.0)),
            _path=p,
        )
