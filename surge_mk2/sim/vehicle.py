"""車両モデル — 自転車モデル + 操舵のむだ時間 + 1次遅れ + 横方向グリップ限界。

**運動学と遅れ、それに簡易グリップ限界だけ。** スリップ角・タイヤの非線形特性・
TC/TVは入れていない。実車の`[dynamics]`（操舵のむだ時間・1次遅れなど）が
未実測の仮値で、合わせ込む相手がまだ無いため。詳細なタイヤモデルを入れるなら実測後。

**グリップ限界（2026-08-28追加）**: 自転車運動学モデルは舵角だけでヨーレートが
決まり、速度に関わらず同じ半径で曲がれてしまう（＝速すぎてコーナーを曲がりきれない
という物理的な必然性が無い）。これは E2E LiDAR の強化学習がコーナー前の減速を
学習する動機を弱めていた（`docs/progress_archive.md`「2026-08-28（続き2）」節）。対策として、
要求される向心加速度 `v^2 * |tan(steer)| / L` が `mu * g` を超えたら、達成できる
曲率を頭打ちにする（＝アンダーステアで外に膨らむ）。**質量は登場しない**——
`F_lat_max = mu*m*g` を運動方程式 `a = F/m` に代入すると質量が消えるため、
車両質量に関わらず摩擦係数`mu`だけで決まる（同じ理由で、急ブレーキの制動距離が
車重に依存しないのと同じ）。

**摩擦円によるRWD連成（2026-09-01追加）**: この車の駆動力・制動力は`drive_ratio`
（後輪ぶん）を通じてどちらも後輪だけが発生させる（`_next_speed()`のブレーキ・
トルクmode参照）。実車では同じ後輪タイヤが「曲がる力」と「加減速する力」を
同じ摩擦の予算 `mu*g` から奪い合うため（摩擦円）、コーナー立ち上がりで加速しながら
曲がると定常円旋回より横方向の余力が減る。これを単純な摩擦円
`a_lat_max = sqrt((mu*g)^2 - a_long^2)` で近似する（前後輪別のグリップ配分・
荷重移動までは踏み込まない）。定常状態（`a_long≈0`）では従来通り`mu*g`に一致する。

**縦加速度の上限（2026-09-01追加）**: `MAX_BRAKE_TORQUE_NM`（後輪モータの
ハードウェア仕様値）から逆算する制動減速度は、タイヤが実際に発生できる摩擦
（`mu*g`）を無条件に超えうる——`mu`の実測値（0.454）では`mu*g≈4.46m/s²`だが
`MAX_BRAKE_TORQUE_NM`由来の制動減速度は≈5.0m/s²で、直進急制動だけでもグリップ
限界を超えてしまう計算になる（実際はスリップして頭打ちになるはず）。`spec`に
`drive_accel_m_s2`/`brake_decel_m_s2`（システム同定タブ「加減速試験」の実測値、
`tools/sysid/fit.py`の`fit_accel()`参照）を追加し、実測済み（>0）ならハードウェア
仕様値より優先して使う——モータのトルク仕様ではなく、実際にタイヤが発生できた
加減速度そのものを使う方が、グリップ限界を含めた実車の挙動に近い。未実測
（既定0.0）の間は従来通りトルク仕様値から逆算する。

**システム同定の作り直しに合わせた拡張（2026-09-24）**: 同定で測る量をすべてここで
表現できるようにした。いずれも既定値は「従来と同じ挙動」になる（未実測のまま
読み込んでも既存のシム・学習の結果は変わらない）。

- **操舵の順序を実機に合わせた**: 指令 →（`COMMAND.steer_rate_limit`による参照ランプ。
  **これが`steer_cmd_echo`**）→ むだ時間 → 1次遅れ → サーボの物理的な角速度上限。
  以前は指令のレート制限を1次遅れの**後**に掛けていたが、実機の`steer_cmd_echo`が
  0.524rad/7.0rad/s≈75msかけてランプすることから、STM32は参照側をランプさせてから
  サーボに渡していると分かった（`tools/sysid/fit.py`の`fit_steer`参照）
- **むだ時間は時刻で引く**（`SteerServo`）。以前の「1step=1エントリ」のキューは
  サブステップ幅`dt`ぶん常に遅れ側へ量子化され（`ceil(d/dt)+1`step）、同定値を
  入れても`dt`によって実効遅延が変わっていた
- **実効舵角**（`SteerLink`）: 報告される舵角（モータ角から換算）と、実際に車が曲がる
  角度の差。リンクのガタ（どちらから切ったかで効きがずれる）→不感帯→
  `steer_gain*d + steer_gain_cubic*d³ + steer_offset_rad`。曲率の計算にだけ使う
- **サーボのヒステリシス**（`SteerServo`）: 実舵角が、どちらから近づいたかで指令の
  少し手前に止まる幅。`steer_actual` に見える（2026-09-25追加）
- **アンダーステア勾配** `understeer_gradient` [rad/(m/s²)]: `δ = L/R + K·a_y`。
  限界まで狙いどおりに曲がって急に頭打ちになる、ではなく速度とともに徐々に膨らむ
- **速度に依存する駆動加速度上限**（`drive_fade_speed_m_s`〜`drive_top_speed_m_s`で
  線形に0へ）と、**速度指令で減速するときの上限**`speed_decel_m_s2`（`brake`とは別物）

**第三者検証を受けた拡張（2026-09-26）**: モデルと違う形の真値で解析を試したところ、黙って
誤った値を返す箇所が見つかったので、実機の構造に寄せた（いずれも既定値は従来と同じ挙動）。

- **速度はファームの PI**（`SpeedController`。`speed_kp>0` のとき）: 目標のランプ→PI→トルク
  上限→車体。以前の1次遅れでは、ファームの形の真値に対して時定数が下限に張り付き、加速度が
  ファームの上限3.0m/s²を超えて読まれ、速度の再現誤差が0.13〜0.27m/sになった
- **曲率の2次遅れ**（`yaw_natural_freq_rad_s`・`yaw_damping`。振動的なヨー応答）: 1次の遅れ
  で表すと、舵の効きが13%ずれ、ガタと不感帯が入れ替わった。ヨーレートそのものではなく曲率に
  掛ける（ヨーレート＝車速×曲率なので、止まった車は回らない）
- **サーボの定常ゲイン**（`steer_servo_gain`）: 位置ループの定常偏差を表せないと、存在しない
  ヒステリシスとむだ時間の偏りに化けた

指令は SI で受ける（整数スケールの解釈は `sim/stm32.py` の仕事）。
"""

from __future__ import annotations

import math
import tomllib
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

__all__ = ["VehicleSpec", "VehicleModel", "DriveInput", "SteerServo", "SteerLink", "next_speed",
           "SpeedController", "SecondOrder",
           "backlash", "deadband",
           "drive_accel_limit", "DEFAULT_SPEC_PATH", "GRAVITY_MPS2"]

DEFAULT_SPEC_PATH = Path(__file__).resolve().parents[1] / "config" / "vehicle.toml"
GRAVITY_MPS2 = 9.81


@dataclass
class VehicleSpec:
    """`config/vehicle.toml` の中身。幾何・質量は実測確定、`[dynamics]` は `tools/sysid/` の実測値。"""

    wheelbase: float = 0.23
    track: float = 0.155
    max_steer: float = 0.524
    wheel_radius: float = 0.03
    mass: float = 2.0
    footprint: list = field(default_factory=lambda: [
        [0.30, 0.09], [0.30, -0.09], [-0.07, -0.09], [-0.07, 0.09]])

    tau_steer_s: float = 0.12
    dead_time_s: float = 0.030
    tau_speed_s: float = 0.35
    rolling_resistance: float = 0.35
    drive_ratio: float = 2.0
    mu: float = 0.8
    #: 実測の最大加速度・減速度 [m/s²]。0.0＝未実測（`MAX_BRAKE_TORQUE_NM`からの
    #: 逆算・`cmd.accel_limit`による外部指定にフォールバック）。
    #: システム同定タブ「加減速試験」（`raspi/auto/sysid_accel.py`）で測る
    drive_accel_m_s2: float = 0.0
    #: 最大制動（`brake`）の減速度上限 [m/s²]。`brake_torque`から逆算した値がこれを
    #: 超えるときはこちらで頭打ちにする（タイヤ・TCが出せる上限）。0.0＝未実測
    #: （`brake_decel_per_nm > 0` のときは「頭打ちなし」）
    brake_decel_m_s2: float = 0.0
    #: 制動トルク（1輪あたり、`COMMAND.brake_torque`）→ 減速度の傾き [(m/s²)/(N·m)]。減速度 =
    #: この値×トルク＋転がり抵抗（`brake_decel_m_s2` で頭打ち）。前後運動試験のブレーキを3段の
    #: 強さで掛けて測る（2026-09-27）。0.0＝未実測（名目の式 トルク×drive_ratio/(車輪半径×質量)）
    brake_decel_per_nm: float = 0.0
    #: MD の制動の境界層と不感帯 [m/s]（**同定しない。MD のファームの定数**。`BLDC/ProgramV4`
    #: `src/app/config.h` の BRAKE_BOUNDARY_SPEED_RAD_S・BRAKE_DEADBAND_SPEED_RAD_S × 後輪半径）。MD は
    #: 制動の電流を「強さ × tanh(車輪の角速度 ÷ 境界層)」にし、不感帯より遅いと0にする（一定トルクの
    #: 符号切り替えはバンバン発振するため）。0 ＝ 境界層なし（一定）
    brake_boundary_speed_m_s: float = 0.0
    brake_deadband_speed_m_s: float = 0.0
    #: ステアサーボの物理的な最大角速度 [rad/s]。`cmd.steer_rate_limit`
    #: （COMMANDで毎回送るレート制限。参照側のランプ）とは別物——こちらは
    #: サーボ出力側の床で、指令が無指定でも常に効く（`SteerServo`参照）。0.0＝制限なし
    #: （指令側のランプの方が遅く、物理上限が観測できなかった場合）
    steer_rate_limit_rad_s: float = 6.0
    #: 実効舵角 = `steer_gain*d + steer_gain_cubic*d³ + steer_offset_rad`。`d` はリンクの
    #: ガタ・不感帯を通した後のタイヤ角（`SteerLink`）。曲率の計算にだけ使う
    steer_gain: float = 1.0
    steer_gain_cubic: float = 0.0
    steer_offset_rad: float = 0.0
    #: サーボのヒステリシス（実舵角が止まる位置の、左右から近づいたときの差）[rad]。0＝無し
    steer_servo_hysteresis_rad: float = 0.0
    #: リンクのガタ（タイヤ角の、左右から切ったときの差）と中立付近の不感帯の幅 [rad]。0＝無し
    steer_link_hysteresis_rad: float = 0.0
    steer_link_deadband_rad: float = 0.0
    #: アンダーステア勾配 K [rad/(m/s²)]。`δ = L/R + K·a_y` ⇔ 曲率 = tan(δ)/(L + K·v²)
    understeer_gradient: float = 0.0
    #: 速度指令モードで**減速する**ときの上限 [m/s²]（`brake`とは別）。0.0＝`drive_accel_m_s2`と同じ
    speed_decel_m_s2: float = 0.0
    #: 駆動加速度の上限が速度とともに落ち始める速度と、0になる速度 [m/s]。
    #: 間は線形。`drive_top_speed_m_s`が0なら減衰なし
    drive_fade_speed_m_s: float = 0.0
    drive_top_speed_m_s: float = 0.0
    #: STM32 の速度PI（`drive.c`、2026-09-26〜）。`speed_kp` が0なら従来の1次遅れ（`tau_speed_s`）。
    #: `speed_kp`・`speed_ki`・`speed_torque_max_nm`・`speed_ramp_max_m_s2` は**ファームの定数**
    #: （`DRIVE_SPEED_KP/KI`・`DRIVE_MAX_TORQUE_NM`×2・`DRIVE_MAX_ACCEL_M_S2`）で同定しない。
    #: 同定するのは車体の側（`speed_plant_gain`・`rolling_resistance`、`SpeedController`参照）
    speed_kp: float = 0.0            # [N·m/(m/s)]
    speed_ki: float = 0.0            # [N·m/(m/s)/s]
    speed_torque_max_nm: float = 0.26  # PI 出力の上限（後輪2輪の合計）[N·m]
    speed_ramp_max_m_s2: float = 3.0   # 目標速度のランプの上限（COMMAND の accel_limit はこれで切られる）
    #: 車体の加速度 / 後輪合計トルク [(m/s²)/(N·m)]。名目は 1/(車輪半径×質量)
    speed_plant_gain: float = 1.0 / (0.03 * 2.0)
    #: STM32 が `speed`/`wheel_speed` に掛けているローパス（1次遅れを `speed_filter_order`
    #: 段つないだもの）の1段あたりの時定数 [s]。0＝無し。**ファームの定数**（2026-09-25〜 1段・
    #: 9.95ms。`docs/uart_protocol.md`「更新レート」）で、同定はしない。
    #: **観測（テレメトリ・学習の入力）だけに効き、車の動きには効かない**
    speed_filter_s: float = 0.0
    speed_filter_order: int = 1
    #: タイヤの緩和長 [m]: 舵を切ってから曲率がついてくるまでの**走行距離**での遅れ
    #: （横力が立ち上がるまでタイヤが転がる距離）。時間にすると低速ほど長い。0＝即座に追従
    yaw_relaxation_m: float = 0.0
    #: STM32 が `yaw_rate` に掛けているローパス（IMUのDLPF等）の時定数 [s]。観測専用。0＝無し
    yaw_rate_filter_s: float = 0.0
    #: 曲率の2次遅れ（車体の慣性とタイヤの横力による振動的なヨー応答）の固有角振動数 [rad/s]と
    #: 減衰比。タイヤの緩和長の後に時間で掛ける。**車の動きに効く**。0＝無し（2026-09-26追加）
    yaw_natural_freq_rad_s: float = 0.0
    yaw_damping: float = 1.0
    #: サーボの位置ループの定常ゲイン（実舵角 / 参照舵角）。負荷で押し戻される位置制御の
    #: 定常偏差。1.0＝偏差なし（2026-09-26追加）
    steer_servo_gain: float = 1.0
    #: LiDARスキャン完了→STM32がCOMMANDを受理するまでの遅れ [s]（Pi側の処理・中継・UART）。
    #: 実ノードを通すインタラクティブシムには不要（パイプライン自体が遅れを作る）。
    #: 実ノードを通さない学習環境（`ml_lidar/env.py`）がこの値で指令を遅らせる
    control_latency_s: float = 0.0
    #: GUIがCOMMANDに毎回載せるレート制限（`[safety]`）。実ノードを通さない呼び出し元が
    #: `DriveInput`を組むときに使う（`VehicleModel`自身は`cmd`の値だけを見る）
    cmd_accel_limit_m_s2: float = 0.0
    cmd_steer_rate_limit_rad_s: float = 0.0

    sensors: dict = field(default_factory=dict)

    @classmethod
    def load(cls, path: str | Path = DEFAULT_SPEC_PATH) -> "VehicleSpec":
        with open(path, "rb") as fp:
            d = tomllib.load(fp)
        dyn = d.get("dynamics", {})
        safety = d.get("safety", {})
        dflt = cls()
        return cls(
            wheelbase=d.get("wheelbase", 0.23),
            track=d.get("track", 0.155),
            max_steer=d.get("max_steer", 0.524),
            wheel_radius=d.get("wheel_radius", 0.03),
            mass=d.get("mass", 2.0),
            footprint=d.get("footprint", dflt.footprint),
            tau_steer_s=dyn.get("tau_steer_s", 0.12),
            dead_time_s=dyn.get("dead_time_s", 0.030),
            tau_speed_s=dyn.get("tau_speed_s", 0.35),
            rolling_resistance=dyn.get("rolling_resistance", 0.35),
            drive_ratio=dyn.get("drive_ratio", 2.0),
            mu=dyn.get("mu", 0.8),
            drive_accel_m_s2=dyn.get("drive_accel_m_s2", 0.0),
            brake_decel_m_s2=dyn.get("brake_decel_m_s2", 0.0),
            brake_decel_per_nm=dyn.get("brake_decel_per_nm", 0.0),
            brake_boundary_speed_m_s=dyn.get("brake_boundary_speed_m_s", 0.0),
            brake_deadband_speed_m_s=dyn.get("brake_deadband_speed_m_s", 0.0),
            steer_rate_limit_rad_s=dyn.get("steer_rate_limit_rad_s", 6.0),
            steer_gain=dyn.get("steer_gain", 1.0),
            steer_gain_cubic=dyn.get("steer_gain_cubic", 0.0),
            steer_offset_rad=dyn.get("steer_offset_rad", 0.0),
            steer_servo_hysteresis_rad=dyn.get("steer_servo_hysteresis_rad", 0.0),
            steer_link_hysteresis_rad=dyn.get("steer_link_hysteresis_rad", 0.0),
            steer_link_deadband_rad=dyn.get("steer_link_deadband_rad", 0.0),
            understeer_gradient=dyn.get("understeer_gradient", 0.0),
            speed_decel_m_s2=dyn.get("speed_decel_m_s2", 0.0),
            drive_fade_speed_m_s=dyn.get("drive_fade_speed_m_s", 0.0),
            drive_top_speed_m_s=dyn.get("drive_top_speed_m_s", 0.0),
            control_latency_s=dyn.get("control_latency_s", 0.0),
            speed_filter_s=dyn.get("speed_filter_s", 0.0),
            speed_filter_order=int(dyn.get("speed_filter_order", 1)),
            yaw_relaxation_m=dyn.get("yaw_relaxation_m", 0.0),
            yaw_rate_filter_s=dyn.get("yaw_rate_filter_s", 0.0),
            yaw_natural_freq_rad_s=dyn.get("yaw_natural_freq_rad_s", 0.0),
            yaw_damping=dyn.get("yaw_damping", 1.0),
            steer_servo_gain=dyn.get("steer_servo_gain", 1.0),
            speed_kp=dyn.get("speed_kp", 0.0),
            speed_ki=dyn.get("speed_ki", 0.0),
            speed_torque_max_nm=dyn.get("speed_torque_max_nm", 0.26),
            speed_ramp_max_m_s2=dyn.get("speed_ramp_max_m_s2", 3.0),
            speed_plant_gain=dyn.get("speed_plant_gain", dflt.speed_plant_gain),
            cmd_accel_limit_m_s2=safety.get("cmd_accel_limit_m_s2", 0.0),
            cmd_steer_rate_limit_rad_s=safety.get("cmd_steer_rate_limit_rad_s", 0.0),
            sensors=d.get("sensors", {}),
        )

    def sensor_pose(self, name: str) -> tuple[float, float, float]:
        """センサの base_link からの (x, y, yaw)。未定義なら原点。"""
        s = self.sensors.get(name, {})
        return float(s.get("x", 0.0)), float(s.get("y", 0.0)), float(s.get("yaw", 0.0))

    def curvature_wheel(self, wheel: float, speed: float) -> float:
        """リンクを通した後のタイヤ角 `wheel` での、グリップ限界で頭打ちにする**前**の曲率 [1/m]。"""
        delta = self.steer_gain * wheel + self.steer_gain_cubic * wheel ** 3 + self.steer_offset_rad
        return math.tan(delta) / (self.wheelbase + self.understeer_gradient * speed * speed)

    def curvature(self, steer_actual: float, speed: float) -> float:
        """静的な曲率（ガタは中央にあるとみなし、不感帯だけ通す）。走行中の状態を持つ
        `VehicleModel` は `SteerLink` を通した `curvature_wheel()` を使う。"""
        return self.curvature_wheel(deadband(steer_actual, self.steer_link_deadband_rad), speed)


def backlash(y: float, u: float, width: float) -> float:
    """幅 `width` のガタ（遊び）。入力 `u` が遊びの端を押している間だけ出力 `y` が動く。"""
    h = 0.5 * width
    if u - y > h:
        return u - h
    if u - y < -h:
        return u + h
    return y


def deadband(u: float, width: float) -> float:
    """中央の幅 `width` では0、外側は端からの距離（連続）。"""
    h = 0.5 * width
    if u > h:
        return u - h
    if u < -h:
        return u + h
    return 0.0


class SteerLink:
    """ステアモータ角（`steer_actual`）→ タイヤ角: リンクのガタ → 中立付近の不感帯。

    `steer_actual` はモータ側のエンコーダから換算した値なので、モータとタイヤの間の
    遊びはそこに現れず、曲率にだけ出る。`VehicleModel` と `tools/sysid/fit.py` が同じ
    実装を使う。
    """

    def __init__(self, hysteresis_rad: float, deadband_rad: float, initial: float = 0.0) -> None:
        self.hysteresis_rad = hysteresis_rad
        self.deadband_rad = deadband_rad
        self._y = initial

    def step(self, steer_actual: float) -> float:
        self._y = backlash(self._y, steer_actual, self.hysteresis_rad)
        return deadband(self._y, self.deadband_rad)


@dataclass
class DriveInput:
    """`COMMAND` を SI に開いたもの。"""

    armed: bool = False
    brake: bool = False
    torque_mode: bool = False
    target_speed: float = 0.0        # m/s
    target_steer: float = 0.0        # rad（路面舵角）
    accel_limit: float = 0.0         # m/s²。0 = 制限なし
    steer_rate_limit: float = 0.0    # rad/s。0 = 制限なし
    brake_torque: float = 0.0        # N·m（後輪各輪）。0 = 未指定 = 最大制動
    target_torque: float = 0.0       # N·m（駆動トルク直接指令、v0.6）
    launch: bool = False             # ローンチコントロールの要求（v0.20。`SpeedController` が見る）


class SteerServo:
    """ステアサーボ: 参照舵角 → むだ時間 → ×定常ゲイン → 1次遅れ → 物理的な角速度上限 → ヒステリシス → 実舵角。

    `VehicleModel`と`tools/sysid/fit.py`（同定の出力誤差フィット）が**同じ実装**を使う。
    同定値が「このシムで実機の応答を再現する値」になることを保証するため。

    むだ時間は「入力が始まった時刻」を記録しておき、ステップ中点 `t + dt/2 - dead`
    時点で有効だった入力を引く（ZOH）。サブステップ幅によらず実効遅延の期待値が
    `dead_time_s`に一致する（以前の1step=1エントリ方式は常に遅れ側へ量子化していた）。
    """

    def __init__(self, dead_time_s: float, tau_s: float, rate_cap: float,
                 initial: float = 0.0, hysteresis_rad: float = 0.0, gain: float = 1.0) -> None:
        self.dead_time_s = dead_time_s
        #: 位置ループの定常ゲイン（`VehicleSpec.steer_servo_gain`）
        self.gain = gain
        self.tau_s = tau_s
        self.rate_cap = rate_cap
        self.hysteresis_rad = hysteresis_rad
        self.reset(initial)

    def reset(self, initial: float = 0.0) -> None:
        self.t = 0.0
        self.actual = initial
        #: ヒステリシスを通す前の（位置制御が目指している）角度
        self._z = initial
        #: (入力が始まった時刻, 値)
        self._hist: deque[tuple[float, float]] = deque([(-math.inf, initial)])

    def step(self, ref: float, dt: float) -> float:
        hist = self._hist
        if ref != hist[-1][1]:
            hist.append((self.t, ref))
        t_q = self.t + 0.5 * dt - self.dead_time_s
        while len(hist) >= 2 and hist[1][0] <= t_q:
            hist.popleft()
        u = hist[0][1]
        self.t += dt
        self._z = self.lag_step(self._z, u * self.gain, dt, self.tau_s, self.rate_cap)
        # 止まる位置は、近づいてきた側に幅の半分だけ手前（静止摩擦で位置制御が押し切れない）
        self.actual = backlash(self.actual, self._z, self.hysteresis_rad) \
            if self.hysteresis_rad > 0 else self._z
        return self.actual

    @staticmethod
    def lag_step(actual: float, u: float, dt: float, tau_s: float, rate_cap: float) -> float:
        """むだ時間の後の1ステップ: 1次遅れ（ZOH厳密離散化）→ 角速度上限。"""
        want = u
        if tau_s > 1e-6:
            want = actual + (u - actual) * (1.0 - math.exp(-dt / tau_s))
        if rate_cap > 1e-6:
            return actual + max(-rate_cap * dt, min(rate_cap * dt, want - actual))
        return want


def drive_accel_limit(spec: VehicleSpec, v: float) -> float:
    """速度`v`で出せる駆動加速度の上限 [m/s²]（未実測なら`inf`）。"""
    a = spec.drive_accel_m_s2
    if a <= 1e-6:
        return math.inf
    vk, vt = spec.drive_fade_speed_m_s, spec.drive_top_speed_m_s
    s = abs(v)
    if vt > 1e-6 and vt > vk and s > vk:
        a *= max(0.0, (vt - s) / (vt - vk))
    return a


def brake_speed_factor(spec: VehicleSpec, v: float) -> float:
    """MD の制動の効き（0〜1）: tanh(|v| ÷ 境界層)、不感帯より遅ければ0（`brake_boundary_speed_m_s`）。
    実機の記録（2026-09-27、4段の強さ）で、境界層なしだと制動中の位置が5〜11mmずれ、あると
    0.9〜2.1mm（強さ→減速度の傾きも強さによらず35〜39にそろった）。"""
    s = abs(v)
    if spec.brake_boundary_speed_m_s <= 1e-6:
        return 1.0
    if s < spec.brake_deadband_speed_m_s:
        return 0.0
    return math.tanh(s / spec.brake_boundary_speed_m_s)


def brake_decel(spec: VehicleSpec, cmd: DriveInput, v: float) -> tuple[float, bool]:
    """制動モード（`cmd.brake`）の減速度 [m/s²] と、それがタイヤのグリップ（`brake_decel_m_s2`）で
    頭打ちになったか。

    **頭打ちになった＝実機では ABS が制動を削る場面**（★v0.15、`FLG_ABS_ACTIVE`）。ABS は後輪を
    ロック手前（滑り率 -0.2）に保つので、減速度はグリップの上限になる——ここの頭打ちと同じ形。
    ABS を切った実機は後輪がロックして減速度・横グリップとも落ちるが、このモデルは後輪の回転を
    持たないので表せない（`sim/stm32.py` は ABS の有効/無効で旗だけを変える）。"""
    sp = spec
    nm = cmd.brake_torque if cmd.brake_torque > 0 else VehicleModel.MAX_BRAKE_TORQUE_NM
    if sp.brake_decel_per_nm > 1e-6:
        # 実測の「トルク→減速度」（`sysid_accel` のブレーキ4段、2026-09-27）× MD の境界層。
        # タイヤのグリップで頭打ち（`brake_decel_m_s2`、0＝頭打ちなし）
        want = sp.brake_decel_per_nm * nm * brake_speed_factor(sp, v) + sp.rolling_resistance
    else:
        # 未実測の間は名目の質量から出す（速度制御の `speed_plant_gain` とは別）。
        # 未指定（=最大制動）なら実測の上限そのもの（モジュールdocstring「縦加速度の上限」参照）
        want = nm * sp.drive_ratio / (sp.wheel_radius * sp.mass)
        if cmd.brake_torque <= 0 and sp.brake_decel_m_s2 > 1e-6:
            want = max(want, sp.brake_decel_m_s2)
    if sp.brake_decel_m_s2 > 1e-6 and want > sp.brake_decel_m_s2:
        return sp.brake_decel_m_s2, True
    return want, False


def next_speed(spec: VehicleSpec, cmd: DriveInput, v: float, dt: float) -> float:
    """1ステップ後の車速。`VehicleModel`と`tools/sysid/fit.py`が共有する。"""
    sp = spec
    if not cmd.armed:
        # ARM されていなければ惰行して止まる（駆動は切れているが慣性はある）。
        # ★×4 に根拠は無い（ファームの DISARM は `Coast`＝モータを放すだけで、減速は転がり抵抗）。
        # 学習環境は DISARM を使わないので、挙動を変えずに残している（2026-09-27）
        return _toward_zero(v, sp.rolling_resistance * 4.0 * dt)

    if cmd.brake:
        return _toward_zero(v, brake_decel(sp, cmd, v)[0] * dt)

    if cmd.torque_mode:
        # トルク直接指令（v0.6）。**無視すると「スポーティなラジコンモード」で
        # 車が一切動かない**ので、加速度への換算だけは入れてある
        a = cmd.target_torque * sp.drive_ratio / (sp.wheel_radius * sp.mass)
        a -= math.copysign(sp.rolling_resistance, v) if abs(v) > 1e-3 else 0.0
        return v + a * dt

    # 速度指令: 1次遅れで追う。加速側は`drive_accel_limit()`（実測・速度依存）、
    # 減速側は`speed_decel_m_s2`（未実測なら`drive_accel_m_s2`＝従来の対称な上限）。
    # `cmd.accel_limit`（COMMANDのレート制限）も効く場合は厳しい方を採る
    target = cmd.target_speed
    if sp.tau_speed_s > 1e-6:
        want = v + (target - v) * (1.0 - math.exp(-dt / sp.tau_speed_s))
    else:
        want = target
    d = want - v
    speeding_up = (v >= 0.0 and d > 0.0) or (v <= 0.0 and d < 0.0)
    if speeding_up:
        lim = drive_accel_limit(sp, v)
    elif sp.speed_decel_m_s2 > 1e-6:
        lim = sp.speed_decel_m_s2
    else:
        lim = sp.drive_accel_m_s2 if sp.drive_accel_m_s2 > 1e-6 else math.inf
    if cmd.accel_limit > 0:
        lim = min(lim, cmd.accel_limit)
    if lim < math.inf:
        step = lim * dt
        want = v + max(-step, min(step, d))
    return want


class SpeedController:
    """STM32 の速度制御（`MainF446RE_V3/src/control/drive.c`）を写したもの（2026-09-26）。

    目標速度を `min(cmd.accel_limit, speed_ramp_max_m_s2)` でランプ → PI（帰還は前輪速度の
    ローパス後 `speed_filter_s`。出力 ±`speed_torque_max_nm` で条件付き積分）→ 加速度の
    上限（`drive_accel_limit()`・`speed_decel_m_s2`。外側の制限なので積分を back-calculation で
    戻す、`UnwindIntegral` と同じ時定数0.1s）→ 車体（加速度 = `speed_plant_gain`×トルク −
    転がり抵抗）。目標と測定速度がどちらも 0.05m/s 未満なら惰行して積分を捨てる
    （`IsStandstill`）。ブレーキ・トルク指令・DISARM の間は目標を0・積分を0に戻す。

    以前の「1次遅れ＋加速度上限」は、ファームのこの構造（ランプ→2次応答・行き過ぎ）を
    表現できず、同定で時定数が下限に張り付いた（検証: 2026-09-26、PROGRESS.md）。
    `VehicleModel`（`speed_kp>0` のとき）と `tools/sysid/fit.py` が同じ実装を使う。

    **ローンチコントロール**（`cmd.launch`、★v0.20。`drive.c` の `UpdateLaunch`）: 目標が測定速度より
    `LAUNCH_MIN_STEP_M_S` 以上速い加速では、ランプとPIを迂回して出せる最大のトルクを要求し
    （加速度の上限 `drive_accel_limit()` が TC の代わり）、目標の `LAUNCH_END_MARGIN_M_S` 手前で
    PI へ戻す。1回の要求で1回（下ろす・制動する・止まる、でまた使える）。発進中かは `launching`。
    """

    STANDSTILL_M_S = 0.05
    ANTIWINDUP_TT_S = 0.10
    LAUNCH_REVERSE_M_S = 0.3
    LAUNCH_MIN_STEP_M_S = 0.5
    LAUNCH_END_MARGIN_M_S = 0.05
    LAUNCH_TIMEOUT_S = 3.0

    def __init__(self, spec: VehicleSpec, launch_exit_torque_nm: float = 0.0) -> None:
        self.spec = spec
        #: 発進を終えて PI へ戻すときの積分の初期値 [N·m、1輪あたり]（`[control]` の同名の値）
        self.launch_exit_torque_nm = launch_exit_torque_nm
        self.reset()

    def reset(self, v: float = 0.0, ref: float = 0.0, integral: float = 0.0) -> None:
        self.ref = ref
        self.integral = integral
        self.v_meas = v
        self.launching = False
        self._launch_spent = False
        self._launch_t = 0.0

    def _update_launch(self, cmd: DriveInput, dt: float) -> bool:
        if not cmd.launch:
            self.launching = self._launch_spent = False
            return False
        sign = 1.0 if cmd.target_speed >= 0.0 else -1.0
        target, v = cmd.target_speed * sign, self.v_meas * sign
        if not self.launching:
            if (self._launch_spent or v <= -self.LAUNCH_REVERSE_M_S
                    or target - v < self.LAUNCH_MIN_STEP_M_S):
                return False
            self.launching = self._launch_spent = True
            self._launch_t = 0.0
        self._launch_t += dt
        if v >= target - self.LAUNCH_END_MARGIN_M_S or self._launch_t >= self.LAUNCH_TIMEOUT_S:
            self.launching = False
        return self.launching

    def step(self, cmd: DriveInput, v: float, dt: float) -> float:
        sp = self.spec
        if sp.speed_filter_s > 1e-6:
            self.v_meas += (v - self.v_meas) * (1.0 - math.exp(-dt / sp.speed_filter_s))
        else:
            self.v_meas = v
        if not cmd.armed or cmd.brake or cmd.torque_mode:
            self.ref = 0.0
            self.integral = 0.0
            self.launching = self._launch_spent = False
            return next_speed(sp, cmd, v, dt)
        rr = sp.rolling_resistance
        t_max = sp.speed_torque_max_nm
        launching = self._update_launch(cmd, dt)
        if launching:
            # 全開を要求する。目標を測定速度に合わせ続け、積分は「その車速を保つトルク」から始める
            sign = 1.0 if cmd.target_speed >= 0.0 else -1.0
            self.ref = self.v_meas
            self.integral = (sign * 2.0 * self.launch_exit_torque_nm / sp.speed_ki
                             if sp.speed_ki > 1e-9 else 0.0)
            t_req = sign * t_max
        else:
            lim = sp.speed_ramp_max_m_s2
            if cmd.accel_limit > 0:
                lim = min(lim, max(0.01, cmd.accel_limit))
            self.ref += max(-lim * dt, min(lim * dt, cmd.target_speed - self.ref))
            if abs(self.ref) < self.STANDSTILL_M_S and abs(self.v_meas) < self.STANDSTILL_M_S:
                self.integral = 0.0
                self._launch_spent = False
                return _toward_zero(v, rr * dt)
            e = self.ref - self.v_meas
            self.integral += e * dt
            t_req = sp.speed_kp * e + sp.speed_ki * self.integral
            if t_req > t_max or t_req < -t_max:
                # PIDライブラリの出力制限と条件付き積分（`pid.h`）
                if (t_req > 0) == (e > 0):
                    self.integral -= e * dt
                t_req = max(-t_max, min(t_max, t_req))
        g = sp.speed_plant_gain
        a_req = g * t_req
        # 外側の加速度上限（モータ・TCで出せる分）。進む向きに押すなら駆動、逆なら減速
        driving = (a_req >= 0.0) == (v >= 0.0) or abs(v) < 1e-3
        a_lim = drive_accel_limit(sp, v) if driving else (
            sp.speed_decel_m_s2 if sp.speed_decel_m_s2 > 1e-6 else math.inf)
        a = max(-a_lim, min(a_lim, a_req))
        if a != a_req and not launching and sp.speed_ki > 1e-9 and g > 1e-9:
            self.integral += (a - a_req) / g / sp.speed_ki * (dt / self.ANTIWINDUP_TT_S)
        v_new = v + a * dt
        # 転がり抵抗は速度を0の向きへ（0をまたいで逆走させない）
        if abs(v_new) > 1e-6:
            v_new = _toward_zero(v_new, rr * dt)
        return v_new


class SecondOrder:
    """2次遅れ `ÿ + 2ζω ẏ + ω² y = ω² u` の、入力を区間で一定とみなした厳密な離散化。"""

    def __init__(self, wn: float, zeta: float, y0: float = 0.0) -> None:
        self.wn, self.zeta = wn, zeta
        self.y, self.yd = y0, 0.0
        self._dt = None

    def _coeffs(self, dt: float):
        wn, z = self.wn, self.zeta
        # e^{At}、A = [[0, 1], [-wn², -2ζwn]]。s = tr/2、q² = s² - det
        s = -z * wn
        disc = s * s - wn * wn
        es = math.exp(s * dt)
        if disc > 1e-12:
            q = math.sqrt(disc)
            c, sq = math.cosh(q * dt), math.sinh(q * dt) / q
        elif disc < -1e-12:
            q = math.sqrt(-disc)
            c, sq = math.cos(q * dt), math.sin(q * dt) / q
        else:
            c, sq = 1.0, dt
        # Φ = e^{st}[c·I + sq·(A - sI)]
        p11 = es * (c + sq * (0.0 - s))
        p12 = es * sq
        p21 = es * sq * (-wn * wn)
        p22 = es * (c + sq * (-2 * z * wn - s))
        # Γ = A⁻¹(Φ - I)B、B = [0, wn²]ᵀ。A⁻¹ = [[-2ζwn, -1], [wn², 0]] / wn²
        g1 = 1.0 - p11
        g2 = -p21
        return p11, p12, p21, p22, g1, g2

    def step(self, u: float, dt: float) -> float:
        if self._dt != dt:
            self._c = self._coeffs(dt)
            self._dt = dt
        p11, p12, p21, p22, g1, g2 = self._c
        y, yd = self.y, self.yd
        self.y = p11 * y + p12 * yd + g1 * u
        self.yd = p21 * y + p22 * yd + g2 * u
        return self.y


class VehicleModel:
    """真値の状態を持つ。座標は世界系（x = 東、y = 北、yaw = 反時計回り）。"""

    #: `brake_torque = 0`（未指定）のときに使う最大制動トルク [N·m]。
    #: `convert.py` の MAX_BRAKE_TORQUE_NM と同値
    MAX_BRAKE_TORQUE_NM = 0.13

    def __init__(self, spec: VehicleSpec, start: tuple[float, float, float]) -> None:
        self.spec = spec
        self.reset(start)

    def reset(self, start: tuple[float, float, float]) -> None:
        self.x, self.y, self.yaw = start
        self.speed = 0.0
        #: STM32 が送ってくる `speed`（`speed_filter_s` のローパス後）。観測専用
        self.speed_measured = 0.0
        self._speed_stages = [0.0] * max(1, int(round(self.spec.speed_filter_order)))
        self.steer_actual = 0.0
        #: STM32がCOMMANDのレート制限で作る参照舵角。**`steer_cmd_echo`はこれ**
        self.steer_ref = 0.0
        self.yaw_rate = 0.0
        #: STM32 が送ってくる `yaw_rate`（`yaw_rate_filter_s` のローパス後）。観測専用
        self.yaw_rate_measured = 0.0
        #: タイヤの緩和長ぶん遅れて追従している曲率（グリップで頭打ちにする前）
        self._curv_state = 0.0
        self.accel_x = 0.0
        self._a_lat_max = self.spec.mu * GRAVITY_MPS2   #: 摩擦円で絞った直近の横加速度上限
        self.odom_front = [0.0, 0.0]     # [FL, FR] 累積 [m]（射影なし = 前輪の実距離）
        self.collided = False
        self.collisions = 0
        self.cmd = DriveInput()
        sp = self.spec
        self._servo = SteerServo(sp.dead_time_s, sp.tau_steer_s, sp.steer_rate_limit_rad_s,
                                 hysteresis_rad=sp.steer_servo_hysteresis_rad,
                                 gain=sp.steer_servo_gain)
        self._link = SteerLink(sp.steer_link_hysteresis_rad, sp.steer_link_deadband_rad)
        #: リンクを通した後のタイヤ角（曲率はこれで決まる）
        self.steer_wheel = 0.0
        #: STM32 の速度PI（`speed_kp>0` のとき。0なら従来の1次遅れ `next_speed()`）
        self._speed_ctl = SpeedController(sp) if sp.speed_kp > 1e-9 else None
        #: 曲率の2次遅れ（`yaw_natural_freq_rad_s>0` のとき）
        self._yaw2 = SecondOrder(sp.yaw_natural_freq_rad_s, sp.yaw_damping) \
            if sp.yaw_natural_freq_rad_s > 1e-6 else None

    @property
    def launching(self) -> bool:
        """ローンチコントロールで発進している最中か（`TELEMETRY.flags` の LAUNCH_ACTIVE）。"""
        return self._speed_ctl is not None and self._speed_ctl.launching

    def set_launch_exit_torque(self, nm: float) -> None:
        """`[control]` の `launch_exit_torque_nm`（STM32 へ `CONFIG_SET` で入る値）。"""
        if self._speed_ctl is not None:
            self._speed_ctl.launch_exit_torque_nm = nm

    # ── 指令 ──

    def apply(self, cmd: DriveInput) -> None:
        self.cmd = cmd

    # ── 積分 ──

    def step(self, dt: float) -> None:
        sp = self.spec
        cmd = self.cmd

        # ── 操舵: 参照ランプ（= steer_cmd_echo） → むだ時間 → 1次遅れ → 物理上限 ──
        target = max(-sp.max_steer, min(sp.max_steer, cmd.target_steer if cmd.armed else 0.0))
        if cmd.steer_rate_limit > 0:
            r = cmd.steer_rate_limit * dt
            self.steer_ref += max(-r, min(r, target - self.steer_ref))
        else:
            self.steer_ref = target
        self.steer_actual = self._servo.step(self.steer_ref, dt)
        self.steer_wheel = self._link.step(self.steer_actual)

        # ── 速度 ──
        prev_speed = self.speed
        if self._speed_ctl is not None:
            self.speed = self._speed_ctl.step(cmd, self.speed, dt)
        else:
            self.speed = next_speed(sp, cmd, self.speed, dt)
        self.accel_x = (self.speed - prev_speed) / dt if dt > 0 else 0.0
        if sp.speed_filter_s > 1e-6:
            a = 1.0 - math.exp(-dt / sp.speed_filter_s)
            x = self.speed
            for i in range(len(self._speed_stages)):
                self._speed_stages[i] += (x - self._speed_stages[i]) * a
                x = self._speed_stages[i]
            self.speed_measured = x
        else:
            self.speed_measured = self.speed

        # ── 運動（自転車モデル。原点は後輪車軸中心） ──
        # 実効舵角・アンダーステア勾配で決まる曲率を、グリップ限界で頭打ちにする。
        # 向心加速度 = speed^2 * 曲率 なので、曲率の上限は a_lat_max / speed^2
        # （低速ほど緩い上限＝ほぼ無制限）。a_lat_max 自体は摩擦円で縦加速度ぶん絞る
        # （駆動・制動は後輪だけが担うため、加減速中は同じ後輪の横方向の余力が減る。
        # モジュールdocstring参照）
        requested_curvature = sp.curvature_wheel(self.steer_wheel, self.speed)
        if sp.yaw_relaxation_m > 1e-6:
            # 走行距離で1次遅れ（タイヤの横力が立ち上がるまでの転がり距離）
            a = 1.0 - math.exp(-abs(self.speed) * dt / sp.yaw_relaxation_m)
            self._curv_state += (requested_curvature - self._curv_state) * a
            requested_curvature = self._curv_state
        if self._yaw2 is not None:
            # 曲率の2次遅れ（振動的なヨー応答）。ヨーレート＝車速×曲率なので、止まった車は回らない
            # （ヨーレートそのものに掛けると、停止後もヨーが振動し続けた）
            requested_curvature = self._yaw2.step(requested_curvature, dt)
        a_lat_max = math.sqrt(max(0.0, (sp.mu * GRAVITY_MPS2) ** 2 - self.accel_x ** 2))
        self._a_lat_max = a_lat_max
        max_curvature = a_lat_max / max(abs(self.speed), 1e-6) ** 2
        curvature = math.copysign(min(abs(requested_curvature), max_curvature), requested_curvature)
        self.yaw_rate = self.speed * curvature
        if sp.yaw_rate_filter_s > 1e-6:
            self.yaw_rate_measured += (self.yaw_rate - self.yaw_rate_measured) * \
                (1.0 - math.exp(-dt / sp.yaw_rate_filter_s))
        else:
            self.yaw_rate_measured = self.yaw_rate
        self.yaw = (self.yaw + self.yaw_rate * dt + math.pi) % (2 * math.pi) - math.pi
        self.x += self.speed * math.cos(self.yaw) * dt
        self.y += self.speed * math.sin(self.yaw) * dt

        # 前輪の走行距離は 1/cos(δ) 倍（`uart_protocol.md` §5.3 の射影の逆）
        d_front = abs(self.speed) * dt / max(0.1, math.cos(self.steer_actual))
        sgn = 1.0 if self.speed >= 0 else -1.0
        self.odom_front[0] += d_front * sgn
        self.odom_front[1] += d_front * sgn

    # ── 衝突 ──

    def note_collision(self, hit: bool) -> None:
        """コース側の判定結果を受けて速度を殺す。**テレポートさせない。**

        壁に押し付けられて止まる挙動にしておくと、「どこでぶつかったか」が
        画面に残る。跳ね返したり戻したりすると原因が見えなくなる。
        """
        if hit and not self.collided:
            self.collisions += 1
        self.collided = hit
        if hit:
            self.speed = 0.0

    # ── 派生量 ──

    @property
    def wheel_speed(self) -> list[float]:
        """[FL, FR, RL, RR]。**射影なし**（前輪は 1/cos(δ) 倍で回る）。"""
        f = self.speed / max(0.1, math.cos(self.steer_actual))
        return [f, f, self.speed, self.speed]

    @property
    def wheel_speed_measured(self) -> list[float]:
        """`wheel_speed` に `speed` と同じローパスを掛けたもの（STM32 が送る値）。"""
        f = self.speed_measured / max(0.1, math.cos(self.steer_actual))
        return [f, f, self.speed_measured, self.speed_measured]

    @property
    def accel_lateral(self) -> float:
        return self.speed * self.yaw_rate

    @property
    def slip_frac(self) -> float:
        """要求向心加速度が（摩擦円で絞った）横加速度上限を超えた分の比率。0=余裕あり、
        1=ちょうど上限ぶん超過。グリップ限界クランプ（`step()`参照）が実際に効いて
        いるかどうかを学習側に伝えるための量。`_a_lat_max`は直近の`step()`で
        縦加速度ぶん絞った値なので、加減速中の摩擦円連成もここに反映される。"""
        requested_a_lat = self.speed ** 2 * abs(self.spec.curvature_wheel(self.steer_wheel, self.speed))
        a_lat_max = self._a_lat_max
        return max(0.0, requested_a_lat - a_lat_max) / a_lat_max if a_lat_max > 1e-9 else 0.0


def _toward_zero(v: float, delta: float) -> float:
    if abs(v) <= delta:
        return 0.0
    return v - math.copysign(delta, v)
