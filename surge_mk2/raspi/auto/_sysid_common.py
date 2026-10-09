"""システム同定プランナー（`sysid_*`）が共有する小道具。

## 手順を固定した理由（2026-09-25、バンビの指摘）

以前は振幅・速度・保持時間・回数などをすべて GUI のスライダ（`ParamSpec`）で変えられた。
しかし各値は「測りたい量が見える」「解析の前提（定常区間の長さ・ステップの大きさ・
速度域）を満たす」「2m の直線／約1.7m 四方に収まる」を同時に満たすように**組み合わせで**
決めてあり、1つだけ動かすと黙って測れなくなる（例: 保持時間を縮めると定常区間が消える、
v_max を下げると旋回の限界に届かない、振幅を下げると物理上限が見えない）。どの組み合わせ
なら測れるかを GUI で保証する方法が無いので、手順は各プランナーの `SETTINGS` に固定し、
GUI からは変えられないようにした。変えるときはコードを直し、`tools/sysid/bench.py` で
測れることを確かめる。選べるのは旋回試験の方向（左右差を見る）だけ。**catalogには登録しない**（`_`始まりの
ファイル名で`raspi/auto/registry.py`の対象外——`Planner`のサブクラスではなく
ヘルパのため）。
"""

from __future__ import annotations

import bisect
import math
import statistics

from ..msgs.types import AutoState, VehicleState
from ..proto.generated.packets import MDS_COMM_OK, MDS_VOLTAGE_OOR

__all__ = ["TestGate", "lateral_saturated", "SATURATION_GROWTH_FRAC", "SETTLE_S",
           "first_saturated", "corner_windows", "SATURATION_DV_M_S", "CORNER_WINDOW_S",
           "REAR_SLIP_LIMIT", "rear_slip", "corner_limit", "first_oversteer",
           "OVERSTEER_CURVATURE_RATIO", "md_down", "MD_NAMES", "ABORT_BRAKE_TORQUE_NM",
           "stop_distance", "brake_decel_guess", "OdomRun", "settle_state", "DeadReckon",
           "SensorGuard", "abort_state"]


#: `md_status` の並び（`uart_protocol.md` §5.5）
MD_NAMES = ("左後輪", "右後輪", "ステア")


def md_down(md_status: list[int]) -> list[str]:
    """無応答のモータドライバの名前（`MD_NAMES`）。STM32 が直近100ms の受信で立てる `MDS_COMM_OK` を見る。
    `md_status` が 0（一度も状態が来ていない・古い記録・テストの既定値）は故障と見なさない
    （`slam2d_raceline._md_fault` と同じ扱い）。プランナー（中止）と解析（記録を使わない）が共用する。"""
    return [MD_NAMES[i] for i, s in enumerate(md_status[:3]) if s and not (s & MDS_COMM_OK)]


class SensorGuard:
    """前輪の速度（`speed`・`odom_dist` の元）を、独立な後輪モータの周速と照合する（2026-09-26）。

    ## なぜ要るか（第三者検証で見つかった暴走）

    試験の「止まる」「直線が尽きた」「戻りきった」の判定は、どれも前輪エンコーダ由来の
    `speed`・`odom_dist` だけを見ていた。エンコーダが途中で止まる（断線・ADC不調）と、
    速度0・距離が進まないと読み続け、ベンチでは**加減速試験が2mの直線で35m前進**、
    ステア試験が11.6m前進・4.2m後退しても終わらなかった（実機ではさらに、ファームの速度PIが
    速度0を見てトルクを振り切る）。後輪の周速（`wheel_speed[2:4]`、モータ側の値）は前輪と
    独立なので、**後輪が回っているのに前輪がそれより大きく遅い**状態が続いたら異常とする。

    逆向き（前輪が後輪より速い）は見ない: 制動で後輪がロックすると正常でも起きるうえ、
    前輪が多めに読む方向の不調は、止まる判定が早まるだけで安全側。全開加速の空転（TCが
    許す滑り）で誤って止めないよう、許容差は 0.2m/s ＋前輪の速さの30%。
    `planning_node` が全サンプルで `on_vehicle_state()` を呼び、それをここへ渡す。

    ## モータドライバの無応答も見る（2026-10-08）

    MD が黙ると、STM32 はその車輪の速度・電流を**最後の値のまま**送り続ける。実機の前後運動試験で、
    駆動バッテリーが垂れて右後輪の MD が電圧異常で止まり（MD は異常の表示中 約0.8s 黙る）、右後輪の
    周速が 1.60→1.04m/s に固まった。車が止まると後輪の平均 0.54m/s・前輪 0.05m/s になり、上の照合が
    「前輪エンコーダの不調？」と**逆の原因**を表示して中止した。止まった MD は駆動も制動もしないので、
    その間の記録は同定に使えない。`MDS_COMM_OK` が落ちた時点で、原因を正しく言って中止する。
    """

    TOLERANCE_M_S = 0.2
    TOLERANCE_FRAC = 0.3
    PERSIST_S = 0.3

    def __init__(self) -> None:
        self.fault: str | None = None
        self._bad_since_ns: int | None = None

    def reset(self) -> None:
        self.fault = None
        self._bad_since_ns = None

    def update(self, vs: VehicleState) -> None:
        if self.fault is not None:
            return
        down = md_down(vs.md_status)
        if down:
            volt = any(s & MDS_VOLTAGE_OOR for s in vs.md_status[:3])
            self.fault = (f"{'・'.join(down)}のモータドライバが無応答（車輪の値が古いまま固まる"
                          f"{'。電圧異常' if volt else ''}）。駆動バッテリーの残量と MD の電源を確かめる")
            return
        if len(vs.wheel_speed) < 4:
            return
        front = vs.speed
        rear = 0.5 * (vs.wheel_speed[2] + vs.wheel_speed[3])
        opposite = front * rear < 0 and min(abs(front), abs(rear)) > self.TOLERANCE_M_S
        slow = abs(rear) - abs(front) > self.TOLERANCE_M_S + self.TOLERANCE_FRAC * abs(front)
        if not (slow or opposite):
            self._bad_since_ns = None
            return
        if self._bad_since_ns is None:
            self._bad_since_ns = vs.t_capture
        elif vs.t_capture - self._bad_since_ns >= self.PERSIST_S * 1e9:
            self.fault = (f"前輪の速度 {front:+.2f}m/s が後輪 {rear:+.2f}m/s と食い違う"
                          f"（{self._cause(vs)}）")

    SPIN_FRONT_M_S = 0.25

    def _cause(self, vs: VehicleState) -> str:
        """後輪が速い原因の見立て。前輪が止まっている（エンコーダが止まった形）のか、前輪も
        動いていて後輪だけ速い（加速がタイヤのグリップを超えて空転した形。ファームの TC が
        介入していれば `tc_active`）のか。どちらでも止めるのは同じで、表示だけ分ける
        （2026-09-27。空転を入れた真値で「前輪エンコーダの不調？」と誤って表示した）。"""
        slip = "・".join(f"{x:+.2f}" for x in vs.tc_slip)
        if vs.tc_active or abs(vs.speed) >= self.SPIN_FRONT_M_S:
            return (f"後輪の空転（加速がグリップを超えた）か前輪エンコーダの不調。"
                    f"TC{'介入中' if vs.tc_active else '介入なし'}・後輪スリップ率 {slip}")
        return "前輪エンコーダの不調？前輪がほぼ止まっている"


#: 同定の試験を止めるときの制動トルク [N·m]（1輪あたり）。実機（2026-09-27）で 0.12N·m 以上は後輪が
#: ロックし（制動中は TC が効かない）、旋回中の最大制動＋舵0で後輪がロックした。0.08N·m はロック
#: せず、1m/s 以上では約3m/s²で止まる
ABORT_BRAKE_TORQUE_NM = 0.08


def abort_state(st: AutoState, why: str, steer: float = 0.0) -> AutoState:
    """試験を中止して制動する指令。舵は `steer`（今の舵）を保ち、後輪がロックしない強さ
    （`ABORT_BRAKE_TORQUE_NM`）で制動する。以前は `ready=False`（planning_node が最大制動に読み替える）
    で舵も0に戻していたので、旋回中の中止で後輪がロックした（2026-09-27 の実機の旋回試験）。"""
    st.ready = True
    st.brake = True
    st.brake_torque = ABORT_BRAKE_TORQUE_NM
    st.target_speed = 0.0
    st.target_steer = steer
    st.reason = f"中止: {why}"
    return st


class DeadReckon:
    """前輪オドメトリの増分・`yaw_rate`（50Hzの全サンプル）から平面上の位置と向きを積算する。

    直線の試験で「置いた向きの直線」に戻るためだけに使う（数十秒・数m程度なら十分）。
    ジャイロのバイアスは**止まっている間のサンプル**の平均で推定し続け、差し引いてから
    積算する（バイアス0.006rad/sを放っておくと1分で20°ずれる）。止まっているかは
    **プランナーが`stationary`で教える**（静止区間・停止区間）——速度の値で判定すると、
    速度にノイズが乗った実機で止まっているサンプルが見つからず、バイアスが推定されない
    まま積算してしまう（ベンチで速度ノイズ4cm/sにすると横に1m以上ずれた）。
    `planning_node`は持っていれば`on_vehicle_state()`を全サンプルで呼ぶ（ダックタイピング）。
    """

    def __init__(self) -> None:
        self.x = self.y = self.yaw = 0.0
        self._t_ns: int | None = None
        self._odom: float | None = None
        self._bias_sum = 0.0
        self._bias_n = 0
        #: 車が止まっているとプランナーが知っている間 True（静止区間・停止区間）
        self.stationary = False

    @property
    def bias(self) -> float:
        return self._bias_sum / self._bias_n if self._bias_n else 0.0

    def reset_origin(self) -> None:
        """今の位置・向きを原点（置いた直線の始点・向き）にする。"""
        self.x = self.y = self.yaw = 0.0

    def update(self, vs: VehicleState) -> None:
        t = vs.t_capture
        odom = (vs.odom_dist[0] + vs.odom_dist[1]) / 2.0
        if self._t_ns is None or t <= self._t_ns:
            self._t_ns = t
            self._odom = odom
            return
        dt = min((t - self._t_ns) / 1e9, 0.1)
        # 距離はオドメトリの増分から積む（speed を時間で積むより、取りこぼしに強い）
        ds = (odom - self._odom) * math.cos(vs.steer_actual)
        self._t_ns = t
        self._odom = odom
        if self.stationary:
            self._bias_sum += vs.yaw_rate
            self._bias_n += 1
            return
        self.yaw += (vs.yaw_rate - self.bias) * dt
        self.x += ds * math.cos(self.yaw)
        self.y += ds * math.sin(self.yaw)

    def lane_keep_steer(self, forward: bool, limit: float, wheelbase: float) -> float:
        """原点を通る元の向きの直線へ戻る舵角（弱いフィードバック、±`limit`）。

        前進: 曲率 = -k·(yaw + ky·y)。後退: 進む向きが車体と逆なので、向きを直す符号も
        横位置へ寄せる向きも反転する: 曲率 = +k·(yaw - ky·y)。"""
        k, ky = 1.5, 1.0
        kappa = -k * (self.yaw + ky * self.y) if forward else k * (self.yaw - ky * self.y)
        return max(-limit, min(limit, math.atan(kappa * wheelbase)))


def settle_state(st: AutoState) -> AutoState:
    """静止区間の指令（速度0・舵0。制動ではなく速度指令0で止めておく）。"""
    st.ready = True
    st.brake = False
    st.target_speed = 0.0
    st.target_steer = 0.0
    st.reason = "静止（ジャイロのバイアスを計測）"
    return st

#: 試験開始直後に止まっている時間 [s]。解析（`tools/sysid/fit.py`）はここで
#: ジャイロのバイアスを読む。**プランナー側で保証する**——「試験の前に止まっているはず」
#: という運用に頼ると、走りながら試験開始を押した記録でバイアスが読めない
SETTLE_S = 1.0

#: 停止距離の見積もりに使う減速度（STM32の`DRIVE_MAX_ACCEL_M_S2`=3.0の8割）と、
#: 指令が効き始めるまでの遅れ [s]。減速度は測る対象そのものなので`vehicle.toml`に頼らない
_STOP_DECEL_M_S2 = 3.0 * 0.8
_STOP_LATENCY_S = 0.1
#: 速度指令0で止めるときの遅れ [s]。ファームは目標速度を3.0m/s²でランプさせ、その目標は
#: 加速中の実速度より先行している（PIの追従の遅れ）ので、指令を0にしてもすぐには減速しない。
#: ファームと同じ形のシム（`SpeedController`）で、2.0m/sからの停止距離が制動の約3倍・
#: 旧見積もりより0.4m長かった（2026-09-26）。ランプの先行ぶん約0.35sに判断の遅れ0.1sを足す
_STOP_LATENCY_SPEED_CMD_S = 0.45


def stop_distance(v: float, brake: bool = True, accel: float = 0.0,
                  decel: float = _STOP_DECEL_M_S2) -> float:
    """速度`v`から止まりきるまでの前進距離の見積もり [m]（保守側）。

    `brake=False` は速度指令0で止めるとき（PIが減速させる。制動より長い）。`accel` は
    今の加速度 [m/s²]: 判断してから止まり始めるまでの遅れの間も加速し続けるので、その間の
    増速ぶんも入れる（全開加速中に入れないと、ベンチで止まる位置が約0.15m先へずれた）。
    `decel` は止まるときの減速度（既定は最大制動の見積もり。弱いブレーキで止めるときは
    `brake_decel_guess()`）。"""
    v = abs(v)
    lat = _STOP_LATENCY_S if brake else _STOP_LATENCY_SPEED_CMD_S
    a = max(0.0, accel)
    v_peak = v + a * lat
    return v * lat + 0.5 * a * lat * lat + v_peak * v_peak / (2.0 * min(decel, _STOP_DECEL_M_S2))


#: 制動トルク（1輪あたり）→ 減速度の見積もりの傾き [(m/s²)/(N·m)]。名目（トルク×2輪÷(車輪半径
#: ×質量) ≒ 33）の0.75倍の保守側。測る対象そのものなので `vehicle.toml` に頼らない
_BRAKE_DECEL_PER_NM_GUESS = 25.0


def brake_decel_guess(brake_torque: float) -> float:
    """制動トルク [N·m]（0 = 最大）で止まるときの減速度の見積もり [m/s²]（保守側）。"""
    if brake_torque <= 0:
        return _STOP_DECEL_M_S2
    return min(_STOP_DECEL_M_S2, _BRAKE_DECEL_PER_NM_GUESS * brake_torque)


class OdomRun:
    """直線を「走る→止まる→後退して開始位置へ戻る」を繰り返す試験の共通部品。

    前輪オドメトリ（`VehicleState.odom_dist`の平均。舵が小さい前提で射影なし）で
    **試験の開始位置**からの前進距離を見て、`run_length_m`に止まりきれなくなる地点
    （前進距離＋`stop_distance()`）を教える。戻りはオドメトリが開始位置に戻るか、
    前進距離から計算したタイムアウトで終える（オドメトリ不調で無限に後退しない）。

    開始位置は最初の1回だけ決める（2026-09-26）。以前はサイクルごとに置き直していたが、
    ファームは目標速度をランプさせるので、後退から前進へ切り替えた直後に車は少し後ろへ
    行き過ぎる。置き直すとその分が毎回積み重なり、加減速試験（6往復）で全長が約0.6m
    伸びた。戻りも、切り替えの行き過ぎぶん（戻る速さ×`RETURN_LEAD_S`）手前で終える。
    """

    RETURN_TOLERANCE_M = 0.05
    #: 戻りを終える先読み [s]（後退→前進の切り替えで目標のランプが0をまたぐまでの行き過ぎ）
    RETURN_LEAD_S = 0.2
    #: 戻りのタイムアウトの下限 [s]。以前は5.0s・「前進距離÷戻り速度×3＋2s」で、オドメトリが
    #: 止まると開始位置より数m後ろまで後退した（2026-09-26）。今は×1.5＋1s（下限2s）
    RETURN_TIMEOUT_MIN_S = 2.0

    def __init__(self) -> None:
        self.origin: float | None = None
        self.return_timeout_s = self.RETURN_TIMEOUT_MIN_S

    @staticmethod
    def odom(vs: VehicleState) -> float:
        return (vs.odom_dist[0] + vs.odom_dist[1]) / 2.0

    def start(self, vs: VehicleState) -> None:
        """開始位置を決める（最初の1回だけ。以降のサイクルも同じ位置を基準にする）。"""
        if self.origin is None:
            self.origin = self.odom(vs)

    def traveled(self, vs: VehicleState) -> float:
        if self.origin is None:
            self.start(vs)
        return self.odom(vs) - self.origin

    def must_stop(self, vs: VehicleState, run_length_m: float, speed: float) -> bool:
        return self.traveled(vs) + stop_distance(speed) >= run_length_m

    def arm_return(self, vs: VehicleState, return_speed: float) -> None:
        """止まった時点で呼ぶ。戻りのタイムアウトを前進距離から決める。"""
        self.return_speed = return_speed
        self.return_timeout_s = max(self.RETURN_TIMEOUT_MIN_S,
                                    abs(self.traveled(vs)) / return_speed * 1.5 + 1.0)

    def returned(self, vs: VehicleState, t_in_phase: float) -> bool:
        lead = self.RETURN_LEAD_S * getattr(self, "return_speed", 0.0)
        return (self.traveled(vs) <= self.RETURN_TOLERANCE_M + lead
                or t_in_phase >= self.return_timeout_s)

#: 速度段を1つ上げたとき、横加速度の伸びが「グリップに余裕がある場合の伸び」の
#: この割合を下回ったらグリップ限界に達したとみなす（`lateral_saturated()`）
SATURATION_GROWTH_FRAC = 0.3


def lateral_saturated(v_prev: float, a_prev: float, v_cur: float, a_cur: float) -> bool:
    """速度段 `(v_prev, a_prev)` → `(v_cur, a_cur)` で横加速度が頭打ちになったか。

    `sysid_corner`（試験を打ち切る判断）と `tools/sysid/fit.py`（`mu` を読む段の
    選別）が**同じ判定**を使う——片方だけ変えると「プランナーは限界に達したと
    思って止めたのに、解析は達していないと言う」食い違いが起きる。

    グリップに余裕があれば曲率はほぼ一定なので横加速度 `v²κ` は速度の2乗で伸びる
    （アンダーステア勾配があっても伸びは少し鈍るだけ）。限界に達すると `mu*g` で
    止まる。**曲率の比（最低速段との比較）で判定しない**のは、アンダーステア勾配でも
    曲率は速度とともに落ちるため、限界と区別できないから。
    """
    if v_prev <= 1e-3 or a_prev <= 1e-6 or v_cur <= v_prev:
        return False
    expected_growth = a_prev * ((v_cur / v_prev) ** 2 - 1.0)
    return (a_cur - a_prev) < SATURATION_GROWTH_FRAC * expected_growth


#: 旋回試験で、頭打ちを判定するときに比べる2つの区間の速度の差 [m/s]（`first_saturated()`）
SATURATION_DV_M_S = 0.1
#: 旋回試験で、横加速度をまとめる区間の長さと刻み [s]（`corner_windows()`）
CORNER_WINDOW_S = 0.5
CORNER_WINDOW_STEP_S = 0.25


def corner_windows(t: list[float], v: list[float], a: list[float]) -> list[tuple[float, float, int, int]]:
    """旋回試験のサンプル（時刻・速さ・横加速度、時刻順）を `CORNER_WINDOW_S` の区間に分け、
    `(速さの中央値, 横加速度の中央値, 区間の頭の添字, 末尾の添字+1)` を返す（`CORNER_WINDOW_STEP_S`
    ずつずらす）。プランナー（打ち切りの判断）と解析（`mu` を読む区間の選別）が同じ区間を使う。"""
    out: list[tuple[float, float, int, int]] = []
    if not t:
        return out
    start = t[0]
    while start + CORNER_WINDOW_S <= t[-1] + 1e-9:
        i0 = bisect.bisect_left(t, start)
        i1 = bisect.bisect_left(t, start + CORNER_WINDOW_S)
        if i1 - i0 >= 5:
            out.append((statistics.median(v[i0:i1]), statistics.median(a[i0:i1]), i0, i1))
        start += CORNER_WINDOW_STEP_S
    return out


def first_saturated(points: list[tuple[float, float]]) -> int | None:
    """`(速さ, 横加速度)` の並び（速さが上がっていく順）で、横加速度が頭打ちになった最初の添字。

    各点を、それより `SATURATION_DV_M_S` 以上遅い直近の点と `lateral_saturated()` で比べる。
    段を刻む旋回試験（2026-09-26まで）では隣の段どうし、速度を連続して上げる試験では約1.7s
    前の区間と比べることになる（隣の区間どうしでは速度の差が小さく、ノイズで判定が揺れる）。"""
    for k, (v_k, a_k) in enumerate(points):
        j = None
        for jj in range(k - 1, -1, -1):
            if points[jj][0] <= v_k - SATURATION_DV_M_S:
                j = jj
                break
        if j is not None and lateral_saturated(points[j][0], points[j][1], v_k, a_k):
            return k
    return None


#: 加速中に後輪が滑ったとみなす滑り率（後輪2輪の周速の平均と車速の差 ÷ 車速）。前後運動試験
#: （`sysid_accel`）と解析が使う。旋回試験の限界の判定には使わない（`corner_limit`）
REAR_SLIP_LIMIT = 0.10


#: `rear_slip` の分母の下限 [m/s]。低速では「後輪が前輪より 0.1m/s（`REAR_SLIP_LIMIT`×これ）以上速い」
#: を求める。0.1m/s にしていたら、全開加速の走り出し（0.1〜0.3m/s）で STM32 の speed のローパス
#: （約10ms）の遅れと後輪の周速のノイズの数cm/s が滑り率0.1を超え、ベンチで滑ったと取り違えた
_SLIP_SPEED_FLOOR_M_S = 1.0


def rear_slip(vs_speed: float, rear_speed: float) -> float:
    """後輪の滑り率（正＝後輪が速い）。車速が小さいと発散するので分母は `_SLIP_SPEED_FLOOR_M_S` で
    下を抑える。"""
    return (rear_speed - vs_speed) / max(abs(vs_speed), _SLIP_SPEED_FLOOR_M_S)


#: 旋回試験で、曲率（ヨーレート÷速さ）が余裕のある区間の何倍になったら「後輪が流れた」とみなすか。
#: 実機の記録2本（2026-09-27・10-08）で、限界より下の区間の曲率は基準の 0.96〜1.02 倍に収まる
OVERSTEER_CURVATURE_RATIO = 1.15
#: 曲率の基準に使う区間の数の下限と、判定する区間からどれだけ手前までを基準にするか（区間の数。
#: 区間は `CORNER_WINDOW_STEP_S` 刻みなので 4 = 1s 前まで）
_OVERSTEER_MIN_REF = 4
_OVERSTEER_GAP = 4


def first_oversteer(points: list[tuple[float, float]]) -> int | None:
    """`(速さ, 横加速度)` の並びで、後輪が流れた（オーバーステア）最初の添字。

    後輪が横に流れると車は内側へ巻き込み、舵が同じでも曲率（横加速度÷速さ²＝ヨーレート÷速さ）が
    増える。横加速度は伸び続けるので `first_saturated` には掛からない。曲率は舵の非線形・
    アンダーステア勾配では速度とともに**減る**だけなので、増えたら滑りと見てよい。基準は
    1s 以上手前の区間すべての中央値。"""
    kappa = [a / (v * v) if v > 1e-3 else 0.0 for v, a in points]
    for k in range(_OVERSTEER_GAP + _OVERSTEER_MIN_REF, len(points)):
        ref = statistics.median(kappa[:k - _OVERSTEER_GAP])
        if ref > 1e-6 and kappa[k] > OVERSTEER_CURVATURE_RATIO * ref:
            return k
    return None


def corner_limit(points: list[tuple[float, float]]) -> tuple[int | None, str]:
    """`(速さ, 横加速度)` の区間の並びで、グリップの限界に達した最初の添字と理由。プランナー
    （打ち切り）と解析（`mu` を読む区間）が同じ判定を使う。

    **車体が滑ったことを IMU（ジャイロ）で見る**: 横加速度（速さ×ヨーレート）が頭打ちになった
    （前が逃げる、`first_saturated`）か、曲率が跳ねた（後ろが流れる、`first_oversteer`）の早い方。

    後輪の空転（滑り率）は見ない（2026-10-08、バンビ「空転は検知しなくてもいい」）。以前は後輪2輪の
    平均の滑り率が 0.10 を超えたら限界としていたが、実機で横加速度が速さの2乗どおりに伸び、曲率も
    変わらない（＝車体は滑っていない）うちに、荷重の抜けた内輪だけが空転して（内 0.15・外 0.07）
    1.42m/s・4.4m/s² で打ち切った。空転が抑えきれないときは `SensorGuard` が止める。"""
    k_sat = first_saturated(points)
    k_over = first_oversteer(points)
    if k_over is not None and (k_sat is None or k_over < k_sat):
        return k_over, "後輪が流れた"
    if k_sat is not None:
        return k_sat, "横加速度が頭打ち"
    return None, ""


class TestGate:
    """内部の状態機械（ステップ番号・経過時間）を、試験が実際に進行中の間だけ動かす。

    ## なぜ要るか（2026-08-31、実機で3回に分けて不具合報告・修正）

    `planning_node.py`はengageしていなくても`plan()`を呼び続ける（他のplanner
    が「engageする前にどう判断するか」をGUIで覗けるようにするための既存の
    設計）。普通のplannerは毎回その場で答えを作り直すだけで内部に時間経過を
    持たないので困らないが、システム同定のplannerはステップ列を時間で進める
    設計なので、**「試験開始」を押していなくてもステップが進む**不具合が
    最初に見つかった。

    最初の修正は`VehicleState.armed`（人間がARMを保持しているか）をゲートに
    したが、これは2つの問題を生んだ：
    - 「試験中止」は`engaged`をFalseにするだけでARM保持自体は解除しない
      （ARMは人間側の安全弁で、ソフトから奪わない）ため、**ARMを保持したまま
      中止しても、armedがTrueのままなので進行が止まらなかった**
    - それを塞ぐために「ARMが一度Falseに落ちるまで再開しない」凍結を足したが、
      今度は**ARMを保持したまま同じ試験をもう一度「試験開始」しても、ARMの
      入り直しが無いので再開できない**という逆方向の不具合になった

    根本原因は`armed`が`engaged`（試験開始/中止の状態）の代用にならないこと。
    `plan()`自体はengagedを受け取れない（`Planner`の共通契約——全plannerに
    影響するので変えない）ので、代わりに`planning_node.py`が`set_engaged()`
    をダックタイピングで呼ぶ（`_apply_e2e_model`の`reload_if_changed`と
    同じパターン）。**進行のゲートは`engaged`、ARMは表示（「ARM待ち」）専用**
    にしたことで、上の2つの不具合を両方解消できる。
    """

    def __init__(self) -> None:
        self._engaged = False
        self._just_started = False
        self._settle_t = 0.0

    def reset(self) -> None:
        self._engaged = False
        self._just_started = False
        self._settle_t = 0.0

    def settling(self, dt: float) -> bool:
        """試験開始から`SETTLE_S`の間 True（呼び出し側は止まっている指令を出す）。

        `tick()`がengaged・armedを返した後に毎周期呼ぶ。試験開始が押し直されると
        また最初から止まる。"""
        if self._settle_t >= SETTLE_S:
            return False
        self._settle_t += dt
        return True

    def set_engaged(self, engaged: bool) -> None:
        """`planning_node.py`が`plan()`の直前に毎周期呼ぶ（ダックタイピング）。

        False→Trueの遷移を「試験開始が新しく押された」と解釈し、次の
        `tick()`が`just_started=True`を返すようにする。
        """
        if engaged and not self._engaged:
            self._just_started = True
            self._settle_t = 0.0
        self._engaged = engaged

    def tick(self, vs: VehicleState | None) -> tuple[bool, bool, bool]:
        """`(engaged, armed, just_started)`。

        `engaged`が内部状態機械を進めてよいかのゲート。`armed`は表示専用
        （engaged中でもARMが無ければ「ARM待ち」を出す）。`just_started`は
        今回だけTrueで、呼び出し側はこれを見て内部カウンタを0に戻す。
        """
        armed = bool(vs and vs.armed)
        just_started = self._just_started
        self._just_started = False
        return self._engaged, armed, just_started
