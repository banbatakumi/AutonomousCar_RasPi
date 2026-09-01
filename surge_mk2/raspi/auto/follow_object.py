"""対象追従（Follow Object）— GUIで映像上にドラッグ選択した対象との車間距離を
保って追従する。

`cam_track_node.py` が前方カメラ＋LiDARの融合で作る `TargetTrack`
（`track/target`）を入力に、対象への方位をPure Pursuitで追い、車間距離の
比例制御で速度を決める。

## Phase 1: 制御則の本実装

`TargetTrack` は`tracking`/`lost`/`lost_ms`/`bearing`/`distance`/
`distance_valid`という**事実だけ**を運ぶ（`raspi/msgs/types.py`の docstring
参照）。ready/brakeへの変換・見失い時の減衰・距離不明時の待ち時間管理は、
すべてこの`plan()`の責務にする。

**初版は前進オンリー・停止のみの安全側スコープ**（ユーザー承認済み・
`docs/plans`の設計メモ参照）:

- 見失って停止条件に達した場合 → 停止のみ（探索・旋回等の追加動作なし）
- 近づきすぎた場合 → 停止のみ（後退はしない）

## 状態遷移

```
tracking=False                      → 停止（対象が選択されていない）
tracking=True, lost=True
    lost_ms <  lost_timeout_s*1000  → 直前の舵を保持しつつ速度だけ0へ減衰（ちらつき吸収）
    lost_ms >= lost_timeout_s*1000  → 停止
tracking=True, lost=False
    距離不明が distance_invalid_timeout_s 以上続く → 停止
    d <= stop_distance              → 停止（後退はしない）
    それ以外                         → Pure Pursuit + 車間距離の比例制御
```
"""

from __future__ import annotations

import math

from ..core.vehicle import Vehicle
from ..msgs.types import TOPIC_TRACK_TARGET, AutoState, TargetTrack, VehicleState
from ..nav.purepursuit import steer_for_target
from .base import ParamSpec, Planner

__all__ = ["FollowObject"]


class FollowObject(Planner):
    id = "follow_object"
    name = "対象追従"
    description = "映像でドラッグ選択した対象との車間距離を保って追従する。前進オンリー・停止のみの安全側スコープ"

    #: `cam_track_node.py` が publish する追跡結果を使う（`Scan` ではない）
    input_topic = TOPIC_TRACK_TARGET
    #: `cam_track_node`の実測ループ周期（2026-09-01、実車計測：待機/追跡とも
    #: 中央値約20ms・p90約30ms、NanoTrack推論はボトルネックにならない）の
    #: 10倍以上の余裕を見つつ、`ftg_cam`/`line_trace`（500ms、CPU推論を前提に
    #: 大きめ）より十分短くして、cam_track_nodeが落ちたときの検出を速くする
    stale_ms = 200
    #: ギャップ探索系の判断根拠（`free_ahead`/`nearest`/`gap`/`valid_ratio`）は
    #: 一切書かないので空にする。車間距離・方位は `AutoState.target_*` に書くが、
    #: 汎用の「判断」欄ではなく `follow_object` 専用UI（`AutoPanel.tsx`）が読む
    stats = ()

    #: 見失い減衰の時定数を`lost_timeout_s`から導く係数。3τで約95%減衰なので、
    #: タイムアウトに達する頃にはほぼ停止している（専用パラメータを増やさずに
    #: `steer_tau`と同じ1次遅れの考え方を流用する）
    _LOST_DECAY_DIV = 3.0

    params = (
        ParamSpec(key="follow_distance", label="目標車間距離", min=0.3, max=3.0, step=0.1,
                  default=1.0, unit="m",
                  note="対象とこの距離を保つよう速度を比例制御する"),
        ParamSpec(key="stop_distance", label="停止する車間距離", min=0.1, max=1.5, step=0.05,
                  default=0.5, unit="m",
                  note="★安全策。対象がこれより近づいたら停止のみ（後退はしない）"),
        ParamSpec(key="kp_speed", label="車間距離Pゲイン", min=0.1, max=3.0, step=0.1,
                  default=1.0, unit="1/s",
                  note="v = ゲイン×(車間距離−目標車間距離)。上げるほど追従が敏感になるが"
                       "距離推定のノイズをそのまま速度に伝えやすくなる"),
        ParamSpec(key="max_speed", label="最高速度", min=0.05, max=2.0, step=0.05,
                  default=1.0, unit="m/s",
                  note="★io_node の --max-speed を超えても Pi 側で切り捨てられるだけ"),
        ParamSpec(key="look_k", label="前方注視の速度係数", min=0.0, max=2.0, step=0.05,
                  default=0.7, unit="s",
                  note="Ld = 係数×速度 + 最小値。`follow_the_gap.py`/`line_trace.py`と同じ式"),
        ParamSpec(key="look_min", label="前方注視の最小値", min=0.15, max=1.5, step=0.05,
                  default=0.35, unit="m",
                  note="低速時の注視距離。小さすぎると舵が振動する"),
        ParamSpec(key="a_lat_max", label="旋回時の横加速度上限", min=0.5, max=8.0, step=0.1,
                  default=3.0, unit="m/s²",
                  note="★実車未計測の暫定値。実際に切る舵角から曲率κ=tan(δ)/Lを求め、"
                       "v ≤ sqrt(これ/κ) で速度を抑える（`follow_the_gap.py`と同じ式）"),
        ParamSpec(key="steer_tau", label="舵の平滑化", min=0.0, max=0.5, step=0.01,
                  default=0.10, unit="s",
                  note="舵指令の1次遅れの時定数。0で平滑化なし。上げると滑らかだが反応が鈍る"),
        ParamSpec(key="lost_timeout_s", label="見失いの許容時間", min=0.2, max=5.0, step=0.1,
                  default=1.5, unit="s",
                  note="対象を見失ってからこの時間が経つまでは、速度を減衰させつつ"
                       "直前の舵を保持する（ちらつき吸収）。超えたら停止"),
        ParamSpec(key="distance_invalid_timeout_s", label="距離不明の許容時間", min=0.1, max=3.0,
                  step=0.1, default=0.8, unit="s",
                  note="LiDARの実測距離が取れない時間がこれを超えたら停止する"
                       "（方位は分かっていても車間距離が分からないまま進ませないため）"),
    )

    def __init__(self) -> None:
        self.vehicle = Vehicle.load()
        self._steer = 0.0
        self._speed = 0.0
        #: 直近に得られた有効な距離。**distance_valid=Falseの間はこれを使い続ける**
        #: （`distance_invalid_timeout_s`以内の一時的な欠測を吸収するため）
        self._last_valid_distance: float | None = None
        self._distance_invalid_ms = 0.0

    def reset(self) -> None:
        self._steer = 0.0
        self._speed = 0.0
        self._last_valid_distance = None
        self._distance_invalid_ms = 0.0

    def plan(self, track: TargetTrack, vs: VehicleState | None,
             p: dict[str, float], dt: float) -> AutoState:
        st = AutoState(mode=self.id, planner=self.name)

        if not track.tracking:
            # **前のモードの続きから動き出さない**のと同じ理由——選択が外れた
            # 状態から動き出すときに、前回追従していた対象の速度・舵が
            # 残っていてはいけない
            self.reset()
            st.reason = "対象が選択されていません"
            return st                      # ready=False ＝ 制動

        # ── 事実の写し。安全判断（ready/brake）は下の各分岐が決める ──
        st.target_locked = not track.lost
        st.target_lost = track.lost
        st.target_lost_ms = track.lost_ms
        st.target_bearing_deg = math.degrees(track.bearing)
        if track.distance_valid:
            st.target_distance = track.distance
        elif self._last_valid_distance is not None:
            st.target_distance = self._last_valid_distance

        if track.lost:
            return self._plan_lost(st, track, p, dt)

        return self._plan_tracking(st, track, vs, p, dt)

    # ── 見失い中 ──

    def _plan_lost(self, st: AutoState, track: TargetTrack,
                   p: dict[str, float], dt: float) -> AutoState:
        timeout_ms = p["lost_timeout_s"] * 1000.0
        if track.lost_ms >= timeout_ms:
            self._speed = 0.0
            st.ready = True
            st.brake = True
            st.target_steer = self._steer      # 舵は直前値を保持（曲がりながら止まる）
            st.target_speed = 0.0
            st.reason = f"対象を見失って{track.lost_ms / 1000:.1f}s経過したため停止"
            return st

        # ★ちらつき吸収。舵は直前値を保持し、速度だけ`lost_timeout_s`の1/3を
        # 時定数に0へ減衰させる（方位が分からない間に舵まで動かすと、直前の
        # 誤差を増幅する側に振れかねないため、速度だけを絞る）
        tau = max(p["lost_timeout_s"] / self._LOST_DECAY_DIV, 1e-3)
        alpha = 1.0 - math.exp(-dt / tau) if dt > 0 else 1.0
        self._speed += (0.0 - self._speed) * alpha
        st.ready = True
        st.target_steer = self._steer
        st.target_speed = self._speed
        st.reason = f"対象を見失い中（{track.lost_ms / 1000:.1f}s）。速度を減衰させ直前の舵を維持"
        return st

    # ── 追跡中（lost=False） ──

    def _plan_tracking(self, st: AutoState, track: TargetTrack, vs: VehicleState | None,
                       p: dict[str, float], dt: float) -> AutoState:
        if track.distance_valid:
            self._last_valid_distance = track.distance
            self._distance_invalid_ms = 0.0
        else:
            self._distance_invalid_ms += dt * 1000.0 if dt > 0 else 0.0

        if self._last_valid_distance is None:
            st.reason = "対象までの距離を計測中です"
            return st                          # ready=False ＝ 制動

        if self._distance_invalid_ms >= p["distance_invalid_timeout_s"] * 1000.0:
            self._speed = 0.0
            st.ready = True
            st.brake = True
            st.target_steer = self._steer
            st.target_speed = 0.0
            st.reason = f"距離が{self._distance_invalid_ms / 1000:.1f}s不明のため停止"
            return st

        d = self._last_valid_distance
        stop_d = p["stop_distance"]
        if d <= stop_d:
            self._speed = 0.0
            st.ready = True
            st.brake = True
            st.target_steer = self._steer
            st.target_speed = 0.0
            st.reason = f"車間 {d:.2f}m で停止（停止距離 {stop_d:.2f}m）"
            return st

        # ── Pure Pursuit。`eta`=対象の方位角という既存の意味と一致する ──
        max_steer = self.vehicle.max_steer
        v_now = vs.speed if vs is not None else 0.0
        ld = p["look_k"] * v_now + p["look_min"]
        target = steer_for_target(track.bearing, ld, self.vehicle.wheelbase, max_steer)
        tau = p["steer_tau"]
        alpha = 1.0 if tau <= 1e-3 or dt <= 0 else 1.0 - math.exp(-dt / tau)
        self._steer += (target - self._steer) * alpha
        st.target_steer = self._steer

        # ── 車間距離の比例制御。前進のみ・後退なし ──
        v = p["kp_speed"] * (d - p["follow_distance"])
        v = max(0.0, min(p["max_speed"], v))

        # 曲率ベースの物理的な上限（`follow_the_gap.py`の⑥と同じ式。
        # **クランプ後の`target`（平滑化前）から求める**——舵が頭打ちのときも
        # 実際に描く円弧に対して正しく、平滑化の遅れぶん速度側が先回りして落ちる）
        kappa = abs(math.tan(target) / self.vehicle.wheelbase)
        v_curve = math.sqrt(p["a_lat_max"] / kappa) if kappa > 1e-6 else math.inf
        v = min(v, v_curve)

        self._speed = v
        st.target_speed = v
        st.ready = True
        st.reason = (f"車間 {d:.2f}m（目標{p['follow_distance']:.2f}m）・"
                     f"方位{math.degrees(track.bearing):+.0f}°")
        return st
