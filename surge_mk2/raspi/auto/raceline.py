"""レーシングライン走行 — 地図を作り、アウトインアウトの経路を引いて周回する。

`docs/architecture.md` のロードマップでいう Phase 3（自己位置推定と環境地図）と
Phase 4（大域経路計画 + 経路追従）。**アルゴリズムは `raspi/nav/` にあり、
ここは段を切り替える状態機械だけ**を持つ。

## 3つの段

| 段 | やること | 走らせ方 |
|---|---|---|
| `EXPLORE` | SLAM で地図を育てる。既定 2 周 | **`FollowTheGap` に委譲**（合成。継承ではない） |
| `BUILD` | 地図を凍結 → 中心線 → レーシングライン → 速度 | `ready=False` ＝ その場で停止 |
| `RACE` | 凍結地図で自己位置だけ推定し、経路を追う | Pure Pursuit |

### EXPLORE を 2 周にしてある理由

1周では**動く物が地図から消えない**。レイに沿った自由空間の彫り込みで痕跡が
消えるには「その場所を別の時刻にもう一度見る」必要があり、1周だと横切られた
場所を二度と見ないまま凍結してしまう。地図の質はレーシングラインの質に
直結するので、この1周は安い買い物。

### BUILD で止まるのは仕様

周回が終わって地図を凍結した直後なので、**止まっているのが正しい状態**。
`ready=False` は planning_node が制動に読み替える（`base.py` の約束3）ので、
「作っている間は止まる」が自然に表現できる。

計算は 2 周期に分けてある。`plan()` が 1 回で 100ms 掛かると `auto/cmd` が
途絶し、telemetry_node の 200ms 途絶検出が制動に落とす（`test_auto.py` の
`test_stale_auto_cmd_becomes_a_brake`）。安全側に倒れるとはいえ `reason` が
出ないので、原因が読めなくなる。

### 段の切り替えは engage を落とさない

`reset()`（モード切替・disengage）で全部捨てて `EXPLORE` に戻る。つまり
**engage し直せば地図から作り直し**。パラメータのスライダを動かしても
`reset()` は呼ばれない（`planning_node._apply_ctrl`）ので、走りながら
λ や速度を触っても地図は生き残る。

## 迷ったら止まる（`base.py` の約束3）

- スキャンマッチの得点が閾値未満 ＝ 自分がどこに居るか分からない
- 横偏差が上限超え ＝ 経路から落ちた
- `vs` が無い ＝ ジャイロが来ていない（SLAM の予測が作れない）
- 前方の帯に動的障害物 ＝ 制動（**避けない。止まる**）
"""

from __future__ import annotations

import math
from dataclasses import replace

import numpy as np

from ..core.vehicle import Vehicle
from ..msgs.types import AutoMap, AutoState, Scan, VehicleState
from ..nav import centerline as cl_mod
from ..nav import obstacles as obs_mod
from ..nav import raceline as rl_mod
from ..nav.grid import pack_trinary
from ..nav.purepursuit import PursuitConfig, follow
from ..nav.slam import Slam, SlamConfig
from .base import ParamSpec, Planner
from .follow_the_gap import FollowTheGap

__all__ = ["RaceLine"]

EXPLORE, BUILD, RACE = "EXPLORE", "BUILD", "RACE"

#: 地図の刻みと広さ。**パラメータにしていない。** 走行中にスライダで変えられると
#: 地図を作り直すしかなくなり、「触ったら消えた」という事故になる。
#:
#: ★ 2.5cm にしてあるのは**分離帯があるコースのため**。コースが2分割されると
#: 車線幅は 0.45m しかなく、そこから車体半幅 0.09m×2 と余裕 0.05m×2 を引くと
#: **振れ幅は 0.08m しか残らない**。5cm 格子だと壁の表現が ±1セル、幅測定の
#: 膨張が 1セルで、誤差だけで 5〜10cm 出てアウトインアウトが誤差に埋もれる。
#:
#: 細かくしたぶん**広さを 16m に削って**計算量を戻してある（480²→640² で 1.8 倍）。
#: 周長 25m 程度のコースなら入る。入り切らないと `OccGrid.out_of_bounds` が
#: 増え続けるので、`reason` に出して気づけるようにしてある
MAP_RES = 0.025
MAP_SIZE_M = 16.0
#: 格子の外に出たレイがこの本数を超えたら警告する。**コースが地図に入っていない**
OUT_OF_BOUNDS_WARN = 2000
#: 中心線の点の間隔 [m]。曲率は2階差分なので、細かすぎるとノイズを拾う
PATH_STEP = 0.10
#: 道幅の測定でこれ以上は探さない [m]。コースの外へ幅が伸びるのを防ぐ
MAX_TRACK_WIDTH = 3.0

#: 周回とみなす累積回頭の割合。**1周＝360° ちょうどを待たない**（コースの
#: 始点が直線の途中だと、位置が戻っても回頭がわずかに足りないことがある）
LAP_TURN_RATIO = 330.0 / 360.0
#: 周回判定に要る最低走行距離 [m]。発進直後の誤検出を殺す
LAP_MIN_DIST = 3.0
LAP_NEAR_M = 0.8                           #: 始点にこれだけ近づけば「戻った」
LAP_YAW_DEG = 35.0                         #: 向きの一致。逆走・往復を弾く
LAP_CONFIRM = 3                            #: 連続でこれだけ成立したら確定

#: 地図を GUI へ作り直す間隔（点群の周回数）。10Hz なので約 1Hz。
#: **凍結の瞬間だけはこれを無視して必ず作り直す**
_MAP_EVERY = 10


class RaceLine(Planner):
    id = "raceline"
    name = "レーシングライン"
    description = "LiDAR で地図を作り、アウトインアウトの経路を引いて周回する"

    params = (
        # ── 自己位置推定 ──
        ParamSpec(key="lidar_only", label="LiDARのみで自己位置推定", min=0, max=1,
                  step=1, default=0, unit="",
                  note="★実験用。1にすると、ジャイロと`speed`（車輪由来）を"
                       "SLAMの入力から外し、スキャン同士の直接照合（scan_to_scan、"
                       "`nav/scan2scan.py`）だけで1周期ぶんの動きを推定する。"
                       "IMU/オドメトリへの依存度を切り分けたいときに使う。"
                       "既定0（ジャイロ+speedを使う、通常の動作）"),

        # ── 地図を作る周（EXPLORE） ──
        ParamSpec(key="explore_laps", label="地図を作る周回数", min=1, max=4, step=1,
                  default=2, unit="周",
                  note="★1周だと動く物が地図から消えない。2周が既定"),
        ParamSpec(key="explore_speed", label="地図作成中の速度", min=0.1, max=1.5,
                  default=0.45, step=0.05, unit="m/s",
                  note="この段は Follow the Gap で走る。速いと点群が歪んで地図が荒れる"),
        ParamSpec(key="explore_gap_min", label="通れると見なす距離", min=0.2, max=3.0,
                  default=0.6, step=0.05, unit="m",
                  note="★この距離以上が続く方向を「隙間」と数える。**道幅の半分より"
                       "小さくすること**。分離帯で車線が 0.42m になると、既定の 1.0m では"
                       "コーナーで隙間が1つも見つからず止まる"),
        ParamSpec(key="explore_stop", label="停止する前方距離", min=0.05, max=1.0,
                  default=0.20, step=0.01, unit="m",
                  note="★正面余裕がこれを切ったら止まる。狭い車線のコーナーでは"
                       "正面 30cm まで詰まるので、0.35m のままだと入口で止まる"),
        ParamSpec(key="explore_bubble", label="安全バブル半径", min=0.05, max=0.6,
                  default=0.12, step=0.01, unit="m",
                  note="最近傍の周りを侵入禁止にする半径。**車線の半幅より小さく**。"
                       "大きいと狭い車線で視野が全部塞がる"),

        # ── 経路の形（BUILD） ──
        ParamSpec(key="line_lam", label="中心線への寄せ", min=0.005, max=2.0,
                  default=0.1, step=0.005, unit="",
                  note="★アウトインアウトの強さ。下げるほど振れ幅が大きい。"
                       "上げると中心線寄り＝最短経路寄りになる"),
        ParamSpec(key="line_margin", label="壁からの余裕", min=0.0, max=0.4,
                  default=0.05, step=0.01, unit="m",
                  note="車体半幅に足す安全代。地図の誤差と局在化の誤差をここで飲む"),
        ParamSpec(key="line_passes", label="経路の作り直し回数", min=1, max=3,
                  default=2, step=1, unit="回",
                  note="経路が動くと法線の向きも変わるので測り直す。"
                       "悪くなった回は自動で捨てる"),

        # ── 速度プロファイル ──
        ParamSpec(key="v_max", label="最高速度", min=0.2, max=3.0, default=1.2,
                  step=0.05, unit="m/s",
                  note="★io_node の --max-speed を超えても Pi 側で切り捨てられるだけ"),
        ParamSpec(key="v_min", label="最低速度", min=0.1, max=1.0, default=0.35,
                  step=0.05, unit="m/s", note="いちばんきついコーナーでもこれ以下にしない"),
        ParamSpec(key="a_lat", label="横加速度の上限", min=0.5, max=8.0, default=2.5,
                  step=0.1, unit="m/s²",
                  note="コーナーの速度を決めているのはこれ。上げると曲がりきれなくなる"),
        ParamSpec(key="a_accel", label="加速度の上限", min=0.2, max=5.0, default=1.2,
                  step=0.1, unit="m/s²", note="立ち上がりでどれだけ速度を戻せるか"),
        ParamSpec(key="a_brake", label="減速度の上限", min=0.2, max=5.0, default=1.8,
                  step=0.1, unit="m/s²",
                  note="★コーナー手前の減速開始点を決める。実車で止まれる値にすること"),

        # ── 追従（RACE） ──
        ParamSpec(key="look_k", label="前方注視の速度係数", min=0.0, max=2.0,
                  default=0.7, step=0.05, unit="s",
                  note="Ld = 係数×速度 + 最小値。上げると滑らかだがコーナーで内を切る"),
        ParamSpec(key="look_min", label="前方注視の最小値", min=0.15, max=1.5,
                  default=0.35, step=0.05, unit="m",
                  note="低速時の注視距離。小さすぎると舵が振動する"),
        ParamSpec(key="delay_s", label="遅延補償", min=0.0, max=0.4, default=0.15,
                  step=0.01, unit="s",
                  note="★舵が効き始めるまでの時間。この秒数だけ先の位置で判断する。"
                       "実測して合わせること（architecture.md §14）"),
        # ── 安全 ──
        ParamSpec(key="min_score", label="自己位置の信頼下限", min=0.1, max=0.8,
                  default=0.35, step=0.01, unit="",
                  note="★スキャンマッチの一致度がこれを切ったら止まる。"
                       "下げると見失ったまま走る"),
        ParamSpec(key="max_cross", label="横偏差の上限", min=0.1, max=1.5, default=0.5,
                  step=0.05, unit="m", note="経路からこれだけ離れたら止まる"),
        ParamSpec(key="obstacle_stop", label="障害物で止まる距離", min=0.0, max=4.0,
                  default=0.0, step=0.1, unit="m",
                  note="★これから通る帯に動く物が居たら制動する（**避けはしない**）。"
                       "**0 で無効。既定は無効にしてある** — 地図の誤差（数%の"
                       "セルは外れる）で壁を動く物と読み、走り出した直後に止まって"
                       "しまうため。自己位置と地図が十分良くなってから 1.2 くらいで"
                       "入れること。検出そのものは常に動いていて GUI に出る"),
        ParamSpec(key="obstacle_pad", label="壁の近くを無視する幅", min=0.05, max=0.6,
                  default=0.25, step=0.01, unit="m",
                  note="★壁からこの範囲に落ちた点は動的障害物と見なさない。"
                       "**地図の誤差（数%のセルは外れる）と自己位置の誤差の合計より"
                       "大きく取ること。** 小さいと壁を障害物と誤検出して止まり続ける"),
    )

    def __init__(self) -> None:
        self.vehicle = Vehicle.load()
        self.ftg = FollowTheGap()
        self.slam = Slam(SlamConfig(
            resolution=MAP_RES, size_m=MAP_SIZE_M,
            lidar_x=self.vehicle.lidar_x, lidar_y=self.vehicle.lidar_y))
        #: `self.slam.cfg.scan_to_scan` の現在値の写し。**`reset()` では触らない**
        #: （`Slam.reset()` は姿勢と地図だけを消し `cfg` はそのまま残るので、ここも
        #: 一緒に消すと「パラメータは変わっていないのに `cfg` だけ古くなる」事故になる）
        self._lidar_only = False
        self.reset()

    # ── 状態 ──

    def reset(self) -> None:
        self.phase = EXPLORE
        self.slam.reset()
        self.ftg.reset()
        self.laps = 0
        self.centerline: cl_mod.Centerline | None = None
        self.path: rl_mod.RaceLine | None = None
        self._lap_idx: list[int] = [0]
        self._lap_hits = 0
        self._build_step = 0
        self._build_error = ""
        self._freeze_requested = False
        self._hint = -1
        self._obs_pad = 0.15
        self._prev_obs: list[obs_mod.Obstacle] = []
        self._obs: list[obs_mod.Obstacle] = []
        self._map: AutoMap | None = None
        self._map_seq = -_MAP_EVERY
        self._map_frozen = False
        #: 中心線・経路が変わった印。**地図の版だけでは検出できない**
        self._map_dirty = True

    def request_freeze(self) -> None:
        """GUI の「地図を確定」。**自動判定が滑ったときの逃げ道。**"""
        self._freeze_requested = True

    def request_clear(self) -> None:
        """GUI の「地図を削除」。**全部捨てて EXPLORE からやり直す。**

        `reset()` と同じ。engage を落とさずに地図だけ作り直せる口が要る
        （途中で地図が崩れたとき、いちいち disengage → engage し直すのは面倒だし、
        その操作は ARM の保持まで巻き込む）。

        削除した直後は経路が無いので、RACE 段だったとしても `ready=False` に
        落ちて制動する。**走りながら押しても安全側**。
        """
        self.reset()

    # ── 本体 ──

    def plan(self, scan: Scan, vs: VehicleState | None,
             p: dict[str, float], dt: float) -> AutoState:
        st = AutoState(mode=self.id, planner=self.name, phase=self.phase)

        if vs is None:
            # オドメトリが無いと脱スキューも探索初期値も作れない。**推測で埋めない**
            st.reason = "車両状態がまだ届いていない"
            return st

        # ★ `lidar_only` が立っていれば、ジャイロと `speed` を SLAM から隠す。
        #   `Slam.update()` は `None` を渡されると自分の前回推定（等速度モデル）で
        #   代用しつつ、`scan_to_scan`（LiDAR のスキャン同士の直接照合。
        #   `nav/scan2scan.py`）で動きを磨く。IMU/オドメトリへの依存を切り分ける
        #   実験用の入口（`raspi/nav/slam.py` の `SlamConfig.scan_to_scan` docstring）
        lidar_only = bool(p.get("lidar_only", 0.0))
        if lidar_only != self._lidar_only:
            self._lidar_only = lidar_only
            self.slam.cfg = replace(self.slam.cfg, scan_to_scan=lidar_only)

        if lidar_only:
            u = self.slam.update(scan, dt, yaw_rate=None, speed=None)
        else:
            # ★ 回頭は**ジャイロ**、前進は `speed`（射影済み・ローパス済みの値）。
            #   `wheel_speed[]` は低速でばたつくので渡してはいけない（`nav/slam.py`）
            u = self.slam.update(scan, dt, yaw_rate=vs.yaw_rate, speed=vs.speed)
        st.pose_x, st.pose_y, st.pose_yaw = u.x, u.y, u.yaw
        st.match_score = u.score
        st.lap_progress = self.slam.lap_progress()
        st.laps = self.laps

        if self.phase == EXPLORE:
            return self._explore(st, scan, vs, p, dt, u.lost)
        if self.phase == BUILD:
            return self._build(st, p)
        return self._race(st, scan, vs, p, u.lost)

    # ── EXPLORE ──

    def _explore(self, st: AutoState, scan: Scan, vs: VehicleState,
                 p: dict[str, float], dt: float, lost: bool) -> AutoState:
        lap_msg = ""
        lap_done, explore_done = self._check_lap(p)
        if lap_done:
            # **凍結する前に、この周のループを閉じる。** 1周ぶんの誤差を軌跡へ
            # 配って地図を焼き直す（`nav/slam.py`）。★ 以前は `explore_laps` の
            # 最終周でしか呼んでおらず、2周目以降の壁が前の周とずれたまま
            # 積み上がって oval が「渦巻き」になっていた（docs/progress_archive.md
            # 「oval が渦巻きになる」節）。**周回を検出するたび**に閉じることで
            # 毎回のドリフトを1周ぶん（20〜30cm程度）に抑える
            residual = self.slam.close_loop(self.slam.last_points)
            if explore_done:
                self.slam.freeze()
                self.phase = st.phase = BUILD
                self._build_step = 0
                st.reason = (f"地図を確定した（最終周の残差 {residual * 100:.0f}cm を"
                             f"配って焼き直し）。経路を作っている"
                             if residual > 0 else
                             "地図を確定した（残差を測れず、焼き直しは省略）。経路を作っている")
                return st
            lap_msg = (f"{self.laps}周目のループを閉じた（残差 {residual * 100:.0f}cm）"
                       if residual > 0 else
                       f"{self.laps}周目のループを閉じた（残差を測れず継続）")

        # 走らせるのは Follow the Gap。**狭い車線に効く4つだけ差し替える**
        # （残りは FTG の既定値。全部をここに並べるとスライダが倍になる）
        fp = FollowTheGap.merged({
            "max_speed": p["explore_speed"],
            "gap_min": p["explore_gap_min"],
            "stop_dist": p["explore_stop"],
            "bubble_m": p["explore_bubble"],
        })
        sub = self.ftg.plan(scan, vs, fp, dt)

        st.ready = sub.ready
        st.brake = sub.brake
        st.target_speed = sub.target_speed
        st.target_steer = sub.target_steer
        st.heading = sub.heading
        st.gap_start_deg = sub.gap_start_deg
        st.gap_end_deg = sub.gap_end_deg
        st.bubble_start_deg = sub.bubble_start_deg
        st.bubble_end_deg = sub.bubble_end_deg
        st.free_ahead = sub.free_ahead
        st.nearest = sub.nearest
        st.nearest_deg = sub.nearest_deg
        st.valid_ratio = sub.valid_ratio

        want = int(p["explore_laps"])
        head = f"地図作成 {self.laps}/{want}周（{st.lap_progress:.2f}周ぶん回頭）"
        if self.slam.grid.out_of_bounds > OUT_OF_BOUNDS_WARN:
            # **コースが地図の枠に収まっていない。** 気づかないと、はみ出した側の
            # 壁が無いまま経路が引かれる
            head += f"・★地図からはみ出している（{self.slam.grid.out_of_bounds}本）"
        if lost:
            # 地図は汚していない（`Slam.update`）。走りは FTG のままでよい
            st.reason = f"{head}・自己位置を見失っている"
        elif lap_msg:
            st.reason = f"{head}・{lap_msg}"
        else:
            st.reason = f"{head}・{sub.reason}"
        return st

    def _check_lap(self, p: dict[str, float]) -> tuple[bool, bool]:
        """周回が終わったか。**主判定は累積回頭**（`nav/slam.py` の docstring）。

        :returns: `(この周が終わったか, 探索そのものが終わったか)`。
            前者が `True` のときは毎回 `close_loop()` を呼ぶこと。後者は
            `explore_laps` に達した（＝ BUILD へ移ってよい）ときだけ `True`
        """
        if self._freeze_requested:
            self._freeze_requested = False
            self._lap_idx.append(len(self.slam.trajectory))
            self.laps = max(1, self.laps)
            return True, True
        if not self.slam.trajectory:
            return False, False

        want = int(p["explore_laps"])
        target = self.laps + 1
        ok = (self.slam.lap_progress() >= LAP_TURN_RATIO * target
              and self.slam.distance >= LAP_MIN_DIST * target
              and self._near_start())
        self._lap_hits = self._lap_hits + 1 if ok else 0
        if self._lap_hits < LAP_CONFIRM:
            return False, False

        self._lap_hits = 0
        self.laps += 1
        self._lap_idx.append(len(self.slam.trajectory))
        return True, self.laps >= want

    def _near_start(self) -> bool:
        sx, sy, syaw = self.slam.trajectory[0]
        x, y, yaw = self.slam.pose
        if math.hypot(x - sx, y - sy) > LAP_NEAR_M:
            return False
        d = (yaw - syaw + math.pi) % (2.0 * math.pi) - math.pi
        return abs(math.degrees(d)) <= LAP_YAW_DEG

    # ── BUILD ──

    def _build(self, st: AutoState, p: dict[str, float]) -> AutoState:
        """経路を作る。**2周期に分ける**（1回で 100ms 掛けると指令が途絶する）。"""
        if self._build_error:
            st.reason = self._build_error
            return st
        try:
            if self._build_step == 0:
                traj = self._last_lap()
                if len(traj) < 20:
                    raise ValueError(f"軌跡が短すぎる（{len(traj)}点）")
                self.centerline = cl_mod.build(
                    self.slam.grid, traj, step=PATH_STEP,
                    max_width=MAX_TRACK_WIDTH)
                self._build_step = 1
                self._map_dirty = True
                st.reason = "経路を作っている 1/2（中心線）"
                return st

            assert self.centerline is not None
            self.path = rl_mod.optimize(
                self.slam.grid, self.centerline,
                half_width=self.vehicle.half_width, margin=p["line_margin"],
                lam=p["line_lam"], passes=int(p["line_passes"]),
                v_max=p["v_max"], v_min=p["v_min"], a_lat=p["a_lat"],
                a_accel=p["a_accel"], a_brake=p["a_brake"],
                max_width=MAX_TRACK_WIDTH)
        except Exception as e:                      # noqa: BLE001
            # **例外で planning_node を落とさない。** 落ちると理由が GUI に出ず、
            # 「engage したのに何も起きない」だけが残る
            self._build_error = f"経路を作れなかった: {e}"
            st.reason = self._build_error
            return st

        self.phase = st.phase = RACE
        self._hint = -1
        self._map_dirty = True             # ★ 作った経路を GUI へ送る
        st.reason = (f"経路ができた（{len(self.path)}点・"
                     f"1周 {self.path.length:.1f}m・"
                     f"{self.path.v.min():.2f}〜{self.path.v.max():.2f} m/s）")
        return st

    def _last_lap(self) -> np.ndarray:
        """**最後の1周ぶん**の軌跡。2周ぶんを渡すと中心線が二重になる。"""
        traj = self.slam.trajectory_array()
        if len(self._lap_idx) >= 2:
            a, b = self._lap_idx[-2], self._lap_idx[-1]
            if b - a >= 20:
                return traj[a:b]
        return traj

    # ── RACE ──

    def _race(self, st: AutoState, scan: Scan, vs: VehicleState,
              p: dict[str, float], lost: bool) -> AutoState:
        if self.path is None:
            st.reason = "経路がまだ無い"
            return st

        if lost or st.match_score < p["min_score"]:
            st.reason = (f"自己位置が信用できない（一致度 {st.match_score:.2f} < "
                         f"{p['min_score']:.2f}）")
            return st

        cfg = PursuitConfig(
            wheelbase=self.vehicle.wheelbase, max_steer=self.vehicle.max_steer,
            lookahead_k=p["look_k"], lookahead_min=p["look_min"],
            delay_s=p["delay_s"])
        pp = follow(self.path, self.slam.pose, vs.speed, vs.steer_actual,
                    cfg, hint=self._hint)
        self._hint = pp.index
        st.target_steer = pp.steer
        st.heading = pp.steer                       # GUI の狙い方位は舵角で代用する
        st.cross_track = pp.cross_track
        st.target_x, st.target_y = pp.target

        if abs(pp.cross_track) > p["max_cross"]:
            st.reason = (f"経路から {abs(pp.cross_track) * 100:.0f}cm 離れた"
                         f"（上限 {p['max_cross'] * 100:.0f}cm）")
            return st

        # ── 動的障害物。**避けずに止まる**（`__init__` の docstring） ──
        # 検出そのものは常に回して GUI に出す。**止まるかどうかだけを切る**
        # （見えていないのか、見えていて無視しているのかを区別できるように）
        self._obs_pad = p["obstacle_pad"]
        self._update_obstacles(scan)
        st.obstacles = [v for o in self._obs for v in (o.x, o.y, o.r)]
        hit, dist = obs_mod.blocking(
            self._obs, self.path.xy, pp.index,
            half_width=self.vehicle.half_width + p["line_margin"],
            ahead_m=p["obstacle_stop"] + self.vehicle.front_overhang,
            step=self.path.length / len(self.path),
            skip_m=self.vehicle.front_overhang)

        st.ready = True
        st.target_speed = pp.speed
        st.free_ahead = dist if hit is not None else math.inf

        stop_at = p["obstacle_stop"]
        if (stop_at > 0.0 and hit is not None
                and dist - self.vehicle.front_overhang <= stop_at):
            st.brake = True
            st.target_speed = 0.0
            ang = math.degrees(math.atan2(hit.y - st.pose_y, hit.x - st.pose_x)
                               - st.pose_yaw)
            ang = (ang + 180.0) % 360.0 - 180.0
            st.reason = (f"前方 {max(0.0, dist - self.vehicle.front_overhang):.1f}m "
                         f"（{ang:+.0f}°）に障害物。停止")
            return st

        st.reason = (f"{self.laps}周走行中・速度 {pp.speed:.2f} m/s・"
                     f"横偏差 {pp.cross_track * 100:+.0f}cm")
        return st

    def _update_obstacles(self, scan: Scan) -> None:
        from ..nav.deskew import deskew
        pts = deskew(scan, self.slam.speed, self.slam.yaw_rate,
                     mount_x=self.vehicle.lidar_x, mount_y=self.vehicle.lidar_y,
                     max_range=self.slam.cfg.max_range)
        now = obs_mod.detect(self.slam.grid, pts, self.slam.pose,
                             wall_pad=self._obs_pad)
        self._obs = obs_mod.confirm(now, self._prev_obs)
        self._prev_obs = now

    # ── GUI へ渡す重い状態 ──

    def snapshot(self) -> AutoMap | None:
        """地図と経路。**`map_seq` が変わったときだけ planning_node が流す。**

        **圧縮は毎回やらない。** 地図は点群が届くたび（10Hz）に変わるが、
        640×640 を詰め直すのは 1 周期の予算に対して安くない。育っていく様子が
        見えれば十分なので、EXPLORE 中は `_MAP_EVERY` 周に1回に間引く。

        **★ 中心線と経路が変わったときも作り直す。** 地図の版（`OccGrid.seq`）
        だけを見ていると、凍結**後**に作られるレーシングラインが永久に送られない
        （凍結した瞬間に1枚送って、以後 `seq` は動かないため）。実機の GUI で
        「段は RACE なのに経路は未生成」と表示されたのはこれ。
        """
        g = self.slam.grid
        fresh = g.seq - self._map_seq
        if (self._map is not None and not self._map_dirty
                and fresh < _MAP_EVERY and g.frozen == self._map_frozen):
            return self._map

        m = AutoMap(map_seq=g.seq, resolution=g.resolution,
                    origin_x=g.origin[0], origin_y=g.origin[1],
                    width=g.width, height=g.height,
                    cells=pack_trinary(g.trinary()))
        if self.centerline is not None:
            m.centerline = self.centerline.xy.reshape(-1).tolist()
        if self.path is not None:
            m.raceline = self.path.xy.reshape(-1).tolist()
            m.raceline_v = self.path.v.tolist()
        self._map = m
        self._map_seq = g.seq
        self._map_frozen = g.frozen
        self._map_dirty = False
        return m
