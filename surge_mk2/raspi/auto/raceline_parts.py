"""`Slam2dRaceLine`（`raspi/auto/slam2d_raceline.py`）の状態を、責務ごとに分けて持つ部品。

## なぜ分けたか

`Slam2dRaceLine` の属性は 200 を超え、その大半が `reset()` の中で初めて定義されていた。どこで
定義された属性か見えにくく、綴りの誤りやリセット漏れの温床になる（`docs/techdebt_2026_11_18.md` 中6）。
ここでは状態を責務ごとの小さなクラスに移し、**属性はクラスの宣言（dataclass の欄）で定義**し、
`reset()` は「宣言どおりの初期値に戻す」だけにする。

## 振る舞いは変えない

- 部品は状態の入れ物だけで、判断のコードは `Slam2dRaceLine` に残してある（実機の自律走行の本体なので、
  まず置き場所だけを分けた）
- `Slam2dRaceLine` は分ける前の属性名（`self._lap_idx` など）を `Alias` で部品へつないでいる。
  テスト・`planning_node`・`sim.bench`・`slam2d_route` が使っている名前はそのまま読み書きできる
- 初期値は分ける前の `reset()` と同じ。毎回作り直していた物（リスト・`VelocityTracker` など）は
  `default_factory` で毎回作り直す
- ワーカーの計算（`*_job`）は部品の `reset()` では消さない。消す前に取り消す（`Future.cancel`）必要が
  あるので、`Slam2dRaceLine.reset()` が先に `_drop_job` で取り消してから部品を戻す
"""

from __future__ import annotations

import math

from dataclasses import dataclass, field, fields
from typing import TYPE_CHECKING

import numpy as np

from ..nav import obstacles as obs_mod
from ..nav.signals import SignalLights

if TYPE_CHECKING:
    from slam2d.core.locscore import LocateScorer

    from ..nav import avoid as av_mod
    from ..nav import overtake as ot_mod
    from ..nav import raceline as rl_mod
    from ..msgs.types import AutoMap
    from .park_to_point import ParkToPoint

__all__ = ["Alias", "AvoidState", "DetourState", "FollowState", "GripState", "LapCounter", "LineState",
           "LocateState", "MapPublish", "MapState", "ObstacleState", "RouteCache", "SafetyState"]


class Alias:
    """`obj.<part>.<name>` を、持ち主のクラスの属性としても読み書きできるようにする。

    分ける前の属性名を外（テスト・`sim.bench`・派生クラス）が使っているので、名前を残すために使う。
    データ記述子（`__set__` を持つ）なので、インスタンスの `__dict__` より優先される。
    """

    __slots__ = ("part", "name")

    def __init__(self, part: str, name: str) -> None:
        self.part = part
        self.name = name

    def __get__(self, obj, owner=None):
        if obj is None:
            return self
        return getattr(getattr(obj, self.part), self.name)

    def __set__(self, obj, value) -> None:
        setattr(getattr(obj, self.part), self.name, value)


class _Part:
    """dataclass の欄の既定値へ戻す `reset()` を持たせる土台。"""

    #: `reset()` で戻さない欄（ワーカーの計算。取り消してから `None` にするのは持ち主の仕事）
    _keep: tuple[str, ...] = ()

    def reset(self) -> None:
        fresh = type(self)()
        for f in fields(self):
            if f.name not in self._keep:
                setattr(self, f.name, getattr(fresh, f.name))


@dataclass
class LapCounter(_Part):
    """周回の数え上げ（`_count_lap`・`_check_lap`）。"""

    laps: int = 0
    #: 地図作成中に周回を検出した軌跡の添字（`_check_lap`）
    lap_idx: list[int] = field(default_factory=lambda: [0])
    lap_hits: int = 0
    #: 最後の周回を検出した後の走り足し中なら (その時の走行距離, ループ拘束の本数)
    overrun: tuple[float, int] | None = None
    #: 前の周期の最寄り点の添字と、前回数えてから経路に沿って進んだ距離 [m]
    prev_idx: int = -1
    lap_s: float = 0.0
    #: 周回を数えるゲート（地図座標の点。`None` = 次に数えた経路の先頭の点にする）と、
    #: それを経路の添字に直した値・直した経路
    lap_gate: tuple[float, float] | None = None
    lap_gate_ref: np.ndarray | None = None
    lap_gate_idx: int = 0
    #: 前の周期にゲートから進んでいた割合 [0, 1)（-1 = まだ無い）
    lap_frac: float = -1.0


@dataclass
class MapState(_Part):
    """地図の作成（BUILD）・凍結（ループ閉じの確定）・保存・読み込みの状態。"""

    _keep = ("build_job",)

    build_step: int = 0
    build_error: str = ""
    freeze_requested: bool = False
    #: 地図の確定（`Slam2dNav.freeze_steps`）の途中ならそのジェネレータ
    freeze_iter: object | None = None
    #: BUILD で引いているライン（`_build`）
    build_job: object | None = None
    #: 保存済み地図から読み込んだ中心線（`centerline` は EXPLORE→BUILD で自前生成するものだけを
    #: 持つので、読み込み経路はここに置く。`snapshot()` が両方を出し分ける）
    loaded_centerline_xy: np.ndarray | None = None
    load_error: str = ""
    #: 地図の読み込みに失敗した（`request_load`）。解くまで指令を出さない
    load_failed: bool = False
    #: BUILD 完了時に自動保存した地図の名前（`DONE` 段の表示用）
    saved_map_name: str = ""
    #: いま走っている地図の名前（読み込んだ・作って保存した）。解除で地図を捨てないために持つ
    active_map_name: str = ""
    save_error: str = ""


@dataclass
class AvoidState(_Part):
    """障害物を避ける一時経路と、追い抜きの計画（`_overtake_step`・`_plan_avoid`）の状態。"""

    _keep = ("elastic_job",)

    #: 障害物を避けている一時経路（`avoid.plan_offset`）と、その元にした経路
    avoid: av_mod.AvoidPlan | None = None
    avoid_path: rl_mod.RaceLine | None = None
    avoid_base: rl_mod.RaceLine | None = None
    avoid_ms: float = 0.0
    #: 追い抜きの計画: 前の周期に選んだ候補と、その元の経路・弧長・法線
    ot_prev: ot_mod.Cand | None = None
    ot_base: np.ndarray | None = None
    ot_scum: np.ndarray | None = None
    ot_nrm: np.ndarray | None = None
    #: これまでに見た動く相手の位置（map フレーム、新しい順に `_OPP_TRAIL_N` 個まで）
    opp_trail: list[tuple[float, float]] = field(default_factory=list)
    #: 覚えた速さによる相手の予測の前側の外れ（実測、`overtake.calibrate_spread`）と、それを測った観測の時刻
    ot_spread: float | None = None
    ot_spread_t: float = -math.inf
    ot_status: str = ""
    ot_lead: ot_mod.Track | None = None
    #: 付いていく相手の予測の速さ（`OvertakeResult.lead_u`。付いていく制御が追跡の生の速さの代わりに使う）
    ot_lead_u: float | None = None
    #: 今走る線の上に居る動く相手と、そこまでの距離（車間を保つ上限、`_moving_ahead`）
    acc: tuple[obs_mod.Obstacle, float] | None = None
    #: 追い抜きの計算時間の記録（`sim.bench` が読む）
    avoid_ms_hist: list[float] = field(default_factory=list)
    #: 動く相手の直近の観測 (観測の時刻, 障害物)（`_GHOST_S`）
    ghosts: list[tuple[float, obs_mod.Obstacle]] = field(default_factory=list)
    #: 横へ避ける経路を引けなかったときの (障害物と条件のキー, 車の添字, 経路)。`_plan_avoid` 参照
    avoid_fail: tuple | None = None
    #: 直近に横へ避ける経路を引けなかった理由（表示用）
    avoid_why: str = ""
    #: 今の避ける経路の作り方（"横ずらし" = 決まった形、"引き直し" = `avoid.plan_elastic`、"追い抜き"）
    avoid_how: str = ""
    #: 今の避ける経路の元にした経路（見通しの上限をそこから取る）
    avoid_room_path: rl_mod.RaceLine | None = None
    #: 別プロセスで引き直している避ける経路（`avoid.plan_elastic`）と、その元の経路
    elastic_job: object | None = None
    elastic_base: np.ndarray | None = None


@dataclass
class FollowState(_Part):
    """周期ごとの経路の追従（`plan()` の `_track` と、点群の合間の `tick`）・経路への合流の状態。"""

    #: 追っている経路の最寄り点の添字（-1 = まだ無い）
    hint: int = -1
    #: 速度指令を前回計算した時刻 [ns]（`VehicleState.t_capture`）
    sff_t: int = 0
    #: 点群の合間の追従（`tick`）に使う、直近の `plan()` の結果:
    #: (追っている経路, 設定, 速度の上限, 追従の起点の添字)。`None` = 合間は更新しない
    tick_ctx: tuple | None = None
    tick_raw: float = float("inf")
    tick_path: rl_mod.RaceLine | None = None
    #: 経路に乗ったか（乗るまでは `join_speed` に抑える、`_join_cap`）
    joined: bool = False
    #: 経路へ合流する一時経路を今の乗りに行く間に作ったか（`_join_back`）・作れたか。作れたら `join_speed` で
    #: 抑えず、周回と同じ速度の計画（横・前後の上限）で乗りに行く
    join_tried: bool = False
    join_ok: bool = False
    #: 経路から外れて止まっている時間 [s] と、乗り直した回数（`_off_route`）
    off_t: float = 0.0
    rejoins: int = 0
    #: 止まってから経路の速度に届くまで（＝発進の間）True（`_launch_ok`）
    launch_armed: bool = False
    #: 直近の `plan()` の経過 [s]
    dt: float = 0.0


@dataclass
class LocateState(_Part):
    """保存済み地図の上で自己位置を探す（`LOCATE`、`_locate`）状態。"""

    #: `LOCATED` で見失った周期数
    located_lost: int = 0
    #: 進行中のグローバルローカリゼーション（`LOCATE` 段でのみ非 None）。C があれば分枝限定法の
    #: `globalmatch.MatchJob`（`_GmLocalizer` で包む）、無ければ総当たりの `GlobalLocalizer`
    localizer: object | None = None
    #: 地図ごとの分枝限定法の前処理（`globalmatch.GlobalMatcher`。地図の `seq` が変わったら作り直す）
    gmatch: object | None = None
    #: 粗探索の上位候補のうち、まだ仕上げていないもの（`None` = 粗探索中）と仕上げた結果
    loc_cands: list | None = None
    loc_scored: list = field(default_factory=list)
    #: 候補の採点器（地図ごとの前処理を持つ。LOCATE ごとに作る）
    loc_scorer: LocateScorer | None = None
    #: 地図パネルのクリックによる絞り込みヒント（map フレーム座標）
    loc_hint: tuple[float, float] | None = None
    #: `loc_hint` を受けた時刻（`time.monotonic()`）
    loc_hint_t: float = 0.0
    #: `_LOC_HINT_RADII` の何段目か（失敗するたびに広げる）
    loc_hint_stage: int = 0
    #: `LOCATE` 中に動いていた連続周期数
    loc_move_n: int = 0
    #: 地図作成直後に走り出すときの「直前の自己位置」（`request_load` 参照）
    locate_seed: tuple[float, float] | None = None
    #: 自動運転を解いて地図を読み直したときの、解く直前の姿勢 `(x, y, yaw)`（`on_disengage`）。
    #: `_locate` はまずこの姿勢の周りを 1 回だけ照合し、合えば全域を探さずに特定済みにする
    locate_pose: tuple[float, float, float] | None = None
    #: `on_disengage` の種の周りを探す半径 [m]。解いてから止まるまでに転がる分（None = 既定）
    locate_seed_r: float | None = None


@dataclass
class ObstacleState(_Part):
    """障害物の検出と、それで止まる・抜ける判断（`_detect_obstacles`・`_obstacle_response`）の状態。"""

    #: 障害物の候補にしてよいセル（`obs_mod.free_mask`）と、それを作った (地図の版, 幅)
    obs_free: np.ndarray | None = None
    obs_near: np.ndarray | None = None
    obs_wdist: np.ndarray | None = None
    obs_free_key: tuple | None = None
    #: 実際に使っている無視幅（`_obstacle_pad`）と、それを決めた経路
    obs_pad: float = 0.0
    obs_pad_ref: np.ndarray | None = None
    #: この周期、前方を塞ぐ障害物を見ているか（`_track`）
    hit_now: bool = False
    #: 障害物の速度を出す（動く物の予測に使う）
    vel_tracker: obs_mod.VelocityTracker = field(default_factory=obs_mod.VelocityTracker)
    obs_t: float = 0.0
    #: Hybrid A* の迂回先の向き（+1=左 -1=右 0=不明）
    detour_side: int = 0
    #: 直近の車速（障害物の「近い」の範囲に使う）
    v_now: float = 0.0
    #: 塞がれて止まっている時間・進めずに止まっている時間 [s]
    blocked_t: float = 0.0
    stuck_t: float = 0.0
    #: 障害物で制動してから検出が途切れている時間 [s]（`None` = 制動していない）
    brake_hold: float | None = None
    prev_obs: list[obs_mod.Obstacle] = field(default_factory=list)
    obs: list[obs_mod.Obstacle] = field(default_factory=list)


@dataclass
class RouteCache(_Part):
    """経路から作って使い回す物（見通し・左右の余裕・曲がり切れない区間・車体の余裕・速度プロファイルの鍵）。"""

    #: 見通しの距離（`_sight_cap`）と、それを掛けた経路（`_with_sight`）のキャッシュ
    sight: np.ndarray | None = None
    sight_ref: np.ndarray | None = None
    sight_seq: int = -1
    sighted: rl_mod.RaceLine | None = None
    sighted_src: rl_mod.RaceLine | None = None
    sighted_key: tuple | None = None
    sight_msg: str = ""
    #: 「曲がり切れない区間」の注記と、それを測った (経路, 曲率の上限)（`_tight_note`）
    tight_key: tuple | None = None
    tight_ref: np.ndarray | None = None
    tight_msg: str = ""
    #: 経路の各点から左右へずらせる量（`_avoid_room`）と、それを作った (経路, 地図の版, 合格線)
    room: tuple | None = None
    room_key: tuple | None = None
    room_ref: np.ndarray | None = None
    #: 壁と車体外形の余裕を測る距離場（地図の版ごとに作る、`_body_check`）
    body: rl_mod._BodyCheck | None = None
    body_seq: int = -1
    #: 今の経路の速度プロファイルを作った設定（`_retime`）
    speed_key: tuple | None = None


@dataclass
class SafetyState(_Part):
    """安全系の見張り（自己位置・据え切り・駆動 MD・電池）と灯火。"""

    #: 自己位置を信用できない状態が続いている時間 [s]（`_localization_ok`）
    coast_t: float = 0.0
    #: 自己位置を信用できないまま止まっている時間 [s]（`LOST_RELOCATE_S` で `LOCATE` へ戻る）
    lost_still_t: float = 0.0
    speed_now: float = 0.0
    yaw_rate_now: float = 0.0
    #: 停車中に舵を動かしている時間 [s]（手動の地図作成での据え切りの注意）と直前の舵角
    still_steer_t: float = 0.0
    prev_steer: float = 0.0
    #: 灯火（パッシング→ウィンカー）。`_signal_step` が毎周期 `st` へ書く
    signals: SignalLights = field(default_factory=SignalLights)
    #: 駆動 MD が片輪だけ無応答の状態が続いている時間 [s]（`_md_fault`）
    md_t: float = 0.0
    #: 駆動電池の電圧低下の警告が続いている時間 [s] と、それで止めたか（この走行の間は戻らない。`_low_batt`）
    batt_t: float = 0.0
    batt_stop: bool = False


@dataclass
class DetourState(_Part):
    """Hybrid A* で抜けている（`DETOUR`）状態。同じ障害物で試した回数（失敗が続いたら止まって待つ）。"""

    detour: ParkToPoint | None = None
    detour_error: str = ""
    detour_tries: int = 0


@dataclass
class LineState(_Part):
    """設定の変更・地図の読み込みで引き直しているライン（`_submit_line`・`_try_adopt_line`）。"""

    _keep = ("line_job",)

    #: 引き直しの計算（取り消しは持ち主の `_drop_job`）
    line_job: object | None = None
    #: 今のライン（引き直し中ならその計算）の設定。`None` = 引き直さない
    line_built: tuple | None = None
    #: 引き直しが済んで、乗り移れる場所を待っているライン
    line_next: rl_mod.RaceLine | None = None
    line_next_key: tuple | None = None


@dataclass
class MapPublish(_Part):
    """GUI へ配る地図（`snapshot()`）の状態。"""

    #: `map_ver` は戻さない（`AutoMap.route_seq` に載せる通し番号。版が戻ると GUI が古い地図と見分けられない）
    _keep = ("map_ver",)

    map: AutoMap | None = None
    #: 前回詰めた地図の版。初期値は持ち主の `reset()` が `-_MAP_EVERY` を入れる（定数が向こうにあるため）
    map_seq: int = 0
    map_frozen: bool = False
    map_dirty: bool = True
    #: `snapshot()` で `AutoMap` を詰め直した回数
    map_ver: int = 0


@dataclass
class GripState(_Part):
    """横グリップ・前後・アンダーステアの学習を走行へつなぐ状態（学習器そのものは `_grip`・`_us`）。

    `reset()` の前に持ち主が `_grip_save()` で保存する（読むのはここの `grip_name`・`grip_saved`・`us_saved`）。
    """

    #: `grip_tok` は戻さない（速度の作り直しの鍵に入れる通し番号。持ち主の `reset()` が 1 つ進める）
    _keep = ("grip_tok",)

    us_saved: tuple = (0.0, 0.0)
    grip_on: bool = False
    #: 読み込んだ地図の名前（学習した横グリップの保存先）
    grip_name: str = ""
    #: 今のラインを引いたときの学習値 `(横, 加速, 減速)`（`None` = 同定値で引いた、`_grip_line_stale`）
    grip_line: tuple | None = None
    #: 学習を止めている理由（空 = 学習してよい）と、それを決めた時刻 [ns]
    grip_gate: str = "走行していない"
    grip_gate_t: int = 0
    #: 今の場所で計画している前後の加速度 [m/s²]（`_speed_cmd`）
    grip_ax: float = 0.0
    #: 同じく、計画の横の使い具合（横加速度 ÷ 横の限界。前後の包絡の学習が使う）
    grip_u: float = 0.0
    #: 追いつく途中か（`_speed_cmd`）と、プロファイルの速度 [m/s]
    grip_catch: bool = False
    grip_vp: float | None = None
    #: この時刻 [ns] までは学習しない（別の線へ乗り換えた直後。横へ数 cm 跳ぶぶんが前兆に見える）と、その理由
    grip_quiet_ns: int = 0
    grip_quiet_why: str = ""
    #: 「舵が上限」が最後に掛かっていた時刻 [ns]（0 = 「舵が上限の直後」も解けている）と、その間の横偏差の最大 [m]
    grip_sat_ns: int = 0
    grip_sat_cross: float = 0.0
    #: 経路の各点の `(求める横加速度, 限界)`（`_grip_levels`）: [(経路の点, 鍵, 配列の組)]（直近の3本）
    grip_cache: list[tuple] = field(default_factory=list)
    #: 曲がり切れない区間の印（`_grip_tight`）
    grip_tight_arr: np.ndarray | None = None
    grip_tight_ref: np.ndarray | None = None
    grip_tight_key: tuple | None = None
    #: 上げた速度を車の先にだけ反映するための上限（`_grip_hold`）:
    #: (経路の点, 点ごとの上限, 窓の始まりの添字, 窓の点の数)
    grip_hold_cap: tuple | None = None
    grip_plain: tuple | None = None
    #: 速度へ反映済みの学習の版 `(ver, ver_down)` と、速度の作り直しの鍵に入れる通し番号
    grip_ver: tuple = (-1, -1)
    grip_tok: int = 0
    #: 前兆で落とした速度をこの周期の指令に掛けたか
    grip_capped: bool = False
    grip_saved: tuple = (0, 0.0)
    grip_save_t: float = 0.0
