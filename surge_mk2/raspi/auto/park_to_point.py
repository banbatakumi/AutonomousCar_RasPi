"""駐車（クリック指定）— LiDAR画面をクリック+ドラッグして指定した位置+向きへ、
事前地図なしで自律的に接近して停止する。

    [目標のクリック] → ScanAnchor（2フレーム登録で姿勢）
                     → LocalMap（局所占有格子＋ESDF）
                     → Hybrid A*（障害物を避ける経路）
                     → 閉ループ追従（横偏差・向き偏差のフィードバック）
                     → 掃引ベースの安全停止

## 自己位置: スキャンアンカ（`raspi/nav/scan_anchor.py`）

**`slam2d`は使わない**（大域整合性の機構＝ポーズグラフ・ループ閉じ・`g2opy`が
現状不安定で、駐車はそれを必要としないため）。代わりに、クリックした瞬間の
スキャンを原点キーフレームとして2フレーム登録（点対線ICP、
`raspi/nav/scan2scan.py`）で姿勢を保つ。地図もポーズグラフも作らない。

これが要る理由: ユーザーが壁際をクリックしたときの意図は「**あの壁に対して**
この位置」なので、推定がドリフトすると**駐車枠そのものが実世界の壁に対して
静かに動き**、plannerは誤った場所で「完了」と言う。誤差が誤差として現れず、
成功として現れる。

シム実測（センサ誤差あり: ジャイロバイアス0.5°/s・オドメトリ+2%）:

| 自己位置 | 成功 | 最終位置誤差(真値) |
|---|---|---|
| 推測航法のみ | 2/12 | 16.3cm（車庫入れ） |
| スキャンアンカ | 7/12 | 6.1cm（車庫入れ） |

★**精度を支配するのは`ScanAnchor.max_pair`（対応点として認める距離）**。
`scan2scan.MAX_PAIR_M`の既定0.30は「1周期ぶんの相対移動」想定の値で駐車には
広すぎ、別の壁と組む誤対応を通す。0.30→0.08で推定誤差が3.65cm→0.96cmに
なった（掃引実測。同フィールドのdocstring参照）。現状の自己位置推定の誤差は
**0.8〜2.0cm**（`sim.park_bench --errors real`、48件）。

## 環境表現: 局所占有格子＋ESDF（`raspi/nav/local_map.py`）

瞬時のスキャンを点群のまま使うのをやめ、1cm格子に溜める。理由:

- **遮蔽と角度分解能で駐車枠の内側が見えていない。** 車庫入れ配置で目標
  周辺のクリアランスを測ると、**3割の地点で実際より3cm以上「余裕がある」と
  誤認**した（最大21cm）。しかも欠測を単に除外していたので、**見えていない
  場所へ楽観的に経路を引いていた**
- 一度見た壁を忘れない（後退中に前方が視野から外れても保持される）

**未知は「空き」と混ぜず、かつ「壁」とも区別する**——壁は硬い拘束、未知は
コスト。矩形の車体はESDFに対して`5×3`の円被覆で判定する（前後1.1cm・
左右1.8cmだけ安全側にはみ出す）。

## 経路計画: Hybrid A*（`raspi/nav/hybrid_astar.py`）

当初は「Reeds-Shepp候補を長さ順に見て、衝突しない最初のものを選ぶ」方式
だった。回避できるのは候補パターンの範囲だけで保証が無く、レイキャスト
点群での実測で**車庫入れ43%・縦列78%しか回避経路が見つからなかった**。
Hybrid A*（解像度完備）へ置き換えて**32/32**（旧方式は6/32）。

Reeds-Shepp（`raspi/nav/reeds_shepp.py`）は捨てずに、解析展開と
ヒューリスティックとして内部で使う。48語（9語族）へ完全化した。

時間予算（既定120ms）で打ち切る——`planning_node`の周期ループ内で同期的に
走るので、`cmd_deadman_ms`(150ms)を超えると指令が途絶えてDISARMに落ちる。
予算内に解が出なければRS候補選択へ落ちる（「動かない」より「少し進んで
別の姿勢から再計画」の方が次の機会で解ける）。

## 追従: 参照経路への閉ループ

以前はオープンループ（`curvature`から舵角を直接出し、進捗は`odom_center`の
累積で追う）だった。「Reeds-Sheppの区間は厳密な弧なので曲率をそのまま
指令すればよい」という理屈は通っているが、**誤差の回復手段が「区間を全部
走り終えてからの再計画」しか無い**。ノイズもバイアスもゼロのシムでも
位置2〜6cm・向き4〜6°が許容誤差に張り付き、8cmの残差のために4区間の
切り返しをもう一巡していた。

今は弧長`s`で見た誤差方程式 `d²e/ds² + k_yaw·de/ds + k_lat·e = 0` を
設計して曲率にフィードバックする（`k_lat`/`k_yaw`のdocstring参照）。
**後退時は向きの項の符号が反転する**（`v<0`で誤差ダイナミクスの符号が
変わるため。位置の項は前後で同符号のまま安定する）。

前後進の切り替えでは必ず停止し、**停止中に次の区間の舵を作っておく**
（切り替え直後に舵が追いつかず円弧の入口を違う曲率で走るのを防ぐ）。
`docs/system_overview.md`§2の「据え切りでステアMDが過熱する」制約は、
切り替えごとの単発動作なら連続据え切りとは切り分けられると判断した。

計画は`0.9 * max_steer`相当の旋回半径で行う（`steer_reserve`）——実舵角の
限界ぴったりで計画すると追従誤差を舵で詰める余地が無くなる。

## 安全: 掃引ベースの停止（`_safety_brake`）

旧`_obstacle_brake()`（進行方向±25°の窓の最近点と`rho`の比較）は撤去した。
旋回中の前後端の振り出しを見られず（経路が衝突する障害物の**16%**は衝突まで
窓に入らない）、さらに「目標と同距離の障害物を無視する」ゲートが縦列駐車で
最も当たりやすい隣の車を無視していた。

今は制動距離ぶんの車体掃引を、**今周期のスキャン（車体座標・自己位置推定を
通らない）**と**蓄積地図**の両方に当てる。塞がれていたら止めるだけでなく
**その場で経路を引き直す**（止まるだけだと地図が育って経路が無効になった
瞬間に固まる）。

**判定の閾値は計画が使った値（`_plan_margin`）と共有する。** 食い違うと
「計画が受け入れた経路を安全側が拒否する」ことになり、制動と再計画が
30秒以上交代し続ける（実測）。

## `auto_stop`はOFF運用が前提（既存gap系プランナーからの意図的な例外）

`auto_stop`（`docs/uart_protocol.md` §5.6.4/§5.8.3）は「進行方向への接近
そのものを止める」機構で、「目的地に意図的に接近して止まる」駐車動作とは
原理的に競合する。GUIは自動でOFFにはせず（他モードへ戻したときの戻し忘れ
リスクを避けるため）`AutoPanel.tsx`が警告バッジを出すに留める。

## 終了条件・限界

`rho < pos_tol_m`かつ`|向き誤差| < yaw_tol_deg`で完了。満たさなければ
その時点の姿勢から再計画する（`max_replans`回まで）。`max_maneuver_s`を
超えたら失敗として打ち切る。

`phase`（`AutoState`の汎用文字列フィールド）は日本語文字列を使う。
`gui/src/components/AutoPanel.tsx`の`st?.phase === 'DONE'`はモードで
gateされていないASCII直接比較（`slam2d_raceline`のenum値）だが、日本語
文字列を使う限り衝突しない。

disengage（GUI操作）すると`reset()`が呼ばれ目標は失われる——再engageする
には目標を再クリックすること（`follow_object`のROI選択と同じ挙動）。

**`plan()`は事実上LiDARの10Hzでしか呼ばれない**（`planning_node._replan()`
が`scan.seq`未変化ならスキップするため）。
"""

from __future__ import annotations

import math

import numpy as np

from ..core.vehicle import Vehicle
from ..msgs.types import AutoState, Scan, VehicleState
from ..nav.deskew import Points, deskew, integrate_pose
from ..nav.hybrid_astar import HybridConfig
from ..nav.hybrid_astar import plan as hybrid_plan
from ..nav.local_map import LocalMap
from ..nav.reeds_shepp import ReedsSheppPath, candidate_paths, sample_path_array
from ..nav.scan_anchor import ScanAnchor
from ..nav.se2 import Pose, between, wrap_angle
from .base import ParamSpec, Planner

__all__ = ["ParkToPoint"]

#: `plan()`が事実上LiDARの10Hzでしか呼ばれないことを踏まえた、検知〜反映の
#: 遅れ見込み[s]。STM32の`auto_stop`の`t_delay`に相当する自前の見積もり
PLAN_PERIOD_S = 0.1

#: 参照経路のサンプル間隔 [m]。横偏差の分解能を決める。0.3m/s・10Hzなら
#: 1周期で3cm進むので、それより細かくないと参照点が飛ぶ
REF_STEP = 0.01
#: 参照点の前方探索窓（サンプル数）。1周期の移動量（3cm＝3点）の10倍あれば、
#: 追従が乱れても追いつける
REF_SEARCH = 30
#: 安全判定の掃引の刻み [m]。車体（全長0.37m）より十分細かければよい
SWEEP_STEP = 0.03
#: GUIへ送る経路の間引き間隔 [m]。50Hzで流すので点数を抑える
PATH_VIEW_STEP = 0.05
#: 局所地図に、始点と目標の**両端から**持たせる余裕 [m]。経路は始点と目標を
#: 結ぶ線分の外へ大きく膨らむ——袋小路での方向転換は目標と反対側へ1m近く
#: 出るので、ここが足りないと外周（塞いである）に当たって計画が歪む
#: （2026-09-17、1.5mにしたとき袋小路の成功率が11/12→8/12に落ちた）
MAP_PAD_M = 2.0
#: 地図の最小の広さ [m]。目標が近くても切り返しの余地は要る
MAP_MIN_SIZE_M = 6.0


class ParkToPoint(Planner):
    id = "park_to_point"
    name = "駐車（クリック指定）"
    description = "LiDAR画面をクリック+ドラッグして指定した位置+向きへ、地図なしで接近して停止する"

    #: ギャップ探索系の判断根拠は書かない（`follow_object.py`と同じ理由）。
    #: 専用UI（`AutoPanel.tsx`）が`park_*`フィールドを読む
    stats = ()

    params = (
        ParamSpec(key="cruise_speed", label="巡航速度", min=0.05, max=1.0, step=0.05,
                  default=0.3, unit="m/s",
                  note="各セグメントの中間で使う速度。駐車動作は低速前提。"
                       "★io_nodeの--max-speedを超えてもPi側で切り捨てられるだけ"),
        ParamSpec(key="v_min_move", label="最低速度", min=0.03, max=0.3, step=0.01,
                  default=0.08, unit="m/s",
                  note="セグメント終端付近で減速する際、指令速度が小さすぎて"
                       "動けなくなる（モータの静止摩擦）のを防ぐ下限。実車で調整すること"),
        ParamSpec(key="seg_decel_dist_m", label="セグメント減速距離", min=0.05, max=0.60,
                  step=0.05, default=0.20, unit="m",
                  note="セグメント終端までこの距離に入ったら巡航速度から"
                       "v_min_moveまで減速する（次の切り返し停止に向けた滑らかな減速）"),
        ParamSpec(key="pos_tol_m", label="位置の許容誤差", min=0.02, max=0.30, step=0.01,
                  default=0.05, unit="m",
                  note="この距離まで近づいたら「到達」とみなす"),
        ParamSpec(key="yaw_tol_deg", label="向きの許容誤差", min=1.0, max=30.0, step=1.0,
                  default=5.0, unit="°",
                  note="ジャイロ積分のドリフトの影響を受けやすいため位置より緩め"),
        ParamSpec(key="max_maneuver_s", label="動作の制限時間", min=5.0, max=90.0, step=1.0,
                  default=40.0, unit="s",
                  note="★ジャイロ積分はドリフトする。短時間で完結する前提の設計なので、"
                       "超えたら「失敗」として打ち切る（無限に彷徨うのを防ぐ）"),
        ParamSpec(key="max_replans", label="経路再計算の上限", min=2.0, max=20.0, step=1.0,
                  default=8.0, unit="回",
                  note="セグメント完了・目標未到達のたびに経路を再計算する回数の上限。"
                       "★これを超えたら「失敗」として打ち切る（デッドレコニング誤差等で"
                       "永遠に再計画し続けるのを防ぐ）"),
        ParamSpec(key="collision_margin_m", label="経路の衝突マージン", min=0.0, max=0.30,
                  step=0.01, default=0.05, unit="m",
                  note="経路計画が**これ以上は通さない**壁までの余裕（硬い拘束）。"
                       "★駐車は意図的に壁へ詰める動作なので、目標自身の余裕が"
                       "これより小さければ planner が必要なぶんだけ緩める"
                       "（`hybrid_astar.plan()`の`goal_slack`）。緩めた値は"
                       "安全判定にも同じ閾値として渡る。"
                       "「できればこれだけ空けたい」方は clearance_target_m"),
        ParamSpec(key="k_lat", label="横偏差のゲイン", min=0.0, max=40.0, step=0.5,
                  default=12.0, unit="1/m²",
                  note="参照経路からの横ずれを曲率で戻す強さ。★0にすると"
                       "オープンループ追従（旧実装）に戻る。"
                       "**弧長`s`で見た誤差の方程式が "
                       "`d²e/ds² + k_yaw·de/ds + k_lat·e = 0` になる**ので、"
                       "収束の空間スケールは `1/√k_lat`。既定12は約0.29mで収束する"
                       "設定。★2.5では15cmの横偏差を戻せず実際に壁へ接触した（実測）"),
        ParamSpec(key="k_yaw", label="向き偏差のゲイン", min=0.0, max=20.0, step=0.5,
                  default=6.0, unit="1/m",
                  note="上の式のダンピング項。減衰比 ζ = k_yaw/(2√k_lat) なので、"
                       "既定(12, 6)で ζ≈0.87（わずかに過減衰＝行き過ぎない）。"
                       "★後退時は符号が反転する（コード内で処理）"),
        ParamSpec(key="steer_tau", label="舵の平滑化", min=0.0, max=0.3, step=0.01,
                  default=0.0, unit="s",
                  note="★他plannerと違い既定0（平滑化なし）。Reeds-Sheppの各区間は"
                       "正確な曲率の再現が前提で、Pi側で追加の遅れを足すと曲率がズレて"
                       "追従誤差が蓄積し、再計画が頻発することを実測で確認した。"
                       "実車の操舵むだ時間・1次遅れは既にハードウェア側にあるので"
                       "二重に足す理由がない"),
        ParamSpec(key="a_lat_max", label="旋回時の横加速度上限", min=0.5, max=6.0, step=0.1,
                  default=2.0, unit="m/s²",
                  note="★実車未計測の暫定値。follow_the_gap.pyと同じ曲率ベースの速度上限。"
                       "Reeds-Sheppのcurvatureは最小旋回半径ぴったりのことが多く、"
                       "低速では効かないことが多い保険"),
        ParamSpec(key="map_range_m", label="地図に取り込む距離", min=1.0, max=8.0, step=0.5,
                  default=5.0, unit="m",
                  note="★**これより遠い壁は planner に見えない。** GUIは生の"
                       "スキャンを描くので画面には映るが、経路計画も安全判定も"
                       "この距離で切った点群しか見ない。旧`obstacle_max_range`"
                       "（既定2.0m・安全窓用）を流用していたため、**2mより遠い壁を"
                       "貫通する経路が出た**（2026-09-17、バンビの画面報告で発覚）。"
                       "LD06の飽和は5.1mなのでそれを上限の目安にする"),
        ParamSpec(key="obstacle_margin_m", label="停止マージン", min=0.02, max=0.30, step=0.01,
                  default=0.10, unit="m",
                  note="★実車未計測の暫定値。制動距離に足す安全余裕（掃引判定の"
                       "先読み距離を決める）"),
        ParamSpec(key="a_brake_est", label="想定制動減速度", min=0.3, max=4.0, step=0.1,
                  default=1.0, unit="m/s²",
                  note="★実車未計測の暫定値。安全側の停止距離計算に使う"),
        ParamSpec(key="plan_budget_ms", label="経路探索の時間予算", min=20.0, max=400.0,
                  step=10.0, default=120.0, unit="ms",
                  note="★探索は`planning_node`の周期ループ内で同期的に走るので、"
                       "ここを`cmd_deadman_ms`(150ms)より大きくすると指令が途絶えて"
                       "DISARMに落ちる。実測では120msで96%・500msで99%なので、"
                       "伸ばす価値は小さい"),
        ParamSpec(key="steer_reserve", label="計画に使う舵角の割合", min=0.5, max=1.0,
                  step=0.05, default=0.9, unit="",
                  note="最小旋回半径を`wheelbase/tan(この割合*max_steer)`で決める。"
                       "★1.0（限界ぴったり）で計画すると追従誤差を舵で詰める余地が"
                       "無くなる。0.9なら舵角3°ぶんの補正余地が残る"),
        ParamSpec(key="clearance_target_m", label="望ましい壁との余裕", min=0.05, max=0.40,
                  step=0.01, default=0.15, unit="m",
                  note="これより壁に近い区間に罰則を掛ける（避けられるなら避ける）。"
                       "collision_margin_mが『これ以上は通さない』硬い下限で、"
                       "こちらは『できればこれだけ空けたい』柔らかい目標"),
        ParamSpec(key="reverse_cost", label="後退の割増", min=1.0, max=3.0, step=0.1,
                  default=1.6, unit="倍",
                  note="前進で済むなら前進する強さ。後退は視界も精度も悪いため"),
        ParamSpec(key="gear_change_cost", label="切り返しのコスト", min=0.0, max=2.0,
                  step=0.1, default=0.6, unit="m相当",
                  note="切り返し1回を経路長これだけぶんに換算して嫌う。"
                       "★大きくすると切り返しが減るが経路が大回りになる"),
        ParamSpec(key="anchor_enabled", label="スキャンアンカ", min=0.0, max=1.0, step=1.0,
                  default=1.0, unit="",
                  note="1=ON。クリック時のスキャンを基準に毎周期2フレーム登録し、"
                       "目標を実世界の壁へ固定する（`raspi/nav/scan_anchor.py`）。"
                       "★OFFにすると純デッドレコニングに戻り、ジャイロのドリフト"
                       "ぶんだけ駐車枠が実世界に対してずれる（誤差が『失敗』では"
                       "なく『誤った場所での完了』として現れる）。切り分け用に残してある"),
    )

    # ── ライフサイクル（生成・リセット・目標の受け取り） ──

    def __init__(self) -> None:
        self.vehicle = Vehicle.load()
        #: クリック時のスキャンを基準にした2フレーム登録。**大域地図は作らない**
        self._anchor = ScanAnchor()
        #: 駐車動作のあいだだけ持つ局所占有格子＋ESDF。**基準フレームに固定**
        self._map = LocalMap(footprint=self.vehicle.footprint)
        #: 車体外形の矩形 `(x0, x1, y0, y1)`。**厳密な掃引判定に使う**
        #: （円被覆の保守性1〜1.8cmを安全判定へ持ち込まないため）
        fp = self.vehicle.footprint or (
            (self.vehicle.front_overhang, self.vehicle.half_width),
            (-self.vehicle.rear_overhang, -self.vehicle.half_width),
        )
        self._body_box = (min(x for x, _ in fp), max(x for x, _ in fp),
                          min(y for _, y in fp), max(y for _, y in fp))
        self.reset()

    def reset(self) -> None:
        self._park_pose: Pose | None = None
        self._target_in_park: Pose | None = None
        self._prev_odom_center: float | None = None
        self._phase = ""
        self._maneuver_t = 0.0
        self._steer = 0.0
        self._failed_reason = ""
        self._path: ReedsSheppPath = ReedsSheppPath(segments=())
        self._replans = 0
        #: 走行区間の切り替え待ち（gearが変わる境界で実際に停止するのを待つ）
        self._waiting_stop = False
        self._set_path(ReedsSheppPath(segments=()), (0.0, 0.0, 0.0))
        self._anchor.reset()
        self._map.reset()
        #: アンカを使うか（`plan()`が毎周期パラメータから読み直す）
        self._anchor_enabled = True
        #: 直近のアンカ結果（診断・GUI表示用）
        self._anchor_ok = False
        self._anchor_inlier = 0.0
        self._anchor_correction = 0.0
        #: 直近の経路計画の結果（診断・GUI表示用）
        self._plan_how = ""
        self._plan_clearance = 0.0
        #: 直近の計画が使った硬い拘束の閾値 [m]。**安全判定も同じ値を使う**
        #: （食い違うと制動と再計画が交代して永久に進まない）
        self._plan_margin = 0.0

    def request_park_target(self, x: float, y: float, yaw: float) -> None:
        self._park_pose = (0.0, 0.0, 0.0)
        self._target_in_park = (x, y, yaw)
        self._prev_odom_center = None
        self._maneuver_t = 0.0
        self._failed_reason = ""
        self._phase = "接近中"
        self._replans = 0
        self._waiting_stop = False
        self._set_path(ReedsSheppPath(segments=()), (0.0, 0.0, 0.0))
        # 基準スキャンも張り直す。**目標と基準は必ず同じ瞬間のものにする**
        # ——違う瞬間のものを組み合わせると、その差だけ目標が実世界からずれる
        self._anchor.reset()
        # ── 地図は目標距離から張り直す ──
        # ★**目標が格子に収まっていなければならない。** 外周1セルは塞いで
        # あるので、目標が縁に乗ると「目標姿勢が障害物と重なっています」に
        # なり、Hybrid A*が即失敗してRS候補フォールバックへ落ちる
        # （2026-09-17、3m先の目標で実測）。始点と目標の中点を中心に取り、
        # 両端から`MAP_PAD_M`の余裕を持たせる
        rho = math.hypot(x, y)
        self._map.configure(size_m=max(MAP_MIN_SIZE_M, rho + 2.0 * MAP_PAD_M),
                            center=(x / 2.0, y / 2.0), resolution=0.01)

    # ── 経路計画（Hybrid A* が本筋、RS候補が縮退経路） ──

    def _turning_radius(self, p: dict[str, float]) -> float:
        """計画に使う最小旋回半径。**実舵角の限界ぴったりでは計画しない。**

        限界で計画すると追従誤差を舵で詰める余地が無くなる（`steer_reserve`）。
        """
        return self.vehicle.wheelbase / math.tan(
            max(0.05, p["steer_reserve"]) * self.vehicle.max_steer)

    def _replan(self, p: dict[str, float]) -> None:
        """今の推定姿勢から目標までの経路を、**障害物を避けつつ**作り直す。

        経路は`(gear, curvature, length)`の列で**座標系に依らない**ので、
        地図フレームで計画してそのまま車体基準の追従に使える。

        Hybrid A*（`raspi/nav/hybrid_astar.py`）が本筋で、解けなかったときだけ
        下の`_replan_fallback()`へ落ちる。実測（センサ誤差あり24件、再計画40回）
        では**77.5%が解析展開で即決、22.5%が予算切れ（時間120ms 6回・展開上限
        3回）でフォールバックへ**、そのうち3回が採用・6回は「食い込むので
        経路なし」だった。
        """
        cfg = HybridConfig(
            turning_radius=self._turning_radius(p),
            margin=p["collision_margin_m"],
            clearance_target=p["clearance_target_m"],
            reverse_cost=p["reverse_cost"],
            gear_change_cost=p["gear_change_cost"],
            time_budget_s=p["plan_budget_ms"] / 1000.0,
        )
        res = hybrid_plan(self._park_pose, self._target_in_park, self._map, cfg)
        self._plan_how = res.how
        self._plan_clearance = res.clearance
        self._plan_margin = res.margin
        if res.ok:
            self._set_path(res.path, self._park_pose)
        else:
            self._replan_fallback(p, cfg.goal_slack)

    def _replan_fallback(self, p: dict[str, float], slack: float) -> None:
        """縮退経路: Reeds-Shepp候補から**余裕がいちばん大きいもの**を選ぶ。

        Hybrid A*が時間予算内に解けなかったときだけ通る。「動かない」よりは
        「少し進んで別の姿勢から再計画する」方が次の機会で解ける
        ——探索は決定論的なので、同じ姿勢で再試行しても同じ失敗になる。

        ★**食い込む経路は採らない。** これが当初の実装（「全候補が衝突するなら
        最短をそのまま採用」）で、**壁を貫通する経路を指令していた**
        （2026-09-17、バンビの画面報告で発覚）。採用の下限はHybrid A*と同じ
        スラック——格子1cm＋円被覆1〜2cmの保守性ぶんは負に見えても許す。
        """
        pose_map = self._park_pose
        turning_radius = self._turning_radius(p)
        rel = between(self._park_pose, self._target_in_park)
        best: ReedsSheppPath | None = None
        best_clear = -math.inf
        for k in (1.0, 1.5, 2.5):
            for cand in candidate_paths((0.0, 0.0, 0.0), rel, turning_radius * k):
                if not cand.segments:
                    continue
                if self._map.ready:
                    poses = sample_path_array(pose_map, cand, step=0.04)
                    clear = self._map.path_clearance(poses)
                else:
                    clear = -cand.length          # 地図が無ければ最短を選ぶ
                if clear > best_clear:
                    best_clear, best = clear, cand
                if best_clear >= p["collision_margin_m"]:
                    break
            if best_clear >= p["collision_margin_m"]:
                break
        if best is not None and self._map.ready and best_clear <= -slack:
            self._plan_how = (self._plan_how
                              + f" → RS候補も食い込む（{best_clear * 100:.0f}cm）").strip()
            self._plan_clearance = best_clear
            self._plan_margin = 0.0
            self._set_path(ReedsSheppPath(segments=()), pose_map)
            return
        if best is not None:
            self._plan_how = (self._plan_how + " → RS候補で代替").strip()
            self._plan_clearance = best_clear if self._map.ready else 0.0
            # **この経路を受け入れた閾値**を安全判定へ渡す（`_safety_brake`）。
            # 代替経路は余裕が足りないこともあるが、それを安全側が知らないと
            # 「計画が受け入れた経路を安全側が拒否する」交代が起きる。
            # 実際の接触は`_sweep_clearance()`の厳密判定（閾値0）が止める
            self._plan_margin = min(p["collision_margin_m"], best_clear)
            self._set_path(best, pose_map)
        else:
            self._set_path(ReedsSheppPath(segments=()), pose_map)

    def _set_path(self, path: ReedsSheppPath, pose_map: Pose) -> None:
        """経路を**地図フレームの参照姿勢列**として展開して保持する。

        閉ループ追従は「今どこを走っているべきか」を知る必要があるので、
        区間列のままでは足りない。`REF_STEP`刻みでサンプルし、各点に
        `gear`・`curvature`・弧長を持たせる。`_run_end`は「同じgearが続く
        区間の最後の添字」で、前後進の切り替え位置（＝停止すべき点）を表す。
        """
        self._path = path
        self._waiting_stop = False
        self._ref_i = 0
        if not path.segments:
            self._ref = np.zeros((0, 3))
            self._ref_gear = np.zeros(0, dtype=np.int8)
            self._ref_kappa = np.zeros(0)
            self._ref_s = np.zeros(0)
            self._run_end = np.zeros(0, dtype=np.int64)
            return

        self._ref = sample_path_array(pose_map, path, step=REF_STEP)
        n = len(self._ref)
        gear = np.zeros(n, dtype=np.int8)
        kappa = np.zeros(n)
        s_arr = np.zeros(n)
        i = 1
        acc = 0.0
        gear[0] = path.segments[0].gear
        kappa[0] = path.segments[0].curvature
        for seg in path.segments:
            m = max(1, int(math.ceil(seg.length / REF_STEP)))
            ds = seg.length / m
            for _ in range(m):
                if i >= n:
                    break
                acc += ds
                gear[i] = seg.gear
                kappa[i] = seg.curvature
                s_arr[i] = acc
                i += 1
        self._ref_gear = gear
        self._ref_kappa = kappa
        self._ref_s = s_arr
        # 同じgearが続く区間の終端（その区間の最後の添字）を各点に配る
        run_end = np.zeros(n, dtype=np.int64)
        j = n - 1
        while j >= 0:
            k = j
            while k > 0 and gear[k - 1] == gear[j]:
                k -= 1
            run_end[k:j + 1] = j
            j = k - 1
        self._run_end = run_end

    # ── 参照経路の追従に使う小物 ──

    def _steer_for(self, curvature: float) -> float:
        """曲率 → 路面舵角（自転車モデル `κ = tan(δ)/L` の逆関数）。"""
        steer = math.atan(curvature * self.vehicle.wheelbase)
        return max(-self.vehicle.max_steer, min(self.vehicle.max_steer, steer))

    def _ref_index(self, pose_map: Pose) -> int | None:
        """今追うべき参照点の添字。**前へしか進まない**（後戻りさせない）。

        `odom_center`の累積で進捗を測るのではなく、**推定姿勢から最近傍点を
        探す**——累積は誤差が溜まる一方で、姿勢はアンカで毎周期直っているため。
        探索は現在の添字から前方の窓に限る（経路が自分自身に近づく配置で、
        遠くの点に飛び移らないように）。走行区間（同じgear）の終端は越えない。
        """
        n = len(self._ref_s)
        if n == 0:
            return None
        i0 = self._ref_i
        if i0 >= n - 1:
            return None
        hi = min(int(self._run_end[i0]), i0 + REF_SEARCH) + 1
        seg = self._ref[i0:hi]
        d2 = (seg[:, 0] - pose_map[0]) ** 2 + (seg[:, 1] - pose_map[1]) ** 2
        self._ref_i = i0 + int(np.argmin(d2))
        return self._ref_i

    # ── 安全（制動距離ぶんの車体掃引） ──

    def _safety_brake(self, pts: Points, pose_map: Pose, v_cmd: float,
                      ref: int, p: dict[str, float]) -> tuple[bool, str]:
        """**制動距離ぶん、いま指令している曲率で進んだときの車体掃引**を見て、
        当たると分かったら止める。

        ## 旧方式（進行方向±25°の窓の最近点）を捨てた理由

        旧`_obstacle_brake()`は「進行方向の窓の最近距離」と「目標までの残り
        距離」を比べ、明確に近い障害物だけを止めていた。2つ穴があった:

        1. **旋回中は前後端が横に振り出す**ので、窓の外の障害物に当たる。
           経路が衝突する1点障害物244件のうち、**衝突前に一度も窓に入らない
           ものが16%**（2026-09-16実測）
        2. `unexpected = nearest < rho - pad` のゲートが「目標と同距離の
           障害物」を無条件に無視する。縦列駐車で隣の車が目標と同距離に
           あるのは常態であり、**そこが最も当たりやすい**

        ## ★ 掃引は「参照経路の形を、いまの車体から描いて」行う

        3つの案を実測で比べた（2026-09-16、センサ誤差あり24件）:

        | 掃引の作り方 | 成功 | 何が起きるか |
        |---|---|---|
        | 参照経路の**絶対姿勢**をそのまま | 12/24 | **推定誤差が障害物に見える**。地図フレームの絶対姿勢と実車体基準の`pts`を突き合わせるので、推定が2cmずれれば2cmの食い込み。制動と再計画が30秒以上交代した |
        | **いまの指令舵角のまま**進む | 6/24 | 推定は通らないが「舵が変わらない」前提が悲観的すぎる。旋回中に壁を掠めると判定され、進めなくなる |
        | **参照経路の形を、いまの車体から**（採用） | — | 意図した曲率の変化を使い、かつ絶対姿勢を通らない |

        採用案は `between(参照点, 先の参照点)` で相対姿勢の列を作り、それを
        今の車体（原点）に置いて掃引する。参照経路から横にずれていること
        自体は横偏差フィードバックが直す仕事で、安全層の仕事ではない。

        ## 2つの情報源を併せて見る

        - **今周期のスキャン**（車体座標・推定を通らない）: 主。`min_hits`
          （2周=0.2s）を待たずに反応する
        - **蓄積地図**: 遮蔽で今は見えない壁も覚えている。こちらは地図
          フレームへ移すため推定誤差を含むので、**計画が受け入れた閾値
          （`_plan_margin`）と同じ緩さで判定する**
        """
        v = abs(v_cmd)
        d_stop = (v * PLAN_PERIOD_S + v * v / (2.0 * p["a_brake_est"])
                  + p["obstacle_margin_m"])
        local = self._sweep_poses(ref, d_stop)
        if len(local) == 0:
            return False, ""

        now = self._sweep_clearance(pts, local)
        if now < 0.0:
            return True, (f"進行方向{d_stop:.2f}m以内に障害物"
                          f"（食い込み{-now * 100:.0f}cm）")

        # 地図側は推定誤差を含むので、計画が受け入れた閾値で見る
        thr = min(0.0, self._plan_margin)
        c, s = math.cos(pose_map[2]), math.sin(pose_map[2])
        mx = pose_map[0] + local[:, 0] * c - local[:, 1] * s
        my = pose_map[1] + local[:, 0] * s + local[:, 1] * c
        myaw = pose_map[2] + local[:, 2]
        clear = float(self._map.body_clearance(mx, my, myaw).min())
        if clear < thr:
            return True, (f"進行方向{d_stop:.2f}m以内で地図上の障害物に"
                          f"接触します（食い込み{-clear * 100:.0f}cm）")
        return False, ""

    def _sweep_poses(self, ref: int, d_stop: float) -> np.ndarray:
        """制動距離ぶんの参照経路を、**いまの車体を原点として**並べ直す。

        `between(参照点, 先の参照点)`の列なので、絶対姿勢（＝自己位置推定）
        を通らない。走行区間（同じgear）の終端は越えない——その先は停止して
        から向きが変わるので、いま当たる話ではない。
        """
        end = int(self._run_end[ref])
        s0 = float(self._ref_s[ref])
        hi = int(np.searchsorted(self._ref_s[:end + 1], s0 + d_stop))
        hi = min(max(hi, ref + 1), end + 1)
        seg = self._ref[ref:hi]
        if len(seg) == 0:
            return seg
        rx, ry, ryaw = float(seg[0, 0]), float(seg[0, 1]), float(seg[0, 2])
        c, s = math.cos(-ryaw), math.sin(-ryaw)
        dx = seg[:, 0] - rx
        dy = seg[:, 1] - ry
        out = np.empty_like(seg)
        out[:, 0] = c * dx - s * dy
        out[:, 1] = s * dx + c * dy
        out[:, 2] = seg[:, 2] - ryaw
        return out

    def _sweep_clearance(self, pts: Points, local: np.ndarray) -> float:
        """**今周期のスキャンに対する掃引の最小余裕** [m]。負なら食い込み。

        `local`は車体座標の姿勢列、`pts`も車体座標なので、**自己位置推定を
        一切通らない**（`min_hits`の待ちも無い）。

        ★ **ここは円被覆ではなく厳密な矩形で測る。** 円被覆は矩形を安全側に
        1〜1.8cmはみ出して覆うので、閾値0で判定すると**その保守性がそのまま
        「食い込み1〜2cm」として報告される**。実測では目標の4cm手前まで
        寄ったところでこれが出続け、制動と再計画が20秒以上交代して失敗した。
        計画側（ESDF＋円被覆）が保守的で安全側が厳密、という非対称は正しい
        ——計画は余裕を持って引き、止めるかどうかは実際の形で決める。
        """
        hx = pts.x[pts.hit]
        hy = pts.y[pts.hit]
        if hx.size == 0 or len(local) == 0:
            return math.inf
        c = np.cos(-local[:, 2])[:, None]
        s = np.sin(-local[:, 2])[:, None]
        dx = hx[None, :] - local[:, 0][:, None]
        dy = hy[None, :] - local[:, 1][:, None]
        px = c * dx - s * dy                     # (N, P) 姿勢iの車体座標での点
        py = s * dx + c * dy
        x0, x1, y0, y1 = self._body_box
        # 矩形への符号付き距離（外は正、内は負）
        ox = np.maximum(np.maximum(x0 - px, 0.0), px - x1)
        oy = np.maximum(np.maximum(y0 - py, 0.0), py - y1)
        outside = np.hypot(ox, oy)
        inside_depth = np.minimum(np.minimum(px - x0, x1 - px),
                                  np.minimum(py - y0, y1 - py))
        d = np.where((ox <= 0.0) & (oy <= 0.0), -inside_depth, outside)
        return float(d.min())

    # ── `AutoState` の組み立て ──

    def _hold(self, st: AutoState, phase: str, reason: str) -> AutoState:
        """制動して待つ。`ready=True`なので指令は生き、速度0が流れる。"""
        st.ready = True
        st.brake = True
        st.phase = phase
        st.target_steer = self._steer
        st.reason = reason
        return st

    def _fail(self, st: AutoState, reason: str) -> AutoState:
        """失敗として打ち切る。**目標を再設定するまで復帰しない。**"""
        self._phase = "失敗"
        self._failed_reason = reason
        return self._hold(st, self._phase, reason)

    def _fill_state(self, st: AutoState, rel: Pose, rho: float) -> None:
        """GUIへ出す値をまとめて詰める（目標・経路・診断）。"""
        st.park_active = True
        st.park_target_x = rel[0]
        st.park_target_y = rel[1]
        st.park_target_yaw = rel[2]
        st.park_rho = rho
        st.park_plan_how = self._plan_how
        st.park_clearance = self._plan_clearance
        st.park_anchor_ok = self._anchor_ok
        st.park_anchor_inlier = self._anchor_inlier
        st.park_anchor_correction = self._anchor_correction
        self._fill_path(st)

    def _fill_path(self, st: AutoState) -> None:
        """計画した経路を**車両基準ローカル座標**へ直してGUIへ載せる。

        参照経路は`REF_STEP`（1cm）刻みで数百点あるので、`PATH_VIEW_STEP`
        ごとに間引く。**車両基準で渡す**のは、GUIが点群を描いている座標系と
        同じにするため（絶対座標で渡すとGUI側が推定姿勢を知る必要が出る）。
        """
        n = len(self._ref)
        if n == 0:
            return
        every = max(1, int(round(PATH_VIEW_STEP / REF_STEP)))
        idx = list(range(0, n, every))
        if idx[-1] != n - 1:
            idx.append(n - 1)
        pose = self._park_pose
        c, s = math.cos(-pose[2]), math.sin(-pose[2])
        dx = self._ref[idx, 0] - pose[0]
        dy = self._ref[idx, 1] - pose[1]
        st.park_path_x = (c * dx - s * dy).tolist()
        st.park_path_y = (s * dx + c * dy).tolist()
        rev = np.nonzero(self._ref_gear[idx] < 0)[0]
        st.park_path_reverse_from = int(rev[0]) if rev.size else -1

    # ── 1周期の流れ ──

    def plan(self, scan: Scan, vs: VehicleState | None,
             p: dict[str, float], dt: float) -> AutoState:
        st = AutoState(mode=self.id, planner=self.name)

        if self._target_in_park is None or self._park_pose is None:
            st.reason = ("目標が設定されていません。LiDAR画面をクリック+ドラッグして"
                         "駐車目標を置いてください")
            return st                          # ready=False ＝ 制動
        if vs is None:
            st.reason = "車両状態を受信していません"
            return st
        if self._phase == "失敗":
            return self._hold(st, self._phase, self._failed_reason)

        self._anchor_enabled = p["anchor_enabled"] >= 0.5
        # 点群は1周期に1回だけ作る（脱スキュー＋取付位置補正込み）
        pts = deskew(scan, vs.speed, vs.yaw_rate,
                     mount_x=self.vehicle.lidar_x, mount_y=self.vehicle.lidar_y,
                     max_range=p["map_range_m"])
        self._update_pose(pts, vs, dt)

        if not self._map.ready:
            # **壁が確定する前に走り出さない。** `min_hits`（既定2周=0.2s）に
            # 届くまで`wall_mask()`は空で地図が「全部空き」に見えるため、
            # ここで計画すると障害物を無視した経路が出る
            return self._hold(st, "地図を初期化中",
                              f"局所地図を作成中（{self._map.observations}/"
                              f"{self._map.min_hits}周）")
        if self._maneuver_t > p["max_maneuver_s"]:
            return self._fail(st, f"制限時間{p['max_maneuver_s']:.0f}sを超過しました。"
                                  "目標を再設定してください")

        rel = between(self._park_pose, self._target_in_park)
        rho = math.hypot(rel[0], rel[1])
        yaw_err_deg = math.degrees(abs(wrap_angle(rel[2])))
        self._fill_state(st, rel, rho)

        if rho < p["pos_tol_m"] and yaw_err_deg < p["yaw_tol_deg"]:
            self._phase = "完了"
            return self._hold(st, self._phase,
                              f"到達（残り{rho * 100:.1f}cm・向き誤差{yaw_err_deg:.1f}°）")

        if not self._path.segments:
            # 経路を使い切ったのにまだ許容誤差内に無い → 引き直す
            if self._replans >= int(p["max_replans"]):
                return self._fail(st, f"経路の再計算が{int(p['max_replans'])}回を"
                                      "超えました。目標を再設定してください")
            self._replans += 1
            self._replan(p)
            self._fill_state(st, rel, rho)     # 新しい経路をGUIへ反映
            if not self._path.segments:
                return self._fail(st, f"経路を生成できませんでした（{self._plan_how}）。"
                                      "目標を再設定してください")

        return self._follow(st, pts, vs, rho, yaw_err_deg, p, dt)

    def _update_pose(self, pts: Points, vs: VehicleState, dt: float) -> None:
        """推測航法で姿勢を進め、基準スキャンへの登録で引き直し、地図へ取り込む。

        `odom_center`は微分を経ない累積値なので「車速センサは低速で使い物に
        ならない」制約を受けない（モジュールdocstring参照）。
        """
        d_center = (0.0 if self._prev_odom_center is None
                    else vs.odom_center - self._prev_odom_center)
        self._prev_odom_center = vs.odom_center
        if dt > 0:
            self._maneuver_t += dt
            self._park_pose = integrate_pose(*self._park_pose,
                                             d_center, vs.yaw_rate * dt)

        if self._anchor_enabled:
            if not self._anchor.has_reference:
                # 目標を受け取った直後の最初の周を基準にする。**このとき車両は
                # 静止している**（ユーザーがクリックした直後）ので、基準
                # スキャンとしていちばん素性が良い
                self._anchor.set_reference(pts)
            else:
                # `ScanAnchor`は**原点キーフレーム（目標を定義した瞬間）の
                # フレーム**で姿勢を返すので、目標も経路も座標変換が要らない
                u = self._anchor.update(pts, self._park_pose)
                self._park_pose = u.pose
                self._anchor_ok = u.ok
                self._anchor_inlier = u.inlier
                self._anchor_correction = u.correction_m

        # **姿勢が確定した後に取り込む**（推測航法の値で彫ってからアンカで
        # 姿勢を直すと、地図が1周期ぶんずれて焼き付く）
        self._map.integrate(pts, self._park_pose)
        self._map.refresh()

    def _follow(self, st: AutoState, pts: Points, vs: VehicleState, rho: float,
                yaw_err_deg: float, p: dict[str, float], dt: float) -> AutoState:
        """参照経路への閉ループ追従。

        ★以前はオープンループだった（`curvature`から舵角を直接出し、進捗は
        `odom_center`の累積だけで追う）。「Reeds-Sheppの区間は厳密な弧なので
        曲率をそのまま指令すればよい」という理屈は通っているが、**誤差の
        回復手段が「区間を全部走り終えてからの再計画」しか無い**。実測では
        ノイズもバイアスもゼロのシムでも位置2〜6cm・向き4〜6°が許容誤差に
        張り付き、8cmの残差のために4区間の切り返しをもう一巡していた。
        """
        pose = self._park_pose
        ref = self._ref_index(pose)
        if ref is None:
            # 参照点を使い切った → 次の周期で許容誤差判定＆再計画へ回す
            self._set_path(ReedsSheppPath(segments=()), pose)
            st.ready = True
            st.target_speed = 0.0
            st.target_steer = self._steer
            st.phase = self._phase
            st.reason = "経路の区間を完了。位置を確認中"
            return st

        gear = int(self._ref_gear[ref])
        st.park_reverse = gear < 0
        self._phase = "後退中" if gear < 0 else "前進中"
        run_end = int(self._run_end[ref])
        remaining = float(self._ref_s[run_end] - self._ref_s[ref])

        if self._waiting_stop or ref >= run_end:
            return self._switch_stop(st, vs, run_end)

        # ── 経路フレームの誤差 ──
        rx, ry, ryaw = (float(self._ref[ref, 0]), float(self._ref[ref, 1]),
                        float(self._ref[ref, 2]))
        dx, dy = pose[0] - rx, pose[1] - ry
        e_lat = -math.sin(ryaw) * dx + math.cos(ryaw) * dy
        e_yaw = wrap_angle(pose[2] - ryaw)
        st.park_cross_track = e_lat
        st.park_heading_err = e_yaw

        # ── 曲率のフィードバック ──
        # 前進: κ = κ_ref − k_lat·e_lat − k_yaw·sin(e_yaw)
        # 後退: 向きの項だけ符号が反転する（`v<0`で誤差ダイナミクスの符号が
        # 変わるため。位置の項は前後で同じ符号のままで安定する）
        kappa = (float(self._ref_kappa[ref])
                 - p["k_lat"] * e_lat
                 - (1.0 if gear > 0 else -1.0) * p["k_yaw"] * math.sin(e_yaw))
        steer = self._steer_for(kappa)
        tau = p["steer_tau"]
        a = 1.0 if tau <= 1e-3 or dt <= 0 else 1.0 - math.exp(-dt / tau)
        self._steer += (steer - self._steer) * a

        # ── 区間終端手前で減速。速度は残り距離に比例、下限はv_min_move ──
        speed_mag = max(p["v_min_move"],
                        min(p["cruise_speed"],
                            p["cruise_speed"] * remaining
                            / max(p["seg_decel_dist_m"], 1e-6)))
        v_cmd = math.copysign(speed_mag, gear)

        # ── 曲率ベースの速度上限（follow_the_gap.pyと同じ式） ──
        k_abs = abs(math.tan(self._steer) / self.vehicle.wheelbase)
        if k_abs > 1e-6:
            v_cmd = math.copysign(min(abs(v_cmd),
                                      math.sqrt(p["a_lat_max"] / k_abs)), v_cmd)

        brake, reason = self._safety_brake(pts, pose, v_cmd, ref, p)
        if brake:
            # ★**塞がれたら止まるだけでなく、その場で経路を引き直す。**
            # 以前は再計画が「経路を全部使い切ったとき」だけで、追従中に
            # 塞がれると永久に停止したままになった（車庫入れで「食い込み1cm」を
            # 報告し続けて制限時間まで固まる挙動を実測）。地図は毎周期育つので、
            # 計画時に空きだった場所が壁に変わることは普通に起きる
            if self._replans < int(p["max_replans"]):
                self._replans += 1
                self._replan(p)
                self._fill_path(st)
            return self._hold(st, self._phase, reason)

        st.ready = True
        st.target_speed = v_cmd
        st.target_steer = self._steer
        st.phase = self._phase
        st.reason = (f"{'後退' if gear < 0 else '前進'}で経路追従中"
                     f"（区間残り{remaining:.2f}m）。"
                     f"目標まで{rho:.2f}m・向き誤差{yaw_err_deg:.0f}°"
                     f"・横偏差{e_lat * 100:+.0f}cm")
        return st

    def _switch_stop(self, st: AutoState, vs: VehicleState, run_end: int) -> AutoState:
        """走行区間の切り替え（前後進の反転）で停止し、次の舵を準備する。

        **停止中に次の区間の舵を作っておく。** 切り替え直後に舵が追いつかない
        と、円弧の入口数cmを違う曲率で走って誤差になる（`docs/system_overview.md`
        §2の据え切り過熱は「切り替えごとに一度」なら連続据え切りとは別物として
        切り分けられる）。
        """
        self._waiting_stop = True
        nxt = run_end + 1
        if nxt < len(self._ref_s):
            self._steer = self._steer_for(float(self._ref_kappa[nxt]))
        if vs.stopped:
            self._waiting_stop = False
            self._ref_i = min(nxt, len(self._ref_s) - 1)
        return self._hold(st, "切替中", "前後進の切り替えのため停止中（次の舵を準備）")
