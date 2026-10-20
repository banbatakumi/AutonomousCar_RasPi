"""自動運転アルゴリズムの共通の型（`docs/architecture.md` §8）。

**1つの planner = 1つの走らせ方。** Follow the Gap・壁沿い走行・経路追従などを
同じ形にはめておき、GUI からどれで走るかを選べるようにする。

## planner が守る約束は4つだけ

1. **`plan()` は状態を持ってよいが、`reset()` で完全に初期化できること。**
   モードを切り替えたときに前のモードの舵の平滑化が残っていると、
   engage した瞬間に前回の続きから舵が動き出す
2. **戻り値は `AutoState`。** 指令（`target_speed` / `target_steer` / `brake`）と
   **その理由（`reason`）を必ず同時に返す。** 「なぜ止まったか」が後から
   分からないのがデバッグを最も消耗させる
3. **走ってよいか分からないときは `ready=False`。** 迷ったら止まる。
   `ready=False` の `AutoState` は planning_node が制動指令に読み替える
4. **パラメータは `params` に宣言する。** GUI のスライダはこの宣言から
   自動生成されるので、**planner を足しても GUI は 1 行も変えなくてよい**

## 単位は SI のまま

`Scan.dist` は [m]、角度は [rad]（車両座標・反時計回りが正）。度に直すのは
`AutoState` の `*_deg`（GUI の表示とギャップの可読性のため）だけにしてある。
"""

from __future__ import annotations

import math
from typing import NamedTuple

import msgspec

from ..msgs.types import TOPIC_SCAN, AutoMap, AutoState, Scan, VehicleState

__all__ = ["CMD_HZ", "ParamSpec", "Planner", "ScanWindow", "StandstillSteer", "blind_reason", "min_filter", "scan_window",
           "sector_of_deg", "wrap_deg"]

#: planner の指令（`auto/cmd`）の周期 [Hz]。**ここが唯一の定義**で、planning_node が発行し、
#: telemetry_node が STM32 への COMMAND をこの周期で中継する（ファームの `drive.h` の
#: DRIVE_HOST_COMMAND_PERIOD_S もこの逆数）。planner は指令の遅れの見積もりに使う
CMD_HZ = 50

#: 受信できた点のうち「距離が分かった」（実測または飽和）点の割合の下限。視野全体と、
#: 正面 ±`BLIND_FRONT_DEG` の扇のそれぞれで見る（`blind_reason`）。**パラメータにしていない**
#: （`MIN_SEEN_RATIO` と同じ理由。「見えていないのに走る」を人がスライダで許可できる場所を作らない）
MIN_KNOWN_RATIO = 0.3
MIN_FRONT_KNOWN_RATIO = 0.5
BLIND_FRONT_DEG = 30


class ParamSpec(msgspec.Struct):
    """調整できるパラメータ1つ。**GUI のスライダはこれ 1 件から作られる。**

    `min`/`max` は「安全に振ってよい範囲」であって好みの範囲ではない。
    ここを広く取ると、スライダを端まで動かしただけで車が壁に突っ込む。
    """

    key: str
    label: str                             #: GUI に出す名前（日本語）
    min: float
    max: float
    step: float
    default: float
    unit: str = ""
    #: なぜこの値なのか・上げ下げすると何が起きるか。GUI にそのまま出る
    note: str = ""
    #: GUI で見出しを付けてまとめる組（例: 「地図作成」「本番走行」）。空なら組分けしない
    group: str = ""
    #: True なら GUI に出さず、保存済みの設定・GUI から来た値も読まない（＝いつも `default`）。
    #: 人が決める必要のない値（同定値や走行中の学習から決まるはずの値・一度決めたら触らない値）を、
    #: コードから消す前の段階として定数にしておくための印。`sim.bench --set` だけは上書きできる
    hidden: bool = False
    #: 値ごとの名前（`min` から `step` 刻みの順）。あれば GUI はスライダではなく**選択肢**として出す
    #: （方式の切り替えなど、数値の大小に意味が無い設定）
    choices: tuple[str, ...] = ()


class Planner:
    """自動運転アルゴリズムの基底。**サブクラスは4つの属性と2つのメソッドだけ。**

    登録は `raspi/auto/registry.py` の `PLANNERS` に1行足すだけでよい。
    """

    #: バス・GUI・設定ファイルを貫く識別子。**変えると保存済みの設定が外れる**
    id: str = ""
    name: str = ""                         #: GUI のボタンに出る名前
    description: str = ""                  #: 1行の説明。GUI の選択肢の下に出る
    #: GUI 側の一覧をどこに出すかの分類。既定 `"drive"` は自動運転タブの
    #: モード選択に出る。`"sysid"` はシステム同定タブ専用（`raspi/auto/sysid_*.py`）
    #: で、自動運転タブの選択肢からは除外される（`AutoPanel.tsx` 側のフィルタ）。
    #: **planning_node・engage・ARM・deadman の仕組みはどちらも完全に共通**——
    #: 分類は GUI の見せ方だけの都合で、エンジン側は関知しない
    category: str = "drive"
    params: tuple[ParamSpec, ...] = ()
    #: `plan()` の第一引数に何を渡すか。**既定は実 LiDAR の `scan`。**
    #: カメラ由来の擬似スキャンで走る planner（`follow_the_gap_cam.py` 等）だけが
    #: ここを上書きする。既存 planner はこの属性の存在を意識しなくてよい
    #: （`planning_node.py` がここを見て購読トピックを組み立てる）
    input_topic: str = TOPIC_SCAN
    #: `input_topic` の鮮度上限 [ms]。これより古ければ制動に読み替える
    #: （`planning_node.py` の `_current()`）。**LiDAR の 10Hz を前提にした値が
    #: 既定なので、ケイデンスが違うセンサの planner は上書きすること**
    stale_ms: int = 300
    #: GUI の「判断」欄（`auto-stats`）にどの `AutoState` フィールドを出すか。
    #: `"free_ahead"`/`"nearest"`/`"gap"`/`"valid_ratio"` の部分集合。
    #: **既定はギャップ探索系（FTG/DE/gap_pursuit/raceline）が使う全部**——
    #: `plan()` で書かないフィールドは `AutoState` の既定値（0.0）のまま残り、
    #: GUI のモード選択に出すか。False でも planner としては作れる（`sim.bench`・派生クラスの土台）
    listed: bool = True
    #: 意味の無い数値（例: gapを持たない planner で「0〜0° ギャップ」）が
    #: GUI に出てしまうため、書くフィールドだけをここで宣言する
    #: （`LineTrace`/`E2ELidar` が上書きしている）。`target_speed`/
    #: `target_steer`/`plan_hz` は全 planner が必ず書くので対象外
    stats: tuple[str, ...] = ("free_ahead", "nearest", "gap", "valid_ratio")

    def reset(self) -> None:
        """内部状態を捨てる。**モード切替・disengage のたびに呼ばれる。**

        ただし `on_disengage()` を持つ planner では、**解除だけのとき**（モードは同じ）は
        `reset()` の代わりにそちらが呼ばれる（`planning_node._apply_ctrl`）。何を残して何を
        捨てるかは planner が決める: 解除のたびに地図・自己位置を作り直したくない planner は
        `on_disengage()` を実装し、捨てる状態だけをそこで捨てる。モード切替では必ず `reset()`。
        `on_disengage()` が何もしなければ、解除で状態は全て残る（次の engage は続きから動く）ので、
        舵の平滑化など「前回の走行の続き」になる状態は自分で捨てること。

        ほかにダックタイピングで呼ばれる任意のメソッド（持っていれば `planning_node` が呼ぶ）:
        `on_vehicle_state(vs)`（全サンプル）、`set_engaged(engaged)`（`plan()` の直前）、
        `on_disengage()`（上記）、`close()`（モード切替で捨てるとき）。
        """

    def plan(self, scan: Scan, vs: VehicleState | None,
             p: dict[str, float], dt: float) -> AutoState:
        """1周期ぶんの判断。

        :param scan: 最新の点群。**`sector_seen` が False の区間は「障害物なし」ではない**
        :param vs: 最新の車両状態。まだ届いていなければ None
        :param p: `params` の既定値に GUI の設定を重ねたもの。**全キーが必ず入っている**
        :param dt: 前回の `plan()` からの経過 [s]。平滑化に使う
        """
        raise NotImplementedError

    def snapshot(self) -> AutoMap | None:
        """表示専用の重い状態（地図・経路）。**既定は None。**

        **これは上の「4つの約束」に入らない。** 判断ではなく表示だからで、
        指令には一切影響しない。地図は 400×400 = 160KB あって `AutoState`
        （10Hz）には載せられないので、別トピック `auto/map` へ逃がすための口。

        planning_node は `map_seq` が変わったときだけ publish する。
        **変わっていないものを返し続けてよい**（毎周期呼ばれる）。
        """
        return None

    def request_freeze(self) -> None:
        """人間が「地図を確定」と言ってきた。**自動判定の逃げ道。**

        地図を作る段から走る段への遷移は不可逆なので、planner の自動判定が
        滑ったときに人間が押せる経路を残しておく。地図を持たない planner は
        何もしなくてよい。
        """

    def request_clear(self) -> None:
        """人間が「地図を削除」と言ってきた。**作り直しの口。**

        `reset()` を呼ぶのと同じだが、**engage を落とさずに**地図だけ捨てられる
        経路として分けてある（disengage → engage し直すと ARM の保持まで巻き込む）。
        """

    def request_load(self, name: str) -> None:
        """人間が保存済み地図を選んで「レーシングライン走行」と言ってきた。

        保存済み地図（占有格子・中心線・レーシングライン）を読み込んで、
        地図作成（EXPLORE）を経ずに走り始める口。対応する planner だけ実装する
        （既定は何もしない＝保存済み地図を持たない planner には無関係）。
        """

    def request_clear_grip(self, name: str) -> None:
        """人間が保存済み地図の「学習値を消す」と言ってきた。

        地図の横に保存した学習値（`<name>.grip.json`）だけを消す。地図・経路・
        経由点は残す。対応する planner だけ実装する（既定は何もしない）。
        """

    def request_locate_hint(self, x: float, y: float) -> None:
        """GUIの地図パネルをクリックして、自己位置探索の絞り込みヒントを送ってきた。

        地図全体からの自己位置推定（グローバルローカリゼーション）に対応する
        planner だけ実装する。既定は何もしない。
        """

    def request_park_target(self, x: float, y: float, yaw: float) -> None:
        """GUIのLiDAR画面をクリック+ドラッグして、駐車目標（位置+向き、
        クリック時点の車両基準ローカル座標）を送ってきた。

        対応する planner（`park_to_point.py`）だけ実装する。既定は何もしない。
        """

    # ── ヘルパ ──

    @classmethod
    def defaults(cls) -> dict[str, float]:
        return {s.key: s.default for s in cls.params}

    @classmethod
    def merged(cls, user: dict[str, float], hidden: bool = False) -> dict[str, float]:
        """既定値にユーザー設定を重ね、**宣言した範囲へクランプする**。

        GUI を信用してクランプを省くと、古い `config/auto.json` や手書きの値が
        そのまま planner に入る。範囲外の値で走り出す方が、無視されるより危ない。

        :param hidden: True なら `ParamSpec.hidden` の項目もユーザー設定で上書きする（`sim.bench` 用）。
            既定は上書きしない——GUI から隠した項目に、古い保存値が黙って効き続けないように
        """
        out = cls.defaults()
        for s in cls.params:
            v = user.get(s.key)
            if v is None or (s.hidden and not hidden):
                continue
            try:
                v = float(v)
            except (TypeError, ValueError):
                continue
            if v != v:                     # NaN
                continue
            out[s.key] = min(s.max, max(s.min, v))
        return out


class StandstillSteer:
    """停車中に舵を切ったまま保持し続けない（据え切りでステア MD が過熱する。
    `docs/system_overview.md` §2・`docs/stm32_interface.md` §8.4）。

    **停車を確認して `HOLD_S` 経ったら舵を中立へ戻す。** 走行中・切り返しの途中（呼び出し側が
    `allow=False` を渡す）では戻さない。止まっている間だけ数え、動き出したら数え直す。
    """

    #: 停車してから舵を戻すまでの時間 [s]。切り返しの停止（数百ms）や制動の停止直後は保持し、
    #: 長い停車だけ戻す。MD ステアの過熱は分単位の据え切りで起きるので、2s で十分間に合う
    HOLD_S = 2.0
    #: 停車とみなす車速 [m/s]
    STILL_SPEED_M_S = 0.05

    def __init__(self) -> None:
        self._still_s = 0.0

    def reset(self) -> None:
        self._still_s = 0.0

    def observe(self, vs: VehicleState | None, dt: float) -> None:
        """毎周期1回、`plan()` の頭で。`vs` が無い・動いているなら数え直す。"""
        # `stopped` 印だけでなく車速そのものも見る（印を作らない経路——同定ベンチなど——で、走行中も
        # 「停車」と数えて減速中に舵を戻し、直進してはみ出した）
        if vs is not None and vs.stopped and abs(vs.speed) < self.STILL_SPEED_M_S:
            self._still_s += max(0.0, dt)
        else:
            self._still_s = 0.0

    @property
    def relaxed(self) -> bool:
        return self._still_s >= self.HOLD_S

    def steer(self, steer: float, allow: bool = True) -> float:
        """`steer` を出してよいか。停車が続いていて `allow` なら 0（中立）を返す。"""
        return 0.0 if allow and self.relaxed else steer


def wrap_deg(deg: float) -> float:
    """0〜359 の車両角を **±180 の符号付き**に直す。

    前方を跨ぐギャップ（例 350°〜10°）を `start <= end` で表せるようにするため。
    0〜359 のままだと「右端 350、左端 10」となり、大小関係が反転して読めない。
    """
    d = (deg + 180.0) % 360.0 - 180.0
    return d


class ScanWindow(NamedTuple):
    """前処理済みの前方視野。**`scan_window()` が返すもの。**

    `dist[j]` は `degs[j]` 方向の「進んでよい距離」で、**0.0 は
    「そこへは進んではいけない」の意味に統一**してある。「見えていない」と
    「近すぎる」を別扱いにすると、以降の分岐が倍に増える。
    """

    degs: list[int]                        #: 視野の角度 [deg]（−half 〜 +half）
    dist: list[float]                      #: 進んでよい距離 [m]。0.0 は侵入禁止
    measured: list[bool]                   #: 実測点か（最近傍の判定に使う）
    seen_ratio: float                      #: セクタが見えていた割合 [0..1]
    valid_ratio: float                     #: 実測点だった割合 [0..1]
    #: 受信できた点のうち距離が分かった（実測または飽和）点の割合 [0..1]。視野全体と正面の扇。
    #: 受信できた点が無ければ 1.0（欠測の多さは `seen_ratio` が見る）。`blind_reason` が使う
    known_ratio: float = 1.0
    front_known_ratio: float = 1.0


def scan_window(scan: Scan, fov_deg: float, max_range: float, *,
                center_deg: float = 0.0) -> ScanWindow:
    """前方 ±`fov_deg`/2 を切り出し、**欠測・飽和・測距不能を安全側に倒す**。

    `center_deg`（既定 0.0＝前方）で切り出す向きをずらせる。既存の呼び出し元は
    全て既定のままなので挙動は変わらない。**後退の安全窓を切り出す
    `park_to_point.py` だけが `center_deg=180.0` を渡す**——前後どちらの窓も
    同じ「欠測=壁・飽和=空き・測距不能=空き」の契約で読めるようにするため
    （実装を1箇所に保つ。下記★の注意はここにも当てはまる）。

    | 入力 | どう読むか | なぜ |
    |---|---|---|
    | `sector_seen[s] == False` | **壁**（距離 0 ＝ 侵入禁止） | 受信できていないだけで、そこが空いている保証はまったく無い |
    | `saturated[i]`（5.10m 以上） | `max_range`（空き） | 「5.10m ちょうど」ではなく「以上」。壁として打つと実在しない円い壁ができる |
    | `dist[i] == 0`・NaN・Inf（測距不能） | `max_range`（空き） | ★下記。**唯一、危険側に倒している判断** |

    `dist == 0` は LD06 が「反射が返らなかった」と言っているだけで、
    **射程外に何も無い**場合と**黒い/斜めの面が至近にある**場合の両方がありうる。
    壁として扱うと、開けた場所で進める方向が1つも見つからず一歩も動けない
    （実用にならない）ので、空きとして扱っている。**この判断の穴は
    STM32 の `auto_stop`（★v0.12・動的停止距離 `d_stop`、LiDAR 主・超音波補助）
    で受ける**という前提で成り立っている。planner 側は同種の固定距離しきい値を
    重ねて持たない（STM32側の方が速度に応じて停止距離が伸びる分高性能なため。
    `docs/uart_protocol.md` §5.6.4）。

    ★ ただし**測距不能が大半を占める点群では走らない**（`blind_reason`）。LiDAR の窓が
    塞がれた・反射が返らないと全点が「空き」に化け、`auto_stop` も同じ点群を見ているので
    働かない。空き扱いにしてよいのは「大半は測れていて、一部だけ返らなかった」ときだけ。

    ★ **planner ごとに書き写さないこと。** ここは「点群をどう安全に読むか」
    という契約そのもので、片方だけ直すと**もう片方が古い読み方のまま走る**。
    """
    half = int(round(fov_deg / 2))
    degs = list(range(-half, half + 1))
    offset = int(round(center_deg))
    dist: list[float] = []
    measured: list[bool] = []
    seen = 0
    known = 0
    front_seen = 0
    front_known = 0
    for d in degs:
        i = (d + offset) % 360
        if not scan.sector_seen[sector_of_deg(i)]:
            dist.append(0.0)               # 欠測は壁。**空きではない**
            measured.append(False)
            continue
        seen += 1
        front = abs(d) <= BLIND_FRONT_DEG
        front_seen += front
        if scan.saturated is not None and scan.saturated[i]:
            dist.append(max_range)         # 「5.10m 以上」。実測点として扱わない
            measured.append(False)
            known += 1                     # 「そこまでは何も無い」と分かっている
            front_known += front
            continue
        raw = scan.dist[i]
        # NaN は比較が常に False なので前半で落ちる。Inf は「測れた距離」ではないので同じ扱い
        if not (raw > 0.0) or raw == math.inf:
            dist.append(max_range)         # 測距不能。★上表の「唯一の危険側」
            measured.append(False)
        else:
            dist.append(min(raw, max_range))
            measured.append(True)
            known += 1
            front_known += front

    n = len(degs)
    return ScanWindow(degs, dist, measured,
                      seen / n if n else 0.0,
                      sum(measured) / n if n else 0.0,
                      known / seen if seen else 1.0,
                      front_known / front_seen if front_seen else 1.0)


def blind_reason(w: ScanWindow) -> str:
    """測距不能な点が多すぎて走ってはいけないなら、その理由。走ってよければ ""。

    `scan_window` は測距不能（`dist == 0`・NaN・Inf）を「空き」と読む。全点がそうなった点群
    （LiDAR の窓が塞がれた・反射が返らない）では全方位が `max_range` の空きになり、何も
    見えていないのに最高速で直進する。受信できた点のうち距離が分かった点の割合が、視野全体
    （`MIN_KNOWN_RATIO`）か正面の扇（`MIN_FRONT_KNOWN_RATIO`）で下限を割ったら走らない。
    欠測セクタは数えない（`seen_ratio` と各 planner の `MIN_SEEN_RATIO` が見る）。
    """
    if w.front_known_ratio < MIN_FRONT_KNOWN_RATIO:
        return (f"正面の点群が測距できていない（正面 ±{BLIND_FRONT_DEG}° の "
                f"{w.front_known_ratio * 100:.0f}% しか距離が分からない）")
    if w.known_ratio < MIN_KNOWN_RATIO:
        return f"点群が測距できていない（視野の {w.known_ratio * 100:.0f}% しか距離が分からない）"
    return ""


def min_filter(r: list[float], half_width: int) -> list[float]:
    """±`half_width` の最小値を取る。**障害物を太らせる方向にしか間違えない。**

    移動平均を掛けると、椅子の脚やパイロンのような**幅の細い障害物が周囲の
    遠い距離に薄められて消える**。最小値なら障害物は決して消えない。

    端は配列を伸ばさずに切り詰める（視野の外を仮定しない）。
    """
    if half_width <= 0:
        return r
    n = len(r)
    return [min(r[max(0, j - half_width):min(n, j + half_width + 1)]) for j in range(n)]


def extend_disparity(r: list[float], disparity: float, half_width: float) -> list[float]:
    """段差の**遠い側**を近い側の値で塗る（Disparity Extender の中核）。

    塗るのは `min()`。**上書きにすると処理の順番で結果が変わる。**

    ★ `disparity_extender.py` と `gap_pursuit.py` の両方が使う共通実装。
    片方だけに書き写すと、`scan_window`/`min_filter` と同じ理由で
    もう片方が古い実装のまま走ることになる。
    """
    out = list(r)
    n = len(r)
    for i in range(n - 1):
        lo, hi = r[i], r[i + 1]
        if abs(hi - lo) < disparity:
            continue
        near = min(lo, hi)
        # 距離 `near` の所で半幅ぶんを見込む角度［度］。至近では 90° で頭打ち。
        # ★ 近い側が 0（欠測セクタ＝侵入禁止）のときは、その縁が**遠い側と同じ距離にある**として
        #   見込む。距離 0 のまま計算すると 90° になり、欠測 30° の両隣 90° ずつ（＝視野のほぼ
        #   全部）が塞がる——1周だけセクタが欠けただけで「どこにも進めない」になっていた
        edge = near if near > 0.0 else max(lo, hi)
        span = min(90.0, math.degrees(math.atan2(half_width, max(edge, 1e-3))))
        k = int(math.ceil(span))           # 点は 1° 刻み
        if hi > lo:
            # 遠いのは右→左方向（添字が増える側）。i+1 から先を塗る
            for j in range(i + 1, min(n, i + 1 + k)):
                out[j] = min(out[j], near)
        else:
            for j in range(max(0, i - k + 1), i + 1):
                out[j] = min(out[j], near)
    return out


def sector_of_deg(deg: int) -> int:
    """車両角 [deg] → `Scan.sector_seen` の添字。

    **境界が 1 つずれている。** セクタ `s` が持つのは `dist[30*s+1]` 〜
    `dist[(30*s+30) % 360]` で、`30*s` ちょうどの点は隣のセクタ由来
    （`Scan` の docstring）。素直に `deg // 30` と書くと、セクタ欠損の判定が
    30 点ぶん隣にずれ、**欠測している方向を「見えている」と誤判定する**。
    """
    return ((deg - 1) % 360) // 30
