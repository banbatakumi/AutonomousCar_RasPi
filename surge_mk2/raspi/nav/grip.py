"""走行中の学習 — 速度プロファイルが使う横・加速・減速の上限を、走りながらコースに合わせる
（`slam2d_raceline` / `slam2d_route`）。

車に固有の値はシステム同定（校正）、**路面やコースで変わる値は走行中に決める**。`config/vehicle.toml` の
校正値は出発点として読むだけで、書き戻さない。校正の床とコースの床は材質が違うので、値が合わないのは前提。

学習するのは 2 つだけ:

- **横の上限**（コースで 1 つ。`GripLearner`）: **周回のタイムで決める**。上限を 1 段上げて 1 周走り、
  速度プロファイルが縮むはずの時間の `time_gain` 以上縮んだら採用、縮まなければやめる
- **前後の上限の包絡**（`LongLearner`）: 横の使い具合ごとに、計画が求めた加減速に車が付いて行けたかで決める

## 横の上限 — タイムで決める（`GripLearner`）

目的は速く走ることで、「前兆が出ない範囲でいちばん高い横加速度」はその代用品にすぎない。実機では
前兆の検出の誤り（誤検出で 4.41m/s² に止まる／検出の穴から 8.0m/s² まで上がってリアが滑る）の両方が起きた。
**上げても縮まなくなった所が、求めている値**。

- 上限を 1 段（`time_step`）上げて 1 周走り、「実際に縮んだ時間 ÷ 計画が縮むはずの時間」が `time_gain` 以上なら
  採用して次へ。届かなければ戻して幅を半分にし、幅が `time_step_min` を切ったら終わり（収束）。終える前に一度だけ、
  採用済みの段を測り直してから元の幅で試し直す（採用済みの段のタイムがたまたま速いと、続けて断って手前で終わる）
- 「縮むはずの時間」は、今の前後の包絡のまま横の上限だけを採用済みの段へ戻した計画との差（planner が出す）。
  前後の学習で縮んだぶんを、横を上げた効果と取り違えない
- タイムは、走り出した場所に置いたゲート線をまたいだ時刻（100Hz で補間）。回避・線の乗り換え・推測航法・
  安全装置で減速した周は捨てる

## 安全装置 — 結果に直結する 3 つだけ

行き過ぎた 1 段を、タイムの判断を待たずにやめるためのもの（上限を探すためではない）。

1. **リアの滑り**: 舵が求める曲率と実曲率が逆向き（逆ハン）のまま、横加速度が `slide_alat` 以上の状態が
   `slide_s` たまった。または、ヨーレート×車速 が IMU の横加速度を `slide_excess` 以上上回った（横滑りが増えている）
   状態が `slide_excess_s` 続いた（2026-10-29 夜。制動しながらのリアの巻き込みは逆ハンが短く、逆ハンだけでは 20 回中 3 回しか
   捕まえられなかった）。目標速度をすぐ落とす
2. **線から外れた**: 経路からの横偏差が `cross_thr` を超えた（横を `cross_use` 以上使っている所）。すぐ落とす
3. **横偏差の増え**（周の終わりに見る）: コーナーの中の横偏差が、その場所の普段（採用済みの走りで測った値）を
   `cross_gain` 以上上回った。外へ膨らむ形は、壁に近づく前にここで捕まえる。探している間だけ

働いたら**最後に上げたものを疑う**:

- 横の段を試している最中 → その段をやめる（採用済みの段へ戻して幅を半分に）
- 横を採用した後に前後の包絡を上げていた → 最後に上げた包絡を上げる前へ戻し、その点はそれより上を試さない
  （`LongLearner.undo_last`）。前後の測り方（付いて行けたか）には、前後の力が横を削った影響が出ないため。
  リアの滑りは、**滑り始めに計画が加速を求めていたら加速の包絡、制動なら減速の包絡だけ**を疑い、どちらでもなければ
  横を疑う（`slide_long`）。後輪だけで駆動も制動もするので、出口のパワーオーバーは加速、入口の巻き込みは減速のせい。
  実機 `111523` では、出口の加速中の滑りで最後に上げた減速を戻し続け、減速を 2.96 → 2.56 まで下げた（2026-10-29 夜）
- どちらでもない → 横を `time_back` だけ下げて測り直す

横で働いた上限には近づかない（`time_ceil`。`GripMap.ceil` に保存し、次の走行でも）。「横偏差の増え」だけは同じ高さで
2 回出てから天井にする。

### 2026-10-29 に外したもの（単純化）

前兆を「舵角から出る曲率と実曲率の差」で見る検出器（実機の雑音を測ってしきい値を決める仕組み・場所ごとのセル・
頭打ち・平均の不足・挟み込み・路面の急変の扱い）と、それで横の上限を決める元の方式（`grip_by_time` = 0）、
発進の加速度からの初期値、壁の余裕で上げ幅を変える処理を消した。前後の包絡のどの点のせいにするかを、加減速の向き・
直前 1.5 秒の横の使い具合から推し量る規則（`lower_at`）も「最後に上げたものを戻す」に替えた。定数は 94 → 42
（`GripConfig` 73 → 22（横滑りの増えの 2 つと、疑う側を選ぶ `slide_long` を含む）、`LongConfig` 21 → 20）。
タイムで決める方式では、検出器は安全装置の 1 つにすぎず、実機では誤検出の出所だった（`054426`、舵 82〜88% の所）。
本当に外へ膨らめば 2・3 が捕まえる。

## 前後の包絡（`LongLearner`）— 横の使い具合ごとに、実際に出た値へ合わせる

場所ごとには持たず、コース全体で 1 つずつ（`GripMap.acc`・`dec`）。横の使い具合 u = 計画の横加速度 ÷ 横の上限 が
0・0.5・1.0（`raceline.ENV_U`）のときに出せる前後の加速度を持ち、間は直線でつなぐ（実測の g-g 図）。出発点は
校正の値 × 既定の形 `ENV_SHAPE`。天井は指令が届く上限（目標車速のランプ `[safety] cmd_accel_limit_m_s2`）だけ。

- **測る量は「計画からの車速のずれ `e = 車速 − 今の場所の計画の速度` の傾き」**。計画が包絡の `near` 倍以上を求めている間
  を 1 つの窓にし、出だしの `settle_s` を捨てて直線を当てる。**出た加速度 = 求めた加速度 ± de/dt**。窓の平均の u の点へ配る
- 点ごとに `node_min_s` 溜まったら、周の境目で判断する: 出た ÷ 求めた が `short` 未満なら下げ、`follow` 以上なら 1 段上げる
- **u が大きいほど小さい**: 上げたら u の小さい点を持ち上げ、下げたら u の大きい点を下げる
- 減速の安全装置: 制動区間の**終わり**（コーナーの入口）で車速が計画より `lag` 以上速ければ、その u の点を 1 段下げる
- 付いて行けない理由は区別しない（タイヤの限界でも、モータのトルクでも、そこがこの車の限界）

## シムと実機の違い

シムの車は限界で曲率がいきなり頭打ちになり、実機のタイヤは徐々に逃げる。実機の記録は
`raspi/tools/grip_replay.py` で学習器に流し直せる。
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field

import numpy as np

from .raceline import ENV_U, env_at as raceline_env_at

__all__ = ["GripConfig", "GripMap", "GripLearner", "LongConfig", "LongLearner"]

#: 保存ファイルの形式の版。1（セルと雑音の幅を持っていた版）も読める（セルは捨てる）
FORMAT = 2
#: 読み込んだ値を丸める範囲 [m/s²]（壊れた値・古い形式で速度が跳ね上がらないように）
A_FLOOR = 1.0
A_CEIL = 9.0
#: 加速・減速の上限の下限 [m/s²]（直線の値）と、包絡の点の下限
X_FLOOR = 0.5
ENV_MIN = 0.2
#: 前後の上限の包絡の既定の形（`ENV_U` の各点 = 横の使い具合 0・0.5・1.0 での、直線の値に対する割合）。円に近いが、
#: 横の限界いっぱい（u = 1）でも 0 にしない: 0 だと計画がそこで加減速を求めず、測れないので学べない
ENV_SHAPE = (1.0, 0.87, 0.4)
#: 「横偏差の増え」の普段の値を持つ格子の刻み [m]（走行中だけ持つ。保存しない）
CELL_M = 0.25


def _cell(x: float, y: float) -> tuple[int, int]:
    return int(math.floor(x / CELL_M)), int(math.floor(y / CELL_M))


def _num(v, lo: float, hi: float) -> float:
    """有限な数なら `[lo, hi]` へ丸める。そうでなければ 0（= 無い）。"""
    try:
        f = float(v)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(f) or f <= 0.0:
        return 0.0
    return max(lo, min(hi, f))


def _env(vals, straight=None) -> list[float]:
    """保存した包絡を検査して読む（3 点・各点 `[ENV_MIN, A_CEIL]`・u が大きいほど小さい）。無ければ、直線の値
    `straight` に既定の形を掛けたもの。どちらも無ければ空（= 未設定）。"""
    if isinstance(vals, list) and len(vals) == len(ENV_SHAPE):
        out = [_num(x, ENV_MIN, A_CEIL) for x in vals]
        if all(x > 0.0 for x in out):
            for i in range(1, len(out)):
                out[i] = min(out[i], out[i - 1])
            return out
    a = _num(straight, X_FLOOR, A_CEIL)
    return [a * k for k in ENV_SHAPE] if a > 0.0 else []


@dataclass
class GripMap:
    """学習した値（モジュール docstring）。地図と一緒に保存する（`mapstore.save_grip`）。"""

    level: float = 0.0          #: 横の上限 [m/s²]（0 = 未設定）
    #: 前後の上限の包絡 [m/s²]: 横の使い具合 u（`raceline.ENV_U` = 0・0.5・1.0）ごとに出せる加速・減速（`LongLearner`。
    #: 空 = 未設定）。`accel`・`decel` は u = 0（直線）の値
    acc: list[float] = field(default_factory=list)
    dec: list[float] = field(default_factory=list)
    #: 横の安全装置が働いた上限のうち、いちばん低い値 [m/s²]（0 = 無い）。この `time_ceil` 倍より上を試さない。
    #: **読み直しても引き継ぐ**（引き継がないと、走行を始めるたびに前回滑った高さを 1 段越えて試し直す）。
    #: 路面が良くなっても上げ直さないので、上げ直したいときは `.grip.json` を消す
    ceil: float = 0.0
    #: **この地図（コース）で学習を終えた値**（0・空 = まだ無い）。読み込むとこの値から**そのまま**始め、探索もしない
    #: （2 回目以降の走行は 1 周目からベストの設定で走る。2026-10-20 バンビ依頼）。安全装置が働いて下げたら、ここも下げる
    best_lat: float = 0.0
    best_acc: list[float] = field(default_factory=list)
    best_dec: list[float] = field(default_factory=list)

    @property
    def accel(self) -> float:
        """加速の上限 [m/s²]（包絡の u = 0、直線の値。0 = 未設定）。"""
        return self.acc[0] if self.acc else 0.0

    @property
    def decel(self) -> float:
        """減速の上限 [m/s²]（包絡の u = 0、直線の値。0 = 未設定）。"""
        return self.dec[0] if self.dec else 0.0

    def to_json(self) -> str:
        return json.dumps({"format": FORMAT, "level": round(self.level, 3),
                           "acc_env": [round(x, 3) for x in self.acc],
                           "dec_env": [round(x, 3) for x in self.dec],
                           "ceil": round(self.ceil, 3), "best_lat": round(self.best_lat, 3),
                           "best_acc_env": [round(x, 3) for x in self.best_acc],
                           "best_dec_env": [round(x, 3) for x in self.best_dec]})

    @classmethod
    def from_json(cls, text: str) -> "GripMap":
        """検査して読む。形式の違い・壊れた中身は `ValueError`。値は `[A_FLOOR, A_CEIL]` へ丸める。"""
        try:
            return cls._from_json(text)
        except (TypeError, OverflowError, AttributeError, IndexError) as e:
            # 型の食い違い・巨大な数も呼び出し側には `ValueError` として渡す
            raise ValueError(f"grip: 中身が不正（{e!r}）") from e

    @classmethod
    def _from_json(cls, text: str) -> "GripMap":
        raw = json.loads(text)
        if not isinstance(raw, dict) or raw.get("format") not in (1, FORMAT):
            raise ValueError("grip: 形式の版が違う")
        m = cls(level=_num(raw.get("level"), A_FLOOR, A_CEIL), ceil=_num(raw.get("ceil"), A_FLOOR, A_CEIL))
        # 包絡。古いファイル（加速・減速が 1 つずつ）は、その値を直線の値にした既定の形から
        m.acc = _env(raw.get("acc_env"), raw.get("accel"))
        m.dec = _env(raw.get("dec_env"), raw.get("decel"))
        m.best_lat = _num(raw.get("best_lat"), A_FLOOR, A_CEIL)
        m.best_acc = _env(raw.get("best_acc_env"), raw.get("best_acc"))
        m.best_dec = _env(raw.get("best_dec_env"), raw.get("best_dec"))
        if not (m.best_lat > 0.0 and m.best_dec and m.best_acc):
            m.best_lat, m.best_acc, m.best_dec = 0.0, [], []
        return m


@dataclass
class LongConfig:
    #: 上げる 1 段の割合。下げたら半分（`step_min` まで）。続けて上げられても広げない（広げた版は、シム toyota2 で
    #: 横を使う所の減速を 1 回に 16% 上げ、その周に 28cm 膨らんだ。8% でも 1 段の行き過ぎで 33cm 膨らんだ）
    step: float = 0.05
    step_min: float = 0.02
    #: 計画が包絡のこの割合以上の加速（減速）を求めている間（＝包絡で速度が決まっている所）を 1 つの窓にする
    near: float = 0.85
    #: 窓の出だしで捨てる時間 [s]（速度ループの遅れが溜まる過渡。planner が速度ループの時定数の `settle_k` 倍を
    #: 入れる。実機 `122530` で 1 倍 → 2.5 倍にすると、窓ごとの減速のばらつきが 0.51 → 0.25m/s² になった）と、
    #: 残りがこれより短い窓は使わない [s]
    settle_s: float = 0.17
    settle_k: float = 2.5
    fit_min_s: float = 0.08
    #: 包絡の点ごとに窓をこの長さ [s] 溜めたら、周の境目でその点を判断する
    node_min_s: float = 0.3
    #: 窓の終わりで車速が計画からこれ以上遅れていた（減速なら速すぎた）窓があった点は、その判断では上げない [m/s]。
    #: 窓の傾きだけだと、出だしの遅れを取り戻す動きで 出た ÷ 求めた が 1 を超え（実機 `surge_20261027_101210` で
    #: 1.02〜1.05）、コーナーの入口へ計画より 0.2m/s 速く入る周があっても、横を使う所の減速を上げ続けた
    end_ok: float = 0.05
    #: 出た ÷ 求めた がこれ未満なら下げる・これ以上なら上げる（その間は保つ）。1 回に下げるのはこの割合まで
    short: float = 0.90
    follow: float = 0.97
    cut_max: float = 0.8
    #: 下げた高さ（付いて行けなかった値）には、この割合より近づかない（`_Side.ceil`。走行中だけ持つ）。
    #: 無いと、上げる判断と減速の入口の安全装置の境目が 1 段より狭い所で、上げては戻すを繰り返した
    #: （シム toyota2: 減速 2.44〜2.74 を 20〜30 秒ごとに往復）
    ceil_gap: float = 0.98
    #: 出た ÷ 求めた で下げたときは、付いて行けなかった値ではなく**実際に出た値**のこの倍を上の天井にする（失敗した値の
    #: 0.98 倍までだと、真値より 2 割高い所で失敗した後に、また真値の上まで上げて失敗するのを繰り返した。シム toyota2）
    got_margin: float = 1.04
    #: 減速の安全装置: 制動区間の終わりで車速が計画より速かったぶん [m/s]。包絡の `zone` 倍以上の減速を求めている
    #: 間を制動区間とし、`min_s` 秒より短い区間は見ない。終わりの `tail_s` 秒の平均を、その区間の値にする
    lag: float = 0.10
    zone: float = 0.5
    min_s: float = 0.3
    tail_s: float = 0.1
    #: 学習値（横も含む）が変わってからこの時間は見ない [s]（速度プロファイルが作り直される）
    quiet_s: float = 0.4
    #: 保存した値から再開するときに掛ける割合と、上限を下げてよい下限（出発点に対する割合）
    back_off: float = 0.92
    floor: float = 0.3


def _hat(u: float) -> tuple[float, float, float]:
    """横の使い具合 `u` の、包絡の各点（`ENV_U`）への重み（線形補間の重み。和は 1）。"""
    u = min(1.0, max(0.0, u))
    if u < 0.5:
        w = u / 0.5
        return (1.0 - w, w, 0.0)
    w = (u - 0.5) / 0.5
    return (0.0, 1.0 - w, w)


class _Side:
    """加速か減速の一方の包絡（`LongLearner`）。窓を集め、包絡の点ごとに溜まったら判断する。"""

    def __init__(self, sign: float) -> None:
        self.sign = sign                    #: +1 = 加速、−1 = 減速
        self.max = A_CEIL
        self.floor = X_FLOOR
        n = len(ENV_SHAPE)
        self.step = [0.0] * n
        self.ceil = [math.inf] * n          #: 点ごとの、付いて行けなかった値のいちばん低いもの（inf = 無い）
        self.got = [0.0] * n                #: 点ごとに溜めた Σ 出た×長さ×重み
        self.req = [0.0] * n                #: 同 求めた
        self.len = [0.0] * n                #: 同 長さ×重み [s]
        self.ratio = [0.0] * n              #: 直近に判断した 出た ÷ 求めた（診断用。0 = まだ）
        self.decisions = 0
        self.short_s = 0.0                  #: 診断: 窓が短くて捨てた時間 [s]
        self.late = [False] * n             #: 点ごとに、終わりで計画に遅れた窓があったか（上げない。`end_ok`）
        self.win: list[tuple[float, float, float, float]] = []  #: 今の窓の (時刻, ずれ, 求めた大きさ, u)

    def close(self, cfg: LongConfig) -> None:
        """今の窓を閉じる。出だしを捨てて、ずれに直線を当て、窓の平均の u の点へ配る。"""
        w, self.win = self.win, []
        if not w:
            return
        t0 = w[0][0] + cfg.settle_s
        k = [r for r in w if r[0] >= t0]
        if len(k) < 3 or k[-1][0] - k[0][0] < cfg.fit_min_s:
            self.short_s += w[-1][0] - w[0][0]
            return
        ts = np.asarray([r[0] for r in k])
        de = float(np.polyfit(ts - ts[0], np.asarray([r[1] for r in k]), 1)[0])
        req = float(np.mean([r[2] for r in k]))
        d = float(ts[-1] - ts[0])
        # 加速: e の傾き = 出た − 求めた。減速（大きさ）: e の傾き = 求めた − 出た
        got = req + self.sign * de
        late = -self.sign * k[-1][1] > cfg.end_ok      # 終わりで計画に遅れていた（減速なら速すぎた）
        for i, h in enumerate(_hat(float(np.mean([r[3] for r in k])))):
            if h > 0.0:
                self.got[i] += got * d * h
                self.req[i] += req * d * h
                self.len[i] += d * h
                self.late[i] = self.late[i] or late


class LongLearner:
    """前後の上限の包絡を走行中に決める（モジュール docstring）。値は `GripMap.acc`・`dec`。"""

    def __init__(self, cfg: LongConfig | None = None) -> None:
        self.cfg = cfg or LongConfig()
        self.reset(GripMap())

    def reset(self, m: GripMap) -> None:
        self.map = m
        self.acc = _Side(1.0)
        self.dec = _Side(-1.0)
        self.down_only = False
        self.ver = 0                        #: 値が変わるたびに進む
        self.ver_down = 0
        self.events: list[dict] = []        #: 値を下げた記録（表示・ベンチ用）
        #: 上げた記録（直近 8 回）: (時刻, 側, 点, 上げる前の包絡)。横の安全装置が働いたら最後のものを戻す（`undo_last`）
        self._raises: list[tuple[float, _Side, int, list[float]]] = []
        self.windows = 0                    #: 判断した回数（包絡の点ごと・減速の安全装置）
        self.last = 0.0                     #: 直近の周の、制動区間の終わりの差のいちばん大きい値 [m/s]（診断用）
        self._zone: list[float] | None = None   #: 今の制動区間の「車速 − 計画の速度」（減速の安全装置）
        self._zone_u = 0.0                  #: その区間の終わりの u
        self._ends: list[tuple[float, float]] = []  #: この周の、区間ごとの (終わりの差, u)
        self._plan = (0.0, 0.0, 0.0)        #: 直近に受け取った計画の (加速度, 速度) と、それが変わった時刻
        self._t = self._dt = 0.0
        self._quiet_t = -1e9
        #: 診断: 包絡の `near` 倍以上を求められていた時間と、そのうち見なかった時間 [s]
        #: （`gate`・学習値が変わった直後・計画が無いか遅い）。実機で判断が出ないときの手がかり（2026-10-26）
        self.want_s = self.gated_s = self.quiet_s = self.noplan_s = 0.0

    def _vals(self, side: _Side) -> list[float]:
        return self.map.acc if side is self.acc else self.map.dec

    def diag(self) -> dict[str, float]:
        """記録用（`AutoState.grip`）。`acc0..2`・`dec0..2` = 包絡の各点 [m/s²]、`*_r` = 直近の 出た ÷ 求めた、
        `*_s` = 溜めた窓 [s]、`acc_n`・`dec_n` = 判断した回数、`*_short` = 短くて捨てた時間 [s]、`long_*` = 強い加減速を
        求められた時間と、そのうち見なかった時間 [s]。"""
        out: dict[str, float] = {}
        for side, k in ((self.acc, "acc"), (self.dec, "dec")):
            vals = self._vals(side)
            for i in range(len(ENV_SHAPE)):
                out[f"{k}{i}"] = round(vals[i], 3) if i < len(vals) else 0.0
                out[f"{k}{i}_r"] = round(side.ratio[i], 3)
                out[f"{k}{i}_s"] = round(side.len[i], 2)
            out[f"{k}_n"] = float(side.decisions)
            out[f"{k}_short"] = round(side.short_s, 2)
        out.update({"long_want": round(self.want_s, 2), "long_gated": round(self.gated_s, 2),
                    "long_quiet": round(self.quiet_s, 2), "long_noplan": round(self.noplan_s, 2)})
        return out

    @property
    def dec_done(self) -> bool:
        """減速の包絡が落ち着いたか（判断を 1 度でもして、上げる途中でない）。ラインの引き直しの判断に使う。"""
        return self.dec.decisions > 0 and max(self.dec.step) <= self.cfg.step * 0.5 + 1e-9

    def configure(self, *, acc_init: float, dec_init: float, dec_max: float, down_only: bool,
                  acc_max: float | None = None, loop_tau_s: float | None = None) -> None:
        """:param acc_init: 加速の直線の値の出発点（校正の値）[m/s²]。包絡は既定の形 `ENV_SHAPE` を掛けて作る
        :param acc_max: 包絡を上げる天井（目標車速のランプの上限）[m/s²]。`None` なら `dec_max`
        :param loop_tau_s: 速度ループの時定数 [s]（窓の出だしで捨てる時間 = その `settle_k` 倍）"""
        m = self.map
        self.down_only = down_only
        if acc_max is None:
            acc_max = dec_max
        if loop_tau_s is not None and loop_tau_s > 0.0:
            self.cfg.settle_s = max(0.05, self.cfg.settle_k * loop_tau_s)
        changed = False
        for side, init, top, attr in ((self.acc, acc_init, acc_max, "acc"), (self.dec, dec_init, dec_max, "dec")):
            side.max = max(X_FLOOR, top)
            init = max(X_FLOOR, min(side.max, init))
            side.floor = max(ENV_MIN, self.cfg.floor * init)
            side.step = [s if s > 0.0 else self.cfg.step for s in side.step]
            cur = getattr(m, attr)
            new = list(cur) if len(cur) == len(ENV_SHAPE) else [init * k for k in ENV_SHAPE]
            new = [min(side.max, max(side.floor, x)) for x in new]
            if new != cur:
                setattr(m, attr, new)
                changed = True
        if changed:
            self._bump(True)

    def load(self) -> None:
        """保存した値から再開する（`reset` の後）。そのまま信じず `back_off` 倍から。"""
        m = self.map
        m.acc = [max(ENV_MIN, x * self.cfg.back_off) for x in m.acc]
        m.dec = [max(ENV_MIN, x * self.cfg.back_off) for x in m.dec]

    def settle(self) -> None:
        """地図に保存した「学習を終えた値」から始める（探さない。下げる向きと、上限近くで付いて行けたら上げる向きは働く）。"""
        for side in (self.acc, self.dec):
            side.step = [self.cfg.step_min] * len(ENV_SHAPE)
            side.decisions = max(side.decisions, 1)

    @property
    def last_raise_t(self) -> float:
        """直近に包絡を上げた時刻 [s]（無ければ −inf）。"""
        return self.raise_t()

    def _last(self, side: _Side | None) -> int:
        """`_raises` の中の、`side`（`None` ならどちらでも）を最後に上げた記録の位置（無ければ −1）。"""
        for k in range(len(self._raises) - 1, -1, -1):
            if side is None or self._raises[k][1] is side:
                return k
        return -1

    def raise_t(self, side: _Side | None = None) -> float:
        """`side`（`None` ならどちらでも）を最後に上げた時刻 [s]（無ければ −inf）。"""
        k = self._last(side)
        return self._raises[k][0] if k >= 0 else -math.inf

    def undo_last(self, t: float, kind: str, side: _Side | None = None) -> bool:
        """直近に上げた包絡を、上げる前へ戻し、その点はそれより上を試さない（横の学習が、横の安全装置が働いたときに
        呼ぶ。モジュール docstring）。戻せた（上げた記録があった）か。

        前後の測り方（付いて行けたか）には、前後の力が横を削った影響が出ない。シム toyota2 では減速の包絡が真値より上がって
        毎周同じ所で線から外れ、それを「直前に包絡いっぱいを求めていた所の u の点」のせいにして下げる版（2026-10-27〜29）は、
        上がり過ぎた点と違う点ばかり下げて外れ続けた。最後に変えたものを疑う方が、確実に元の走りへ戻る。

        :param side: 戻す側（`None` ならどちらでも、最後に上げたもの）。滑り始めに加速していたか制動していたかで
            横の学習が選ぶ（実機 `111523` では、出口の加速中のパワーオーバーで減速を 2.96 → 2.56 まで下げた）"""
        k = self._last(side)
        if k < 0:
            return False
        _, side, i, before = self._raises.pop(k)
        name = "acc" if side is self.acc else "dec"
        cur = list(getattr(self.map, name))
        # 上げた後に下げた点は、下げたままにする
        setattr(self.map, name, [min(c, b) for c, b in zip(cur, before)])
        side.ceil[i] = min(side.ceil[i], before[i] / self.cfg.ceil_gap)
        self.events.append({"t": t, "kind": kind, "u": ENV_U[i], "ask": cur[i], "got": before[i]})
        side.got = [0.0] * len(before)
        side.req = [0.0] * len(before)
        side.len = [0.0] * len(before)
        side.late = [False] * len(before)
        self._bump(True)
        return True

    def observe(self, t: float, v: float, a_plan: float, gate: str, v_plan: float | None,
                u: float = 0.0) -> None:
        """車両状態 1 つぶん。

        :param a_plan: 今の場所でプロファイルが求めている前後の加速度 [m/s²]（減速は負）
        :param v_plan: 今の場所のプロファイルの速度 [m/s]
        :param u: 今の場所の計画の横の使い具合（横加速度 ÷ 横の限界）
        :param gate: 空でなければ見ない理由（通常の周回でない・前兆で減速している）
        """
        cfg, m = self.cfg, self.map
        self._dt, self._t = t - self._t, t
        if v_plan is not None and (a_plan, v_plan) != self._plan[:2]:
            self._plan = (a_plan, v_plan, t)
        caps = [(side, raceline_env_at(vals, u) if (vals := self._vals(side)) else 0.0)
                for side in (self.acc, self.dec)]
        binding = any(c > 0.0 and side.sign * a_plan >= cfg.near * c for side, c in caps)
        if 0.0 < self._dt < 0.1 and binding:
            self.want_s += self._dt
            if gate:
                self.gated_s += self._dt
            elif v_plan is None or v < 0.5:
                self.noplan_s += self._dt
            elif t - self._quiet_t < cfg.quiet_s:
                self.quiet_s += self._dt
        if gate or v_plan is None or v < 0.5 or t - self._quiet_t < cfg.quiet_s:
            self._zone = None                   # 途中で切れた区間は捨てる
            self.acc.win, self.dec.win = [], []
            return
        # 計画は planner の周期（10Hz）でしか更新されない。その間は計画の加速度で今へ進める
        e = v - (v_plan + a_plan * min(t - self._plan[2], 0.2))
        for side, cap in caps:
            want = side.sign * a_plan
            if cap > 0.0 and want >= cfg.near * cap:
                if side.win and t - side.win[-1][0] > 0.05:
                    side.close(cfg)
                side.win.append((t, e, want, u))
            elif side.win:
                side.close(cfg)
        # 減速の安全装置（制動区間の終わりの遅れ）
        if -a_plan < cfg.zone * caps[1][1]:
            z, self._zone = self._zone, None
            dt = self._dt if 0.0 < self._dt < 0.1 else 0.01
            if z is not None and len(z) * dt >= cfg.min_s:
                k = max(1, int(round(cfg.tail_s / dt)))
                self._ends.append((sum(z[-k:]) / len(z[-k:]), self._zone_u))
            return
        if self._zone is None:
            self._zone = []
        self._zone.append(e)
        self._zone_u = u

    def lap(self, t: float) -> None:
        """1 周した。包絡の点ごとに溜まっていれば判断し、減速の安全装置を見る。"""
        cfg = self.cfg
        ends, self._ends = self._ends, []
        lowered = False
        for side, name in ((self.acc, "加速"), (self.dec, "減速")):
            vals = self._vals(side)
            for i in range(len(vals)):
                if side.len[i] < cfg.node_min_s or side.req[i] <= 0.0:
                    continue
                r = side.ratio[i] = side.got[i] / side.req[i]
                late, side.late[i] = side.late[i], False
                side.got[i] = side.req[i] = side.len[i] = 0.0
                side.decisions += 1
                self.windows += 1
                val = vals[i]
                if r < cfg.short:
                    self._set(side, i, val * max(r, cfg.cut_max), t, name, ratio=r)
                    side.ceil[i] = min(side.ceil[i], val * r * cfg.got_margin / cfg.ceil_gap)
                    lowered = lowered or side is self.dec
                elif r >= cfg.follow and not late and not self.down_only:
                    new = min(side.max, val * (1.0 + side.step[i]), cfg.ceil_gap * side.ceil[i])
                    if new > val + 1e-6:
                        self._set(side, i, new, t, name, ratio=r)
        if ends:
            # 減速の安全装置: この周に減速を下げていなければ、入口で遅れた周は、その入口の u の点を 1 段下げる
            e, u = max(ends)
            self.last = e
            if e > cfg.lag and not lowered:
                self.windows += 1
                self.dec.decisions = max(self.dec.decisions, 1)
                i = int(np.argmax(_hat(u)))
                self._set(self.dec, i, self.map.dec[i] / (1.0 + cfg.step), t, "減速（入口で遅れ）", lag=e)

    def _set(self, side: _Side, i: int, new: float, t: float, kind: str, **info) -> None:
        """包絡の点 `i` を `new` にする。u が大きいほど出せる前後の力は小さい（u = 0 の直線がいちばん大きい）ので、
        上げたら u の小さい点を同じ値まで持ち上げ、下げたら u の大きい点を同じ値まで下げる。"""
        vals = list(self._vals(side))
        val = vals[i]
        new = min(side.max, max(side.floor, new))
        if new < val:
            side.step[i] = max(self.cfg.step_min, 0.5 * side.step[i])
            side.ceil[i] = min(side.ceil[i], val)
            self.events.append({"t": t, "kind": kind, "u": ENV_U[i], "ask": val, "got": new, **info})
            vals[i] = new
            for j in range(i + 1, len(vals)):
                vals[j] = min(vals[j], new)
        elif new > val:
            self._raises = (self._raises + [(t, side, i, list(vals))])[-8:]
            vals[i] = new
            for j in range(i):
                vals[j] = min(side.max, max(vals[j], new))
        else:
            return
        setattr(self.map, "acc" if side is self.acc else "dec", vals)
        side.got = [0.0] * len(vals)            # 包絡が変わった。溜めた窓は今の値の判断に使わない
        side.req = [0.0] * len(vals)
        side.len = [0.0] * len(vals)
        side.late = [False] * len(vals)
        self._bump(new < val)

    def _bump(self, down: bool) -> None:
        self.ver += 1
        if down:
            self.ver_down += 1
        self.note_change()

    def note_change(self) -> None:
        """学習値（横も含む）が変わった。今の区間・窓を捨て、しばらく見ない。"""
        self._quiet_t = self._t
        self._zone = None
        self.acc.win, self.dec.win = [], []


#: 周回のタイムに数えてよい「学習を止める理由」（走り方は変わっていない）
_TIME_OK_GATES = ("舵が上限", "舵が上限の直後", "曲がり切れない区間")


@dataclass
class GripConfig:
    # ── 横の上限をタイムで決める ──
    #: 1 段で上げる割合（縮まなければ半分にして、`time_step_min` を切ったら終わり）。8% の段で縮む時間は
    #: 実機で 0.15〜0.2s、同じ計画で続けて走った周のばらつきは sd 0.015〜0.04s
    time_step: float = 0.08
    time_step_min: float = 0.03
    #: 採用の条件: 実際に縮んだ時間 ÷ 速度プロファイルが縮むはずの時間 がこれ以上。実機 4 走行（2026-10-13）では、
    #: 横 6.0 までの段は 0.7〜1.4、6.2〜6.9 の段は 0.35〜0.6、7 より上の段は負（上げるほど遅い）
    time_gain: float = 0.6
    #: 採用したら幅をこの倍率で戻す（`time_step` まで）。戻さないと、雑音で 1 回断っただけで幅が半分のまま残る
    time_regrow: float = 2.0
    #: 計画のラップがこの割合も縮まない段は採用しない（`v_max` や加減速で決まっていて、上げる意味が無い）
    time_plan_min: float = 0.003
    #: これより短い「1 周」は数えない [s]（自己位置が跳んでゲートを 2 度またいだ）
    time_lap_min_s: float = 2.0
    #: 採用済みの段で安全装置が働いたときに下げる割合と、続けて下げない時間 [s]（同じ出来事の続き）
    time_back: float = 0.05
    time_guard_quiet_s: float = 3.0
    #: 安全装置が働いた上限には、この割合より近づかない（`GripMap.ceil`）
    time_ceil: float = 0.97
    # ── 安全装置（モジュール docstring）──
    #: リアの滑り: 逆ハンのまま横加速度がこれ以上 [m/s²] の状態が `slide_s` 秒たまった。実機では上限 6.5 未満で 0 秒、
    #: 6.6 以上で毎周 0.05〜0.24 秒
    slide_alat: float = 3.0
    slide_s: float = 0.06
    #: リアの滑り（横滑りの増え）: ヨーレート×車速 が IMU の横加速度を これ [m/s²] 以上上回った状態（0.05s の一次ローパス）が
    #: `slide_excess_s` 秒続いた。逆ハンより早く、制動しながらの巻き込みも捕まえる。実機 3 走行（2026-10-29）を流し直して決めた:
    #: `083921` の滑らない前半 23 周は 0 回・減速を上げすぎた後半は入口でほぼ毎周、`111523` は出口のパワーオーバーと入口の巻き込み。
    #: 1.1 だと `111523` の出口の 2 回を逃し、0.9 だと `083921` の滑らない周で出始める
    slide_excess: float = 1.0
    slide_excess_s: float = 0.1
    #: リアの滑りが続いている間に計画が求めていた前後の加速度（絶対値の最大）[m/s²] がこれ以上なら加速の包絡、−これ以下なら減速の包絡を、
    #: 間なら横の上限を疑う（`_guard`）。実機 `111523`: 出口のパワーオーバー +1.2〜1.5、入口の制動 −2.2〜−3.3
    slide_long: float = 0.3
    #: 線から外れた: 横偏差がこれを超えた [m]（横を上限の `cross_use` 以上使っている所）
    cross_thr: float = 0.12
    cross_use: float = 0.8
    #: 横偏差の増え: コーナーの中の横偏差が、その場所の普段より これ以上 [m] 大きかった周。実機の同じ場所の周ごとの差は
    #: 最大 2.6cm。シム `--real-noise` では、これが無いと `cross_thr` に届く前に何周も 5〜9cm 膨らみ続けた
    cross_gain: float = 0.04
    # ── 安全装置が働いたときに目標速度を落とす ──
    #: 落とす先の余裕・落とす速さ（横の上限に対する割合。強く減速するとそのぶん横に使える力が減る）・続ける時間 [s]
    cap_margin: float = 0.03
    cap_decel: float = 0.35
    cap_max_s: float = 1.5
    # ── 歯止め・再開 ──
    #: 横の上限を下げてよいのは、出発点のこの割合まで（加速・減速の `LongConfig.floor` と同じ考え）
    floor: float = 0.5
    #: 保存した値（学習を終えていない値）から再開するときに掛ける割合（日が変わると路面は変わる）
    back_off: float = 0.92


class GripLearner:
    """横の上限をタイムで決め、前後の包絡の学習（`LongLearner`）へ車両状態を渡す（モジュール docstring）。
    planner が 100Hz で `observe` を呼ぶ。"""

    def __init__(self, yaw_wn: float = 0.0, yaw_zeta: float = 1.0, cfg: GripConfig | None = None) -> None:
        self.cfg = cfg or GripConfig()
        #: 舵角から出る曲率に掛けるヨーの2次遅れ（同定値。固有振動数 [rad/s] と減衰比）。0 なら掛けない。
        #: リアの滑り（逆ハン）の判定で、切り返しの遅れを逆ハンと取り違えない
        self.yaw_wn = max(0.0, yaw_wn)
        self.yaw_zeta = max(0.1, yaw_zeta)
        self.map = GripMap()
        self.long = LongLearner()
        self.reset()

    def reset(self, keep_map: bool = False) -> None:
        if not keep_map:
            self.map = GripMap()
        self.long.reset(self.map)
        cfg = self.cfg
        #: 値が変わるたびに進む（速度プロファイルの作り直しの鍵）。`ver_down` は下げたときだけ
        self.ver = 0
        self.ver_down = 0
        self.max_level = A_CEIL
        self.floor = A_FLOOR                #: 横の上限の下限 [m/s²]（`configure`）
        self.down_only = False
        self.converged = False
        self.t_converged: float | None = None
        self.events: list[dict] = []        #: 安全装置が働いた記録（`reason`・ベンチ用）
        self.steps: list[dict] = []         #: 段ごとの判断の記録（ベンチ・診断用）
        self.cap: float | None = None       #: 安全装置で抑えている目標速度 [m/s]（`None` = 抑えていない）
        self._cap_t = 0.0
        # 100Hz の状態
        self._t = 0.0
        self._k2 = (0.0, 0.0)               #: ヨーの2次遅れの状態（値・傾き）
        self._kexp = 0.0                    #: 舵角から出る曲率（遅れを通した後）
        self._kmeas = 0.0                   #: 実曲率（ヨーレート ÷ 車速）
        self._cross = 0.0                   #: 直近の横偏差 [m]（planner が 10Hz で渡す）
        self._u = 0.0                       #: 今の場所の計画の横の使い具合
        self._slide = 0.0
        #: 逆ハンがたまっている間に計画が求めていた前後の加速度のうち、絶対値の大きいもの [m/s²]（`_guard` の疑う先）。
        #: 制動の始めに滑り始めると、始めの瞬間はまだ 0（実機 `083921` の入口）
        self._slide_ax = 0.0
        self._guard_t = -1e9
        #: 横滑りの増え（`slide_excess`）: ならした増え [m/s²]、続いた時間 [s]
        self._yex = 0.0
        self._yex_s = 0.0
        self._yex_ax = 0.0                  #: 同じく、横滑りの増えが続いている間 [m/s²]
        #: 横偏差の普段の値: セル → [平均, 入れた周の数]（走行中だけ）と、この周のセルごとの最大（→ (横偏差, x, y, 横加速度)）
        self._cx: dict[tuple[int, int], list[float]] = {}
        self._cx_lap: dict[tuple[int, int], tuple[float, float, float, float]] = {}
        #: 横の上限を最後に上げた時刻 [s]（`_guard` が、最後に上げたのが横か前後の包絡かを比べる）
        self._lat_up_t = -math.inf
        self.last_cross_gain = 0.0          #: 直近の周の、普段からの横偏差の増えの最大 [m]（診断用）
        # タイム
        self._step = cfg.time_step
        #: 採用した段の (上限, ラップ, 計画のラップ)。`None` = 今の段を測ってから決める
        self._ref: tuple[float, float, float] | None = None
        self._ref_old: tuple[float, float, float] | None = None     #: 測り直す前の値（平均する）
        self._confirmed = False             #: 採用済みの段の測り直しを済ませたか
        self._soft: list[float] = []        #: 「横偏差の増え」が出た上限 [m/s²]
        self._gate_line: tuple[float, float, float, float] | None = None
        self._gate_prev: tuple[float, float, float, float] | None = None    #: 直前の (時刻, x, y, 進み)
        self._lap_t0 = 0.0
        self._lap_ok = False
        self._plan_sum = [0.0, 0.0]         #: 周の中で足した [計画のラップ, 横だけ戻した計画のラップ]
        self._plan_n = [0, 0]
        self.last_lap = (0.0, 0.0)          #: 直近に数えた (ラップ, 計画のラップ) [s]
        #: 読み込み（か作り直し）の後に、1 周を測り終えたか（`worth_saving`）
        self._measured = False
        #: 地図に保存した「学習を終えた値」（`GripMap.best_*`）から始めたか
        self.from_best = False

    # ── 設定と保存 ──

    def configure(self, *, init_level: float, max_level: float, down_only: bool) -> None:
        """planner が毎周期渡す設定。横の上限が未設定なら `init_level` から始める。"""
        self.max_level = max(A_FLOOR, min(A_CEIL, max_level))
        self.down_only = down_only
        self.floor = max(A_FLOOR, min(self.max_level, self.cfg.floor * init_level))
        m = self.map
        if m.level <= 0.0:
            m.level = max(A_FLOOR, min(self.max_level, init_level))
            self._bump(down=True)
        elif m.level < self.floor:
            m.level = self.floor            # 保存した値が歯止めより低い（以前の測り違い）
            self._bump(down=False)
        elif m.level > self.max_level:
            m.level = self.max_level
            self._bump(down=True)

    def load(self, m: GripMap) -> None:
        """保存した値から再開する。学習を終えた値（`best_*`）があればそこからそのまま、無ければ `back_off` 倍から。"""
        if m.level > 0.0:
            m.level = max(A_FLOOR, m.level * self.cfg.back_off)
        self.map = m
        self.long.reset(m)
        self.long.load()
        self.converged = False
        self._ref = self._ref_old = None
        self._confirmed = False
        self._step = self.cfg.time_step
        self._measured = False
        self.from_best = False
        if m.best_lat > 0.0 and m.best_dec and m.best_acc:
            m.level = m.best_lat
            m.acc, m.dec = list(m.best_acc), list(m.best_dec)
            self.converged = True
            self.long.settle()
            self.from_best = True
        self._bump(down=True)

    def settled(self) -> bool:
        """この地図での学習を終えたか（横の探索を終え、1 周を測った）。"""
        return self.converged and self._measured and self.cap is None

    def mark_best(self) -> None:
        """終えていれば今の値を `GripMap.best_*` に書く。終えていなくても、下げた値は best にも反映する
        （安全装置で下げた後に終えないまま走行をやめても、次に高い値から始めない）。"""
        m = self.map
        if self.settled():
            m.best_lat = self.ref_level or m.level
            m.best_acc, m.best_dec = list(m.acc), list(m.dec)
        elif m.best_lat > 0.0 and m.best_acc and m.best_dec:
            m.best_lat = min(m.best_lat, self.ref_level or m.level)
            m.best_acc = [min(a, b) for a, b in zip(m.best_acc, m.acc)]
            m.best_dec = [min(a, b) for a, b in zip(m.best_dec, m.dec)]

    def worth_saving(self) -> bool:
        """保存してよいか。読み込んだ値は `back_off` 倍から始めるので、1 周も測らないうちに保存すると、
        短い走行を繰り返すたびに下がっていく。安全装置が働いたときは保存する。"""
        return self._measured or bool(self.events)

    def new_line(self) -> None:
        """追う線が変わった（planner が呼ぶ）。1 周の長さも、場所ごとの横偏差の普段の値も変わる。"""
        self._ref = self._ref_old = None
        self._confirmed = False
        self._lap_ok = False
        self._cx, self._cx_lap = {}, {}

    # ── 速度プロファイルへ渡す値 ──

    def levels(self, xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """経路の各点の `(求める横加速度, 摩擦円の半径)` [m/s²]（どちらも今の横の上限。コースで 1 つ）。"""
        lv = np.full(len(xy), max(self.floor, self.map.level))
        return lv, lv

    @property
    def ref_level(self) -> float:
        """比べる相手の横の上限 [m/s²]（段を試している間だけ。0 = 無い）。"""
        ref = self._ref
        return ref[0] if ref is not None and self.map.level > ref[0] * (1.0 + 1e-6) else 0.0

    def observe_plan(self, lap_s: float, ref_lap_s: float = 0.0) -> None:
        """今の速度プロファイルどおりに走ったときの 1 周の時間 [s]（planner が計画の周期で渡す）。

        :param ref_lap_s: 横の上限だけを `ref_level` に戻したときの同じ時間 [s]（0 = 無い）
        """
        for k, val in enumerate((lap_s, ref_lap_s)):
            if val > 0.0:
                self._plan_sum[k] += val
                self._plan_n[k] += 1

    def observe_cross(self, cross: float) -> None:
        """経路からの横偏差 [m]（planner が計画の周期で渡す。学習を止めている間は 0）。"""
        self._cross = cross

    # ── 100Hz の観測 ──

    def observe(self, t: float, x: float, y: float, v: float, yaw_rate: float, kappa_steer: float,
                *, a_x: float = 0.0, gate: str = "", tc_active: bool = False,
                v_plan: float | None = None, long_gate: str = "", u_plan: float = 0.0,
                a_y: float | None = None) -> None:
        """車両状態 1 つぶん。

        :param kappa_steer: 今の実舵角から出る曲率 [1/m]（同定した舵の効きを通したもの）
        :param a_x: 今の場所で計画している前後の加速度 [m/s²]（速度プロファイルの傾き）
        :param v_plan: 今の場所のプロファイルの速度 [m/s]（加速・減速の学習が使う）
        :param gate: 空でなければ学習を止める理由（planner が決める）
        :param long_gate: 空でなければ、加速・減速の学習だけを止める理由（横は見る）
        :param u_plan: 今の場所の計画の横の使い具合（横加速度 ÷ 横の上限）
        :param a_y: IMU の横加速度 [m/s²]（左が正。`None` = 使えない。横滑りの増えを見ない）
        """
        cfg = self.cfg
        dt = t - self._t
        if 0.0 <= dt < 1e-4:
            return                              # 同じ時刻の状態が続けて届いた（まとめて受信）。捨てる
        self._t = t
        if not 0.0 < dt < 0.1:
            # 間が空いた。遅れの状態を今の値から始める
            self._kexp = kappa_steer
            self._k2 = (kappa_steer, 0.0)
            self._kmeas = yaw_rate / v if v > 0.3 else 0.0
            self._gate_prev, self._lap_ok, self._slide = None, False, 0.0
            return
        # 舵角から出る曲率 → ヨーの2次遅れ → 実曲率と同じローパス
        k2, k2d = self._k2
        if self.yaw_wn > 1.0:
            n = max(1, int(math.ceil(dt * self.yaw_wn / 0.2)))
            h = dt / n
            for _ in range(n):
                k2d += (self.yaw_wn ** 2 * (kappa_steer - k2) - 2.0 * self.yaw_zeta * self.yaw_wn * k2d) * h
                k2 += k2d * h
        else:
            k2 = kappa_steer
        self._k2 = (k2, k2d)
        lp = 1.0 - math.exp(-dt / 0.03)
        self._kexp += (k2 - self._kexp) * lp
        self._kmeas += ((yaw_rate / v if v > 0.3 else 0.0) - self._kmeas) * lp
        if a_y is not None:
            # IMU の横加速度はそのまま使う（ファームが起動時に零点を合わせる。実機の停止中は −0.13〜0.00m/s²）。
            # 直線で零点を測る版（2026-10-29 夜）は、直線が無いコースで S 字の切り返しの過渡を拾って 0.25〜0.56 になり、
            # 左のコーナーだけ感度が上がっていた（右より約 0.9m/s² 敏感）
            rv = yaw_rate * v
            raw = math.copysign(1.0, rv) * (rv - a_y) if v > 0.8 and abs(rv) >= cfg.slide_alat else 0.0
            self._yex += (raw - self._yex) * (1.0 - math.exp(-dt / 0.05))
        self._u = u_plan
        if self.cap is not None and t - self._cap_t > cfg.cap_max_s:
            self.cap = None
        self.long.observe(t, v, a_x, gate or long_gate or ("横の安全装置" if self.cap is not None else ""),
                          v_plan, u_plan)
        if (gate and gate not in _TIME_OK_GATES) or self.cap is not None:
            self._lap_ok = False
        if not gate and v > 0.8:
            self._watch(t, x, y, v, dt, a_x, tc_active)
        else:
            self._slide = max(0.0, self._slide - dt)
        self._time_lap(t, x, y, v, gate)

    def _watch(self, t: float, x: float, y: float, v: float, dt: float, a_x: float, tc_active: bool) -> None:
        """安全装置 1・2 を見て、3 のための横偏差を溜める（モジュール docstring）。"""
        cfg = self.cfg
        a = v * v * abs(self._kmeas)
        # 1. リアの滑り: 舵が求める曲率と実曲率が逆向きのまま、横加速度が出ている
        if self._kexp * self._kmeas < 0.0 and a >= cfg.slide_alat:
            if self._slide <= 0.0 or abs(a_x) > abs(self._slide_ax):
                self._slide_ax = a_x
            self._slide += dt
            if self._slide >= cfg.slide_s:
                self._slide = 0.0
                self._slow(t, v)
                self._guard(t, x, y, "リアの滑り", a, along=self._slide_ax)
        else:
            self._slide = max(0.0, self._slide - dt)
        # 1'. リアの滑り（横滑りの増え）: 車がヨーで回っているぶんに、横加速度が付いて来ていない
        if self._yex > cfg.slide_excess:
            if self._yex_s <= 0.0 or abs(a_x) > abs(self._yex_ax):
                self._yex_ax = a_x
            self._yex_s += dt
            if self._yex_s >= cfg.slide_excess_s:
                self._yex_s = 0.0
                self._slow(t, v)
                self._guard(t, x, y, "リアの滑り", a, self._yex, along=self._yex_ax)
        else:
            self._yex_s = max(0.0, self._yex_s - dt)
        # 2. 線から外れた
        c = abs(self._cross)
        if c > cfg.cross_thr and a > cfg.cross_use * self.map.level and self._slow(t, v):
            self._guard(t, x, y, "横偏差", a, c)
        # 3. のために、コーナーの中（計画が横の上限の半分以上を使う所）の横偏差の、この周の場所ごとの最大を持つ
        if self._u >= 0.5:
            k = _cell(x, y)
            if c >= self._cx_lap.get(k, (0.0,))[0]:
                self._cx_lap[k] = (c, x, y, a)

    def _slow(self, t: float, v: float) -> bool:
        """目標速度をすぐ落とす（出ている横加速度の 95% で回れる速度へ）。もう落としている途中なら `False`。"""
        cap = v * math.sqrt(0.95) * (1.0 - self.cfg.cap_margin)
        self._cap_t = t
        if self.cap is not None:
            self.cap = min(self.cap, cap)
            return False
        self.cap = cap
        return True

    # ── 周のタイム ──

    def _time_lap(self, t: float, x: float, y: float, v: float, gate: str) -> None:
        """ゲート線をまたいだ時刻を補間して、1 周の時間を出す。"""
        prev, line = self._gate_prev, self._gate_line
        if line is None:
            self._gate_prev = (t, x, y, 0.0)
            if prev is None or gate or v < 0.8:
                return
            dx, dy = x - prev[1], y - prev[2]
            n = math.hypot(dx, dy)
            if n < 1e-3:
                return
            # 走り出して最初に学習が見ている場所。進む向きに直交する線
            self._gate_line = (x, y, dx / n, dy / n)
            self._lap_t0, self._lap_ok = t, True
            self._plan_sum, self._plan_n = [0.0, 0.0], [0, 0]
            self._cx_lap = {}
            return
        x0, y0, nx, ny = line
        d = (x - x0) * nx + (y - y0) * ny
        self._gate_prev = (t, x, y, d)
        if prev is None or not (prev[3] < 0.0 <= d) or d - prev[3] > 0.5:
            return
        if abs(-(x - x0) * ny + (y - y0) * nx) > 0.6:
            return                              # 線の延長を、コースの別の場所でまたいだ
        tc = prev[0] + (t - prev[0]) * (-prev[3]) / max(d - prev[3], 1e-9)
        lap = tc - self._lap_t0
        if lap < self.cfg.time_lap_min_s:
            return
        ok, (sp, sr), (n, nr) = self._lap_ok, self._plan_sum, self._plan_n
        plan = sp / n if n else 0.0
        # 横だけ戻した計画は、周のほとんどで渡されていたときだけ使う（途中で段が変わった周は無し）
        plan_ref = sr / nr if nr and nr >= 0.8 * n else 0.0
        self._lap_t0, self._lap_ok = tc, True
        self._plan_sum, self._plan_n = [0.0, 0.0], [0, 0]
        self.long.lap(tc)
        cx, self._cx_lap = self._cx_lap, {}
        if ok and not self._lap_cross(tc, cx) and plan > 0.0:
            self.last_lap = (lap, plan)
            self._lap_done(tc, lap, plan, plan_ref)

    def _lap_cross(self, t: float, lap: dict) -> bool:
        """安全装置 3: この周のコーナーの横偏差を、その場所の普段と比べる。働いたら `True`。働かなければ普段の値に入れる。"""
        cfg = self.cfg
        gain, at = 0.0, None
        for k, rec in lap.items():
            b = self._cx.get(k)
            if b is not None and b[1] >= 2 and rec[0] - b[0] > gain:
                gain, at = rec[0] - b[0], k
        self.last_cross_gain = gain
        # 探している間だけ見る。収束した後は、線から大きく外れる・リアが滑るといったはっきりした出来事でだけ下げる
        # （1 回の「増え」で 5% ずつ下げていくと、測り違いのたびに遅くなる）
        if at is not None and gain > cfg.cross_gain and not self.converged:
            _, x, y, a = lap[at]
            self._guard(t, x, y, "横偏差の増え", a, gain, soft=True)
            return True
        for k, rec in lap.items():
            b = self._cx.setdefault(k, [0.0, 0.0])
            b[1] += 1.0
            b[0] += (rec[0] - b[0]) / min(b[1], 5.0)
        return False

    def _lap_done(self, t: float, lap: float, plan: float, plan_ref: float = 0.0) -> None:
        cfg, m = self.cfg, self.map
        self._measured = True
        if self.converged:
            return
        ref = self._ref
        if ref is None or m.level <= ref[0] * (1.0 + 1e-6):
            # この段が基準（初め・下げた後・終える前の測り直し）
            old, self._ref_old = self._ref_old, None
            if old is not None and abs(old[0] - m.level) < 1e-6:
                # 前に測った値と平均する。計画が動いたぶん（減速の学習）は今の計画へ寄せてから
                lap = 0.5 * (lap + old[1] - (old[2] - plan))
            self._ref = (m.level, lap, plan)
            self._raise(t)
            return
        # 横を上げたぶんで縮むはずの時間と、実際に縮んだ時間（加速・減速が変わって縮んだぶんは除く）
        base = plan_ref if plan_ref > 0.0 else ref[2]
        plan_gain = base - plan
        gain = (ref[1] - lap) - (ref[2] - base)
        ok = plan_gain > cfg.time_plan_min * ref[2] and gain >= cfg.time_gain * plan_gain
        self.steps.append({"t": t, "from": ref[0], "to": m.level, "gain": gain, "plan_gain": plan_gain,
                           "ok": ok})
        if ok:
            self._ref = (m.level, lap, plan)
            self._step = min(cfg.time_step, self._step * cfg.time_regrow)
            self._confirmed = False
            self._raise(t)
        else:
            self._reject(t)

    def _raise(self, t: float) -> None:
        """採用した段から 1 段上を試す。"""
        m = self.map
        if self.converged or self.down_only:
            return
        new = self._next(m.level)
        if new <= m.level * (1.0 + 0.5 * self.cfg.time_step_min):
            self._set_converged(t)              # 設定の上限（か、安全装置が働いた値のすぐ下）まで来た
            return
        m.level = new
        self._lat_up_t = t
        self._bump(down=False)

    def _next(self, level: float) -> float:
        """`level` から今の幅で 1 段上（設定の上限と、安全装置が働いた値の手前まで）。"""
        ceil = self.map.ceil if self.map.ceil > 0.0 else math.inf
        return min(self.max_level, self.cfg.time_ceil * ceil, level * (1.0 + self._step))

    def _reject(self, t: float) -> None:
        """試した段をやめる。採用済みの段へ戻し、幅を半分にしてもう一度（幅が尽きたら終わり）。"""
        m, ref, cfg = self.map, self._ref, self.cfg
        if ref is None:
            return
        self._step *= 0.5
        room = self._next(ref[0]) > ref[0] * (1.0 + 0.5 * cfg.time_step_min)
        if self._confirmed or self._step < cfg.time_step_min or not room:
            # 幅が尽きた（か、測り直した後の試し直しも断った・上にもう試せる所が無い）
            m.level = ref[0]
            self._step = cfg.time_step
            if not self._confirmed and self._next(ref[0]) > ref[0] * (1.0 + 0.5 * cfg.time_step_min):
                # 終える前に採用済みの段を測り直し、元の幅で 1 回だけ試し直す
                self._confirmed = True
                self._ref_old, self._ref = ref, None
            else:
                self._set_converged(t)
        else:
            m.level = self._next(ref[0])
            self._lat_up_t = t
        self._bump(down=True)

    def _set_converged(self, t: float) -> None:
        if not self.converged:
            self.converged = True
            self.t_converged = t
            self.ver += 1                       # 値は変わらない（状態の保存だけ進める）

    # ── 安全装置が働いた ──

    def _guard(self, t: float, x: float, y: float, kind: str, a: float, excess: float = 0.0,
               soft: bool = False, along: float | None = None) -> None:
        """安全装置が働いた。**最後に上げたものを疑う**（モジュール docstring）: 横の段を試している最中ならその段をやめ、
        横を上げた後に前後の包絡を上げていたらその包絡を戻す。どちらでもなければ横を少し下げて測り直す。

        :param soft: 「横偏差の増え」。1 回ではその上限を天井にしない。近い高さ（1 段ぶん上まで）で 2 回目が出たらする
        :param along: リアが滑り始めたときに計画が求めていた前後の加速度 [m/s²]。あれば、疑う前後の包絡をその向きの側だけに
            絞り、どちらでもない（`slide_long` 未満）なら横を疑う。`None` = 分からない（線から外れた・横偏差の増え）
        """
        cfg, m = self.cfg, self.map
        self._lap_ok = False
        self.events.append({"t": t, "x": x, "y": y, "a": a, "kind": kind, "applied": m.level, "excess": excess,
                            "along": along})
        if t - self._guard_t < cfg.time_guard_quiet_s:
            return                              # 同じ出来事の続き
        self._guard_t = t
        ref = self._ref
        trying = ref is not None and m.level > ref[0] * (1.0 + 1e-6) and not self.converged
        lg = self.long
        side = None if along is None else lg.acc if along >= cfg.slide_long \
            else lg.dec if along <= -cfg.slide_long else "lat"
        if not trying and not self.down_only and side != "lat" and lg.raise_t(side) > self._lat_up_t \
                and lg.undo_last(t, kind, side):
            # 包絡が変わると前のタイム・計画は比べられない。採用済みの段を測り直す（収束していれば、包絡が変わった
            # ぶん横を上げられるかを探し直す）
            self._ref = None
            self._step = max(cfg.time_step_min, 0.5 * self._step)
            self.converged = False
            self._bump(down=True)
            return
        if not soft:
            m.ceil = min(m.ceil, m.level) if m.ceil > 0.0 else m.level
        else:
            near = [lv for lv in self._soft if lv <= m.level * (1.0 + 1.3 * cfg.time_step)]
            if near:
                c = max(m.level, min(near))
                m.ceil = min(m.ceil, c) if m.ceil > 0.0 else c
            self._soft.append(m.level)
        if trying:
            self._reject(t)
            return
        # 採用済みの段（か、まだ基準の無い段）で出た。下げて、そこを測り直す。上げ直すのは小さい幅で
        m.level = max(self.floor, m.level * (1.0 - cfg.time_back))
        self._ref = None
        self._step = max(cfg.time_step_min, 0.5 * self._step)
        self.converged = False
        self._bump(down=True)

    def _bump(self, down: bool = False) -> None:
        """速度プロファイルに効く値が変わった。上げただけの変更は、車の足元のプロファイルを
        変えない（planner の `_grip_hold`）ので、加速・減速の測定は止めない。"""
        self.ver += 1
        if down:
            self.ver_down += 1
            self.long.note_change()

    # ── 表示 ──

    def diag(self) -> dict:
        """記録に残す診断（`AutoState.grip` に足す。値は float だけ）。"""
        ref = self._ref or (0.0, 0.0, 0.0)
        return {"t_lap": round(float(self.last_lap[0]), 3), "t_plan": round(float(self.last_lap[1]), 3),
                "ref_lat": round(float(ref[0]), 3), "ref_lap": round(float(ref[1]), 3),
                "ref_plan": round(float(ref[2]), 3), "step": round(float(self._step), 4),
                "steps": float(len(self.steps)), "cross_gain": round(float(self.last_cross_gain), 4),
                "ceil": round(float(self.map.ceil), 3)}

    def state(self, gate: str = "") -> str:
        if gate:
            return f"停止: {gate}"
        if self.cap is not None:
            return "★安全装置で減速"
        if self.down_only:
            return "下げるだけ"
        if self.converged:
            return "収束"
        return f"タイムで探索 +{self._step * 100:.0f}%"
