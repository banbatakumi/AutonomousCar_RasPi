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

import msgspec

from ..msgs.types import AutoState, Scan, VehicleState

__all__ = ["ParamSpec", "Planner", "sector_of_deg", "wrap_deg"]


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


class Planner:
    """自動運転アルゴリズムの基底。**サブクラスは4つの属性と2つのメソッドだけ。**

    登録は `raspi/auto/registry.py` の `PLANNERS` に1行足すだけでよい。
    """

    #: バス・GUI・設定ファイルを貫く識別子。**変えると保存済みの設定が外れる**
    id: str = ""
    name: str = ""                         #: GUI のボタンに出る名前
    description: str = ""                  #: 1行の説明。GUI の選択肢の下に出る
    params: tuple[ParamSpec, ...] = ()

    def reset(self) -> None:
        """内部状態を捨てる。**モード切替・disengage のたびに呼ばれる。**"""

    def plan(self, scan: Scan, vs: VehicleState | None,
             p: dict[str, float], dt: float) -> AutoState:
        """1周期ぶんの判断。

        :param scan: 最新の点群。**`sector_seen` が False の区間は「障害物なし」ではない**
        :param vs: 最新の車両状態。まだ届いていなければ None
        :param p: `params` の既定値に GUI の設定を重ねたもの。**全キーが必ず入っている**
        :param dt: 前回の `plan()` からの経過 [s]。平滑化に使う
        """
        raise NotImplementedError

    # ── ヘルパ ──

    @classmethod
    def defaults(cls) -> dict[str, float]:
        return {s.key: s.default for s in cls.params}

    @classmethod
    def merged(cls, user: dict[str, float]) -> dict[str, float]:
        """既定値にユーザー設定を重ね、**宣言した範囲へクランプする**。

        GUI を信用してクランプを省くと、古い `config/auto.json` や手書きの値が
        そのまま planner に入る。範囲外の値で走り出す方が、無視されるより危ない。
        """
        out = cls.defaults()
        for s in cls.params:
            v = user.get(s.key)
            if v is None:
                continue
            try:
                v = float(v)
            except (TypeError, ValueError):
                continue
            if v != v:                     # NaN
                continue
            out[s.key] = min(s.max, max(s.min, v))
        return out


def wrap_deg(deg: float) -> float:
    """0〜359 の車両角を **±180 の符号付き**に直す。

    前方を跨ぐギャップ（例 350°〜10°）を `start <= end` で表せるようにするため。
    0〜359 のままだと「右端 350、左端 10」となり、大小関係が反転して読めない。
    """
    d = (deg + 180.0) % 360.0 - 180.0
    return d


def sector_of_deg(deg: int) -> int:
    """車両角 [deg] → `Scan.sector_seen` の添字。

    **境界が 1 つずれている。** セクタ `s` が持つのは `dist[30*s+1]` 〜
    `dist[(30*s+30) % 360]` で、`30*s` ちょうどの点は隣のセクタ由来
    （`Scan` の docstring）。素直に `deg // 30` と書くと、セクタ欠損の判定が
    30 点ぶん隣にずれ、**欠測している方向を「見えている」と誤判定する**。
    """
    return ((deg - 1) % 360) // 30
