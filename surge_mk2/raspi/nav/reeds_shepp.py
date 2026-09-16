"""Reeds-Sheppパス — 前進・後退どちらも使える車両の最短経路（車体非依存）。

Reeds, J.A., Shepp, L.A., "Optimal paths for a car that goes both forwards
and backwards", Pacific J. Math., 1990。非ホロノミック拘束（最小旋回半径`R`）
の下で、任意の始点姿勢から終点姿勢へ、**前進・後退のセグメント列**として
必ず到達可能な経路を解析的に求める。`park_to_point.py`の単一フィードバック
制御（Aicardi/Astolfi）では、車両の向きと目標の向きが揃った後に位置だけが
横方向にずれている配置で位置が全く縮まらない（非ホロノミック拘束下では
その場での真横移動が物理的に不可能なため）ことを実測で確認しており、
これを解くには本モジュールのような明示的な経路計画が必要になる。

## 実装方針: 9つの語族 + time-flip/reflect変換 ＝ 原論文の48語

Reeds-Sheppの原論文は48通りの経路（語, word）を列挙するが、少数の「語族」に
対称性変換を掛けることで全パターンを覆える:

- **reflect（左右反転）**: 目標の`y`と`φ`の符号を反転して式に渡すと、
  L(左旋回)とR(右旋回)を入れ替えた経路が得られる
- **time-flip（前後反転）**: 目標の`x`と`φ`の符号を反転して式に渡し、
  得た語の長さを全て反転すると、前進と後退を入れ替えた経路が得られる

実装した語族は9つ:

| 語族 | 形 | 何に効くか |
|---|---|---|
| CSC | L+S+L+ / L+S+R+ | 直線を挟む素直な接近 |
| CCC | L+R-L+ / L+R-L- | 狭い場所での方向転換（中間区間だけ逆走行） |
| CCCC | L+R+L-R- / L+R-L-R+ | さらに狭い場所での方向転換（4区間） |
| CCSC | L+R-S-L- / L+R-S-R- | **後退の直線を挟む。車庫入れで頻出** |
| CCSCC | L+R-S-L-R+ | 5区間。原論文で最も長い形 |

当初はCSC+CCCの**6語だけ**の実装だった。9語族へ拡張した効果（2026-09-16実測、
ランダム400配置）:

- **ユニークな候補数 6.5本 → 13.8本**（障害物回避で選べる選択肢が倍増）
- 最短経路長が **206/400 の配置で短縮**（中央値0.1%・最大87.6%）

**実装の正しさは公式の暗記ではなく検証で担保する**——生成した経路を実際に
`integrate_path()`で積分し、始点から終点へ到達するかを単体テストで必ず確認
すること（`raspi/tests/test_reeds_shepp.py`）。★到達性の検証だけでは
**「1件も生成されない語族」に気付けない**（0件は失敗として現れない）ので、
語族ごとの生成数も別に数える（`_wrap_pi`のdocstring参照）。

## 使い方

```python
path = shortest_path((0, 0, 0), (1.0, 0.5, math.radians(90)), turning_radius=0.4)
for seg in path.segments:
    ...  # seg.gear（+1前進/-1後退）、seg.curvature（0=直進、±1/R=左右）、seg.length
```

`turning_radius`は`wheelbase / tan(max_steer)`（最小旋回半径）を渡す想定。

## 障害物回避での使い方

`shortest_path()`は最短の1本しか返さないが、`candidate_paths()`は長さ昇順の
候補リストを返す。`park_to_point.py`はこれを2通りに使う:

1. **Hybrid A*の解析展開**（`raspi/nav/hybrid_astar.py`）: 探索中のノードから
   目標へRS経路を撃ち、衝突しなければ即接続する。探索が終盤で発散しない
2. **素の候補選択**: Hybrid A*を使わない設定（`hybrid_enabled=0`）では、
   最短から順に衝突しない候補を選ぶ

衝突判定はこのモジュールには持たない——`raspi/nav/local_map.py`の
`LocalMap.path_clearance()`（ESDF＋円被覆）が受け持つ。以前は点群に対する
掃引（`path_collides()`、軸並行矩形の近似）も持っていたが、ESDF方式に
置き換わって使われなくなったので撤去した（2026-09-17）。

"""

from __future__ import annotations

import math
from typing import NamedTuple

__all__ = ["PathSegment", "ReedsSheppPath", "shortest_path", "candidate_paths",
          "sample_path", "sample_path_array", "integrate_path"]

_EPS = 1e-9


class PathSegment(NamedTuple):
    """1区間ぶん。**符号付きの弧長/直線距離**（`turning_radius`で実スケール済み）。"""

    gear: int          #: +1=前進、-1=後退
    curvature: float   #: [1/m]。0=直進、正=左旋回、負=右旋回
    length: float      #: [m]。**常に非負**（向きは`gear`が持つ）


class ReedsSheppPath(NamedTuple):
    segments: tuple[PathSegment, ...]

    @property
    def length(self) -> float:
        return sum(s.length for s in self.segments)


def _mod2pi(theta: float) -> float:
    """角度を [0, 2π) へ畳む。

    ★ **旋回角の畳み込みは `(-π,π]` ではなく `[0,2π)` でなければならない。**
    `t`/`u`/`v` は「実際に走行する弧の角度」で必ず非負だが、本来
    `[π, 2π)` にある正しい解を `(-π,π]` に畳むと符号が反転し、有効な解が
    「負」に見えて誤って無効判定される（数値的に`fsolve`で正解
    `t≈4.62rad`を求めて発見・修正した不具合）。
    """
    v = theta % (2.0 * math.pi)
    if v < 0.0:
        v += 2.0 * math.pi
    return v


def _wrap_pi(theta: float) -> float:
    """角度を `(-π, π]` へ畳む。

    ★ **`_mod2pi`との使い分けが正しさの分かれ目。** 語（word）の区間長は
    **符号がそのまま前後進**を表す:

    - 条件が「非負」の量（前進と分かっている弧）は `_mod2pi`（`[0,2π)`）。
      `(-π,π]` に畳むと `[π,2π)` にある正しい解が負に見えて誤って無効判定される
      （`_mod2pi`のdocstringの不具合）
    - 条件が「非正」の量（後退と分かっている弧）は **こちら**。`[0,2π)` に
      畳むと負の値が表現できず、**その語が1件も生成されなくなる**
      （CCCC/CCSC系を実装した当初、条件`v<=0`に`_mod2pi`を使って全滅させた。
      `integrate_path`による到達性検証では「0件」は失敗として現れないので、
      **候補数そのものを数えて気付いた**）
    """
    return (theta + math.pi) % (2.0 * math.pi) - math.pi


def _polar(x: float, y: float) -> tuple[float, float]:
    return math.hypot(x, y), math.atan2(y, x)


def _tau_omega(u: float, v: float, xi: float, eta: float, phi: float, *,
               signed_omega: bool = False) -> tuple[float, float]:
    """CCCC系が共有する補助量。4区間の語で両端の弧長を一度に決める。"""
    delta = _mod2pi(u - v)
    a = math.sin(u) - math.sin(delta)
    b = math.cos(u) - math.cos(delta) - 1.0
    t1 = math.atan2(eta * a - xi * b, xi * a + eta * b)
    t2 = 2.0 * (math.cos(delta) - math.cos(v) - math.cos(u)) + 3.0
    tau = _mod2pi(t1 + math.pi) if t2 < 0 else _mod2pi(t1)
    omega = (_wrap_pi if signed_omega else _mod2pi)(tau - u + v - phi)
    return tau, omega


# ── 語（word）の生成器 ────────────────────────────────────────────────
#
# すべて「旋回半径1・始点(0,0,0)」に正規化した局所座標での式。戻り値は
# `[(文字, 符号付き長さ), ...]`（生成できなければ `None`）。**長さの符号が
# そのまま前後進**（正=前進、負=後退）で、曲線区間は弧の角度[rad]、直線区間は
# 距離。文字は L=左旋回・R=右旋回・S=直進。
#
# 9つの語族すべてについて、生成した経路を`integrate_path()`で実際に積分して
# 目標へ到達することを確認済み（`raspi/tests/test_reeds_shepp.py`。2026-09-16の
# 実装時に600配置×全対称変換=約9000件で到達率100%を確認）。

_Word = list[tuple[str, float]]


def _lsl(x: float, y: float, phi: float) -> _Word | None:
    """CSC: L+ S+ L+"""
    u, t = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if t >= -_EPS:
        v = _mod2pi(phi - t)
        if v >= -_EPS:
            return [("L", t), ("S", u), ("L", v)]
    return None


def _lsr(x: float, y: float, phi: float) -> _Word | None:
    """CSC: L+ S+ R+"""
    u1, t1 = _polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    if u1 * u1 >= 4.0:
        u = math.sqrt(u1 * u1 - 4.0)
        t = _mod2pi(t1 + math.atan2(2.0, u))
        v = _mod2pi(t - phi)
        if t >= -_EPS and v >= -_EPS:
            return [("L", t), ("S", u), ("R", v)]
    return None


def _lrl_fwd(x: float, y: float, phi: float) -> _Word | None:
    """CCC: L+ R- L+（最終区間は前進）"""
    xi, eta = x - math.sin(phi), y - 1.0 + math.cos(phi)
    u1, theta = _polar(xi, eta)
    if u1 <= 4.0:
        a = math.acos(0.25 * u1)
        t = _mod2pi(theta + 0.5 * math.pi + a)
        u = _mod2pi(math.pi - 2.0 * a)
        v = _mod2pi(phi - t - u)
        if t >= -_EPS and u >= -_EPS:
            return [("L", t), ("R", -u), ("L", v)]
    return None


def _lrl_back(x: float, y: float, phi: float) -> _Word | None:
    """CCC: L+ R- L-（最終区間も後退）。`_lrl_fwd`と別解になる配置がある。"""
    xi, eta = x - math.sin(phi), y - 1.0 + math.cos(phi)
    u1, theta = _polar(xi, eta)
    if u1 <= 4.0:
        a = math.acos(0.25 * u1)
        t = _mod2pi(theta + 0.5 * math.pi + a)
        u = _mod2pi(math.pi - 2.0 * a)
        v = _mod2pi(t + u - phi)
        if t >= -_EPS and u >= -_EPS and v >= -_EPS:
            return [("L", t), ("R", -u), ("L", -v)]
    return None


def _lrlr_a(x: float, y: float, phi: float) -> _Word | None:
    """CCCC: L+ R+ L- R-（4区間。狭い場所での方向転換に効く）"""
    xi, eta = x + math.sin(phi), y - 1.0 - math.cos(phi)
    rho = (2.0 + math.hypot(xi, eta)) / 4.0
    if rho <= 1.0:
        u = math.acos(rho)
        t, v = _tau_omega(u, -u, xi, eta, phi, signed_omega=True)
        if t >= -_EPS and v <= _EPS:
            return [("L", t), ("R", u), ("L", -u), ("R", v)]
    return None


def _lrlr_b(x: float, y: float, phi: float) -> _Word | None:
    """CCCC: L+ R- L- R+"""
    xi, eta = x + math.sin(phi), y - 1.0 - math.cos(phi)
    rho = (20.0 - xi * xi - eta * eta) / 16.0
    if 0.0 <= rho <= 1.0:
        u = -math.acos(rho)
        if u >= -0.5 * math.pi:
            t, v = _tau_omega(u, u, xi, eta, phi)
            if t >= -_EPS and v >= -_EPS:
                return [("L", t), ("R", u), ("L", u), ("R", v)]
    return None


def _lrsl(x: float, y: float, phi: float) -> _Word | None:
    """CCSC: L+ R- S- L-（後退の直線を挟む。車庫入れで頻出する形）"""
    xi, eta = x - math.sin(phi), y - 1.0 + math.cos(phi)
    rho, theta = _polar(xi, eta)
    if rho >= 2.0:
        r = math.sqrt(rho * rho - 4.0)
        u = 2.0 - r
        t = _mod2pi(theta + math.atan2(r, -2.0))
        v = _wrap_pi(phi - 0.5 * math.pi - t)
        if t >= -_EPS and u <= _EPS and v <= _EPS:
            return [("L", t), ("R", -0.5 * math.pi), ("S", u), ("L", v)]
    return None


def _lrsr(x: float, y: float, phi: float) -> _Word | None:
    """CCSC: L+ R- S- R-"""
    xi, eta = x + math.sin(phi), y - 1.0 - math.cos(phi)
    rho, theta = _polar(-eta, xi)
    if rho >= 2.0:
        t = theta
        u = 2.0 - rho
        v = _wrap_pi(t + 0.5 * math.pi - phi)
        if t >= -_EPS and u <= _EPS and v <= _EPS:
            return [("L", t), ("R", -0.5 * math.pi), ("S", u), ("R", v)]
    return None


def _lrslr(x: float, y: float, phi: float) -> _Word | None:
    """CCSCC: L+ R- S- L- R+（5区間。原論文48語のうち最も長い形）"""
    xi, eta = x + math.sin(phi), y - 1.0 - math.cos(phi)
    rho, _ = _polar(xi, eta)
    if rho >= 2.0:
        u = 4.0 - math.sqrt(rho * rho - 4.0)
        if u <= _EPS:
            t = _mod2pi(math.atan2((4.0 - u) * xi - 2.0 * eta,
                                   -2.0 * xi + (u - 4.0) * eta))
            v = _mod2pi(t - phi)
            if t >= -_EPS and v >= -_EPS:
                return [("L", t), ("R", -0.5 * math.pi), ("S", u),
                        ("L", -0.5 * math.pi), ("R", v)]
    return None


#: 9つの語族。原論文の48語は、これらに time-flip / reflect の対称変換を
#: 掛けて得られる（下の`_candidates`）
_GENERATORS = (_lsl, _lsr, _lrl_fwd, _lrl_back, _lrlr_a, _lrlr_b,
               _lrsl, _lrsr, _lrslr)

_SWAP_LR = str.maketrans("LR", "RL")


def _build(word: _Word, turning_radius: float) -> ReedsSheppPath:
    """語を実スケールの区間列へ。**長さの符号が前後進、絶対値が弧長/距離。**

    曲線区間の「長さ」は角度[rad]、直線区間は半径1基準の距離だが、
    **どちらも同じ係数`turning_radius`で実距離に戻る**（正規化座標系なので）。
    """
    curv = {"L": 1.0 / turning_radius, "R": -1.0 / turning_radius, "S": 0.0}
    segs = tuple(
        PathSegment(gear=(1 if length > 0.0 else -1), curvature=curv[letter],
                    length=abs(length) * turning_radius)
        for letter, length in word
        if abs(length) > _EPS
    )
    return ReedsSheppPath(segments=segs)


def _candidates(x: float, y: float, phi: float,
                turning_radius: float) -> list[ReedsSheppPath]:
    """正規化座標`(x,y,phi)`に対する候補経路（重複を含む）。

    4つの対称変換で語族を展開する:

    - **そのまま**
    - **time-flip**（前後反転）: `(-x, y, -phi)`を式に渡し、得た語の長さを
      全て反転する。★`x`だけ反転して`phi`を忘れると、位置は合うがyawの符号が
      逆転する（`fsolve`との数値比較で発見した不具合）
    - **reflect**（左右反転）: `(x, -y, -phi)`を渡し、L↔Rを入れ替える
    - **両方**: `(-x, -y, phi)`を渡し、長さを反転しL↔Rを入れ替える
    """
    out: list[ReedsSheppPath] = []
    for gen in _GENERATORS:
        for args, negate, swap in (
            ((x, y, phi), False, False),
            ((-x, y, -phi), True, False),
            ((x, -y, -phi), False, True),
            ((-x, -y, phi), True, True),
        ):
            word = gen(*args)
            if word is None:
                continue
            if negate:
                word = [(c, -length) for c, length in word]
            if swap:
                word = [(c.translate(_SWAP_LR), length) for c, length in word]
            path = _build(word, turning_radius)
            if path.segments:
                out.append(path)
    return out


def candidate_paths(start: tuple[float, float, float], goal: tuple[float, float, float],
                    turning_radius: float) -> list[ReedsSheppPath]:
    """`start`から`goal`への候補経路を**すべて**、長さの昇順で返す。

    `shortest_path()`が最短の1本だけを返すのに対し、こちらは
    `park_to_point.py`が「最短から順に、車両フットプリントが障害物と
    衝突しない最初の候補を選ぶ」という使い方をするために公開してある。
    候補が1つも無い場合（数値的に稀）は空リスト。
    """
    sx, sy, syaw = start
    gx, gy, gyaw = goal
    dx, dy = gx - sx, gy - sy
    if math.hypot(dx, dy) < 1e-9 and abs(_mod2pi(gyaw - syaw)) < 1e-9:
        return [ReedsSheppPath(segments=())]   # 既に目標姿勢。動く必要がない
    c, s = math.cos(syaw), math.sin(syaw)
    # start を原点・yaw=0 とする局所座標へ変換し、turning_radius で正規化
    lx = (c * dx + s * dy) / turning_radius
    ly = (-s * dx + c * dy) / turning_radius
    lphi = _mod2pi(gyaw - syaw)

    raw = [p for p in _candidates(lx, ly, lphi, turning_radius=1.0) if p.segments]
    raw.sort(key=lambda p: p.length)
    # 正規化(半径1)で求めた長さを実スケールへ戻す
    return [
        ReedsSheppPath(segments=tuple(
            PathSegment(s.gear, s.curvature / turning_radius, s.length * turning_radius)
            for s in p.segments))
        for p in raw
    ]


def shortest_path(start: tuple[float, float, float], goal: tuple[float, float, float],
                  turning_radius: float) -> ReedsSheppPath:
    """`start`から`goal`への最短経路。座標は共通の平面座標系、姿勢は`(x,y,yaw)`。

    `turning_radius`は正の値（最小旋回半径 `wheelbase/tan(max_steer)`）。
    候補が1つも見つからない場合（数値的に稀）は空の`ReedsSheppPath`を返す
    ——呼び出し側は`length==0`かつ`start!=goal`ならこれを検知できる。
    """
    candidates = candidate_paths(start, goal, turning_radius)
    return candidates[0] if candidates else ReedsSheppPath(segments=())


def sample_path_array(start: tuple[float, float, float], path: ReedsSheppPath,
                      step: float = 0.05) -> "np.ndarray":
    """経路に沿った姿勢列を `(N, 3)` の配列で返す（`start`を含む）。

    ★ **区間ごとにnumpyでまとめて作る。** 点ごとにPythonで`sin`/`cos`を
    呼ぶ実装だと、Hybrid A*の解析展開（1回の探索で数万回サンプルする）が
    探索時間の8割を食う（2026-09-16実測で12秒のうち10秒）。
    """
    import numpy as np

    x, y, yaw = start
    chunks = [np.array([[x, y, yaw]], dtype=np.float64)]
    for seg in path.segments:
        n = max(1, int(math.ceil(seg.length / step)))
        ds = (seg.length / n) * seg.gear
        s_arr = ds * np.arange(1, n + 1, dtype=np.float64)
        if abs(seg.curvature) < 1e-9:
            xs = x + s_arr * math.cos(yaw)
            ys = y + s_arr * math.sin(yaw)
            yaws = np.full(n, yaw)
        else:
            dyaw = s_arr * seg.curvature
            r = 1.0 / seg.curvature
            xs = x + r * (np.sin(yaw + dyaw) - math.sin(yaw))
            ys = y - r * (np.cos(yaw + dyaw) - math.cos(yaw))
            yaws = yaw + dyaw
        chunks.append(np.column_stack((xs, ys, yaws)))
        x, y, yaw = float(xs[-1]), float(ys[-1]), float(yaws[-1])
    return np.vstack(chunks)


def sample_path(start: tuple[float, float, float], path: ReedsSheppPath,
                step: float = 0.05) -> list[tuple[float, float, float]]:
    """経路に沿って一定間隔でサンプリングした姿勢列（`start`を含む）を返す。

    衝突判定（`local_map.LocalMap.path_clearance`）と、単体テストでの
    到達性検証（`integrate_path`は本関数の最終要素）の両方に使う。
    配列が要る呼び出し側は`sample_path_array()`を直接使うこと。
    """
    return [(float(a), float(b), float(c))
            for a, b, c in sample_path_array(start, path, step)]


def integrate_path(start: tuple[float, float, float], path: ReedsSheppPath,
                   step: float = 0.02) -> tuple[float, float, float]:
    """経路を実際に積分し、最終姿勢を返す。**単体テストでの検証専用**
    （経路生成の公式が正しいかを、暗記ではなく実際の到達点で確認するため）。
    """
    return sample_path(start, path, step)[-1]
