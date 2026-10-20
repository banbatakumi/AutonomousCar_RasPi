"""校正 8: TC・ABS のゲイン（交差周波数と、PI の P と I の比）を実機の閉ループで決める。解析は `tools/calib/tc_gain.py`、
設計は `docs/calibration.md` §4.8。

## なぜ設計しないか

スリップ速度 u は、トルクに対してほぼ積分器 `u' = (r/J)·T` で、その先に MD の遅れ（校正 6）がある。位相余裕の式で kp・ki を
決めたゲインは、実機で高すぎて振動した（2026-10-19）。TC・ABS を切ったランプでも周速はトルクを掛けると揺れる（制動側 0.03m/s、
定速 0.004）ので、揺れの一部は制御器の外にあり、式の前提（滑らかなプラント）が成り立たない。そこで、**交差周波数 ωc を低い方から
上げて**、全開の発進（TC）と最大の制動（ABS）を実走し、発振する手前を採る。

## 基準のゲイン（toml のゲインは使わない）

積分器プラントでは、交差周波数 ωc を決めれば kp が決まる:

    kp = ωc·J/r,   ki = kp·ωc/ratio       （PI の零点を交差の 1/ratio に置く。ratio = 4 が基準）

J は校正 6 の後輪の慣性（`[control.plant]`）、r は後輪の半径。現行の kp 0.097 は ωc ≈ 80rad/s に当たる。

## 手順（前進の片道 = 全開の発進 → 最大の制動、を 7 回）

- 片道 0〜4: ωc = `LADDER`（15・30・60・120・240rad/s、比 4）。TC は発進の区間、ABS は制動の区間で使う
- 発振（揺れが大きく、一つの周波数に集まる）を見つけた側は、**そのあとの片道では発振した一段手前（ゲイン余裕 2 倍）に戻す**
  （発振するゲインで走り続けない）。もう一方は上げ続ける
- 片道 5・6: その ωc（一度も発振しなければ梯子の最大の一段手前）で、比を `RATIOS`（2.5・8）に変える

必要なスペース: 直線 2m＋車長。約 60 秒。目標スリップ率は試験の間だけ `TEST_TARGETS`（制御器が働く値）に上書きするので、校正 7 の値に
依らない（基準のゲインに要る校正 6 の値は適用してから録る）。
"""

from __future__ import annotations

import math
import tomllib

import numpy as np

from ..core.control_params import DEFAULT_TOML
from ..msgs.types import VehicleState
from ._calib_common import Step
from .calib_tyre import TractionLegs

__all__ = ["CalibTcGain", "base_gains", "oscillation", "WIN", "WHEEL_RADIUS_M", "KEYS", "OSC_STD_MAX", "OSC_SHARE_MAX", "OSC_FREQ_MIN_HZ", "OSC_GROWTH", "TEST_TARGETS", "jitter_rms", "is_oscillation"]

WHEEL_RADIUS_M = 0.030
#: 発振の判定（`tools/calib/traction.py` の `SLIP_STD_MAX`・`PEAK_SHARE_MAX` と同じ）
OSC_STD_MAX = 0.04
OSC_SHARE_MAX = 0.5
#: 発振とみなす周波数の下限 [Hz]。それより遅い揺れはゲインが低くて目標に保てていない（空転が広がる）だけで、発振ではない
OSC_FREQ_MIN_HZ = 12.0
#: 発振の判定は「一つの周波数に集まる」だけでは足りない: 実機は TC・ABS のゲインによらず 20〜30Hz の揺れ（タイヤ・MD の性質）が常にあり
#: （TC・ABS を切ったランプでも制動側は 0.03m/s）、ゲインを上げても増えなかった。ゲインを上げて**揺れ（高周波の大きさ）が
#: 手前の片道の最小の `OSC_GROWTH` 倍を超え**、かつ一つの周波数に集まったときだけ発振とみなす
OSC_GROWTH = 1.6
_CHUNK = 12                  #: 揺れの標準偏差を取る区切り [サンプル]（100Hz で 0.12 秒）
WIN = 24                     #: 周波数の集中度を見る窓 [サンプル]（最大の制動は 0.5 秒足らずで、32 では窓が取れない）
_SKIP = 8                   #: 区間の頭で捨てるサンプル（始まりの一瞬の空転・ロックは別に見る）
_MIN_V = 0.6
#: 試験の間だけ使う目標スリップ率。ゲインを測るには制御器が**働いて**目標に保つ場面が要る。校正 7 の目標は「最大トルクまで滑らせる」値に
#: なりうる（この路面では TC・ABS がほぼ働かない）ので、ここでは働く低めの値に固定する（全開の発進のスリップ率は 0.16 前後、
#: 最大の制動は 0.2〜0.5 まで上がる）。ゲインは目標の値にあまり依らない（スリップ速度 → トルクの積分器が主）
TEST_TARGETS = {"tc_slip_target": 0.09, "abs_slip_target": 0.18}
#: 設定の名前（TC・ABS）
KEYS = {"tc": ("tc_kp_nm_per_m_s", "tc_ki_nm_per_m"), "abs": ("abs_kp_nm_per_m_s", "abs_ki_nm_per_m")}


def base_gains(wc: float, ratio: float, inertia_kgm2: float) -> tuple[float, float]:
    """交差周波数 `wc` [rad/s]・PI の零点の比 `ratio`（零点 = wc/ratio）から (kp, ki)。スリップ速度が
    トルクに対して積分器 (r/J)/s とみなせるとして、|L(jωc)| = 1 から。"""
    kp = wc * inertia_kgm2 / WHEEL_RADIUS_M
    return kp, kp * wc / ratio


def oscillation(series) -> tuple[float, float, float]:
    """スリップ率の並び（100Hz）から (区間内の揺れの標準偏差の最大, 一番強い周波数への集中度, その周波数 [Hz])。
    解析（`tools/calib/traction.py`）と同じ見方: 揺れが大きく、一つの周波数に集まり、その周波数が `OSC_FREQ_MIN_HZ`
    以上なら発振。"""
    x = np.asarray(series, dtype=float)
    std = max((float(np.std(x[i:i + _CHUNK])) for i in range(0, len(x) - _CHUNK + 1, _CHUNK)), default=0.0)
    acc = None
    for j in range(0, len(x) - WIN + 1, WIN):
        w = x[j:j + WIN]
        w = w - np.polyval(np.polyfit(np.arange(WIN), w, 1), np.arange(WIN))
        P = np.abs(np.fft.rfft(w * np.hanning(WIN))) ** 2
        acc = P if acc is None else acc + P
    if acc is None or acc[1:].sum() <= 0:
        return std, 0.0, 0.0
    P = acc[1:]
    k = int(np.argmax(P))
    share = float(P[max(0, k - 1):k + 2].sum() - min(P[max(0, k - 1)], P[min(len(P) - 1, k + 1)])) / float(P.sum())
    return std, share, float(np.fft.rfftfreq(WIN, 0.01)[k + 1])


def jitter_rms(series, k: int = 5) -> float:
    """スリップ率の高周波の大きさ: 並びから k 点の移動平均（遅い変化）を引いた残りの二乗平均平方根。
    目標への追従の遅い変化（空転の広がり・収束）を含めずに、揺れだけを見る。"""
    x = np.asarray(series, dtype=float)
    if len(x) < 2 * k + 4:
        return float("nan")
    r = (x - np.convolve(x, np.ones(k) / k, mode="same"))[k:-k]
    return float(np.sqrt(np.mean(r ** 2)))


def is_oscillation(std: float, share: float, freq: float, jitter: float, baseline: float | None) -> bool:
    """発振か。揺れが大きく、一つの周波数（`OSC_FREQ_MIN_HZ` 以上）に集まり、手前の片道の最小の揺れの `OSC_GROWTH` 倍を超える。
    手前の片道が無ければ（baseline が None）判定できないので False。"""
    if baseline is None or not math.isfinite(jitter) or not math.isfinite(baseline):
        return False
    return bool(std > OSC_STD_MAX and share > OSC_SHARE_MAX and freq >= OSC_FREQ_MIN_HZ and jitter > OSC_GROWTH * baseline)


def _read_inertia() -> float:
    """後輪の慣性 [kg·m²]（校正 6 の結果）。基準のゲインを作るのに使う。"""
    with open(DEFAULT_TOML, "rb") as f:
        d = tomllib.load(f)
    J = d.get("control", {}).get("plant", {}).get("wheel_inertia_kgm2")
    if not isinstance(J, (int, float)) or not math.isfinite(J) or J <= 0:
        raise ValueError("[control.plant] wheel_inertia_kgm2 が無い（校正 6 を先に適用する）")
    return float(J)


class CalibTcGain(TractionLegs):
    id = "calib_tc_gain"
    name = "校正 8: TC・ABS のゲイン"
    description = ("交差周波数を低い方から上げて（比は固定）発進と制動を実走し、発振する手前のゲインを探す。続けて PI の P と I の"
                   "比も変えて比べる。校正 7 の目標を適用してから。直線2m＋車長。約60秒")

    #: 交差周波数の並び [rad/s]（比 `ZERO_RATIO`）と、採った交差周波数で試す比
    SETTINGS = {**TractionLegs.SETTINGS, "v_top": 2.1}      #: ABS の区間で評価できる時間を取るため少し高く（通路 2m に収まるかは土台の見積もりが見る）
    LADDER = (15.0, 30.0, 60.0, 120.0, 240.0)
    ZERO_RATIO = 4.0
    RATIOS = (2.5, 8.0)

    def brake_decel_guess(self, leg: int) -> float:
        return 1.6

    def __init__(self) -> None:
        self._inertia = _read_inertia()
        super().__init__()

    def _reset_state(self) -> None:
        super()._reset_state()
        self._ov: dict[str, float] = {}
        self._ov_leg = -1
        #: 種類（tc・abs）ごとの、梯子の片道（番号 n）で発振したか。判定できた（サンプルが足りた）片道だけ入る
        self._osc: dict[str, dict[int, bool]] = {"tc": {}, "abs": {}}
        #: 種類ごとの、これまでの片道の揺れ（`jitter_rms`）。発振の判定の基準は、その最小
        self._jit: dict[str, list[float]] = {"tc": [], "abs": []}
        #: 今の片道の、区間（drive・brake）ごとのスリップ率と車速の並び
        self._buf: dict[str, list[float]] = {"drive": [], "brake": []}
        self._buf_key: tuple[int, str] | None = None
        self._skip = 0
        self._used: dict[str, float] = {}

    # ── 片道ごとのゲイン ─────────────────────────────────────────────────

    def leg_count(self) -> int:
        return 2 * (len(self.LADDER) + len(self.RATIOS))

    def _first_osc(self, kind: str) -> int | None:
        """発振が見つかった最初の梯子の番号。なければ None。"""
        return min((n for n, o in self._osc[kind].items() if o), default=None)

    def _sel(self, kind: str) -> float:
        """種類 `kind` の、いま採る交差周波数。ゲイン余裕 2 倍: 発振した梯子の一段手前（梯子の刻みは 2 倍）。一度も発振しなければ
        梯子の最大の一段手前（最大は 2 倍の側が試されていない）。最初の段で発振したら、その半分。"""
        n = self._first_osc(kind)
        if n is None:
            return self.LADDER[-2]
        return self.LADDER[n - 1] if n > 0 else self.LADDER[0] / 2

    def _make_overrides(self, n: int) -> dict[str, float]:
        out: dict[str, float] = dict(TEST_TARGETS)
        self._used = {}
        for kind, (kp_key, ki_key) in KEYS.items():
            if n < len(self.LADDER):
                wc, ratio = self.LADDER[n], self.ZERO_RATIO
                if self._first_osc(kind) is not None:
                    wc = self._sel(kind)           # 発振したゲインで走り続けない
            else:
                wc, ratio = self._sel(kind), self.RATIOS[n - len(self.LADDER)]
            kp, ki = base_gains(wc, ratio, self._inertia)
            out[kp_key], out[ki_key] = kp, ki
            self._used[kind] = wc
        self._n = n
        return out

    def leg_overrides(self, leg: int) -> dict[str, float]:
        return self._ov

    def _finish_leg(self) -> None:
        """終わった片道の発進（TC）・制動（ABS）の発振を見て、次の片道のゲインに反映する。"""
        for kind, stage in (("tc", "drive"), ("abs", "brake")):
            x = self._buf[stage]
            if len(x) >= WIN and self._n < len(self.LADDER):
                jit = jitter_rms(x)
                if self._first_osc(kind) is None:
                    std, share, freq = oscillation(x)
                    base = min((j for j in self._jit[kind] if math.isfinite(j)), default=None)
                    self._osc[kind][self._n] = is_oscillation(std, share, freq, jit, base)
                self._jit[kind].append(jit)
        self._buf = {"drive": [], "brake": []}

    def leg_step(self, leg: int, t: float, vs: VehicleState, traveled: float) -> Step:
        if leg % 2 == 0 and leg != self._ov_leg:
            if self._ov_leg >= 0:
                self._finish_leg()
            self._ov_leg = leg
            self._ov = self._make_overrides(leg // 2)
        return super().leg_step(leg, t, vs, traveled)

    def on_vehicle_state(self, vs: VehicleState) -> None:
        super().on_vehicle_state(vs)
        # 前進の片道の走行中だけ集める（戻る片道・止まる・静止は除く。戻る片道は低速で滑りが大きく、混ざると発振と誤判定する）
        stage = getattr(self, "_stage_name", "")
        leg = self._leg
        if (leg < 0 or leg % 2 or self._stage != "leg" or leg != self._cur_leg
                or stage not in self._buf or len(vs.tc_slip) < 2):
            return
        key = (leg, stage)
        if key != self._buf_key:
            self._buf_key, self._skip = key, _SKIP
        if self._skip > 0:
            self._skip -= 1
            return
        if abs(vs.speed) < _MIN_V:
            return
        s = (abs(vs.tc_slip[0]), abs(vs.tc_slip[1]))
        # TC は各輪を目標に抑える（左右でグリップが違う車では滑りやすい方が効く）、ABS は滑りやすい方に合わせる（select-low）。
        # どちらも滑りの大きい方を見る
        self._buf[stage].append(max(s))
