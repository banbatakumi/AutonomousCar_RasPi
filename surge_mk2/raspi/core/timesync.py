"""STM32 ⇄ Pi のクロックオフセット推定（PING/PONG 4タイムスタンプ方式）。

docs/uart_protocol.md §6。目的は「STM32 の t_us を Pi の CLOCK_MONOTONIC に変換する」こと。
SLAM・アクチュエータ遅延の実測・センサ融合がすべてこれに依存する。

    T1 = Pi が PING を送った時刻          （Pi 単調 ns）
    T2 = STM32 が PING を受けた時刻        （STM32 u32 μs = PONG.t_ping_rx_us）
    T3 = STM32 が PONG を送り始めた時刻    （STM32 u32 μs = PONG.t_pong_tx_us）
    T4 = Pi が PONG を受けた時刻          （Pi 単調 ns）

    往復遅延 delay  = (T4 − T1) − (T3 − T2)
    対応点          STM32 中点 (T2+T3)/2  ↔  Pi 中点 (T1+T4)/2

`delay` が小さいサンプルほど信頼できる（待ち行列で伸びていない）。
直近ウィンドウの中から delay の小さいものを使い、線形回帰で切片（オフセット）と
傾き（クロック比≈ドリフト）を同時に推定する。

STM32 の μs カウンタは u32 で約 71.6 分でラップするため、内部では 64bit に伸ばして扱う。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

U32 = 1 << 32
US_TO_NS = 1000


@dataclass(frozen=True, slots=True)
class Sample:
    t_stm_ns: int   # STM32 中点（unwrap 済み・ns 換算）
    t_pi_ns: int    # Pi 中点（ns）
    delay_ns: int   # 往復遅延


class TimeSync:
    """PONG サンプルからクロック対応を推定する。スレッド安全ではない（1ループ内で使う）。

    :param window: 保持するサンプル数（直近のみで回帰し、古いドリフトを引きずらない）
    :param good_factor: 「良い」サンプルの閾値。best_delay * factor 以内を回帰に使う
    """

    __slots__ = ("_samples", "_ref_stm_us", "_good_factor", "_good_margin_ns",
                 "_min_fit_span_ns")

    def __init__(self, window: int = 256, good_factor: float = 2.0,
                 good_margin_ns: int = 300_000,
                 min_fit_span_ns: int = 15 * 1_000_000_000) -> None:
        self._samples: deque[Sample] = deque(maxlen=window)
        self._ref_stm_us: int = 0            # unwrap の基準（最新の STM32 時刻・u64 μs）
        self._good_factor = good_factor
        self._good_margin_ns = good_margin_ns
        # ドリフト推定に必要な最小時間スパン。短い窓の回帰は ms 級ジッタで傾きが暴れ、
        # 実在しない数千 ppm を出す。docs §6.3 の「30秒窓」に倣い、貯まるまでは
        # 傾き=1（オフセットのみ）にフォールバックする。
        self._min_fit_span_ns = min_fit_span_ns

    # ── STM32 u32 μs の unwrap ──

    def observe_stm_us(self, raw_u32: int) -> int:
        """任意の STM32 タイムスタンプ（TELEMETRY.t_us 等）を u64 μs に伸ばして返す。

        50Hz で来る TELEMETRY を毎回これに通しておくと基準が最新に保たれ、
        まれにしか来ない PONG の T2/T3 も正しい周回で unwrap できる。
        """
        u = self._unwrap(raw_u32)
        if u > self._ref_stm_us:
            self._ref_stm_us = u
        return u

    def _unwrap(self, raw_u32: int) -> int:
        """下位32bitが raw_u32 の値のうち、基準に最も近いものを選ぶ。"""
        ref = self._ref_stm_us
        high = ref >> 32
        best = raw_u32                       # high = 0 の場合
        best_dist = abs(best - ref)
        for h in (high - 1, high, high + 1):
            if h < 0:
                continue
            v = (h << 32) | raw_u32
            d = abs(v - ref)
            if d < best_dist:
                best, best_dist = v, d
        return best

    # ── サンプル追加 ──

    def add_pong(self, t1_ns: int, t2_us: int, t3_us: int, t4_ns: int) -> Sample | None:
        """1往復分を取り込む。異常なサンプルは無視して None を返す。"""
        t2 = self._unwrap(t2_us)
        t3 = self._unwrap(t3_us)
        if t3 < t2:                          # ラップ跨ぎ（T3 は T2 の直後のはず）
            t3 += U32

        stm_dur_ns = (t3 - t2) * US_TO_NS
        pi_dur_ns = t4_ns - t1_ns
        delay = pi_dur_ns - stm_dur_ns
        # STM32 側処理時間が往復より長い＝物理的にありえない → 破棄
        if delay < 0 or pi_dur_ns <= 0:
            return None

        sample = Sample(
            t_stm_ns=(t2 + t3) * US_TO_NS // 2,
            t_pi_ns=(t1_ns + t4_ns) // 2,
            delay_ns=delay,
        )
        self._samples.append(sample)
        if t3 > self._ref_stm_us:
            self._ref_stm_us = t3
        return sample

    # ── 推定値 ──

    @property
    def n_samples(self) -> int:
        return len(self._samples)

    @property
    def best_delay_ns(self) -> int | None:
        if not self._samples:
            return None
        return min(s.delay_ns for s in self._samples)

    @property
    def offset_ns(self) -> int | None:
        """min filter によるオフセット（offset = t_pi − t_stm）。回帰前の素朴推定。"""
        if not self._samples:
            return None
        best = min(self._samples, key=lambda s: s.delay_ns)
        return best.t_pi_ns - best.t_stm_ns

    def _fit(self) -> tuple[int, float, float] | None:
        """delay の小さいサンプルで t_pi = a + b*(t_stm − x0) を回帰。

        戻り値 (x0, a, b)。b はクロック比（≈1）。x0 は数値精度のための基準点。
        """
        if len(self._samples) < 4:
            return None
        best = min(s.delay_ns for s in self._samples)
        thr = best * self._good_factor + self._good_margin_ns
        good = [s for s in self._samples if s.delay_ns <= thr]
        if len(good) < 4:
            return None
        # 時間スパンが短いと傾きがジッタで暴れるので、貯まるまで回帰しない
        span = good[-1].t_stm_ns - good[0].t_stm_ns
        if span < self._min_fit_span_ns:
            return None

        x0 = good[0].t_stm_ns
        n = len(good)
        xs = [s.t_stm_ns - x0 for s in good]
        ys = [s.t_pi_ns for s in good]
        mx = sum(xs) / n
        my = sum(ys) / n
        sxx = sum((x - mx) ** 2 for x in xs)
        if sxx == 0:                          # 時間差ゼロ（サンプルが同時刻）
            return None
        sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
        b = sxy / sxx
        a = my - b * mx
        return x0, a, b

    @property
    def drift_ppm(self) -> float | None:
        """クロックドリフト [ppm]。(b − 1)×1e6。回帰できないうちは None。"""
        fit = self._fit()
        if fit is None:
            return None
        return (fit[2] - 1.0) * 1e6

    def to_pi_ns(self, t_stm_raw_u32: int) -> int | None:
        """STM32 の生 t_us（u32）を Pi の単調 ns に変換する。推定できないうちは None。"""
        if not self._samples:
            return None
        t_stm = self._unwrap(t_stm_raw_u32) * US_TO_NS   # ns（回帰と同じ単位）
        fit = self._fit()
        if fit is not None:
            x0, a, b = fit                                # x0, a は ns、b は無次元
            return round(a + b * (t_stm - x0))
        off = self.offset_ns
        return None if off is None else t_stm + off

    def ready(self, min_samples: int = 8) -> bool:
        """デッドレコニング等に使える程度に収束したか。"""
        return len(self._samples) >= min_samples
