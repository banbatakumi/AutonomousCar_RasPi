"""自律走行の灯火（パッシング・ウィンカー・ハザード）の出し方。

実レース（Super GT・Le Mans）の使い方に倣う:

- **パッシング**は「前に何かある・気づいてくれ」の予告。数回だけ点滅させて終わる
  （Le Mans は 2 秒以内・4 回以内）。点灯しっぱなしにしない。
- **ウィンカー**は動く前の意思表示。パッシングのあとに、避ける側へ出す。
- **ハザード**は止まって動かない合図（駐車完了）。

STM32 は `passing` を立てている間ずっと前照灯を全光量にするだけで、点滅の周期は持たない
（ウィンカーは STM32 が点滅させる）。そのためパッシングの点滅は Pi が時間で作る。
**`plan()` の周期（LiDAR の 10Hz）で分解できる 0.25s 点灯／0.25s 消灯**にしてある。
"""

from __future__ import annotations

#: パッシングの点滅（点灯 s・消灯 s・回数）
PASS_ON_S = 0.25
PASS_OFF_S = 0.25
PASS_FLASHES = 2
#: 障害物の判断が途切れてから、次に見つけたときまた最初（パッシング）から出し直すまでの間 [s]。
#: 検出が数周期途切れるたびにパッシングし直さない
REARM_S = 3.0
#: 避け終わって（または避ける向きが分からなくなって）からウィンカーを出し続ける時間 [s]。
#: 経路へ戻る間も出しておく
WINKER_HOLD_S = 1.5


class SignalLights:
    """障害物を見つけたら「パッシング → ウィンカー」の順に出す。駐車完了ではハザード。"""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._active = False          #: 障害物の一連の動作の最中（パッシング→ウィンカー）
        self._t = 0.0                 #: 動作を始めてからの経過 [s]
        self._quiet = 0.0             #: 障害物が無くなってからの経過 [s]
        self._side = 0                #: 出しているウィンカーの向き（+1=左 -1=右 0=なし）
        self._hold = 0.0              #: ウィンカーの残り [s]

    @property
    def pass_total_s(self) -> float:
        return PASS_FLASHES * (PASS_ON_S + PASS_OFF_S)

    def update(self, dt: float, *, obstacle: bool, side: int = 0,
               hazard: bool = False) -> tuple[bool, bool, bool]:
        """`(passing, winker_left, winker_right)`。

        :param obstacle: 障害物に対応している最中か（避ける・止まる・迂回する）
        :param side: 避ける向き（+1=左 -1=右 0=まだ決まっていない）
        :param hazard: ハザードを出す（左右同時点滅。他の灯火より優先）
        """
        if hazard:
            self.reset()
            return False, True, True
        dt = max(0.0, dt)

        if obstacle:
            self._quiet = 0.0
            if not self._active:
                self._active, self._t = True, 0.0
        else:
            self._quiet += dt
            if self._active and self._quiet >= REARM_S:
                self._active = False

        passing = False
        if self._active and self._t < self.pass_total_s:
            passing = (self._t % (PASS_ON_S + PASS_OFF_S)) < PASS_ON_S
            self._t += dt
        elif self._active:
            self._t += dt

        # ウィンカーはパッシングが終わってから。向きは決まった時点で出し、避け終わるまで保つ
        signalling = self._active and self._t >= self.pass_total_s
        if signalling and obstacle and side != 0:
            self._side, self._hold = side, WINKER_HOLD_S
        elif self._hold > 0.0:
            self._hold = max(0.0, self._hold - dt)
            if self._hold == 0.0:
                self._side = 0
        left = self._side > 0 and self._hold > 0.0
        right = self._side < 0 and self._hold > 0.0
        return passing, left, right
