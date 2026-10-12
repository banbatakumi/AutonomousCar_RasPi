"""灯火の出し方（`raspi/nav/signals.py`）と、planner・中継への配線。"""

from __future__ import annotations

from raspi.msgs.types import AutoState, DriveCmd
from raspi.nav.signals import PASS_FLASHES, REARM_S, WINKER_HOLD_S, SignalLights

DT = 0.1


def run(sl: SignalLights, n: int, **kw):
    return [sl.update(DT, **kw) for _ in range(n)]


def test_idle_is_dark():
    assert all(o == (False, False, False) for o in run(SignalLights(), 20, obstacle=False))


def test_passing_then_winker_in_order():
    sl = SignalLights()
    out = run(sl, 40, obstacle=True, side=1)
    first_winker = next(i for i, o in enumerate(out) if o[1])
    last_passing = max(i for i, o in enumerate(out) if o[0])
    assert last_passing < first_winker                        # パッシング → ウィンカー
    # 点滅の回数
    rises = sum(1 for i, o in enumerate(out) if o[0] and (i == 0 or not out[i - 1][0]))
    assert rises == PASS_FLASHES
    assert all(o[1] and not o[2] for o in out[first_winker:])  # 左だけ点灯し続ける
    assert not any(o[0] for o in out[first_winker:])


def test_right_side():
    out = run(SignalLights(), 40, obstacle=True, side=-1)
    assert out[-1] == (False, False, True)


def test_no_side_no_winker():
    assert not any(o[1] or o[2] for o in run(SignalLights(), 40, obstacle=True, side=0))


def test_winker_holds_then_off_and_no_repassing_on_flicker():
    sl = SignalLights()
    run(sl, 40, obstacle=True, side=1)
    out = run(sl, int(WINKER_HOLD_S / DT) + 3, obstacle=False)
    assert out[0][1] and out[-1] == (False, False, False)
    # 検出が途切れてすぐ戻っても、パッシングはやり直さない
    sl2 = SignalLights()
    run(sl2, 40, obstacle=True, side=1)
    run(sl2, 5, obstacle=False)
    assert not any(o[0] for o in run(sl2, 10, obstacle=True, side=1))


def test_rearm_after_quiet():
    sl = SignalLights()
    run(sl, 40, obstacle=True, side=1)
    run(sl, int(REARM_S / DT) + 2, obstacle=False)
    assert any(o[0] for o in run(sl, 10, obstacle=True))


def test_hazard_overrides():
    sl = SignalLights()
    run(sl, 3, obstacle=True, side=1)
    assert sl.update(DT, obstacle=True, side=1, hazard=True) == (False, True, True)


def test_planning_cmd_carries_lights():
    from raspi.nodes.planning_node import PlanningNode
    assert "passing" in AutoState.__struct_fields__ and "winker_left" in DriveCmd.__struct_fields__
