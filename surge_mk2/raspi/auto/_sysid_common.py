"""システム同定プランナー3つ（`sysid_steer`/`sysid_speed`/`sysid_corner`）が
共有する小道具。**catalogには登録しない**（`_`始まりのファイル名で
`raspi/auto/registry.py`の対象外——`Planner`のサブクラスではなくヘルパのため）。
"""

from __future__ import annotations

from ..msgs.types import VehicleState

__all__ = ["TestGate"]


class TestGate:
    """内部の状態機械（ステップ番号・経過時間）を、試験が実際に進行中の間だけ動かす。

    ## なぜ要るか（2026-08-31、実機で3回に分けて不具合報告・修正）

    `planning_node.py`はengageしていなくても`plan()`を呼び続ける（他のplanner
    が「engageする前にどう判断するか」をGUIで覗けるようにするための既存の
    設計）。普通のplannerは毎回その場で答えを作り直すだけで内部に時間経過を
    持たないので困らないが、システム同定のplannerはステップ列を時間で進める
    設計なので、**「試験開始」を押していなくてもステップが進む**不具合が
    最初に見つかった。

    最初の修正は`VehicleState.armed`（人間がARMを保持しているか）をゲートに
    したが、これは2つの問題を生んだ：
    - 「試験中止」は`engaged`をFalseにするだけでARM保持自体は解除しない
      （ARMは人間側の安全弁で、ソフトから奪わない）ため、**ARMを保持したまま
      中止しても、armedがTrueのままなので進行が止まらなかった**
    - それを塞ぐために「ARMが一度Falseに落ちるまで再開しない」凍結を足したが、
      今度は**ARMを保持したまま同じ試験をもう一度「試験開始」しても、ARMの
      入り直しが無いので再開できない**という逆方向の不具合になった

    根本原因は`armed`が`engaged`（試験開始/中止の状態）の代用にならないこと。
    `plan()`自体はengagedを受け取れない（`Planner`の共通契約——全plannerに
    影響するので変えない）ので、代わりに`planning_node.py`が`set_engaged()`
    をダックタイピングで呼ぶ（`_apply_e2e_model`の`reload_if_changed`と
    同じパターン）。**進行のゲートは`engaged`、ARMは表示（「ARM待ち」）専用**
    にしたことで、上の2つの不具合を両方解消できる。
    """

    def __init__(self) -> None:
        self._engaged = False
        self._just_started = False

    def reset(self) -> None:
        self._engaged = False
        self._just_started = False

    def set_engaged(self, engaged: bool) -> None:
        """`planning_node.py`が`plan()`の直前に毎周期呼ぶ（ダックタイピング）。

        False→Trueの遷移を「試験開始が新しく押された」と解釈し、次の
        `tick()`が`just_started=True`を返すようにする。
        """
        if engaged and not self._engaged:
            self._just_started = True
        self._engaged = engaged

    def tick(self, vs: VehicleState | None) -> tuple[bool, bool, bool]:
        """`(engaged, armed, just_started)`。

        `engaged`が内部状態機械を進めてよいかのゲート。`armed`は表示専用
        （engaged中でもARMが無ければ「ARM待ち」を出す）。`just_started`は
        今回だけTrueで、呼び出し側はこれを見て内部カウンタを0に戻す。
        """
        armed = bool(vs and vs.armed)
        just_started = self._just_started
        self._just_started = False
        return self._engaged, armed, just_started
