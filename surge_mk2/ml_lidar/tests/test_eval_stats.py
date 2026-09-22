"""`ml_lidar/eval_stats.py` のラップタイム指標（v21で追加）のテスト。

`reference_lap()`（対MCL理想ラップ）は理想ラインと速度プロファイルの合成なので、
物理的に自明な性質（コース幅を広げれば理想は速くなる・車両のグリップを上げれば
速くなる）で検証する。`run_episode()`側は「周回できたときだけ比が出る」「比の
分解が恒等式を満たす」という契約を固定する。
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

import dataclasses  # noqa: E402

import numpy as np  # noqa: E402

from ml_lidar.course_gen import walk_loop_course  # noqa: E402
from ml_lidar.eval_stats import reference_lap  # noqa: E402
from sim.raceline import compute_speed_profile  # noqa: E402
from sim.vehicle import VehicleSpec  # noqa: E402


def _course(width_m: float = 1.0):
    return walk_loop_course(8, 2.5, 0.30, 0.30, width_m, seed=1003, role="eval", name="t")


class TestReferenceLap(unittest.TestCase):
    def test_returns_positive_time_and_length(self) -> None:
        lap_s, len_m, travel = reference_lap(_course(), VehicleSpec.load(), max_speed=2.0)
        self.assertGreater(lap_s, 0.0)
        self.assertGreater(len_m, 0.0)
        # 平均速度が上限を超えていたら速度プロファイルの読み違い
        self.assertLessEqual(len_m / lap_s, 2.0 + 1e-6)
        # 理想ラインが要する舵の総量 [rad/m]。閉ループなので必ず正
        self.assertGreater(travel, 0.0)
        self.assertLess(travel, 5.0, "理想ラインの舵の総量が非現実的に大きい")

    def test_handles_course_without_width_attribute(self) -> None:
        """`Course.width`が`None`のコース（`sim/editor.py`製の手描きコース）でも動くこと。

        `compute_raceline_offsets`に`course=`を渡し忘れると`ValueError`で落ちる。
        手続き生成コースは`width`がスカラーなので、このテストが無いと気づけない。
        """
        from sim.course import Course
        path = Path(__file__).resolve().parents[2] / "sim" / "courses" / "toyota.json"
        if not path.exists():
            self.skipTest("観戦用コースが無い環境")
        course = Course.load(path)
        self.assertIsNone(course.width, "前提が変わった: toyota.jsonがwidthを持つようになった")
        lap_s, len_m, travel = reference_lap(course, VehicleSpec.load(), max_speed=2.0)
        self.assertGreater(lap_s, 0.0)
        self.assertGreater(len_m, 0.0)
        self.assertGreater(travel, 0.0)

    def test_mcl_is_faster_than_centerline(self) -> None:
        """基準に最小曲率線(MCL)を使う意味そのもの——センターラインをなぞるより速い。

        ここが逆転したら`reference_lap`は「理想」を名乗れず、`lap_ratio`も意味を失う。

        なお道幅を広げても理想ラップは縮まない（幅0.9→3.0mでオフセット最大0.17mの
        まま不変、2026-09-17実測）。**これは正則化`λ‖α‖²`のせいではない**——λを
        1/1000にしてもオフセットは0.171mのままで、境界に張り付く点も0%（内点解）。
        この手続き生成コース（ランダムウォーク閉ループ）では曲率二乗和の最小が
        素で±0.17m程度にある、というだけ。基準として妥当かは別途、素朴な平滑化線と
        Σκ²・ラップタイムを突き合わせて確認済み（12コースで平滑化線が勝ったのは0本）。
        """
        course, spec = _course(), VehicleSpec.load()
        mcl_s, _, _ = reference_lap(course, spec, max_speed=2.0)

        v = compute_speed_profile(course.centerline, np.zeros(len(course.centerline)),
                                  mu=spec.mu, max_speed=2.0,
                                  drive_accel_m_s2=spec.drive_accel_m_s2,
                                  brake_decel_m_s2=spec.brake_decel_m_s2)
        closed = np.vstack([course.centerline[:, :2], course.centerline[:1, :2]])
        seg = np.hypot(*np.diff(closed, axis=0).T)
        centerline_s = float(np.sum(seg / np.maximum(0.5 * (v + np.roll(v, -1)), 1e-3)))
        self.assertLess(mcl_s, centerline_s)

    def test_higher_grip_is_faster(self) -> None:
        """`mu`はエピソードごとのドメインランダム化で振れる。理想側もそれに追従して
        いないと比が歪む（`reference_lap`のdocstring参照）ので、効いていることを固定する。"""
        base = VehicleSpec.load()
        low = dataclasses.replace(base, mu=base.mu * 0.6)
        high = dataclasses.replace(base, mu=base.mu * 1.4)
        self.assertLess(reference_lap(_course(), high, 2.0)[0],
                        reference_lap(_course(), low, 2.0)[0])

    def test_speed_cap_slows_ideal_lap(self) -> None:
        fast, _, _ = reference_lap(_course(), VehicleSpec.load(), max_speed=2.0)
        slow, _, _ = reference_lap(_course(), VehicleSpec.load(), max_speed=1.0)
        self.assertGreater(slow, fast * 1.2)


class TestEpisodeRatios(unittest.TestCase):
    """`run_episode()`が返す比の契約（実モデルを使わず辞書の性質だけ確認する）。"""

    @staticmethod
    def _ratios(lap_done: bool):
        """`run_episode()`の比の計算と同じ式（配線ではなく式の恒等性を見る）。"""
        lap_time_s, ideal_lap_s = 9.6, 8.0
        path_len_m, ideal_len_m = 17.5, 17.7
        if not lap_done:
            return None, None, None
        return (lap_time_s / ideal_lap_s, path_len_m / ideal_len_m,
                (path_len_m / lap_time_s) / (ideal_len_m / ideal_lap_s))

    def test_decomposition_identity(self) -> None:
        """`lap_ratio == dist_ratio / speed_ratio`——この恒等式が成り立つからこそ
        「タイムをラインで失ったのか速度で失ったのか」が読める（docstring参照）。"""
        lap, dist, speed = self._ratios(True)
        self.assertAlmostEqual(lap, dist / speed, places=9)

    def test_no_ratio_without_lap(self) -> None:
        self.assertEqual(self._ratios(False), (None, None, None))



class TestGeneralizationScore(unittest.TestCase):
    """v21: `best_model_generalized`の選定スコアのテスト。

    `mean_reward`基準は**v21で実際に26%遅いモデルを選んでいた**（reward 178.76 対
    179.26で区別がつかないのに、lap_ratioは1.286 対 1.022）。同じ失敗を繰り返さない
    ように、スコアの向きと縮退ケースを固定する。
    """

    @staticmethod
    def _score(lap_ratio, lap_rate):
        from ml_lidar.train_rl import GeneralizationEvalCallback
        return GeneralizationEvalCallback._score(lap_ratio, lap_rate)

    def test_smaller_is_better(self) -> None:
        self.assertLess(self._score(1.022, 0.995), self._score(1.286, 0.99))

    def test_lap_rate_penalises(self) -> None:
        """同じラップタイムでも、周回できない率が高いほどスコアは悪化する。"""
        self.assertLess(self._score(1.05, 1.0), self._score(1.05, 0.5))

    def test_unrankable_returns_none(self) -> None:
        """1本も完走できなかった評価回は順位づけしない（前のbestを保持させる）。"""
        self.assertIsNone(self._score(None, 0.0))
        self.assertIsNone(self._score(1.1, 0.0))
        self.assertIsNone(self._score(float("nan"), 0.5))

if __name__ == "__main__":
    unittest.main()
