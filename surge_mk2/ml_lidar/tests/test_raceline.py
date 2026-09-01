"""`sim/raceline.py` のテスト。曲率二乗和を厳密に最小化することまでは検証せず
（`torch`の勾配降下は反復回数依存の近似解）、`SimE2EEnv`の報酬に使う上で
壊れていないこと（道幅制約を破らない・曲率を悪化させない・曲率が高いほど
目標速度が下がる）を確認する。
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

import numpy as np  # noqa: E402

from sim.raceline import compute_raceline_offsets, compute_speed_profile  # noqa: E402
from sim.random_course import (  # noqa: E402
    generate_circuit_course,
    generate_corridor_course,
    generate_narrow_course,
    generate_random_course,
)
from raspi.nav.centerline import resample_loop  # noqa: E402
from sim.track import centerline as track_centerline  # noqa: E402


def _closed_circle_centerline(radius: float, *, step: float = 0.1) -> np.ndarray:
    """半径`radius`の閉ループ円の中心線 `(N,3)`。`track_centerline`は1周分の弧を
    そのまま返すと**始点と終点が重複**する（弧長がほぼ0のセグメントができ、曲率
    計算が破綻する）ので、`sim/random_course.py`の生成器と同じく`resample_loop`で
    重複を除いた均等間隔の点列に作り直してから使う。"""
    pts = track_centerline([("arc", radius, 360.0)], radius, 0.0, math.pi / 2, step=step)
    xy = resample_loop(pts[:, :2], step)
    nxt = np.roll(xy, -1, axis=0)
    yaw = np.arctan2(nxt[:, 1] - xy[:, 1], nxt[:, 0] - xy[:, 0])
    return np.column_stack((xy, yaw))


def _signed_curvature(xy: np.ndarray) -> np.ndarray:
    """`sim/raceline.py`の`_discrete_curvature`と同じ式（テスト側で独立に再実装し、
    実装のコピペではなく仕様として検証する）。"""
    loop = np.vstack([xy, xy[:1]])
    seg = np.hypot(*np.diff(loop, axis=0).T)
    yaw = np.arctan2(np.diff(loop[:, 1]), np.diff(loop[:, 0]))
    dyaw = np.diff(np.unwrap(np.concatenate([yaw, yaw[:1]])))
    return dyaw / np.maximum(seg, 1e-6)


def _max_offset_for(width, n_pts: int, *, vehicle_half_width_m: float = 0.09,
                    safety_margin_m: float = 0.03) -> np.ndarray:
    half_w = width / 2.0 if isinstance(width, np.ndarray) else np.full(n_pts, width / 2.0)
    return np.maximum(0.0, half_w - vehicle_half_width_m - safety_margin_m)


class TestComputeRacelineOffsets(unittest.TestCase):
    def test_offsets_stay_within_track_bounds(self):
        """narrow/obstacleを含む複数アーキタイプ・複数seedで、道幅制約
        （車体半幅+安全マージンを引いた分）を一度も破らないこと。"""
        for seed in range(3):
            for gen in (generate_random_course, generate_circuit_course,
                       generate_corridor_course, generate_narrow_course):
                rng = np.random.default_rng(seed)
                course = gen(rng)
                offset = compute_raceline_offsets(
                    course.centerline, course.width, vehicle_half_width_m=0.09)
                max_offset = _max_offset_for(course.width, len(course.centerline))
                self.assertTrue(np.all(np.abs(offset) <= max_offset + 1e-6),
                                f"{gen.__name__} seed={seed} で道幅制約を破った")

    def test_offsets_do_not_worsen_total_curvature(self):
        """理想ラインは中心線そのままより曲率二乗和が悪化してはいけない
        （曲率最小化が目的なので、少なくとも中心線=オフセット0以下であるべき）。"""
        for seed in range(3):
            for gen in (generate_random_course, generate_circuit_course, generate_corridor_course):
                rng = np.random.default_rng(seed)
                course = gen(rng)
                xy = course.centerline[:, :2]
                yaw = course.centerline[:, 2]
                before = np.sum(_signed_curvature(xy) ** 2)

                offset = compute_raceline_offsets(
                    course.centerline, course.width, vehicle_half_width_m=0.09)
                normal = np.column_stack((-np.sin(yaw), np.cos(yaw)))
                after = np.sum(_signed_curvature(xy + offset[:, None] * normal) ** 2)

                self.assertLessEqual(after, before * 1.01,
                                     f"{gen.__name__} seed={seed} で曲率二乗和が悪化した")

    def test_full_circle_loop_inflates_uniformly_to_box_limit(self):
        """直線区間を持たない完全な円（曲率が全周で一定・非ゼロ）では、外側へ
        均一にオフセットするほど曲率二乗和は単調に下がり続ける（半径R+offsetの円は
        半径Rの円より必ず曲率が小さい）ので、**box制約いっぱいまで均一に膨らむのが
        数学的に正しい最適解**——中心線付近に留まる理由は無い。

        当初このテストは逆に「ほとんど動かないはず」と書かれていたが、これは
        Adam勾配降下(旧実装)が200反復では収束しきらず、たまたま小さいオフセットで
        止まっていただけだった（L-BFGSへの変更でこの誤りが発覚。反復を増やすほど
        Adamの解は境界張り付き点が増える一方でギザギザが悪化する不良設定に陥っており、
        `sim/raceline.py`のdocstring参照）。直線とコーナーが混在する実コース
        （circuit/fuji等）では、直線区間はコーナー区間と違って一方向に膨らませ続ける
        理由が無いぶん、この退化した挙動は起きない——`test_offsets_do_not_worsen_total_curvature`
        で実コースでは曲率が悪化しないことを別途確認している。"""
        centerline = _closed_circle_centerline(radius=20.0)
        xy, yaw = centerline[:, :2], centerline[:, 2]

        offset = compute_raceline_offsets(centerline, 1.0, vehicle_half_width_m=0.09)
        max_offset = _max_offset_for(1.0, len(centerline))[0]

        # 全点がbox制約いっぱい(符号は円の向きで決まり、一貫して同じ側)まで
        # 均一に膨らんでいること
        self.assertTrue(np.all(np.abs(offset) > max_offset * 0.9))
        self.assertTrue(np.all(offset > 0) or np.all(offset < 0))

        normal = np.column_stack((-np.sin(yaw), np.cos(yaw)))
        before = np.mean(np.abs(_signed_curvature(xy)))
        after = np.mean(np.abs(_signed_curvature(xy + offset[:, None] * normal)))
        expected = 1.0 / (20.0 + max_offset)  # 半径R+offsetの円の理論曲率
        self.assertLess(after, before)
        self.assertAlmostEqual(after, expected, delta=expected * 0.05)

    def test_narrow_width_array_clips_to_zero_where_track_is_too_tight(self):
        """`narrow`アーキタイプ（`Course.width`が配列）でも例外にならず、
        車体+安全マージンより道幅が狭い区間ではオフセットが0に張り付く。"""
        rng = np.random.default_rng(0)
        course = generate_narrow_course(rng)
        self.assertIsInstance(course.width, np.ndarray)
        offset = compute_raceline_offsets(course.centerline, course.width,
                                          vehicle_half_width_m=0.09)
        max_offset = _max_offset_for(course.width, len(course.centerline))
        too_tight = max_offset <= 1e-9
        if np.any(too_tight):
            self.assertTrue(np.allclose(offset[too_tight], 0.0))


class TestComputeSpeedProfile(unittest.TestCase):
    def _circle(self, radius: float, width: float = 1.0) -> tuple[np.ndarray, float]:
        return _closed_circle_centerline(radius), width

    def test_tighter_curvature_yields_lower_target_speed(self):
        tight, _ = self._circle(radius=1.0)
        wide, _ = self._circle(radius=5.0)
        offset_tight = np.zeros(len(tight))
        offset_wide = np.zeros(len(wide))

        v_tight = compute_speed_profile(tight, offset_tight, mu=0.45, max_speed=10.0,
                                        drive_accel_m_s2=0.0, brake_decel_m_s2=0.0)
        v_wide = compute_speed_profile(wide, offset_wide, mu=0.45, max_speed=10.0,
                                       drive_accel_m_s2=0.0, brake_decel_m_s2=0.0)
        self.assertLess(np.max(v_tight), np.max(v_wide))

    def test_speed_capped_by_max_speed(self):
        wide, _ = self._circle(radius=50.0)
        offset = np.zeros(len(wide))
        v = compute_speed_profile(wide, offset, mu=0.9, max_speed=1.2,
                                  drive_accel_m_s2=0.0, brake_decel_m_s2=0.0)
        self.assertTrue(np.all(v <= 1.2 + 1e-9))

    def test_explicit_drive_accel_overrides_default_fallback(self):
        """`drive_accel_m_s2`が実測(>0)で既定フォールバック(`DEFAULT_DRIVE_ACCEL_M_S2`)
        よりずっと小さいなら、コーナー立ち上がり後の平均速度がフォールバック使用時より
        低く抑えられるはず。曲率0の完全な円（前の2テストの`_circle`）だと、
        すべての点が最初からグリップ限界=`max_speed`に張り付いて前進パスで
        何もクランプされず違いが出ないため、実際にタイトなコーナー+直線を持つ
        `generate_corridor_course`（本番のコース生成器）を使う。"""
        rng = np.random.default_rng(0)
        course = generate_corridor_course(rng, width=1.0)
        offset = np.zeros(len(course.centerline))

        v_default = compute_speed_profile(course.centerline, offset, mu=0.9, max_speed=5.0,
                                          drive_accel_m_s2=0.0, brake_decel_m_s2=0.0)
        v_slow_accel = compute_speed_profile(course.centerline, offset, mu=0.9, max_speed=5.0,
                                             drive_accel_m_s2=0.05, brake_decel_m_s2=0.0)
        self.assertLess(np.mean(v_slow_accel), np.mean(v_default))


if __name__ == "__main__":
    unittest.main()
