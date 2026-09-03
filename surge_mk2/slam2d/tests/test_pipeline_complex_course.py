"""`pipeline.SlamSystem`の複雑コース(シケイン・ヘアピンあり)でのループ閉じ効果検証。

長距離ドリフト対策の計画（フェーズ1、評価ハーネス整備）。`test_pipeline_oval.py`と
同じ検証パターンを、より複雑な形状（`sim/courses/make_random_courses.py`が生成した
`circuit_chicane_*`、procedural生成のシケイン・ヘアピン付きコース）に拡張する。
ovalよりコーナーが多く曲率も急なため、`Frontend`単体でのヨードリフトがより大きく
出ることを期待し、そのドリフトをループ閉じでどこまで補正できるかを位置誤差[m]・
ヨー誤差[deg]の両方でログに残す（oval用テストはヨー誤差のみだった）。

**oval用テストと同じ理由で「改善した」ことをassertしない**——`backend/loop_detection.py`
のdocstring「既知の限界」参照。実測値をログし、実用上破綻しない範囲であることのみ
確認する。パラメータチューニング自体は計画フェーズ3の作業範囲。
"""

import math
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from slam2d.backend.loop_detection import LoopDetectorConfig  # noqa: E402
from slam2d.core.frontend import Frontend, FrontendConfig  # noqa: E402
from slam2d.core.grid import OccGrid  # noqa: E402
from slam2d.core.motion import ExternalTwistModel  # noqa: E402
from slam2d.core.types import Pose2D, Twist2D, between, wrap_angle  # noqa: E402
from slam2d.pipeline import PipelineConfig, SlamSystem  # noqa: E402
from slam2d.tests.helpers import (  # noqa: E402
    centerline_arclengths, centerline_loop_length, centerline_pose_at,
    centerline_twist_at, make_raw_scan, make_track_from_centerline,
)

#: `sim/courses/make_random_courses.py`が固定シードで生成したコース名。
#: 中心線は同スクリプトが隣に保存した`<name>_centerline.npz`から読む
#: （`sim`/`raspi`パッケージには一切依存せず、numpy単体で読める）
COURSES_DIR = Path(__file__).resolve().parents[2] / "sim" / "courses"
COURSE_NAMES = ["circuit_chicane_a", "circuit_chicane_b", "circuit_chicane_c"]
#: コースごとに1シードだけ回す（複数シードでの本格的な多条件検証はフェーズ3の
#: 診断ツールの役割。このテストは「壊れていないか」の高速な回帰確認に徹する）
SEEDS = {"circuit_chicane_a": 7, "circuit_chicane_b": 8, "circuit_chicane_c": 9}
SPEED = 0.5
DT = 0.1
NOISE_SIGMA = 0.01
N_LAPS = 2


def _load_centerline(name: str) -> tuple[np.ndarray, float]:
    data = np.load(COURSES_DIR / f"{name}_centerline.npz")
    return data["centerline"], float(data["width"])


def _grid_for(centerline: np.ndarray, *, margin: float = 2.0,
              resolution: float = 0.05) -> OccGrid:
    xs, ys = centerline[:, 0], centerline[:, 1]
    cx, cy = float(xs.mean()), float(ys.mean())
    half = float(max(xs.max() - xs.min(), ys.max() - ys.min())) / 2.0 + margin
    return OccGrid(resolution=resolution, size_m=2.0 * half, origin=(cx - half, cy - half))


def _scan_generator(centerline: np.ndarray, width: float, seed: int):
    track = make_track_from_centerline(centerline, width)
    arc = centerline_arclengths(centerline)
    total = centerline_loop_length(centerline, arc)
    n_steps = math.ceil(total * N_LAPS / SPEED / DT) + 5
    rng = np.random.default_rng(seed)
    s = 0.0
    for _ in range(n_steps):
        s += SPEED * DT
        tx, ty, tyaw = centerline_pose_at(centerline, arc, total, s)
        tv, twr = centerline_twist_at(centerline, arc, total, s, SPEED)
        raw = make_raw_scan(tx, ty, tyaw, segs=track, max_range=8.0,
                           noise_sigma=NOISE_SIGMA, rng=rng)
        yield s, Pose2D(tx, ty, tyaw), Twist2D(tv, 0.0, twr), raw, total


class TestComplexCourseLoopClosureDriftComparison(unittest.TestCase):
    def test_pipeline_vs_frontend_drift_is_recorded(self):
        for name in COURSE_NAMES:
            centerline, width = _load_centerline(name)
            arc = centerline_arclengths(centerline)
            total = centerline_loop_length(centerline, arc)
            start_true = Pose2D(*centerline_pose_at(centerline, arc, total, 0.0))
            seed = SEEDS[name]

            def run(use_pipeline: bool) -> list[tuple[float, float]]:
                grid = _grid_for(centerline)
                holder = {"twist": Twist2D(SPEED, 0.0, 0.0)}
                motion = ExternalTwistModel(lambda: holder["twist"])
                fe_config = FrontendConfig(anisotropic=True, kf_dist=0.02,
                                           kf_yaw=math.radians(1.0), max_range=8.0)
                if use_pipeline:
                    system = SlamSystem(grid, motion, PipelineConfig(
                        frontend=fe_config,
                        loop_detector=LoopDetectorConfig(min_index_gap=150, radius=0.3,
                                                         yaw_tolerance=math.radians(30.0)),
                    ))
                    frontend = system.frontend
                else:
                    frontend = Frontend(grid, motion, fe_config)
                    system = None

                errors: list[tuple[float, float]] = []
                next_boundary = total
                true_pose = start_true
                for s, tp, twist, raw, lap_len in _scan_generator(centerline, width, seed):
                    true_pose = tp
                    holder["twist"] = twist
                    if system is not None:
                        system.update(raw, DT)
                    else:
                        frontend.update(raw, DT)
                    if s >= next_boundary and len(errors) < N_LAPS:
                        true_rel = between(start_true, true_pose)
                        err = between(true_rel, frontend.pose)
                        errors.append((math.hypot(err.x, err.y), abs(wrap_angle(err.yaw))))
                        next_boundary += lap_len
                if system is not None:
                    system.flush()   # 保留中の拘束を必ず反映してから最終誤差も見る
                true_rel = between(start_true, true_pose)
                err = between(true_rel, frontend.pose)
                errors.append((math.hypot(err.x, err.y), abs(wrap_angle(err.yaw))))
                return errors

            t0 = time.time()
            pipeline_errors = run(True)
            t1 = time.time()
            frontend_errors = run(False)
            t2 = time.time()

            # 各リストは [1周目, ..., N_LAPS周目, flush後の最終値] の順、要素は(位置誤差m, ヨー誤差rad)
            pipeline_fmt = [(round(p * 100, 1), round(math.degrees(y), 1))
                           for p, y in pipeline_errors]
            frontend_fmt = [(round(p * 100, 1), round(math.degrees(y), 1))
                           for p, y in frontend_errors]
            print(f"\n[complex course={name} seed={seed}] "
                  f"with loop closure (位置cm, ヨー°): {pipeline_fmt} ({t1 - t0:.1f}s)")
            print(f"[complex course={name} seed={seed}] "
                  f"frontend only     (位置cm, ヨー°): {frontend_fmt} ({t2 - t1:.1f}s)")

            self.assertEqual(len(pipeline_errors), N_LAPS + 1)
            self.assertEqual(len(frontend_errors), N_LAPS + 1)
            # 実用上破綻していない（例外・見失い続けての発散が起きていない）ことだけ確認する。
            # 複雑コースはovalよりドリフトが大きく出うるので閾値はoval用テストより緩める
            self.assertLess(pipeline_errors[-1][1], math.radians(90.0))
            self.assertLess(pipeline_errors[-1][0], 5.0)


if __name__ == "__main__":
    unittest.main()
