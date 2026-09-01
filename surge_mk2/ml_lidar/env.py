"""`sim/gym_env.py` の `SimE2EEnv` を `gymnasium.Env` としてラップする。

`sim/` を軽量に保つ方針（`gymnasium`・`torch`を持ち込まない）なので、SB3 が要求する
インターフェース（`observation_space`/`action_space`・`[-1,1]`区間の行動）はこちら側で足す。

    観測: [scan/max_range を [0,1] に正規化した SCAN_DIM 点, 自車速度/max_speed を
          [0,1] に正規化した1個, 現在の平滑化後ステア角/max_steer を [-1,1] に
          正規化した1個]（末尾。ステア角は2026-09-02追加——`sim/gym_env.py`の
          `SimE2EEnv`docstring参照）
    行動: [-1,1]^2 → steer は [-max_steer,+max_steer]、speed は [0,max_speed] へ線形変換
          （後退は今回のスコープ外）。**max_steerは引数を持たない**——
          `config/vehicle.toml`の車両物理限界（`SimE2EEnv.spec.max_steer`）を
          そのまま使う（2026-08-28）
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Callable

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # surge_mk2/

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
from gymnasium import spaces  # noqa: E402

from sim.course import Course  # noqa: E402
from sim.gym_env import OBS_DIM, SCAN_DIM, SimE2EEnv  # noqa: E402

__all__ = ["GymSurgeEnv"]


class GymSurgeEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, courses: list[Course] | None = None, *,
                course_fn: Callable[[], Course] | None = None,
                max_steps: int = 2000, max_speed: float = 1.5,
                seed: int = 0, **env_kwargs) -> None:
        super().__init__()
        self._env = SimE2EEnv(courses, course_fn=course_fn, max_steps=max_steps,
                              max_speed=max_speed, seed=seed, **env_kwargs)
        self._max_speed = max_speed
        self._max_steer = self._env.spec.max_steer
        self._max_range = self._env.max_range

        # 先頭SCAN_DIM+1個（scan・speed）は[0,1]、末尾1個（steer）だけ[-1,1]
        # （2026-09-02、ステア観測追加。scalarのBox(0.0,1.0,...)のままだと
        # 負値を取りうるsteerがobservation_spaceをはみ出しgymnasiumが警告する）
        low = np.zeros(OBS_DIM, dtype=np.float32)
        low[-1] = -1.0
        high = np.ones(OBS_DIM, dtype=np.float32)
        self.observation_space = spaces.Box(low, high, dtype=np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)

    @property
    def sim(self) -> SimE2EEnv:
        """内部の `SimE2EEnv`。`ml_lidar/watch.py` が車両位置・コース形状を
        描画のために直接読むための公開口（正規化を挟まない生の状態が要るため）。"""
        return self._env

    def _to_physical(self, action: np.ndarray) -> np.ndarray:
        a = np.clip(action, -1.0, 1.0)
        steer = a[0] * self._max_steer
        speed = (a[1] + 1.0) * 0.5 * self._max_speed      # [-1,1] -> [0, max_speed]
        return np.array([steer, speed], dtype=np.float32)

    def _to_obs(self, raw: np.ndarray) -> np.ndarray:
        """先頭`SCAN_DIM`点は`scan/max_range`([0,1])、次の1個は`speed/max_speed`
        ([0,1])、末尾1個は`steer/max_steer`([-1,1])で正規化する
        （2026-09-02、ステア角追加。`raspi/auto/e2e_lidar.py`の観測構築と揃えること）。
        """
        scan_n = np.clip(raw[:SCAN_DIM] / self._max_range, 0.0, 1.0)
        speed_n = np.clip(raw[SCAN_DIM:SCAN_DIM + 1] / self._max_speed, 0.0, 1.0)
        steer_n = np.clip(raw[SCAN_DIM + 1:] / self._max_steer, -1.0, 1.0)
        return np.concatenate([scan_n, speed_n, steer_n]).astype(np.float32)

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        if seed is not None:
            self._env.rng = np.random.default_rng(seed)
        obs = self._env.reset()
        return self._to_obs(obs), {}

    def step(self, action: np.ndarray):
        obs, reward, terminated, truncated, info = self._env.step(self._to_physical(action))
        return self._to_obs(obs), float(reward), terminated, truncated, info

    def set_curriculum_progress(self, progress: float) -> None:
        """`SimE2EEnv.set_curriculum_progress()`へ委譲する（`ml_lidar/train_rl.py`の
        `CurriculumCallback`が`VecEnv.env_method("set_curriculum_progress", ...)`
        経由でこのメソッドを呼ぶ）。"""
        self._env.set_curriculum_progress(progress)
