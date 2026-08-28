"""`sim/gym_env.py` の `SimE2EEnv` を `gymnasium.Env` としてラップする。

`sim/` を軽量に保つ方針（`gymnasium`・`torch`を持ち込まない）なので、SB3 が要求する
インターフェース（`observation_space`/`action_space`・`[-1,1]`区間の行動）はこちら側で足す。

    観測: [scan/max_range を [0,1] に正規化した SCAN_DIM 点, 自車速度/max_speed を
          [0,1] に正規化した1個]（末尾）
    行動: [-1,1]^2 → steer は [-max_steer,+max_steer]、speed は [0,max_speed] へ線形変換
          （後退は今回のスコープ外）
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
                max_steer: float = 0.45, seed: int = 0, **env_kwargs) -> None:
        super().__init__()
        self._env = SimE2EEnv(courses, course_fn=course_fn, max_steps=max_steps,
                              max_speed=max_speed, max_steer=max_steer, seed=seed,
                              **env_kwargs)
        self._max_speed = max_speed
        self._max_steer = max_steer
        self._max_range = self._env.max_range

        self.observation_space = spaces.Box(0.0, 1.0, shape=(OBS_DIM,), dtype=np.float32)
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
        """先頭`SCAN_DIM`点は`scan/max_range`、末尾1個は`speed/max_speed`で正規化する。"""
        scan_n = np.clip(raw[:SCAN_DIM] / self._max_range, 0.0, 1.0)
        speed_n = np.clip(raw[SCAN_DIM:] / self._max_speed, 0.0, 1.0)
        return np.concatenate([scan_n, speed_n]).astype(np.float32)

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        if seed is not None:
            self._env.rng = np.random.default_rng(seed)
        obs = self._env.reset()
        return self._to_obs(obs), {}

    def step(self, action: np.ndarray):
        obs, reward, terminated, truncated, info = self._env.step(self._to_physical(action))
        return self._to_obs(obs), float(reward), terminated, truncated, info
