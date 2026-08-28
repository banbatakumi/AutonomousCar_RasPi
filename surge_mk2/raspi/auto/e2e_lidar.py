"""End-to-End LiDAR（強化学習）— 点群をそのままステア/速度へ回帰するモデルで走る。

`ml_lidar/train_rl.py`（Stable-Baselines3 PPO、シム上の強化学習）で学習した方策を
`ml_lidar/export_onnx_rl.py` でONNX化したものを読み込んで推論するだけの薄いPlanner。

## 学習方法は模倣ではない

`raspi/auto/disparity_extender.py`（id `de`）のような教師データを使う模倣学習とは
違い、シム上で「コースに沿って進めたら+報酬・衝突したら-報酬」を頼りに試行錯誤で
方策を最適化したもの。**`de`の実力を再現するのではなく、それを超えうる**——その分、
未知の点群パターンに対してどう振る舞うかの保証は無いので、下記の独立安全策が要る。

## モデルはGUIから名前で選ぶ（`e2e/model`）

`raspi/nodes/cam_perception_node.py` の `SegmentationModel`/`reload_if_changed()` と
同じ設計。`models/e2e_lidar/<name>.onnx`（+ 同名 `.json`。前処理契約。
`ml_lidar/export_onnx_rl.py` が書く）を`e2e/model`トピック経由の名前で選び、
`planning_node.py`（`e2e_lidar`エンゲージ中のみ）が毎周期`reload_if_changed()`を
呼ぶ。**カメラ用モデル（`models/`直下）とは別ディレクトリ**にしてある——前処理契約が
全く違うので、同じ一覧に混ぜると選び間違いの温床になる。

モデルのロードは失敗しても例外を投げない・**今のモデルを保持したまま**にする
（存在しない名前を選んでも、走行中のモデルが「壁」扱いに落ちるより前のモデルで
続行する方が安全側という、同じ理由）。
未選択（`name`が空）の間は`ready=False`を返し続ける。

## 独立した安全策（`stop_dist`）

回帰モデルは「学習中に見た点群パターン」の外側でどう振る舞うか原理的に保証できない。
`disparity_extender.py` の `stop_dist`（測距不能を空き扱いにしている穴を受ける
「最後の砦」）と同じ理由で、**正面の余裕が閾値を切ったらモデル出力を無視して止める**
独立した判定を持つ。
"""

from __future__ import annotations

import json
import math
from pathlib import Path

from ..msgs.types import AutoState, Scan, VehicleState
from .base import ParamSpec, Planner, scan_window

__all__ = ["E2ELidar"]

#: `surge_mk2/`。既定モデルの場所を解決する基準（`cam_perception_node.py` と同じ深さ）
REPO_ROOT = Path(__file__).resolve().parents[2]
#: カメラ用（`models/`直下）とは別ディレクトリ。前処理契約が違うので混ぜない
DEFAULT_MODELS_DIR = REPO_ROOT / "models" / "e2e_lidar"

#: 視野内でセクタが見えている割合がこれを下回ったら計画を放棄する
#: （`disparity_extender.py` の `MIN_SEEN_RATIO` と同じ理由でパラメータにしない）
MIN_SEEN_RATIO = 0.6


def _load_from_path(path: Path) -> dict:
    """`path`のONNXモデル＋同名`.json`を読み、結果をdictで返す。

    **失敗したら例外を投げる。** `reload_if_changed()`側が捕まえて「今のモデルを
    保持する」を判断するので、ここでは握りつぶさない
    （`cam_perception_node.py`の`_load_model_by_name()`と同じ形）。
    """
    import onnxruntime as ort

    cfg_path = path.with_suffix(".json")
    cfg = json.loads(cfg_path.read_text()) if cfg_path.exists() else {}
    session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    return {
        "session": session,
        "input_name": session.get_inputs()[0].name,
        "fov_deg": float(cfg.get("fov_deg", 360.0)),
        "max_range": float(cfg.get("max_range", 5.10)),
        "max_steer": float(cfg.get("max_steer", 0.45)),
        "max_speed": float(cfg.get("max_speed", 1.5)),
    }


class E2ELidar(Planner):
    id = "e2e_lidar"
    name = "E2E LiDAR"
    description = ("点群をそのままステア/速度へ回帰する方策（強化学習）。"
                   "地図も経路も持たない実験的なモード")

    params = (
        ParamSpec(key="max_speed", label="最高速度", min=0.05, max=2.0, step=0.05,
                  default=1.0, unit="m/s",
                  note="モデル出力をこの値でクランプする。学習時の上限（`train_rl.py`の"
                       "`--max-speed`）より大きくしても意味が無い（出力レンジがそこまで届かない）"),
        ParamSpec(key="max_steer", label="最大舵角", min=0.1, max=0.524, step=0.005,
                  default=0.45, unit="rad",
                  note="モデル出力をこの値でクランプする。学習時の上限（`train_rl.py`の"
                       "`--max-steer`）と揃えること"),
        ParamSpec(key="stop_dist", label="停止する前方距離", min=0.1, max=1.0, step=0.01,
                  default=0.30, unit="m",
                  note="★独立した安全策。モデルの判断を経由せず、正面がこれを切ったら"
                       "無条件で停止する（`de`の`stop_dist`と同じ役目）"),
    )

    def __init__(self, model_path: str | Path | None = None, *,
                models_dir: str | Path | None = None) -> None:
        """:param model_path: 特定の`.onnx`を直接指定してロードする（主にテスト用）。
            通常運用では指定せず、`reload_if_changed()`（`e2e/model`トピック経由）で
            名前から選ぶ
        :param models_dir: `<名前>.onnx`を探すディレクトリ。既定`models/e2e_lidar/`
        """
        self.models_dir = Path(models_dir) if models_dir else DEFAULT_MODELS_DIR
        self._loaded_model_name = ""
        self._session = None
        self._input_name = ""
        self._fov_deg = 360.0
        self._max_range = 5.10
        self._model_max_steer = 0.45
        self._model_max_speed = 1.5
        self._load_error = "未選択"
        if model_path:
            self._apply_loaded(Path(model_path))

    def _apply_loaded(self, path: Path) -> None:
        """`_load_from_path()`の結果を自分に反映する。**失敗しても例外を投げない**
        （`model_path`を直接渡すコンストラクタ経由でのみ使う。`reload_if_changed()`は
        別に「失敗時は今のモデルを保持する」ロジックを持つので、ここは使わない）。
        """
        try:
            self._commit(_load_from_path(path))
        except Exception as e:                                       # noqa: BLE001
            self._session = None
            self._load_error = str(e)

    def _commit(self, loaded: dict) -> None:
        self._session = loaded["session"]
        self._input_name = loaded["input_name"]
        self._fov_deg = loaded["fov_deg"]
        self._max_range = loaded["max_range"]
        self._model_max_steer = loaded["max_steer"]
        self._model_max_speed = loaded["max_speed"]

    def reload_if_changed(self, desired_name: str) -> bool:
        """GUIが選んだモデル名（`e2e/model`）が今と違えば読み込み直す。

        **失敗しても今のモデルを保持する。** 存在しない名前を選んでも、それだけで
        走行が止まる（`ready=False`に戻る）よりは、前のモデルで走り続けられる方が
        安全側——ただし選び間違いに気づけるよう `self._load_error` には残す
        （`cam_perception_node.py`の`reload_if_changed()`と同じ契約）。

        :returns: 切り替えられたら`True`。名前が同じ・失敗なら`False`
        """
        if not desired_name or desired_name == self._loaded_model_name:
            return False
        try:
            loaded = _load_from_path(self.models_dir / f"{desired_name}.onnx")
        except Exception as e:                                        # noqa: BLE001
            self._load_error = str(e)
            return False
        self._commit(loaded)
        self._loaded_model_name = desired_name
        return True

    def reset(self) -> None:
        pass                    # 平滑化などの内部状態は持たない

    def plan(self, scan: Scan, vs: VehicleState | None,
             p: dict[str, float], dt: float) -> AutoState:
        st = AutoState(mode=self.id, planner=self.name)

        if self._session is None:
            name = self._loaded_model_name or "未選択"
            st.reason = f"モデル未ロード（{name}）: {self._load_error}"
            return st

        w = scan_window(scan, self._fov_deg, self._max_range)
        st.valid_ratio = w.valid_ratio
        if w.seen_ratio < MIN_SEEN_RATIO:
            st.reason = f"点群の欠測が多すぎる（視野の {w.seen_ratio * 100:.0f}%）"
            return st

        # ★独立した安全策。モデルの判断を経由しない「事実」としての正面の余裕
        front = [w.dist[j] for j, d in enumerate(w.degs) if abs(d) <= 20]
        st.free_ahead = min(front) if front else 0.0

        try:
            import numpy as np

            # 訓練側(`sim/gym_env.py`の`_obs()`・`ml_lidar/env.py`の`_to_obs()`)と
            # 揃える: 点群のあとに自車速度を1個足す。速度が無ければ0（停止中の既定）
            speed_now = 0.0 if vs is None else float(vs.speed)
            speed_norm_in = max(0.0, min(1.0, speed_now / self._model_max_speed)) \
                if self._model_max_speed > 0 else 0.0
            scan_n = np.asarray(w.dist, dtype=np.float32) / self._max_range
            x = np.concatenate([scan_n, [speed_norm_in]]).astype(np.float32)[None, :]
            out = self._session.run(None, {self._input_name: x})[0]
            steer_norm, speed_norm = float(out[0, 0]), float(out[0, 1])
        except Exception as e:                                        # noqa: BLE001
            st.reason = f"推論に失敗: {e}"
            return st

        if not (math.isfinite(steer_norm) and math.isfinite(speed_norm)):
            st.reason = "モデル出力が不正（NaN/Inf）"
            return st

        # `ml_lidar/env.py` の `_to_physical()` と対になる後処理（[-1,1]でクリップしてから
        # 学習時のレンジに戻す）。`p["max_steer"]`/`p["max_speed"]`（GUIの安全側クランプ）
        # で最終的にもう一段絞る
        steer_norm = max(-1.0, min(1.0, steer_norm))
        speed_norm = max(-1.0, min(1.0, speed_norm))
        steer = steer_norm * self._model_max_steer
        speed = (speed_norm + 1.0) * 0.5 * self._model_max_speed

        max_steer, max_speed = p["max_steer"], p["max_speed"]
        st.target_steer = max(-max_steer, min(max_steer, steer))
        st.ready = True

        stop_d = p["stop_dist"]
        if st.free_ahead <= stop_d:
            st.brake = True
            st.target_speed = 0.0
            st.reason = f"正面 {st.free_ahead * 100:.0f}cm で停止（モデル出力を安全側で上書き）"
            return st

        st.target_speed = max(0.0, min(max_speed, speed))
        st.reason = f"モデル出力 steer={st.target_steer:+.2f}rad speed={st.target_speed:.2f}m/s"
        return st
