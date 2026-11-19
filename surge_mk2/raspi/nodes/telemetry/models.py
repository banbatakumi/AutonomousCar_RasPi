"""推論モデルの選択（GUI → telemetry_node → 各推論ノード）。

モデルの種類（セグメンテーション・LiDAR E2E・カメラ E2E・E2E 融合・ステレオ E2E）ごとに、置き場・保存先・
トピック・engage を落とす planner が違うだけで、処理は全部同じ。それを `ModelSlot` の表にして、一覧・選択・
保存・送信を1つの実装で回す（以前は種類ごとにコピーした5組があり、中身がずれ始めていた）。

約束（どの種類も同じ）:

- **一覧は要求したクライアントにだけ返す**（`logs_list` と同じ流儀）。返す形は
  `{"type": "<key>s", "<key>_files": [...]}`。キー名を種類ごとに分けるのは、GUI 側の `ServerMsg`
  （型で振り分けられない1つの受け皿）で混ざらないようにするため
- **置き場に実在しない名前は黙って捨てる**（一覧の取得後にファイルが消された等のズレを弾く）
- **モデルを変えたら、そのモデルを使う planner の engage は必ず落とす**（前のモデルのつもりで engage
  したまま推論だけ入れ替わると挙動が急変する。`_on_auto` の「モードを変えたら落とす」と同じ形）
- 選んだ名前は `config/<種類>_model.json` に保存し、次の起動で戻す（消えたモデルなら空文字）
- 受け手が再起動しても戻るよう、**現在の意思を周期的に流し直す**（`MODEL_HZ`）
- 空文字の意味は種類で違う（未選択／自動）。`raspi/msgs/types.py` の `ModelSelectCtrl` の表
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import msgspec

from raspi.msgs.types import (
    TOPIC_CAM_E2E_MODEL,
    TOPIC_CAM_MODEL,
    TOPIC_E2E_FUSION_MODEL,
    TOPIC_E2E_MODEL,
    TOPIC_STEREO_E2E_MODEL,
)

from .conf import read_conf, write_conf

__all__ = ["MODEL_HZ", "MODEL_SLOTS", "ModelSlot", "onnx_model_files"]

REPO_ROOT = Path(__file__).resolve().parents[3]
MODELS_DIR = REPO_ROOT / "models"
CONFIG_DIR = REPO_ROOT / "config"
#: 選択を受け手へ流し直す周期 [Hz]（受け手の再起動から1秒以内に戻す）
MODEL_HZ = 1.0


def onnx_model_files(models_dir: Path) -> list[dict]:
    """`models_dir` にある `.onnx` の一覧（GUI のモデル選択欄に並べる形）。

    `note` は同名 `.json` の自由記述の備考（エクスポート時に書かれる）——どんな変更・どのコースの
    モデルかを選ぶときに確認できるようにするため。`.json` が壊れていても一覧自体は壊さない（空文字）。
    """
    files = []
    if models_dir.is_dir():
        for p in sorted(models_dir.iterdir()):
            if not p.is_file() or p.suffix != ".onnx":
                continue
            st = p.stat()
            cfg_path = p.with_suffix(".json")
            note = ""
            if cfg_path.exists():
                try:
                    note = str(msgspec.json.decode(cfg_path.read_bytes()).get("note", ""))
                except (OSError, msgspec.DecodeError, AttributeError):
                    pass                       # 備考が読めないだけ。一覧には出す
            files.append({"name": p.stem, "size": st.st_size, "mtime": st.st_mtime,
                          "has_config": cfg_path.exists(), "note": note})
    return files


@dataclass
class ModelSlot:
    """モデルの種類1つぶん。`key` から GUI とのメッセージの名前が全部決まる。

    （テストが置き場を差し替えられるよう frozen にしない）
    """

    #: `cam_model` など。状態（`status`）のキー・`config/` の保存名・メッセージ名の元
    key: str
    models_dir: Path
    conf_path: Path
    #: 受け手へ流すトピック（型は `ModelSelectCtrl`）
    topic: str
    #: モデルを変えたら engage を落とす planner の id
    auto_mode: str

    @property
    def list_kind(self) -> str:
        return f"{self.key}_list"

    @property
    def select_kind(self) -> str:
        return f"{self.key}_select"

    def exists(self, name: str) -> bool:
        """`name` が置き場に実在するモデルか。パスの部品（`/`・先頭の `.`）を含む名前は弾く。"""
        if not name or "/" in name or "\\" in name or name.startswith("."):
            return False
        return (self.models_dir / f"{name}.onnx").is_file()

    def list_message(self) -> dict:
        return {"type": f"{self.key}s", f"{self.key}_files": onnx_model_files(self.models_dir)}

    def load(self) -> str:
        """保存してある名前（無い・壊れている・消えたモデルなら空文字）。"""
        raw = read_conf(self.conf_path)
        name = raw.get("name") if raw is not None else None
        return name if isinstance(name, str) and self.exists(name) else ""

    def save(self, name: str) -> None:
        write_conf(self.conf_path, {"name": name})


def _slot(key: str, subdir: str, conf: str, topic: str, auto_mode: str) -> ModelSlot:
    return ModelSlot(key=key, models_dir=MODELS_DIR / subdir, conf_path=CONFIG_DIR / conf,
                     topic=topic, auto_mode=auto_mode)


#: モデルの種類の表。置き場は各ノードの `DEFAULT_MODELS_DIR` と同じ場所（テストが突き合わせる）。
#: e2e_fusion・stereo_e2e のノードは telemetry が居ないとき保存先を自分で読む
MODEL_SLOTS: dict[str, ModelSlot] = {s.key: s for s in (
    # セグメンテーション（`ml_cam`）→ cam_perception_node。`ftg_cam` が使う
    _slot("cam_model", "seg", "cam_model.json", TOPIC_CAM_MODEL, "ftg_cam"),
    # LiDAR E2E（`ml_lidar`）→ planning_node。セグメンテーションとは前処理の契約が違うので置き場を分ける
    _slot("e2e_model", "e2e_lidar", "e2e_lidar_model.json", TOPIC_E2E_MODEL, "e2e_lidar"),
    # カメラ E2E（`ml_cam_e2e`）→ cam_e2e_node
    _slot("cam_e2e_model", "cam_e2e", "cam_e2e_model.json", TOPIC_CAM_E2E_MODEL, "cam_e2e"),
    # LiDAR＋カメラの融合 E2E（`ml_e2e`）→ e2e_fusion_node
    _slot("e2e_fusion_model", "e2e_fusion", "e2e_fusion_model.json", TOPIC_E2E_FUSION_MODEL, "e2e_fusion"),
    # ステレオの画像＋視差の E2E（`ml_stereo_e2e`）→ stereo_e2e_node
    _slot("stereo_e2e_model", "stereo_e2e", "stereo_e2e_model.json", TOPIC_STEREO_E2E_MODEL, "stereo_e2e"),
)}
