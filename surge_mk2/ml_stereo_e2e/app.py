"""ml_stereo_e2e/app.py — ステレオ E2E（左右の画像＋視差の帯 → 経路点）の、標本作成・ラベル確認・学習・エクスポート・評価の操作パネル。

    .venv/bin/python ml_stereo_e2e/app.py

画面は `ml_e2e/app.py`（E2E 融合）と同じもの（タブ・置き場の並び・手本の決まり方も同じ）を、このパッケージの
`extract`・`train`・`export_onnx`・`eval` で開く。違い:

- 置き場は `ml_stereo_e2e/runs/<モデル名>/`、書き出し先は `models/stereo_e2e/`
- ①の標本作成は、記録の左右の組を照合し直して視差の帯も作る（`extract.py`。1万枚で数分）
- 左右反転の水増しは無い・評価で入力を伏せる選択は無い
- ②ラベル確認は `ml_e2e.check_labels` をそのまま使う（標本の形が同じ。点群は記録の LiDAR）
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from ml_e2e.app import Flavor, main  # noqa: E402

__all__ = ["STEREO"]

STEREO = Flavor("ml_stereo_e2e", "ステレオ E2E（カメラだけ）", REPO_ROOT / "ml_stereo_e2e" / "runs",
                REPO_ROOT / "models" / "stereo_e2e", "視差", can_flip=False, eval_masks=False)


if __name__ == "__main__":
    raise SystemExit(main(STEREO))
