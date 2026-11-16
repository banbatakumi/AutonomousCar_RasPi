"""ml_stereo_e2e — 前方ステレオの左右の画像＋視差の帯＋車両状態 → 前方の経路点（LiDAR を使わない模倣学習）。

手順: `extract`（記録 → 標本。視差は記録の左右の組を照合し直して作る）→ `train` → `export_onnx` → `eval`。
GUI は `app.py`（launcher の「ml_stereo_e2e」）。入出力の形は `raspi/core/stereo_e2e_preproc.py` が正。
経路のラベルと記録の読み方は `ml_e2e` のものをそのまま使う。
"""
