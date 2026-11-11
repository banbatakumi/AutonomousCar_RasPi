"""ml_e2e — LiDAR＋前方カメラ（生の画像）＋車両状態から、前方の経路点を出すモデルの学習（Mac）。

実機の記録（`.mcap`）から標本を作り（`extract.py`）、学習して ONNX にし、Pi の
`raspi/nodes/e2e_fusion_node.py` が推論する。全体の設計は `docs/development.md` §12.4。
"""
