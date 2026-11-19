"""`telemetry_node.py`（`TelemetryServer`）から切り出した部品。

- `conf`: `config/*.json`（機械ごとの状態）の読み書き
- `models`: 推論モデルの選択（置き場・保存先・トピックの表と、汎用の一覧・選択・保存・送信）
- `recorder`: MCAP・動画の記録の子プロセスの管理
- `periodic`: 「現在の意思」を周期的に流し直す処理の1本のループ
"""
