"""カメラ校正ツール（Mac 側）— チェッカーボード写真から魚眼レンズの内部パラメータを求める。

    .venv/bin/python -m tools.cam_calib.gui                      # 画面（ランチャーの「カメラ校正」）
    .venv/bin/python -m tools.cam_calib --cam front --board 9x6 --square 0.025 ~/Downloads/surge_front_*.png
    .venv/bin/python -m tools.cam_calib --make-board 9x6 --square 0.025 -o board.png

## 流れ

1. 印刷用ボードを作って印刷する（`board.py`。印刷後に1マスを定規で測る）
2. GUI（ブラウザ）のタブバーの 📷 で 20 枚前後撮る → Mac の ~/Downloads に落ちる
3. このツールで校正 → `config/vehicle.toml` の、写真を撮ったレンズのプロファイル `[sensors.cam_*.lenses.<レンズ>.fisheye]` に書き戻し、
   `config/generate.py`（GUI 用の定数）も再生成する
4. `tools/deploy.sh --restart` で Pi に反映（GUI の再ビルドとノードの再起動まで入る）

## モデル

OpenCV の `cv2.fisheye`（Kannala-Brandt, k1..k4）。式と座標の約束（フル画角・下端クロップ前の
画素で持つ）は `raspi/core/camera_model.py` の docstring が正。

## Pi に入れない

cv2 は Pi にも入っているが、校正は写真を見比べながら何度もやり直す作業なので Mac で回す
（`tools/sysid` と同じ位置づけ）。依存は `tools/requirements.txt`。
"""
