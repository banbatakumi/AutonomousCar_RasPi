"""STM32 のファーム（`MainF446RE_V3` の `src/control`）を**そのままホストでコンパイル**して、後輪＋タイヤ＋車体の
モデル（ファーム側 `host/host_sim.c`）と閉ループで回す道具（Mac 側）。

    .venv/bin/python -m tools.fwsim.bench    # 制御ロジックの比較（作業ツリー と コミット済みのファーム）
    .venv/bin/python -m pytest tools/fwsim/tests   # ファームの定数の写し・車速のカルマンフィルタ

- `fw.py`: コンパイルと呼び出し（`Firmware`・`Session`）
- `plant.py`・`scenarios.py`: 車のモデルの値と、比べる場面（発進・制動・旋回の加速など）
- `kf_raw_replay.py`: 2kHz の生のキャプチャに車速のカルマンフィルタ（`speed_kf.c`）を通す

車両の校正（`tools/calib`）も、解析の確かめ（真値の分かっている車で試験を走らせる）と TC・ABS の設計の確かめに
ここを使う。2026-10-18 までは `tools/ctrl_tune`（TC・ABS の同定と数値最適化）の一部だった。
"""
