"""足回りの制御（TC・ABS・片輪浮き対策・TV）の調整ツール（Mac側）。

    .venv/bin/python -m tools.ctrl_tune.gui      # 記録を解析して vehicle.toml [control] を更新
    .venv/bin/python -m tools.ctrl_tune.bench    # 制御ロジックの比較（現行の値・コミット済みのファーム）

STM32 のファーム（`MainF446RE_V3` の `src/control`）を**そのままホストでコンパイル**し、
実機の記録で同定した後輪＋タイヤ＋車体のモデルと閉ループにして、パラメータを最適化する。
調整するのは実物のコードなので、Python に写した制御とのずれが起きない。

`raspi/`・`sim/` からは独立している（システム同定 `tools/sysid` と同じ位置づけ）。
"""
