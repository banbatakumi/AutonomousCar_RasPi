"""システム同定ツール（Mac側） — GUIの「システム同定」タブが録ったmcapログから
`config/vehicle.toml` `[dynamics]` の実測値を推定して書き戻す。

    .venv/bin/python -m tools.sysid.gui

`raspi/`・`sim/` からは独立している（Piの実行環境を汚さないため）。
依存は `tools/requirements.txt` を参照。
"""
