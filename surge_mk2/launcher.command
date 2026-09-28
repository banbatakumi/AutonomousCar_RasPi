#!/bin/bash
# ダブルクリックで補助GUIのランチャー（シム・同定・SLAM再処理・ml系）を起動する。
# ターミナルは一瞬開くが、コマンドを打つ必要は無い。
cd "$(dirname "$0")"
exec .venv/bin/python -m tools.launcher
