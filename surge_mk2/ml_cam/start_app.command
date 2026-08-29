#!/bin/bash
# ダブルクリックで ml_cam/app.py（操作パネル）を起動する。
# ターミナルは一瞬開くが、コマンドを打つ必要は無い。
cd "$(dirname "$0")/.."
exec .venv/bin/python ml_cam/app.py
