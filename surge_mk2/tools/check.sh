#!/usr/bin/env bash
# CI と**同じ検査**をローカルで走らせる。`.github/workflows/ci.yml` と対で保つこと。
#
#   ./tools/check.sh              全部
#   ./tools/check.sh --fast       生成物の --check だけ（数秒。コミット前向き）
#
# ## なぜ要るか（2026-08-21 のレビュー 🟡5）
#
# 生成器の `--check` を3つ持っているのに、**実行を人間の記憶に頼っていた。**
# GitHub Actions が動くのは push 後なので、手元でも同じものを1コマンドで
# 打てないと「push してから気づく」を繰り返す。
#
# pre-commit に入れるなら（リポジトリ根で。シンボリックリンクにすると `$0` が `.git/hooks/` を指し、
# 下の `cd` が別の場所へ行くので、絶対パスで呼ぶ小さなフックにする）:
#   printf '#!/bin/sh\nexec "$(git rev-parse --show-toplevel)/surge_mk2/tools/check.sh" --fast\n' \
#     > .git/hooks/pre-commit && chmod +x .git/hooks/pre-commit
set -u

cd "$(dirname "$0")/.." || exit 1
PY=".venv/bin/python"
[ -x "$PY" ] || PY="python3"
FAST=0
[ "${1:-}" = "--fast" ] && FAST=1
fail=0

run() {                 # run <説明> <コマンド...>
  local label=$1; shift
  printf '── %s\n' "$label"
  if "$@"; then
    return 0
  fi
  echo "   ✗ 失敗: $label" >&2
  fail=1
}

# ── 生成物が正と一致しているか（これが本体） ──
run "vehicle.toml → gui/src/generated/vehicle.ts" "$PY" config/generate.py --check
run "protocol.toml → proto/packets.py, surge_proto.h" "$PY" raspi/proto/generate.py --check
run "msgs/types.py → gui/src/generated/msgs.ts" "$PY" config/gen_msgs.py --check
# 生成コードだけでなく**文書中の版番号**も見る。`docs/README.md` が2版遅れたまま
# 誰も気づかなかったことがある（2026-08-21 のレビュー 🟢13）
run "protocol.toml → 文書中の版番号" "$PY" config/check_docs.py --check

# 未使用 import・空 f-string・未定義名（issue #24 の再発防止）。
# ruff は dev 専用（Pi の requirements には入れない）なので、無ければ黙って飛ばす
RUFF=".venv/bin/ruff"
[ -x "$RUFF" ] || RUFF="$(command -v ruff || true)"
if [ -n "$RUFF" ]; then
  run "ruff (F401/F541/F821)" "$RUFF" check --select F401,F541,F821 \
    --exclude raspi/proto/generated raspi slam2d config tools sim
else
  echo "── ruff … 見つからないので飛ばす（.venv/bin/pip install ruff で有効化）"
fi

if [ "$FAST" = 0 ]; then
  run "pytest" "$PY" -m pytest raspi/tests -q
  # Mac 側の補助ツール。校正（`tools/calib`）の値は全ノードとファームが信じて走るので、解析が真値を
  # 復元するかのテスト（真値の分かっているシムで試験を走らせる。約 40 秒）も入れる。ファームの定数の写しの
  # 突き合わせ（`tools/fwsim/tests/test_fw_constants.py`）も。車速のカルマンフィルタのシムのテスト
  # （`tools/fwsim/tests/test_speed_kf*.py`）は重いので入れない。ファームを直したら個別に回すこと
  run "pytest (tools)" "$PY" -m pytest tools/tests tools/calib/tests tools/fwsim/tests/test_fw_constants.py \
    tools/fwsim/tests/test_control_logic.py -q
  run "pytest (sim)" "$PY" -m pytest sim/tests -q
  # ML のテスト（torch が要る）。ml_cam と ml_cam_e2e は `dataset` をフラットに import するので別プロセスで回す
  if "$PY" -c "import torch" 2>/dev/null; then
    for d in ml_cam ml_cam_e2e ml_lidar ml_common; do
      run "pytest ($d)" "$PY" -m pytest "$d/tests" -q
    done
  else
    echo "── pytest (ml_*) … torch が無いので飛ばす"
  fi
  if [ -d gui/node_modules ]; then
    printf '── tsc -b\n'
    (cd gui && npx tsc -b) || { echo "   ✗ 失敗: tsc" >&2; fail=1; }
  else
    echo "── tsc -b … gui/node_modules が無いので飛ばす（npm ci を先に）"
  fi
fi

[ "$fail" = 0 ] && echo "すべて通った" || echo "失敗あり" >&2
exit "$fail"
