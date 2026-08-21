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
# pre-commit に入れるなら:
#   ln -s ../../surge_mk2/tools/check.sh .git/hooks/pre-commit
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

if [ "$FAST" = 0 ]; then
  run "pytest" "$PY" -m pytest raspi/tests -q
  if [ -d gui/node_modules ]; then
    printf '── tsc -b\n'
    (cd gui && npx tsc -b) || { echo "   ✗ 失敗: tsc" >&2; fail=1; }
  else
    echo "── tsc -b … gui/node_modules が無いので飛ばす（npm ci を先に）"
  fi
fi

[ "$fail" = 0 ] && echo "すべて通った" || echo "失敗あり" >&2
exit "$fail"
