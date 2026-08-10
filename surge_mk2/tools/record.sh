#!/usr/bin/env bash
# ★ PC（Mac）側で実行する。Pi の記録を **SD カードに書かずに** PC へ流し込む。
#
# **通常は GUI の「ログ」タブの録画ボタンで足りる**（同じ `logger_node -o -` を
# WebSocket 経由で使っている）。このスクリプトは GUI を開けない・ヘッドレスで
# 長時間録りたいときの代替経路として残してある。
#
#   tools/record.sh                       # Ctrl-C で終了
#   tools/record.sh --duration 60         # 60秒で自動終了
#   tools/record.sh --image-hz 15         # 画像を増やす（PC のディスクなので余裕がある）
#   tools/record.sh --image-hz 0          # 画像なし（数値だけ）
#   tools/record.sh -o ~/logs/test.mcap
#
# ## なぜ PC 側で録るのか
#
# Pi の記録は SD カードに書かれる。実測で **`.mcap` は 10.3MB/分（1日 15GB）**で、
# 大半がカメラ画像。**SD は書き換え回数に寿命がある**ので、常時これを書くのは高い。
#
# `logger_node -o -` は MCAP を標準出力に流すので、ssh のパイプに乗せれば
# **記録は PC のディスクにだけ落ち、SD には1バイトも書かれない。**
# 帯域は画像込みで約 200KB/s（実測 3.6MB / 20秒）なので Wi-Fi でも足りる。
#
# Pi 側の `.sfl`（io_node が常時記録・2.5MB/分）はそのまま残るので、
# **PC を繋いでいない間も UART のデータは失われない。**
#
# ## 途中で切れたら
#
# ssh が切れると MCAP の索引が書かれず、そのままでは開けない。復旧できる:
#
#   ssh surge-mk2 は要らない。PC 側で:
#   .venv/bin/python -m raspi.tools.mcap_repair <落ちたファイル> --inplace
set -u

cd "$(dirname "$0")/.." || exit 1

HOST=${SURGE_HOST:-surge-mk2}
KEY=${SURGE_KEY:-$HOME/.ssh/id_surge_mk2}
OUT=""
ARGS=()

while [ $# -gt 0 ]; do
  case "$1" in
    -o|--out) OUT=$2; shift 2 ;;
    -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
    *) ARGS+=("$1"); shift ;;
  esac
done

mkdir -p logs
[ -n "$OUT" ] || OUT="logs/pc_$(date +%Y%m%d_%H%M%S).mcap"

# 鍵が使えるなら明示する（~/.ssh/config が古い鍵を指していても通るように）
SSH_OPTS=()
[ -f "$KEY" ] && SSH_OPTS=(-i "$KEY" -o IdentitiesOnly=yes)

echo "# $HOST → $OUT （SD には書かない）"
echo "# 終了は Ctrl-C。索引はそのとき書かれる"
echo

# **stdout は MCAP そのもの。** logger_node は人間向けの出力を全部 stderr に出す
ssh "${SSH_OPTS[@]}" "$HOST" \
  "cd ~/surge_mk2 && .venv/bin/python -u -m raspi.nodes.logger_node -o - ${ARGS[*]:-}" \
  > "$OUT"
rc=$?

echo
if [ -s "$OUT" ]; then
  echo "=== $OUT  $(du -h "$OUT" | cut -f1) ==="
  if [ $rc -ne 0 ]; then
    echo "!! ssh が異常終了した（rc=$rc）。索引が無い可能性がある。直すなら:"
    echo "   .venv/bin/python -m raspi.tools.mcap_repair '$OUT' --inplace"
  fi
else
  echo "!! 何も受け取れなかった（rc=$rc）" >&2
  exit 1
fi
