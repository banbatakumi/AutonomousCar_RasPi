#!/usr/bin/env bash
# ★ PC（Mac）側で実行する。作業ツリーを Pi(`~/surge_mk2`) に反映する。
#
#   tools/deploy.sh                  # GUI をビルドして rsync（これが既定・いちばん安全）
#   tools/deploy.sh --no-gui         # Python だけ直したとき（npm を回さない）
#   tools/deploy.sh --test           # 反映後に Pi 上でユニットテストを回す
#   tools/deploy.sh --services       # ★ systemd unit を入れ直す（--max-speed 等を変えたとき）
#   tools/deploy.sh --restart-io     # ★★ surge-io を再起動する（E-Stop がラッチする）
#
# ## 何を再起動すべきかは「何を直したか」で決まる
#
# | 直したもの | 必要な操作 |
# |---|---|
# | `gui/` | **rsync だけ。** telemetry_node は毎リクエストでファイルを読む |
# | `raspi/nodes/telemetry_node.py` | `--restart` 相当（surge-telemetry の再起動。E-Stop は無関係） |
# | `raspi/setup/install_services.sh` の引数（`--max-speed` 等） | `--services` ＋ `--restart-io` |
# | `raspi/nodes/io_node.py` | `--restart-io` |
#
# **`--services` は unit ファイルを書き換えるだけで、走っている io_node は
# 古い引数のまま動き続ける。** 反映には `--restart-io` が要る。
#
# ## ★★ `--restart-io` は E-Stop をラッチさせる
#
# io_node は GPIO6 に 100Hz のハートビートを出している（安全第1層）。
# **止めた瞬間に STM32 が E-Stop をラッチし、車両のボタン2を物理的に押すまで戻らない。**
# 車体に手が届く状態でだけ使うこと。仕様どおりの動作であり、故障ではない。
set -u

cd "$(dirname "$0")/.." || exit 1
ROOT=$(pwd)

HOST=${SURGE_HOST:-surge-mk2}
KEY=${SURGE_KEY:-$HOME/.ssh/id_surge_mk2}
DEST=${SURGE_DEST:-'~/surge_mk2/'}

BUILD_GUI=1
RUN_TEST=0
DO_SERVICES=0
DO_RESTART_IO=0
DO_RESTART=0

while [ $# -gt 0 ]; do
  case "$1" in
    --no-gui)     BUILD_GUI=0 ;;
    --test)       RUN_TEST=1 ;;
    --services)   DO_SERVICES=1 ;;
    --restart-io) DO_RESTART_IO=1 ;;
    --restart)    DO_RESTART=1 ;;
    -h|--help)    sed -n '2,32p' "$0"; exit 0 ;;
    *) echo "知らない引数: $1（--help）" >&2; exit 2 ;;
  esac
  shift
done

# 鍵を明示する（~/.ssh/config が古い鍵を指していても通るように。record.sh と同じ）
SSH_OPTS=()
[ -f "$KEY" ] && SSH_OPTS=(-i "$KEY" -o IdentitiesOnly=yes)
ssh_pi() { ssh "${SSH_OPTS[@]}" "$HOST" "$@"; }

# ── 0. 疎通 ──
echo "# $HOST へ接続を確認"
ssh_pi -o ConnectTimeout=8 -o BatchMode=yes true || {
  echo "!! $HOST に繋がらない。" >&2
  echo "   直結の生死は IF を明示して見る: ping -b <en1x> 169.254.55.2" >&2
  echo "   mDNS が Wi-Fi 側しか返さないときは: ssh $HOST 'sudo systemctl restart avahi-daemon'" >&2
  exit 1
}

# ── 1. GUI をビルド ──
if [ "$BUILD_GUI" = 1 ]; then
  echo "# GUI をビルド"
  ( cd gui && npm run build ) || exit 1
else
  echo "# GUI のビルドは省略（--no-gui）"
fi

# ── 2. rsync ──
# **`.venv` と `node_modules` は絶対に運ばない。** Pi の venv は
# `--system-site-packages` で作られていて中身が Mac と別物。上書きすると壊れる。
# `docs/setup_credentials.md` は機密なので運ばない（Pi 側には要らない）。
# **`--info=stats1` は使わない。** macOS の rsync は 2.6.9 系（openrsync）で
# `--info` を知らない。GNU rsync 前提のオプションを書かないこと
echo "# rsync → $HOST:$DEST"
rsync -az --stats \
  --exclude '.venv' \
  --exclude 'node_modules' \
  --exclude '__pycache__' \
  --exclude '*.pyc' \
  --exclude 'docs/setup_credentials.md' \
  --exclude 'logs/' \
  --exclude '.DS_Store' \
  --exclude '*.mcap' \
  --exclude '*.sfl' \
  -e "ssh ${SSH_OPTS[*]}" \
  "$ROOT/" "$HOST:$DEST" || exit 1

# `gui/dist` だけは --delete する。ビルドごとにファイル名のハッシュが変わるので、
# そうしないと古い `index-*.js` が Pi に溜まり続ける
if [ "$BUILD_GUI" = 1 ] && [ -d gui/dist ]; then
  rsync -az --delete -e "ssh ${SSH_OPTS[*]}" "$ROOT/gui/dist/" "$HOST:$DEST/gui/dist/" || exit 1
fi

# ── 3. テスト ──
if [ "$RUN_TEST" = 1 ]; then
  echo
  echo "# Pi 上でユニットテスト"
  ssh_pi 'cd ~/surge_mk2 && .venv/bin/python -m unittest discover -s raspi/tests -t . 2>&1 | tail -5' || exit 1
fi

# ── 4. systemd unit の入れ直し ──
if [ "$DO_SERVICES" = 1 ]; then
  echo
  echo "# systemd unit を入れ直す"
  ssh_pi 'sudo bash ~/surge_mk2/raspi/setup/install_services.sh' || exit 1
  echo
  echo "  ※ unit を書き換えただけ。**走っている io_node は古い引数のまま。**"
  echo "     反映するには --restart-io（E-Stop がラッチする）"
fi

# ── 5. 再起動 ──
if [ "$DO_RESTART" = 1 ]; then
  echo
  echo "# surge-telemetry / surge-camera を再起動（E-Stop には影響しない）"
  ssh_pi 'sudo systemctl restart surge-telemetry surge-camera' || exit 1
fi

if [ "$DO_RESTART_IO" = 1 ]; then
  echo
  echo "★★ surge-io を再起動する。**STM32 が E-Stop をラッチする。**"
  echo "   解除には車両のボタン2を物理的に押す必要がある。"
  ssh_pi 'sudo systemctl restart surge-io' || exit 1
  sleep 2
  ssh_pi 'systemctl show surge-io -p ExecStart --value | tr " " "\n" | grep -A1 -- "--max-speed" | tr "\n" " "; echo'
fi

# ── 6. 状態 ──
echo
echo "=== $HOST の状態 ==="
ssh_pi 'systemctl is-active surge-io surge-camera surge-telemetry | paste -d" " - - - | sed "s/^/  io camera telemetry = /"'
echo
echo "  GUI  : http://surge-mk2.local:8000/"
echo "  ログ : ssh $HOST 'tail -F ~/surge_mk2/logs/surge-io.out'"
