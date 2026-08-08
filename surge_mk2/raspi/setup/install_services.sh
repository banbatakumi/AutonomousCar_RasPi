#!/usr/bin/env bash
# Pi 起動時に Phase 0 のノード一式を自動で上げる systemd unit を入れる。
#
#   sudo ./raspi/setup/install_services.sh          # ★ arm 有効（バンビの選択）
#   sudo ./raspi/setup/install_services.sh --safe   #    arm 封印（DISARM 固定）
#   sudo ./raspi/setup/install_services.sh --remove
#
# ## ★★ 既定で `--allow-arm` が入る
#
# **電源を入れた状態で GUI から ARM を押せばモータが回る。**
# 車輪を浮かせるか、周囲を確認してから電源を入れること。
#
# ただし電源投入だけでは動かない。`cmd` が 150ms 来なければ io_node は DISARM を送るので、
# **誰かが GUI で明示的に ARM を押すまでモータは励磁されない。**
#
# 封印したいときは `--safe` で入れ直すか、`sudo systemctl stop surge-io` して手で起動する。
#
# ## GPIO6 ハートビートについて
#
# unit は**ハートビートを出す**（安全第1層）。この結果:
#
# - **`surge-io` を止める / Pi が落ちる = STM32 が E-Stop をラッチする。**
#   解除には車両のボタン2を押す必要がある
# - `Restart=on-failure` で io_node が再起動しても、**E-Stop は解除されない**。
#   ラッチは人間の操作でしか戻らない（それが仕様）
set -eu

cd "$(dirname "$0")/../.." || exit 1
ROOT=$(pwd)
USER_NAME=${SUDO_USER:-pi}
PY="$ROOT/.venv/bin/python -u"
UNITS=(surge-io surge-camera surge-telemetry)

ARM="--allow-arm --max-speed 0.3 --max-steer 1.05"   # 1.05 rad ≒ 60°
MODE="arm 有効"
case "${1:-}" in
  --safe)   ARM=""; MODE="arm 封印（DISARM 固定）" ;;
  --remove)
    systemctl disable --now "${UNITS[@]}" 2>/dev/null || true
    rm -f /etc/systemd/system/surge-*.service
    systemctl daemon-reload
    echo "削除した"
    exit 0 ;;
esac

if [ "$(id -u)" -ne 0 ]; then
  echo "sudo で実行すること" >&2
  exit 1
fi

write_unit() {          # write_unit <名前> <説明> <ExecStart の引数> [After...]
  local name=$1 desc=$2 args=$3 after=${4:-}
  cat > "/etc/systemd/system/$name.service" <<EOF
# raspi/setup/install_services.sh が生成。手で編集しても再実行で上書きされる
[Unit]
Description=SURGE Mk.2 — $desc
After=network.target $after
Wants=network.target

[Service]
Type=simple
User=$USER_NAME
WorkingDirectory=$ROOT
ExecStart=$PY -m $args
Restart=on-failure
RestartSec=3
# 記録は 1分あたり約 2.5MB。放置すると増え続けるので
# raspi/setup/install_services.sh のタイマーで 7日より古いものを消している
StandardOutput=append:$ROOT/logs/$name.out
StandardError=append:$ROOT/logs/$name.out

[Install]
WantedBy=multi-user.target
EOF
}

mkdir -p "$ROOT/logs"
chown "$USER_NAME" "$ROOT/logs"

write_unit surge-io        "UART/GPIO ノード" "raspi.nodes.io_node --quiet --log $ARM"
write_unit surge-camera    "カメラ"           "raspi.nodes.camera_node --quiet"
write_unit surge-telemetry "WebSocket サーバ"  "raspi.nodes.telemetry_node" "surge-io.service"

# ── 古い .sfl を消すタイマー（放置するとカードが埋まる） ──
cat > /etc/systemd/system/surge-logclean.service <<EOF
[Unit]
Description=SURGE Mk.2 — 7日より古い .sfl を消す
[Service]
Type=oneshot
ExecStart=/usr/bin/find $ROOT/logs -name '*.sfl' -mtime +7 -delete
EOF
cat > /etc/systemd/system/surge-logclean.timer <<EOF
[Unit]
Description=SURGE Mk.2 — ログ掃除（毎日）
[Timer]
OnCalendar=daily
Persistent=true
[Install]
WantedBy=timers.target
EOF

systemctl daemon-reload
systemctl enable --now "${UNITS[@]}" surge-logclean.timer

echo
echo "=== 入れた（$MODE） ==="
systemctl --no-pager --plain list-units 'surge-*' | head -8
echo
echo "  状態  : systemctl status surge-io"
echo "  ログ  : journalctl -u surge-io -f   /   tail -F $ROOT/logs/surge-io.out"
echo "  停止  : sudo systemctl stop surge-io surge-camera surge-telemetry"
echo "  無効化: sudo $0 --remove"
if [ -n "$ARM" ]; then
  echo
  echo "  ★★ arm 有効。GUI で ARM を押せばモータが回る（上限 0.3 m/s / 1.05 rad ≒ 60°）"
  echo "  ★  surge-io を止めた時点で STM32 が E-Stop をラッチする（車両のボタン2で解除）"
fi
