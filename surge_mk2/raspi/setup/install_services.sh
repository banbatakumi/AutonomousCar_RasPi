#!/usr/bin/env bash
# Pi 起動時に Phase 0 のノード一式を自動で上げる systemd unit を入れる。
#
#   sudo ./raspi/setup/install_services.sh                # ★ arm 有効（バンビの選択）
#   sudo ./raspi/setup/install_services.sh --safe         #    arm 封印（DISARM 固定）
#   sudo ./raspi/setup/install_services.sh --with-logger  #    MCAP も SD に録る
#   sudo ./raspi/setup/install_services.sh --remove
#
# ## ★ MCAP 記録（surge-logger）は既定で止めてある — SD カードを削らないため
#
# 実測で `.mcap` は **10.3MB/分**（大半がカメラ画像）。**1日 15GB を SD に
# 書き続けることになり、書き換え寿命に効く。** そして数値だけなら
# `.sfl`（io_node が常時記録・2.5MB/分）とほぼ同じ内容なので、
# **SD に二重に書く価値があるのは実質カメラ画像だけ。**
#
# そこで既定は「Pi は `.sfl` だけ」にし、画像込みの記録は **PC 側で受ける**:
#
#     tools/record.sh                 # Mac で実行。ssh 越しに MCAP を PC へ流す
#
# こうすると SD への書き込みは 12.8MB/分 → 2.5MB/分（**8割減**）になる。
# それでも Pi 単体で画像を録りたいときだけ `--with-logger` を付ける。
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
# **surge-logger は既定でこの一覧に入れない**（SD 書き込みを減らすため。上記）
UNITS=(surge-io surge-camera surge-telemetry)
WITH_LOGGER=0

ARM="--allow-arm --max-speed 0.3 --max-steer 1.05"   # 1.05 rad ≒ 60°
MODE="arm 有効"
case "${1:-}" in
  --safe)   ARM=""; MODE="arm 封印（DISARM 固定）" ;;
  --with-logger) WITH_LOGGER=1; UNITS+=(surge-logger) ;;
  --remove)
    systemctl disable --now "${UNITS[@]}" surge-logger 2>/dev/null || true
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
# 記録は 1分あたり .sfl 約 2.5MB ＋ .mcap 約 10.3MB（実測。大半は画像）。放置すると増え続けるので
# surge-logclean.timer が毎時「7日より古いもの」と「合計 8GB 超過ぶん」を消している
StandardOutput=append:$ROOT/logs/$name.out
StandardError=append:$ROOT/logs/$name.out

[Install]
WantedBy=multi-user.target
EOF
}

mkdir -p "$ROOT/logs"
chown "$USER_NAME" "$ROOT/logs"

# ── ★★ RemoveIPC を止める（2026-08-08 に踏んだ罠） ──
#
# systemd-logind の既定 `RemoveIPC=yes` は、**そのユーザーの最後のセッションが
# 終わった時点で uid 所有の POSIX 共有メモリを全部消す。** User=pi で動く
# systemd サービスの分まで巻き添えになるので、**SSH を切った瞬間に
# `/dev/shm/surge_cam0` が unlink される。**
#
# 症状が分かりにくい: camera_node はマッピングを保持したまま動き続ける
# （`image/front` は 30Hz で流れる）が、**新しく attach するプロセスだけが失敗する。**
# つまり「GUI のカメラだけ映らない」「ロガーの画像だけ入らない」という形で出る。
mkdir -p /etc/systemd/logind.conf.d
cat > /etc/systemd/logind.conf.d/10-surge-removeipc.conf <<'EOF'
# raspi/setup/install_services.sh が生成。
# camera_node の /dev/shm/surge_cam* が SSH ログアウトで消えるのを防ぐ。
[Login]
RemoveIPC=no
EOF
systemctl restart systemd-logind || true

write_unit surge-io        "UART/GPIO ノード" "raspi.nodes.io_node --quiet --log $ARM"
write_unit surge-camera    "カメラ"           "raspi.nodes.camera_node --quiet"
write_unit surge-telemetry "WebSocket サーバ"  "raspi.nodes.telemetry_node" "surge-io.service"
# ロガーの unit は**常に置く**（`--with-logger` を付けたときだけ enable する）。
# 置いておけば `sudo systemctl start surge-logger` で一時的に録れる
write_unit surge-logger    "MCAP 記録"        "raspi.nodes.logger_node --quiet" \
           "surge-io.service surge-camera.service"
if [ "$WITH_LOGGER" = 0 ]; then
  # 既定では止める。前回 --with-logger で入れたまま残っていることがある
  systemctl disable --now surge-logger 2>/dev/null || true
fi

# ── 古いログを消すタイマー（放置するとカードが埋まる） ──
#
# ## ★ 日付だけで消すのでは間に合わない（2026-08-08 の実測で判明）
#
# 実測は **`.sfl` 2.5MB/分 ＋ `.mcap` 10.3MB/分 = 約 12.8MB/分**。
# 電源を入れっぱなしにすると **1日で 18GB** 増える。空きは 22GB しかないので、
# 「毎日・7日より古いものを消す」では**掃除が来る前にカードが埋まる**。
#
# そこで2本立てにする:
#   ① 7日より古いものを消す（従来どおり。普段はこれで足りる）
#   ② 合計が MAX_LOG_MB を超えている間、**古い順に消す**（連続運転の保険）
# タイマーも毎日ではなく**毎時**に変更した。
MAX_LOG_MB=8000

cat > /etc/systemd/system/surge-logclean.service <<EOF
[Unit]
Description=SURGE Mk.2 — 古い記録（.sfl / .mcap）を消す
[Service]
Type=oneshot
ExecStart=/usr/bin/find $ROOT/logs -name '*.sfl' -mtime +7 -delete
ExecStart=/usr/bin/find $ROOT/logs -name '*.mcap' -mtime +7 -delete
# 合計が上限を超えている間、古い順に消す。**記録中のファイルも消えうる**が、
# カードが埋まって全ノードが書けなくなるよりは安い
ExecStart=/bin/sh -c 'cd $ROOT/logs || exit 0; \
  while [ "\$(du -sm . | cut -f1)" -gt $MAX_LOG_MB ]; do \
    f=\$(ls -tr *.sfl *.mcap 2>/dev/null | head -1); \
    [ -n "\$f" ] || break; \
    echo "上限 ${MAX_LOG_MB}MB 超過のため削除: \$f"; rm -f "\$f"; \
  done'
EOF
cat > /etc/systemd/system/surge-logclean.timer <<EOF
[Unit]
Description=SURGE Mk.2 — ログ掃除（毎時）
[Timer]
OnCalendar=hourly
Persistent=true
[Install]
WantedBy=timers.target
EOF

systemctl daemon-reload
systemctl enable --now "${UNITS[@]}" surge-logclean.timer

echo
echo "=== 入れた（$MODE） ==="
systemctl --no-pager --plain list-units 'surge-*' | head -10
echo
echo "  状態  : systemctl status surge-io"
echo "  ログ  : journalctl -u surge-io -f   /   tail -F $ROOT/logs/surge-io.out"
echo "  停止  : sudo systemctl stop ${UNITS[*]}"
echo "  無効化: sudo $0 --remove"
if [ "$WITH_LOGGER" = 1 ]; then
  echo
  echo "  ★ surge-logger 有効。SD に約 10.3MB/分（1日 15GB）書く。書き換え寿命に注意"
else
  echo
  echo "  記録: Pi は .sfl のみ（2.5MB/分）。**画像込みの MCAP は PC 側で受ける**:"
  echo "        tools/record.sh          # Mac で実行（SD には書かない）"
fi
if [ -n "$ARM" ]; then
  echo
  echo "  ★★ arm 有効。GUI で ARM を押せばモータが回る（上限 0.3 m/s / 1.05 rad ≒ 60°）"
  echo "  ★  surge-io を止めた時点で STM32 が E-Stop をラッチする（車両のボタン2で解除）"
fi
