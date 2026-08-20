#!/usr/bin/env bash
# Pi 起動時に Phase 0 のノード一式を自動で上げる systemd unit を入れる。
#
#   sudo ./raspi/setup/install_services.sh                # ★ arm 有効（バンビの選択）
#   sudo ./raspi/setup/install_services.sh --safe         #    arm 封印（DISARM 固定）
#   sudo ./raspi/setup/install_services.sh --with-logger  #    MCAP も SD に録る
#   sudo ./raspi/setup/install_services.sh --remove
#
# ## ★ 記録は GUI の「ログ」タブから操作する（2026-08-10 に運用変更）
#
# `.sfl`（UART生ログ）・`.mcap`（画像込み）とも、既定では**何も自動記録しない**。
# 走行中に GUI の「ログ」タブで開始/停止する:
#
# - `.sfl` は `io_node` が実時間ループの中で直接 Pi の SD に書く
#   （`log/ctrl` というバスのメッセージで開始/停止を伝えるだけ。ネットワーク越しには
#   流さない — Wi-Fi が切れても Pi 単体で記録が続くのがこの形式の価値なので）。
#   GUI の「ログ」タブから一覧・ダウンロード・削除できる
# - `.mcap` は `logger_node -o -` の標準出力を WebSocket でそのまま中継し、
#   **Pi の SD には一切書かない**。ブラウザが受け取ったバイト列を直接 PC へ
#   ダウンロードする（`tools/record.sh` が ssh パイプでやっていることを
#   GUI から起動できるようにしただけ）
#
# `surge-logger`（`--with-logger`）は「GUI を開けない・ヘッドレスで長時間
# 録りたい」ときの代替として残してある。既定では入れない
# （実測で `.mcap` は 10.3MB/分・1日 15GB と大きく、SD の書き換え寿命に効くため）。
#
#     tools/record.sh                 # Mac で実行。ssh 越しに MCAP を PC へ流す
#                                      # （GUI を開けないときの代替経路）
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
UNITS=(surge-io surge-camera surge-telemetry surge-planning)
WITH_LOGGER=0

# ⚠ `--max-speed` / `--max-steer` は **GUI の `PI_MAX_SPEED_CAP` / `PI_MAX_STEER_CAP`
# （gui/src/store/ui.ts）と一致させること。** ずれると、GUI の設定パネルでは
# 上限まで上げられるのに Pi 側で黙って切り捨てられる、という分かりにくい状態になる。
# 0.3 → 0.6 に変更（2026-08-09）。0.3 では加速ランプもブレーキも体感できなかったため。
# 0.6 → 2.0 / 45° → 60° に変更（2026-08-10、指示による）。
# 2.0 → 3.0 に変更（2026-08-11、指示による）。
# ⚠ 60°（1.047 rad）はモータ機械角の可動域であって路面舵角ではなかった。
#    リンク比 0.5（2026-08-20 実測確定）によりタイヤは半分しか切れないため、
#    路面舵角の上限は 30°（0.524 rad）に修正した。据え切りを続けるとステア MD が
#    過熱するので `temp[2]` を見ておくこと。速度 3.0 m/s ＝ 10.8 km/h は既定の GUI 設定
#    （0.6 m/s）の5倍で、ミニカーの大きさに対してはかなり速い。上げて走らせるときは周囲との距離に注意。
#
# ⚠ **速度の上限は3段ある。ここを上げても他が低いままなら、そちらで頭打ちになる。**
#    1. GUI  `PI_MAX_SPEED_CAP`（gui/src/store/ui.ts）… スライダの上限
#    2. Pi   この `--max-speed`               … `command_from_cmd` で黙って切り捨てる
#    3. STM32 `PARAM_MAX_SPEED`（CONFIG_SET 0x0001）… **Pi 側が未実装で読み書きできない**
#    3 の既定値はこのリポジトリからは分からない。3.0 m/s まで上げても実車がそこまで
#    出ないなら、STM32 のファームウェア側で頭打ちになっている可能性を先に疑うこと。
ARM="--allow-arm --max-speed 3.0 --max-steer 0.524"   # 0.524 rad ≒ 30°（路面舵角の実機上限）
MODE="arm 有効"
case "${1:-}" in
  --safe)   ARM=""; MODE="arm 封印（DISARM 固定）" ;;
  --with-logger) WITH_LOGGER=1; UNITS+=(surge-logger) ;;
  --remove)
    systemctl disable --now "${UNITS[@]}" surge-logger 2>/dev/null || true
    rm -f /etc/systemd/system/surge-*.service
    rm -f /etc/udev/rules.d/99-surge-fan.rules
    systemctl daemon-reload
    udevadm control --reload-rules 2>/dev/null || true
    # 節電のために切ったものを元に戻す（下の「節電」節を参照）
    systemctl enable --now bluetooth 2>/dev/null || true
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

# ── ★ 節電（2026-08-10 の PMIC 実測にもとづく） ──
#
# `vcgencmd pmic_read_adc` で全レールを実測して切り分けた結果（3ノード稼働・
# GUI 未接続 = 走行相当で **合計 4.06W**）:
#
# | カメラ2台（モジュール+ISP+node） | 1.15W | 28% |
# | Wi-Fi/BT レール                 | 0.42W | 10% |
# | telemetry_node                  | 0.26W |  6% |
# | HDMI / ディスプレイスタック      | 0.17W |  4% |
# | io_node + OS + レール固定        | 2.07W | 51% |
#
# ## ★★ CPU クロックを下げても節電にはならない（実測で確認）
#
# 負荷をかけて「電力」と「処理速度」を同時に測ると、**1仕事あたりの
# エネルギーはクロックによらずほぼ一定**だった:
#
#   2400MHz: 6.64W / 16.6 ops/s = 0.400 J/op
#   2000MHz: 5.51W / 13.9 ops/s = 0.395 J/op
#   1500MHz: 4.15W / 10.5 ops/s = 0.394 J/op
#
# アイドルの固定消費（約2.5W）が大きいので、クロックを絞ると瞬時電力は
# 下がるが**その分だけ時間が伸びて総エネルギーは変わらない**（race to idle）。
# よって `scaling_max_freq` や governor はいじらない。**ondemand のままが正解。**
# 下げる意味があるのはピーク電流・発熱・ファン回転を抑えたいときだけ。
#
# ここで削るのは「仕事をしていないのに消えている分」だけにする。
#
# ## ★★ HDMI を落とす案は**採用できなかった**（2026-08-10）
#
# `vcgencmd display_power 0` を oneshot unit で叩く案を入れかけたが、
# **Pi 5 では動かない**:
#
#     $ sudo vcgencmd display_power 0
#     vc_gencmd_read_response returned -1
#     error=1 error_msg="Command not registered"     # 終了コード 255
#
# 原因は Pi 5 のファームウェアが **`variant start_cd`**（機能削減版。
# `dmesg | grep firmware` で確認できる）で、ディスプレイ系の gencmd を
# 持っていないため。Pi 5 は表示が Linux 側の KMS に移っており、VideoCore に
# 頼む昔のやり方が通らない。
#
# ⚠ **この失敗は最初の実測を汚染していた。** エラーを `2>/dev/null` で潰したまま
#    測ったので「HDMI を切ると 0.17W 減る」と読めてしまったが、実際にはコマンドは
#    何もしていない。あの差はノイズだった。**効果を測るときは終了コードを見ること。**
#
# HDMI レールは実測 0.061W で、モニタ未接続でも出ている（コネクタの HPD 用）。
# ソフトからは落とせないので**諦める**。`cmdline.txt` の `video=HDMI-A-1:d` は
# 未検証（再起動が要るうえ、効くのは KMS の処理分だけで 0.06W ではない）。

# Bluetooth は**一度も使う予定がない**ので落とす。
# ただし実測の差は測定ノイズ以下（<0.05W）だった。**節電目的では効かない。**
# 動かす理由のないデーモンを消しておく、以上の意味はないと理解しておくこと。
# ⚠ `config.txt` の `dtoverlay=disable-bt` は**使わない**。あれは BT を UART から
#    切り離す副作用で `/dev/serial0` の割り当てを動かすので、STM32 との UART が壊れる
systemctl disable --now bluetooth 2>/dev/null || true

# **`--log` は付けない。** `.sfl` の記録は GUI の「ログ」タブ（`log/ctrl`）で
# セッション中に開始/停止する運用にしたため、既定では記録なしで起動する
write_unit surge-io        "UART/GPIO ノード" "raspi.nodes.io_node --quiet $ARM"
write_unit surge-camera    "カメラ"           "raspi.nodes.camera_node --quiet"
write_unit surge-telemetry "WebSocket サーバ"  "raspi.nodes.telemetry_node" "surge-io.service"
# **常時上げてよい。** planning_node は `auto/cmd` に出すだけで、`cmd` には
# 一切 publish しない。engage するのは GUI の自動運転タブから人間が行うので、
# 起動していること自体が車を動かす条件にはならない（`--allow-arm` とは独立）
write_unit surge-planning  "自動運転"         "raspi.nodes.planning_node --quiet" \
           "surge-io.service"
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

# Pi5純正ファン（`raspi/io/fan.py`）を pi ユーザーから書けるようにする udev ルール。
# 既定では root:root 644 で書けない（実機で確認済み、2026-08-17）。`--safe` でも要る
# （ファンの手動制御と ARM 封印は無関係なため）
cp "$ROOT/raspi/setup/99-surge-fan.rules" /etc/udev/rules.d/99-surge-fan.rules
udevadm control --reload-rules
udevadm trigger --subsystem-match=hwmon
udevadm trigger --subsystem-match=thermal

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
  echo "  記録: 既定では何も録らない。GUI の「ログ」タブから開始/停止する"
  echo "        .sfl  … Pi の SD に書く（GUI から一覧・ダウンロード・削除できる）"
  echo "        .mcap … SD には書かず、ブラウザに直接ダウンロードされる"
  echo "        GUI を開けないときは代わりに: tools/record.sh（Mac で実行）"
fi
if [ -n "$ARM" ]; then
  echo
  # 上限は `$ARM` から拾って出す。**数字をここに直書きすると必ず腐る**（実際に腐った）
  echo "  ★★ arm 有効。GUI で ARM を押せばモータが回る（上限 $ARM）"
  echo "  ★  surge-io を止めた時点で STM32 が E-Stop をラッチする（車両のボタン2で解除）"
fi
