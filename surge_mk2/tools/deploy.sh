#!/usr/bin/env bash
# ★ PC（Mac）側で実行する。作業ツリーを Pi(既定 `~/surge_mk2`、`SURGE_DEST` で変更) に反映する。
#
#   tools/deploy.sh                  # GUI をビルドして rsync（これが既定・いちばん安全）
#   tools/deploy.sh --no-gui         # Python だけ直したとき（npm を回さない）
#   tools/deploy.sh --test           # 反映後に Pi 上でユニットテストを回す
#   tools/deploy.sh --no-prune       # Mac に無くなった .py を Pi から消さない（既定は消す）
#   tools/deploy.sh --services       # ★ systemd unit を入れ直す（--max-speed 等を変えたとき）
#   tools/deploy.sh --restart        # io 以外のノードを再起動する（E-Stop には影響しない。対象は下の表）
#   tools/deploy.sh --restart-io     # ★★ surge-io を再起動する（E-Stop がラッチする）
#
# ## 何を再起動すべきかは「何を直したか」で決まる
#
# | 直したもの | 必要な操作 |
# |---|---|
# | `gui/` | **rsync だけ。** telemetry_node は毎リクエストでファイルを読む |
# | `raspi/` のうち io_node 以外（`raspi/nodes/*_node.py`・`raspi/auto/`・`raspi/nav/`・`raspi/core/`）・`slam2d/`・`ml_*/` の推論の前処理 | `--restart`（`RESTART_UNITS` の再起動。unit の無い機械では飛ばす。E-Stop は無関係） |
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
PRUNE=1
#: `--restart` で再起動する unit（io 以外。`raspi/setup/install_services.sh` の UNITS から io・mdns を除いたもの）
RESTART_UNITS="surge-telemetry surge-camera surge-planning surge-cam-track surge-cam-perception surge-cam-e2e surge-e2e-fusion surge-line-perception surge-stereo surge-stereo-e2e"
#: 一度に消してよい古い .py の数の上限（環境変数で変える）。超えたら消さずに止める
PRUNE_MAX=${SURGE_PRUNE_MAX:-30}

while [ $# -gt 0 ]; do
  case "$1" in
    --no-gui)     BUILD_GUI=0 ;;
    --test)       RUN_TEST=1 ;;
    --services)   DO_SERVICES=1 ;;
    --restart-io) DO_RESTART_IO=1 ;;
    --restart)    DO_RESTART=1 ;;
    --no-prune)   PRUNE=0 ;;
    -h|--help)    sed -n '2,28p' "$0"; exit 0 ;;
    *) echo "知らない引数: $1（--help）" >&2; exit 2 ;;
  esac
  shift
done

# 鍵を明示する（~/.ssh/config が古い鍵を指していても通るように。record.sh と同じ）
# 空の配列は `${A[@]+"${A[@]}"}` で展開する（macOS の bash 3.2 は `set -u` のもとで空の
# `"${A[@]}"` を未定義変数として止まる）
SSH_OPTS=()
[ -f "$KEY" ] && SSH_OPTS=(-i "$KEY" -o IdentitiesOnly=yes)
ssh_pi() { ssh ${SSH_OPTS[@]+"${SSH_OPTS[@]}"} "$HOST" "$@"; }
#: GUI の案内に出す宛先。名前が引けず IP で繋いだときは IP にする
GUI_HOST=surge-mk2.local

# ── 0. 疎通 ──
echo "# $HOST へ接続を確認"
if ! ssh_pi -o ConnectTimeout=8 -o BatchMode=yes true; then
  # Wi-Fi の Mac は surge-mk2.local が引けなくなることがある（mDNSResponder が固まる。
  # `tools/find_pi.py`）。既定の宛先のときだけ、IP を直接引いて繋ぎ直す。
  # ホスト鍵は名前で登録済みなので HostKeyAlias で同じ鍵を照合させる
  IP=""
  if [ "$HOST" = surge-mk2 ] && [ -x .venv/bin/python ]; then
    IP=$(.venv/bin/python -m tools.find_pi 2>/dev/null)
  fi
  if [ -n "$IP" ] && SSH_OPTS+=(-o "HostName=$IP" -o HostKeyAlias=surge-mk2.local) \
      && ssh_pi -o ConnectTimeout=8 -o BatchMode=yes true; then
    echo "# 名前が引けないので IP で繋いだ: $IP"
    GUI_HOST=$IP
  else
    echo "!! $HOST に繋がらない。" >&2
    echo "   直結の生死は IF を明示して見る: ping -b <en1x> 169.254.55.2" >&2
    echo "   mDNS が Wi-Fi 側しか返さないときは: ssh $HOST 'sudo systemctl restart avahi-daemon'" >&2
    exit 1
  fi
fi
# rsync の -e は空白で分けて読むので、鍵のパスに空白があっても壊れないよう各引数を単引用符で囲む
# （疎通の後で組む。IP に切り替えたときの -o HostName を rsync にも渡すため）
RSYNC_SSH="ssh"
for _o in ${SSH_OPTS[@]+"${SSH_OPTS[@]}"}; do RSYNC_SSH+=" '${_o}'"; done

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
# **機械ごとの状態も運ばない**（パラメータ・プリセット・カメラ設定・選んだモデル・総走行距離・保存した地図）。
# 運ぶと Mac の値（シムで触った値）で Pi の値を上書きしてしまう。地図の受け渡しは GUI の DL/UL。
# 先頭の `/` は転送元の直下に固定する（無いと別の階層の同名ファイルまで外れる）
# **`--info=stats1` は使わない。** macOS の rsync は 2.6.9 系（openrsync）で
# `--info` を知らない。GNU rsync 前提のオプションを書かないこと
# 学習の途中経過（`runs/`・`checkpoints/`）は Mac だけのもの（合わせて約2GB）。Pi が読むモデルは
# 書き出した先の `models/` にある
echo "# rsync → $HOST:$DEST"
rsync -az --stats \
  --exclude '.venv' \
  --exclude 'node_modules' \
  --exclude '__pycache__' \
  --exclude '/raspi/core/_native_build/' \
  --exclude '*.pyc' \
  --exclude 'docs/setup_credentials.md' \
  --exclude 'logs/' \
  --exclude '.DS_Store' \
  --exclude '*.mcap' \
  --exclude '*.sfl' \
  --exclude '/config/auto.json' \
  --exclude '/config/auto_presets.json' \
  --exclude '/config/drive_settings.json' \
  --exclude '/config/camera.json' \
  --exclude '/config/cam_model.json' \
  --exclude '/config/e2e_lidar_model.json' \
  --exclude '/config/cam_e2e_model.json' \
  --exclude '/ml_cam_e2e/runs/' \
  --exclude '/ml_e2e/runs/' \
  --exclude '/config/e2e_fusion_model.json' \
  --exclude '/ml_stereo_e2e/runs/' \
  --exclude '/config/stereo_e2e_model.json' \
  --exclude '/ml_lidar/runs/' \
  --exclude '/ml_cam/runs/' \
  --exclude '/ml_cam/checkpoints/' \
  --exclude '/config/odometer.json' \
  --exclude '/saved_maps/' \
  -e "$RSYNC_SSH" \
  "$ROOT/" "$HOST:$DEST" || exit 1

# 全体には `--delete` を掛けない（Pi にだけある状態・記録を消さないため）。代わりに、Mac で消した・
# 名前を変えた `.py` だけを Pi からも消す。残すと、古いテストが `unittest discover` を落とし、
# 古いモジュールが import できてしまう。**消すのは `raspi sim slam2d tools config ml_*` 配下の `.py` だけ**
# （設定の json・地図・モデル・記録は検出に入らない）。
# 数が `PRUNE_MAX` を超えたら、別ブランチや空の作業ツリーから反映した事故を疑って消さずに止める。
# `comm` は並びが揃っていないと別のファイルを「片側にだけある」と数える。macOS と Pi で `sort` の並びが
# 違う（`_` の位置）ので、どちらも `LC_ALL=C` に固定する
PRUNE_DIRS="raspi sim slam2d tools config $(ls -d ml_* 2>/dev/null | tr '\n' ' ')"
STALE=$(LC_ALL=C comm -13 \
  <(find $PRUNE_DIRS -name '*.py' -not -path '*/__pycache__/*' | LC_ALL=C sort) \
  <(ssh_pi "cd $DEST && find $PRUNE_DIRS -name '*.py' -not -path '*/__pycache__/*' 2>/dev/null | LC_ALL=C sort"))
if [ -n "$STALE" ]; then
  N_STALE=$(printf '%s\n' "$STALE" | wc -l | tr -d ' ')
  echo
  if [ "$PRUNE" != 1 ]; then
    echo "!! Mac に無い .py が Pi に残っている（--no-prune のため消していない）: ${N_STALE} 個"
    printf '%s\n' "$STALE" | sed 's/^/     /'
  elif [ "$N_STALE" -gt "$PRUNE_MAX" ]; then
    echo "!! Mac に無い .py が Pi に ${N_STALE} 個ある（上限 ${PRUNE_MAX} を超えたので消さない）。" >&2
    echo "   別のブランチ・古い作業ツリーから反映していないか確かめること。本当に不要なら SURGE_PRUNE_MAX を上げて再実行。" >&2
    printf '%s\n' "$STALE" | head -10 | sed 's/^/     /' >&2
    exit 1
  elif printf '%s\n' "$STALE" | grep -qvE '^(raspi|sim|slam2d|tools|config|ml_[A-Za-z0-9_]+)/[A-Za-z0-9_./-]+\.py$' || \
       printf '%s\n' "$STALE" | grep -qF '..'; then
    echo "!! 想定外のパスが含まれているので消さない:" >&2
    printf '%s\n' "$STALE" | sed 's/^/     /' >&2
    exit 1
  else
    echo "# Mac に無くなった .py を Pi から消す（${N_STALE} 個）"
    printf '%s\n' "$STALE" | ssh_pi "cd $DEST && xargs -d '\n' rm -v --" | sed 's/^/     /' || exit 1
  fi
fi

# 経路の計算・SLAM のレイの行進・ステレオの後処理の C 版（`raspi/nav/_native.c`・`slam2d/core/_native.c`・
# `raspi/core/stereo_native.c`）を組んでおく。組まないと各ノードの最初の import で組む（Pi で約1秒）。
# 組めなくても Python/numpy 版で走る（同じ結果で遅い）ので、止めずに知らせるだけ
if ssh_pi "cd $DEST && .venv/bin/python -c 'import sys; from raspi.nav import native; from slam2d.core import native as s; from raspi.core.stereo_native import load; sys.exit(native.lib is None or s.lib is None or load() is None)'"; then
  echo "# C 版（経路の計算・SLAM のレイ・ステレオの後処理）: OK"
else
  echo "!! C 版を組めなかった（Python 版で走る。経路の生成が数倍〜十数倍、走行中の plan()・ステレオの照合も遅い）" >&2
fi

# `gui/dist` だけは --delete する。ビルドごとにファイル名のハッシュが変わるので、
# そうしないと古い `index-*.js` が Pi に溜まり続ける
if [ "$BUILD_GUI" = 1 ] && [ -d gui/dist ]; then
  rsync -az --delete -e "$RSYNC_SSH" "$ROOT/gui/dist/" "$HOST:$DEST/gui/dist/" || exit 1
fi

# ── 3. テスト ──
if [ "$RUN_TEST" = 1 ]; then
  echo
  echo "# Pi 上でユニットテスト"
  # `| tail` の終了コードで失敗を隠さないよう、テストの終了コードを持ち帰る
  ssh_pi "cd $DEST && out=\$(.venv/bin/python -m unittest discover -s raspi/tests -t . 2>&1); rc=\$?; \
printf '%s\n' \"\$out\" | tail -5; exit \$rc" || { echo "!! Pi 上のテストが失敗した" >&2; exit 1; }
fi

# ── 4. systemd unit の入れ直し ──
if [ "$DO_SERVICES" = 1 ]; then
  echo
  echo "# systemd unit を入れ直す"
  ssh_pi "sudo bash $DEST/raspi/setup/install_services.sh" || exit 1
  echo
  echo "  ※ unit を書き換えただけ。**走っている io_node は古い引数のまま。**"
  echo "     反映するには --restart-io（E-Stop がラッチする）"
fi

# ── 5. 再起動 ──
if [ "$DO_RESTART" = 1 ]; then
  echo
  echo "# io 以外のノードを再起動（E-Stop には影響しない）: $RESTART_UNITS"
  # 推論・知覚のノード: auto/ctrl は telemetry_node が「現在の意思」を繰り返し流す設計（AutoCtrl の docstring
  # 参照）なので、再起動しても GUI 側の再選択なしに数秒で復帰する。途絶は stale_ms（500ms）超過で自然に
  # ブレーキへ倒れる（安全側）
  #
  # surge-planning: `raspi/auto/registry.py` の PLANNERS はプロセス起動時に固定されるので、`raspi/auto/` を
  # 直しても再起動しないと反映されない。io_node とは無関係な別プロセスなので E-Stop はラッチしない。
  # ★ raceline/slam2d_raceline（SLAM地図生成モード）の EXPLORE/BUILD 中に再起動すると、まだ確定（freeze）
  # していない地図はメモリ上にしか無いため失われる
  #
  # unit が無い機械（--services の前・一部のノードを入れていない）では飛ばして続ける。1つ欠けただけで
  # 残りの再起動と状態の表示を止めない
  # shellcheck disable=SC2016
  ssh_pi "for u in $RESTART_UNITS; do
    if systemctl cat \$u >/dev/null 2>&1; then sudo systemctl restart \$u || echo \"  !! \$u の再起動に失敗\";
    else echo \"  \$u の unit が無い（tools/deploy.sh --services で作る）\"; fi
  done" || exit 1
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
echo "  GUI  : http://$GUI_HOST:8000/"
echo "         （Wi-Fi で名前が引けないときは launcher の「実機 GUI を開く」か tools/find_pi.py --open）"
echo "  ログ : ssh $HOST 'tail -F ${DEST%/}/logs/surge-io.out'"
