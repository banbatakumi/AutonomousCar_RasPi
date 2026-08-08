#!/usr/bin/env bash
# Pi 上で Phase 0 のノード一式を起動/停止する。systemd unit を書くまでの繋ぎ。
#
#   ./raspi/setup/run_stack.sh start          # io + camera + telemetry
#   ./raspi/setup/run_stack.sh start --heartbeat   # ★ GPIO6 ハートビートも出す
#   ./raspi/setup/run_stack.sh status
#   ./raspi/setup/run_stack.sh stop
#   ./raspi/setup/run_stack.sh logs
#
# ## なぜスクリプトにしたか
#
# `ssh pi@... 'pkill -f raspi.nodes; python -m raspi.nodes.io_node &'` と書くと、
# **pkill が自分自身のコマンドラインにマッチして SSH セッションごと落ちる**
# （リモートシェルの cmdline にパターンと同じ文字列が入っているため）。
# スクリプトにすれば cmdline は `bash run_stack.sh` だけになり、この事故が起きない。
#
# ## arm は絶対にここで解禁しない
#
# `io_node` に `--allow-arm` は渡さない。**モータを回す日は手で打つこと。**
# 起動スクリプトに書いておくと、いつの間にか常時解禁になる。
set -u

cd "$(dirname "$0")/../.." || exit 1
# -u は必須。**付けないと出力がバッファされ、ログファイルが空のまま**になる
# （リダイレクト先が tty でないと Python は 8KB 単位でしか書かない）。
# 「起動したのにログに何も出ない」を毎回疑うことになる
PY=".venv/bin/python -u"
LOGS=logs
PATTERN='raspi\.nodes\.'

# GPIO6 ハートビートは既定で出さない。
# **一度出すと、止めた時点で STM32 が E-Stop をラッチし、車両のボタン2を押すまで
# 解除されない。** 配線やGUIの確認で何度も再起動する段階では邪魔になる。
HB=--no-heartbeat
for a in "${@:2}"; do
  [ "$a" = "--heartbeat" ] && HB=""
done

start_one() {           # start_one <名前> <モジュール> [引数...]
  local name=$1 mod=$2; shift 2
  # shellcheck disable=SC2086
  setsid nohup $PY -m "$mod" "$@" > "$LOGS/$name.out" 2>&1 < /dev/null &
  # shellcheck disable=SC2086
  echo "  起動 $name (pid $!)"
}

case "${1:-status}" in
  start)
    mkdir -p "$LOGS"
    pkill -f "$PATTERN" 2>/dev/null && sleep 1     # pkill は自分自身にはマッチしない
    echo "起動中…"
    [ -n "$HB" ] || echo "  ★ GPIO6 ハートビートを出す（停止時に E-Stop がラッチする）"
    start_one io       raspi.nodes.io_node       --quiet $HB --log
    sleep 3
    start_one camera   raspi.nodes.camera_node   --quiet
    sleep 4
    start_one telemetry raspi.nodes.telemetry_node
    sleep 3
    exec "$0" status
    ;;

  stop)
    pkill -f "$PATTERN" && echo "停止した" || echo "動いていない"
    ;;

  status)
    echo "=== 稼働中 ==="
    pgrep -af "$PATTERN" | sed 's#.*/python -m ##' || echo "  （無し）"
    echo
    for f in io camera telemetry; do
      [ -f "$LOGS/$f.out" ] || continue
      echo "=== $f ==="
      grep -v '^$' "$LOGS/$f.out" | tail -6
      echo
    done
    ;;

  logs)
    tail -n 40 -F "$LOGS"/io.out "$LOGS"/camera.out "$LOGS"/telemetry.out
    ;;

  *)
    echo "使い方: $0 {start|stop|status|logs} [--heartbeat]" >&2
    exit 2
    ;;
esac
