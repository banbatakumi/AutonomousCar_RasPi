# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Raspberry Pi 3 + STM32F446RE ロボットカー遠隔操縦システム。
RasPi が Wi-Fi AP として動作し、FastAPI サーバーでブラウザ UI からの WebSocket コマンドを受け取り、UART 経由で STM32 に転送する。

## Repository Layout

```
robotcar/
├── raspi/          # Raspberry Pi 側サーバー（Python / FastAPI）
│   ├── main.py         # エントリポイント・WebSocket・MJPEG配信
│   ├── uart_handler.py # STM32 との UART 送受信（50Hz送信スレッド）
│   ├── camera_stream.py# picamera2 による MJPEG キャプチャスレッド
│   ├── gpio_handler.py # LED1/LED2/ブザー制御
│   ├── requirements.txt
│   └── setup/          # hostapd.conf / dnsmasq.conf / robotcar.service
└── app/            # ブラウザ UI（バニラ JS）
    ├── index.html
    ├── style.css
    └── main.js         # WebSocket・W/A/S/D キーボード制御・ゲームパッド制御
```

## Running the Server (on Raspberry Pi)

```bash
cd /home/pi/robotcar
source venv/bin/activate
uvicorn raspi.main:app --host 0.0.0.0 --port 8000
# または systemd 管理:
sudo systemctl start robotcar
```

## Key Architecture Points

- **静的ファイル配信**: `main.py` が `app/` を `/` にマウント。`/ws` と `/stream` は StaticFiles より先に登録されるため衝突しない。
- **UART フレーミング (送信)**: ヘッダ `0xFF` / フッタ `0xAA`、6バイト固定長。flags バイトにビットフラグ: bit0=do_stop, bit1=do_remote_control, bit2=do_brake, bit3=on_headlight, bit4=on_hazard。
- **UART フレーミング (受信)**: 11バイト固定長。`_recv_loop` はバッファから `buf[i]==0xFF` かつ `buf[i+10]==0xAA` を検索し、最後に見つかったフレームを採用（古いフレームをスキップ）。
- **EMA センサーフィルタ**: `_parse_packet` で指数移動平均を適用。速度/加速度α=0.35、距離α=0.50、電圧α=0.15。定数は `uart_handler.py` 上部で調整可能。
- **WebSocket 切断時の安全停止**: 接続ゼロになったとき `do_stop=True` を UART に即送信する（`main.py` の `finally` ブロック）。
- **GPIO グレースフル**: `GPIOHandler` は `RPi.GPIO` のインポート失敗を握りつぶすため、Mac 上でも動作確認できる。接続時に C5→E5→G5→C6 のアルペジオメロディを再生。
- **速度エンコーディング**: `move_speed[m/s] = int8 × 0.1`、`steer[-1.0..+1.0] = int8 / 127.0`。速度スライダーは整数（0..127）で保持し `× 0.1` して送信（マイナスなし）。加速度スライダーは 0..127。
- **カメラ**: picamera2 の `JpegEncoder + start_recording` による連続配信。センサーモード 1640×1232（フルFOV）、lores=320×192 で MJPEG 出力。`ScalerCrop=(0,0,3280,1971)` で下20%（車体）をカット。フレーム同期は `threading.Condition`。

## UI (app/)

レーシングコックピット風ダークテーマ（バニラJS / CSS）。

- **ゲージ（2×2 SVG アークゲージ）**:
  - 上段: SPEED（赤、0-5 m/s）/ ACCEL（黄、0-5 m/s²）
  - 下段: SIGNAL V（青、0-5V）/ POWER V（緑、0-15V、色が電圧で変化）
- **スライダー**: SPD (0–12.7 m/s) / ACC (0–12.7 m/s²)　デフォルト値: 1.0 m/s / 1.0 m/s²
- **スイッチ**: DRIVE（do_stop 反転）/ REMOTE（W/A/S/D・ゲームパッド操作モード）
- **ボタン**: LIGHT（ヘッドライト）/ HAZARD（ハザード）
- **キーボード**: W/A/S/D=移動方向（REMOTEモード時）、Space長押し=ブレーキ
- **ゲームパッド**: Elecom JC-U3613M（USB接続、**Chrome のみ対応**、Safari 非対応）
  - `axes[1]`（左スティック Y）→ 速度（上=前進、SPD スライダーで最大値設定）
  - `axes[2]`（右スティック X）→ ステアリング（±1.0）
  - `button[5]`（R1）→ ブレーキ（押している間）
  - `button[4]`（L1）→ ライト ON/OFF（押した瞬間にトグル）
  - 接続状態はヘッダーの `GAMEPAD --` / `GAMEPAD READY` で確認
  - ページロード後にコントローラーのボタンを1回押すと認識される
- **テレメトリ**: MOTOR エラー表示のみ（距離は画面左下の dist-bar で表示）

## Network / Access

- RasPi Wi-Fi AP: SSID `robotcar` / PW `robotcar1234`
- RasPi 固定 IP: `192.168.10.1`
- ブラウザアクセス: `http://192.168.10.1:8000`
- SSH: `ssh pi@192.168.10.1`（PW: `robotcar`）

### 開発時（有線 LAN 接続時）
- 有線 LAN 接続中は `robotcar.local` で解決可能（DHCP）
- ファイル更新: `scp <file> pi@robotcar.local:/home/pi/robotcar/app/`
- 静的ファイル（HTML/CSS/JS）の変更はサーバー再起動不要
- **転送後は必ず** `curl -s http://robotcar.local:8000/main.js | node --check /dev/stdin` で構文確認すること（並列 scp によるファイル破損を防ぐため）

## Setup Reference

詳細な初期セットアップ手順（APモード設定・venv・systemd登録）は `robotcar/README.md` を参照。
