# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Raspberry Pi 3 + STM32F446RE ロボットカー遠隔操縦システム。
RasPi が Wi-Fi AP として動作し、FastAPI サーバーでブラウザ UI からの WebSocket コマンドを受け取り、UART 経由で STM32 に転送する。

## Repository Layout

```
robotcar/
├── raspi/          # Raspberry Pi 側サーバー（Python / FastAPI）
│   ├── main.py         # エントリポイント・WebSocket・カメラWS・システム監視
│   ├── uart_handler.py # STM32 との UART 送受信（20Hz送信スレッド）
│   ├── camera_stream.py# picamera2 による JPEG キャプチャスレッド
│   ├── gpio_handler.py # LED1/LED2/ブザー制御
│   ├── requirements.txt
│   └── setup/          # hostapd.conf / dnsmasq.conf / robotcar.service
└── app/            # ブラウザ UI（バニラ JS）
    ├── index.html
    ├── style.css
    └── main.js         # WebSocket・キーボード制御・ゲームパッド制御
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

- **静的ファイル配信**: `main.py` が `app/` を `/` にマウント。`/ws`・`/ws/camera` は StaticFiles より先に登録されるため衝突しない。
- **UART フレーミング (送信)**: ヘッダ `0xFF` / フッタ `0xAA`、6バイト固定長、20Hz。
  - flags: bit0=do_stop, bit1=do_brake, bit2=on_headlight, bit3=on_hazard, bit4=play_sound, bit5=enable_auto_brake, bit7-6=mode(0-3)
  - move_speed: int8 × 0.1 m/s、acceleration: int8 × 0.1 m/s²、steer: int8 / 127.0
- **UART フレーミング (受信)**: 733バイト固定長、1Mbps。
  - `[0xFF][motor_err][speed][accel][volt_s][volt_p][accel_xy][pitch][roll][tmp_l][tmp_r][tmp_s][360×uint16 big-endian mm][0xAA]`
  - LiDAR: 360点 × uint16 big-endian（mm単位、0=範囲外、最大12000mm）
  - 電圧: uint8 × 0.1V、accel_xy: 4bit nibble符号付き × 0.5g、pitch/roll: int8（度）、温度: uint8（°C）
- **EMA センサーフィルタ**: 速度/加速度α=0.35、距離α=0.50、電圧α=0.15、IMU加速度α=0.50、IMUピッチ/ロールα=0.30、温度α=0.10。定数は `uart_handler.py` 上部で調整可能。
- **WebSocket 切断時の安全停止**: 接続ゼロになったとき `do_stop=True` を UART に即送信する（`main.py` の `finally` ブロック）。
- **GPIO グレースフル**: `GPIOHandler` は `RPi.GPIO` のインポート失敗を握りつぶすため、Mac 上でも動作確認できる。接続時に C5→E5→G5→C6 のアルペジオメロディを再生。
- **カメラストリーム**: WebSocket binary `/ws/camera` エンドポイント。picamera2 `JpegEncoder + start_recording`、lores=320×182 YUV420、30fps、JPEG quality 70。`FrameDurationLimits=(33333, 100000)` で暗所自動露光延長（最低10fps）。`ScalerCrop=(0,0,3280,1866)` で下3/7（車体）をカット。
- **カメラWS (JS側)**: `binaryType="arraybuffer"` → `new Blob([e.data], {type:"image/jpeg"})` → `URL.createObjectURL` で `<img>` に表示。切断時2秒後自動再接続。
- **センサー配信**: WebSocket JSON `/ws`、10Hz。CPU温度・負荷・メモリ・WiFi RSSI(dBm) を含む。
- **システム監視**: `_poll_wifi()` が3秒ごとに CPU温度(`/sys/class/thermal`)・CPU負荷(`/proc/stat`)・メモリ(`/proc/meminfo`)・WiFi RSSI(`iw dev wlan1 station dump`) を収集。
- **Wi-Fi AP**: USB アダプタ MT7610U（wlan1）、2.4GHz ch1、802.11n、CCMP。静的IP は rc.local で `ip addr add 192.168.10.1/24 dev wlan1`。dnsmasq は systemctl enable 済み。

## UI (app/)

レーシングコックピット風ダークテーマ（バニラJS / CSS）。左カラム（カメラ+コントロール）と右カラム（インストルメント、幅500px）の2カラムレイアウト。

### ヘッダー
- CPU温度・CPU負荷・メモリ使用率・MOTORエラー状態・WiFi RSSI(dBm) をインライン表示
- RSSI: -60以上=正常、-75以上=黄、それ以下=赤

### カメラエリア（左カラム）
- **WebSocket binary**: `/ws/camera` から受信したJPEGバイナリを `<img>` に表示。
- **HUD オーバーレイ**: カメラ画像の下中央に SPEED / ACCEL の SVG アークゲージを表示（117px、半透明背景付き）。
- **コントロールバー**（カメラ下、flex-row）:
  - 左側 `.ctrl-main`（flex-column）:
    - mode-bar: DRIVE（独立トグル）/ AUTO BRAKE（独立トグル）/ REMOTE・MODE 1・MODE 2（排他選択）
    - slider-action-row: SPD・ACC スライダー ＋ BRAKE・LIGHT・HAZARD・HORN ボタン（4列grid）
  - 右側 `.temp-gauge-row`（高さはctrl-barに自動フィット）: SIGNAL / POWER / LEFT / RIGHT / STEER の5ゲージ横並び

### インストルメントエリア（右カラム）
- **LiDAR レーダー**（最上部）: 360点ポリゴンのみ（ドットなし）。スライダーで最大表示距離変更（500mm〜12000mm）。
- **レーダーレンジスライダー**: ポリゴン下に配置。
- **AHI（Artificial Horizon Indicator）**: SVG で空（青）と地面（茶）を表示。ピッチ/ロール値も数値表示。
- **G-METER**: SVG ドットが加速度ベクトルを表示（緑=0.3g以下、黄=0.6g以下、赤=0.6g超）。

### ゲージ仕様（SVG arc r=38、270°スパン）
- SPEED: 0–5 m/s（HUDオーバーレイ、赤）
- ACCEL: 0–5 m/s²（HUDオーバーレイ、赤）
- SIGNAL: 8–12V（ctrl-bar右、赤）
- POWER: 8–12V（ctrl-bar右、赤）
- LEFT / RIGHT / STEER 温度: 0–100°C、50°C以上で黄、75°C以上で赤（ctrl-bar右）

### ゲームパッド（Elecom JC-U3613M、Chrome 推奨）
- `axes[1]`（左スティック Y）→ 速度（上=前進）
- `axes[2]`（右スティック X）→ ステアリング
- `button[5]`（R1）→ ブレーキ（押している間）
- `button[4]`（L1）→ ライト ON/OFF（押した瞬間にトグル）
- Safari は `gamepadconnected` イベントが発火しないため、500ms ポーリングでフォールバック。

### キーボード（REMOTE モード時）
- W/A/S/D: 移動方向、Space: ブレーキ（押している間）

## Network / Access

- RasPi Wi-Fi AP: SSID `robotcar` / PW `robotcar1234` / **2.4GHz ch1 802.11n**（USB adapter wlan1）
- RasPi 固定 IP: `192.168.10.1`
- ブラウザアクセス: `http://192.168.10.1:8000`
- SSH: `ssh pi@192.168.10.1`（PW: `robotcar`）

### 開発時（有線 LAN 接続時）
- 有線 LAN 接続中は `robotcar.local` で解決可能（DHCP）
- 静的ファイル更新: `scp <file> pi@robotcar.local:/home/pi/robotcar/app/`（サーバー再起動不要）
- Python ファイル更新: `scp` 後に `sudo systemctl kill -s SIGKILL robotcar && sleep 2 && sudo systemctl start robotcar`
- サービス再起動は `systemctl restart` ではなく SIGKILL → start を使う（接続待ちで止まるため）

## Setup Reference

詳細な初期セットアップ手順（APモード設定・venv・systemd登録）は `README.md` を参照。
