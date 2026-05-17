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
│   ├── uart_handler.py # STM32 との UART 送受信（20Hz送信スレッド）
│   ├── camera_stream.py# picamera2 による MJPEG キャプチャスレッド
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

- **静的ファイル配信**: `main.py` が `app/` を `/` にマウント。`/ws` と `/stream` は StaticFiles より先に登録されるため衝突しない。
- **UART フレーミング (送信)**: ヘッダ `0xFF` / フッタ `0xAA`、6バイト固定長、20Hz。
  - flags: bit0=do_stop, bit1=do_brake, bit2=on_headlight, bit3=on_hazard, bit4=play_sound, bit5=enable_auto_brake, bit7-6=mode(0-3)
  - move_speed: int8 × 0.1 m/s、acceleration: int8 × 0.1 m/s²、steer: int8 / 127.0
- **UART フレーミング (受信)**: 31バイト固定長。`_recv_loop` はバッファから `buf[i]==0xFF` かつ `buf[i+30]==0xAA` を検索し、最後のフレームを採用（古いフレームをスキップ）。
  - [1]=speed, [2]=accel, [3..20]=36センサー距離nibble×2, [21]=volt_signal, [22]=volt_power, [23]=motor_err, [24]=accel_xy nibble, [25]=pitch, [26]=roll, [27]=temp_left, [28]=temp_right, [29]=temp_steer
- **LiDAR**: 36センサー（0°〜350°、10°刻み）、各センサー4bit nibble（値×10cm、0=範囲外、最大150cm）。2センサーで1バイト（high nibble=偶数インデックス、low nibble=奇数）。
- **IMU**: accel_x（前後G）とaccel_y（左右G）は4bit符号付き×0.1g、pitch/rollはint8（度）。
- **モーター温度**: temp_left/temp_right/temp_steer はuint8（°C）。50°C以上で黄、75°C以上で赤表示。
- **EMA センサーフィルタ**: 速度/加速度α=0.35、距離α=0.50、電圧α=0.15、IMU加速度α=0.50、IMUピッチ/ロールα=0.30、温度α=0.10。定数は `uart_handler.py` 上部で調整可能。
- **WebSocket 切断時の安全停止**: 接続ゼロになったとき `do_stop=True` を UART に即送信する（`main.py` の `finally` ブロック）。
- **GPIO グレースフル**: `GPIOHandler` は `RPi.GPIO` のインポート失敗を握りつぶすため、Mac 上でも動作確認できる。接続時に C5→E5→G5→C6 のアルペジオメロディを再生。
- **カメラ**: picamera2 の `JpegEncoder + start_recording` による連続配信。センサーモード 1640×1232（フルFOV）、lores=240×144 で MJPEG 出力（FPS=30、JPEG quality=80）。`ScalerCrop=(0,0,3280,1971)` で下20%（車体）をカット。フレーム同期は `threading.Condition`。
- **ストリームウォッチドッグ**: 2秒間隔でピクセル比較、1回フリーズ検出で即再接続（`main.js`）。
- **Wi-Fi**: チャンネル11（hostapd）、brcmfmac NVRAM `PM=0` 設定済み、`/sys/class/net/wlan0/power/control = on`（rc.local）。

## UI (app/)

レーシングコックピット風ダークテーマ（バニラJS / CSS）。左カラム（カメラ+コントロール）と右カラム（インストルメント）の2カラムレイアウト（右幅430px）。

### カメラエリア（左カラム）
- **MJPEG ストリーム**: `/stream` エンドポイント。ウォッチドッグで自動再接続（2秒検知）。
- **HUD オーバーレイ**: カメラ画像の下中央に SPEED / ACCEL の SVG アークゲージを表示（117px、半透明背景付き）。
- **コントロールバー**（カメラ下、flex-row）:
  - 左側 `.ctrl-main`（flex-column）:
    - mode-bar: DRIVE（独立トグル）/ AUTO BRAKE（独立トグル）/ REMOTE・MODE 1・MODE 2（排他選択）
    - slider-action-row: SPD・ACC スライダー ＋ LIGHT・HAZARD・HORN ボタン（アイコン上・テキスト下、grid均等幅）
  - 右側 `.temp-gauge-row`（高さはctrl-barに自動フィット）: SIGNAL / POWER / LEFT / RIGHT / STEER の5ゲージ横並び

### インストルメントエリア（右カラム）
- **LiDAR レーダー**（最上部）: 36センサーをポリゴン＋ドットで表示（緑=遠、黄=中、赤=40cm以下）。
- **AHI（Artificial Horizon Indicator）**: SVG で空（青）と地面（茶）を表示。ピッチ/ロール値も数値表示。
- **G-METER**: SVG ドットが加速度ベクトルを表示（緑=0.3g以下、黄=0.6g以下、赤=0.6g超）。
- **テレメトリ**: MOTOR エラー状態表示。

### ゲージ仕様（全て赤、SVG arc r=38、270°スパン）
- SPEED: 0–5 m/s（HUDオーバーレイ）
- ACCEL: 0–5 m/s²（HUDオーバーレイ）
- SIGNAL: 8–12V（ctrl-bar右）
- POWER: 8–12V（ctrl-bar右）
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

- RasPi Wi-Fi AP: SSID `robotcar` / PW `robotcar1234` / **チャンネル11**
- RasPi 固定 IP: `192.168.10.1`
- ブラウザアクセス: `http://192.168.10.1:8000`
- SSH: `ssh pi@192.168.10.1`（PW: `robotcar`）

### 開発時（有線 LAN 接続時）
- 有線 LAN 接続中は `robotcar.local` で解決可能（DHCP）
- 静的ファイル更新: `scp <file> pi@robotcar.local:/home/pi/robotcar/app/`（サーバー再起動不要）
- Python ファイル更新: `scp` 後に `sudo systemctl restart robotcar`
- **転送後は必ず** `curl -s http://robotcar.local:8000/main.js | node --check /dev/stdin` で構文確認（並列 scp によるファイル破損を防ぐため）

## Setup Reference

詳細な初期セットアップ手順（APモード設定・venv・systemd登録）は `README.md` を参照。
