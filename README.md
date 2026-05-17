# RobotCar

Raspberry Pi 3 + STM32F446RE によるロボットカー遠隔操縦システム。

- **RasPi** が FastAPI サーバーを起動し、Wi-Fi AP として動作
- **ブラウザ**（Mac 等）が `http://192.168.10.1:8000` へアクセスして操作
- **STM32** とは UART 230400bps で制御コマンド・センサーデータを交換

---

## 1. AP モード設定

### 必要パッケージのインストール

```bash
sudo apt update
sudo apt install -y hostapd dnsmasq
sudo systemctl stop hostapd dnsmasq
```

### 静的 IP の設定（`/etc/dhcpcd.conf` に追記）

```
interface wlan0
    static ip_address=192.168.10.1/24
    nohook wpa_supplicant
```

### hostapd 設定

```bash
sudo cp raspi/setup/hostapd.conf /etc/hostapd/hostapd.conf
echo 'DAEMON_CONF="/etc/hostapd/hostapd.conf"' | sudo tee -a /etc/default/hostapd
```

### dnsmasq 設定

```bash
sudo mv /etc/dnsmasq.conf /etc/dnsmasq.conf.bak
sudo cp raspi/setup/dnsmasq.conf /etc/dnsmasq.conf
```

### サービス有効化

```bash
sudo systemctl unmask hostapd
sudo systemctl enable hostapd dnsmasq
sudo reboot
```

### Wi-Fi 省電力モードの無効化

```bash
echo on | sudo tee /sys/class/net/wlan0/power/control
# 永続化（/etc/rc.local に追記）
echo 'echo on > /sys/class/net/wlan0/power/control' | sudo tee -a /etc/rc.local
```

> **注**: brcmfmac ドライバーの AP モードでは省電力モードを完全に無効化できない場合があります。  
> `hostapd.conf` のチャンネルを 11 に設定することで干渉を軽減しています。  
> カメラストリームのウォッチドッグ（2秒間隔）が自動で再接続を行います。

---

## 2. アプリケーションのセットアップ

### リポジトリの配置

```bash
git clone https://github.com/banbatakumi/AutonomousCar_RasPi.git /home/pi/robotcar
```

### Python 仮想環境の作成

`picamera2` はシステムパッケージを使うため `--system-site-packages` で作成します。

```bash
cd /home/pi/robotcar
python3 -m venv --system-site-packages venv
source venv/bin/activate
pip install -r raspi/requirements.txt
```

### UART の有効化

```bash
# /boot/firmware/config.txt に追記（Pi 3 は /boot/config.txt）
echo "enable_uart=1" | sudo tee -a /boot/firmware/config.txt
echo "dtoverlay=disable-bt" | sudo tee -a /boot/firmware/config.txt
sudo systemctl disable hciuart
```

---

## 3. systemd への登録・自動起動

```bash
sudo cp raspi/setup/robotcar.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable robotcar
sudo systemctl start robotcar

# ログ確認
sudo journalctl -u robotcar -f
```

---

## 4. Mac からの接続手順

1. Mac の Wi-Fi で `robotcar`（パスワード: `robotcar1234`）に接続
2. ブラウザ（Chrome 推奨）で `http://192.168.10.1:8000` を開く
3. **DRIVE** ボタンをクリックして走行開始

---

## ハードウェア構成

| 部品 | 詳細 |
|------|------|
| Raspberry Pi 3 Model B | ホスト名: `robotcar`、ユーザー: `pi` |
| STM32F446RE | UART（GPIO14/TX, 15/RX）、230400bps |
| カメラ | Raspberry Pi Camera Module v2（CSI接続、30fps / JPEG quality 80） |
| ブザー | GPIO18（PWM0） |
| LED1 | GPIO19 |
| LED2 | GPIO13（PWM1） |

---

## 通信プロトコル

### RasPi → STM32（6 バイト、20Hz）

```
[0xFF][flags][move_speed][acceleration][steer][0xAA]
```

| フィールド | 型 | 説明 |
|---|---|---|
| flags | uint8 | bit0=do_stop, bit1=do_brake, bit2=headlight, bit3=hazard, bit4=play_sound, bit5=auto_brake, bit7-6=mode(0-3) |
| move_speed | int8 | 速度 × 0.1 m/s |
| acceleration | int8 | 加速度 × 0.1 m/s² |
| steer | int8 | ステア -127〜+127（左〜右） |

### STM32 → RasPi（31 バイト）

```
[0xFF][speed][accel][dist×18bytes(36nibbles)][volt_s][volt_p][motor_err][accel_xy][pitch][roll][temp_l][temp_r][temp_s][0xAA]
```

| バイト | 説明 |
|---|---|
| [1] | 速度 int8 × 0.1 m/s |
| [2] | 加速度 int8 × 0.1 m/s² |
| [3..20] | 36 センサー距離（各4bit nibble、値×10cm、0=範囲外） |
| [21] | 信号電圧 × 0.1 V |
| [22] | バッテリー電圧 × 0.1 V |
| [23] | モーターエラーフラグ |
| [24] | IMU 加速度 high nibble=X(前後)、low nibble=Y(左右)、4bit符号付き×0.1g |
| [25] | ピッチ int8（度） |
| [26] | ロール int8（度） |
| [27] | 左モーター温度 uint8（°C） |
| [28] | 右モーター温度 uint8（°C） |
| [29] | ステアリングモーター温度 uint8（°C） |
| [30] | 0xAA（フッタ） |

---

## 開発時のファイル更新

有線 LAN 接続時は `robotcar.local` で解決可能。

```bash
# 静的ファイル（HTML/CSS/JS）の転送（再起動不要）
scp app/index.html app/style.css app/main.js pi@robotcar.local:/home/pi/robotcar/app/

# Python ファイルの転送（転送後にサービス再起動が必要）
scp raspi/uart_handler.py pi@robotcar.local:/home/pi/robotcar/raspi/
ssh pi@robotcar.local "sudo systemctl restart robotcar"

# 構文確認（転送後に必ず実行）
curl -s http://robotcar.local:8000/main.js | node --check /dev/stdin
```
