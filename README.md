# RobotCar

Raspberry Pi 3 + STM32F446RE によるロボットカー遠隔操縦システム。

- **RasPi** が FastAPI サーバーを起動し、Wi-Fi AP として動作
- **ブラウザ**（Mac 等）が `http://192.168.10.1:8000` へアクセスして操作
- **STM32** とは UART 1Mbps で制御コマンド・センサーデータを交換

---

## ハードウェア構成

| 部品 | 詳細 |
|------|------|
| Raspberry Pi 3 Model B | ホスト名: `robotcar`、ユーザー: `pi` |
| STM32F446RE | UART（GPIO14/TX, 15/RX）、**1,000,000bps** |
| カメラ | Raspberry Pi Camera Module v2（CSI接続） |
| USB WiFi | MT7610U（Archer T2U相当）、wlan1、2.4GHz 802.11n |
| ブザー | GPIO18（PWM0） |
| LED1 | GPIO19（リモコン表示） |
| LED2 | GPIO13（PC接続表示） |

---

## 1. AP モード設定（USB WiFi アダプタ wlan1 使用）

### 必要パッケージのインストール

```bash
sudo apt update
sudo apt install -y hostapd dnsmasq
sudo systemctl stop hostapd dnsmasq
```

### hostapd 設定（`/etc/hostapd/hostapd.conf`）

```
interface=wlan1
driver=nl80211
ssid=robotcar
hw_mode=g
channel=1
country_code=JP
ieee80211n=1
wmm_enabled=1
ht_capab=[HT40+][SHORT-GI-20][SHORT-GI-40]
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=robotcar1234
wpa_key_mgmt=WPA-PSK
wpa_pairwise=CCMP
rsn_pairwise=CCMP
```

```bash
echo 'DAEMON_CONF="/etc/hostapd/hostapd.conf"' | sudo tee -a /etc/default/hostapd
```

### hostapd の systemd drop-in（`/etc/systemd/system/hostapd.service.d/wait-wlan0.conf`）

```ini
[Unit]
After=systemd-networkd.service sys-subsystem-net-devices-wlan1.device
Requires=sys-subsystem-net-devices-wlan1.device
```

### dnsmasq 設定（`/etc/dnsmasq.conf`）

```
interface=wlan1
dhcp-range=192.168.10.2,192.168.10.20,255.255.255.0,24h
domain=local
address=/robotcar.local/192.168.10.1
```

### 静的 IP と省電力無効化（`/etc/rc.local`）

```bash
#!/bin/sh -e
echo on > /sys/class/net/wlan1/power/control
ip addr add 192.168.10.1/24 dev wlan1 2>/dev/null || true
ip link set wlan1 up
exit 0
```

### wlan0 の無効化（`/etc/dhcpcd.conf` に追記）

```
denyinterfaces wlan0
interface wlan1
    static ip_address=192.168.10.1/24
    nohook wpa_supplicant
```

### サービス有効化

```bash
sudo systemctl daemon-reload
sudo systemctl unmask hostapd
sudo systemctl enable hostapd dnsmasq
sudo reboot
```

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

> **サービス再起動**: `systemctl restart` ではなく以下を使う（接続待ちで止まるため）
> ```bash
> sudo systemctl kill -s SIGKILL robotcar && sleep 2 && sudo systemctl start robotcar
> ```

---

## 4. 接続手順

1. Wi-Fi で `robotcar`（パスワード: `robotcar1234`）に接続
2. ブラウザ（Chrome 推奨）で `http://192.168.10.1:8000` を開く
3. **DRIVE** ボタンをクリックして走行開始

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

### STM32 → RasPi（733 バイト、1Mbps）

```
[0xFF][motor_err][speed][accel][volt_s][volt_p][accel_xy][pitch][roll][tmp_l][tmp_r][tmp_s][dist×360×2bytes][0xAA]
```

| バイト | 説明 |
|---|---|
| [1] | モーターエラーフラグ uint8 |
| [2] | 速度 int8 × 0.1 m/s |
| [3] | 加速度 int8 × 0.1 m/s² |
| [4] | 信号電圧 uint8 × 0.1 V |
| [5] | バッテリー電圧 uint8 × 0.1 V |
| [6] | IMU加速度 high nibble=X(前後)、low nibble=Y(左右)、4bit符号付き×0.5g |
| [7] | ピッチ int8（度） |
| [8] | ロール int8（度） |
| [9] | 左モーター温度 uint8（°C） |
| [10] | 右モーター温度 uint8（°C） |
| [11] | ステアリングモーター温度 uint8（°C） |
| [12..731] | LiDAR 360点 × uint16 big-endian（mm、0=範囲外、最大12000mm） |
| [732] | 0xAA（フッタ） |

---

## カメラ設定

| 項目 | 値 |
|---|---|
| 解像度 | 320×182（下3/7カット） |
| FPS | 30fps（暗所では自動で最低10fpsまで低下） |
| JPEG品質 | 70 |
| センサーモード | 1640×1232（フルFOV 2x2ビニング） |
| 配信方式 | WebSocket binary `/ws/camera` |
| 自動露光 | 有効（暗所で露光時間を自動延長） |

---

## 開発時のファイル更新

有線 LAN 接続時は `robotcar.local` で解決可能。

```bash
# 静的ファイル（HTML/CSS/JS）の転送（再起動不要）
scp app/index.html app/style.css app/main.js pi@robotcar.local:/home/pi/robotcar/app/

# Python ファイルの転送（転送後にサービス再起動が必要）
scp raspi/main.py raspi/uart_handler.py raspi/camera_stream.py pi@robotcar.local:/home/pi/robotcar/raspi/
ssh pi@robotcar.local "sudo systemctl kill -s SIGKILL robotcar && sleep 2 && sudo systemctl start robotcar"
```
