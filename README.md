# RobotCar

Raspberry Pi 3 + STM32F446RE によるロボットカー遠隔操縦システム。

- **RasPi** が FastAPI サーバーを起動し、Wi-Fi AP として動作
- **ブラウザ**（Mac 等）が `http://192.168.10.1:8000` へアクセスして操作
- **STM32** とは UART 230400bps で制御コマンド・センサーデータを交換

---

## 1. APモード設定

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
# /etc/default/hostapd の DAEMON_CONF を設定
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

---

## 2. アプリケーションのセットアップ

### リポジトリ配置

```bash
# RasPi 上で
git clone <repo-url> /home/pi/robotcar
```

### Python 仮想環境の作成と依存パッケージのインストール

`picamera2` は Raspberry Pi OS に同梱されているシステムパッケージを使うため、
`--system-site-packages` オプションで仮想環境を作成します。

```bash
cd /home/pi/robotcar
python3 -m venv --system-site-packages venv
source venv/bin/activate
pip install -r raspi/requirements.txt
```

### UART の有効化

```bash
# /boot/config.txt（または /boot/firmware/config.txt）に追記
echo "enable_uart=1" | sudo tee -a /boot/config.txt

# Bluetooth と UART の競合を解消（Pi 3 の場合）
echo "dtoverlay=disable-bt" | sudo tee -a /boot/config.txt
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
2. ブラウザで `http://192.168.10.1:8000` を開く
3. 「走行」スイッチを ON にして操作開始

---

## ハードウェア構成

| 部品 | 詳細 |
|------|------|
| Raspberry Pi 3 Model B | ホスト名: `robotcar`、ユーザー: `pi` |
| STM32F446RE | UART（GPIO14/TX, 15/RX）、230400bps |
| カメラ | Raspberry Pi Camera Module（CSI接続） |
| ブザー | GPIO18（PWM0） |
| LED1（リモコン表示） | GPIO19 |
| LED2（PC接続表示） | GPIO13（PWM1） |

## プロトコル概要

**RasPi → STM32（6 バイト）**

```
[0xFF][flags][move_speed×0.1m/s][accel×0.1m/s²][steer/127][0xAA]
```

flags: bit0=do_stop, bit1=do_remote_control, bit2=do_brake, bit3=on_headlight

**STM32 → RasPi（11 バイト）**

```
[0xFF][speed][accel][front][left][right][back][v_sig][v_pow][motor_err][0xAA]
```
