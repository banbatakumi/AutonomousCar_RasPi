# このサイトについて

## 目的と範囲

SURGE Mark.2 の制御・システム設計・自動運転の考え方を、同じような大会に出る人が**自分の車に持ち帰れる形**で残すためのサイトです。

- **扱う**: STM32 ファームウェア（駆動制御・TC・ABS・トルクベクタリング・安全層）、Pi ⇄ STM32 のプロトコル、Raspberry Pi 側のソフトウェア（走行モード、SLAM、レーシングライン）、開発・検証の道具。
- **扱わない**: 回路（基板）、モータドライバ（MD）のファーム、V1 の手動遠隔操縦システム（`robotcar/`）。

## どうやって書いたか

既存のドキュメントは古くなっていたり誤りを含んでいることがあるため、**ソースコードを読み直して**書きました。

- 数値は、定数の定義・パラメータ表・`vehicle.toml` から取りました。ドキュメントの数値は、そのまま写していません。
- 設計の理由は、ソースコード中のコメントと開発記録（`PROGRESS.md`）から取り、**出どころを各ページの末尾か本文に示して**います。
- 「実機で確認した」と言えるものと、シミュレータ・机上の値は分けて書きました（[検証状況](overview/status.md)）。
  開発者本人の記録を引いている箇所は、そう書いてあります。**第三者による再現確認ではありません。**
- 実機の走行記録そのもの（mcap）は、このサイトの作成では見ていません。実機の数値は開発記録の引用です。

### 数値を複製しない

リポジトリの方針は「同じ数字を 2 箇所に書くと必ず片方が古くなる」です。このサイトも、仕様の**正を持ちません。**
プロトコルのバイト配置は [`protocol.toml`][protocol-toml]、車両パラメータは [`vehicle.toml`][vehicle-toml]、制御の定数は `drive.h` / `control_params.h` を見てください。
ページ内の数値は、読むときの目安として、確認した版を明記して載せています。

## 更新のしかた

サイトの元は [`surge_mk2/site/`][site-dir] にある Markdown です。

```bash
cd surge_mk2/site
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/mkdocs serve          # http://127.0.0.1:8000 でプレビュー
.venv/bin/mkdocs build --strict # 壊れたリンクがあると失敗する
```

`master` に push すると、GitHub Actions（[`pages.yml`][pages-yml]）がビルドして公開します。プルリクエストではビルドだけを確かめます。
初回だけ、リポジトリの Settings → Pages → Source を「GitHub Actions」にしてください。

**コードを変えたときに見直すページ**

| 変えたもの | 見直すページ |
|---|---|
| `drive.c` / `drive.h` / `control_params.h` | [駆動制御](stm32/drive.md)、[スリップ制御](stm32/traction.md) |
| `torque_vectoring.c` | [トルクベクタリング](stm32/torque-vectoring.md) |
| `vehicle.c` / `heartbeat.c` / `power.h` | [安全層](stm32/safety.md) |
| `protocol.toml` / `ras_link.h` | [UART プロトコル](link/protocol.md) |
| `io_node.py` / `telemetry_node.py` | [上位の安全設計](raspi/safety.md)、[ノード構成](raspi/index.md) |
| `raspi/auto/` / `raspi/nav/` / `slam2d/` | [走行モード](raspi/modes.md)、[地図](raspi/mapping.md)、[レーシングライン](raspi/raceline.md)、[追従](raspi/tracking.md) |
| 実機で何かを確かめた | [検証状況](overview/status.md) |

## この作成中に見つけた食い違い

既存の文書やコードのコメントと、実際のコードが合っていなかった箇所です（確認した版 `19291eb` / `2a2e9a8` の時点）。サイトは**コードの側**に合わせて書いています。

| 箇所 | 内容 |
|---|---|
| ファーム `drive.h` の API コメント | `Drive_SetTractionControlEnabled` に「既定は有効」、`TorqueVectoring_Init` に「初期状態は有効」とあるが、実装は**どちらも無効で初期化**し、上位が毎周期送る値で上書きする（同じヘッダの構造体のコメントは「既定は無効（上位が送る）」） |
| ファーム `docs/README.md` と `CLAUDE.md` | `v0.22` までの差分しか触れておらず、現行版を `0x0016` としている。コードは `0x0017`（v0.23）で、`pi_uart_protocol_v0.23_delta.md` もある |
| 開発記録とファームのソース | `PROGRESS.md`（2026-10-14）は、ファーム側にも「再 ARM の規則（`rearm_required`）」を入れたと読める記述があるが、**確認したファームのコミットには見当たらない**（[安全層](stm32/safety.md#再-arm-の扱い)） |
| ファームのリポジトリの既定ブランチ | `main` は基板の履歴で、ファームの最新ではない（最新は `FW/MainBoard_V3`） |
| `vehicle.toml` の `mu` | `[dynamics]` は 0.42（旋回試験の限界から）、`[control.plant]` は 0.51（タイヤモデルのフィット）。用途が違う値だが、同じ名前で食い違っている |
| Pi への OpenCV の導入 | `slam2d/README.md` は Pi 5 に `opencv-contrib-python-headless` が入っていると書き、`cam_e2e_preproc.py` は「Pi には OpenCV も Pillow も入れていない」と書く。`requirements.txt`（開発）には OpenCV があり、`requirements.lock.txt`（実機）には見当たらない。どちらが正しいかは確定できなかった |

## セキュリティについて

このサイトの本文には、認証情報（パスワード・鍵・トークン）、IP アドレス、実際のホスト名、個人の氏名は載せていません。

- 共有トークンは `config/secret.txt`（`.gitignore` 済み）か環境変数から読む仕組みで、リポジトリには含まれません。
- 制御用の API は既定で `127.0.0.1` にしか開かず、外へ出すときは明示が要ります（[上位の安全設計](raspi/safety.md#公開面の守り方)）。

リポジトリ全体の公開前の確認は、サイトの範囲外です。

## 連絡・訂正

誤りを見つけたら、リポジトリの issue かプルリクエストでお知らせください。**コードと食い違っていたら、コードが正です。**

[protocol-toml]: https://github.com/banbatakumi/AutonomousCar_RasPi/blob/master/surge_mk2/raspi/proto/protocol.toml
[vehicle-toml]: https://github.com/banbatakumi/AutonomousCar_RasPi/blob/master/surge_mk2/config/vehicle.toml
[site-dir]: https://github.com/banbatakumi/AutonomousCar_RasPi/tree/master/surge_mk2/site
[pages-yml]: https://github.com/banbatakumi/AutonomousCar_RasPi/blob/master/.github/workflows/pages.yml
