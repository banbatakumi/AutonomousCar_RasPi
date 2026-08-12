"""SURGE Mk.2 シミュレータ — **Mac 専用。Pi には配らない。**

`SerialLink` の位置に `SimLink` を差し、その裏に仮想 STM32・車両モデル・
仮想 LD06・コースを置く。`io_node` より上流（`convert.py` / `ScanAssembler` /
`telemetry_node` / GUI）は 1 行も変えずに動く。

## 設計の原則 — シミュレータが偽装するのは STM32 だけ

- **実機側のコードに、シミュレータ固有の概念を持ち込まない。**
  コース切替・ノイズ量・欠損率は実機に対応物が無い。それらの操作 UI は
  実機 GUI ではなく `sim.gui`（pygame）に置く
- **シム ↔ シムGUI は ZeroMQ バスを使わず UDP ループバック**（`sim/channel.py`）。
  バスを使うと `raspi/msgs/types.py` と `raspi/bus/zbus.py` にシムの語彙が入り込む
- 実機側への例外は `LinkDiag.sim` の 1 ビットだけ。これは利便性ではなく**安全要件**で、
  シムと実機の画面が見分けられないと「シムのつもりで --allow-arm した実車が動く」が起きる

## 動かす

    python -m raspi.nodes.io_node --sim --course sim/courses/oval.png \\
        --allow-arm --no-heartbeat --no-leds
    python -m raspi.nodes.telemetry_node --no-camera
    python -m sim.gui
"""
