"""`raspi/tests` 共通の準備（pytest が最初に読む）。

## ライン生成のワーカーをテスト向けに軽くする

本番の `slam2d_raceline.worker_pool()` は別プロセスで、見積もりラップの仕上げ
（`raceline.refine_time`）を出発点 2 つ・評価 15000 回まで回す（Mac で 6〜8 秒）。経路を作る
テストは数十あり、そのまま回すと全体が数分延びる。ここで

- ワーカーをスレッドにする（`SURGE_LINE_WORKER=thread`。子プロセスの起動を待たない）
- 仕上げの評価回数と出発点を小さくする

仕上げの中身は `test_speedctl.py`（`refine_time` 単体）と、本番の設定で引く
`test_line_worker.py` が確かめる。
"""

import os

os.environ.setdefault("SURGE_LINE_WORKER", "thread")

from raspi.auto import slam2d_raceline  # noqa: E402

slam2d_raceline.LINE_REFINE_EVALS = 300
slam2d_raceline.LINE_REFINE_STARTS = 1
