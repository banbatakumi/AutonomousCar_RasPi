"""`raspi/tests` 共通の準備（パッケージの読み込み時に走る。pytest でも `unittest discover` でも効く）。

## ライン生成のワーカーをテスト向けに軽くする

本番の `slam2d_raceline.worker_pool()` は別プロセスで、見積もりラップの仕上げ
（`raceline.refine_time`）を出発点 2 つ・評価 15000 回まで回す（Mac で 6〜8 秒、Pi 5 で 50 秒以上）。
経路を作るテストは数十あり、そのまま回すと全体が数分〜数十分延びる。ここで

- ワーカーをスレッドにする（`SURGE_LINE_WORKER=thread`。子プロセスの起動を待たない）
- 仕上げの評価回数と出発点を小さくする

仕上げの中身は `test_speedctl.py`（`refine_time` 単体）と、本番の設定で引く
`test_line_worker.py` が確かめる。

★ ここに置く理由: 以前は `conftest.py` に書いていたが、それは pytest しか読まない。`tools/deploy.sh --test`
は Pi に pytest が無いので `unittest discover` で回しており、本番の設定のまま経路を作って、
`test_route.py` の 30 秒の待ちを超えて失敗し、残ったワーカーを待つので終わらなかった。
"""

import os

os.environ.setdefault("SURGE_LINE_WORKER", "thread")

from raspi.auto import slam2d_raceline  # noqa: E402

slam2d_raceline.LINE_REFINE_EVALS = 300
slam2d_raceline.LINE_REFINE_STARTS = 1
