"""`raspi/tests` 共通の準備（パッケージの読み込み時に走る。pytest でも `unittest discover` でも効く）。

## ライン生成のワーカーをテスト向けに軽くする

本番の `slam2d_raceline.worker_pool()` は別プロセスで、見積もりラップの仕上げ
（`raceline.refine_time`）を出発点 2 つ・評価 15000 回まで回す（magp2 で Mac 約1秒、Pi 5 で約5秒。
C 版の無い環境では Pi 5 で 50 秒以上）。
経路を作るテストは数十あり、そのまま回すと全体が数分〜数十分延びる。ここで

- ワーカーをスレッドにする（`SURGE_LINE_WORKER=thread`。子プロセスの起動を待たない）
- 仕上げの評価回数と出発点を小さくする

仕上げの中身は `test_speedctl.py`（`refine_time` 単体）と、本番の設定で引く
`test_line_worker.py` が確かめる。

★ ここに置く理由: `conftest.py` は pytest しか読まない。`tools/deploy.sh --test`
は Pi に pytest が無いので `unittest discover` で回しており、`conftest.py` に書くと本番の設定のまま経路を作って、
`test_route.py` の 30 秒の待ちを超えて失敗し、残ったワーカーを待つので終わらない。
"""

import os

os.environ.setdefault("SURGE_LINE_WORKER", "thread")

from raspi.auto import slam2d_raceline  # noqa: E402

slam2d_raceline.LINE_REFINE_EVALS = 300
slam2d_raceline.LINE_REFINE_STARTS = 1
