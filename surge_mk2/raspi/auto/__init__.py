"""自動運転アルゴリズム（`docs/architecture.md` §8）。

**アルゴリズムだけを置く。** バス・WS・GUI のことは何も知らない純粋な計算に
してあり、`Scan` と `VehicleState` を渡すと `AutoState` が返る。テストで
合成した点群を流し込めるのはこのため（`raspi/tests/test_auto.py`）。

配線は `raspi/nodes/planning_node.py` の仕事。

**このパッケージの `__init__.py` はサブモジュールを再エクスポートしない**
（issue #22）。`from raspi.auto import sector_of_deg` のように書くだけで
`registry` 経由で全 planner が芋づる式に読み込まれ、`slam2d_raceline`/
`slam2d_route` を通じて `slam2d.pipeline`（`backend/posegraph.py` 経由で
g2opy が必須）まで連鎖してしまう。`raspi.auto.base` の関数を1つ使うだけの
カメラ系ノード（cam_perception/cam_e2e/cam_track）でも g2opy が無いと
落ちる、という事故はこれが原因だった。使う側はサブモジュールを直接
import すること::

    from raspi.auto.base import ParamSpec, Planner, sector_of_deg, wrap_deg
    from raspi.auto.registry import PLANNERS, catalog, make_planner, merged_params
    from raspi.auto.e2e_lidar import E2ELidar
    from raspi.auto import mapstore  # サブモジュールなので再エクスポート無しでも import 可
"""

from __future__ import annotations
