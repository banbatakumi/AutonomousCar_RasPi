"""記録と再生。

- `framelog` … 生フレームログ `.sfl`。**stdlib のみ**で書けるので io_node の
  実時間ループから直接叩ける。UART を流れたバイトそのものが残る
- `mcap_log` … MCAP 書き出し。`mcap` に依存するので、**io_node には入れず**
  別プロセス（`logger_node`）とオフライン変換（`tools/sfl2mcap.py`）だけが使う
- `logclean` … 記録先パーティションの空き容量監視と、世代管理での自動削除
  （`surge-logclean.timer` との役割分担は `logclean.py` のモジュール docstring 参照）

**このパッケージの `__init__.py` はサブモジュールを再エクスポートしない**
（issue #22）。`from raspi.rec import FrameLogWriter` のように書くと、実時間の
`io_node` が `.mcap_log`（`mcap`/`zstandard` に依存、約23ms）まで芋づる式に
読み込んでしまう。使う側は次のようにサブモジュールを直接 import すること::

    from raspi.rec.framelog import FrameLogWriter, FrameLogReader, Kind, default_log_path
    from raspi.rec.logclean import DiskStatus, check_disk, disk_free_pct
    from raspi.rec.mcap_log import HAS_MCAP, McapLog, default_mcap_path  # mcap が要る側だけ
"""
