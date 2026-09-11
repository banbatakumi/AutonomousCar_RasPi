"""記録先パーティションの空き容量監視と、世代管理による自動削除。

## `surge-logclean.timer` との役割分担

`raspi/setup/install_services.sh` が入れる `surge-logclean.timer` は、毎時
「7日より古いもの」と「`logs/` 合計が `MAX_LOG_MB` 超過ぶん」を `find`/`du`/`rm`
で消す（systemd タイマー・シェルのみ・Python コードの外）。運用の主力はそちら。

ここが埋める隙間は2つ:

1. **最大1時間の遅れがある。** 走行中に SD の空きがゼロへ向かっているのを、
   次の毎時タイマーより先に検知したい（`io_node`/`logger_node` はまさに
   今そこへ書き込んでいるプロセスなので、一番早く気づける立場にある）。
2. **`surge-logclean` は合計サイズ（ディスク使用量）で判断するが、
   ここは空き容量そのもの（`shutil.disk_usage`）で判断する。** ログ以外の
   何か（OS のジャーナル肥大など）が空きを食っていても、こちらは反応できる。

**`.sfl`/`.mcap` の書き込み側（`FrameLogWriter`/`McapLog`）の ENOSPC 保護とは別物。**
あちらは「書けなかったときにクラッシュしない」ための最終防御で、こちらは
「そもそも書けなくなる前に空ける」ための予防。両方要る。
"""

from __future__ import annotations

import shutil
import time
from dataclasses import dataclass, field
from pathlib import Path

__all__ = [
    "DiskStatus",
    "check_disk",
    "disk_free_pct",
    "LOG_PATTERNS",
    "DEFAULT_WARN_FREE_PCT",
    "DEFAULT_CRITICAL_FREE_PCT",
    "DEFAULT_MAX_DELETE_PER_CALL",
]

#: 世代管理の削除候補にする拡張子。`.sfl`（UART生ログ）と `.mcap`（オフライン
#: 変換・`--with-logger` 実行時の生成物）の両方。他のファイル（設定・写真など）
#: には触らない
LOG_PATTERNS = ("*.sfl", "*.mcap")

#: 空き容量がこれを下回ったら警告する（まだ削除しない）。
#: SD カードの容量は機体ごとに違うので、絶対容量ではなく割合で見る
DEFAULT_WARN_FREE_PCT = 15.0

#: 空き容量がこれを下回ったら、古い順に消して確保する
DEFAULT_CRITICAL_FREE_PCT = 5.0

#: 1回の呼び出しで削除するファイル数の上限。**暴走防止**（何らかの理由で
#: 空き容量計算が壊れていても、ログを全部消し切ってしまわないようにする）。
#: 定期的に何度も呼ばれる前提なので、1回で足りなくても次回また削る
DEFAULT_MAX_DELETE_PER_CALL = 50


def disk_free_pct(path: str | Path) -> float:
    """`path` が乗っているパーティションの空き容量 [%]。"""
    usage = shutil.disk_usage(path)
    if usage.total <= 0:
        return 100.0
    return usage.free / usage.total * 100.0


@dataclass(slots=True)
class DiskStatus:
    """`check_disk()` の結果。"""

    free_pct: float
    free_bytes: int
    total_bytes: int
    #: 警告しきい値を下回っていたか（削除の有無に関わらず）
    warned: bool = False
    #: 実際に削除したファイル名（古い順）
    deleted: list[str] = field(default_factory=list)
    freed_bytes: int = 0
    #: 空き容量そのものが取得できなかった場合の理由。非 None なら他のフィールドは無意味
    error: str | None = None


def check_disk(log_dir: str | Path, *,
                warn_free_pct: float = DEFAULT_WARN_FREE_PCT,
                critical_free_pct: float = DEFAULT_CRITICAL_FREE_PCT,
                patterns: tuple[str, ...] = LOG_PATTERNS,
                protect: set[Path] | None = None,
                max_delete: int = DEFAULT_MAX_DELETE_PER_CALL) -> DiskStatus:
    """空き容量を調べ、`critical_free_pct` を下回っていれば古いログから消す。

    :param log_dir: 空き容量を見るパーティション兼、削除候補を探すディレクトリ
        （`.sfl`/`.mcap` の保存先そのもの。実在しなくても例外にはしない呼び出し側
        で `exists()` を見ること——ここでは `shutil.disk_usage` に任せる）
    :param protect: 削除してはいけないパス（**記録中の `.sfl` など**）。
        世代管理は古い順に消すので通常は最新のファイルまで届かないが、
        「他に消せるものが無い」極限状態で記録中ファイルまで巻き込まないための保険
    :param max_delete: 1回の呼び出しで削除する上限

    :return: 呼び出し後の状態。`shutil.disk_usage` 自体が失敗する環境
        （パス不在等）では `error` を立てて `free_pct=100.0` を返す
        （＝「判断材料が無いので何もしない」を上位に伝える）。
    """
    log_dir = Path(log_dir)
    protect_set = {Path(p) for p in (protect or ())}

    try:
        usage = shutil.disk_usage(log_dir)
    except OSError as e:
        return DiskStatus(free_pct=100.0, free_bytes=0, total_bytes=0,
                          error=f"{type(e).__name__}: {e}")

    free_pct = (usage.free / usage.total * 100.0) if usage.total else 100.0
    status = DiskStatus(free_pct=free_pct, free_bytes=usage.free, total_bytes=usage.total)

    if free_pct >= warn_free_pct:
        return status
    status.warned = True

    if free_pct >= critical_free_pct:
        return status

    # ── 世代管理: mtime の古い順に、critical を上回るまで消す ──
    candidates: list[Path] = []
    for pat in patterns:
        candidates.extend(log_dir.glob(pat))
    candidates = [p for p in candidates if p not in protect_set]
    candidates.sort(key=_safe_mtime)

    for p in candidates:
        if len(status.deleted) >= max_delete:
            break
        try:
            size = p.stat().st_size
            p.unlink()
        except OSError:
            continue    # 他プロセスが既に消した等。次の候補へ
        status.deleted.append(p.name)
        status.freed_bytes += size

        try:
            usage = shutil.disk_usage(log_dir)
        except OSError:
            break
        status.free_bytes = usage.free
        status.total_bytes = usage.total
        status.free_pct = (usage.free / usage.total * 100.0) if usage.total else 100.0
        if status.free_pct >= critical_free_pct:
            break

    return status


def _safe_mtime(p: Path) -> float:
    """壊れて `stat()` できないファイルは「一番新しい」扱いにして後回しにする。"""
    try:
        return p.stat().st_mtime
    except OSError:
        return time.time()
