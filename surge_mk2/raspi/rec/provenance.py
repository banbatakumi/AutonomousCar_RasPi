"""`.mcap` に残す「この記録をどの設定・どのコードで撮ったか」（MCAP メタデータ `provenance`）。

ステレオ（など）の開発は、**記録を Mac で再生して同じ処理をやり直し、結果を比べる**ことが中心になる。そのとき校正値や
コードの版が分からないと、「照合を変えたから変わったのか、撮った時の設定が違ったのか」を切り分けられない。
`StereoInfo`（`stereo/info`）がフレームごとのパラメータを持ち、こちらは**記録の間変わらないもの**を持つ。

- `vehicle_toml` … `config/vehicle.toml` の全文（取付・魚眼の校正・`[sensors.stereo]`・制御の値。秘密は入っていない）
- `git` … `git describe --always --dirty`。**Pi は rsync で配るので `.git` が無い**ことが多く、その場合は `unknown`
- `src_sha` … 主要ファイルの内容の短いハッシュ。git が無い Pi でも「Mac のどのコードと同じか」を突き合わせられる
- `protocol_version` … UART プロトコルの版
"""

from __future__ import annotations

import hashlib
import subprocess
from pathlib import Path

__all__ = ["collect", "SRC_FILES"]

ROOT = Path(__file__).resolve().parents[2]

#: 内容のハッシュを残すファイル（ROOT からの相対）。ステレオと、その入力・出力に関わるもの
SRC_FILES = (
    "config/vehicle.toml",
    "raspi/proto/protocol.toml",
    "raspi/core/stereo_match.py",
    "raspi/core/stereo_model.py",
    "raspi/core/stereo_native.c",
    "raspi/core/stereo_obs.py",
    "raspi/nodes/stereo_node.py",
    "raspi/nodes/camera_node.py",
)


def _git() -> str:
    try:
        out = subprocess.run(["git", "-C", str(ROOT), "describe", "--always", "--dirty"],
                             capture_output=True, text=True, timeout=2.0)
        return out.stdout.strip() if out.returncode == 0 and out.stdout.strip() else "unknown"
    except (OSError, subprocess.SubprocessError):
        return "unknown"


def _sha(path: Path) -> str:
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()[:12]
    except OSError:
        return "missing"


def collect(root: Path = ROOT) -> dict[str, str]:
    """メタデータに載せる辞書（値は全部文字列）。読めないものは空・`missing` にして、記録の開始は妨げない。"""
    try:
        toml = (root / "config" / "vehicle.toml").read_text(encoding="utf-8")
    except OSError:
        toml = ""
    try:
        from raspi.proto.generated.packets import PROTOCOL_VERSION

        proto = f"{PROTOCOL_VERSION:#06x}"
    except ImportError:
        proto = "unknown"
    return {
        "git": _git(),
        "protocol_version": proto,
        "src_sha": ",".join(f"{rel}={_sha(root / rel)}" for rel in SRC_FILES),
        "vehicle_toml": toml,
    }
