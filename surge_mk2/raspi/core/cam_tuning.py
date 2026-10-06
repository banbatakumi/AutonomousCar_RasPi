"""レンズごとの ISP チューニング（色むら補正）を、センサー標準のチューニングへ重ねる。

libcamera はセンサーごとのチューニングファイル（`imx219.json` 等）を持ち、その中の
色むら補正（`rpi.alsc`）は**純正レンズ**で校正した表になっている。レンズを替えると
合わなくなる——IMX219 160° 広角では、センサーの生の色むらが ±5〜8% しか無いのに標準の表が
約 ±20% 掛けるので、白い壁が「中央は緑・周辺はピンク」に写った（2026-10-06 実測）。

そこで**差分だけ**を `config/cam_tuning/<名前>.json` に持ち、起動時に標準のチューニングへ
重ねる。丸ごと写しを持たないのは、OS 更新で標準のチューニングが変わったときに他の項目
（AWB・色行列など）が古いまま残らないようにするため。

どのレンズがどの差分を使うかは `config/vehicle.toml` のレンズのプロファイルの
`isp_tuning`（`[sensors.cam_*.lenses.<レンズ>]`）。**書いていないレンズは標準のまま**
（純正レンズ `stock` はこれまでどおり）。差分を作る道具は `raspi/tools/alsc_calib.py`。

**チューニングはプロセスに1つ。** libcamera は最初にカメラを数えた時点（`Picamera2()` や
`Picamera2.global_camera_info()` の初回）で全カメラのチューニングを決め、後から開くカメラに
別のものを渡しても黙って無視する（2026-10-06 実機で確認: 先に別のカメラを開く・
`global_camera_info()` を呼ぶだけで、差分を渡しても標準のまま写った）。だから

- `camera_node` は**最初のカメラを開く前に**1つ選び（`pick_name`）、全カメラへ同じものを渡す
- 前後で `isp_tuning` が違うとき（片方だけ純正レンズ等）は**先頭（前）のカメラのものが両方に効く**
- 標準のチューニングの名前を知るのにカメラへ問い合わせられないので、差分ファイルに
  `sensor` を書いておく（開いた後に `check_sensor` で食い違いを知らせる）

差分ファイルの形:

    {"sensor": "imx219",                       # 標準のチューニング `imx219.json` へ重ねる
     "rpi.alsc": {"calibrations_Cr": [...], "calibrations_Cb": [...]}}

`rpi.` で始まるキーが、標準のチューニングの同名のアルゴリズムへ浅く上書きされる。
"""

from __future__ import annotations

import copy
import json
import sys
from pathlib import Path
from typing import Callable

__all__ = ["TUNING_DIR", "apply_override", "pick_name", "lens_tuning", "check_sensor"]

TUNING_DIR = Path(__file__).resolve().parents[2] / "config" / "cam_tuning"


def apply_override(base: dict, override: dict) -> dict:
    """標準のチューニング `base` に差分 `override` を重ねたものを返す（`base` は書き換えない）。

    差分にあるアルゴリズムが標準に無ければ `ValueError`——名前の打ち間違いで
    「当てたつもりで何も変わっていない」のを防ぐ。
    """
    out = copy.deepcopy(base)
    algos = out.get("algorithms")
    if not isinstance(algos, list):
        raise ValueError("標準のチューニングに algorithms が無い（version 2 以外）")
    for name, params in override.items():
        if not name.startswith("rpi."):
            continue
        target = next((a[name] for a in algos if name in a), None)
        if target is None:
            raise ValueError(f"標準のチューニングに {name} が無い")
        target.update(params)
    return out


def pick_name(names: list[str]) -> str:
    """開くカメラの `isp_tuning`（開く順）から、プロセスで使う1つを選ぶ。"" ＝ 標準。

    チューニングはプロセスに1つしか持てない（冒頭の説明）。食い違うときは先頭のカメラに
    合わせ、合わない方のカメラの色がずれることを知らせる。
    """
    if not names:
        return ""
    if len(set(names)) > 1:
        print(f"!! カメラごとの isp_tuning が違う {names}。チューニングはプロセスに1つなので "
              f"先頭の {names[0] or '標準'} を全カメラに使う（合わないカメラは色むらが出る）",
              file=sys.stderr)
    return names[0]


def lens_tuning(name: str, load_base: Callable[[str], dict],
                tuning_dir: Path = TUNING_DIR) -> tuple[dict | None, str]:
    """`isp_tuning = "<name>"` の (チューニング, そのセンサー名) を返す。**`None` ＝ 標準のまま撮る。**

    `load_base` は標準のチューニングを読む関数（`Picamera2.load_tuning_file`。カメラを
    数えないので、開く前に呼んでよい）。差分が読めないときは理由を stderr に出して
    `None` を返す——**色が合わないだけで、映像は止めない。**
    """
    if not name:
        return None, ""
    path = tuning_dir / name
    try:
        override = json.loads(path.read_text(encoding="utf-8"))
        sensor = override.get("sensor")
        if not sensor:
            raise ValueError("sensor が書かれていない（どの標準へ重ねるか分からない）")
        return apply_override(load_base(f"{sensor}.json"), override), str(sensor)
    except Exception as e:                                     # noqa: BLE001
        print(f"!! ISP チューニング {path.name} を使えない（標準のまま撮る）: "
              f"{type(e).__name__}: {e}", file=sys.stderr)
        return None, ""


def check_sensor(want: str, model: str, idx: int) -> bool:
    """開いたカメラのセンサーが、当てたチューニングのものか。違えば知らせて False。

    違うセンサーに当たると色行列・黒レベルまで別物になる（センサーを替えたのに
    レンズのプロファイルの `isp_tuning` を消し忘れたとき）。`want` は `lens_tuning` が返した
    センサー名（"" ＝ 標準で開いた）。
    """
    if not want or not model or want == model:
        return True
    print(f"!! cam{idx} は {model} なのに {want} 用の ISP チューニングが当たっている。"
          f"vehicle.toml の isp_tuning を消すか作り直す", file=sys.stderr)
    return False
