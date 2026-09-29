"""arrow_signal_node のHSVしきい値を、実機・実会場でその場チューニングする道具。

    .venv/bin/python -m raspi.tools.arrow_signal_preview                  # camera_node の共有メモリから
    .venv/bin/python -m raspi.tools.arrow_signal_preview --image shot.png  # 保存済み画像から

`cv2.imshow`のウィンドウにROI・点灯マスク・判定結果（left/right/straight/非該当）
を重ねて表示し、トラックバーで`ArrowSignalDetector`のしきい値（彩度・明度・
ROI帯）をその場で動かしながら追い込む。ディスプレイ（HDMI/VNC）が要る——
Piにヘッドレスでsshしているだけの環境では動かない（`--image`で保存画像を
Mac側で調整する運用も可）。

追い込んだ値は `q` で終了したときに `arrow_signal_node` へそのまま渡せる
CLI引数の形で表示する。色相帯（`--hue-bands`）はこのツールでは固定——
色そのものに意味は無く「薄いかどうか」（彩度・明度）だけが判定に効くため
（`arrow_signal_node.py`のモジュールdocstring参照）。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from raspi.nodes.arrow_signal_node import (  # noqa: E402
    _DEFAULT_HUE_BANDS,
    ArrowSignalDetector,
)

_WIN = "arrow_signal_preview（qで終了）"


def _load_image(path: Path) -> np.ndarray:
    arr = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if arr is None:
        raise FileNotFoundError(f"画像を読めません: {path}")
    return arr


def _frame_source_from_shm(name: str):
    """`camera_node.py`の共有メモリから最新フレームを取り出すイテレータ。

    `raspi/tools/shm_view.py`と同じ attach 手順。BGR888/RGB888いずれの並びでも
    `arrow_signal_node.signal_mask()`はBGR前提なので、`RGB888`格納時のみ並びを戻す
    （`shm_view.to_rgb()`と同じ判定）。
    """
    from raspi.bus import FrameRing

    ring = FrameRing.attach(name)
    try:
        last_seq = 0
        while True:
            ref = ring.latest()
            if ref is None or ref.desc.seq == last_seq:
                time.sleep(0.03)
                continue
            arr = ref.as_array().copy()
            ok = ref.still_valid()
            if not ok:
                continue
            last_seq = ref.desc.seq
            if ref.desc.fmt.startswith("RGB"):
                arr = arr[..., ::-1].copy()
            yield arr
    finally:
        ring.close()


def _draw_overlay(frame: np.ndarray, detector: ArrowSignalDetector) -> np.ndarray:
    h, w = frame.shape[:2]
    v0 = max(0, int(detector.roi_band[0] * h))
    v1 = min(h, int(detector.roi_band[1] * h))
    result = detector.process_frame(frame)

    vis = frame.copy()
    cv2.rectangle(vis, (0, v0), (w - 1, v1), (0, 255, 255), 2)
    if v1 > v0:
        from raspi.nodes.arrow_signal_node import signal_mask
        roi = frame[v0:v1, :, :]
        mask = signal_mask(roi, hue_bands=detector.hue_bands, sat_range=detector.sat_range,
                           val_min=detector.val_min)
        overlay = vis[v0:v1, :, :]
        overlay[mask] = (0, 0, 255)

    label = f"{result.value}  lit={result.lit_frac:.3f}"
    cv2.putText(vis, label, (8, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    return vis


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--image", type=Path, default=None,
                    help="共有メモリの代わりに1枚の保存画像を繰り返し表示する")
    ap.add_argument("--shm-name", default="surge_cam0",
                    help="camera_node の共有メモリ名（既定 surge_cam0）")
    args = ap.parse_args()

    detector = ArrowSignalDetector()

    cv2.namedWindow(_WIN)
    cv2.createTrackbar("sat_min", _WIN, detector.sat_range[0], 255, lambda v: None)
    cv2.createTrackbar("sat_max", _WIN, detector.sat_range[1], 255, lambda v: None)
    cv2.createTrackbar("val_min", _WIN, detector.val_min, 255, lambda v: None)
    cv2.createTrackbar("roi_top_pct", _WIN, int(detector.roi_band[0] * 100), 100, lambda v: None)
    cv2.createTrackbar("roi_bottom_pct", _WIN, int(detector.roi_band[1] * 100), 100,
                       lambda v: None)

    if args.image is not None:
        still = _load_image(args.image)
        source = iter(lambda: still, None)
    else:
        source = _frame_source_from_shm(args.shm_name)

    try:
        for frame in source:
            detector.sat_range = (cv2.getTrackbarPos("sat_min", _WIN),
                                  cv2.getTrackbarPos("sat_max", _WIN))
            detector.val_min = cv2.getTrackbarPos("val_min", _WIN)
            top = cv2.getTrackbarPos("roi_top_pct", _WIN) / 100.0
            bottom = cv2.getTrackbarPos("roi_bottom_pct", _WIN) / 100.0
            detector.roi_band = (min(top, bottom), max(top, bottom))

            cv2.imshow(_WIN, _draw_overlay(frame, detector))
            if cv2.waitKey(30) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()

    hue_str = ";".join(f"{lo},{hi}" for lo, hi in _DEFAULT_HUE_BANDS)
    print("# 追い込んだ値（色相帯は固定 — 上のモジュールdocstring参照）:")
    print(f"#   --roi-band {detector.roi_band[0]:.2f},{detector.roi_band[1]:.2f} "
          f"--sat-min {detector.sat_range[0]} --sat-max {detector.sat_range[1]} "
          f"--val-min {detector.val_min}")
    print(f"#   （色相帯 既定: {hue_str}）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
