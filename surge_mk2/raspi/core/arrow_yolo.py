"""矢印信号の YOLO 検出器（`arrow_signal_node` の「yolo」方式）。

古典CV（HSV しきい値＋重心）と GUI で切り替えて使う。`ArrowSignalDetector` と同じく
**1フレーム → `ArrowResult`** の純粋な部品で、バス・共有メモリを知らない。

## 置き場とモデルの形

`models/arrow_yolo/<名前>.onnx`。ultralytics（YOLOv8/11 など）の書き出しをそのまま置く:

    yolo export model=best.pt format=onnx imgsz=320 opset=12        # 出力 (1, 4+nc, N)
    yolo export model=best.pt format=onnx imgsz=320 nms=True        # 出力 (1, N, 6)（NMS 込み）

どちらの出力形も受ける。クラス名は ONNX のメタデータ（ultralytics が `names` を書く）から
取り、同名の `.json` に `{"names": [...], "note": "..."}` があればそちらを優先する。

## クラス名 → 方向

`left`/`right`/`straight`（別名 `center`・`forward`・`up` は `straight`）だけを方向として読む。
**それ以外のクラス（例: 濃い色の矢印を `left_dark` として学習させたもの）は「非該当」**——最も
確からしい検出がそれなら `None` を返す（安全側＝今の経路グループのまま。古典CVの「薄い色帯に
入らない点灯は無視」と同じ考え）。

## ROI

モデルには**画面全体**を入れる（学習も全体画像のはずで、切り出すと看板の大きさが学習時と
ずれる）。検出の中心が ROI 帯（画面高さの割合）の外なら捨てる——GUI の ROI 枠線が
古典CVと同じ意味を持つ。
"""

from __future__ import annotations

import ast
import time
from pathlib import Path

import cv2
import numpy as np

__all__ = ["YoloArrowDetector", "class_to_direction", "postprocess", "letterbox"]

#: クラス名 → 方向。ここに無い名前は「非該当」
_DIRECTION_ALIASES = {
    "left": "left", "l": "left", "arrow_left": "left",
    "right": "right", "r": "right", "arrow_right": "right",
    "straight": "straight", "center": "straight", "forward": "straight", "up": "straight",
    "arrow_straight": "straight",
}

#: 入力の大きさが ONNX から読めない（動的）ときの既定
_DEFAULT_IMGSZ = 320
#: NMS の IoU しきい値
_NMS_IOU = 0.45


def class_to_direction(name: str) -> str | None:
    return _DIRECTION_ALIASES.get(name.strip().lower())


def letterbox(frame: np.ndarray, size: tuple[int, int]) -> tuple[np.ndarray, float, tuple[int, int]]:
    """BGR `(H, W, 3)` → 縦横比を保って `size=(h, w)` に収め、余白を 114 で埋めた画像。

    戻り値の `scale`・`(pad_x, pad_y)` で検出枠を元画像の座標へ戻す（ultralytics と同じ前処理）。
    """
    h0, w0 = frame.shape[:2]
    th, tw = size
    scale = min(th / h0, tw / w0)
    nh, nw = max(1, round(h0 * scale)), max(1, round(w0 * scale))
    resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
    out = np.full((th, tw, 3), 114, dtype=np.uint8)
    px, py = (tw - nw) // 2, (th - nh) // 2
    out[py:py + nh, px:px + nw] = resized
    return out, scale, (px, py)


def postprocess(out: np.ndarray, *, conf_min: float, n_classes: int,
                iou: float = _NMS_IOU) -> list[tuple[float, float, float, float, float, int]]:
    """ONNX の生出力 → `[(x0, y0, x1, y1, score, cls), ...]`（入力画像の画素座標、score の降順）。

    - `(1, 4+nc, N)`: ultralytics の既定。cx,cy,w,h＋クラスごとの確率。ここで NMS を掛ける
    - `(1, N, 6)`: NMS 込みの書き出し（`nms=True`・YOLOv10）。x0,y0,x1,y1,score,cls
    """
    a = np.asarray(out, dtype=np.float32)
    if a.ndim == 3:
        a = a[0]
    if a.ndim != 2:
        return []
    if a.shape[1] == 6 and a.shape[0] != 4 + n_classes:
        dets = [tuple(map(float, r[:5])) + (int(r[5]),) for r in a if r[4] >= conf_min]
        return sorted(dets, key=lambda d: -d[4])
    if a.shape[0] == 4 + n_classes:
        a = a.T                                  # (N, 4+nc)
    if a.shape[1] < 5:
        return []
    scores_all = a[:, 4:]
    cls = scores_all.argmax(axis=1)
    score = scores_all[np.arange(len(a)), cls]
    keep = score >= conf_min
    if not keep.any():
        return []
    b, score, cls = a[keep, :4], score[keep], cls[keep]
    x0 = b[:, 0] - b[:, 2] / 2
    y0 = b[:, 1] - b[:, 3] / 2
    rects = [[float(x), float(y), float(w), float(h)]
             for x, y, w, h in zip(x0, y0, b[:, 2], b[:, 3])]
    idx = cv2.dnn.NMSBoxes(rects, score.astype(float).tolist(), conf_min, iou)
    idx = np.asarray(idx).reshape(-1)
    dets = [(rects[i][0], rects[i][1], rects[i][0] + rects[i][2], rects[i][1] + rects[i][3],
             float(score[i]), int(cls[i])) for i in idx]
    return sorted(dets, key=lambda d: -d[4])


def _read_names(session, json_path: Path) -> list[str]:
    """クラス名。同名 `.json` の `names` → ONNX のメタデータ（ultralytics）の順に探す。"""
    import msgspec

    if json_path.is_file():
        try:
            raw = msgspec.json.decode(json_path.read_bytes())
            names = raw.get("names") if isinstance(raw, dict) else None
            if isinstance(names, dict):
                names = [names[k] for k in sorted(names, key=lambda k: int(k))]
            if isinstance(names, list) and names:
                return [str(n) for n in names]
        except (OSError, msgspec.DecodeError, ValueError, AttributeError):
            pass
    meta = session.get_modelmeta().custom_metadata_map or {}
    if "names" in meta:
        try:
            d = ast.literal_eval(meta["names"])    # "{0: 'left', 1: 'right'}"
            if isinstance(d, dict):
                return [str(d[k]) for k in sorted(d)]
            if isinstance(d, (list, tuple)):
                return [str(n) for n in d]
        except (ValueError, SyntaxError):
            pass
    return []


class YoloArrowDetector:
    """1フレーム → `ArrowResult`。`load()` に失敗したら `error` に理由が入り、`ready` が偽になる。

    推論は重いので `min_period_s` より短い間隔では回さない（`due()` を呼び手が見る）。
    """

    def __init__(self, *, conf_min: float = 0.5, roi_band: tuple[float, float] = (0.0, 1.0),
                 min_period_s: float = 0.1, intra_threads: int = 2,
                 clock=time.monotonic) -> None:
        self.conf_min = conf_min
        self.roi_band = roi_band
        self.min_period_s = min_period_s
        self.intra_threads = intra_threads
        self._clock = clock
        self._t_last = -1e9
        self.session = None
        self.names: list[str] = []
        self.input_name = ""
        self.imgsz: tuple[int, int] = (_DEFAULT_IMGSZ, _DEFAULT_IMGSZ)
        self.model_path: Path | None = None
        self.error = ""
        #: 直近の推論時間 [ms]（GUI・ログ用）
        self.last_infer_ms = 0.0

    @property
    def ready(self) -> bool:
        return self.session is not None

    def load(self, path: str | Path, *, session=None) -> bool:
        """`path` の ONNX を読む。`session` はテストが偽物を差し込む口。"""
        path = Path(path)
        self.session, self.names, self.error = None, [], ""
        self.model_path = path
        try:
            if session is None:
                from raspi.core.onnx_session import make_session
                session = make_session(path, intra_threads=self.intra_threads)
            inp = session.get_inputs()[0]
            self.input_name = inp.name
            shape = list(inp.shape)
            h = shape[2] if len(shape) == 4 and isinstance(shape[2], int) else _DEFAULT_IMGSZ
            w = shape[3] if len(shape) == 4 and isinstance(shape[3], int) else _DEFAULT_IMGSZ
            self.imgsz = (h, w)
            self.names = _read_names(session, path.with_suffix(".json"))
            if not self.names:
                raise ValueError("クラス名が読めない（.json の names か ONNX のメタデータが要る）")
            if not any(class_to_direction(n) for n in self.names):
                raise ValueError(f"left/right/straight のクラスが無い: {self.names}")
        except Exception as e:  # noqa: BLE001 — 読めない理由は何でも GUI に出して古典CVへ戻す
            self.session = None
            self.error = f"{path.name}: {e}"
            return False
        self.session = session
        return True

    def unload(self) -> None:
        self.session, self.names, self.model_path, self.error = None, [], None, ""

    def due(self) -> bool:
        return self._clock() - self._t_last >= self.min_period_s

    def process_frame(self, frame: np.ndarray):
        """BGR の `(H, W, 3)` → `ArrowResult`（`score` に確からしさ、`box` に正規化した枠）。"""
        from raspi.nodes.arrow_signal_node import ArrowResult

        self._t_last = self._clock()
        if self.session is None:
            return ArrowResult(None)
        h0, w0 = frame.shape[:2]
        img, scale, (px, py) = letterbox(frame[..., :3], self.imgsz)
        x = img[..., ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        t0 = time.perf_counter()
        out = self.session.run(None, {self.input_name: np.ascontiguousarray(x)})[0]
        self.last_infer_ms = (time.perf_counter() - t0) * 1e3
        dets = postprocess(out, conf_min=self.conf_min, n_classes=len(self.names))
        for x0, y0, x1, y1, score, c in dets:
            # 入力画像 → 元画像の正規化座標
            bx0 = (x0 - px) / scale / w0
            by0 = (y0 - py) / scale / h0
            bx1 = (x1 - px) / scale / w0
            by1 = (y1 - py) / scale / h0
            cy = (by0 + by1) / 2
            if not (self.roi_band[0] <= cy <= self.roi_band[1]):
                continue                          # ROI 帯の外（看板ではない場所）
            name = self.names[c] if 0 <= c < len(self.names) else ""
            box = [max(0.0, bx0), max(0.0, by0), min(1.0, bx1), min(1.0, by1)]
            # 最も確からしい検出だけで決める。方向でないクラスなら非該当（安全側）
            return ArrowResult(class_to_direction(name), score=score, box=box, label=name)
        return ArrowResult(None)
