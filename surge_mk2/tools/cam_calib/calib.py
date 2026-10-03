"""チェッカーボード写真 → 魚眼レンズの校正値（`FisheyeCalib`）。画面を持たない純ロジック。

`gui.py`（画面）と `__main__.py`（CLI）がここを呼ぶ。テストは `tests/` で、既知の
レンズで描いた合成画像から元の値に戻るかを確かめる。

## 下端クロップされた写真をそのまま使う

📷 で撮った写真は camera_node が ISP で下端を切り落とした画像（前カメラは下 1/4）。
`raspi/core/camera_model.py` の docstring のとおり、クロップは「フル画像の下を切っただけ」
なので、**写真の画素座標はそのままフル画像の画素座標**として扱える。校正には
クロップ前の大きさ `(幅, 高さ / (1 - crop))` を渡し、主点 `cy` もフル画像の座標で求まる。
クロップ率はファイル名（`surge_front_640x360_crop0.25_…png`）から読む。

ただし切り落とした部分には角点が無いので、画像の下端付近の歪みは外挿になる。
自動運転が使うのも切り落とした後の画像なので、実用上はそれで足りる。
"""

from __future__ import annotations

import math
import re
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from raspi.core.camera_model import FisheyeCalib, Undistorter, horizontal_fov  # noqa: E402

__all__ = ["Detection", "CalibResult", "parse_name", "detect", "calibrate",
           "coverage", "undistort_preview", "write_vehicle_toml", "DEFAULT_TOML"]

DEFAULT_TOML = REPO_ROOT / "config" / "vehicle.toml"

#: 📷 の保存名（`telemetry_node._serve_snapshot`）。`surge_<cam>_<W>x<H>_crop<率>_<時刻>.png`
_NAME_RE = re.compile(r"surge_(front|rear)_(\d+)x(\d+)_crop([0-9.]+)_")

#: これ未満の枚数では校正しない（fisheye.calibrate は 4 パラメータの歪みを解くので
#: 画面の広い範囲に角点が要る。少ないと端の歪みが外挿になって GUI のガイドが曲がる）
MIN_IMAGES = 6

#: レンズの公称の対角画角 [deg]。焦点距離の初期値（等距離射影 r = f·θ で
#: 対角の端が θ = 公称/2 になる f）に使う。**初期値を与えないと OpenCV は
#: f = 幅/π から始め、端の角点が光軸から 90° を超えた扱いになって初期値計算
#: （`InitExtrinsics`）が破綻する**（合成画像のテストで再現）
NOMINAL_DIAG_FOV_DEG = 160.0

#: 再投影誤差がこれを超えたら結果を使わせない（どこかで破綻している）
MAX_RMS_PX = 3.0


def parse_name(path: str | Path) -> tuple[str, float] | None:
    """ファイル名から `(カメラ, 下端クロップ率)`。📷 の名前でなければ None。"""
    m = _NAME_RE.search(Path(path).name)
    if not m:
        return None
    return m.group(1), float(m.group(4))


@dataclass
class Detection:
    """1枚ぶんの角点検出の結果。"""

    path: Path
    ok: bool
    #: 角点 (N,1,2) float64。画像（＝フル画像）の画素座標
    corners: np.ndarray | None
    #: 写真の大きさ (幅, 高さ)。クロップ後
    size: tuple[int, int]
    crop: float
    error: str = ""

    @property
    def full_size(self) -> tuple[int, int]:
        """クロップ前のフル画像の大きさ。"""
        w, h = self.size
        return w, int(round(h / (1.0 - self.crop)))


@dataclass
class CalibResult:
    calib: FisheyeCalib
    rms: float
    #: 使った写真 → 再投影誤差の RMS [px]
    per_image: dict[Path, float]
    #: 外した写真 → 理由
    rejected: dict[Path, str] = field(default_factory=dict)

    @property
    def hfov(self) -> float:
        return horizontal_fov(self.calib)


def _cv2():
    try:
        import cv2
    except ImportError as e:                                  # pragma: no cover
        raise RuntimeError("OpenCV が無い: .venv/bin/pip install -r tools/requirements.txt") from e
    return cv2


def _fisheye_flag(cv2, name: str) -> int:
    """`cv2.fisheye.CALIB_*`。OpenCV 5 では `cv2.CALIB_*` に移った（値も変わった）。"""
    v = getattr(cv2.fisheye, name, None)
    return int(v if v is not None else getattr(cv2, name))


def detect(path: str | Path, board: tuple[int, int],
           crop: float | None = None) -> Detection:
    """チェッカーボードの内側の角点を探す。

    :param board: 内側の角点の数（列, 行）。9x6 なら白黒 10x7 マスのボード。
        **片方を偶数・片方を奇数にする**——両方奇数（7x5 など）だとボードが 180° 回転
        対称になり、写真ごとに角点の並び順が逆転して校正が破綻する
    :param crop: 下端クロップ率。None ならファイル名から読む（読めなければ 0）
    """
    cv2 = _cv2()
    path = Path(path)
    if crop is None:
        parsed = parse_name(path)
        crop = parsed[1] if parsed else 0.0
    img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        return Detection(path, False, None, (0, 0), crop, "画像を読めない")
    size = (img.shape[1], img.shape[0])
    # SB（セクターベース）版の方が魚眼の端で強く歪んだボードでも拾え、精度も高い
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    ok, corners = cv2.findChessboardCornersSB(img, board, flags)
    if not ok:
        ok, corners = cv2.findChessboardCorners(
            img, board, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ok:
            corners = cv2.cornerSubPix(
                img, corners, (5, 5), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.01))
    if not ok:
        return Detection(path, False, None, size, crop, "ボードが見つからない")
    return Detection(path, True, corners.reshape(-1, 1, 2).astype(np.float64), size, crop)


def _object_points(board: tuple[int, int], square: float) -> np.ndarray:
    cols, rows = board
    obj = np.zeros((cols * rows, 1, 3), np.float64)
    obj[:, 0, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    return obj


def _per_image_errors(cv2, obj, dets, K, D, rvecs, tvecs) -> list[float]:
    errs = []
    for d, r, t in zip(dets, rvecs, tvecs):
        proj, _ = cv2.fisheye.projectPoints(obj.reshape(1, -1, 3), r, t, K, D)
        diff = proj.reshape(-1, 2) - d.corners.reshape(-1, 2)
        errs.append(float(np.sqrt(np.mean(np.sum(diff * diff, axis=1)))))
    return errs


def _find_breaking_image(cv2, obj, dets, size, flags, K0) -> int | None:
    """1枚抜くと校正が通るようになる写真の番号。見つからなければ None。"""
    crit = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 20, 1e-6)
    for i in range(len(dets)):
        rest = dets[:i] + dets[i + 1:]
        try:
            cv2.fisheye.calibrate(
                [obj.reshape(1, -1, 3)] * len(rest),
                [d.corners.reshape(1, -1, 2) for d in rest], size,
                K0.copy(), np.zeros((4, 1)), flags=flags, criteria=crit)
        except cv2.error:
            continue
        return i
    return None


def calibrate(detections: list[Detection], board: tuple[int, int], square: float,
              *, outlier_px: float = 1.0, max_drop: int | None = None,
              nominal_diag_fov_deg: float = NOMINAL_DIAG_FOV_DEG) -> CalibResult:
    """検出できた写真から `cv2.fisheye.calibrate` で校正する。

    - 焦点距離・主点の初期値を公称画角から与える（`NOMINAL_DIAG_FOV_DEG` 参照）
    - `CALIB_CHECK_COND` は**使わない**。良い写真でも「条件が悪い」と言って先頭から
      順に外していくほど過敏で、合成画像のテストで正しい写真の半分を捨てた
    - OpenCV が初期値計算で落ちたら（どの写真かは教えてくれない）、1枚ずつ抜いて試し、
      抜くと通る写真を外す（`_find_breaking_image`）
    - 校正後、再投影誤差が `max(outlier_px, 中央値×3)` を超える写真（角点の誤検出・ブレ）を
      1枚ずつ外してやり直す（`max_drop` 枚まで。既定は全体の 1/3）

    :raises ValueError: 使える写真が `MIN_IMAGES` 枚未満、または解像度が揃っていない
    """
    cv2 = _cv2()
    dets = [d for d in detections if d.ok]
    rejected: dict[Path, str] = {d.path: d.error for d in detections if not d.ok}
    sizes = {d.full_size for d in dets}
    if len(sizes) > 1:
        raise ValueError(f"解像度（クロップ前）が揃っていない: {sorted(sizes)}"
                         "——カメラ・解像度ごとに分けて校正する")
    if len(dets) < MIN_IMAGES:
        raise ValueError(f"ボードが写っている写真が {len(dets)} 枚しかない（{MIN_IMAGES} 枚以上要る）")
    full_w, full_h = sizes.pop()
    obj = _object_points(board, square)
    if max_drop is None:
        max_drop = len(dets) // 3
    crit = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 200, 1e-9)
    dropped = 0

    while True:
        if len(dets) < MIN_IMAGES:
            raise ValueError(f"外れ値を外したら {len(dets)} 枚しか残らなかった"
                             f"（{MIN_IMAGES} 枚以上要る）。撮り直しを勧める")
        flags = (_fisheye_flag(cv2, "CALIB_RECOMPUTE_EXTRINSIC")
                 | _fisheye_flag(cv2, "CALIB_FIX_SKEW")
                 | _fisheye_flag(cv2, "CALIB_USE_INTRINSIC_GUESS"))
        f0 = math.hypot(full_w, full_h) / 2.0 / math.radians(nominal_diag_fov_deg / 2.0)
        K0 = np.array([[f0, 0.0, full_w / 2.0], [0.0, f0, full_h / 2.0], [0.0, 0.0, 1.0]])
        try:
            # 形は (1, N, 3)/(1, N, 2)。(N, 1, 3) は OpenCV 5 の fisheye で落ちる
            rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
                [obj.reshape(1, -1, 3)] * len(dets),
                [d.corners.reshape(1, -1, 2) for d in dets], (full_w, full_h),
                K0.copy(), np.zeros((4, 1)), flags=flags, criteria=crit)
        except cv2.error as e:
            # `InitExtrinsics` の初期値計算が特定の写真で破綻する等。
            # OpenCV はどの写真かを教えてくれない → 1枚ずつ抜いて試し、通った写真を外す
            bad_i = _find_breaking_image(cv2, obj, dets, (full_w, full_h), flags, K0)
            if bad_i is None:
                raise ValueError(f"校正が収束しなかった: {e}") from e
            bad = dets.pop(bad_i)
            rejected[bad.path] = "校正の初期値計算が破綻する（ボードの向き・写り方が極端）"
            continue

        errs = _per_image_errors(cv2, obj, dets, K, D, rvecs, tvecs)
        med = float(np.median(errs))
        worst = int(np.argmax(errs))
        if errs[worst] > max(outlier_px, 3.0 * med) and dropped < max_drop:
            bad = dets.pop(worst)
            rejected[bad.path] = f"再投影誤差が大きい（{errs[worst]:.2f}px）"
            dropped += 1
            continue
        break

    if rms > MAX_RMS_PX:
        raise ValueError(
            f"校正結果が破綻している（再投影誤差 {rms:.1f}px）。角点の並び順が写真ごとに"
            "ばらついている可能性が高い——ボードの内側の角点数は片方を偶数・片方を奇数"
            "（例 9x6）にして向きが一意に決まるようにし、指定した数が実物と合っているか確認する")
    calib = FisheyeCalib(fx=float(K[0, 0]), fy=float(K[1, 1]), cx=float(K[0, 2]),
                         cy=float(K[1, 2]), k=tuple(float(v) for v in D.ravel()),  # type: ignore[arg-type]
                         width=full_w, height=full_h, rms=float(rms))
    return CalibResult(calib=calib, rms=float(rms),
                       per_image={d.path: e for d, e in zip(dets, errs)}, rejected=rejected)


def coverage(detections: list[Detection], grid: tuple[int, int] = (4, 3)) -> float:
    """写真全体で、画面を `grid` に区切ったマスのうち角点が1つでも入った割合。

    魚眼は端ほど歪みが大きいので、**端・四隅にもボードを写した写真が要る。**
    中央ばかりだと RMS が小さくても端のガイドが曲がる。目安は 0.8 以上。
    """
    gx, gy = grid
    hit = np.zeros((gy, gx), bool)
    for d in detections:
        if not d.ok:
            continue
        w, h = d.size
        pts = d.corners.reshape(-1, 2)
        ix = np.clip((pts[:, 0] / w * gx).astype(int), 0, gx - 1)
        iy = np.clip((pts[:, 1] / h * gy).astype(int), 0, gy - 1)
        hit[iy, ix] = True
    return float(hit.mean())


def undistort_preview(path: str | Path, calib: FisheyeCalib, crop: float,
                      hfov_out: float = math.radians(110)) -> np.ndarray:
    """左に元画像、右に補正画像（GUI の補正映像と同じ変換）を並べた BGR 画像。"""
    cv2 = _cv2()
    img = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError(f"画像を読めない: {path}")
    und = Undistorter(calib, hfov_out, crop)(img)
    return np.hstack([img, und])


def write_vehicle_toml(cam: str, result: CalibResult,
                       toml_path: str | Path = DEFAULT_TOML,
                       *, regenerate: bool = True) -> None:
    """`[sensors.cam_<cam>.fisheye]` を書く（既にあれば丸ごと置き換える）。

    **`hfov`（未校正時のピンホール近似）は書き換えない。** 160° 級では水平画角が
    180° に迫り、ピンホールの式（tan(hfov/2)）に入れると破綻するため。校正値から
    求めた水平画角は参考値として `fisheye.hfov_deg` に残す。

    コメント・書式は `tomlkit` で保つ（`tools/sysid/toml_update.py` と同じ流儀）。
    `regenerate` なら `config/generate.py` も走らせて GUI の定数を揃える
    （揃えないと CI の `--check` が落ちる）。
    """
    import tomlkit

    if cam not in ("front", "rear"):
        raise ValueError(f"カメラは front/rear: {cam}")
    path = Path(toml_path)
    doc = tomlkit.parse(path.read_text(encoding="utf-8"))
    sensors = doc.get("sensors")
    if sensors is None or f"cam_{cam}" not in sensors:
        raise ValueError(f"{path} に [sensors.cam_{cam}] が無い")
    tbl = sensors[f"cam_{cam}"]
    c = result.calib
    fe = tomlkit.table()
    fe.add(tomlkit.comment(f"tools/cam_calib が書いた（{len(result.per_image)} 枚、"
                           f"再投影誤差 {result.rms:.3f}px）。手で書き換えない"))
    fe["width"] = int(c.width)
    fe["height"] = int(c.height)
    fe["fx"] = float(round(c.fx, 4))
    fe["fy"] = float(round(c.fy, 4))
    fe["cx"] = float(round(c.cx, 4))
    fe["cy"] = float(round(c.cy, 4))
    fe["k"] = [float(round(v, 8)) for v in c.k]
    fe["rms"] = float(round(result.rms, 4))
    fe["hfov_deg"] = float(round(math.degrees(result.hfov), 1))     # 参考値（読み手は使わない）
    tbl["fisheye"] = fe
    path.write_text(tomlkit.dumps(doc), encoding="utf-8")
    if regenerate and path.resolve() == DEFAULT_TOML.resolve():
        subprocess.run([sys.executable, str(REPO_ROOT / "config" / "generate.py")],
                       check=True, cwd=REPO_ROOT)
