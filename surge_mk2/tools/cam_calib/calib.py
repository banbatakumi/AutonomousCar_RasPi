"""チェッカーボード写真 → 魚眼レンズの校正値（`FisheyeCalib`）。画面を持たない純ロジック。

`gui.py`（画面）と `__main__.py`（CLI）がここを呼ぶ。テストは `tests/` で、既知の
レンズで描いた合成画像から元の値に戻るかを確かめる。

## 下端クロップされた写真をそのまま使う

📷 で撮った写真は camera_node が ISP で下端を切り落とした画像（前カメラは下 1/4）。
`raspi/core/camera_model.py` の docstring のとおり、クロップは「フル画像の下を切っただけ」
なので、**写真の画素座標はそのままフル画像の画素座標**として扱える。校正には
クロップ前の大きさ `(幅, 高さ / (1 - crop))` を渡し、主点 `cy` もフル画像の座標で求まる。
クロップ率はファイル名（`surge_front_wide160_640x360_crop0.25_…png`）から読む。

## 校正値はレンズのプロファイルごと

`vehicle.toml` はレンズを `[sensors.cam_*.lenses.<名前>]` に分けて持ち、`lens = "<名前>"` で
選ぶ（`raspi/core/vehicle.py` の `resolve_lens`）。校正値はその**プロファイルの中**
（`[….lenses.<名前>.fisheye]`）に書くので、レンズを付け替えても両方の値が残る。
書き込み先は、写真のファイル名に入っているレンズ名（📷 が撮ったときに付いていたレンズ）が既定。

ただし切り落とした部分には角点が無いので、画像の下端付近の歪みは外挿になる。
自動運転が使うのも切り落とした後の画像なので、実用上はそれで足りる。

## ステレオ（`docs/stereo_plan.md` 段階2）

📷 は右（`front`）と左（`left`）を順に落とす。名前の時刻が近い2枚を組にし
（`pair_shots`）、左右それぞれの内部パラメータ（`calibrate`）を固定して
`cv2.fisheye.stereoCalibrate` で左 → 右の R・T を求める（`stereo_calibrate`）。
結果は `[sensors.stereo]`（`write_stereo_toml`）。内部パラメータの書き込み先は、写真の名前の
レンズ（📷 が撮ったときの各カメラの `lens`）。
"""

from __future__ import annotations

import math
import re
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import NamedTuple

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from raspi.core.camera_model import FisheyeCalib, Undistorter, horizontal_fov  # noqa: E402
from raspi.core.stereo_model import StereoCalib  # noqa: E402

__all__ = ["ShotName", "Detection", "CalibResult", "parse_name", "detect", "calibrate",
           "coverage", "undistort_preview", "lens_profiles", "write_vehicle_toml", "DEFAULT_TOML",
           "CAMS", "shot_time", "StereoPair", "StereoResult", "pair_shots", "stereo_calibrate",
           "rect_dy", "aligned_left_corners", "write_stereo_toml", "StereoCalibration",
           "calibrate_stereo_set", "write_stereo_set", "rectified_preview"]

DEFAULT_TOML = REPO_ROOT / "config" / "vehicle.toml"

#: 📷 の保存名（`telemetry_node._serve_snapshot`）。
#: `surge_<cam>_<lens>_<W>x<H>_crop<率>_<時刻>.png`。`<lens>_` はレンズのプロファイルを
#: 使わない設定のときは付かない（2026-10-03 より前の名前もこの形）
_NAME_RE = re.compile(r"surge_(front|left)_(?:([A-Za-z0-9-]+)_)?(\d+)x(\d+)_crop([0-9.]+)_")
#: 保存名の末尾の時刻 `YYYYmmdd_HHMMSS<ミリ秒3桁>`（ステレオの組を作るのに使う）
_STAMP_RE = re.compile(r"_(\d{8})_(\d{6})(\d{3})(?:\D[^/]*)?\.png$")

#: カメラ名（📷 の `<cam>`）。前方ステレオの右＝主カメラ `front` と左 `left`
CAMS = ("front", "left")

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


class ShotName(NamedTuple):
    cam: str        #: "front"（右・主）/ "left"
    lens: str       #: レンズのプロファイル名。名前に無ければ ""
    crop: float     #: 下端クロップ率


def parse_name(path: str | Path) -> ShotName | None:
    """📷 のファイル名を読む。📷 の名前でなければ None。"""
    m = _NAME_RE.search(Path(path).name)
    if not m:
        return None
    return ShotName(m.group(1), m.group(2) or "", float(m.group(5)))


def shot_time(path: str | Path) -> float | None:
    """📷 のファイル名の時刻 [s]（エポック秒・ローカル時刻）。読めなければ None。

    ブラウザが同名のファイルに付ける ` (1)` などの後置きは無視する。
    """
    import time as _time

    m = _STAMP_RE.search(Path(path).name)
    if not m:
        return None
    try:
        t = _time.mktime(_time.strptime(m.group(1) + m.group(2), "%Y%m%d%H%M%S"))
    except ValueError:
        return None
    return t + int(m.group(3)) / 1000.0


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
    #: 撮ったときのレンズ（ファイル名から。不明なら ""）
    lens: str = ""

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
    #: 写真のレンズ名（ファイル名から。不明なら ""）。書き込み先の既定になる
    lens: str = ""

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
    parsed = parse_name(path)
    lens = parsed.lens if parsed else ""
    if crop is None:
        crop = parsed.crop if parsed else 0.0
    img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        return Detection(path, False, None, (0, 0), crop, "画像を読めない", lens)
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
        return Detection(path, False, None, size, crop, "ボードが見つからない", lens)
    return Detection(path, True, corners.reshape(-1, 1, 2).astype(np.float64), size, crop,
                     lens=lens)


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
    lenses = {d.lens for d in dets}
    if len(lenses) > 1:
        raise ValueError(f"別のレンズの写真が混ざっている: {sorted(lenses)}——レンズごとに分けて校正する")
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
                       per_image={d.path: e for d, e in zip(dets, errs)}, rejected=rejected,
                       lens=lenses.pop())


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


def _selected_lens(cam_tbl) -> str:
    """表で今選ばれているレンズ名（`resolve_lens` と同じ規則）。"""
    return str(cam_tbl.get("lens", "") or "")


def lens_profiles(cam: str, toml_path: str | Path = DEFAULT_TOML) -> tuple[str, list[str]]:
    """`(今選ばれているレンズ, プロファイル名の一覧)`。プロファイルを使わない書き方なら `("", [])`。"""
    import tomllib

    with open(toml_path, "rb") as f:
        d = tomllib.load(f)
    sensors = d.get("sensors", {})
    tbl = sensors.get(f"cam_{cam}", {})
    return _selected_lens(tbl), sorted(tbl.get("lenses", {}) or {})


def _replace_table(parent, key: str, new) -> None:
    """`parent[key]` を表 `new` に置き換える。**元の表の末尾の空行・コメントは引き継ぐ。**

    tomlkit では、次の表の見出しの上に書いたコメント（`# ステレオの左カメラ`）は
    前の表の末尾に属する。丸ごと置き換えるとそれが消えるので、最後のキーより後ろの
    空行・コメントを新しい表の末尾に付け直す。
    """
    import tomlkit
    from tomlkit.items import Comment

    old = parent.get(key)
    comments = []
    if isinstance(old, tomlkit.items.Table):
        for k, item in reversed(old.value.body):
            if k is not None:
                break
            if isinstance(item, Comment):
                comments.append(item)
    if comments:
        new.add(tomlkit.ws("\n"))                 # 最後のキーと次の見出しのコメントの間を1行あける
        for item in reversed(comments):
            new.value.body.append((None, item))
    parent[key] = new


def write_vehicle_toml(cam: str, result: CalibResult,
                       toml_path: str | Path = DEFAULT_TOML,
                       *, lens: str | None = None, regenerate: bool = True) -> str:
    """校正値を `[sensors.cam_<cam>.lenses.<lens>.fisheye]` に書く（既にあれば丸ごと置き換える）。

    書き込み先のレンズは `lens` → 写真のレンズ（`result.lens`）→ 今選ばれている `lens` の順に決める。
    **存在しないプロファイルには書かない**（打ち間違いで新しいプロファイルができると、
    選ばれていないので黙って使われない）。プロファイルを使わない書き方（`lens` が無い）の
    ときは従来どおり `[sensors.cam_<cam>.fisheye]` に書く。書いた先のレンズ名を返す（無ければ ""）。

    **`hfov`（未校正時のピンホール近似）は書き換えない。** 160° 級では水平画角が
    180° に迫り、ピンホールの式（tan(hfov/2)）に入れると破綻するため。校正値から
    求めた水平画角は参考値として `fisheye.hfov_deg` に残す。

    コメント・書式は `tomlkit` で保つ（`tools/calib/toml_io.py` と同じ流儀）。
    `regenerate` なら `config/generate.py` も走らせて GUI の定数を揃える
    （揃えないと CI の `--check` が落ちる）。
    """
    import tomlkit

    if cam not in CAMS:
        raise ValueError(f"カメラは {'/'.join(CAMS)}: {cam}")
    path = Path(toml_path)
    doc = tomlkit.parse(path.read_text(encoding="utf-8"))
    sensors = doc.get("sensors")
    if sensors is None or f"cam_{cam}" not in sensors:
        raise ValueError(f"{path} に [sensors.cam_{cam}] が無い")
    cam_tbl = sensors[f"cam_{cam}"]
    profiles = cam_tbl.get("lenses")
    target_name = lens or result.lens or _selected_lens(cam_tbl)
    if profiles is not None:
        if not target_name:
            raise ValueError(f"[sensors.cam_{cam}] に lens が無く、書き込み先のレンズが決まらない")
        if target_name not in profiles:
            raise ValueError(f"レンズ \"{target_name}\" のプロファイルが無い（あるのは "
                             f"{sorted(profiles)}）。[sensors.cam_{cam}.lenses.{target_name}] を"
                             "先に vehicle.toml に足す")
        tbl = profiles[target_name]
    else:
        if lens:
            raise ValueError(f"[sensors.cam_{cam}] にレンズのプロファイル（lenses）が無い")
        tbl, target_name = cam_tbl, ""
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
    _replace_table(tbl, "fisheye", fe)
    path.write_text(tomlkit.dumps(doc), encoding="utf-8")
    if regenerate and path.resolve() == DEFAULT_TOML.resolve():
        subprocess.run([sys.executable, str(REPO_ROOT / "config" / "generate.py")],
                       check=True, cwd=REPO_ROOT)
    return target_name


# ── ステレオ校正（`docs/stereo_plan.md` 段階2） ──────────────────────────────

#: 📷 は前（右）と左を順に落とすので、名前の時刻は数百 ms ずれる。これ以内を同じ瞬間の組とみなす。
#: ボードは静止させて撮る前提（同期は要らない）なので、次の1枚（数秒後）と取り違えない幅にする
PAIR_MAX_DT_S = 1.5

#: これ未満の組数ではステレオ校正しない（R・T は 6 自由度だが、角点の検出誤差をならすため）
MIN_PAIRS = 5

#: 合否（`docs/stereo_plan.md` 段階2）: 平行化後の対応点の縦ずれ（**中央値**）[px] と基線の公称・許容 [m]。
#: 95% 点にしないのは、角点の検出の外れ値（魚眼の周辺で 1px 級）に支配されて校正の良し悪しを
#: 測れないため。合成画像で、真の R・T でも 95% 点は 0.69px、中央値は 0.10px。R にピッチ 0.1° の
#: 誤差を入れると中央値は 0.53px になる（系統的なずれは中央値をそのまま動かす）
MAX_RECT_DY_PX = 0.5
NOMINAL_BASELINE_M = 0.075
BASELINE_TOL_M = 0.002

#: 合否を見る平行化画像の水平画角（`stereo_node` が使う予定の値。縦ずれは画角にほぼ比例する）
CHECK_HFOV_RAD = math.radians(100)


@dataclass
class StereoPair:
    """同じ瞬間に撮った右（主＝`front`）と左の検出結果。"""

    right: Detection
    left: Detection


def pair_shots(right: list[Detection], left: list[Detection],
               max_dt_s: float = PAIR_MAX_DT_S) -> tuple[list[StereoPair], list[Path]]:
    """ファイル名の時刻が近い右と左を1対1で組にする。`(組, 相手が見つからない写真)`。

    両方でボードが見つかった組だけを返す（見つからなかった写真は相手なしに入れない）。
    """
    def stamped(ds):
        return [(t, d) for d in ds if (t := shot_time(d.path)) is not None]

    rs, ls = stamped(right), stamped(left)
    cand = sorted(((abs(tr - tl), i, j) for i, (tr, _) in enumerate(rs)
                   for j, (tl, _) in enumerate(ls) if abs(tr - tl) <= max_dt_s))
    used_r: set[int] = set()
    used_l: set[int] = set()
    pairs: list[StereoPair] = []
    for _, i, j in cand:                     # 近い順に貪欲に決める（1枚は1組にしか入らない）
        if i in used_r or j in used_l:
            continue
        used_r.add(i)
        used_l.add(j)
        pairs.append(StereoPair(rs[i][1], ls[j][1]))
    pairs.sort(key=lambda p: p.right.path.name)
    unpaired = [d.path for k, (_, d) in enumerate(rs) if k not in used_r] \
        + [d.path for k, (_, d) in enumerate(ls) if k not in used_l] \
        + [d.path for d in right + left if shot_time(d.path) is None]
    ok = [p for p in pairs if p.right.ok and p.left.ok]
    return ok, unpaired


@dataclass
class StereoResult:
    """`cv2.fisheye.stereoCalibrate`（内部パラメータは固定）の結果。約束は `raspi/core/stereo_model.py`。"""

    stereo: StereoCalib
    rms: float
    #: 使った組（右の写真で代表）→ その組の再投影誤差の RMS [px]（左右の角点をまとめて）
    per_pair: dict[Path, float]
    #: 平行化後の対応点の縦ずれ |v左 − v右| の中央値・95% 点・最大 [px]（`CHECK_HFOV_RAD` の画像の中の角点）
    rect_dy_median: float
    rect_dy_p95: float
    rect_dy_max: float
    rejected: dict[Path, str] = field(default_factory=dict)

    @property
    def baseline(self) -> float:
        return self.stereo.baseline

    @property
    def passed(self) -> bool:
        """`docs/stereo_plan.md` 段階2 の合否。"""
        return (self.rect_dy_median <= MAX_RECT_DY_PX
                and abs(self.baseline - NOMINAL_BASELINE_M) <= BASELINE_TOL_M)


def _K(c: FisheyeCalib) -> np.ndarray:
    return np.array([[c.fx, 0.0, c.cx], [0.0, c.fy, c.cy], [0.0, 0.0, 1.0]])


def aligned_left_corners(pair: StereoPair) -> np.ndarray:
    """右と同じ並び順にした左の角点 (N,1,2)。

    **検出器は角点を逆順（ボードを 180° 回した並び）で返すことがあり、左右で食い違う。**
    単眼の校正では姿勢が変わるだけで無害だが、ステレオでは角点の対応が壊れて
    `stereoCalibrate` が発散する（合成画像のテストで 16 組中 2 組で起きた）。
    2台は 75mm しか離れておらず同じ向きに見ているので、先頭 → 末尾の角点の向きを
    左右で比べ、逆を向いていれば左を反転する。
    """
    cr = pair.right.corners.reshape(-1, 2)
    cl = pair.left.corners.reshape(-1, 2)
    if float((cr[-1] - cr[0]) @ (cl[-1] - cl[0])) < 0.0:
        return pair.left.corners[::-1].copy()
    return pair.left.corners


def rect_dy(pairs: list[StereoPair], right: FisheyeCalib, left: FisheyeCalib,
            stereo: StereoCalib, hfov: float = CHECK_HFOV_RAD) -> np.ndarray:
    """組ごとの角点を平行化したときの縦ずれ |v左 − v右| [px]（全角点を1列に並べる）。

    **左右どちらかで平行化画像の外に出る角点は除く。** 外側（魚眼の周辺）はピンホールに
    直すと何倍にも引き伸ばされ、0.3px の検出誤差が数 px の縦ずれに化ける。照合に使わない所で
    合否を決めないため（合成画像で、真の R・T でも 95% 点が 0.8px になった）。
    """
    from raspi.core.stereo_model import StereoRectifier

    rect = StereoRectifier(right, left, stereo, right.width, right.height, hfov)
    out = []
    for p in pairs:
        cr, cl = p.right.corners.reshape(-1, 2), aligned_left_corners(p).reshape(-1, 2)
        ur, vr = rect.rectify_points("right", cr[:, 0], cr[:, 1], right.width, right.height)
        ul, vl = rect.rectify_points("left", cl[:, 0], cl[:, 1], left.width, left.height)
        inside = np.ones(len(cr), bool)
        for u, v in ((ur, vr), (ul, vl)):
            with np.errstate(invalid="ignore"):
                inside &= (u >= 0) & (u < rect.out_w) & (v >= 0) & (v < rect.out_h)
        out.append(np.abs(vl - vr)[inside])
    return np.concatenate(out) if out else np.zeros(0)


def stereo_calibrate(pairs: list[StereoPair], board: tuple[int, int], square: float,
                     right: FisheyeCalib, left: FisheyeCalib,
                     *, lens_right: str = "", lens_left: str = "",
                     outlier_px: float = 1.0, max_drop: int | None = None) -> StereoResult:
    """右と左の内部パラメータを固定して、左 → 右の R・T を求める。

    - 内部パラメータは先に `calibrate` で左右別々に求めておく（`docs/stereo_plan.md` の手順）。
      1台ずつの方が写真の枚数が多く（片方にしか写っていない写真も使える）、端の歪みまで決まる
    - 外れ値の外し方は `calibrate` と同じ（組の再投影誤差が `max(outlier_px, 中央値×3)` を超えたら
      1組ずつ外す。`max_drop` 組まで、既定は全体の 1/3）
    - 写真は下端クロップ済みでもよい（角点の座標はそのままフル画像の座標。モジュールの docstring）

    :raises ValueError: 組が `MIN_PAIRS` 未満、解像度が内部パラメータと合わない、収束しない
    """
    cv2 = _cv2()
    pairs = list(pairs)
    for p in pairs:
        for d, c, name in ((p.right, right, "右"), (p.left, left, "左")):
            if d.full_size != (c.width, c.height):
                raise ValueError(f"{name}の写真 {d.path.name} の大きさ（クロップ前 {d.full_size}）が"
                                 f"内部パラメータ（{c.width}x{c.height}）と合わない")
    obj = _object_points(board, square).reshape(1, -1, 3)
    K1, D1 = _K(left), np.array(left.k, np.float64).reshape(4, 1)
    K2, D2 = _K(right), np.array(right.k, np.float64).reshape(4, 1)
    flags = _fisheye_flag(cv2, "CALIB_FIX_INTRINSIC") | _fisheye_flag(cv2, "CALIB_FIX_SKEW")
    crit = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 200, 1e-10)
    rejected: dict[Path, str] = {}
    if max_drop is None:
        max_drop = len(pairs) // 3
    dropped = 0
    while True:
        if len(pairs) < MIN_PAIRS:
            raise ValueError(f"左右の両方にボードが写った組が {len(pairs)} 組しかない"
                             f"（{MIN_PAIRS} 組以上要る）")
        try:
            rms, *_, R, T, rvecs, tvecs = cv2.fisheye.stereoCalibrate(
                [obj] * len(pairs),
                [aligned_left_corners(p).reshape(1, -1, 2) for p in pairs],
                [p.right.corners.reshape(1, -1, 2) for p in pairs],
                K1.copy(), D1.copy(), K2.copy(), D2.copy(), (right.width, right.height),
                flags=flags, criteria=crit)
        except cv2.error as e:
            raise ValueError(f"ステレオ校正が収束しなかった: {e}") from e
        # 組ごとの再投影誤差（rvecs/tvecs は左カメラから見たボードの姿勢）
        errs = []
        for p, rv, tv in zip(pairs, rvecs, tvecs):
            Rl = cv2.Rodrigues(np.asarray(rv, np.float64))[0]
            rv_r = cv2.Rodrigues(R @ Rl)[0]
            tv_r = R @ np.asarray(tv, np.float64).reshape(3, 1) + T.reshape(3, 1)
            sq = []
            for corners, K, D, r_, t_ in ((aligned_left_corners(p), K1, D1, rv, tv),
                                          (p.right.corners, K2, D2, rv_r, tv_r)):
                proj, _ = cv2.fisheye.projectPoints(obj, np.asarray(r_, np.float64).reshape(3, 1),
                                                    np.asarray(t_, np.float64).reshape(3, 1), K, D)
                diff = proj.reshape(-1, 2) - corners.reshape(-1, 2)
                sq.append(np.sum(diff * diff, axis=1))
            errs.append(float(np.sqrt(np.mean(np.concatenate(sq)))))
        med = float(np.median(errs))
        worst = int(np.argmax(errs))
        if errs[worst] > max(outlier_px, 3.0 * med) and dropped < max_drop:
            bad = pairs.pop(worst)
            rejected[bad.right.path] = f"組の再投影誤差が大きい（{errs[worst]:.2f}px）"
            dropped += 1
            continue
        break

    stereo = StereoCalib(R=tuple(float(v) for v in np.asarray(R).ravel()),
                         T=tuple(float(v) for v in np.asarray(T).ravel()),  # type: ignore[arg-type]
                         lens_right=lens_right, lens_left=lens_left, rms=float(rms))
    dy = rect_dy(pairs, right, left, stereo)
    return StereoResult(stereo=stereo, rms=float(rms),
                        per_pair={p.right.path: e for p, e in zip(pairs, errs)},
                        rect_dy_median=float(np.median(dy)) if dy.size else math.inf,
                        rect_dy_p95=float(np.percentile(dy, 95)) if dy.size else math.inf,
                        rect_dy_max=float(dy.max()) if dy.size else math.inf,
                        rejected=rejected)


def write_stereo_toml(result: StereoResult, toml_path: str | Path = DEFAULT_TOML,
                      *, regenerate: bool = True) -> None:
    """外部パラメータを `[sensors.stereo]` に書く（既にあれば中身を丸ごと置き換える）。

    左右の内部パラメータは各レンズの `fisheye`（`write_vehicle_toml`）に**先に**書いておくこと。
    `lens_right`・`lens_left` が今のレンズと違うと `Vehicle` はこの値を使わない。
    """
    import tomlkit

    path = Path(toml_path)
    doc = tomlkit.parse(path.read_text(encoding="utf-8"))
    sensors = doc.get("sensors")
    if sensors is None:
        raise ValueError(f"{path} に [sensors] が無い")
    s = result.stereo
    tbl = tomlkit.table()
    tbl.add(tomlkit.comment(f"tools/cam_calib --stereo が書いた（{len(result.per_pair)} 組、再投影誤差 "
                            f"{result.rms:.3f}px、平行化後の縦ずれ 中央値 {result.rect_dy_median:.2f}px）。"
                            "手で書き換えない"))
    tbl.add(tomlkit.comment("X_右 = R·X_左 + T（各カメラ座標。約束は raspi/core/stereo_model.py）"))
    tbl["lens_right"] = s.lens_right
    tbl["lens_left"] = s.lens_left
    R = tomlkit.array()
    for row in np.array(s.R).reshape(3, 3):
        R.append([float(round(float(v), 9)) for v in row])
    tbl["R"] = R
    tbl["T"] = [float(round(v, 7)) for v in s.T]
    tbl["baseline_mm"] = float(round(s.baseline * 1000.0, 2))       # 参考値（読み手は T を使う）
    tbl["rms"] = float(round(result.rms, 4))
    tbl["rect_dy_median"] = float(round(result.rect_dy_median, 3))
    _replace_table(sensors, "stereo", tbl)
    path.write_text(tomlkit.dumps(doc), encoding="utf-8")
    if regenerate and path.resolve() == DEFAULT_TOML.resolve():
        subprocess.run([sys.executable, str(REPO_ROOT / "config" / "generate.py")],
                       check=True, cwd=REPO_ROOT)


@dataclass
class StereoCalibration:
    """ステレオ校正の一式（右・左の内部パラメータと R・T）。GUI と CLI が共通に使う。"""

    right: CalibResult
    left: CalibResult
    stereo: StereoResult
    pairs: list[StereoPair]
    #: 相手が見つからなかった写真
    unpaired: list[Path]


def calibrate_stereo_set(right: list[Detection], left: list[Detection],
                         board: tuple[int, int], square: float) -> StereoCalibration:
    """右（`front`）と左（`left`）の写真から、内部パラメータ2つ → R・T の順に求める。"""
    cr = calibrate(right, board, square)
    cl = calibrate(left, board, square)
    # 内部パラメータの校正で外れ値として外した写真は、組からも外す
    bad = set(cr.rejected) | set(cl.rejected)
    pairs, unpaired = pair_shots([d for d in right if d.path not in bad],
                                 [d for d in left if d.path not in bad])
    st = stereo_calibrate(pairs, board, square, cr.calib, cl.calib,
                          lens_right=cr.lens, lens_left=cl.lens)
    return StereoCalibration(cr, cl, st, pairs, unpaired)


def write_stereo_set(sc: StereoCalibration, toml_path: str | Path = DEFAULT_TOML) -> tuple[str, str]:
    """右・左の `fisheye` と `[sensors.stereo]` を書く。書いた先のレンズ名 `(右, 左)` を返す。"""
    lr = write_vehicle_toml("front", sc.right, toml_path, regenerate=False)
    ll = write_vehicle_toml("left", sc.left, toml_path, regenerate=False)
    if (lr, ll) != (sc.stereo.stereo.lens_right, sc.stereo.stereo.lens_left):
        raise ValueError(f"書き込み先のレンズ {lr}/{ll} が写真のレンズ "
                         f"{sc.stereo.stereo.lens_right}/{sc.stereo.stereo.lens_left} と違う")
    write_stereo_toml(sc.stereo, toml_path)
    return lr, ll


def rectified_preview(pair: StereoPair, right: FisheyeCalib, left: FisheyeCalib,
                      stereo: StereoCalib, hfov: float = CHECK_HFOV_RAD,
                      line_step: int = 24) -> np.ndarray:
    """平行化した左・右を並べ、横線を引いた BGR 画像（左が左）。同じ物が同じ線に乗っていれば OK。"""
    from raspi.core.stereo_model import StereoRectifier

    cv2 = _cv2()
    ir = cv2.imread(str(pair.right.path), cv2.IMREAD_COLOR)
    il = cv2.imread(str(pair.left.path), cv2.IMREAD_COLOR)
    if ir is None or il is None:
        raise ValueError("画像を読めない")
    rect = StereoRectifier(right, left, stereo, right.width, right.height, hfov,
                           crop_right=pair.right.crop, crop_left=pair.left.crop)
    rr, rl = rect(ir, il)
    img = np.hstack([rl, rr])
    for y in range(line_step // 2, img.shape[0], line_step):
        cv2.line(img, (0, y), (img.shape[1] - 1, y), (0, 200, 0), 1)
    return img
