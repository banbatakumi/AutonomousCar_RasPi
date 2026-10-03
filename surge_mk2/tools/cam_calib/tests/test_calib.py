"""カメラ校正（`tools/cam_calib/calib.py`）のテスト。

**実機もカメラも要らない。** 既知の魚眼レンズ（`raspi/core/camera_model.py` の式）で
チェッカーボードを描いた合成写真を作り、校正して元の値に戻るかを見る
（`raspi/tests/test_ipm.py` の「合成して答え合わせする」流儀）。写真は 📷 と同じく
下端を切り落とし、同じ名前の付け方にする。
"""

from __future__ import annotations

import math
import shutil
import tempfile
from pathlib import Path

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from raspi.core.camera_model import FisheyeCalib, project_rays, unproject_pixels  # noqa: E402
from tools.cam_calib.board import make_board  # noqa: E402
from tools.cam_calib.calib import (calibrate, coverage, detect, parse_name,  # noqa: E402
                                   write_vehicle_toml)

BOARD = (8, 5)              # 内側の角点（偶数x奇数で向きが一意）
SQUARE = 0.03               # [m]
W, H, CROP = 640, 480, 0.25
TRUE = FisheyeCalib(fx=200.0, fy=200.0, cx=318.0, cy=243.0,
                    k=(0.03, -0.012, 0.004, -0.0008), width=W, height=H)


def _rot(rx: float, ry: float, rz: float) -> np.ndarray:
    return cv2.Rodrigues(np.array([rx, ry, rz], dtype=np.float64))[0]


def _render(R: np.ndarray, t: np.ndarray, ss: int = 2) -> np.ndarray:
    """ボード（ボード座標の z=0 平面、原点＝左上の内側角点）をカメラ姿勢 R,t で写した画像。"""
    hh, ww = H * ss, W * ss
    vs, us = np.mgrid[0:hh, 0:ww].astype(np.float64)
    us = (us + 0.5) / ss - 0.5
    vs = (vs + 0.5) / ss - 0.5
    x, y, z = unproject_pixels(us, vs, TRUE.fx, TRUE.fy, TRUE.cx, TRUE.cy, TRUE.k)
    ray = np.stack([x, y, z], axis=-1)
    Rt = R.T
    rb = ray @ Rt.T                          # ボード座標での光線の向き
    ob = -(Rt @ t)                           # ボード座標でのカメラ位置
    s = -ob[2] / np.where(np.abs(rb[..., 2]) < 1e-9, 1e-9, rb[..., 2])
    px = ob[0] + s * rb[..., 0]
    py = ob[1] + s * rb[..., 1]
    ix = np.floor(px / SQUARE).astype(int)
    iy = np.floor(py / SQUARE).astype(int)
    cols, rows = BOARD
    on_board = (ix >= -1) & (ix <= cols - 1) & (iy >= -1) & (iy <= rows - 1)
    margin = (px > -2 * SQUARE) & (px < (cols + 1) * SQUARE) \
        & (py > -2 * SQUARE) & (py < (rows + 1) * SQUARE)
    img = np.full(px.shape, 90.0)
    img[margin] = 235.0
    black = on_board & ((ix + iy) % 2 == 0)
    img[black] = 20.0
    img[s <= 0] = 90.0
    img = img.reshape(H, ss, W, ss).mean(axis=(1, 3))
    return np.clip(img, 0, 255).astype(np.uint8)


def _corners_px(R, t) -> np.ndarray:
    cols, rows = BOARD
    obj = np.zeros((cols * rows, 3))
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * SQUARE
    pc = obj @ R.T + t
    u, v = project_rays(pc[:, 0], pc[:, 1], pc[:, 2], TRUE.fx, TRUE.fy, TRUE.cx, TRUE.cy, TRUE.k)
    return np.stack([u, v], axis=1)


def _poses(n: int, seed: int = 1) -> list[tuple[np.ndarray, np.ndarray]]:
    """クロップ後の画面に収まり、画面全体（端も）に散らばる姿勢を n 個。"""
    rng = np.random.default_rng(seed)
    keep_h = H * (1 - CROP)
    out = []
    while len(out) < n:
        R = _rot(rng.uniform(-0.6, 0.6), rng.uniform(-0.6, 0.6), rng.uniform(-0.4, 0.4))
        # 視線方向を画角いっぱいに散らす
        ang = rng.uniform(0, math.radians(55))
        az = rng.uniform(-math.pi, math.pi)
        dist = rng.uniform(0.18, 0.32)
        center = dist * np.array([math.sin(ang) * math.cos(az), math.sin(ang) * math.sin(az),
                                  math.cos(ang)])
        cols, rows = BOARD
        t = center - R @ np.array([(cols - 1) * SQUARE / 2, (rows - 1) * SQUARE / 2, 0.0])
        pc = (np.zeros(3) @ R.T) + t
        if pc[2] <= 0.05:
            continue
        uv = _corners_px(R, t)
        if (uv[:, 0].min() < 12 or uv[:, 0].max() > W - 12 or uv[:, 1].min() < 12
                or uv[:, 1].max() > keep_h - 12):
            continue
        # 表から見ていて、斜めすぎないもの（裏から見ると模様が鏡像になり、実物では
        # 起こらない対応になる。ボード法線 +z はカメラから遠ざかる向き）
        normal = R[:, 2]
        if normal @ (center / np.linalg.norm(center)) < 0.45:
            continue
        out.append((R, t))
    return out


@pytest.fixture(scope="module")
def shots():
    d = Path(tempfile.mkdtemp())
    paths = []
    for i, (R, t) in enumerate(_poses(14)):
        img = _render(R, t)[: int(round(H * (1 - CROP)))]
        p = d / f"surge_front_{W}x{img.shape[0]}_crop{CROP:g}_20261003_0000{i:02d}.png"
        cv2.imwrite(str(p), img)
        paths.append(p)
    yield paths
    shutil.rmtree(d)


def test_parse_name():
    assert parse_name("surge_rear_640x450_crop0.0625_20261003_101010123.png") == ("rear", 0.0625)
    assert parse_name("IMG_0001.png") is None


def test_calibration_recovers_lens(shots):
    dets = [detect(p, BOARD) for p in shots]
    assert sum(d.ok for d in dets) >= 10
    assert all(d.full_size == (W, H) for d in dets if d.ok)
    res = calibrate(dets, BOARD, SQUARE)
    c = res.calib
    assert res.rms < 0.5
    assert (c.width, c.height) == (W, H)
    assert c.fx == pytest.approx(TRUE.fx, rel=0.03)
    assert c.fy == pytest.approx(TRUE.fy, rel=0.03)
    assert c.cx == pytest.approx(TRUE.cx, abs=4)
    assert c.cy == pytest.approx(TRUE.cy, abs=4)
    # 係数そのものは相関が強く個別には一致しにくいので、像の位置で比べる
    th = np.linspace(0.05, math.radians(70), 20)
    x, z = np.sin(th), np.cos(th)
    u_true, _ = project_rays(x, 0 * x, z, TRUE.fx, TRUE.fy, TRUE.cx, TRUE.cy, TRUE.k)
    u_est, _ = project_rays(x, 0 * x, z, c.fx, c.fy, c.cx, c.cy, c.k)
    assert np.max(np.abs(u_true - u_est)) < 3.0
    assert 0.0 < coverage(dets) <= 1.0


def test_too_few_images_is_an_error(shots):
    dets = [detect(p, BOARD) for p in shots[:3]]
    with pytest.raises(ValueError):
        calibrate(dets, BOARD, SQUARE)


def test_write_vehicle_toml_keeps_comments(shots, tmp_path):
    from tools.cam_calib.calib import DEFAULT_TOML
    from raspi.core.vehicle import Vehicle

    dets = [detect(p, BOARD) for p in shots]
    res = calibrate(dets, BOARD, SQUARE)
    toml = tmp_path / "vehicle.toml"
    toml.write_text(DEFAULT_TOML.read_text(encoding="utf-8"), encoding="utf-8")
    write_vehicle_toml("front", res, toml, regenerate=False)
    text = toml.read_text(encoding="utf-8")
    assert "下端カット率（下1/4）" in text            # 既存のコメントが残る
    v = Vehicle.load(toml)
    assert v.cam_front_fisheye is not None
    assert v.cam_front_fisheye.fx == pytest.approx(res.calib.fx, abs=1e-3)
    assert v.cam_rear_fisheye is None
    assert v.cam_front_hfov == pytest.approx(1.152)   # 未校正時の近似は触らない
    # 2回書いても表が増えない（上書きされる）
    write_vehicle_toml("front", res, toml, regenerate=False)
    lines = toml.read_text(encoding="utf-8").splitlines()
    assert sum(ln.strip() == "[sensors.cam_front.fisheye]" for ln in lines) == 1


def test_board_image():
    img = make_board((9, 6), 0.025, dpi=100)
    # 10x7 マス + 余白1マスずつ
    px = round(0.025 / 0.0254 * 100)
    assert img.shape == (9 * px, 12 * px)
    ok, _ = cv2.findChessboardCornersSB(img, (9, 6))
    assert ok
