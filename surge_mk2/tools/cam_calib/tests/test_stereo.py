"""ステレオ校正（`tools/cam_calib/calib.py` の `pair_shots`・`stereo_calibrate`）のテスト。

`test_calib.py` と同じく、既知の左右のレンズと既知の R・T でチェッカーボードを描いた合成写真を
📷 と同じ名前（右 = `front`、左 = `left`、時刻を数百 ms ずらす）で作り、
内部パラメータ → R・T の順に校正して元に戻るか、合否（縦ずれ・基線）が出るかを見る。
"""

from __future__ import annotations

import math
import shutil
import tempfile
from pathlib import Path

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from raspi.core.camera_model import FisheyeCalib  # noqa: E402
from raspi.core.stereo_model import StereoCalib  # noqa: E402
from tools.cam_calib.calib import (calibrate, detect, pair_shots, shot_time,  # noqa: E402
                                   stereo_calibrate, write_stereo_toml, write_vehicle_toml)
from tools.cam_calib.tests.test_calib import BOARD, SQUARE, _corners_px, _render, _rot  # noqa: E402

W, H = 640, 480
RIGHT = FisheyeCalib(fx=200.0, fy=200.5, cx=318.0, cy=243.0,
                     k=(0.03, -0.012, 0.004, -0.0008), width=W, height=H)
LEFT = FisheyeCalib(fx=203.0, fy=202.0, cx=324.0, cy=236.0,
                    k=(0.025, -0.010, 0.003, -0.0006), width=W, height=H)
#: 左 → 右（X_右 = R·X_左 + T）。取付の誤差として 0.6°・0.3°・0.4° 回し、基線 75.4mm
R_TRUE = _rot(math.radians(0.6), math.radians(-0.3), math.radians(0.4))
T_TRUE = np.array([-0.0754, 0.0008, -0.0006])


def _in_frame(uv: np.ndarray, margin: float = 12.0) -> bool:
    return bool(uv[:, 0].min() > margin and uv[:, 0].max() < W - margin
                and uv[:, 1].min() > margin and uv[:, 1].max() < H - margin)


def _stereo_poses(n: int, seed: int = 3):
    """右カメラ座標でのボードの姿勢。左右どちらの画面にも収まるもの。"""
    rng = np.random.default_rng(seed)
    cols, rows = BOARD
    out = []
    while len(out) < n:
        R = _rot(rng.uniform(-0.6, 0.6), rng.uniform(-0.6, 0.6), rng.uniform(-0.4, 0.4))
        ang = rng.uniform(0, math.radians(50))
        az = rng.uniform(-math.pi, math.pi)
        dist = rng.uniform(0.2, 0.34)
        center = dist * np.array([math.sin(ang) * math.cos(az), math.sin(ang) * math.sin(az),
                                  math.cos(ang)])
        t = center - R @ np.array([(cols - 1) * SQUARE / 2, (rows - 1) * SQUARE / 2, 0.0])
        if R[:, 2] @ (center / np.linalg.norm(center)) < 0.45:
            continue
        Rl, tl = R_TRUE.T @ R, R_TRUE.T @ (t - T_TRUE)
        if not (_in_frame(_corners_px(R, t, RIGHT)) and _in_frame(_corners_px(Rl, tl, LEFT))):
            continue
        out.append((R, t, Rl, tl))
    return out


@pytest.fixture(scope="module")
def stereo_shots():
    d = Path(tempfile.mkdtemp())
    rights, lefts = [], []
    for i, (R, t, Rl, tl) in enumerate(_stereo_poses(16)):
        # 📷 は右 → 左の順に落とすので、左の時刻は 0.2〜0.6 秒あと。組の間は 10 秒あける
        sec = 10 * i
        ms_l = 200 + 25 * i
        pr = d / f"surge_front_wide160sr_{W}x{H}_crop0_20261112_10{sec // 60:02d}{sec % 60:02d}000.png"
        pl = d / (f"surge_left_wide160sl_{W}x{H}_crop0_20261112_10{sec // 60:02d}{sec % 60:02d}"
                  f"{ms_l:03d}.png")
        cv2.imwrite(str(pr), _render(R, t, calib=RIGHT))
        cv2.imwrite(str(pl), _render(Rl, tl, calib=LEFT))
        rights.append(pr)
        lefts.append(pl)
    yield rights, lefts
    shutil.rmtree(d)


@pytest.fixture(scope="module")
def calibrated(stereo_shots):
    rights, lefts = stereo_shots
    dr = [detect(p, BOARD) for p in rights]
    dl = [detect(p, BOARD) for p in lefts]
    cr = calibrate(dr, BOARD, SQUARE)
    cl = calibrate(dl, BOARD, SQUARE)
    pairs, unpaired = pair_shots(dr, dl)
    res = stereo_calibrate(pairs, BOARD, SQUARE, cr.calib, cl.calib,
                           lens_right=cr.lens, lens_left=cl.lens)
    return cr, cl, pairs, unpaired, res


def test_shot_time():
    a = shot_time("surge_front_wide160sr_640x480_crop0_20261112_101010123.png")
    b = shot_time("surge_left_wide160sl_640x480_crop0_20261112_101010456 (1).png")
    assert b - a == pytest.approx(0.333, abs=1e-6)
    assert shot_time("IMG_0001.png") is None


def test_pairs_by_time(calibrated, stereo_shots):
    _, _, pairs, unpaired, _ = calibrated
    assert len(pairs) >= len(stereo_shots[0]) - 2      # 両方で見つかった組だけ（見つからない写真もある）
    assert not unpaired
    for p in pairs:
        assert abs(shot_time(p.right.path) - shot_time(p.left.path)) < 1.0


def test_pair_shots_does_not_cross_pair(stereo_shots):
    """左が1枚欠けても、別の瞬間の右と組にしない。"""
    rights, lefts = stereo_shots
    dr = [detect(p, BOARD) for p in rights[:3]]
    dl = [detect(p, BOARD) for p in lefts[:3]]
    pairs, unpaired = pair_shots(dr, dl[1:])
    assert {p.right.path for p in pairs} == {rights[1], rights[2]}
    assert unpaired == [rights[0]]


def test_stereo_recovers_extrinsics(calibrated):
    _, _, _, _, res = calibrated
    s = res.stereo
    assert res.rms < 0.5
    assert s.baseline == pytest.approx(np.linalg.norm(T_TRUE), abs=1e-3)
    assert np.allclose(s.T_vec, T_TRUE, atol=1.5e-3)
    dR = s.R_mat @ R_TRUE.T
    angle = math.degrees(math.acos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))
    assert angle < 0.15
    assert (s.lens_right, s.lens_left) == ("wide160sr", "wide160sl")
    # 合否: 平行化後の縦ずれと基線
    assert res.rect_dy_median < 0.3
    assert res.passed


def test_rect_dy_detects_misalignment(calibrated):
    """検証器の確認: R にピッチ 0.2° の誤差を入れると縦ずれの合否で落ちる。"""
    from tools.cam_calib.calib import MAX_RECT_DY_PX, rect_dy

    cr, cl, pairs, _, res = calibrated
    s = res.stereo
    bad = StereoCalib(R=tuple((_rot(math.radians(0.2), 0, 0) @ s.R_mat).ravel()), T=s.T)
    assert np.median(rect_dy(pairs, cr.calib, cl.calib, s)) < MAX_RECT_DY_PX
    assert np.median(rect_dy(pairs, cr.calib, cl.calib, bad)) > MAX_RECT_DY_PX


def test_write_stereo_into_vehicle_toml(calibrated, tmp_path):
    from raspi.core.vehicle import Vehicle
    from tools.cam_calib.calib import DEFAULT_TOML

    cr, cl, _, _, res = calibrated
    toml = tmp_path / "vehicle.toml"
    toml.write_text(DEFAULT_TOML.read_text(encoding="utf-8"), encoding="utf-8")
    assert write_vehicle_toml("front", cr, toml, regenerate=False) == "wide160sr"
    assert write_vehicle_toml("left", cl, toml, regenerate=False) == "wide160sl"
    write_stereo_toml(res, toml, regenerate=False)
    write_stereo_toml(res, toml, regenerate=False)          # 2回書いても表は1つ
    text = toml.read_text(encoding="utf-8")
    assert sum(ln.strip() == "[sensors.stereo]" for ln in text.splitlines()) == 1
    v = Vehicle.load(toml)
    assert v.cam_front_fisheye.fx == pytest.approx(cr.calib.fx, abs=1e-3)
    assert v.cam_left_fisheye.fx == pytest.approx(cl.calib.fx, abs=1e-3)
    assert v.stereo is not None
    assert v.stereo.baseline == pytest.approx(res.baseline, abs=1e-6)
    assert np.allclose(v.stereo.R_mat, res.stereo.R_mat, atol=1e-8)


def test_stereo_ignored_when_lens_changed(calibrated, tmp_path):
    """校正したときと別のレンズが選ばれていれば、R・T を使わない（黙って古い値で照合しない）。"""
    import tomlkit

    from raspi.core.vehicle import Vehicle
    from tools.cam_calib.calib import DEFAULT_TOML

    *_, res = calibrated
    toml = tmp_path / "vehicle.toml"
    toml.write_text(DEFAULT_TOML.read_text(encoding="utf-8"), encoding="utf-8")
    write_stereo_toml(res, toml, regenerate=False)
    doc = tomlkit.parse(toml.read_text(encoding="utf-8"))
    doc["sensors"]["stereo"]["lens_left"] = "other"
    toml.write_text(tomlkit.dumps(doc), encoding="utf-8")
    assert Vehicle.load(toml).stereo is None


def test_too_few_pairs(calibrated):
    cr, cl, pairs, _, _ = calibrated
    with pytest.raises(ValueError, match="組"):
        stereo_calibrate(pairs[:3], BOARD, SQUARE, cr.calib, cl.calib)


def test_stereo_calib_from_dict_rejects_bad_values():
    good = {"R": np.eye(3).tolist(), "T": [-0.075, 0.0, 0.0]}
    assert isinstance(StereoCalib.from_dict(good), StereoCalib)
    assert StereoCalib.from_dict({}) is None
    assert StereoCalib.from_dict({"R": (2 * np.eye(3)).tolist(), "T": [-0.075, 0, 0]}) is None
    assert StereoCalib.from_dict({"R": np.eye(3).tolist(), "T": [0, 0, 0]}) is None
