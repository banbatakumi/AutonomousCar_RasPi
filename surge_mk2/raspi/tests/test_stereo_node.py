"""`raspi/nodes/stereo_node.py` のテスト。

左右の魚眼画像に、模様のある**床**・奥の**壁**・手前の**低い箱**（高さ 10cm。LiDAR の走査面 9.5cm より低い）を描き、
`scan/stereo` が箱の方位にだけ箱の距離を出し、床を障害物にしないかを見る。
ノードの組の扱い（同期待ち・IDLE・同じ瞬間を2度処理しない）は `StereoNode.step` を直接叩く。
"""

from __future__ import annotations

import dataclasses
from types import SimpleNamespace

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from raspi.core.camera_model import FisheyeCalib, unproject_pixels  # noqa: E402
from raspi.core.stereo_model import StereoCalib, camera_to_base_rotation  # noqa: E402
from raspi.core.vehicle import Vehicle  # noqa: E402
from raspi.msgs.types import (TOPIC_CAM_CONFIG, TOPIC_IMAGE_FRONT,  # noqa: E402
                              TOPIC_IMAGE_LEFT, TOPIC_STEREO_DEPTH)
from raspi.nodes.stereo_node import DEFAULT_ROWS, StereoNode, StereoProcessor  # noqa: E402

RIGHT = FisheyeCalib(fx=309.5, fy=309.8, cx=305.1, cy=241.5,
                     k=(-0.024, -0.036, 0.041, -0.0196), width=640, height=480)
LEFT = FisheyeCalib(fx=312.2, fy=312.6, cx=305.8, cy=243.8,
                    k=(-0.042, 0.0178, -0.0257, 0.0094), width=640, height=480)
R_LR = cv2.Rodrigues(np.radians([-0.12, -0.75, 1.5]))[0]
T_LR = np.array([-0.0760, 0.0009, -0.0056])
CAM = (0.09, -0.0375, 0.08, 0.0, 0.0)

BOX_X, BOX_H, BOX_HALF_W = 0.80, 0.10, 0.06       # 箱の前面（base_link の x）・高さ・半幅（y = 0 中心）
WALL_X = 2.6


def _vehicle() -> Vehicle:
    v = Vehicle.load()
    return dataclasses.replace(
        v, cam_front_fisheye=RIGHT, cam_left_fisheye=LEFT,
        stereo=StereoCalib(R=tuple(R_LR.ravel()), T=tuple(T_LR), lens_right="r", lens_left="l"),
        cam_front_x=CAM[0], cam_front_y=CAM[1], cam_front_z=CAM[2], cam_front_pitch=0.0,
        cam_front_yaw=0.0, cam_front_bottom_crop=0.0, cam_left_bottom_crop=0.0)


def _tex(seed, n=1024):
    rng = np.random.default_rng(seed)
    t = cv2.GaussianBlur(rng.uniform(0, 255, (n, n)).astype(np.float32), (0, 0), 1.2)
    return cv2.normalize(t, None, 20, 235, cv2.NORM_MINMAX)


TEX = [_tex(1), _tex(2), _tex(3)]
TEX_M = 0.003                                      # 1px = 3mm


def _sample(tex, a, b):
    mx = (a / TEX_M + tex.shape[1] / 2).astype(np.float32)
    my = (b / TEX_M + tex.shape[0] / 2).astype(np.float32)
    return cv2.remap(tex, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)


def _render(calib, R_cb, o):
    vs, us = np.mgrid[0:480, 0:640].astype(np.float64)
    x, y, z = unproject_pixels(us, vs, calib.fx, calib.fy, calib.cx, calib.cy, calib.k)
    d = np.stack([x, y, z], -1) @ R_cb.T
    big = np.full(d.shape[:2], np.inf)
    img = np.full(d.shape[:2], 60.0, np.float32)
    with np.errstate(divide="ignore", invalid="ignore"):
        s_floor = np.where(d[..., 2] < -1e-6, -o[2] / d[..., 2], np.inf)
        s_wall = np.where(d[..., 0] > 1e-6, (WALL_X - o[0]) / d[..., 0], np.inf)
        s_box = np.where(d[..., 0] > 1e-6, (BOX_X - o[0]) / d[..., 0], np.inf)
    by, bz = o[1] + s_box * d[..., 1], o[2] + s_box * d[..., 2]
    s_box = np.where((np.abs(by) <= BOX_HALF_W) & (bz >= 0) & (bz <= BOX_H), s_box, np.inf)
    s = np.minimum(np.minimum(s_floor, s_wall), np.minimum(s_box, big))
    P = o + s[..., None] * d
    for sk, tex, (a, b) in ((s_floor, TEX[0], (P[..., 0], P[..., 1])),
                            (s_wall, TEX[1], (P[..., 1], -P[..., 2])),
                            (s_box, TEX[2], (P[..., 1], -P[..., 2]))):
        m = np.isfinite(s) & (s == sk)
        if m.any():
            img[m] = _sample(tex, np.where(m, a, 0), np.where(m, b, 0))[m]
    return cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_GRAY2BGR)


@pytest.fixture(scope="module")
def images():
    R_rb = camera_to_base_rotation(0.0, 0.0)
    o_r = np.array(CAM[:3])
    right = _render(RIGHT, R_rb, o_r)
    left = _render(LEFT, R_rb @ R_LR, o_r + R_rb @ T_LR)
    return right, left


@pytest.fixture(scope="module")
def proc():
    return StereoProcessor(_vehicle())


def test_low_box_is_found_and_floor_is_not(proc, images):
    st, d, guide = proc.process(*images, "BGR888", t_capture_ns=5, seq=3)
    dist = np.array(st.dist)
    # 箱の方位（base_link 原点から ±4°）には箱の前面までの距離
    box_degs = [i % 360 for i in range(-3, 4)]
    found = dist[box_degs]
    assert (found > 0).sum() >= 5, found
    assert np.median(found[found > 0]) == pytest.approx(BOX_X, abs=0.05)
    # 箱の外（床と、2m より遠い壁しか無い）では何も見つからない
    others = [i % 360 for i in list(range(-45, -8)) + list(range(9, 46))]
    assert (dist[others] > 0).sum() <= 2, dist[others]
    assert st.sector_seen[0] and st.sector_seen[11]          # 正面の両側のセクタは視野の中
    assert not st.sector_seen[5]                              # 真後ろは見えない
    assert (st.seq, st.t_capture) == (3, 5)
    assert d.shape == (DEFAULT_ROWS[1] - DEFAULT_ROWS[0], 640)
    assert guide.shape == d.shape
    assert proc.depth_jpeg(d)[:2] == b"\xff\xd8"
    assert proc.depth_jpeg(d, guide)[:2] == b"\xff\xd8"           # 表示用の穴埋めあり


def _ref(cam, t, synced=True):
    return SimpleNamespace(cam=cam, t_capture=t, synced=synced, fmt="BGR888", shm_name=cam, ring_seq=1)


class _Pub:
    def __init__(self):
        self.sent = []

    def send(self, topic, msg):
        self.sent.append((topic, msg))


def test_step_gating(proc, images):
    node = StereoNode(proc, rate_hz=10)
    calls = []

    def read_at(img):
        def f(ref, t, tol):
            calls.append((ref.cam, t))
            return img, t
        return f
    node._readers["front"].read_at = read_at(images[0])
    node._readers["left"].read_at = read_at(images[1])
    pub = _Pub()
    latest = {TOPIC_IMAGE_FRONT: _ref("front", 1000), TOPIC_IMAGE_LEFT: _ref("left", 1100)}
    # 深度を使う者がいなければ照合しない
    st = node.step(latest, 10**9)
    assert not any(st.sector_seen) and not calls
    latest[TOPIC_CAM_CONFIG] = SimpleNamespace(depth=True, depth_view=True)
    # 同期していなければ照合しない
    latest[TOPIC_IMAGE_LEFT] = _ref("left", 1100, synced=False)
    assert "同期" in node.step(latest, 10**9).reason and not calls
    latest[TOPIC_IMAGE_LEFT] = _ref("left", 1100)
    st = node.step(latest, 10**9)
    assert any(st.sector_seen)
    assert calls == [("front", 1000), ("left", 1000)]         # 古い方の時刻に合わせて両方を読む
    assert not pub.sent                                       # 深度映像は scan/stereo を出した後で焼く
    node.publish_depth(pub)
    assert [t for t, _ in pub.sent] == [TOPIC_STEREO_DEPTH]
    assert node.step(latest, 2 * 10**9) is None               # 同じ瞬間は2度処理しない
    ms = 1_000_000
    # 頻度の上限（10Hz）は撮影時刻の間隔で判定する。1コマ後（33ms）の組は回さない
    latest[TOPIC_IMAGE_FRONT] = _ref("front", 1000 + 33 * ms)
    latest[TOPIC_IMAGE_LEFT] = _ref("left", 1000 + 33 * ms)
    assert node.step(latest, 3 * 10**9) is None
    # 片方だけ新しいコマが届いた瞬間は、もう片方を待つ（1コマ古い組を処理しない）
    latest[TOPIC_IMAGE_FRONT] = _ref("front", 1000 + 100 * ms)
    latest[TOPIC_IMAGE_LEFT] = _ref("left", 1000 + 67 * ms)
    calls.clear()
    assert node.step(latest, 10**9 + 120 * ms) is None and not calls
    latest[TOPIC_IMAGE_LEFT] = _ref("left", 1000 + 100 * ms)
    # 深度映像を見ている人がいなければ焼かない（走行中の時間と電力を食わない）
    latest[TOPIC_CAM_CONFIG] = SimpleNamespace(depth=True, depth_view=False)
    assert node.step(latest, 10**9 + 121 * ms) is not None
    assert calls == [("front", 1000 + 100 * ms), ("left", 1000 + 100 * ms)]
    node.publish_depth(pub)
    assert len(pub.sent) == 1


def test_without_calibration():
    v = dataclasses.replace(Vehicle.load(), stereo=None)
    with pytest.raises(ValueError):
        StereoProcessor(v)
    node = StereoNode(None, init_error="校正値が無い")
    st = node.step({TOPIC_CAM_CONFIG: SimpleNamespace(depth=True)}, 0)
    assert st.reason == "校正値が無い" and not any(st.sector_seen)


def test_obstacle_behind_floor_mismatch_is_kept():
    """同じ方位で、手前に床の取り違え（高さのそろった点）、奥に本物の障害物（縦に広がった点）があるとき、奥を採る。"""
    from raspi.core.stereo_match import obstacle_scan

    rng = np.random.default_rng(0)
    floor = np.stack([np.full(30, 0.5), np.zeros(30), 0.04 + rng.uniform(-0.003, 0.003, 30)], 1)
    wall = np.stack([np.full(30, 1.2), np.zeros(30), np.linspace(0.04, 0.15, 30)], 1)
    res = obstacle_scan(np.concatenate([floor, wall]), [0])
    assert res.dist[0] == pytest.approx(1.2, abs=0.01)
    assert obstacle_scan(floor, [0]).dist[0] == 0.0


def test_display_filler_fills_holes_but_keeps_measured_values():
    """表示用の穴埋め: 測った画素は変えず、模様の無い所を周りの値で埋める。手がかりの無い所は黒のまま。"""
    from raspi.core.stereo_match import DisplayFiller

    rng = np.random.default_rng(0)
    guide = np.full((100, 200), 120, np.uint8)
    # 実際の画像に近い、なめらかな模様（画素ごとの乱数だと全画素が「縁」になり、何も広がらない）
    tex = cv2.GaussianBlur(rng.uniform(0, 255, (100, 100)).astype(np.float32), (0, 0), 3.0)
    guide[:, :100] = cv2.normalize(tex, None, 60, 200, cv2.NORM_MINMAX).astype(np.uint8)
    d = np.full((100, 200), -1.0, np.float32)
    d[:, 20:100] = 30.0                        # 左半分（模様あり）だけ測れた
    d[40:60, 40:60] = -1.0                     # 模様の中の小さな穴
    out = DisplayFiller()(d, guide)
    assert np.all(out[d > 0] == d[d > 0])
    assert np.all(np.abs(out[40:60, 40:60] - 30.0) < 1.0)    # 小さな穴は埋まる
    assert np.mean(out > 0) > np.mean(d > 0)


def test_depth_worker_renders_in_background(proc, images):
    """深度映像は別スレッドで作り、できたものをメインのループが取り出す。"""
    import time as _t

    from raspi.nodes.stereo_node import DepthWorker

    node = StereoNode(proc)
    st, d, guide = proc.process(*images)
    w = DepthWorker(node)
    try:
        w.submit((d, guide, 123))
        for _ in range(200):
            msg = w.take_done()
            if msg is not None:
                break
            _t.sleep(0.01)
        assert msg is not None and msg.jpeg[:2] == b"\xff\xd8" and msg.t_capture == 123
        assert node.view_ms > 0
    finally:
        w.stop()
