"""走行中の採点（`tools/stereo_bench/drive.py`）。合成の LiDAR と自車の動きで、補正と判定を確かめる。

    .venv/bin/python -m pytest tools/stereo_bench/tests/test_drive.py
"""

from __future__ import annotations

import math

import numpy as np

from tools.stereo_bench.drive import (DriveLog, DriveParams, LidarScan, PoseTrack, drive_eval,
                                      fit_yaw, lidar_ranges_at, score_frame)

LIDAR = (0.07, 0.0, 0.095, 0.0)
MS = 1_000_000


def _sector_times(t0: int):
    """1周 100ms。`sector_t_ns[s]` を s の順に 100/12ms ずつ進める（点ごとの時刻は `LidarScan.point_times` の式）。"""
    st = np.array([t0 + int(s * 100 * MS / 12) for s in range(12)], np.int64)
    dur = np.full(12, int(100_000 / 12), np.int64)
    return st, dur


def test_point_times_follow_scan_docstring():
    st, dur = _sector_times(1_000 * MS)
    sc = LidarScan(int(st[0]), np.ones(360), st, dur)
    t = sc.point_times()
    # 車両角 0 → センサ角 0 → セクタ 11・添字 0
    assert t[0] == st[11]
    # 車両角 359 → センサ角 1 → セクタ 11・添字 1
    assert math.isclose(t[359], st[11] + dur[11] * 1000 / 29)
    # 車両角 1 → センサ角 359 → セクタ 0・添字 29
    assert math.isclose(t[1], st[0] + dur[0] * 1000)


def _straight_log(v=2.0, wall_x=3.0, n_scans=8, side_y=None):
    """x 方向に v [m/s] で直進。世界の x = wall_x に正面の壁（y 方向に無限）。`side_y` なら y = ±side_y に横の壁
    （廊下）。LiDAR は点ごとの時刻で撮る。"""
    t = np.arange(0, 1_200) * MS                                  # 1.2 秒、1ms 刻み
    poses = PoseTrack(t.astype(np.float64), v * t / 1e9, np.zeros_like(t, float), np.zeros_like(t, float))
    scans = []
    for k in range(n_scans):
        st, dur = _sector_times(int(100 * MS + k * 100 * MS))
        sc = LidarScan(int(st[0]), np.zeros(360), st, dur)
        tp = sc.point_times()
        x_car = v * tp / 1e9
        deg = np.arange(360)
        a = np.radians(deg)
        with np.errstate(divide="ignore", invalid="ignore"):
            r = (wall_x - x_car - LIDAR[0]) / np.cos(a)
        ok = (np.cos(a) > 0.5) & (r > 0)
        if side_y is not None:
            with np.errstate(divide="ignore", invalid="ignore"):
                rs = np.abs((side_y - np.sign(np.sin(a)) * LIDAR[1]) / np.sin(a))
            side = np.isfinite(rs) & (np.abs(np.sin(a)) > 1e-6)
            r = np.where(ok, np.minimum(np.where(ok, r, np.inf), np.where(side, rs, np.inf)),
                         np.where(side, rs, 0.0))
            ok = np.isfinite(r) & (r > 0)
        sc.dist[ok] = r[ok]
        scans.append(sc)
    scans.sort(key=lambda s: s.t_mid)
    return DriveLog([], scans, poses, t.astype(np.float64), np.full(t.size, v), np.zeros(t.size), 0)


def test_motion_compensation_puts_wall_where_it_is():
    log = _straight_log()
    t_ref = 500 * MS
    want = 3.0 - 2.0 * 0.5                                      # 撮影時刻の base_link から壁まで
    got = lidar_ranges_at(log, t_ref, LIDAR, compensate=True)
    assert abs(got[0] - want) < 0.005
    raw = lidar_ranges_at(log, t_ref, LIDAR, compensate=False, max_dt_ms=200)
    # 補正しないと、別の時刻に撮った点のぶんずれる（2m/s で数 cm 以上）
    assert abs(raw[0] - want) > abs(got[0] - want)


def test_score_frame_classes_and_recall():
    lidar = np.zeros(360)
    lidar[0], lidar[10], lidar[20], lidar[30] = 1.0, 1.0, 1.0, 1.0
    stereo = np.zeros(360)
    stereo[0] = 1.05          # 一致
    stereo[10] = 0.6          # LiDAR より手前
    stereo[20] = 1.5          # 奥
    stereo[45] = 0.8          # LiDAR に何も無い
    sc = score_frame(stereo, lidar, fov_degs=range(-50, 51), p=DriveParams())
    cls = dict(zip(sc.deg.tolist(), sc.cls.tolist()))
    assert cls == {0: 0, 10: 1, 20: 2, 45: 1}
    # LiDAR が 0.4〜1.5m に点を持つ 4 方位のうち、ステレオが ±1° に何か出したのは 0・10・20 の 3
    assert (sc.n_recall, sc.n_recalled) == (4, 3)


def test_fit_yaw_finds_rotated_stereo():
    log = _straight_log(v=1.5, side_y=0.6)
    # 斜めの壁も写るよう、ステレオは LiDAR の距離を 3° 回して出す（向きが 3° ずれたカメラ）
    stereo = []
    for t in range(200, 1000, 100):
        L = lidar_ranges_at(log, t * MS, LIDAR)
        stereo.append((t * MS, np.roll(np.where((L > 0) & (L < 2.0), L, 0.0), -3)))
    rows = fit_yaw(log, range(-50, 51), LIDAR, stereo, degs=range(-6, 7), min_speed=1.0)
    best = max(rows, key=lambda r: (r[1], -r[2]))
    assert best[0] == 3


def test_drive_eval_counts_frames():
    log = _straight_log()
    stereo = [(t * MS, np.where((d := lidar_ranges_at(log, t * MS, LIDAR)) < 2.0, d, 0.0))
              for t in range(200, 1000, 100)]
    res = drive_eval(log, range(-50, 51), LIDAR, stereo=stereo)
    s = res.summary()["全部"]
    assert s["frames"] == len(stereo)
    assert s["match"] == 1.0
