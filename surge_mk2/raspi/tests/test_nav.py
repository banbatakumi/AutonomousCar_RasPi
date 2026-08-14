"""自己位置推定と地図（`raspi/nav/`）のテスト。

**ハードウェアもバスも要らない。** 合成した部屋から点群を作って流し込むだけ
（`test_auto.py` と同じ流儀）。地図と姿勢は数値で答え合わせできるので、
「止まる」を厚く試す `test_auto.py` と違い、こちらは**精度を数値で縛る**。

`make_room_scan()` が実機と同じ約束の `Scan` を作る:

- `dist` の添字がそのまま車両角 [deg]（x=前 が 0°、反時計回り正）
- `sector_t_ns` / `sector_dur_us` は**車両角では添字が減る向きに時刻が進む**
- `sector_seen[s]` が持つのは `dist[30*s+1]` 〜 `dist[(30*s+30) % 360]`（境界が1ずれる）
"""

import math
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.auto.base import sector_of_deg  # noqa: E402
from raspi.msgs import Scan  # noqa: E402
from raspi.nav import OccGrid, deskew, match  # noqa: E402
from raspi.nav.deskew import _point_times_ns, integrate_pose  # noqa: E402
from raspi.nav.grid import FREE, OCCUPIED, UNKNOWN, dilate  # noqa: E402
from raspi.nav.slam import Slam, SlamConfig  # noqa: E402

NS = 1_000_000_000

#: 部屋の壁（軸に平行な線分の集まり）。**左右非対称にしてある** — 正方形だと
#: 90° 回転しても同じ形になり、スキャンマッチが別解に落ちても気づけない
ROOM = [
    (0.0, 0.0, 6.0, 0.0),      # 下
    (6.0, 0.0, 6.0, 4.0),      # 右
    (6.0, 4.0, 0.0, 4.0),      # 上
    (0.0, 4.0, 0.0, 0.0),      # 左
    (1.5, 1.0, 2.5, 1.0),      # 中の板（対称性を壊すためだけに置いてある）
    (2.5, 1.0, 2.5, 2.2),
]


def _ray_segments(ox, oy, ang, segs, max_range):
    """線分の集まりに対するレイキャスト。当たらなければ 0.0（実機の「無効」と同じ）。"""
    dx, dy = math.cos(ang), math.sin(ang)
    best = max_range
    hit = False
    for x1, y1, x2, y2 in segs:
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        if abs(den) < 1e-12:
            continue
        t = ((x1 - ox) * ey - (y1 - oy) * ex) / den
        u = ((x1 - ox) * dy - (y1 - oy) * dx) / den
        if t > 1e-6 and 0.0 <= u <= 1.0 and t < best:
            best, hit = t, True
    return best if hit else 0.0


def make_room_scan(x, y, yaw, *, t0=1_000_000_000, missing=(), max_range=8.0,
                   segs=ROOM, sweep_ns=100_000_000) -> Scan:
    """部屋の中の姿勢 `(x, y, yaw)` から見える点群を作る。

    `missing` に入れたセクタ番号は `sector_seen=False`（受信できなかった）にする。
    """
    dist = [0.0] * 360
    for deg in range(360):
        if sector_of_deg(deg) in missing:
            continue
        dist[deg] = _ray_segments(x, y, yaw + math.radians(deg), segs, max_range)

    # センサは 12 セクタを順に送る。**車両角では添字が減る向きに時刻が進む**ので、
    # セクタ s の時刻は s が大きいほど早い（`Scan` の docstring の式の裏返し）
    dur = sweep_ns // 12
    t_sector = [t0 + (11 - s) * dur for s in range(12)]
    return Scan(
        t_capture=t0, dist=dist,
        sector_t_ns=[0 if s in missing else t_sector[s] for s in range(12)],
        sector_dur_us=[0 if s in missing else dur // 1000 for s in range(12)],
        sector_seen=[s not in missing for s in range(12)],
        rot_speed_dps=3600.0,
    )


class TestPointTimes(unittest.TestCase):
    """点ごとの時刻は `Scan` の docstring の式と一致しなければならない。"""

    def test_matches_sector_of_deg(self):
        """時刻を引くセクタと `sector_seen` を引くセクタは**同じ**でなければならない。

        `deg // 30` と書くと 30 点ぶんずれ、脱スキューが歪みを増やす方向に働く。
        """
        scan = make_room_scan(3.0, 2.0, 0.0)
        t = _point_times_ns(scan)
        for deg in range(360):
            s = sector_of_deg(deg)
            lo = scan.sector_t_ns[s]
            hi = lo + scan.sector_dur_us[s] * 1000
            self.assertTrue(lo <= t[deg] <= hi,
                            f"deg={deg} の時刻 {t[deg]} がセクタ {s} の範囲外")

    def test_time_advances_as_index_decreases(self):
        """車両角の添字が**減る**向きに時刻が進む（LD06 が裏向きのため）。"""
        scan = make_room_scan(3.0, 2.0, 0.0)
        t = _point_times_ns(scan)
        self.assertGreater(t[1], t[359])
        self.assertGreater(t[100], t[200])


class TestDeskew(unittest.TestCase):
    def test_static_scan_is_unchanged(self):
        """止まっていれば補正しない。**速度のノイズで点群を汚さない。**"""
        scan = make_room_scan(3.0, 2.0, 0.0)
        pts = deskew(scan, max_range=8.0)
        self.assertFalse(pts.corrected)
        # 部屋の隅までの距離。1° 刻みなので隅ちょうどの方位は撃っていない
        d = np.hypot(pts.x, pts.y)
        self.assertAlmostEqual(float(d.max()), math.hypot(3.0, 2.0), delta=0.05)

    def test_mount_offset_is_applied(self):
        """LiDAR の取付位置ぶん、点は base_link 座標で前へずれる。"""
        scan = make_room_scan(3.0, 2.0, 0.0)
        a = deskew(scan, mount_x=0.0)
        b = deskew(scan, mount_x=0.12)
        self.assertAlmostEqual(float(np.mean(b.x - a.x)), 0.12, places=6)

    def test_missing_sector_points_are_dropped(self):
        """**欠測セクタの点は1点も出さない。** 空きとしても彫らせない。"""
        full = deskew(make_room_scan(3.0, 2.0, 0.0))
        holed = deskew(make_room_scan(3.0, 2.0, 0.0, missing=(2, 3)))
        self.assertEqual(len(full) - len(holed), 60)

    def test_zero_distance_is_dropped_not_treated_as_free(self):
        """`dist == 0` は捨てる。**FTG と違い「空き」に倒さない。**"""
        scan = make_room_scan(3.0, 2.0, 0.0)
        scan.dist[10] = 0.0
        pts = deskew(scan)
        ang = np.degrees(np.arctan2(pts.y, pts.x)) % 360
        self.assertFalse(np.any(np.abs(ang - 10.0) < 0.5))

    def test_saturated_point_is_free_only(self):
        """飽和点は `hit=False`。**壁として打たないが、手前は空きとして使う。**"""
        scan = make_room_scan(3.0, 2.0, 0.0)
        scan.saturated = [i == 45 for i in range(360)]
        pts = deskew(scan)
        ang = np.degrees(np.arctan2(pts.y, pts.x)) % 360
        j = int(np.argmin(np.abs(ang - 45.0)))
        self.assertFalse(bool(pts.hit[j]))
        self.assertTrue(bool(pts.hit.sum()) and len(pts) > 300)

    def test_straight_motion_is_undone(self):
        """直進しながら測った歪んだ点群が、補正で元の形に戻る。

        1周 100ms のあいだに 0.2m 進むので、補正しないと壁が最大 20cm 二重になる。
        """
        speed = 2.0
        segs = ROOM
        t0 = 10 * NS
        sweep = 100_000_000
        dist = [0.0] * 360
        for deg in range(360):
            s = sector_of_deg(deg)
            sensor = (360 - deg) % 360
            i = sensor % 30
            t = t0 + (11 - s) * (sweep // 12) + (sweep // 12) * i // 29
            tau = (t0 + sweep - t) / NS          # 基準時刻までの残り
            # 基準時刻に (3,2) に居るように、その tau 秒前の位置から測る
            px = 3.0 - speed * tau
            dist[deg] = _ray_segments(px, 2.0, math.radians(deg), segs, 8.0)
        dur = sweep // 12
        scan = Scan(t_capture=t0, dist=dist,
                    sector_t_ns=[t0 + (11 - s) * dur for s in range(12)],
                    sector_dur_us=[dur // 1000] * 12,
                    sector_seen=[True] * 12, rot_speed_dps=3600.0)

        raw = deskew(scan)                          # 補正なし
        fixed = deskew(scan, speed)                 # 補正あり
        self.assertTrue(fixed.corrected)

        # 正解は「基準時刻に (3,2) で止まって測った点群」。そこからの最近傍距離で測る
        # （壁までの距離で測ると、部屋の中の板に落ちた点が誤差として乗ってしまう）
        ideal = deskew(make_room_scan(3.0, 2.0, 0.0))

        def err(p):
            d = np.hypot(p.x[:, None] - ideal.x[None, :],
                         p.y[:, None] - ideal.y[None, :])
            return float(np.mean(d.min(axis=1)))

        self.assertLess(err(fixed), err(raw) * 0.5)
        self.assertLess(err(fixed), 0.02)


class TestOccGrid(unittest.TestCase):
    def setUp(self):
        self.g = OccGrid(resolution=0.05, size_m=12.0, origin=(-1.0, -1.0))

    def test_walls_need_repeated_hits(self):
        """1回しか当たらないセルは壁にならない（＝**動く物を壁にしない**）。"""
        scan = make_room_scan(3.0, 2.0, 0.0)
        self.g.integrate(deskew(scan), (3.0, 2.0, 0.0))
        self.assertEqual(int(self.g.wall_mask().sum()), 0)
        for _ in range(2):
            self.g.integrate(deskew(scan), (3.0, 2.0, 0.0))
        self.assertGreater(int(self.g.wall_mask().sum()), 100)

    def test_missing_sector_is_not_carved_free(self):
        """★欠測セクタの方向は「空き」として彫られない。

        `follow_the_gap.py:19-32` と同じ規約。ここを破ると、受信できなかっただけの
        方向が地図の上で「通れる」ことになる。
        """
        holed = make_room_scan(3.0, 2.0, 0.0, missing=(0,))
        for _ in range(4):
            self.g.integrate(deskew(holed), (3.0, 2.0, 0.0))
        free = self.g.known_free_mask()
        # セクタ0 は車両角 1〜30°。その方向 1m 先のセルが「空き」になっていないこと
        for deg in (5, 15, 25):
            a = math.radians(deg)
            col, row = self.g.to_cell(3.0 + math.cos(a), 2.0 + math.sin(a))
            self.assertFalse(bool(free[row, col]),
                             f"{deg}° が空きとして彫られている")

    def test_saturated_carves_free_but_no_wall(self):
        """飽和点は手前を空きにするが、終端に壁を作らない。"""
        scan = make_room_scan(3.0, 2.0, 0.0)
        scan.saturated = [True] * 360
        for _ in range(5):
            self.g.integrate(deskew(scan), (3.0, 2.0, 0.0))
        self.assertEqual(int(self.g.wall_mask().sum()), 0)
        self.assertGreater(int(self.g.known_free_mask().sum()), 100)

    def test_trinary_values(self):
        scan = make_room_scan(3.0, 2.0, 0.0)
        for _ in range(4):
            self.g.integrate(deskew(scan), (3.0, 2.0, 0.0))
        t = self.g.trinary()
        self.assertEqual(set(np.unique(t)), {UNKNOWN, FREE, OCCUPIED})

    def test_freeze_stops_updates(self):
        scan = make_room_scan(3.0, 2.0, 0.0)
        for _ in range(4):
            self.g.integrate(deskew(scan), (3.0, 2.0, 0.0))
        self.g.freeze()
        before = self.g.hits.copy()
        self.g.integrate(deskew(scan), (3.0, 2.0, 0.0))
        self.assertTrue(np.array_equal(before, self.g.hits))

    def test_raycast_measures_room_width(self):
        """壁までの距離が実際の部屋と合う（**道幅の測定がこれに乗る**）。

        1点に止まったままだと 1° 刻みの点群が 3m 先で 5.2cm 間隔になり、
        5cm のセルに穴が空く。実際は走りながら測るので、少し動かして埋める。
        """
        for i in range(6):
            x = 3.0 + 0.02 * i
            self.g.integrate(deskew(make_room_scan(x, 3.0, 0.0)), (x, 3.0, 0.0))
        d = self.g.raycast(3.05, 3.0, np.array([math.pi / 2, -math.pi / 2]), 4.0)
        self.assertAlmostEqual(float(d[0]), 1.0, delta=0.08)   # 上の壁 y=4
        self.assertAlmostEqual(float(d[1]), 3.0, delta=0.08)   # 下の壁 y=0

    def test_dilate(self):
        m = np.zeros((7, 7), dtype=bool)
        m[3, 3] = True
        self.assertEqual(int(dilate(m, 1).sum()), 5)
        self.assertEqual(int(dilate(m, 0).sum()), 1)


class TestScanMatch(unittest.TestCase):
    """★ 既知のずれを与えて、それを取り戻せることを数値で縛る。"""

    def setUp(self):
        # **少しずつ動かしながら焼く。** 1点に止まって焼いた地図は 1° 刻みの
        # 点間隔（3m 先で 5.2cm）がセルより粗く、壁に穴だらけになる
        self.g = OccGrid(resolution=0.05, size_m=12.0, origin=(-1.0, -1.0))
        for i in range(10):
            p = (3.0 + 0.015 * (i % 4), 2.0 + 0.015 * (i // 4), 0.0)
            self.g.integrate(deskew(make_room_scan(*p)), p)

    def _check(self, truth, guess, *, tol_xy=0.03, tol_deg=1.0, cycles=12):
        """**数周期かけて寄る**のを許す。

        1回で戻せる量は探索範囲（±8cm / ±4°）までで、それより大きなずれは
        毎周期少しずつ縮む。実機も 10Hz で繰り返し呼ばれるので、
        「1回で戻せること」を要求する方が実態に合わない。
        """
        pts = deskew(make_room_scan(*truth))
        m = match(self.g, pts, guess)
        for _ in range(cycles - 1):
            m = match(self.g, pts, (m.x, m.y, m.yaw))
        self.assertTrue(m.searched)
        self.assertAlmostEqual(m.x, truth[0], delta=tol_xy)
        self.assertAlmostEqual(m.y, truth[1], delta=tol_xy)
        self.assertLess(abs(math.degrees(m.yaw - truth[2])), tol_deg)
        self.assertGreater(m.score, 0.5)

    def test_recovers_translation(self):
        self._check((3.06, 1.95, 0.0), (3.0, 2.0, 0.0))

    def test_recovers_rotation(self):
        self._check((3.0, 2.0, math.radians(3.0)), (3.0, 2.0, 0.0))

    def test_recovers_both(self):
        self._check((3.05, 2.04, math.radians(-2.5)), (3.0, 2.0, 0.0))

    def test_large_error_converges_over_time(self):
        """★大きくずれても**時間をかけて**寄る。

        1回で戻せる量は探索範囲（±8cm）まで。これを広げれば速く戻せるが、
        代わりに毎周期そのぶん滑る余地を与えることになる（`nav/scanmatch.py`）。
        10Hz で 2 秒かけて 15cm 戻せれば、実用上それで足りる。
        """
        truth = (3.15, 2.0, 0.0)
        pts = deskew(make_room_scan(*truth))
        guess = (3.0, 2.0, 0.0)
        for _ in range(20):
            m = match(self.g, pts, guess)
            guess = (m.x, m.y, m.yaw)
        self.assertAlmostEqual(guess[0], truth[0], delta=0.04)

    def test_empty_map_does_not_search(self):
        """地図が空なら探索しない。**全候補 0 点で原点へ吸い寄せられるのを防ぐ。**"""
        empty = OccGrid(resolution=0.05, size_m=12.0, origin=(-1.0, -1.0))
        pts = deskew(make_room_scan(3.0, 2.0, 0.0))
        m = match(empty, pts, (1.0, 1.0, 0.5))
        self.assertFalse(m.searched)
        self.assertEqual((m.x, m.y, m.yaw), (1.0, 1.0, 0.5))


class TestIntegratePose(unittest.TestCase):
    def test_straight(self):
        x, y, th = integrate_pose(0.0, 0.0, 0.0, 1.0, 0.0)
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 0.0)

    def test_arc_is_not_a_chord(self):
        """円弧で積む。**直線近似だと 100ms・最大舵角で数 cm ずれる。**"""
        x, y, th = integrate_pose(0.0, 0.0, 0.0, math.pi / 2, math.pi / 2)
        self.assertAlmostEqual(x, 1.0, places=6)     # 半径 1 の 90° 旋回
        self.assertAlmostEqual(y, 1.0, places=6)
        self.assertAlmostEqual(th, math.pi / 2, places=6)


class TestSlam(unittest.TestCase):
    def setUp(self):
        self.slam = Slam(SlamConfig(resolution=0.05, size_m=16.0, max_range=8.0))

    @staticmethod
    def creep(n, *, x0=3.0, y=2.0, step=0.02, yaw=0.0):
        """じりじり前進する姿勢の並び。

        **止まったままでは地図が育たない**（`nav/slam.py` のキーフレーム条件）。
        同じ場所を何度焼いても情報は増えないので、それが正しい振る舞い。
        """
        return [(x0 + step * i, y, yaw) for i in range(n)]

    def _drive(self, poses, *, dt=0.1, warmup=4):
        """姿勢の並びを順に食わせる。

        **実機と同じ入力を渡す**: 点群に加えてジャイロ（`yaw_rate`）と車速
        （`speed`）。ここを渡さないと SLAM は推測航法を持たないので、
        テストだけが実機より不利な条件になる（`nav/slam.py` の冒頭）。

        先頭で `warmup` 周ぶん止まったまま食わせる（実車も engage 直後は止まっている）。
        """
        seq = [poses[0]] * warmup + list(poses)
        out = []
        prev = None
        for p in seq:
            if prev is None:
                spd = rate = 0.0
            else:
                spd = math.hypot(p[0] - prev[0], p[1] - prev[1]) / dt
                rate = _wrap(p[2] - prev[2]) / dt
            out.append(self.slam.update(make_room_scan(*p), dt,
                                        yaw_rate=rate, speed=spd))
            prev = p
        return out

    def test_stationary_pose_does_not_drift(self):
        """止まっていれば姿勢は動かない。

        **SLAM の原点は「最初の姿勢」**（`map` フレーム）であって部屋の座標ではない。
        真値との差ではなく、動いていないことを見る。
        """
        self._drive([(3.0, 2.0, 0.0)] * 12)
        x, y, yaw = self.slam.pose
        self.assertAlmostEqual(x, 0.0, delta=0.03)
        self.assertAlmostEqual(y, 0.0, delta=0.03)
        self.assertLess(abs(math.degrees(yaw)), 1.0)

    def test_tracks_a_straight_run(self):
        """★ まっすぐ 1m 走ったら、地図の中でも 1m 進んでいる。

        SLAM の原点は「最初の姿勢」なので、真値との差ではなく**移動量**で比べる。
        """
        poses = [(3.0 - 0.5 + 0.05 * i, 2.0, 0.0) for i in range(21)]
        self._drive(poses)
        x, y, _ = self.slam.pose
        self.assertAlmostEqual(x, 1.0, delta=0.08)
        self.assertAlmostEqual(y, 0.0, delta=0.05)

    def test_heading_total_accumulates_beyond_180(self):
        """累積回頭は ±180 に畳まない。**周回の判定がこれに乗っている。**"""
        poses = [(3.0, 2.0, math.radians(a)) for a in range(0, 300, 5)]
        self._drive(poses)
        self.assertGreater(self.slam.lap_progress(), 0.7)

    def test_gyro_bias_is_estimated(self):
        """★ジャイロに一定のバイアスを乗せても、推定して引けること。

        引けないと方位がゆっくり流れ、**それが地図の歪みとして焼き付く**。
        """
        bias = math.radians(3.0)                 # 3°/s の大きめのバイアス
        self._drive(self.creep(14))              # まず地図を作る（止まったままでは育たない）
        for i in range(400):                     # 時定数 10 秒 × 数本ぶん
            # まっすぐ進んでいるのに、ジャイロだけが回っていると言い続ける
            self.slam.update(make_room_scan(3.3 + 0.002 * i, 2.0, 0.0), 0.1,
                             yaw_rate=bias, speed=0.02)
        self.assertAlmostEqual(math.degrees(self.slam.gyro_bias),
                               math.degrees(bias), delta=1.0)
        # **バイアスを引けているので姿勢は流れない**
        self.assertLess(abs(math.degrees(self.slam.yaw)), 8.0)

    def test_reset_clears_everything(self):
        self._drive(self.creep(10))
        self.assertTrue(self.slam.trajectory)
        self.slam.reset()
        self.assertEqual(self.slam.trajectory, [])
        self.assertEqual(self.slam.pose, (0.0, 0.0, 0.0))
        self.assertEqual(int(self.slam.grid.hits.sum()), 0)
        self.assertEqual(self.slam.distance, 0.0)

    def test_lost_scan_does_not_poison_the_map(self):
        """★見失った周は地図を更新しない。

        ずれた姿勢の壁を混ぜると、次の周期はもっと合わなくなる（一度崩れると戻らない）。
        """
        self._drive(self.creep(14))
        walls = int(self.slam.grid.wall_mask().sum())
        self.assertGreater(walls, 50)
        # まったく形の違う部屋（細い廊下）を1周ぶん食わせる。壁が合うはずがない
        corridor = [(-9.0, 1.7, 9.0, 1.7), (-9.0, 2.3, 9.0, 2.3),
                    (-9.0, 1.7, -9.0, 2.3), (9.0, 1.7, 9.0, 2.3)]
        far = make_room_scan(3.0, 2.0, 0.0, segs=corridor, max_range=8.0)
        u = self.slam.update(far, 0.1, yaw_rate=0.0, speed=0.0)
        self.assertTrue(u.lost)
        self.assertEqual(int(self.slam.grid.wall_mask().sum()), walls)

    def test_update_is_fast_enough(self):
        """1周期の予算は 10Hz に対して十分小さいこと。

        planning_node は 50Hz で `auto/cmd` を出しており、`plan()` が 100ms
        掛かると中継側が制動に落とす（`test_auto.py` の `test_stale_auto_cmd_...`）。
        """
        self._drive([(3.0, 2.0, 0.0)] * 5)
        scan = make_room_scan(3.05, 2.0, 0.0)
        t0 = time.perf_counter()
        for _ in range(5):
            self.slam.update(scan, 0.1, yaw_rate=0.0, speed=0.5)
        dt_ms = (time.perf_counter() - t0) / 5 * 1000
        self.assertLess(dt_ms, 50.0, f"1周期 {dt_ms:.1f}ms は遅すぎる")


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


if __name__ == "__main__":
    unittest.main()
