"""`raspi/auto/park_to_point.py`（`ParkToPoint.plan()`）の単体テスト。

バス・実機不要。`plan()`を直接ループして、Reeds-Sheppパス生成・追従・
デッドレコニング積分・障害物安全策の健全性を確認する。
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.auto.park_to_point import ParkToPoint  # noqa: E402
from raspi.msgs.types import Scan, VehicleState  # noqa: E402
from raspi.nav.deskew import Points  # noqa: E402
from raspi.nav.reeds_shepp import (  # noqa: E402
    PathSegment, ReedsSheppPath, sample_path_array,
)

import numpy as np  # noqa: E402


def _points(xy):
    """`(x,y)`のリストから`Points`（脱スキュー済み点群）を作る。"""
    arr = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    return Points(x=arr[:, 0], y=arr[:, 1],
                  hit=np.ones(len(arr), dtype=bool),
                  t_ref_ns=0, corrected=True)

DT = 0.1  # `plan()`が事実上LiDARの10Hzでしか呼ばれない前提に合わせる


def _open_scan() -> Scan:
    """全方位が「見えていて」十分遠い、障害物なしのスキャン。"""
    return Scan(dist=[5.0] * 360, sector_seen=[True] * 12)


#: 理想運動学テストでは**スキャンアンカを切る**。
#:
#: ★ここは黙って壊れやすい。合成スキャンは毎周期同じものを渡すので、
#: 「車体は動いているのにスキャンが変わらない」という物理的にあり得ない
#: 入力になっている。アンカ（スキャン同士の登録）はそれを素直に信じて
#: 「動いていない」と答えるため、姿勢が原点に張り付いて収束しない。
#: 以前これらのテストが通っていたのは、点群の取り込み距離が`2.0m`で
#: 合成スキャン（5.0m）の全点が非ヒットになり、**アンカが偶然無効化されて
#: いた**から。取り込み距離を5.0mへ直した瞬間に露出した（2026-09-17）。
#: アンカの検証は`test_scan_anchor.py`と`sim.park_bench`（世界と整合した
#: スキャンを毎周期作る）が受け持つ。
IDEAL = {"anchor_enabled": 0.0}


def _drive_until_done(pp: ParkToPoint, max_steps: int = 800):
    """理想運動学（自転車モデル）で`plan()`の出力を積分する簡易シム。

    `curvature`から直接舵角が決まる設計なので、`target_steer`→`yaw_rate`の
    変換は自転車モデルの順方向（`tan(steer)/L`）でよい。
    """
    scan = _open_scan()
    vs = VehicleState(odom_center=0.0, yaw_rate=0.0)
    st = None
    wheelbase = pp.vehicle.wheelbase
    for _ in range(max_steps):
        st = pp.plan(scan, vs, pp.merged(IDEAL), DT)
        if st.phase in ("完了", "失敗"):
            break
        v = 0.0 if st.brake else st.target_speed
        yaw_rate = v / wheelbase * math.tan(st.target_steer) if wheelbase else 0.0
        vs = VehicleState(odom_center=vs.odom_center + v * DT, yaw_rate=yaw_rate)
    return st


def _warm(pp, scan, vs, p, n=4):
    """局所地図が`ready`になるまで`plan()`を回す。

    `LocalMap`は`min_hits`周ぶん観測しないと壁を確定しないので、planner は
    それまで「地図を初期化中」として制動する（障害物を無視した経路を出さない
    ための意図的な待ち）。単体テストは1回の`plan()`で判定したいので、
    ここで待ちを消化する。
    """
    st = None
    for _ in range(n):
        st = pp.plan(scan, vs, p, DT)
    return st


class TestNoTarget(unittest.TestCase):
    def test_no_target_is_not_ready(self):
        pp = ParkToPoint()
        st = pp.plan(_open_scan(), VehicleState(), pp.merged({}), DT)
        self.assertFalse(st.ready)
        self.assertIn("目標", st.reason)

    def test_no_vehicle_state_is_not_ready(self):
        pp = ParkToPoint()
        pp.request_park_target(1.0, 0.0, 0.0)
        st = pp.plan(_open_scan(), None, pp.merged({}), DT)
        self.assertFalse(st.ready)


class TestConvergence(unittest.TestCase):
    def test_front_target_converges_forward(self):
        pp = ParkToPoint()
        pp.request_park_target(1.0, 0.0, 0.0)
        st = _drive_until_done(pp)
        self.assertEqual(st.phase, "完了")

    def test_rear_target_converges(self):
        pp = ParkToPoint()
        pp.request_park_target(-1.0, 0.0, 0.0)
        st = _drive_until_done(pp)
        self.assertEqual(st.phase, "完了")

    def test_quarter_turn_converges(self):
        """正面1m・90度回転——単一フィードバック方式では位置は合うが向きが
        揃わなかった配置（PROGRESS.md参照）。Reeds-Sheppパスなら到達できるはず。"""
        pp = ParkToPoint()
        pp.request_park_target(1.0, 0.0, math.radians(90))
        st = _drive_until_done(pp)
        self.assertEqual(st.phase, "完了")

    def test_lateral_offset_same_heading_converges(self):
        """向きは同じで真横にオフセット——単一フィードバック方式では位置が
        全く収束しなかった配置。切り返し（S字）で解けるはず。"""
        pp = ParkToPoint()
        pp.request_park_target(0.0, 0.5, 0.0)
        st = _drive_until_done(pp)
        self.assertEqual(st.phase, "完了")

    def test_side_target_with_yaw_converges(self):
        pp = ParkToPoint()
        pp.request_park_target(0.6, 0.6, math.radians(90))
        st = _drive_until_done(pp)
        self.assertEqual(st.phase, "完了")

    def test_reverse_into_spot_converges(self):
        pp = ParkToPoint()
        pp.request_park_target(-1.0, 0.5, math.radians(180))
        st = _drive_until_done(pp)
        self.assertEqual(st.phase, "完了")


class TestDeadReckoning(unittest.TestCase):
    def test_odom_center_diff_moves_target_as_expected(self):
        """車体が直進1mするとき、目標のローカル座標が1m分手前に近づくこと。"""
        pp = ParkToPoint()
        pp.request_park_target(2.0, 0.0, 0.0)
        vs = VehicleState(odom_center=0.0, yaw_rate=0.0)
        st = _warm(pp, _open_scan(), vs, pp.merged({}))
        rho0 = st.park_rho
        vs2 = VehicleState(odom_center=1.0, yaw_rate=0.0)  # 1m 直進した想定
        st2 = pp.plan(_open_scan(), vs2, pp.merged({}), DT)
        self.assertAlmostEqual(rho0 - st2.park_rho, 1.0, places=3)

    def test_reset_clears_target(self):
        pp = ParkToPoint()
        pp.request_park_target(1.0, 0.0, 0.0)
        pp.reset()
        st = pp.plan(_open_scan(), VehicleState(), pp.merged({}), DT)
        self.assertFalse(st.ready)
        self.assertFalse(st.park_active)


class TestSafetyBrake(unittest.TestCase):
    """掃引ベースの安全停止（`_safety_brake`）。

    ★旧`_obstacle_brake()`（進行方向±25°の窓の最近点）は撤去した。
    旋回中の前後端の振り出しを見られず（経路が衝突する障害物の16%は衝突まで
    窓に入らない）、さらに「目標と同距離の障害物を無視する」ゲートが
    縦列駐車で最も当たりやすい隣の車を無視していたため。
    """

    def _straight_path(self, pp, length=1.0, gear=1):
        """まっすぐ`length`進む参照経路を仕込む（原点・+x向き）。

        掃引は**指令舵角**（`pp._steer`）で作られるので、それも合わせる。
        """
        path = ReedsSheppPath(segments=(PathSegment(gear=gear, curvature=0.0,
                                                    length=length),))
        pp._set_path(path, (0.0, 0.0, 0.0))
        pp._steer = 0.0
        return path

    def test_obstacle_on_the_swept_path_brakes(self):
        """制動距離以内で車体が当たる位置に点があれば止まること。"""
        pp = ParkToPoint()
        p = pp.merged({})
        self._straight_path(pp)
        # 車体前端(0.30m)のすぐ先、制動距離の内側に点を置く
        pts = _points([(0.34, 0.0)])
        brake, reason = pp._safety_brake(pts, (0.0, 0.0, 0.0), 0.3, 0, p)
        self.assertTrue(brake, reason)
        self.assertIn("障害物", reason)

    def test_obstacle_far_ahead_does_not_brake(self):
        """制動距離よりずっと先の障害物では止まらないこと。"""
        pp = ParkToPoint()
        p = pp.merged({})
        self._straight_path(pp)
        pts = _points([(1.5, 0.0)])
        brake, reason = pp._safety_brake(pts, (0.0, 0.0, 0.0), 0.3, 0, p)
        self.assertFalse(brake, reason)

    def test_obstacle_beside_the_path_does_not_brake(self):
        """真横（車幅の外）の壁では止まらないこと——**目標の壁で止まらない**
        ことの本質。旧実装はここを`rho`との比較というヒューリスティックで
        誤魔化していた。"""
        pp = ParkToPoint()
        p = pp.merged({})
        self._straight_path(pp)
        pts = _points([(0.3, 0.30), (0.5, 0.30), (0.3, -0.30)])
        brake, reason = pp._safety_brake(pts, (0.0, 0.0, 0.0), 0.3, 0, p)
        self.assertFalse(brake, reason)

    def test_reverse_path_looks_behind(self):
        """後退の参照経路では**後方**の掃引を見ること。"""
        pp = ParkToPoint()
        p = pp.merged({})
        self._straight_path(pp, gear=-1)
        ahead = _points([(0.34, 0.0)])          # 前方の近接物は無関係
        behind = _points([(-0.12, 0.0)])        # 後端(-0.07m)のすぐ後ろ
        self.assertFalse(pp._safety_brake(ahead, (0.0, 0.0, 0.0), -0.3, 0, p)[0])
        pp._set_path(pp._path, (0.0, 0.0, 0.0))  # 参照位置を巻き戻す
        self.assertTrue(pp._safety_brake(behind, (0.0, 0.0, 0.0), -0.3, 0, p)[0])

    def test_swept_check_catches_corner_swing(self):
        """**旋回中の前端の振り出し**を捕まえること（旧方式が見落とした16%）。

        最大舵角で前進する経路の、進行方向±25°の窓の外に障害物を置く。
        """
        pp = ParkToPoint()
        p = pp.merged({})
        kappa = math.tan(pp.vehicle.max_steer) / pp.vehicle.wheelbase
        path = ReedsSheppPath(segments=(PathSegment(gear=1, curvature=kappa,
                                                    length=0.5),))
        pp._set_path(path, (0.0, 0.0, 0.0))
        pp._steer = pp.vehicle.max_steer       # 掃引は指令舵角で作られる
        poses = sample_path_array((0.0, 0.0, 0.0), path, step=0.02)
        # 制動距離内の姿勢で、車体の左前角が通る位置
        i = min(6, len(poses) - 1)
        x, y, yaw = poses[i]
        ox = x + 0.30 * math.cos(yaw) - 0.09 * math.sin(yaw)
        oy = y + 0.30 * math.sin(yaw) + 0.09 * math.cos(yaw)
        ang = math.degrees(math.atan2(oy, ox))
        self.assertGreater(abs(ang), 25.0,
                           f"この点は旧方式の安全窓(±25°)の中にある({ang:.0f}°)")
        brake, reason = pp._safety_brake(_points([(ox, oy)]), (0.0, 0.0, 0.0),
                                         0.3, 0, p)
        self.assertTrue(brake, reason)


class TestObstacleAwarePlanning(unittest.TestCase):
    """経路計画そのものが障害物を避けること（`_replan()`の統合）。"""

    def test_obstacle_on_the_chosen_path_forces_a_different_clear_path(self):
        """**選ばれた経路の上に**障害物を置くと、別の経路になり、かつ避けること。

        ★以前は「curvatureの符号が反転する」ことを見ていたが、それは
        「RS候補から左右反対の候補を選ぶ」という旧実装の機構に固有の性質で、
        Hybrid A*は同じ初期曲率のまま別の避け方をすることがある。また
        「適当な方向に障害物を置く」のでは、そもそも経路の上に無くて
        避ける必要が無い場合がある（実際にこれで偽陽性のテストになっていた）。
        縛るべきなのは機構ではなく**結果**——経路が変わり、余裕が保たれること。
        """
        vs0 = VehicleState(odom_center=0.0, yaw_rate=0.0)
        pp_clear = ParkToPoint()
        p = pp_clear.merged({})
        pp_clear.request_park_target(0.0, 0.5, 0.0)
        st_clear = _warm(pp_clear, _open_scan(), vs0, p)
        self.assertFalse(st_clear.brake, st_clear.reason)
        clear_path = [(s.gear, round(s.curvature, 3), round(s.length, 3))
                      for s in pp_clear._path.segments]

        # 選ばれた経路の中点あたりの車体中心へ障害物を置く
        poses = sample_path_array(pp_clear._park_pose, pp_clear._path, step=0.02)
        mid = poses[len(poses) // 2]
        ox = mid[0] + 0.115 * math.cos(mid[2])       # base_link → 車体中心
        oy = mid[1] + 0.115 * math.sin(mid[2])
        # LiDAR座標（原点はlidar_x前方）へ直してスキャンに焼く
        lx = ox - pp_clear.vehicle.lidar_x
        deg = int(round(math.degrees(math.atan2(oy, lx)))) % 360
        dist = math.hypot(lx, oy)
        scan = Scan(dist=[5.0] * 360, sector_seen=[True] * 12)
        for d in range(deg - 6, deg + 7):
            scan.dist[d % 360] = dist

        pp = ParkToPoint()
        pp.request_park_target(0.0, 0.5, 0.0)
        st = _warm(pp, scan, vs0, p)
        blocked_path = [(s.gear, round(s.curvature, 3), round(s.length, 3))
                        for s in pp._path.segments]
        self.assertNotEqual(clear_path, blocked_path, "経路上に障害物があるのに経路が同じ")
        if not st.brake:
            got = pp._map.path_clearance(
                sample_path_array(pp._park_pose, pp._path, step=0.03))
            # ★`collision_margin_m`と比較してはいけない: 駐車は壁に詰める
            # 動作なので、目標自身の余裕がマージンより小さければ planner が
            # 必要なぶんだけマージンを緩める（`hybrid_astar.plan()`参照）。
            # 縛れるのは「食い込んでいないこと」
            self.assertGreater(got, 0.0,
                               f"選ばれた経路が障害物に食い込んでいる: {got * 100:.1f}cm")

    def test_far_target_still_sees_the_walls(self):
        """★**目標が遠くても壁を見落とさないこと。**

        2026-09-17、バンビの画面報告（通路の壁が点群として写っているのに
        経路が貫通する）で見つけた不具合の回帰テスト。原因は2つの連鎖:

        1. 局所地図へ取り込む距離が`obstacle_max_range`（安全窓用の既定
           2.0m）を流用していたため、**2mより遠い壁が地図に入らなかった**
        2. 地図が原点中心の6m固定だったため、3m先の目標が外周（塞いである）
           に乗り、Hybrid A*が「目標が障害物と重なる」で即失敗して
           RS候補フォールバックへ落ちていた

        どちらも「壁を知らないまま経路を引く」に帰着する。
        """
        half_w = 0.75                        # 幅1.5mの通路
        scan = Scan(dist=[0.0] * 360, sector_seen=[True] * 12)
        for deg in range(360):
            a = math.radians(deg)
            sa = math.sin(a)
            if abs(sa) < 1e-6:
                continue
            d = half_w / abs(sa)
            if d <= 5.5:
                scan.dist[deg] = d

        pp = ParkToPoint()
        p = pp.merged({})
        pp.request_park_target(3.0, 0.1, math.radians(100))
        st = _warm(pp, scan, VehicleState(odom_center=0.0, yaw_rate=0.0), p)
        self.assertGreater(pp._map.size_m, 3.0 + 2.0,
                           "目標距離に対して地図が狭い（目標が外周に乗る）")
        self.assertTrue(pp._map.contains(3.0, 0.1, pad=0.3),
                        "目標が地図の内側に入っていない")
        # 3m先の壁が地図に入っていること
        self.assertLess(float(pp._map.clearance(2.8, 0.0)), half_w + 0.05)
        self.assertGreater(float(pp._map.clearance(2.8, 0.0)), 0.3)
        self.assertTrue(pp._path.segments, st.reason)
        poses = sample_path_array(pp._park_pose, pp._path, step=0.02)
        # 真値: 通路の壁に対する車体4隅の最小余裕
        worst = min(half_w - abs(y + cx * math.sin(yaw) + cy * math.cos(yaw))
                    for x, y, yaw in poses
                    for cx in (-0.07, 0.30) for cy in (-0.09, 0.09))
        self.assertGreater(worst, 0.0, f"経路が通路の壁を貫通している（{worst * 100:.1f}cm）")

    def test_converges_around_obstacle(self):
        """障害物があっても（別候補を選んで）最終的に到達すること。"""
        pp = ParkToPoint()
        pp.request_park_target(0.0, 0.5, 0.0)
        scan = Scan(dist=[5.0] * 360, sector_seen=[True] * 12)
        for d in range(58, 63):
            scan.dist[d % 360] = 0.35

        st = None
        vs = VehicleState(odom_center=0.0, yaw_rate=0.0)
        wheelbase = pp.vehicle.wheelbase
        p = pp.merged(IDEAL)          # 合成スキャンなのでアンカは切る（上記）
        for _ in range(800):
            st = pp.plan(scan, vs, p, DT)
            if st.phase in ("完了", "失敗"):
                break
            v = 0.0 if st.brake else st.target_speed
            yaw_rate = v / wheelbase * math.tan(st.target_steer) if wheelbase else 0.0
            vs = VehicleState(odom_center=vs.odom_center + v * DT, yaw_rate=yaw_rate)
        self.assertEqual(st.phase, "完了")

    def test_fully_blocked_brakes(self):
        """進路が完全にふさがれている場合は停止すること（安全側フォールバック）。"""
        pp = ParkToPoint()
        pp.request_park_target(1.0, 0.0, 0.0)
        scan = Scan(dist=[5.0] * 360, sector_seen=[True] * 12)
        for d in range(-90, 91):            # 前方半周をまるごとふさぐ
            scan.dist[d % 360] = 0.2
        for _ in range(3):
            st = pp.plan(scan, VehicleState(odom_center=0.0, yaw_rate=0.0),
                         pp.merged({}), DT)
        self.assertTrue(st.brake)


if __name__ == "__main__":
    unittest.main()
