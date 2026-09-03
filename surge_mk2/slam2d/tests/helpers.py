"""テスト用の合成シーン生成。`raspi/tests/test_nav.py`の`make_room_scan`と同じ流儀。

**ハードウェアもセンサ実装も要らない。** 合成した部屋にレイキャストして点群を
作るだけなので、地図と姿勢を数値で答え合わせできる。
"""

from __future__ import annotations

import math

import numpy as np

from slam2d.core.types import RawScan, ScanPoints, wrap_angle

#: 部屋の壁（軸に平行な線分の集まり）。**左右非対称にしてある** — 正方形だと
#: 90°回転しても同じ形になり、スキャンマッチが別解に落ちても気づけない
ROOM = [
    (0.0, 0.0, 6.0, 0.0),      # 下
    (6.0, 0.0, 6.0, 4.0),      # 右
    (6.0, 4.0, 0.0, 4.0),      # 上
    (0.0, 4.0, 0.0, 0.0),      # 左
    (1.5, 1.0, 2.5, 1.0),      # 中の板（対称性を壊すためだけに置いてある）
    (2.5, 1.0, 2.5, 2.2),
]


def ray_segments(ox: float, oy: float, ang: float, segs, max_range: float) -> float:
    """線分の集まりに対するレイキャスト。当たらなければ0.0。"""
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


def make_oval_track(*, straight: float = 4.0, radius: float = 2.0,
                    width: float = 1.0, arc_segments: int = 24) -> list[tuple[float, float, float, float]]:
    """直線2本+半円2つからなる、角の無いoval形状の内壁・外壁を線分リストで返す。

    中心線は原点を中心に、x軸方向`straight`の直線区間の両端に半径`radius`の
    半円が付く閉じたループ。`oval_centerline_pose()`と整合する形状・向きで
    生成する（下の直線は+x向き、右の半円は反時計回り、という規約）。
    """
    outer_r = radius + width / 2.0
    inner_r = radius - width / 2.0
    half = straight / 2.0

    def arc_points(cx, cy, r, a0, a1, n):
        return [(cx + r * math.cos(a0 + (a1 - a0) * i / n),
                cy + r * math.sin(a0 + (a1 - a0) * i / n)) for i in range(n + 1)]

    def loop_points(r):
        pts = [(-half, -r), (half, -r)]
        pts += arc_points(half, 0.0, r, -math.pi / 2, math.pi / 2, arc_segments)[1:]
        pts.append((-half, r))
        pts += arc_points(-half, 0.0, r, math.pi / 2, 1.5 * math.pi, arc_segments)[1:]
        return pts

    def to_segs(pts):
        n = len(pts)
        return [(pts[i][0], pts[i][1], pts[(i + 1) % n][0], pts[(i + 1) % n][1])
               for i in range(n)]

    return to_segs(loop_points(outer_r)) + to_segs(loop_points(inner_r))


def oval_centerline_length(*, straight: float = 4.0, radius: float = 2.0) -> float:
    return 2.0 * straight + 2.0 * math.pi * radius


def oval_centerline_pose(s: float, *, straight: float = 4.0,
                         radius: float = 2.0) -> tuple[float, float, float]:
    """oval中心線上、弧長`s`[m]における姿勢(x, y, yaw)。

    区間の並び: 下の直線(+x向き) → 右の半円(反時計回り) → 上の直線(-x向き)
    → 左の半円(反時計回り) → (下の直線に戻る)。`make_oval_track()`と同じ規約。
    """
    half = straight / 2.0
    seg_straight = straight
    seg_arc = math.pi * radius
    total = 2.0 * seg_straight + 2.0 * seg_arc
    s = s % total

    if s < seg_straight:
        return (-half + s, -radius, 0.0)
    s -= seg_straight
    if s < seg_arc:
        ang = -math.pi / 2 + s / radius
        return (half + radius * math.cos(ang), radius * math.sin(ang), ang + math.pi / 2)
    s -= seg_arc
    if s < seg_straight:
        return (half - s, radius, math.pi)
    s -= seg_straight
    ang = math.pi / 2 + s / radius
    yaw = ang + math.pi / 2
    return (-half + radius * math.cos(ang), radius * math.sin(ang), yaw)


def oval_centerline_twist(s: float, v: float, *, straight: float = 4.0,
                          radius: float = 2.0) -> tuple[float, float]:
    """oval中心線に沿って弧長速度`v`[m/s]で進むときの(vx, yaw_rate)。"""
    seg_straight = straight
    seg_arc = math.pi * radius
    total = 2.0 * seg_straight + 2.0 * seg_arc
    s = s % total
    on_straight = s < seg_straight or (seg_straight + seg_arc <= s < 2 * seg_straight + seg_arc)
    yaw_rate = 0.0 if on_straight else v / radius
    return v, yaw_rate


def make_room_points(x: float, y: float, yaw: float, *, n_angles: int = 360,
                     segs=ROOM, max_range: float = 8.0,
                     t_ref_ns: int = 1_000_000_000,
                     noise_sigma: float = 0.0,
                     rng: np.random.Generator | None = None) -> ScanPoints:
    """姿勢`(x, y, yaw)`から部屋`segs`を見た点群を、既に脱スキュー済みの体で作る。

    `slam2d.core`は脱スキューをテストする`test_deskew.py`とは別に、grid/scanmatch
    単体は「既に共通フレームへ変換された点群」を入力に取るので、ここでは
    センサ座標→base_link変換や時刻処理を省いて直接`ScanPoints`を作る。
    """
    angles = np.linspace(0.0, 2.0 * math.pi, n_angles, endpoint=False)
    ranges = np.array([ray_segments(x, y, yaw + a, segs, max_range) for a in angles])
    valid = ranges > 0.0
    if noise_sigma > 0.0 and rng is not None:
        ranges = np.where(valid, ranges + rng.normal(0.0, noise_sigma, ranges.shape), ranges)

    # レイキャスト原点(x,y,yaw)から見た「センサ座標」の点を作り、姿勢の逆変換で
    # base_link相当（センサ座標系そのもの、ここではmount offset無し）へ戻す
    px = ranges[valid] * np.cos(angles[valid])
    py = ranges[valid] * np.sin(angles[valid])
    hit = np.ones(px.shape, dtype=bool)
    return ScanPoints(px, py, hit, t_ref_ns, False)


def make_out_and_back_path(*, start=(3.0, 2.0, 0.0), v: float = 0.1, dt: float = 0.1,
                          out_steps: int = 20, turn_steps: int = 16):
    """前進→その場でUターン(180度)→前進、で出発点付近に戻る経路を作る。

    `(pose, twist)`のペアのリストを返す。各ステップの`twist`は、その区間の
    真の速度・ヨーレートと一致させてある——`ExternalTwistModel`に真値を
    そのまま渡すテスト（ループ検出・グラフ最適化の統合テスト）で、
    推測航法と真の軌道が整合しないと`Frontend`が意図通りに動かないため。
    """
    x, y, yaw = start
    steps = []
    for _ in range(out_steps):
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        steps.append(((x, y, yaw), (v, 0.0, 0.0)))
    turn_rate = math.pi / (turn_steps * dt)
    for _ in range(turn_steps):
        yaw += turn_rate * dt
        steps.append(((x, y, yaw), (0.0, 0.0, turn_rate)))
    for _ in range(out_steps):
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        steps.append(((x, y, yaw), (v, 0.0, 0.0)))
    return steps


def make_raw_scan(x: float, y: float, yaw: float, *, n_angles: int = 360,
                  segs=ROOM, max_range: float = 8.0,
                  t_point_ns: int = 1_000_000_000,
                  noise_sigma: float = 0.0,
                  rng: np.random.Generator | None = None) -> RawScan:
    """姿勢`(x, y, yaw)`から`segs`を見た`RawScan`。全点同一時刻（脱スキュー
    不要な「瞬間スキャン」）として作る——`core/frontend.py`の統合テスト用に、
    センサ座標系の生データからパイプライン全体（脱スキューを含む）を通す。
    """
    angles = np.linspace(0.0, 2.0 * math.pi, n_angles, endpoint=False)
    ranges = np.array([ray_segments(x, y, yaw + a, segs, max_range) for a in angles])
    valid = ranges > 0.0
    if noise_sigma > 0.0 and rng is not None:
        ranges = np.where(valid, ranges + rng.normal(0.0, noise_sigma, ranges.shape), ranges)
    saturated = np.zeros(n_angles, dtype=bool)
    t_point_ns_arr = np.full(n_angles, t_point_ns, dtype=np.int64)
    return RawScan(angles, ranges, valid, saturated, t_point_ns_arr)


def make_track_from_centerline(centerline: np.ndarray,
                               width: float) -> list[tuple[float, float, float, float]]:
    """任意の閉じた中心線`(N,3)`(x, y, yaw)から、内壁・外壁の線分列を作る。

    `make_oval_track()`と同じ発想（中心線を法線方向に`±width/2`だけオフセット
    する）を、閉じた形状であれば任意の形（`sim/random_course.py`の procedural
    コース等）に一般化したもの。中心線が十分細かく再標本化されている前提
    （`sim.random_course._build_loop`は`final_step`間隔で`resample_loop`済み）
    なので、点ごとの`yaw`をそのまま法線方向に使い、区間ごとの円弧近似はしない。
    """
    xs, ys, yaws = centerline[:, 0], centerline[:, 1], centerline[:, 2]
    nx, ny = -np.sin(yaws), np.cos(yaws)

    def to_segs(ox: np.ndarray, oy: np.ndarray) -> list[tuple[float, float, float, float]]:
        n = len(ox)
        return [(float(ox[i]), float(oy[i]), float(ox[(i + 1) % n]), float(oy[(i + 1) % n]))
               for i in range(n)]

    half = width / 2.0
    outer = to_segs(xs + half * nx, ys + half * ny)
    inner = to_segs(xs - half * nx, ys - half * ny)
    return outer + inner


def centerline_arclengths(centerline: np.ndarray) -> np.ndarray:
    """`centerline`の各点までの累積弧長[m]（先頭は0.0）。周を閉じる最終区間は含まない。"""
    seg = np.hypot(*np.diff(centerline[:, :2], axis=0).T)
    return np.concatenate([[0.0], np.cumsum(seg)])


def centerline_loop_length(centerline: np.ndarray, arc: np.ndarray | None = None) -> float:
    """1周ぶんの弧長[m]（最後の点から最初の点に戻る区間も含む）。"""
    if arc is None:
        arc = centerline_arclengths(centerline)
    close = math.hypot(centerline[0, 0] - centerline[-1, 0], centerline[0, 1] - centerline[-1, 1])
    return float(arc[-1] + close)


def centerline_pose_at(centerline: np.ndarray, arc: np.ndarray, total_len: float,
                       s: float) -> tuple[float, float, float]:
    """中心線上、弧長`s`[m]（1周を超えたら周回）における姿勢(x, y, yaw)。

    `oval_centerline_pose()`の一般化版。区間内は線形補間（`yaw`は`wrap_angle`
    で最短方向に補間）する——`final_step`間隔（既定0.1m）で十分細かいので、
    円弧の厳密解の代わりに線形補間で足りる
    """
    s = s % total_len
    n = len(centerline)
    idx = int(np.searchsorted(arc, s, side="right")) - 1
    idx = max(0, min(idx, n - 1))
    nxt = (idx + 1) % n
    seg_len = (arc[nxt] if nxt != 0 else total_len) - arc[idx]
    t = 0.0 if seg_len <= 1e-9 else (s - arc[idx]) / seg_len
    x = centerline[idx, 0] + t * (centerline[nxt, 0] - centerline[idx, 0])
    y = centerline[idx, 1] + t * (centerline[nxt, 1] - centerline[idx, 1])
    dyaw = wrap_angle(centerline[nxt, 2] - centerline[idx, 2])
    yaw = wrap_angle(centerline[idx, 2] + t * dyaw)
    return float(x), float(y), float(yaw)


def centerline_twist_at(centerline: np.ndarray, arc: np.ndarray, total_len: float,
                        s: float, v: float) -> tuple[float, float]:
    """弧長速度`v`[m/s]で中心線を進むときの(vx, yaw_rate)。

    `oval_centerline_twist()`の一般化版。区間の向き変化量を、その区間を
    速度`v`で通過する所要時間で割って角速度とする（区間内は等角速度の近似）。
    """
    s = s % total_len
    n = len(centerline)
    idx = int(np.searchsorted(arc, s, side="right")) - 1
    idx = max(0, min(idx, n - 1))
    nxt = (idx + 1) % n
    seg_len = (arc[nxt] if nxt != 0 else total_len) - arc[idx]
    if seg_len <= 1e-9 or v <= 0.0:
        return v, 0.0
    dyaw = wrap_angle(centerline[nxt, 2] - centerline[idx, 2])
    dt = seg_len / v
    return v, dyaw / dt
