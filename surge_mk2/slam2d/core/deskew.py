"""点群のモーションスキュー補正 — 走りながら測った1周を「1瞬の点群」に直す。

`raspi/nav/deskew.py`から移植・一般化。回転式LiDARは1周が瞬時ではないため、
車体が動いていると点群が扇形に歪む。歪んだまま地図に焼くと壁が二重になり、
スキャンマッチはその二重の壁に対して「どちらにも合わない」姿勢を返す。

## 車体依存の値を数値としてだけ受け取る

補正に要る「1周のあいだにどれだけ動いたか」は`Twist2D`として**数値で**
受け取る。どのセンサ由来か（ジャイロ+速度、車輪速+IMU、等速度モデルでの
代用等）はここでは関知しない（`core/motion.py`の`MotionModel`が決める）。

## 1周期のあいだ twist が一定とは限らない

`deskew()`は「この周のあいだ twist は一定」と仮定する。直線やゆるい旋回なら
それでよいが、**ヘアピンを高速で抜ける場面では破綻する**——1周100msのあいだに
ヨーレートが0から2.2rad/sまで立ち上がると、一定と見なした補正は最大で数度ぶん
外れ、3m先の点が30cmずれる（`sim/slam_bench.py` course3、2m/sで残差RMSが
10〜15cmに達し、位置合わせが誤った姿勢に貼り付いた）。

`deskew_traj()`は**ジャイロ・車速の時系列**（`TwistBuffer`）から各点の時刻の
姿勢を作って補正する。サンプル間（実機は50Hz＝20ms）は姿勢を線形補間する
（2.2rad/sでも区間内の弦と弧の差は0.1mm未満）。

## 3種類の点を区別する

| 入力 | 出力 | なぜ |
|---|---|---|
| 実測点 | `hit=True`。終端に壁を打ち、手前を空きとして彫る | |
| 飽和点（測距上限） | `hit=False`。手前を空きとして彫るだけ | 「ちょうど測距上限」ではない。壁として打つと実在しない壁ができるが、そこまでは何も無いという情報は捨てない |
| 無効点（`valid=False`） | 捨てる（1点も出さない） | 知らないことは書かない。無効を「空き」として彫ると、後からそこに壁があると分かってもヒット/ミス比が汚れたまま残る |
"""

from __future__ import annotations

import numpy as np

import math

from .types import Pose2D, RawScan, ScanPoints, Twist2D, integrate_twist

__all__ = ["deskew", "deskew_traj", "truncate", "TwistBuffer"]

NS = 1_000_000_000
#: これ以下のヨーレート・速度は「動いていない」として扱う
_STRAIGHT_EPS = 1e-4
_STILL_SPEED_EPS = 1e-3


def deskew(raw: RawScan, twist: Twist2D, *,
           mount_x: float = 0.0, mount_y: float = 0.0,
           max_range: float = float("inf")) -> ScanPoints:
    """1周ぶんの点群を`t_ref_ns`（点群中でいちばん新しい時刻）へ揃える。

    :param twist: この周のあいだ一定だったと仮定する、車体座標系での速度
    :param mount_x: センサの取付位置オフセット[m]（車体前方+）
    :param mount_y: 同、車体左方+
    :param max_range: これより遠い点は「この距離まで空き」として切り詰める[m]
    """
    valid = raw.valid
    idx = np.flatnonzero(valid)
    if idx.size == 0:
        empty = np.empty(0, dtype=np.float64)
        return ScanPoints(empty, empty, np.empty(0, dtype=bool), 0, False)

    ang = raw.angles[idx]
    r = raw.ranges[idx]
    sat = raw.saturated[idx]
    over = ~np.isfinite(r) | (r > max_range)
    rng = np.where(over, max_range, r)
    hit = ~sat & ~over

    # センサ座標 → 車体座標（取付は yaw=0 前提）
    px = mount_x + rng * np.cos(ang)
    py = mount_y + rng * np.sin(ang)

    t_pt = raw.t_point_ns[idx]
    t_ref = int(t_pt.max()) if t_pt.size and t_pt.max() > 0 else 0

    v, vy, w = twist.vx, twist.vy, twist.yaw_rate
    # 時刻が無い（合成した点群等）か、そもそも動いていないなら補正しない。
    # **動いていないのに補正すると、速度のノイズぶんだけ点群を汚す**
    movable = (t_pt.min() > 0
               and (abs(v) > _STILL_SPEED_EPS or abs(vy) > _STILL_SPEED_EPS
                    or abs(w) > _STRAIGHT_EPS))
    if not movable:
        return ScanPoints(px, py, hit, t_ref, False)

    # 各点を測ってから基準時刻までに車体が動いた量。tau >= 0（過去 → 基準）
    tau = (t_ref - t_pt).astype(np.float64) / NS

    if abs(w) > _STRAIGHT_EPS:
        wt = w * tau
        s, c = np.sin(wt), np.cos(wt)
        dx = (v * s + vy * (c - 1.0)) / w
        dy = (v * (1.0 - c) + vy * s) / w
    else:
        wt = np.zeros_like(tau)
        dx = v * tau
        dy = vy * tau

    # (dx, dy, wt) は「点を測った時刻の車体から見た、基準時刻の車体の姿勢」。
    # 点を基準時刻の座標へ移すのはその**逆変換**（`core/types.py`の`inverse`と同じ式）
    c, s = np.cos(wt), np.sin(wt)
    ox, oy = px - dx, py - dy
    return ScanPoints(ox * c + oy * s, -ox * s + oy * c, hit, t_ref, True)


def truncate(pts: ScanPoints, r: float) -> ScanPoints:
    """`r`[m]より遠い点を「そこまでは空き」に切り詰める。**地図に焼く用。**

    照合には全点を使うが、地図に焼くのは近い点だけにする。測距誤差は距離が
    伸びるほど大きくなるセンサが多く、遠い壁は点線状の太い滲みとして焼かれ、
    次の周の照合を狂わせる悪循環になりうる。近い点だけで焼いても地図は
    埋まる（同じ壁を、そこへ近づいたときに焼けばよい）。
    """
    if r <= 0 or len(pts) == 0:
        return pts
    d = np.hypot(pts.x, pts.y)
    far = d > r
    if not far.any():
        return pts
    scale = np.where(far, r / np.maximum(d, 1e-9), 1.0)
    return ScanPoints(pts.x * scale, pts.y * scale, pts.hit & ~far,
                      pts.t_ref_ns, pts.corrected)


class TwistBuffer:
    """時刻つきの twist の履歴。**`deskew_traj()`と推測航法の予測が共有する。**

    実機の`VehicleState`（ジャイロ+車速、50Hz）を届いた順に`add()`する。
    保持するのは直近`window_s`秒ぶんだけ（1周の点群は最大でも1回転ぶん過去）。
    """

    def __init__(self, *, window_s: float = 0.6, max_samples: int = 256) -> None:
        self._window_ns = int(window_s * NS)
        self._max = max_samples
        self.t: list[int] = []
        self.vx: list[float] = []
        self.vy: list[float] = []
        self.w: list[float] = []

    def __len__(self) -> int:
        return len(self.t)

    def clear(self) -> None:
        self.t.clear(); self.vx.clear(); self.vy.clear(); self.w.clear()

    def add(self, t_ns: int, twist: Twist2D) -> None:
        """**時刻が戻るサンプルは捨てる**（同じ値の再送・時刻の巻き戻り対策）。"""
        t_ns = int(t_ns)
        if t_ns <= 0 or (self.t and t_ns <= self.t[-1]):
            return
        self.t.append(t_ns)
        self.vx.append(float(twist.vx))
        self.vy.append(float(twist.vy))
        self.w.append(float(twist.yaw_rate))
        cut = t_ns - self._window_ns
        drop = 0
        while drop < len(self.t) - 2 and (self.t[drop] < cut or len(self.t) - drop > self._max):
            drop += 1
        if drop:
            del self.t[:drop], self.vx[:drop], self.vy[:drop], self.w[:drop]

    def covers(self, t0_ns: int, t1_ns: int, *, max_extrap_s: float = 0.06) -> bool:
        """`[t0, t1]`をだいたい挟んでいるか（端の外挿が`max_extrap_s`以内か）。

        テレメトリ（50Hz）は点群より少し古いところまでしか無いのが普通なので、
        **端は最後のサンプルの twist で外挿する**ことを前提に、その外挿量が
        小さいことだけを確かめる。
        """
        if len(self.t) < 2:
            return False
        pad = int(max_extrap_s * NS)
        return self.t[0] <= t0_ns + pad and self.t[-1] >= t1_ns - pad

    def _samples(self, t_lo: int, t_hi: int):
        """`[t_lo, t_hi]`を挟むように、端を一定 twist で外挿したサンプル列。"""
        t = list(self.t); vx = list(self.vx); vy = list(self.vy); w = list(self.w)
        if t_lo < t[0]:
            t.insert(0, t_lo); vx.insert(0, vx[0]); vy.insert(0, vy[0]); w.insert(0, w[0])
        if t_hi > t[-1]:
            t.append(t_hi); vx.append(vx[-1]); vy.append(vy[-1]); w.append(w[-1])
        return (np.asarray(t, dtype=np.int64), np.asarray(vx), np.asarray(vy), np.asarray(w))

    def poses(self, t_ref_ns: int, *, t_lo: int | None = None, t_hi: int | None = None):
        """サンプル時刻ごとの、`t_ref_ns`時点の車体から見た姿勢 (t, x, y, yaw)。

        `t_ref`での姿勢が原点。`t < t_ref`の姿勢は「そのとき車体はここに居た
        （t_ref の車体座標で）」を表す。`t_lo`/`t_hi`を与えると、その範囲を
        挟むように端を一定 twist で外挿してから積む。
        """
        lo = min(t_ref_ns, t_lo if t_lo is not None else t_ref_ns)
        hi = max(t_ref_ns, t_hi if t_hi is not None else t_ref_ns)
        t, vxs, vys, ws = self._samples(lo, hi)
        n = t.size
        x = np.zeros(n); y = np.zeros(n); yaw = np.zeros(n)
        # 前向きに積み、最後に t_ref 時点の姿勢を引いて基準を移す
        for i in range(1, n):
            dt = (t[i] - t[i - 1]) / NS
            d = integrate_twist(Twist2D(float(vxs[i - 1]), float(vys[i - 1]), float(ws[i - 1])), dt)
            c, s = math.cos(yaw[i - 1]), math.sin(yaw[i - 1])
            x[i] = x[i - 1] + c * d.x - s * d.y
            y[i] = y[i - 1] + s * d.x + c * d.y
            yaw[i] = yaw[i - 1] + d.yaw
        rx, ry, ryaw = _interp_pose(t, x, y, yaw, np.array([t_ref_ns], dtype=np.int64))
        # 基準（t_ref の姿勢）の逆変換を掛ける
        c, s = math.cos(-ryaw[0]), math.sin(-ryaw[0])
        dx, dy = x - rx[0], y - ry[0]
        return t, c * dx - s * dy, s * dx + c * dy, yaw - ryaw[0]

    def delta(self, t0_ns: int, t1_ns: int) -> Pose2D:
        """`t0`の車体から見た`t1`の姿勢（推測航法の1周期ぶんの予測）。"""
        t, x, y, yaw = self.poses(t1_ns, t_lo=min(t0_ns, t1_ns), t_hi=max(t0_ns, t1_ns))
        px, py, pyaw = _interp_pose(t, x, y, yaw, np.array([t0_ns], dtype=np.int64))
        # t1 の姿勢は原点なので、t0 から見た t1 は t0 の姿勢の逆変換
        c, s = math.cos(-pyaw[0]), math.sin(-pyaw[0])
        return Pose2D(float(-(c * px[0] - s * py[0])), float(-(s * px[0] + c * py[0])),
                      float(-pyaw[0]))

    def mean_twist(self, t0_ns: int, t1_ns: int) -> Twist2D:
        """`[t0, t1]`の平均的な twist（脱スキューのフォールバック・診断用）。"""
        dt = max((t1_ns - t0_ns) / NS, 1e-6)
        d = self.delta(t0_ns, t1_ns)
        return Twist2D(d.x / dt, d.y / dt, d.yaw / dt)


def _interp_pose(t: np.ndarray, x: np.ndarray, y: np.ndarray, yaw: np.ndarray,
                 tq: np.ndarray):
    """サンプル姿勢を時刻`tq`へ線形補間する（範囲外は端の値で止める）。"""
    tf = t.astype(np.float64)
    q = np.clip(tq.astype(np.float64), tf[0], tf[-1])
    return (np.interp(q, tf, x), np.interp(q, tf, y), np.interp(q, tf, yaw))


def deskew_traj(raw: RawScan, buf: TwistBuffer, *, mount_x: float = 0.0, mount_y: float = 0.0,
                max_range: float = float("inf")) -> ScanPoints:
    """点ごとの時刻の姿勢（`TwistBuffer`）で脱スキューする。

    サンプルが足りない（点群の時刻を挟んでいない）場合は、その区間の平均 twist を
    使った`deskew()`と同じ扱いになる（`_interp_pose`が端で止まるため）。
    """
    valid = raw.valid
    idx = np.flatnonzero(valid)
    if idx.size == 0 or len(buf) < 2:
        empty = np.empty(0, dtype=np.float64)
        if idx.size == 0:
            return ScanPoints(empty, empty, np.empty(0, dtype=bool), 0, False)
        return deskew(raw, Twist2D(0.0, 0.0, 0.0), mount_x=mount_x, mount_y=mount_y,
                      max_range=max_range)

    ang = raw.angles[idx]
    r = raw.ranges[idx]
    sat = raw.saturated[idx]
    over = ~np.isfinite(r) | (r > max_range)
    rng = np.where(over, max_range, r)
    hit = ~sat & ~over
    px = mount_x + rng * np.cos(ang)
    py = mount_y + rng * np.sin(ang)

    t_pt = raw.t_point_ns[idx]
    t_ref = int(t_pt.max()) if t_pt.size and t_pt.max() > 0 else 0
    if t_ref <= 0 or not (t_pt > 0).any():
        return deskew(raw, Twist2D(0.0, 0.0, 0.0), mount_x=mount_x, mount_y=mount_y,
                      max_range=max_range)

    ts, sx, sy, syaw = buf.poses(t_ref, t_lo=int(t_pt[t_pt > 0].min()), t_hi=t_ref)
    qx, qy, qyaw = _interp_pose(ts, sx, sy, syaw, t_pt)
    # (qx, qy, qyaw) は「点を測った時刻の車体」の、t_ref の車体座標での姿勢。
    # 点はその車体の座標で測られているので、合成すれば t_ref の座標へ移る
    c, s = np.cos(qyaw), np.sin(qyaw)
    return ScanPoints(qx + c * px - s * py, qy + s * px + c * py, hit, t_ref, True)
