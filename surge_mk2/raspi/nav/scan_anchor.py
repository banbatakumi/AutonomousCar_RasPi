"""スキャンアンカ — 基準スキャン**1枚**に対する2フレーム登録で、累積しない姿勢を保つ。

## 何を解くのか

`park_to_point`は目標をデッドレコニングだけで追っていた。これには原理的な
問題がある: ユーザーが壁際をクリックしたときの意図は「**あの壁に対して**この
位置」なので、推定がドリフトすると**駐車枠そのものが実世界の壁に対して静かに
動き**、plannerは誤った場所で「完了」と言う。誤差が誤差として現れず、成功と
して現れる——最悪の壊れ方。

これを地図なしで解く。**クリックした瞬間のスキャンを基準として1枚だけ保持し、
毎周期「現在スキャン ↔ 基準スキャン」を直接合わせる。** 目標はその基準
フレームで定義されているので、登録結果がそのまま「今の車体から見た目標姿勢」
になる。

**ドリフトしないのは、常に固定の1枚に対して合わせるから**（逐次合成をしない）。
地図・キーフレーム列・ポーズグラフ・ループ閉じのどれも要らない。

## 姿勢は「目標を定義した瞬間のフレーム」で返す

内部ではキーフレームを進めるが、`chain`（原点フレームから見た直近
キーフレームの姿勢）を持っているので、**外から見えるフレームは最初から
最後まで変わらない**。目標が定義されているのはそのフレームなので、
呼び出し側が座標変換を持ち回る必要が無い。

毎周期: 直近キーフレームに合わせ、`chain`で原点フレームへ移す。
直近キーフレームから`reanchor_dist`/`reanchor_yaw`以上離れたらキーフレームを
進める（1リンクあたりの相対移動を小さく保つのが精度の条件）。

## ★ 原点キーフレームへ直接合わせる案は不採用（2026-09-16、実測）

「まず原点キーフレームに合わせてみて、通れば誤差が一切累積しない」という
2段構えを実装して測ったが、**成功率が 13/24 → 6/24 に悪化した**。長い
基線（1m並進＋90°回頭）では誤対応が「残差は小さいが姿勢は間違っている」
解を作り、`inlier`/`residual`/`max_correction`の判定を素通りしてしまう。
**残差が小さいことは正しさの証拠にならない**——別の壁にきれいに合って
しまえば残差は小さい。判定を厳しくしても通り抜けるものは通り抜けるため、
この経路自体を持たないことにした。

## ★ 基準1枚では通せない（2026-09-16、実測で判明）

当初は「LD06は360°なので90°回頭しても視野は失われず、基準1枚で駐車動作を
通せる」と考えて実装した。**これは誤りだった。** 視野は失われないが、

- 1m並進すると**遮蔽構造が大きく変わる**（車庫の内側は入ってから初めて見える）
- 視差が`scan2scan.MAX_PAIR_M`（対応点として認める距離、30cm）に対して大きく、
  **誤った対応が大量に付く**（別の壁の別の場所と組んでしまう）

シム実測では、基準1枚方式を有効にすると車庫入れの成功率が 4/6 → 1/6、
最終位置誤差が 2.9cm → 50.6cm に**悪化**した（純デッドレコニングの方が良い）。

そこで**キーフレーム連鎖**に改めた: 基準から`reanchor_dist`/`reanchor_yaw`
以上離れたら、その時点のスキャンを新しい基準にして変換を合成する。
1リンクあたりの相対移動が小さいのでPLICPが本来の精度で効く。誤差はリンクを
またぐぶんだけ累積するが、**ジャイロのバイアスのように時間で積み上がるのとは
性質が違う**（走行距離に比例し、かつ壁が拘束する方向には溜まらない）。

## 観測できない方向は自動的にデッドレコニングへ退避する

`scan2scan.match_scans()`はティホノフ正則化で推測航法の初期値へ引き戻す項を
持つ（同モジュールdocstringの実測経緯参照——正則化なしでovalが11.6cm→84.8cm
に悪化した）。したがって

- 壁が拘束する方向 … データの情報量が大きいのでICPが勝つ
- 拘束の無い方向  … 情報量がほぼ0なので初期値（推測航法）が残る

が自動的に成り立つ。**広い駐車場で壁が1枚しか見えなければ、その法線方向だけ
補正され、他はデッドレコニングのまま**。これは幾何の事実に沿った正しい劣化で、
「特徴が乏しいと壊れる」のではなく「観測できるぶんだけ効く」。

## 登録できないとき

`inlier`が閾値を割り続けたら（遮蔽・大きく動いた等）、今のスキャンを
キーフレームにして繋ぎ直す。**このときだけ誤差が1回ぶん乗る**（毎周期は
乗らない）。呼び出し側の座標系は変わらない（`chain`が内部で吸収する）。

## 暴走の防ぎ方

ICPが破綻して飛んだ値を返す場合に備え、**推測航法の初期値から`max_correction`
以上離れた解は捨てる**。毎周期の初期値は「前回の補正済み姿勢＋今周期の
デッドレコニング差分」なので、正常時の補正量は数mm〜数cmに収まる。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .deskew import Points
from .scan2scan import MIN_POINTS, Delta, match_scans
from .se2 import Pose, between, compose

__all__ = ["ScanAnchor", "AnchorUpdate"]


@dataclass
class AnchorUpdate:
    """1周期ぶんの結果。`pose`は**原点キーフレーム**での車体姿勢。"""

    pose: Pose
    #: 登録が採用されたか。Falseなら`pose`は渡された推測航法の値そのまま
    ok: bool
    inlier: float
    residual: float
    #: 補正量 [m]（推測航法の初期値との差）。診断・GUI表示用
    correction_m: float


class ScanAnchor:
    """基準スキャン1枚に対する2フレーム登録。**地図を作らない。**

    :param min_inlier: 対応が付いた点の割合がこれ未満なら採用しない
    :param max_residual: 残差の中央値 [m] がこれを超えたら採用しない
    :param max_correction: 推測航法の初期値からこれ [m] 以上離れた解は捨てる
    :param reanchor_after: 連続でこの回数採用できなければキーフレームを繋ぎ直す
    :param reanchor_dist: 基準からこれ [m] 以上離れたら基準を張り替える。
        **大きくすると誤対応が増えて精度が落ちる**（上の実測参照）
    :param reanchor_yaw: 同 [rad]
    """

    def __init__(self, *, min_inlier: float = 0.60, max_residual: float = 0.04,
                 max_correction: float = 0.20, reanchor_after: int = 3,
                 reanchor_dist: float = 0.20,
                 reanchor_yaw: float = math.radians(15.0),
                 iters: int = 16, max_pair: float = 0.08) -> None:
        self.min_inlier = float(min_inlier)
        self.max_residual = float(max_residual)
        self.max_correction = float(max_correction)
        self.reanchor_after = int(reanchor_after)
        self.reanchor_dist = float(reanchor_dist)
        self.reanchor_yaw = float(reanchor_yaw)
        #: ICPの反復回数。**既定16**（`match_scans`の既定8より多い）
        self.iters = int(iters)
        #: 対応点として認める距離 [m]。★**ここが推定精度を支配する。**
        #: `scan2scan.MAX_PAIR_M`の既定0.30は「1周期ぶんの相対移動（10Hz・
        #: 1m/sで10cm）」を想定した値だが、駐車は0.3m/s・キーフレーム間
        #: 0.20m以下なので広すぎ、**別の壁と組む誤対応を大量に通していた**。
        #: 掃引実測（2026-09-17、センサ誤差あり12件）の推定誤差:
        #: 0.30→3.65cm / 0.18→3.14cm / 0.12→2.63cm / **0.08→0.96cm**
        self.max_pair = float(max_pair)
        self.reset()

    def reset(self) -> None:
        #: 直近キーフレームと、その「原点フレーム（目標を定義した瞬間の
        #: 車体）での姿勢」。**外から見えるフレームは原点で固定**
        self._kf: Points | None = None
        self._chain: Pose = (0.0, 0.0, 0.0)
        self._miss = 0
        #: 診断用の累計
        self.updates = 0
        self.accepted = 0
        self.keyframes = 0

    @property
    def has_reference(self) -> bool:
        return self._kf is not None

    def set_reference(self, pts: Points) -> None:
        """原点キーフレームを張る。`pts`は脱スキュー済み・base_link座標。"""
        self._kf = pts
        self._chain = (0.0, 0.0, 0.0)
        self._miss = 0

    def update(self, pts: Points, dr_pose: Pose) -> AnchorUpdate:
        """現在スキャンを基準に合わせ、**原点フレームでの**姿勢を返す。

        :param pts: 現在の脱スキュー済み点群（base_link座標）
        :param dr_pose: 推測航法で進めた現在姿勢（原点フレーム）。ICPの初期値。
            「前回の採用姿勢＋今周期のデッドレコニング差分」を渡すこと
        """
        self.updates += 1
        if self._kf is None:
            return AnchorUpdate(dr_pose, False, 0.0, math.inf, 0.0)

        # ── 直近キーフレームに合わせ、chainで原点フレームへ移す ──
        guess_kf = between(self._chain, dr_pose)
        d1 = match_scans(self._kf, pts, guess_kf, iters=self.iters,
                         max_pair=self.max_pair)
        corr1 = math.hypot(d1.dx - guess_kf[0], d1.dy - guess_kf[1])
        if self._acceptable(d1, corr1):
            pose = compose(self._chain, (d1.dx, d1.dy, d1.dyaw))
            self._miss = 0
            self.accepted += 1
            self._maybe_keyframe(pts, pose)
            return AnchorUpdate(pose, True, d1.inlier, d1.residual, corr1)

        # ── 信用できない → 推測航法のまま ──
        self._miss += 1
        if self._miss >= self.reanchor_after and self._can_be_reference(pts):
            # 登録が続かないなら、今のスキャンをキーフレームにして繋ぎ直す。
            # **基点は「今の最良推定」＝`dr_pose`**（今周期の登録は信用でき
            # ないので、その値を基点には使わない）
            self._kf = pts
            self._chain = dr_pose
            self._miss = 0
            self.keyframes += 1
        return AnchorUpdate(dr_pose, False, d1.inlier, d1.residual, corr1)

    def _acceptable(self, d: Delta, corr: float) -> bool:
        return bool(d.ok and d.inlier >= self.min_inlier
                    and d.residual <= self.max_residual
                    and corr <= self.max_correction)

    def _maybe_keyframe(self, pts: Points, pose: Pose) -> None:
        """直近キーフレームから離れすぎていたら進める。

        1リンクあたりの相対移動を小さく保つのが精度の条件（モジュール
        docstringの実測参照）。**原点キーフレームは差し替えない**——
        そこに合えるうちは合わせ続けたいため。
        """
        rel = between(self._chain, pose)
        if (math.hypot(rel[0], rel[1]) >= self.reanchor_dist
                or abs(rel[2]) >= self.reanchor_yaw) and self._can_be_reference(pts):
            self._kf = pts
            self._chain = pose
            self.keyframes += 1

    @staticmethod
    def _can_be_reference(pts: Points) -> bool:
        """キーフレームとして使える点数があるか。

        点が少なすぎるスキャン（全周が飽和・大半が欠測）へ繋ぎ直しても
        次の周でまた失敗するだけで、**繋ぎ直すたびに誤差が1回ぶん乗る**
        ので損しかしない。その場合は今のキーフレームを保持する。
        """
        return int(pts.hit.sum()) >= MIN_POINTS
