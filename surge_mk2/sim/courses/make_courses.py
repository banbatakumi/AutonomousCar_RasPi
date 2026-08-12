"""同梱コースの PNG と JSON を生成する。

    python -m sim.courses.make_courses

自作のコースは PNG を直接描いてもよい（**白 = 走行可、黒 = 壁**）。その場合は
同名の JSON を隣に置いて解像度とスタート姿勢を書く。この生成器は同梱の3枚を
再現可能にしておくためのもので、実行時には使わない。

画像は行0が上、世界座標は y が上に増えるので、描画時に y を反転する（`_px`）。
"""

from __future__ import annotations

import json
import math
from pathlib import Path

from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
RES = 0.02          # m/px
FREE, WALL = 255, 0


def _canvas(w_m: float, h_m: float):
    w, h = round(w_m / RES), round(h_m / RES)
    img = Image.new("L", (w, h), FREE)
    return img, ImageDraw.Draw(img), w, h


def _px(h: int, x: float, y: float) -> tuple[float, float]:
    """世界座標 [m] → 画像座標 [px]（原点は画像左下）。"""
    return x / RES, (h - 1) - y / RES


def _save(img, name: str, start, note: str) -> None:
    img.save(HERE / f"{name}.png")
    meta = {
        "name": name,
        "note": note,
        "resolution": RES,
        "origin": [0.0, 0.0],
        "start": list(start),
        "wall_threshold": 128,
    }
    (HERE / f"{name}.json").write_text(
        json.dumps(meta, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"  {name}.png  {img.size[0]}x{img.size[1]}px  "
          f"{img.size[0]*RES:.1f}x{img.size[1]*RES:.1f}m")


def _rect(d, h, x0, y0, x1, y1, fill=WALL) -> None:
    a = _px(h, x0, y0)
    b = _px(h, x1, y1)
    d.rectangle([min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])],
                fill=fill)


def room() -> None:
    """6×4m の部屋。**左右非対称**にしてある。

    スタートは (1.0, 2.0) で +x を向く。ブロックは y=3.2 ＝ **車から見て左**に置いた。
    LiDAR の鏡像反転（`ScanAssembler`）が壊れていると GUI では右に出るので、
    これが最小の検算になる。左右対称なコースだけで確認すると一生気づけない。
    """
    img, d, w, h = _canvas(6.0, 4.0)
    t = 0.10                                   # 壁の厚み [m]
    _rect(d, h, 0, 0, 6.0, t)
    _rect(d, h, 0, 4.0 - t, 6.0, 4.0)
    _rect(d, h, 0, 0, t, 4.0)
    _rect(d, h, 6.0 - t, 0, 6.0, 4.0)
    _rect(d, h, 2.75, 3.0, 3.25, 3.5)          # ★ 左側だけにあるブロック
    _save(img, "room", (1.0, 2.0, 0.0),
          "最小確認用。左側だけにブロックがある非対称な部屋（鏡像反転の検算）")


def oval() -> None:
    """10×6m のオーバル周回路。走行幅 1.0m。"""
    img, d, w, h = _canvas(10.0, 6.0)
    cx, cy = 5.0, 3.0
    outer, inner = (4.2, 2.2), (3.2, 1.2)

    def ellipse(rx, ry, fill):
        a = _px(h, cx - rx, cy - ry)
        b = _px(h, cx + rx, cy + ry)
        d.ellipse([min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])],
                  fill=fill)

    d.rectangle([0, 0, w, h], fill=WALL)
    ellipse(*outer, FREE)
    ellipse(*inner, WALL)
    # 走行帯は下側の直線で y ∈ [0.8, 1.8]。その中央に置く。
    # **最初 y=0.3 と書いて壁の中からスタートしていた**（車が動かず auto_stop が
    # 効き続ける。原因を追う羽目になるので `_verify` で自動検出するようにした）
    _save(img, "oval", (5.0, 1.3, 0.0), "周回路。走行幅 1.0m")


def slalom() -> None:
    """12×3m の直線路にパイロンを互い違いに置いたもの。"""
    img, d, w, h = _canvas(12.0, 3.0)
    t = 0.10
    _rect(d, h, 0, 0, 12.0, t)
    _rect(d, h, 0, 3.0 - t, 12.0, 3.0)
    _rect(d, h, 0, 0, t, 3.0)
    _rect(d, h, 12.0 - t, 0, 12.0, 3.0)
    r = 0.12
    for i in range(9):
        x = 2.0 + i * 1.1
        y = 1.0 if i % 2 == 0 else 2.0
        a = _px(h, x - r, y - r)
        b = _px(h, x + r, y + r)
        d.ellipse([min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])],
                  fill=WALL)
    _save(img, "slalom", (0.6, 1.5, 0.0), "スラローム。パイロン間隔 1.1m")


def _verify() -> int:
    """全コースのスタート姿勢が壁に埋まっていないか確かめる。

    埋まっていると車が一切動かず、超音波 2cm → auto_stop が効き続けるという
    分かりにくい症状になる（実際にやった）。生成した直後に気づけるようにする。
    """
    import sys
    sys.path.insert(0, str(HERE.parents[1]))
    from sim.course import Course, list_courses
    from sim.vehicle import VehicleSpec

    spec = VehicleSpec.load()
    bad = 0
    for png in list_courses(HERE):          # PNG もセンターライン方式も見る
        c = Course.load(png)
        body = c.body_samples(spec.footprint)
        hit = c.collides(c.start[0], c.start[1], c.start[2], body)
        d = c.ray(c.start[0], c.start[1], c.start[2], 12.0)
        print(f"  {c.name:8s} start={c.start} 埋まり={'★あり' if hit else 'なし'} 前方={d:.2f}m")
        bad += hit
    return bad


def main() -> None:
    print(f"# コース生成 (解像度 {RES} m/px) -> {HERE}")
    room()
    oval()
    slalom()
    print("# 検証")
    if _verify():
        raise SystemExit("!! スタート姿勢が壁に埋まっているコースがある")


if __name__ == "__main__":
    main()
