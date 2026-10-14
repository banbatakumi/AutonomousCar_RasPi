"""`sim/image_course.py`（画像 → 占有格子）のテスト。合成画像で、色推定・ノイズ除去・
手動編集・保存時の切り出しが壊れていないことを確かめる。
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

import numpy as np  # noqa: E402

from sim import image_course as ic  # noqa: E402

ORANGE = (230, 120, 20)
FLOOR = (90, 163, 124)
BG = (25, 25, 45)


def _photo() -> np.ndarray:
    """背景 + 床 + 管のリング（縞入り）+ 小さな汚れ + 離れた別の壁。"""
    img = np.zeros((200, 400, 3), dtype=np.uint8)
    img[:] = BG
    img[20:180, 40:360] = FLOOR
    img[30:50, 60:340] = ORANGE          # 上の壁
    img[150:170, 60:340] = ORANGE        # 下の壁
    img[30:170, 60:80] = ORANGE          # 左の壁
    img[30:170, 320:340] = ORANGE        # 右の壁
    img[30:50:4, 60:340] = (40, 20, 5)   # 縞（管の継ぎ目）
    img[100:102, 200:202] = ORANGE       # 汚れ（2x2）
    img[90:110, 380:395] = ORANGE        # 床の外の別の塊
    return img


class TestGuess(unittest.TestCase):
    def test_orange_walls_beat_the_floor(self):
        r, g, b = ic.guess_wall_color(_photo())
        self.assertGreater(r, 200)
        self.assertLess(b, 80)

    def test_binary_image_dark_is_wall(self):
        img = np.full((50, 50, 3), 255, dtype=np.uint8)
        img[10:20, :] = 0
        self.assertEqual(ic.guess_wall_color(img), (0, 0, 0))


class TestMask(unittest.TestCase):
    def test_closing_fills_stripes_and_min_area_drops_specks(self):
        img = _photo()
        ref = ic.guess_wall_color(img)
        raw = ic.build_mask(img, ref, 30, 0, 0, [])
        self.assertFalse(raw[34, 100])                    # 縞は抜けている
        clean = ic.build_mask(img, ref, 30, 2, 50, [])
        self.assertTrue(clean[34, 100])                   # 閉じて塞がった
        self.assertFalse(clean[100, 200])                 # 2x2 の汚れは消えた
        self.assertTrue(clean[100, 70])                   # 本物の壁は残る

    def test_blob_erase_removes_only_the_clicked_component(self):
        img = _photo()
        ref = ic.guess_wall_color(img)
        m = ic.build_mask(img, ref, 30, 2, 50, [("blob", 387, 100)])
        self.assertFalse(m[100, 387])
        self.assertTrue(m[100, 70])

    def test_blob_click_near_edge_still_hits(self):
        img = _photo()
        ref = ic.guess_wall_color(img)
        m = ic.build_mask(img, ref, 30, 2, 50, [("blob", 375, 100)])   # 塊の5px外
        self.assertFalse(m[100, 387])

    def test_rect_erase_and_fill_are_ordered(self):
        img = _photo()
        ref = ic.guess_wall_color(img)
        ops = [("erase", 60, 30, 100, 50), ("fill", 60, 100, 70, 110)]
        m = ic.build_mask(img, ref, 30, 2, 50, ops)
        self.assertFalse(m[40, 80])
        self.assertTrue(m[105, 65])
        ops = [("fill", 60, 100, 70, 110), ("erase", 60, 100, 70, 110)]
        self.assertFalse(ic.build_mask(img, ref, 30, 2, 50, ops)[105, 65])


class TestGrid(unittest.TestCase):
    def test_sample_and_crop_roundtrip(self):
        mask = np.zeros((200, 400), dtype=bool)
        mask[50:60, 100:300] = True
        s = ic.sample_grid(mask, scale=100.0, origin=(1.0, 2.0))
        self.assertEqual(s.grid.shape, (100, 200))        # 2m x 4m @ 0.02
        grid, shift = ic.crop_for_save(s.grid, s.origin)
        pad = int(round(ic.MARGIN / ic.RES))
        ys, xs = np.nonzero(grid)
        self.assertEqual((ys.min(), xs.min()), (pad, pad))
        # 画像の行50（上から）= 世界 y = 2 + (200-1-50)/100 付近 → 新座標との差が shift
        wall_y_world = 2.0 + (200 - 1 - 55) / 100.0
        wall_y_new = (ys.mean() + 0.5) * ic.RES
        self.assertAlmostEqual(wall_y_world - shift[1], wall_y_new, delta=0.03)

    def test_crop_without_walls_raises(self):
        with self.assertRaises(ValueError):
            ic.crop_for_save(np.zeros((10, 10), dtype=bool), (0.0, 0.0))


class TestSampleGridThinWall(unittest.TestCase):
    def test_one_pixel_line_survives_downsampling(self):
        mask = np.zeros((100, 100), dtype=bool)
        mask[:, 51] = True                       # 幅1画素の縦線（セル中心から外れる位置）
        g = ic.sample_grid(mask, scale=100.0, res=0.05).grid   # 1セル = 5画素
        self.assertTrue(g[:, 10].all())          # 51/5 = 10 列目

    def test_orientation_row0_is_bottom(self):
        mask = np.zeros((100, 100), dtype=bool)
        mask[99, :] = True                       # 画像の最下行 = y 最小
        g = ic.sample_grid(mask, scale=100.0, res=0.05).grid
        self.assertTrue(g[0].all() and not g[1:].any())


if __name__ == "__main__":
    unittest.main()
