"""進路ガイド（GUI の `cameraModel.ts`）と IPM（Python の `ipm.py`）が同じ場所を指すこと。

**GUI の投影式を Python の基準実装と数値で突き合わせる。** 画像サイズ（`--size`・
`bottom_crop`・縮小配信）が変わっても、前後カメラ・魚眼/ピンホール/補正映像のどれでも
ずれないことを、サイズの組み合わせを振って確かめる。
node と `gui/node_modules`（esbuild）が無い環境（クラウド CI 等）では skip する。
"""

import json
import math
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.core.camera_model import FisheyeCalib, virtual_pinhole_maps  # noqa: E402
from raspi.nav.ipm import CameraExtrinsics, camera_intrinsics, ground_to_pixel  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
ESBUILD = ROOT / "gui" / "node_modules" / ".bin" / "esbuild"

CALIB = {"fx": 307.9092, "fy": 307.5942, "cx": 307.4241, "cy": 241.533,
         "k": [-0.03208693, 0.00352001, -0.01074923, 0.00351499],
         "width": 640, "height": 480}
#: (幅, 高さ, bottom_crop)。高さは 480*(1-crop) 相当（実機の配信）と、縮小・別比率の両方
SIZES = [(640, 360, 0.25), (640, 450, 0.0625), (640, 274, 0.43), (320, 180, 0.25),
         (1280, 720, 0.25), (800, 600, 0.0), (480, 200, 0.4)]
# 路面の点（base_link, x=前 y=左）。前は前方、後ろは後方に取る
GROUND = [(x, y) for x in (0.3, 0.6, 1.0, 1.5, 2.5, 4.0) for y in (-0.8, -0.2, 0.0, 0.3, 0.9)]
POSES = [  # (x, y, height, pitch)
    (0.1, 0.0, 0.088, 0.0),
    (0.039, 0.0, 0.088, 0.0),
    (0.1, 0.01, 0.12, 0.25),
    (0.05, -0.02, 0.07, -0.1),
]


def _bundle_and_run(cases: list[dict]) -> list:
    """`cameraModel.ts` を esbuild で束ねて node で走らせ、各ケースの画素を返す。"""
    with tempfile.TemporaryDirectory() as d:
        entry = Path(d) / "run.ts"
        entry.write_text(f"""
import {{ fisheyeLens, pinholeLens, groundProjector }} from "{(ROOT / 'gui/src/render/cameraModel.ts').as_posix()}"
const cases = JSON.parse(process.argv[2])
const out = cases.map((c: any) => {{
  const lens = c.lens === 'fisheye' ? fisheyeLens(c.w, c.h, c.calib, c.crop)
    : pinholeLens(c.w, c.h, c.hfov, c.crop)
  const proj = groundProjector(lens, c.pose, c.facing, c.roll ?? 0)
  return c.points.map((p: number[]) => proj(p[0], p[1]))
}})
console.log(JSON.stringify(out))
""")
        js = Path(d) / "run.js"
        subprocess.run([str(ESBUILD), str(entry), "--bundle", "--platform=node",
                        f"--outfile={js}", "--log-level=error"], check=True)
        res = subprocess.run(["node", str(js), json.dumps(cases)], check=True,
                             capture_output=True, text=True)
    return json.loads(res.stdout)


def _python_ref(case: dict) -> list:
    calib = FisheyeCalib.from_dict(case["calib"]) if case["lens"] == "fisheye" else None
    intr = camera_intrinsics(case["hfov"], case["w"], case["h"], case["crop"], calib)
    x, y, height, pitch = case["pose"]["x"], case["pose"]["y"], case["pose"]["height"], \
        case["pose"]["pitch"]
    yaw = 0.0 if case["facing"] == "front" else math.pi
    ext = CameraExtrinsics(x=x, y=y, height=height, pitch=pitch, yaw=yaw)
    return [ground_to_pixel(px, py, intr, ext) for px, py in case["points"]]


@unittest.skipUnless(ESBUILD.exists() and shutil.which("node"),
                     "node と gui/node_modules が無い（GUI の依存を入れていない）")
class TestGuiProjectionMatchesIpm(unittest.TestCase):
    def _cases(self, lens: str, facing: str) -> list[dict]:
        cases = []
        for w, h, crop in SIZES:
            for px, py, ph, pp in POSES:
                pts = GROUND if facing == "front" else [(-x + 0.0, -y) for x, y in GROUND]
                cases.append({"lens": lens, "facing": facing, "w": w, "h": h, "crop": crop,
                              "hfov": 2.23, "calib": CALIB,
                              "pose": {"x": px, "y": py, "height": ph, "pitch": pp},
                              "points": [list(p) for p in pts]})
        return cases

    def _compare(self, lens: str, facing: str) -> None:
        cases = self._cases(lens, facing)
        ts = _bundle_and_run(cases)
        worst = 0.0
        compared = 0
        for case, got in zip(cases, ts):
            ref = _python_ref(case)
            for g, r, pt in zip(got, ref, case["points"]):
                tag = (lens, facing, case["w"], case["h"], case["crop"], case["pose"], pt)
                self.assertEqual(g is None, r is None, f"映る/映らないが食い違う {tag}")
                if g is None:
                    continue
                err = math.hypot(g[0] - r[0], g[1] - r[1])
                worst = max(worst, err)
                compared += 1
                self.assertLess(err, 1e-6, f"{err:.3g}px ずれる {tag}")
        self.assertGreater(compared, 100, "比較した点が少なすぎる（全部画面外？）")

    def test_fisheye_front(self):
        self._compare("fisheye", "front")

    def test_fisheye_rear(self):
        self._compare("fisheye", "rear")

    def test_pinhole_front(self):
        self._compare("pinhole", "front")

    def test_pinhole_rear(self):
        self._compare("pinhole", "rear")

    def test_undistorted_display_matches_fisheye_source(self):
        """補正映像の画素 → Pi の remap が参照する魚眼の画素 が、GUI の魚眼投影と一致する。

        GUI は補正映像を表示中、仮想ピンホール（`undistortHfov`）で描く。同じ地面の点が
        補正映像の `(u', v')` に出るなら、Pi の `virtual_pinhole_maps` は `(u', v')` について
        魚眼画像の同じ点を引いているはず（そうでないと補正映像とガイドがずれる）。
        """
        hfov_out = 1.92
        cases = []
        for w, h, crop in SIZES:
            pose = {"x": 0.1, "y": 0.0, "height": 0.088, "pitch": 0.0}
            pts = [list(p) for p in GROUND]
            base = {"facing": "front", "w": w, "h": h, "crop": crop, "calib": CALIB,
                    "pose": pose, "points": pts}
            cases.append({**base, "lens": "fisheye", "hfov": 0})
            cases.append({**base, "lens": "pinhole", "hfov": hfov_out})
        ts = _bundle_and_run(cases)
        calib = FisheyeCalib.from_dict(CALIB)
        compared = 0
        for i, (w, h, crop) in enumerate(SIZES):
            fish, pin = ts[2 * i], ts[2 * i + 1]
            fx, fy, cx, cy = calib.scaled(w, h, crop)
            mx, my = virtual_pinhole_maps(fx, fy, cx, cy, tuple(calib.k), w, h, hfov_out, crop)
            for f, p in zip(fish, pin):
                if f is None or p is None:
                    continue
                u, v = p
                if not (1 <= u < w - 2 and 1 <= v < h - 2):
                    continue                                 # 補正映像の外
                x0, y0 = int(u), int(v)
                ax, ay = u - x0, v - y0

                def bil(m):
                    return ((1 - ax) * (1 - ay) * m[y0, x0] + ax * (1 - ay) * m[y0, x0 + 1]
                            + (1 - ax) * ay * m[y0 + 1, x0] + ax * ay * m[y0 + 1, x0 + 1])
                su, sv = float(bil(np.asarray(mx))), float(bil(np.asarray(my)))
                err = math.hypot(su - f[0], sv - f[1])
                compared += 1
                # マップは float32・双一次補間なので 1e-6 とはいかない
                self.assertLess(err, 0.05, f"補正映像とずれる {(w, h, crop)} err={err:.3f}px")
        self.assertGreater(compared, 30)


if __name__ == "__main__":
    unittest.main()
