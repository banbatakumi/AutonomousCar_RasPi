"""mcap の左右画像 → Foxglove 用の点群。壁を描いた合成記録（`stereo_bench` のテストと同じ）で、点群の距離を確かめる。"""

from __future__ import annotations

import base64
import dataclasses
import json

import numpy as np
import pytest

pytest.importorskip("cv2")
pytest.importorskip("mcap")

from mcap.reader import make_reader

from raspi.core.vehicle import Vehicle
from tools import mcap_stereo_cloud as msc
from tools.stereo_bench.tests import test_bench as tb
from tools.stereo_bench.tests.test_bench import (
    recording,  # noqa: F401  （フィクスチャ）
)


def _vehicle() -> Vehicle:
    x, y, z, pitch, yaw = tb.CAM
    return dataclasses.replace(Vehicle(), cam_front_fisheye=tb.RIGHT, cam_left_fisheye=tb.LEFT, stereo=tb.EXT,
                               cam_front_x=x, cam_front_y=y, cam_front_z=z,
                               cam_front_pitch=pitch, cam_front_yaw=yaw)


def test_cloud_topic_added_and_original_kept(tmp_path, monkeypatch, recording):
    monkeypatch.setattr(msc, "_vehicle", lambda src, override: (_vehicle(), "test"))
    out = tmp_path / "out.mcap"
    n, _ = msc.add_stereo_cloud(recording, out)
    assert n == 2

    topics: dict[str, list] = {}
    with open(out, "rb") as f:
        for _s, ch, m in make_reader(f).iter_messages(log_time_order=True):
            topics.setdefault(ch.topic, []).append(m)
    assert len(topics["/viz/image/front"]) == 2 and len(topics["/scan"]) == 2   # 元の中身が残る
    clouds = topics[msc.CLOUD_TOPIC]
    assert len(clouds) == 2

    # 壁は x = 0.8 / 1.6 m（base_link）。点群の x の中央値が壁に合う
    for m, wall in zip(clouds, (0.8, 1.6)):
        d = json.loads(m.data)
        assert d["frame_id"] == "base_link" and d["point_stride"] == 16
        pts = np.frombuffer(base64.b64decode(d["data"]), msc._POINT_DTYPE)
        assert len(pts) > 500
        assert float(np.median(pts["x"])) == pytest.approx(wall, rel=0.05)
        assert (pts["alpha"] == 255).all()


def test_rerun_does_not_duplicate(tmp_path, monkeypatch, recording):
    monkeypatch.setattr(msc, "_vehicle", lambda src, override: (_vehicle(), "test"))
    a, b = tmp_path / "a.mcap", tmp_path / "b.mcap"
    msc.add_stereo_cloud(recording, a, every=2)
    msc.add_stereo_cloud(a, b, every=2)
    with open(b, "rb") as f:
        n = sum(1 for _s, ch, _m in make_reader(f).iter_messages() if ch.topic == msc.CLOUD_TOPIC)
    assert n == 1
