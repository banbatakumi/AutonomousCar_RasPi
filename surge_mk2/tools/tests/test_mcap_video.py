"""mcap の画像 → mp4。コマ数（＝再生時間）とカメラ間の同期を見る。"""

from __future__ import annotations

import cv2
import numpy as np

from raspi.rec.mcap_log import McapLog
from tools.mcap_video import default_out_path, export_videos

S = 1_000_000_000


def _jpeg(value: int) -> bytes:
    ok, buf = cv2.imencode(".jpg", np.full((48, 64, 3), value, np.uint8))
    assert ok
    return buf.tobytes()


def _frames(path) -> list[int]:
    """各コマの平均輝度。"""
    cap = cv2.VideoCapture(str(path))
    out = []
    while True:
        ok, img = cap.read()
        if not ok:
            break
        out.append(int(round(float(img.mean()))))
    cap.release()
    return out


def test_front_and_rear_are_written_on_a_shared_timeline(tmp_path):
    src = tmp_path / "run.mcap"
    with McapLog(src, t0_mono_ns=0, t0_unix_ns=0) as log:
        log.write_viz_image(_jpeg(40), "front", t_mono_ns=0)
        log.write_viz_image(_jpeg(200), "rear", t_mono_ns=S // 2)
        log.write_viz_image(_jpeg(120), "front", t_mono_ns=1 * S)
        log.write_viz_image(_jpeg(200), "rear", t_mono_ns=2 * S)

    seen: list[float] = []
    results = export_videos(src, fps=10, progress=seen.append)

    assert [r.cam for r in results] == ["front", "rear"]
    assert [r.images for r in results] == [2, 2]
    assert seen[-1] == 1.0
    front, rear = (_frames(r.path) for r in results)
    # 0〜2s を 10fps で: 両方 21 コマ（前は 1s で、後は最初から終わりまで同じ画像）
    assert len(front) == len(rear) == 21
    assert abs(front[9] - 40) < 12 and abs(front[10] - 120) < 12 and abs(front[20] - 120) < 12
    assert all(abs(v - 200) < 12 for v in rear)


def test_camera_without_images_is_skipped(tmp_path):
    src = tmp_path / "run.mcap"
    with McapLog(src, t0_mono_ns=0, t0_unix_ns=0) as log:
        log.write_viz_image(_jpeg(80), "front", t_mono_ns=0)

    results = export_videos(src, tmp_path / "out_missing", cams=["rear"])
    assert results == []

    results = export_videos(src, fps=10)
    assert [r.cam for r in results] == ["front"]
    assert results[0].path == default_out_path(src, "front")
    assert not default_out_path(src, "rear").exists()


def test_truncated_recording_exports_what_was_readable(tmp_path):
    src = tmp_path / "run.mcap"
    # 雑音画像で1枚を重くして、記録を複数のチャンク（既定1MB）に分ける。
    # 1チャンクに収まると、末尾を落とした時点で1枚も読めなくなる
    noise = np.random.default_rng(0).integers(0, 256, (480, 640, 3), dtype=np.uint8)
    ok, buf = cv2.imencode(".jpg", noise)
    assert ok
    with McapLog(src, t0_mono_ns=0, t0_unix_ns=0) as log:
        for i in range(12):
            log.write_viz_image(buf.tobytes(), "front", t_mono_ns=i * S // 10)
    data = src.read_bytes()
    src.write_bytes(data[: len(data) // 2])

    results = export_videos(src, fps=10)

    assert len(results) == 1
    assert 0 < results[0].images < 12
    assert results[0].truncated
    assert len(_frames(results[0].path)) > 0
