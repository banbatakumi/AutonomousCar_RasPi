"""`raspi/nodes/stereo_e2e_node.py` のうち、モデルを要らない部分（推論のきっかけ・契約の照合・ループの配線）。

ONNX を通す推論は `ml_stereo_e2e/tests/test_stereo_e2e.py` が書き出したモデルで見ている。
"""

from __future__ import annotations

import json
import tempfile
from pathlib import Path
from types import SimpleNamespace

import pytest

from raspi.core.stereo_e2e_preproc import PREPROC_VERSION
from raspi.msgs import StereoDisp
from raspi.msgs.types import TOPIC_AUTO_CTRL, TOPIC_STEREO_E2E_PLAN
from raspi.nodes.stereo_e2e_node import StereoE2ENode, load_model

MS = 1_000_000


def _d(seq: int, t_ms: int) -> StereoDisp:
    return StereoDisp(seq=seq, t_capture=t_ms * MS)


def test_new_disp_is_throttled_by_capture_time():
    node = StereoE2ENode(rate_hz=10)
    assert node.take_new_disp(None) is None
    assert node.take_new_disp(_d(1, 1000)) is not None
    assert node.take_new_disp(_d(1, 1000)) is None              # 同じ組は2度使わない
    assert node.take_new_disp(_d(2, 1067)) is None              # 予定（1100）より早い組は間引く
    assert node.take_new_disp(_d(3, 1133)) is not None          # 予定を過ぎた組は通す（次の予定は 1200）
    assert node.take_new_disp(_d(4, 1195)) is not None          # 揺れの幅（10%）なら早くても通す


def test_rate_is_kept_on_average_from_a_15hz_stream():
    """15Hz の視差から既定 10Hz に間引いて、平均が 10Hz になる（前は 1 組おきの 7.5Hz だった。レビュー M3）。"""
    node = StereoE2ENode(rate_hz=10)
    n = sum(node.take_new_disp(StereoDisp(seq=i + 1, t_capture=int(1e9 + i * 1e9 / 15))) is not None
            for i in range(150))                              # 10 秒ぶん
    assert 99 <= n <= 101


def test_rate_restarts_after_a_gap():
    node = StereoE2ENode(rate_hz=10)
    assert node.take_new_disp(_d(1, 1000)) is not None
    assert node.take_new_disp(_d(2, 3000)) is not None          # 途絶のあと
    assert node.take_new_disp(_d(3, 3067)) is None              # 溜めた分を一気に通さない
    assert node.take_new_disp(_d(4, 3100)) is not None


def test_without_model_reports_reason():
    node = StereoE2ENode()
    p = node.infer_once(disp=_d(1, 1000), rgb=None, rgb_left=None, vs=None, now_ns=1000 * MS)
    assert not p.ready and "モデル" in p.reason


def test_contract_must_describe_disparity(tmp_path: Path):
    onnx = tmp_path / "m.onnx"
    onnx.write_bytes(b"")
    onnx.with_suffix(".json").write_text(json.dumps({"preproc_version": PREPROC_VERSION, "image_preproc_version": 3,
                                                     "cams": ["front", "left"]}))
    with pytest.raises(ValueError, match="契約に"):
        load_model(onnx)


class _Pub:
    def __init__(self):
        self.sent = []

    def send(self, topic, msg):
        self.sent.append((topic, msg))


class _Sub:
    """`poll` を呼ぶたびに `latest` を差し替える偽の購読。"""

    def __init__(self, frames):
        self.frames = list(frames)
        self.latest = {}

    def poll(self, _timeout):
        if self.frames:
            self.latest = self.frames.pop(0)
        return []


def test_run_reports_not_ready_while_mode_is_not_selected():
    node = StereoE2ENode(models_dir=Path(tempfile.mkdtemp()))
    sub = _Sub([{TOPIC_AUTO_CTRL: SimpleNamespace(mode="slam2d_route", engaged=False)}])
    pub = _Pub()
    node.run(sub=sub, pub=pub, duration_s=0.05, fixed_model=True)
    plans = [m for t, m in pub.sent if t == TOPIC_STEREO_E2E_PLAN]
    assert plans and not any(p.ready for p in plans)
    assert any(t == "hb/stereo_e2e" for t, _ in pub.sent)


def test_gui_choice_loads_in_background(tmp_path: Path, monkeypatch):
    """GUI で選んだモデルは別スレッドで読み、読み終えるまで今のモデルで続ける（レビュー L21）。"""
    import threading
    import time

    import raspi.nodes.stereo_e2e_node as N

    gate = threading.Event()
    fake = SimpleNamespace(name="new")

    def slow_load(path):
        gate.wait(2.0)
        return fake

    monkeypatch.setattr(N, "load_model", slow_load)
    (tmp_path / "new.onnx").write_bytes(b"")
    node = StereoE2ENode(models_dir=tmp_path, model=SimpleNamespace(name="old"), model_name="old")
    node.reload_in_background("new")
    assert node.model_name == "old" and not node.take_loaded()   # 読んでいる間は今のまま・塞がない
    node.reload_in_background("new")                            # 二重に読まない
    gate.set()
    for _ in range(200):
        if node.take_loaded():
            break
        time.sleep(0.01)
    assert node.model_name == "new" and node.model is fake
