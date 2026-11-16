"""`raspi/nodes/stereo_e2e_node.py` のうち、モデルを要らない部分（推論のきっかけ・契約の照合・ループの配線）。

ONNX を通す推論は `ml_stereo_e2e/tests/test_stereo_e2e.py` が書き出したモデルで見ている。
"""

from __future__ import annotations

import json
import tempfile
from pathlib import Path
from types import SimpleNamespace

import pytest

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
    assert node.take_new_disp(_d(2, 1067)) is None              # 15Hz の次の組（67ms）は間引く
    assert node.take_new_disp(_d(3, 1133)) is not None          # 133ms 後は通す
    assert node.take_new_disp(_d(4, 1210)) is not None          # 周期の 3/4（75ms）以上なら通す


def test_without_model_reports_reason():
    node = StereoE2ENode()
    p = node.infer_once(disp=_d(1, 1000), rgb=None, rgb_left=None, vs=None, now_ns=1000 * MS)
    assert not p.ready and "モデル" in p.reason


def test_contract_must_describe_disparity(tmp_path: Path):
    onnx = tmp_path / "m.onnx"
    onnx.write_bytes(b"")
    onnx.with_suffix(".json").write_text(json.dumps({"preproc_version": 1, "image_preproc_version": 3,
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
