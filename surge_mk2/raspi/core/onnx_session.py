"""ONNX Runtime のセッションを Pi 向けの省電力設定で作る（推論ノードと planner の共通部品）。

★省電力化: 既定の `SessionOptions` は全4コアへ並列化し、さらにスレッドをスピンウェイトさせて
推論の合間も CPU を回し続ける（`session.*.allow_spinning`）。224×224 程度の軽量モデルでは
並列化のオーバーヘッドの方が大きく、他ノード（camera_node・planning_node 等）とコアを取り合う。
LiDAR E2E（観測273次元の MLP）では、10Hz 推論で制御系電流が +100mA・CPU31% だったのが、
1スレッド・スピン無効で +3mA になり推論時間も変わらなかった。
参照: https://onnxruntime.ai/docs/performance/tune-performance/threading.html

`onnxruntime` は呼んだときに import する（Mac の GUI やテストで入っていなくても import できるように）。
"""

from __future__ import annotations

from pathlib import Path

__all__ = ["make_session"]


def make_session(path: str | Path, *, intra_threads: int = 2):
    """CPU・スピン無効・`intra_threads` スレッドの `InferenceSession` を返す。

    :param intra_threads: 演算内の並列数。画像のモデルは 2、小さな MLP は 1
    """
    import onnxruntime as ort

    opts = ort.SessionOptions()
    opts.intra_op_num_threads = int(intra_threads)
    opts.inter_op_num_threads = 1
    opts.add_session_config_entry("session.intra_op.allow_spinning", "0")
    opts.add_session_config_entry("session.inter_op.allow_spinning", "0")
    return ort.InferenceSession(str(path), sess_options=opts, providers=["CPUExecutionProvider"])
