"""ml_common — 学習用GUI（`ml_lidar/app.py`・`ml_cam_e2e/app.py`）が
共有する薄い部品群。

3つのGUIは同じ設計思想（Tkinter/ttk標準ライブラリのみ、subprocessで学習スクリプトを
呼ぶだけの操作パネル）で作られており、ログ表示・run名管理・note.txt・学習曲線グラフ等が
ほぼ同一実装でコピペされていた。ここに切り出し、各GUIはコンポジションで組み立てる。

巨大な基底クラス／mixinにはしない——3つの`App`クラスはウィジェットレイアウトが
大きく異なるため、多重継承より明示的な部品の組み合わせの方が既存の薄い設計に合う。
例外は `job_panel.JobPanelBase` で、ジョブの実行・停止・ログ・備考という、4つの `App` でほぼ同じ文だった
部分だけを持つ小さな基底クラス（ウィジェットの配置は各 `App` が持つ）。

学習・書き出しのスクリプト側の共通部品:

- `train_utils`: 装置の選択・乱数の固定・最適化器・best の選び方・かたまり単位の train/val 分割
- `onnx_export`: `torch.onnx.export` の設定・PyTorch と ONNXRuntime の突き合わせ・契約 JSON・版の一致の検査
"""
