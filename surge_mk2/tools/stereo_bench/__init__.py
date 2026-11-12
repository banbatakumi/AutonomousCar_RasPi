"""ステレオ照合のベンチ（Mac 側）— 実機の記録（`.mcap`）を再生して照合の方式を比べる。

    .venv/bin/python -m tools.stereo_bench logs/surge_*.mcap
    .venv/bin/python -m tools.stereo_bench REC.mcap --wall 1.00          # 物差し（LiDAR）を既知の壁で確かめる
    .venv/bin/python -m tools.stereo_bench REC.mcap --requality 70       # JPEG 品質 70 で焼き直して比べる
    .venv/bin/python -m tools.stereo_bench REC.mcap --save /tmp/stereo   # 平行化した左と視差の絵

計画は `docs/stereo_plan.md` 段階3。シムはカメラを描かないので、照合は実機の記録で開発する。

## 何を比べるか

- 方式（`StereoBM` / `StereoSGBM`）・解像度（640 / 320 幅）・照合する行の範囲（`--rows`）。`PRESETS` を参照
- 1組あたりの計算時間（既定は 1 スレッド。Pi の 1 コアの見積もりに使う）、視差が出た画素の割合

## 精度の物差しは LiDAR

LiDAR の走査面（路面から `[sensors.lidar] z`。カメラの 1.5cm 上）の高さ ±`--band` に入るステレオの点を、
LiDAR と同じ 1° の方位ごとに中央値でまとめ、同じ方位の LiDAR の距離と比べる（`compare_with_lidar`）。
車が止まっている組だけを使う（`--max-speed`。走っていると 1 周 100ms の間に点群が歪む）。

**改善の前に物差しを確かめる**（`--wall`）: 既知の距離の平らな壁に向けて止めて録り、LiDAR とステレオの
正面の距離が壁の距離と合うかを見る。

## 記録

右（`/viz/image/front`）と左（`/viz/image/left`）の撮影時刻が `--pair-tol-ms`（既定 0.5ms）以内のものを組にする。
`logger_node` は同期中の左右を同じ瞬間の組で焼く（段階3 の変更。それより前の記録は最大1コマずれるので、
止まった場面だけ `--pair-tol-ms 40` で使える）。
"""
