"""ml_e2e/extract.py — 記録（`.mcap`）から学習用の標本を作る。

    .venv/bin/python -m ml_e2e.extract ~/Downloads/toyota/*.mcap --out ml_e2e/data/v1

画像つきで録った記録（GUI の MCAP ボタン・`tools/record.sh --image-hz 10`）の画像1枚ごとに、
その時刻の点群・車両状態と、ラベル（前方の経路点・各点の速度）を組にする。

出力（記録1本につき）:

- `<out>/frames/<記録名>/<撮影時刻>.jpg` — 画像（記録のまま）。右の画像で、同じ瞬間の左は
  `<撮影時刻>_left.jpg`。組にならなかったコマは「左のコマが無い」で使わない
- `<out>/<記録名>.npz` — 標本の配列（下の「配列」）
- `<out>/dataset.json` — 抽出の設定（`K`・`ds` など）と記録ごとの内訳

## ラベル

| 記録の状況 | 経路点（`path_src`） |
|---|---|
| `slam2d_route`／`slam2d_raceline` が RACE で走行中 | 1 = 地図の最適ライン（`labels.path_from_line`）。速度はラインの計画速度 |
| 上記以外の自動運転（回避中・探索中・他の planner） | 2 = その後に通った軌跡（`labels.path_from_future`）。速度は実速度 |
| 人の手動運転 | 2 = その後に通った軌跡。**速度は手本にしない**（`v_ok=False`） |

`/cmd` は「誰が運転していたか」を見るのに使う。舵・速度の指令そのものは手本にしない（モデルは経路点だけを出す）。

人の走行に最適ラインのラベルを付ける（線から外れた所で戻る経路を教える）には、地図上の姿勢が要る。
記録に `slam2d_route` の姿勢が入っていない区間は今は 2 になる。

## 使わないコマ（理由ごとに数えて最後に表示する）

DISARM 中・入力途絶の指令（`:stale`）・トルクモード・このモデル自身の走行（`auto:e2e_fusion`。
`--include-self` で使う）・止まっている・点群が無い・経路点が1点も作れない。

## 時刻

- 基準は**画像の撮影時刻**。経路点はその時刻の車体座標
- 点群は、先頭セクタが撮影時刻の `SCAN_MIN_AGE_S`（0.07s）以上前の最新の周と、その前の周。実車のノードは
  点群の1周が届いた時（先頭セクタの約 0.1〜0.12s 後）に最新のコマと組にするので、画像から見た点群の古さは
  0.07〜0.13s。記録の画像は 10Hz なので標本では 0.07〜0.17s に散り、実車の範囲を含む
- 地図上の姿勢は `AutoState` の姿勢（点群の基準時刻のもの）を推測航法で撮影時刻まで進める。
  姿勢の時刻のずれは記録ごとに `records.estimate_pose_lag` で求める
- 推論の遅れぶんラベルをずらすことはしない（経路点は実行時に推測航法でずらせる）
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from collections import Counter
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # repo root

from ml_e2e import labels as L  # noqa: E402
from ml_e2e import records as R  # noqa: E402

__all__ = ["ExtractConfig", "extract_recording", "main", "SRC_NONE", "SRC_LINE", "SRC_FUTURE",
           "KIND_MANUAL", "KIND_AUTO", "KIND_SELF"]

NS = 1_000_000_000
SRC_NONE, SRC_LINE, SRC_FUTURE = 0, 1, 2
KIND_MANUAL, KIND_AUTO, KIND_SELF = 0, 1, 2
#: 最適ラインを追って走っている planner（`AutoState.mode`）
_LINE_MODES = ("slam2d_route", "slam2d_raceline")
_SELF_SOURCE = "auto:e2e_fusion"
#: 点群の先頭セクタが、画像の撮影時刻よりこれ以上前の周を使う [s]（モジュール docstring「時刻」）
SCAN_MIN_AGE_S = 0.07
#: 姿勢の時刻のずれが求まらない記録（直進だけ等）に使う値 [s]。2026-10〜11 の記録 16 本で 50〜65ms
DEFAULT_POSE_LAG_S = 0.055


class ExtractConfig:
    def __init__(self, *, k: int = 40, ds: float = 0.25, history: int = 3,
                 min_speed: float = 0.15, min_score: float = 0.35,
                 max_cross: float = 1.0, include_self: bool = False) -> None:
        self.k = k                          #: 経路点の数（学習時に間引ける多めの数）
        self.ds = ds                        #: 経路点の間隔 [m]
        self.history = history              #: 保存する点群の周の数（最新を含む）
        self.min_speed = min_speed          #: これ未満の車速のコマは「止まっている」として使わない
        self.min_score = min_score          #: 最適ラインのラベルに使う自己位置の一致度の下限
        self.max_cross = max_cross          #: 最適ラインからこれ以上離れていたらラインのラベルにしない
        self.include_self = include_self

    def to_json(self) -> dict:
        return dict(self.__dict__)


def _nearest(times: np.ndarray, t: int) -> int:
    i = int(np.searchsorted(times, t))
    if i <= 0:
        return 0
    if i >= len(times):
        return len(times) - 1
    return i if times[i] - t < t - times[i - 1] else i - 1


def _kind(rec: R.Recording, i: int, cfg: ExtractConfig) -> tuple[int | None, str]:
    """指令 `i` の運転者の種別。使わないなら `(None, 理由)`。"""
    if not rec.cmd_arm[i]:
        return None, "DISARM中"
    if rec.cmd_torque_mode[i]:
        return None, "トルクモード"
    src = rec.cmd_source[i]
    mode = int(rec.cmd_mode[i])
    if mode == 2:
        if src.endswith(":stale"):
            return None, "入力途絶中の指令(stale)"
        if src.startswith(_SELF_SOURCE):
            return (KIND_SELF, "") if cfg.include_self else (None, "このモデル自身の走行")
        return KIND_AUTO, ""
    if mode == 1:
        return KIND_MANUAL, ""
    return None, "手動でも自動でもない"


def extract_recording(path: Path, out_dir: Path, cfg: ExtractConfig, *,
                      rejects: Counter | None = None, write_images: bool = True) -> dict:
    """記録1本を標本にして `<out_dir>/<記録名>.npz` に書く。内訳の dict を返す。"""
    rec = R.load(path)
    rejects = rejects if rejects is not None else Counter()
    #: `path` は元の記録の場所（評価で同じ記録を実車と同じコードに通すのに使う。`ml_e2e/app.py`）
    info: dict = {"file": Path(path).name, "path": str(Path(path).resolve()), "samples": 0}
    if len(rec.vs_t) < 10 or len(rec.od_t) < 10 or len(rec.cmd_t) < 10 or len(rec.scan_t) < cfg.history:
        info["skipped"] = "車両状態・推測航法・指令・点群のどれかが無い"
        return info

    lag, lag_rms = R.estimate_pose_lag(rec)
    info["pose_lag_ms"] = None if not np.isfinite(lag) else round(lag * 1e3, 1)
    if not np.isfinite(lag):
        lag = DEFAULT_POSE_LAG_S
    pose_t = rec.st_t - (rec.st_scan_age * NS).astype(np.int64) + int(lag * NS) if len(rec.st_t) else rec.st_t
    line_cache: dict[int, tuple] = {}
    stem = Path(path).stem
    img_dir = out_dir / "frames" / stem
    if write_images:
        # 同じ記録を作り直すときは前の画像を残さない（設定を変えると使うコマが変わる）
        shutil.rmtree(img_dir, ignore_errors=True)
        img_dir.mkdir(parents=True, exist_ok=True)

    rows: dict[str, list] = {k: [] for k in (
        "t", "scan_idx", "scan_age", "state", "path_xy", "path_v", "path_valid", "path_src", "cross",
        "v_ok", "kind", "map_pose")}
    names: list[str] = []
    names_left: list[str] = []
    for t, jpg, jpg_left in R.iter_image_pairs(path, rejects):
        ci = _nearest(rec.cmd_t, t)
        if abs(int(rec.cmd_t[ci]) - t) > 0.1 * NS:
            rejects["近い指令が無い"] += 1
            continue
        kind, why = _kind(rec, ci, cfg)
        if kind is None:
            rejects[why] += 1
            continue
        speed, steer, yaw_rate = rec.state_at(t)
        if speed < cfg.min_speed:
            rejects["止まっている"] += 1
            continue
        si = int(np.searchsorted(rec.scan_t, t - int(SCAN_MIN_AGE_S * NS), side="right")) - 1
        if si < 0 or t - int(rec.scan_t[si]) > 0.35 * NS:
            rejects["点群が無い"] += 1
            continue
        idx = [si - h if si - h >= 0 and rec.scan_t[si] - rec.scan_t[si - h] < (h + 0.5) * 0.1 * NS
               else -1 for h in range(cfg.history)]

        # ── 経路点 ──
        label = None
        src = SRC_NONE
        map_pose = (np.nan, np.nan, np.nan)
        if kind in (KIND_AUTO, KIND_SELF) and len(rec.st_t):
            pi = _nearest(pose_t, t)
            ln = rec.line_at(t)
            if (ln is not None and rec.st_mode[pi] in _LINE_MODES and rec.st_phase[pi] == "RACE"
                    and rec.st_score[pi] >= cfg.min_score and rec.st_obstacles[pi] == 0
                    and abs(int(pose_t[pi]) - t) <= 0.25 * NS):
                key = id(ln)
                if key not in line_cache:
                    line_cache[key] = L.resample_line(ln.xy, ln.v if len(ln.v) else None)
                od0 = [float(a) for a in rec.odom_at(int(pose_t[pi]))[:3]]
                od1 = [float(a) for a in rec.odom_at(t)[:3]]
                pose = L.advance_pose(tuple(rec.st_pose[pi]), tuple(od0), tuple(od1))
                label = L.path_from_line(line_cache[key], pose, cfg.k, cfg.ds, max_cross=cfg.max_cross)
                if label is not None and np.isfinite(label.v).all():
                    src = SRC_LINE
                    map_pose = pose
                else:
                    label = None
        if label is None:
            label = L.path_from_future(rec.od_x, rec.od_y, rec.od_yaw, rec.od_dist, rec.od_t,
                                       rec.vs_t, rec.vs_speed, t, cfg.k, cfg.ds)
            if label is not None:
                src = SRC_FUTURE
        if label is None:
            rejects["経路点が作れない"] += 1
            continue

        name = f"{t}.jpg"
        if write_images:
            (img_dir / name).write_bytes(jpg)
        names.append(f"frames/{stem}/{name}")
        name_left = f"{t}_left.jpg"
        if write_images:
            (img_dir / name_left).write_bytes(jpg_left)
        names_left.append(f"frames/{stem}/{name_left}")
        rows["t"].append(t)
        rows["scan_idx"].append(idx)
        rows["scan_age"].append((t - int(rec.scan_t[si])) / NS)
        rows["state"].append((speed, steer, yaw_rate))
        rows["path_xy"].append(label.xy)
        rows["path_v"].append(label.v)
        rows["path_valid"].append(label.valid)
        rows["path_src"].append(src)
        rows["cross"].append(label.cross)
        rows["v_ok"].append(src == SRC_LINE or kind == KIND_AUTO)
        rows["kind"].append(kind)
        rows["map_pose"].append(map_pose)

    n = len(names)
    info["samples"] = n
    if n == 0:
        return info
    src_arr = np.array(rows["path_src"], dtype=np.int8)
    kind_arr = np.array(rows["kind"], dtype=np.int8)
    info.update(line=int((src_arr == SRC_LINE).sum()), future=int((src_arr == SRC_FUTURE).sum()),
                auto=int((kind_arr == KIND_AUTO).sum()), manual=int((kind_arr == KIND_MANUAL).sum()),
                pose_lag_rms=None if not np.isfinite(lag_rms) else round(lag_rms, 4),
                maps=sorted({ln.map_name for ln in rec.lines if ln.map_name}))
    np.savez_compressed(
        out_dir / f"{stem}.npz",
        image=np.array(names),
        image_left=np.array(names_left),
        t=np.array(rows["t"], dtype=np.int64),
        scan_idx=np.array(rows["scan_idx"], dtype=np.int32),
        scan_age=np.array(rows["scan_age"], dtype=np.float32),
        scan_dist=rec.scan_dist, scan_seen=rec.scan_seen, scan_sat=rec.scan_sat,
        state=np.array(rows["state"], dtype=np.float32),
        path_xy=np.array(rows["path_xy"], dtype=np.float32),
        path_v=np.array(rows["path_v"], dtype=np.float32),
        path_valid=np.array(rows["path_valid"], dtype=bool),
        path_src=src_arr,
        cross=np.array(rows["cross"], dtype=np.float32),
        v_ok=np.array(rows["v_ok"], dtype=bool),
        kind=kind_arr,
        map_pose=np.array(rows["map_pose"], dtype=np.float64),
    )
    return info


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mcap_files", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--k", type=int, default=40, help="経路点の数（既定 40）")
    ap.add_argument("--ds", type=float, default=0.25, help="経路点の間隔 [m]（既定 0.25）")
    ap.add_argument("--include-self", action="store_true", help="このモデル自身の走行も使う")
    args = ap.parse_args()

    cfg = ExtractConfig(k=args.k, ds=args.ds, include_self=args.include_self)
    args.out.mkdir(parents=True, exist_ok=True)
    meta_path = args.out / "dataset.json"
    meta = json.loads(meta_path.read_text()) if meta_path.exists() else {"config": cfg.to_json(), "records": {}}
    if meta["config"] != cfg.to_json():
        print(f"{meta_path} の設定が今回と違います。別の --out へ抽出してください", file=sys.stderr)
        return 2

    rejects: Counter = Counter()
    total = 0
    for p in args.mcap_files:
        if not p.exists():
            print(f"# skip: {p}（見つからない）", file=sys.stderr)
            continue
        try:
            info = extract_recording(p, args.out, cfg, rejects=rejects)
        except Exception as e:                              # noqa: BLE001
            print(f"# skip: {p.name}（読めない: {type(e).__name__}: {e}）。壊れていれば "
                  f"`python -m raspi.tools.mcap_repair {p} --inplace`", file=sys.stderr)
            continue
        meta["records"][p.stem] = info
        total += info["samples"]
        if info.get("skipped"):
            print(f"# {p.name}: 使わない（{info['skipped']}）", flush=True)
        else:
            print(f"# {p.name}: {info['samples']}枚（ライン {info.get('line', 0)}・軌跡 {info.get('future', 0)}／"
                  f"自動 {info.get('auto', 0)}・手動 {info.get('manual', 0)}／姿勢のずれ {info.get('pose_lag_ms')}ms）",
                  flush=True)
    meta_path.write_text(json.dumps(meta, ensure_ascii=False, indent=1))
    print(f"# 合計 {total}枚 → {args.out}")
    if rejects:
        print("# 使わなかったコマ: " + "・".join(f"{r} {n}枚" for r, n in rejects.most_common()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
