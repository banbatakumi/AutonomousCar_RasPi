"""ml_cam_e2e/extract_pairs.py — `.mcap` から (前方カメラ画像, 人間の操舵指令) のペアを取り出す。

    python3 ml_cam_e2e/extract_pairs.py logs/run1.mcap logs/run2.mcap \\
        --out ml_cam_e2e/runs/v1/frames

`ml_cam/extract_frames.py` と同じく、GUI の「ログ」タブ（画像を含める設定で
録画した `.mcap`）に既に入っている `/viz/image/front`（JPEG）を読む。
**それに加えて `/cmd`（`DriveCmd`。50Hzで流れる人間の操舵指令）も読み、
画像フレームごとに時刻最近傍の指令をペアリングする。** Pi 側に新しい記録
コードは不要——`raspi/nodes/logger_node.py` が `cmd` を既に mcap へ記録している。

## セグメンテーション（`ml_cam/`）と違い、手作業のアノテーションが要らない

ラベルは人間がその瞬間に実際に切っていた舵角そのもの。**その代わり、
質の悪いペアを機械的に弾く責任がここに集まる**:

- **`arm=False` のフレームは除外。** ARM していない待機中のデータは
  「常に舵0」という無意味な信号を混ぜるだけで、学習を悪化させる
- **`mode != MANUAL` のフレームは除外。** `mode=AUTO` 中の `cmd` は
  planning_node（何らかの既存 planner）の出力であって人間の操作ではない。
  ここを弾かないと「模倣学習のはずが別の planner を模倣する」事故になる
- **画像とペアリングした `cmd` の時刻差が `--max-gap-ms` を超えたら破棄。**
  録画が途切れていた区間などで、無関係な指令と画像が対応付けられるのを防ぐ
"""

from __future__ import annotations

import argparse
import base64
import bisect
import csv
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # repo root

try:
    from mcap.reader import make_reader
except ImportError:
    print("mcap が入っていません。`pip install -r ml_cam_e2e/requirements.txt` してください",
          file=sys.stderr)
    raise

from raspi.msgs.types import TOPIC_CMD  # noqa: E402

__all__ = ["VIZ_IMAGE_PREFIX", "CMD_MCAP_TOPIC", "MODE_MANUAL",
           "load_cmd_series", "nearest_cmd", "iter_valid_pairs",
           "count_valid_pairs", "extract_one"]

#: `raspi/rec/mcap_log.py` の `VIZ_IMAGE_PREFIX` と同じ値（`ml_cam/extract_frames.py` 参照）
VIZ_IMAGE_PREFIX = "/viz/image/"
#: `raspi/rec/mcap_log.py` の `_mcap_topic()`（バスのトピック名の先頭に `/` を付けるだけ）
#: と同じ変換を `TOPIC_CMD`（`raspi/msgs/types.py`）に適用したもの。
CMD_MCAP_TOPIC = "/" + TOPIC_CMD
#: `raspi/msgs/types.py` の `DriveCmd.mode` の値（`0=DISARM 1=MANUAL 2=AUTO`）
MODE_MANUAL = 1


def load_cmd_series(mcap_path: Path) -> tuple[list[int], list[dict]]:
    """`/cmd` を全部読み、時刻昇順の `(時刻の一覧, 中身の一覧)` に分けて返す。

    2本の並行リストにしておくと `bisect` で時刻探索できる（`nearest_cmd()` 参照）。
    録画は概ね時系列順に書かれるが、**保証はしないので明示的にソートする。**
    """
    times: list[int] = []
    cmds: list[dict] = []
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for _schema, _channel, message in reader.iter_messages(topics=[CMD_MCAP_TOPIC]):
            times.append(message.log_time)
            cmds.append(json.loads(message.data))
    order = sorted(range(len(times)), key=lambda i: times[i])
    return [times[i] for i in order], [cmds[i] for i in order]


def nearest_cmd(cmd_times: list[int], cmd_series: list[dict], t_ns: int,
                max_gap_ns: int) -> dict | None:
    """`t_ns` に最も近い `cmd` を返す。`max_gap_ns` を超えて離れていたら `None`。"""
    if not cmd_times:
        return None
    i = bisect.bisect_left(cmd_times, t_ns)
    candidates = [j for j in (i - 1, i) if 0 <= j < len(cmd_times)]
    best = min(candidates, key=lambda j: abs(cmd_times[j] - t_ns))
    if abs(cmd_times[best] - t_ns) > max_gap_ns:
        return None
    return cmd_series[best]


def _is_valid_cmd(cmd: dict) -> bool:
    """ARM中・MANUAL中の指令だけを教師データとして使ってよいと判断する
    （モジュールdocstring参照）。"""
    return bool(cmd.get("arm")) and int(cmd.get("mode", 0)) == MODE_MANUAL


def iter_valid_pairs(mcap_path: Path, cams: set[str], cmd_times: list[int],
                     cmd_series: list[dict], *, max_gap_ns: int):
    """`(cam, t_ns, jpeg_bytes, target_steer, target_speed)` を時系列に生成する。

    フィルタ（ARM・MANUAL・時刻差）を通らないフレームは黙って読み飛ばす。
    """
    topics = [VIZ_IMAGE_PREFIX + c for c in cams]
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for _schema, channel, message in reader.iter_messages(topics=topics):
            cam = channel.topic[len(VIZ_IMAGE_PREFIX):]
            t_ns = message.log_time
            cmd = nearest_cmd(cmd_times, cmd_series, t_ns, max_gap_ns)
            if cmd is None or not _is_valid_cmd(cmd):
                continue
            obj = json.loads(message.data)
            jpg = base64.b64decode(obj["data"])
            yield cam, t_ns, jpg, float(cmd.get("target_steer", 0.0)), \
                float(cmd.get("target_speed", 0.0))


def count_valid_pairs(mcap_path: Path, cams: set[str], cmd_times: list[int],
                      cmd_series: list[dict], *, max_gap_ns: int) -> int:
    """`--target-count` の間引き幅を決めるための下見（JPEGデコードはしない軽い走査）。"""
    topics = [VIZ_IMAGE_PREFIX + c for c in cams]
    n = 0
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for _schema, _channel, message in reader.iter_messages(topics=topics):
            cmd = nearest_cmd(cmd_times, cmd_series, message.log_time, max_gap_ns)
            if cmd is not None and _is_valid_cmd(cmd):
                n += 1
    return n


def extract_one(mcap_path: Path, out_dir: Path, cams: set[str], writer: csv.writer, *,
                max_gap_ns: int, min_interval_ns: int = 0, keep_ratio: float = 1.0,
                acc_by_cam: dict[str, float] | None = None,
                ) -> tuple[int, dict[str, float]]:
    """1つの `.mcap` からペアを書き出す。`(書いた枚数, 更新後の acc_by_cam)` を返す。

    間引きの2方式（誤差蓄積法 `keep_ratio` / 時間間隔 `min_interval_ns`）は
    `ml_cam/extract_frames.py` の `extract_one()` と同じ実装・同じ選び方
    （`keep_ratio < 1.0` の方が優先）。ここでは**フィルタ後の有効なペアだけ**を
    間引きの対象にする——除外されるフレームまで数えると、目標枚数
    （`--target-count`）からのズレが大きくなる。
    """
    n = 0
    acc_by_cam = dict(acc_by_cam) if acc_by_cam else {}
    last_t_by_cam: dict[str, int] = {}
    cmd_times, cmd_series = load_cmd_series(mcap_path)
    for cam, t_ns, jpg, steer, speed in iter_valid_pairs(
            mcap_path, cams, cmd_times, cmd_series, max_gap_ns=max_gap_ns):
        if keep_ratio < 1.0:
            acc = acc_by_cam.get(cam, 0.0) + keep_ratio
            take = acc >= 1.0
            acc_by_cam[cam] = acc - 1.0 if take else acc
        else:
            last_t = last_t_by_cam.get(cam)
            take = last_t is None or t_ns - last_t >= min_interval_ns
        if not take:
            continue
        name = f"{mcap_path.stem}_{cam}_{t_ns}.jpg"
        (out_dir / name).write_bytes(jpg)
        writer.writerow([name, mcap_path.name, cam, t_ns, steer, speed])
        last_t_by_cam[cam] = t_ns
        n += 1
    return n, acc_by_cam


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mcap_files", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, default=Path("ml_cam_e2e/data/frames"))
    ap.add_argument("--cam", choices=("front", "rear", "both"), default="front",
                    help="取り出すカメラ（既定 front。カメラE2Eは前方走行が対象）")
    ap.add_argument("--max-gap-ms", type=int, default=100,
                    help="画像フレームと `cmd` の時刻差の許容上限[ms]。"
                         "これを超えて近い指令が無ければそのフレームは捨てる")
    group = ap.add_mutually_exclusive_group()
    group.add_argument("--min-interval-ms", type=int, default=0,
                       help="この間隔未満のフレームは間引く（既定 0 = 全件出力）")
    group.add_argument("--target-count", type=int, default=0,
                       help="全 .mcap 合計でこの枚数に近づくよう均等に間引く")
    args = ap.parse_args()

    cams = {"front", "rear"} if args.cam == "both" else {args.cam}
    args.out.mkdir(parents=True, exist_ok=True)
    manifest_path = args.out / "manifest.csv"
    is_new = not manifest_path.exists()

    existing = [p for p in args.mcap_files if p.exists()]
    for p in args.mcap_files:
        if p not in existing:
            print(f"# skip: {p}（見つからない）", file=sys.stderr)

    max_gap_ns = args.max_gap_ms * 1_000_000
    keep_ratio = 1.0
    if args.target_count > 0:
        total = 0
        for p in existing:
            cmd_times, cmd_series = load_cmd_series(p)
            total += count_valid_pairs(p, cams, cmd_times, cmd_series, max_gap_ns=max_gap_ns)
        keep_ratio = min(1.0, args.target_count / total) if total else 1.0
        print(f"# ARM中・MANUAL中の有効フレーム: 合計{total}枚 → keep_ratio={keep_ratio:.4f}")

    min_interval_ns = args.min_interval_ms * 1_000_000
    total_written = 0
    acc_by_cam: dict[str, float] = {}
    with open(manifest_path, "a", newline="") as mf:
        w = csv.writer(mf)
        if is_new:
            w.writerow(["file", "source_mcap", "cam", "t_capture_ns",
                       "target_steer", "target_speed"])
        for p in existing:
            n, acc_by_cam = extract_one(p, args.out, cams, w, max_gap_ns=max_gap_ns,
                                        min_interval_ns=min_interval_ns,
                                        keep_ratio=keep_ratio, acc_by_cam=acc_by_cam)
            print(f"# {p.name}: {n}枚")
            total_written += n

    print(f"# 合計 {total_written}枚 → {args.out}/manifest.csv に追記")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
