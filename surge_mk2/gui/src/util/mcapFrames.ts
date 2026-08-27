/**
 * mcapFrames.ts — `.mcap` からカメラフレーム(JPEG)を取り出す。**ブラウザ内で完結。**
 *
 * `raspi/rec/mcap_log.py`（Pi側の書き手）が `/viz/image/front`・`/viz/image/rear` に
 * `foxglove.CompressedImage`（JSON + base64 JPEG。`compressed_image()` 参照）として
 * 書き込む形式と対になる読み手。`ml/extract_frames.py`（Python版・大量の `.mcap` を
 * まとめて処理する用）と同じ仕事を、ここではブラウザだけでできるようにする——
 * 録画したその場で Python 環境を用意しなくてもフレームを取り出せる。
 *
 * **zstd 圧縮のみ対応。** GUI から始めた録画は常にこの圧縮で書かれる
 * （`raspi/rec/mcap_log.py` の既定 `compression="zstd"`）。それ以外の圧縮の
 * `.mcap` を渡すとエラーメッセージ付きで失敗する（無圧縮or lz4のファイルは
 * 今のところ Python 版 `ml/extract_frames.py` を使うこと）。
 */
import { McapStreamReader } from '@mcap/core'
import * as fzstd from 'fzstd'

//: `raspi/rec/mcap_log.py` の `VIZ_IMAGE_PREFIX` と同じ値。**そちらが正**
const VIZ_IMAGE_PREFIX = '/viz/image/'

export interface ExtractedFrame {
  /** ファイル名（カメラ名 + 撮像時刻）。**重複しない前提**（同一時刻に同一カメラが2枚は無い） */
  name: string
  cam: string
  tCaptureNs: bigint
  data: Uint8Array
}

function base64ToBytes(b64: string): Uint8Array {
  const bin = atob(b64)
  const out = new Uint8Array(bin.length)
  for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i)
  return out
}

/**
 * `.mcap` のバイト列からカメラフレームを取り出す。
 *
 * @param cams 取り出すカメラ名の集合（`"front"` / `"rear"`）。
 * @param minIntervalNs カメラごとに、直前に採用したフレームからこの時間未満
 *   しか経っていないフレームを間引く（既定 0n = 間引きなし）。連続フレームは
 *   ほぼ同じ構図になりがちで、全件出すと `ml/annotate.py` での手間が
 *   そのまま比例して増えるため（`ml/extract_frames.py` の `--min-interval-ms`
 *   と同じ考え方）。
 * @throws 未対応の圧縮方式（zstd 以外）が含まれていた場合
 */
export function extractFramesFromMcap(
  buf: ArrayBuffer,
  cams: Set<string>,
  minIntervalNs: bigint = 0n,
): ExtractedFrame[] {
  const reader = new McapStreamReader({
    decompressHandlers: {
      zstd: (data, decompressedSize) => fzstd.decompress(data, new Uint8Array(Number(decompressedSize))),
    },
  })
  reader.append(new Uint8Array(buf))

  const channelTopics = new Map<number, string>()
  const frames: ExtractedFrame[] = []
  const lastByCam = new Map<string, bigint>()
  const decoder = new TextDecoder()

  for (let rec; (rec = reader.nextRecord()); ) {
    if (rec.type === 'Channel') {
      channelTopics.set(rec.id, rec.topic)
      continue
    }
    if (rec.type !== 'Message') continue
    const topic = channelTopics.get(rec.channelId)
    if (!topic || !topic.startsWith(VIZ_IMAGE_PREFIX)) continue
    const cam = topic.slice(VIZ_IMAGE_PREFIX.length)
    if (!cams.has(cam)) continue

    const obj = JSON.parse(decoder.decode(rec.data)) as { data: string; timestamp?: { sec: number; nsec: number } }
    const tCaptureNs = obj.timestamp
      ? BigInt(obj.timestamp.sec) * 1_000_000_000n + BigInt(obj.timestamp.nsec)
      : rec.logTime

    const last = lastByCam.get(cam)
    if (last !== undefined && tCaptureNs - last < minIntervalNs) continue
    lastByCam.set(cam, tCaptureNs)

    const jpg = base64ToBytes(obj.data)
    frames.push({ name: `${cam}_${tCaptureNs}.jpg`, cam, tCaptureNs, data: jpg })
  }
  return frames
}
