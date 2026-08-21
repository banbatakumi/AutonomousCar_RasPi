/**
 * `/ws/map` — 地図と経路を受けて `live.map` に置く。**変わったときだけ届く。**
 *
 * 点群やカメラと違って地図は「育っている間だけ 1Hz、確定したら二度と来ない」
 * ので、**繋いだ瞬間にサーバが最新の1枚を送ってくる**（`telemetry_node._map_channel`）。
 * そうしないと、走り終わってから画面を開いた人は何も見られない。
 *
 * ## 展開はブラウザ標準の機能だけで済ませる
 *
 * サーバは 3 値（未知/空き/占有）を 2bit に詰めて `zlib` で圧縮している。
 * `DecompressionStream('deflate')` がそのまま zlib 形式を読めるので、
 * **フロントに解凍ライブラリを増やさなくてよい**（`'deflate-raw'` ではない。
 * `zlib.compress` はヘッダ付きなので `'deflate'` が正しい）。
 *
 * ## ImageData への焼き付けはここで1回だけやる
 *
 * 400×400 の展開と着色を rAF の中でやると 60fps を割る。**受信のたびに 1 回**
 * だけ焼いて `ImageBitmap` にし、描画側は貼るだけにする。
 */
import { decode } from '@msgpack/msgpack'
import { live } from '../bus/live'
import type { AutoMapMsg, MapData } from '../types'
import { wsUrl } from './url'

const RECONNECT_MIN_MS = 300
const RECONNECT_MAX_MS = 3000

/** 0=未知 1=空き 2=占有。**未知と空きは必ず別の色にする。**
 *  同じ色にすると「まだ見ていない」と「何も無いと分かっている」の区別が消え、
 *  動的障害物の誤検出をデバッグできなくなる。 */
/**
 * 2bit の値 → 色。**添字は 0〜3 の4通りすべてを埋める。**
 *
 * Python 側（`AutoMap.cells`）が書くのは 0/1/2 だけだが、圧縮データが途中で
 * 壊れれば 3 も出る。3 要素だと `PALETTE[3]` が `undefined` になり、
 * 直後の分割代入がその場で例外を投げてフレームごと描画が止まる。
 * **壊れた1セルのために地図全体が消えるほうが困る**ので、未知として塗る。
 */
const PALETTE: readonly [
  readonly [number, number, number],
  readonly [number, number, number],
  readonly [number, number, number],
  readonly [number, number, number],
] = [
  [13, 17, 20], // 0 未知
  [26, 36, 46], // 1 空き
  [134, 169, 196], // 2 占有
  [13, 17, 20], // 3 ありえない値。未知として塗る
]

async function inflate(data: Uint8Array): Promise<Uint8Array> {
  const stream = new Blob([data as BlobPart]).stream().pipeThrough(new DecompressionStream('deflate'))
  return new Uint8Array(await new Response(stream).arrayBuffer())
}

async function build(msg: AutoMapMsg): Promise<MapData | null> {
  const { width, height } = msg
  if (!width || !height) return null

  const packed = await inflate(new Uint8Array(msg.cells))
  const n = width * height
  const cells = new Uint8Array(n)
  // 展開後が短ければ末尾は 0（未知）のまま。**途中で例外にせず地図は出す**
  for (let i = 0; i < n; i++) cells[i] = ((packed[i >> 2] ?? 0) >> ((i & 3) * 2)) & 3

  // 行0 は y が最小。画面は下に向かって y が増えるので**上下を入れ替えて焼く**
  const img = new ImageData(width, height)
  const px = img.data
  let minC = width, maxC = -1, minR = height, maxR = -1
  for (let r = 0; r < height; r++) {
    const src = r * width
    const dst = (height - 1 - r) * width
    for (let c = 0; c < width; c++) {
      const v = (cells[src + c] ?? 0) as 0 | 1 | 2 | 3   // 2bit なので必ずこの4値
      const [rr, gg, bb] = PALETTE[v]
      const o = (dst + c) * 4
      px[o] = rr
      px[o + 1] = gg
      px[o + 2] = bb
      px[o + 3] = 255
      if (v !== 0) {
        if (c < minC) minC = c
        if (c > maxC) maxC = c
        if (r < minR) minR = r
        if (r > maxR) maxR = r
      }
    }
  }

  // **観測できた範囲だけを画面に合わせる。** 地図の枠（24m四方）に合わせると、
  // 10m のコースが画面の隅で豆粒になる
  const known =
    maxC >= minC
      ? {
          x0: msg.origin_x + minC * msg.resolution,
          y0: msg.origin_y + minR * msg.resolution,
          x1: msg.origin_x + (maxC + 1) * msg.resolution,
          y1: msg.origin_y + (maxR + 1) * msg.resolution,
        }
      : null

  return {
    seq: msg.map_seq,
    res: msg.resolution,
    originX: msg.origin_x,
    originY: msg.origin_y,
    width,
    height,
    bitmap: await createImageBitmap(img),
    centerline: Float64Array.from(msg.centerline ?? []),
    raceline: Float64Array.from(msg.raceline ?? []),
    racelineV: Float64Array.from(msg.raceline_v ?? []),
    known,
  }
}

export function connectMap(onOpenChange: (open: boolean) => void): () => void {
  let ws: WebSocket | null = null
  let timer: number | undefined
  let backoff = RECONNECT_MIN_MS
  let closed = false

  const open = () => {
    ws = new WebSocket(wsUrl('/ws/map'))
    ws.binaryType = 'arraybuffer'
    ws.onopen = () => {
      backoff = RECONNECT_MIN_MS
      onOpenChange(true)
    }
    ws.onmessage = async (ev) => {
      if (!(ev.data instanceof ArrayBuffer)) return
      try {
        const msg = decode(new Uint8Array(ev.data)) as AutoMapMsg
        const built = await build(msg)
        // **古い版で新しい版を上書きしない。** 展開は非同期なので、続けて
        // 届いた2枚の完成順が入れ替わりうる
        if (built && (!live.map || built.seq >= live.map.seq)) {
          live.map?.bitmap?.close()
          live.map = built
        }
      } catch {
        // 壊れた1枚は捨てる。**次の版が来れば直る**ので接続は切らない
      }
    }
    ws.onclose = () => {
      onOpenChange(false)
      if (closed) return
      timer = window.setTimeout(open, backoff)
      backoff = Math.min(backoff * 2, RECONNECT_MAX_MS)
    }
    ws.onerror = () => ws?.close()
  }
  open()

  return () => {
    closed = true
    window.clearTimeout(timer)
    ws?.close()
  }
}
