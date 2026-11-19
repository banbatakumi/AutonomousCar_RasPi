/**
 * JPEG を `createImageBitmap` でデコードする。**デコード中に届いたものは最新の1枚だけ残す**
 * （`CameraView.tsx` の effect から分けた。技術的負債の棚卸し 2026-11-18 高6）。
 *
 * - 溜めると遅延が伸び続け、届いた側を捨てると次の到着まで1フレーム分遅れる。だから「最新の1枚」
 * - 世代（`gen`）で、固まりとみなして見捨てた古いデコードが後から終わっても状態に触らせない
 * - `close()` の後に終わったデコードの絵は解放して捨てる（解放しないとリークする）
 *
 * `keepPending=false` はデコード中に届いたものを捨てる（マスクの従来の挙動）。
 */
export class LatestFrameDecoder {
  private decoding = false
  private decodeStartMs = 0
  private gen = 0
  /** デコード中に届いた最新の1枚 */
  private pending: ArrayBuffer | null = null
  private closed = false

  constructor(
    /** デコードできた1枚。所有権は受け取った側に移る（不要になったら `close()` すること） */
    private readonly onFrame: (bmp: ImageBitmap) => void,
    private readonly keepPending = true,
  ) {}

  push(buf: ArrayBuffer): void {
    if (this.closed) return
    if (this.decoding) {
      if (this.keepPending) this.pending = buf
      return
    }
    void this.decode(buf)
  }

  /** デコードが `stuckMs` 終わらなければ見捨てて、待っていた1枚へ進む（watchdog から呼ぶ） */
  checkStuck(now: number, stuckMs: number): void {
    if (this.closed || !this.decoding || now - this.decodeStartMs <= stuckMs) return
    this.gen++ // 終わらないデコードは見捨てる
    this.decoding = false
    const next = this.pending
    this.pending = null
    if (next) void this.decode(next)
  }

  close(): void {
    this.closed = true
    this.pending = null
  }

  private async decode(buf: ArrayBuffer): Promise<void> {
    this.decoding = true
    this.decodeStartMs = performance.now()
    const gen = ++this.gen
    try {
      const bmp = await createImageBitmap(new Blob([buf], { type: 'image/jpeg' }))
      if (this.closed) {
        bmp.close()
        return
      }
      this.onFrame(bmp)
    } catch {
      /* 壊れたフレームは捨てる */
    } finally {
      // 固まりとみなして先へ進めたあとに終わった古いデコードは、状態に触らない
      if (gen === this.gen) this.decoding = false
    }
    if (gen !== this.gen) return
    const next = this.pending
    this.pending = null
    if (next && !this.closed) void this.decode(next)
  }
}
