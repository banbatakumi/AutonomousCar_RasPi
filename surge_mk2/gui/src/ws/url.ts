/**
 * WebSocket の接続先。**同一オリジンに固定する。**
 *
 * 開発中は Vite の dev サーバが `/ws` を Pi に中継する（`vite.config.ts`）。
 * 本番は telemetry_node が GUI 本体も配るので、どちらでも同じコードで繋がる。
 * ここにホスト名を書くと「手元では動くのに Pi では繋がらない」を必ず踏む。
 */
export function wsUrl(path: string): string {
  const proto = location.protocol === 'https:' ? 'wss:' : 'ws:'
  return `${proto}//${location.host}${path}`
}
