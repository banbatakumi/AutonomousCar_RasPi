import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// 開発中は Vite の dev サーバ(5173)で動かし、WebSocket だけ Pi へ中継する。
// こうすると GUI と WS が同一オリジンのままなので、
// **本番(Pi の telemetry_node が配る)と開発で接続先の書き分けが要らない。**
//
//   SURGE_HOST=surge.local npm run dev     # 実車に繋ぐ
//   npm run dev                            # 手元の telemetry_node (localhost:8000)
const host = process.env.SURGE_HOST ?? 'localhost'
const port = process.env.SURGE_PORT ?? '8000'

export default defineConfig({
  plugins: [react()],
  // telemetry_node がどのパスから配っても動くように相対パスで吐く
  base: './',
  build: { outDir: 'dist', emptyOutDir: true },
  server: {
    proxy: {
      '/ws': { target: `ws://${host}:${port}`, ws: true, changeOrigin: true },
    },
  },
})
