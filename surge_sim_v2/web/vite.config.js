import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// VITE_WS_URL を .env.local（シミュ）/ .env.production（実機）で切り替える。
//   .env.local      : VITE_WS_URL=ws://localhost:8000/ws
//   .env.production : VITE_WS_URL=ws://192.168.x.x:8000/ws
export default defineConfig({
  plugins: [react()],
  server: {
    port: 5173,
    proxy: {
      // 開発時は API を FastAPI(8000) にプロキシ
      "/api": "http://localhost:8000",
    },
  },
  build: {
    outDir: "dist",
  },
});
