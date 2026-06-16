import { useCallback, useEffect, useRef, useState } from "react";

// 接続先 URL の決定：
//  1. VITE_WS_URL が指定されていればそれを使う（Vite 開発サーバー時など）
//  2. 無ければ「今開いているページのホスト」から導出する
//     → FastAPI が配信するビルドなら sim=localhost / 実機=Pi の IP に自動追従
function resolveWsUrl() {
  const env = import.meta.env.VITE_WS_URL;
  if (env) return env;
  const proto = window.location.protocol === "https:" ? "wss:" : "ws:";
  return `${proto}//${window.location.host}/ws`;
}

const WS_URL = resolveWsUrl();
const MAX_RETRIES = 5;
const RETRY_DELAY_MS = 3000;
const PING_INTERVAL_MS = 5000;

// WebSocket 接続・自動再接続・レイテンシ計測を管理するフック。
export function useWebSocket() {
  const [systemState, setSystemState] = useState(null);
  const [connected, setConnected] = useState(false);
  const [latency, setLatency] = useState(0);
  const [retriesExhausted, setRetriesExhausted] = useState(false);

  const wsRef = useRef(null);
  const retryCountRef = useRef(0);
  const pingTimerRef = useRef(null);
  const reconnectTimerRef = useRef(null);
  const manualCloseRef = useRef(false);

  const send = useCallback((message) => {
    const ws = wsRef.current;
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(message));
      return true;
    }
    return false;
  }, []);

  const connect = useCallback(() => {
    manualCloseRef.current = false;
    setRetriesExhausted(false);
    const ws = new WebSocket(WS_URL);
    wsRef.current = ws;

    ws.onopen = () => {
      setConnected(true);
      retryCountRef.current = 0;
      // ping ループ開始
      if (pingTimerRef.current) clearInterval(pingTimerRef.current);
      pingTimerRef.current = setInterval(() => {
        send({ type: "ping", timestamp: performance.now() });
      }, PING_INTERVAL_MS);
    };

    ws.onmessage = (ev) => {
      let msg;
      try {
        msg = JSON.parse(ev.data);
      } catch {
        return;
      }
      if (msg.type === "state") {
        setSystemState(msg);
      } else if (msg.type === "pong") {
        if (typeof msg.timestamp === "number") {
          setLatency(performance.now() - msg.timestamp);
        }
      }
    };

    ws.onclose = () => {
      setConnected(false);
      if (pingTimerRef.current) clearInterval(pingTimerRef.current);
      if (manualCloseRef.current) return;
      if (retryCountRef.current < MAX_RETRIES) {
        retryCountRef.current += 1;
        reconnectTimerRef.current = setTimeout(connect, RETRY_DELAY_MS);
      } else {
        setRetriesExhausted(true);
      }
    };

    ws.onerror = () => {
      ws.close();
    };
  }, [send]);

  const reconnect = useCallback(() => {
    retryCountRef.current = 0;
    connect();
  }, [connect]);

  useEffect(() => {
    connect();
    return () => {
      manualCloseRef.current = true;
      if (pingTimerRef.current) clearInterval(pingTimerRef.current);
      if (reconnectTimerRef.current) clearTimeout(reconnectTimerRef.current);
      if (wsRef.current) wsRef.current.close();
    };
  }, [connect]);

  return { systemState, connected, latency, retriesExhausted, send, reconnect };
}
