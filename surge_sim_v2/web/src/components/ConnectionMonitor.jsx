// 通信状態監視パネル：WS/UART/LiDAR/STM32/最終受信経過。
export default function ConnectionMonitor({ state, connected, latency, isSim }) {
  const conn = state?.connection;
  const now = Date.now() / 1000;
  const lastRx = conn?.last_received_at;
  const elapsed = lastRx ? Math.max(0, now - lastRx) : null;

  const Dot = ({ ok, dim }) => (
    <span className={dim ? "text-gray-600" : ok ? "text-surge-ok" : "text-surge-danger"}>●</span>
  );

  const lidarOk = Boolean(conn?.lidar_receiving);
  const wsAbnormal = !connected;
  const lidarAbnormal = !lidarOk;

  return (
    <div className="bg-surge-panel rounded p-3 text-sm">
      <div className="font-semibold mb-2">CONNECTION MONITOR</div>
      <div className={`flex justify-between py-0.5 ${wsAbnormal ? "bg-red-900/40 rounded px-1" : ""}`}>
        <span>
          <Dot ok={connected} /> WS
        </span>
        <span className="font-mono">Latency: {latency != null ? `${latency.toFixed(0)}ms` : "-"}</span>
      </div>
      <div className="flex justify-between py-0.5">
        <span>
          <Dot ok={conn?.uart_connected} dim={isSim} /> UART
        </span>
        <span className={`flex items-center gap-1 ${lidarAbnormal ? "text-surge-danger" : ""}`}>
          <Dot ok={lidarOk} /> LiDAR {lidarOk ? "OK" : "-"}
        </span>
      </div>
      <div className="flex justify-between py-0.5">
        <span>
          <Dot ok={conn?.stm32_connected} dim={isSim} /> STM32
        </span>
        <span className="font-mono">
          Last RX: {elapsed == null ? "-" : `${elapsed.toFixed(1)}s ago`}
        </span>
      </div>
    </div>
  );
}
