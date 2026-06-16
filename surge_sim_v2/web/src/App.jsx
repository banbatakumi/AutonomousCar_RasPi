import { useWebSocket } from "./hooks/useWebSocket";
import { useKeyboard } from "./hooks/useKeyboard";
import { MODES } from "./constants/modes";
import StatusBar from "./components/StatusBar.jsx";
import ModeSelector from "./components/ModeSelector.jsx";
import Dashboard from "./components/Dashboard.jsx";
import LidarView from "./components/LidarView.jsx";
import SlamMap from "./components/SlamMap.jsx";
import ConnectionMonitor from "./components/ConnectionMonitor.jsx";
import Controller from "./components/Controller.jsx";

export default function App() {
  const { systemState, connected, latency, retriesExhausted, send, reconnect } =
    useWebSocket();

  const mode = systemState?.mode ?? MODES.MANUAL;
  // 手動操作はManual/MapBuilding時のみ有効
  const keyboardEnabled = mode === MODES.MANUAL || mode === MODES.MAP_BUILDING;
  const cmd = useKeyboard(send, keyboardEnabled);

  // SIM/REAL 判定：UART/STM32 が接続されていれば REAL とみなす
  const conn = systemState?.connection;
  const isSim = !(conn?.uart_connected || conn?.stm32_connected);

  return (
    <div className="h-full flex flex-col bg-surge-bg text-gray-200">
      <StatusBar state={systemState} connected={connected} isSim={isSim} />

      {retriesExhausted && (
        <div className="bg-surge-danger/80 text-white text-sm px-4 py-2 flex items-center justify-between">
          <span>接続が切れました。再接続できません。</span>
          <button onClick={reconnect} className="px-3 py-1 rounded bg-black/30">
            手動再接続
          </button>
        </div>
      )}

      <div className="flex-1 grid grid-cols-[1fr_400px] gap-3 p-3 overflow-hidden">
        {/* 左：SLAM MAP */}
        <div className="overflow-hidden">
          <SlamMap state={systemState} />
        </div>

        {/* 右：操作系パネル */}
        <div className="flex flex-col gap-3 overflow-y-auto">
          <ModeSelector state={systemState} />
          <Dashboard state={systemState} />
          <LidarView state={systemState} />
          <ConnectionMonitor
            state={systemState}
            connected={connected}
            latency={latency}
            isSim={isSim}
          />
        </div>
      </div>

      {/* 下：CONTROLLER */}
      <div className="p-3 pt-0">
        <Controller state={systemState} isSim={isSim} cmd={cmd} />
      </div>
    </div>
  );
}
