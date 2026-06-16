import { useEffect, useState } from "react";
import { apiPost } from "../constants/modes";

// 画面上部固定バー：接続状態・モード・REC・EMERGENCY STOP。
export default function StatusBar({ state, connected, isSim }) {
  const [recording, setRecording] = useState(false);
  const [blink, setBlink] = useState(false);

  useEffect(() => {
    setRecording(Boolean(state?.is_recording));
  }, [state?.is_recording]);

  useEffect(() => {
    if (!recording) return;
    const t = setInterval(() => setBlink((b) => !b), 500);
    return () => clearInterval(t);
  }, [recording]);

  const toggleRec = async () => {
    if (recording) await apiPost("/api/log/stop");
    else await apiPost("/api/log/start");
  };

  const emergencyStop = async () => {
    await apiPost("/api/emergency_stop");
  };

  const modeLabel = state?.mode ?? "-";

  return (
    <div className="flex items-center justify-between bg-surge-panel px-4 py-2 border-b border-black/40">
      <div className="flex items-center gap-4">
        <span className="font-bold text-lg">SURGE Mark.2</span>
        <span className="flex items-center gap-1 text-sm">
          <span className={connected ? "text-surge-ok" : "text-surge-danger"}>●</span>
          {connected ? "Connected" : "Disconnected"}
          <span className="ml-1 px-1.5 py-0.5 rounded bg-black/40 text-xs">
            {isSim ? "SIM" : "REAL"}
          </span>
        </span>
        <span className="text-sm text-gray-300">mode: {modeLabel}</span>
      </div>

      <div className="flex items-center gap-3">
        <button
          onClick={toggleRec}
          className={`px-3 py-1 rounded text-sm font-semibold ${
            recording ? "bg-surge-danger" : "bg-gray-700"
          }`}
        >
          {recording ? (blink ? "● REC" : "○ REC") : "● REC"}
        </button>
        <button
          onClick={emergencyStop}
          className="px-3 py-1 rounded bg-surge-danger text-white font-bold text-sm animate-pulse"
        >
          🔴 EMERGENCY STOP
        </button>
      </div>
    </div>
  );
}
