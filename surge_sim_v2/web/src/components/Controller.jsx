import { useState } from "react";
import { MODES, apiPost } from "../constants/modes";
import MapManager from "./MapManager.jsx";

// モードに応じて操作UIを切替。
//   Manual       : キーガイド＋現在の指令値表示
//   MapBuilding  : 操作＋地図保存/リセット＋MapManager
//   Autonomous   : スタート/ストップ＋目標速度スライダー
export default function Controller({ state, isSim, cmd }) {
  const mode = state?.mode ?? MODES.MANUAL;
  const [targetSpeed, setTargetSpeed] = useState(1.0);

  const autonomousRunning = Boolean(state?.autonomous_running);
  const paused = Boolean(state?.is_paused);
  const speedMult = state?.speed_multiplier ?? 1.0;

  // --- シミュ専用コントロール ---
  const SimControls = () =>
    isSim ? (
      <div className="flex items-center gap-2">
        <button
          onClick={() => apiPost("/api/sim/pause")}
          className="px-3 py-1 rounded bg-gray-700 hover:bg-gray-600 text-sm"
        >
          {paused ? "▶ Resume" : "⏸ Pause"}
        </button>
        <button
          onClick={() => apiPost("/api/sim/reset")}
          className="px-3 py-1 rounded bg-gray-700 hover:bg-gray-600 text-sm"
        >
          ↺ Reset
        </button>
        <label className="text-sm text-gray-400">Speed:</label>
        <select
          value={speedMult}
          onChange={(e) => apiPost("/api/sim/speed", { multiplier: Number(e.target.value) })}
          className="bg-gray-800 rounded px-2 py-1 text-sm"
        >
          <option value={0.5}>0.5x</option>
          <option value={1.0}>1x</option>
          <option value={2.0}>2x</option>
        </select>
      </div>
    ) : null;

  const KeyGuide = () => (
    <div className="text-sm text-gray-400">
      ↑↓: Speed(0.1m/s) ←→: Steer(5°) ESC: Emergency Stop
      <span className="ml-3 font-mono text-gray-200">
        cmd: {cmd.targetSpeed.toFixed(1)} m/s / {cmd.targetSteer.toFixed(0)}°
      </span>
    </div>
  );

  const saveMap = async () => {
    const name = window.prompt("保存するマップ名を入力してください", "map_01");
    if (!name) return;
    await apiPost("/api/map/save", { name });
  };

  const resetMap = async () => {
    if (!window.confirm("地図をリセットしますか？")) return;
    await apiPost("/api/map/reset");
  };

  return (
    <div className="bg-surge-panel rounded p-3">
      <div className="flex items-center justify-between flex-wrap gap-2">
        <div className="font-semibold">CONTROLLER</div>
        <SimControls />
      </div>

      <div className="mt-2">
        {mode === MODES.MANUAL && <KeyGuide />}

        {mode === MODES.MAP_BUILDING && (
          <div>
            <KeyGuide />
            <div className="flex gap-2 mt-2">
              <button onClick={saveMap} className="px-3 py-1 rounded bg-surge-accent text-sm">
                地図を保存
              </button>
              <button onClick={resetMap} className="px-3 py-1 rounded bg-gray-700 text-sm">
                地図をリセット
              </button>
            </div>
            <MapManager />
          </div>
        )}

        {mode === MODES.AUTONOMOUS && (
          <div className="flex items-center gap-3 flex-wrap">
            <button
              onClick={() => apiPost("/api/autonomous/start", { target_speed: targetSpeed })}
              disabled={autonomousRunning}
              className={`px-4 py-1 rounded text-sm font-semibold ${
                autonomousRunning ? "bg-gray-800 text-gray-600" : "bg-surge-ok text-black"
              }`}
            >
              スタート
            </button>
            <button
              onClick={() => apiPost("/api/autonomous/stop")}
              className="px-4 py-1 rounded bg-surge-danger text-sm font-semibold"
            >
              ストップ
            </button>
            <label className="text-sm text-gray-400">目標速度: {targetSpeed.toFixed(1)} m/s</label>
            <input
              type="range"
              min={0}
              max={3}
              step={0.1}
              value={targetSpeed}
              onChange={(e) => setTargetSpeed(Number(e.target.value))}
              className="flex-1 min-w-[120px]"
            />
          </div>
        )}
      </div>
    </div>
  );
}
