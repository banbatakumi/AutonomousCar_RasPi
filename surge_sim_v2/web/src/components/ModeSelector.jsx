import { MODE_LABELS, MODE_LIST, MODES, apiPost } from "../constants/modes";

// Manual / MapBuilding / Autonomous の切替。
// Autonomous は slam_map がある時のみ有効。切替時に確認ダイアログ。
export default function ModeSelector({ state }) {
  const currentMode = state?.mode ?? MODES.MANUAL;
  // Phase2: course_map（カンニング中心線）でも追従可能。Phase3 で slam_map が加わる。
  const hasMap = state?.slam_map != null || state?.course_map != null;

  const changeMode = async (mode) => {
    if (mode === currentMode) return;
    if (mode === MODES.AUTONOMOUS && !hasMap) return;
    const ok = window.confirm(`モードを「${MODE_LABELS[mode]}」に切り替えますか？`);
    if (!ok) return;
    await apiPost("/api/mode", { mode });
  };

  return (
    <div className="flex gap-2 p-2 bg-surge-panel rounded">
      {MODE_LIST.map((mode) => {
        const active = mode === currentMode;
        const disabled = mode === MODES.AUTONOMOUS && !hasMap;
        return (
          <button
            key={mode}
            onClick={() => changeMode(mode)}
            disabled={disabled}
            className={`flex-1 py-2 rounded text-sm font-semibold transition ${
              active
                ? "bg-surge-accent text-white"
                : disabled
                ? "bg-gray-800 text-gray-600 cursor-not-allowed"
                : "bg-gray-700 hover:bg-gray-600"
            }`}
            title={disabled ? "地図/経路の準備後に有効になります" : ""}
          >
            {MODE_LABELS[mode]}
          </button>
        );
      })}
    </div>
  );
}
