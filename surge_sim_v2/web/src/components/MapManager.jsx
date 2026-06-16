import { useEffect, useState } from "react";
import { apiGet, apiPost } from "../constants/modes";

// 保存済みマップ一覧・読込UI（MapBuilding時に表示）。
export default function MapManager() {
  const [maps, setMaps] = useState([]);

  const refresh = async () => {
    try {
      const res = await apiGet("/api/maps");
      setMaps(res.maps || []);
    } catch {
      setMaps([]);
    }
  };

  useEffect(() => {
    refresh();
  }, []);

  const load = async (name) => {
    await apiPost("/api/map/load", { name });
  };

  return (
    <div className="mt-2 bg-black/30 rounded p-2">
      <div className="flex justify-between items-center mb-1">
        <span className="text-sm font-semibold">保存済みマップ</span>
        <button onClick={refresh} className="text-xs text-gray-400 hover:text-white">
          ↻ 更新
        </button>
      </div>
      {maps.length === 0 ? (
        <div className="text-xs text-gray-500">保存済みマップなし</div>
      ) : (
        <ul className="space-y-1">
          {maps.map((m) => (
            <li key={m} className="flex justify-between items-center text-sm">
              <span className="font-mono">{m}</span>
              <button
                onClick={() => load(m)}
                className="text-xs px-2 py-0.5 rounded bg-gray-700 hover:bg-gray-600"
              >
                読込
              </button>
            </li>
          ))}
        </ul>
      )}
    </div>
  );
}
