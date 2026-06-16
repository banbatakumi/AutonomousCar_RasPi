import { useEffect, useRef } from "react";

const MAX_RANGE = 12.0;

// LiDAR 生データを極座標で表示。中心=車両、近い点=赤→遠い点=青。
export default function LidarView({ state }) {
  const canvasRef = useRef(null);
  const lidar = state?.lidar;
  const heading = state?.vehicle?.heading ?? 0;

  useEffect(() => {
    const cv = canvasRef.current;
    if (!cv) return;
    const ctx = cv.getContext("2d");
    const W = cv.width;
    const H = cv.height;
    const cx = W / 2;
    const cy = H / 2;
    const R = Math.min(W, H) / 2 - 8;

    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = "#0d0d12";
    ctx.fillRect(0, 0, W, H);

    // 距離リング（3,6,9,12m）
    ctx.strokeStyle = "#2a2a33";
    for (let r = 3; r <= 12; r += 3) {
      ctx.beginPath();
      ctx.arc(cx, cy, (r / MAX_RANGE) * R, 0, Math.PI * 2);
      ctx.stroke();
    }

    if (!lidar?.distances?.length) {
      ctx.fillStyle = "#888";
      ctx.font = "14px monospace";
      ctx.textAlign = "center";
      ctx.fillText("LiDAR未接続", cx, cy);
      return;
    }

    const { distances, angles } = lidar;
    for (let i = 0; i < distances.length; i++) {
      const d = distances[i];
      if (d >= MAX_RANGE - 1e-3) continue;
      // 上を前方（heading 方向）に。極座標：角度はセンサ相対角。
      const a = (angles[i] * Math.PI) / 180 - Math.PI / 2;
      const rr = (d / MAX_RANGE) * R;
      const x = cx + rr * Math.cos(a);
      const y = cy + rr * Math.sin(a);
      // 近い=赤(0) → 遠い=青(240)
      const hue = (d / MAX_RANGE) * 240;
      ctx.fillStyle = `hsl(${hue}, 90%, 55%)`;
      ctx.fillRect(x - 1.5, y - 1.5, 3, 3);
    }

    // 車両（中心の三角・前方=上）
    ctx.fillStyle = "#3c82f0";
    ctx.beginPath();
    ctx.moveTo(cx, cy - 7);
    ctx.lineTo(cx - 5, cy + 5);
    ctx.lineTo(cx + 5, cy + 5);
    ctx.closePath();
    ctx.fill();
  }, [lidar, heading]);

  return (
    <div className="bg-surge-panel rounded p-3">
      <div className="font-semibold mb-2">LIDAR VIEW</div>
      <canvas
        ref={canvasRef}
        width={300}
        height={240}
        className="w-full rounded bg-black"
      />
    </div>
  );
}
