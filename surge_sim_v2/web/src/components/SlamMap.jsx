import { useEffect, useRef } from "react";
import { MODES } from "../constants/modes";

// SLAMマップ＋最適ライン表示。slam_map=null ならグレー背景に「地図未生成」。
export default function SlamMap({ state }) {
  const canvasRef = useRef(null);
  const slam = state?.slam_map;
  const loc = state?.localization;
  const courseMap = state?.course_map;
  const mode = state?.mode;

  useEffect(() => {
    const cv = canvasRef.current;
    if (!cv) return;
    const ctx = cv.getContext("2d");
    const W = cv.width;
    const H = cv.height;
    ctx.clearRect(0, 0, W, H);

    if (slam == null) {
      ctx.fillStyle = "#2a2a30";
      ctx.fillRect(0, 0, W, H);
      // Phase2: SLAM 地図は無いが course_map（カンニング経路）があれば描画
      if (courseMap?.center_line?.length) {
        drawCourseMap(ctx, W, H, courseMap, loc, mode);
        ctx.fillStyle = "#888";
        ctx.font = "12px sans-serif";
        ctx.textAlign = "left";
        ctx.fillText("SLAM地図未生成（Phase2: カンニング経路を表示）", 8, 18);
      } else {
        ctx.fillStyle = "#888";
        ctx.font = "20px sans-serif";
        ctx.textAlign = "center";
        ctx.fillText("地図未生成", W / 2, H / 2);
      }
      return;
    }

    ctx.fillStyle = "#0d0d12";
    ctx.fillRect(0, 0, W, H);

    const grid = slam.grid;
    const rows = grid.length;
    const cols = grid[0]?.length ?? 0;
    if (!rows || !cols) return;

    // 自動スケーリング（Canvas全体に収める）
    const scale = Math.min(W / cols, H / rows);
    const offX = (W - cols * scale) / 2;
    const offY = (H - rows * scale) / 2;

    // 占有格子描画：-1=グレー, 0=白, 100=黒
    for (let r = 0; r < rows; r++) {
      for (let c = 0; c < cols; c++) {
        const v = grid[r][c];
        if (v === -1) ctx.fillStyle = "#555";
        else if (v >= 100 || v >= 50) ctx.fillStyle = "#111";
        else ctx.fillStyle = "#e8e8e8";
        // 行は下から上に（ワールドy上向き）
        const sy = offY + (rows - 1 - r) * scale;
        ctx.fillRect(offX + c * scale, sy, scale + 0.5, scale + 0.5);
      }
    }

    // ワールド→Canvas変換
    const res = slam.resolution || 0.05;
    const w2s = (wx, wy) => {
      const cxg = (wx - slam.origin_x) / res;
      const cyg = (wy - slam.origin_y) / res;
      return [offX + cxg * scale, offY + (rows - 1 - cyg) * scale];
    };

    // Autonomous時：racing_line を黄色でオーバーレイ
    if (mode === MODES.AUTONOMOUS && courseMap?.racing_line?.length) {
      ctx.strokeStyle = "#fbbf24";
      ctx.lineWidth = 2;
      ctx.beginPath();
      courseMap.racing_line.forEach((p, i) => {
        const [x, y] = w2s(p[0], p[1]);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();
    }

    // 車両マーカー（青三角形）
    if (loc) {
      const [vx, vy] = w2s(loc.x, loc.y);
      const h = (loc.heading * Math.PI) / 180;
      ctx.fillStyle = "#3c82f0";
      ctx.beginPath();
      const s = 8;
      ctx.moveTo(vx + s * Math.cos(-h), vy + s * Math.sin(-h));
      ctx.lineTo(vx + s * 0.6 * Math.cos(-h + 2.5), vy + s * 0.6 * Math.sin(-h + 2.5));
      ctx.lineTo(vx + s * 0.6 * Math.cos(-h - 2.5), vy + s * 0.6 * Math.sin(-h - 2.5));
      ctx.closePath();
      ctx.fill();
    }
  }, [slam, loc, courseMap, mode]);

  return (
    <div className="bg-surge-panel rounded p-3 h-full flex flex-col">
      <div className="font-semibold mb-2">SLAM MAP</div>
      <canvas
        ref={canvasRef}
        width={560}
        height={560}
        className="w-full flex-1 rounded"
      />
    </div>
  );
}

// Phase2 用：course_map（左右壁・中心線・追従ライン）と車両を自動スケールで描画。
function drawCourseMap(ctx, W, H, courseMap, loc, mode) {
  const all = [
    ...(courseMap.left_wall || []),
    ...(courseMap.right_wall || []),
    ...(courseMap.center_line || []),
  ];
  if (all.length === 0) return;

  const xs = all.map((p) => p[0]);
  const ys = all.map((p) => p[1]);
  const minX = Math.min(...xs);
  const maxX = Math.max(...xs);
  const minY = Math.min(...ys);
  const maxY = Math.max(...ys);
  const margin = 0.4;
  const spanX = maxX - minX + 2 * margin;
  const spanY = maxY - minY + 2 * margin;
  const scale = Math.min(W / spanX, H / spanY);
  const offX = (W - spanX * scale) / 2;
  const offY = (H - spanY * scale) / 2;
  const w2s = (x, y) => [
    offX + (x - minX + margin) * scale,
    H - (offY + (y - minY + margin) * scale), // y 反転
  ];

  const stroke = (pts, color, width, dashed, closed) => {
    if (!pts || pts.length < 2) return;
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.setLineDash(dashed ? [6, 6] : []);
    ctx.beginPath();
    pts.forEach((p, i) => {
      const [x, y] = w2s(p[0], p[1]);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    if (closed) {
      const [x0, y0] = w2s(pts[0][0], pts[0][1]);
      ctx.lineTo(x0, y0);
    }
    ctx.stroke();
    ctx.setLineDash([]);
  };

  // 壁（白）・中心線（シアン破線）・追従ライン（黄）
  stroke(courseMap.left_wall, "#cccccc", 1.5, false, true);
  stroke(courseMap.right_wall, "#cccccc", 1.5, false, true);
  stroke(courseMap.center_line, "#00c8c8", 1.5, true, true);
  stroke(courseMap.racing_line, "#fbbf24", 2, false, true);

  // 車両
  if (loc) {
    const [vx, vy] = w2s(loc.x, loc.y);
    const h = (loc.heading * Math.PI) / 180;
    const s = 9;
    ctx.fillStyle = "#3c82f0";
    ctx.beginPath();
    ctx.moveTo(vx + s * Math.cos(-h), vy + s * Math.sin(-h));
    ctx.lineTo(vx + s * 0.6 * Math.cos(-h + 2.5), vy + s * 0.6 * Math.sin(-h + 2.5));
    ctx.lineTo(vx + s * 0.6 * Math.cos(-h - 2.5), vy + s * 0.6 * Math.sin(-h - 2.5));
    ctx.closePath();
    ctx.fill();
  }
}
