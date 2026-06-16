import { useEffect, useRef } from "react";

const HISTORY_SEC = 10;
const MAX_POINTS = 200;

function fmt(v, d = 2) {
  return v == null ? "-" : Number(v).toFixed(d);
}

// 速度・加速度・ステア・位置・Heading・LiDAR最小距離・SLAM信頼度＋時系列グラフ。
export default function Dashboard({ state }) {
  const canvasRef = useRef(null);
  const histRef = useRef([]); // {t, speed, steer}

  const v = state?.vehicle;
  const loc = state?.localization;
  const lidar = state?.lidar;

  const lidarMin =
    lidar?.distances?.length ? Math.min(...lidar.distances) : null;
  const slamConf = state?.slam_map != null ? loc?.confidence : null;

  // 履歴更新
  useEffect(() => {
    if (!v) return;
    const now = v.timestamp ?? Date.now() / 1000;
    const hist = histRef.current;
    hist.push({ t: now, speed: v.speed, steer: v.steer_angle });
    while (hist.length > MAX_POINTS) hist.shift();
    while (hist.length > 1 && now - hist[0].t > HISTORY_SEC) hist.shift();
    drawGraph();
  }, [v?.timestamp]);

  const drawGraph = () => {
    const cv = canvasRef.current;
    if (!cv) return;
    const ctx = cv.getContext("2d");
    const W = cv.width;
    const H = cv.height;
    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = "#0d0d12";
    ctx.fillRect(0, 0, W, H);

    const hist = histRef.current;
    if (hist.length < 2) return;
    const t0 = hist[0].t;
    const t1 = hist[hist.length - 1].t;
    const span = Math.max(t1 - t0, 1e-3);

    // 中央線
    ctx.strokeStyle = "#333";
    ctx.beginPath();
    ctx.moveTo(0, H / 2);
    ctx.lineTo(W, H / 2);
    ctx.stroke();

    const plot = (key, color, scale) => {
      ctx.strokeStyle = color;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      hist.forEach((p, i) => {
        const x = ((p.t - t0) / span) * W;
        const y = H / 2 - (p[key] / scale) * (H / 2 - 4);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();
    };
    plot("speed", "#34d399", 3.0); // ±3 m/s
    plot("steer", "#fbbf24", 40.0); // ±40 deg
  };

  const Row = ({ label, value }) => (
    <div className="flex justify-between text-sm py-0.5">
      <span className="text-gray-400">{label}</span>
      <span className="font-mono">{value}</span>
    </div>
  );

  return (
    <div className="bg-surge-panel rounded p-3">
      <div className="font-semibold mb-2">DASHBOARD</div>
      <Row label="Speed" value={`${fmt(v?.speed)} m/s`} />
      <Row label="Accel" value={`${fmt(v?.acceleration)} m/s²`} />
      <Row label="Steer" value={`${fmt(v?.steer_angle, 1)}°`} />
      <Row
        label="Pos"
        value={`(${fmt(loc?.x)}, ${fmt(loc?.y)}) m`}
      />
      <Row label="Heading" value={`${fmt(loc?.heading, 1)}°`} />
      <Row label="LiDAR Min" value={lidarMin == null ? "-" : `${fmt(lidarMin)} m`} />
      <Row label="SLAM Conf" value={slamConf == null ? "-" : fmt(slamConf)} />
      <div className="mt-2">
        <div className="flex justify-between text-xs text-gray-500 mb-1">
          <span>
            <span className="text-surge-ok">━</span> speed
          </span>
          <span>
            <span className="text-surge-warn">━</span> steer (直近10s)
          </span>
        </div>
        <canvas ref={canvasRef} width={320} height={90} className="w-full rounded" />
      </div>
    </div>
  );
}
