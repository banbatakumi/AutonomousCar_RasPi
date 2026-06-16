import { useEffect, useRef, useState } from "react";
import {
  MAX_SPEED,
  MAX_STEER,
  SPEED_STEP,
  STEER_STEP,
} from "../constants/modes";

const SEND_INTERVAL_MS = 50; // 20Hz で指令を送信

function clamp(v, lo, hi) {
  return Math.max(lo, Math.min(hi, v));
}

// キーボード入力（↑↓←→ / ESC）を管理し、WebSocket 経由で指令を送るフック。
//   send       : useWebSocket の send
//   enabled    : 操作を受け付けるか（モードによって切替）
export function useKeyboard(send, enabled = true) {
  const [targetSpeed, setTargetSpeed] = useState(0);
  const [targetSteer, setTargetSteer] = useState(0);
  const keysRef = useRef({ up: false, down: false, left: false, right: false });
  const speedRef = useRef(0);
  const steerRef = useRef(0);
  const enabledRef = useRef(enabled);

  useEffect(() => {
    enabledRef.current = enabled;
    if (!enabled) {
      // 無効化時は即座にゼロへ
      keysRef.current = { up: false, down: false, left: false, right: false };
      speedRef.current = 0;
      steerRef.current = 0;
      setTargetSpeed(0);
      setTargetSteer(0);
    }
  }, [enabled]);

  useEffect(() => {
    const onKeyDown = (e) => {
      if (e.key === "Escape") {
        send({ type: "emergency_stop" });
        return;
      }
      if (!enabledRef.current) return;
      switch (e.key) {
        case "ArrowUp":
          keysRef.current.up = true;
          e.preventDefault();
          break;
        case "ArrowDown":
          keysRef.current.down = true;
          e.preventDefault();
          break;
        case "ArrowLeft":
          keysRef.current.left = true;
          e.preventDefault();
          break;
        case "ArrowRight":
          keysRef.current.right = true;
          e.preventDefault();
          break;
        default:
          break;
      }
    };
    const onKeyUp = (e) => {
      switch (e.key) {
        case "ArrowUp":
          keysRef.current.up = false;
          break;
        case "ArrowDown":
          keysRef.current.down = false;
          break;
        case "ArrowLeft":
          keysRef.current.left = false;
          break;
        case "ArrowRight":
          keysRef.current.right = false;
          break;
        default:
          break;
      }
    };
    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);
    return () => {
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
    };
  }, [send]);

  // 50ms ごとに指令値を更新して送信
  useEffect(() => {
    const timer = setInterval(() => {
      if (!enabledRef.current) return;
      const k = keysRef.current;

      // 速度：押している間だけ増減、離したら 0 に戻す
      if (k.up) speedRef.current = clamp(speedRef.current + SPEED_STEP, 0, MAX_SPEED);
      else if (k.down) speedRef.current = clamp(speedRef.current - SPEED_STEP, -MAX_SPEED, 0);
      else speedRef.current = 0;

      // ステア：押している間だけ増減、離したら 0 に戻す
      if (k.left) steerRef.current = clamp(steerRef.current + STEER_STEP, -MAX_STEER, MAX_STEER);
      else if (k.right) steerRef.current = clamp(steerRef.current - STEER_STEP, -MAX_STEER, MAX_STEER);
      else steerRef.current = 0;

      setTargetSpeed(speedRef.current);
      setTargetSteer(steerRef.current);
      send({
        type: "command",
        target_speed: speedRef.current,
        target_steer: steerRef.current,
      });
    }, SEND_INTERVAL_MS);
    return () => clearInterval(timer);
  }, [send]);

  return { targetSpeed, targetSteer };
}
