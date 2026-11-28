/**
 * カメラ映像に重ねる描画（進路ガイド・障害物・計画・白線の目標点・矢印信号・追跡枠・古さの警告）。
 * `CameraView.tsx` から分けた（技術的負債の棚卸し 2026-11-18 高6）。投影の考え方は `CameraView.tsx` 冒頭が正。
 */
import { live } from '../bus/live'
import { BUMPER_X, HALF_WIDTH, findPathObstacle, scanPointsNow, type ObstacleLevel } from '../bus/pathObstacles'
import { VEHICLE as VEHICLE_GEOM } from '../generated/vehicle'
import { reverseSelected } from '../store/gear'
import { useUi } from '../store/ui'
import { camGeom, type CamId } from './cameraLayout'
import { fisheyeLens, groundProjector, pinholeLens, type Lens } from './cameraModel'

/** 進路ガイドを出すか。カメラはどれも前向きなので、前進するとき（走行中は実測速度の符号、
 * 停止中は D/R レンジ）だけ出す（後退中に前向きのカメラへ出しても意味が無い） */
export function guideFacing(_cam: CamId): boolean {
  const speed = live.vs?.speed ?? 0
  // 止まっているときは選んでいるレンジ（MT は R 段）で決める
  const reverse = Math.abs(speed) > 0.05 ? speed < 0 : reverseSelected(useUi.getState())
  return !reverse
}

/** 実測舵角から予測進路を路面平面に置き、カメラモデル（`lensFor()`）で画像に落とす。
 * `camHeightSetting` は `vehicle.toml` 由来の固定値（左右とも同じ高さに付いている）。
 * `pitch`/レンズ（`fisheye`/`hfov`）/`topCrop`/`bottomCrop` も `vehicle.toml` 由来の固定値（`cam` に応じて
 * `camFront`/`camLeft` を選ぶ）。`vs.pitch`/`vs.roll`（IMU 実測）で走行中の車体姿勢ぶんを毎フレーム補正する
 * （符号の向きはファイル冒頭コメント参照）。 */
export function drawGuide(
  ctx: CanvasRenderingContext2D,
  w: number,
  h: number,
  camHeightSetting: number,
  cam: CamId,
) {
  const vs = live.vs
  if (!vs) return
  const project = makeProjector(w, h, camHeightSetting, cam)
  // 「今の舵角のまま前進したら」の軌跡を積分する刻み [m]
  const ds = 0.08

  // 車体の左右端をなぞる2本
  for (const off of [-VEHICLE_HALF_WIDTH, VEHICLE_HALF_WIDTH]) {
    ctx.beginPath()
    let x = 0
    let y = 0
    let th = 0
    let started = false
    for (let i = 0; i < 45; i++) {
      th += (ds / VEHICLE_GEOM.wheelbase) * Math.tan(vs.steer_actual)
      // 舵いっぱいだと弧が横〜後ろへ回り込み、カメラに映らない上に魚眼で暴れる。
      // 進行方向が90°を超えた先は進路ガイドとして意味が無いので打ち切る
      if (Math.abs(th) > Math.PI / 2) break
      x += ds * Math.cos(th)
      y += ds * Math.sin(th)
      // 車体端は進行方向の法線（左）へ off ずらす。車両座標の y 方向へずらすと、
      // 旋回して車体が向きを変えたぶん幅が潰れて（または広がって）崩れる。
      // 車両座標 (x=前, y=左) をそのまま渡す（カメラ取付位置の差し引きは投影器の中）
      const p = project(x - off * Math.sin(th), y + off * Math.cos(th))
      // 写らない点は線を繋がずに分断する（繋ぐと画面を横切る長い直線になる）
      if (!p) {
        started = false
        continue
      }
      if (!started) {
        ctx.moveTo(p[0], p[1])
        started = true
      } else ctx.lineTo(p[0], p[1])
    }
    ctx.strokeStyle = 'rgba(255,198,63,0.75)'
    ctx.lineWidth = 2
    ctx.stroke()
  }
}

/** どのレンズモデルで描くか。**補正映像を表示中なら仮想ピンホール**（Pi が remap 済み）、
 * そうでなければ校正値があれば魚眼、無ければ `hfov` のピンホール近似。
 * 補正映像かどうかはサーバ真値（`status.camera_config.undistort` と、そのカメラが
 * 校正済みで実際に補正されているか `undistort_available`）で決める。 */
function lensFor(cam: CamId, w: number, h: number): Lens {
  const geom = camGeom(cam)
  const cfg = useUi.getState().cameraConfig
  if (cfg?.undistort && cfg.undistort_available?.[cam]) {
    return pinholeLens(w, h, geom.undistortHfov, geom.bottomCrop, 'undistorted', geom.topCrop)
  }
  if (geom.fisheye) return fisheyeLens(w, h, geom.fisheye, geom.bottomCrop, geom.topCrop)
  return pinholeLens(w, h, geom.hfov, geom.bottomCrop, 'pinhole', geom.topCrop)
}

/** 路面座標（base_link、x=前 y=左）→ 画像座標の投影器を作る。IMU による姿勢（pitch/roll）補正込み。
 * `drawGuide`（進路ガイド）と `drawLineTarget`（認識ラインの重畳）が共用する
 * ——同じカメラ内部・外部パラメータ・IMU補正を2箇所に書くと片方だけ直し忘れる。
 *
 * レンズ（魚眼/補正映像/未校正ピンホール）は `lensFor()` が選ぶ。姿勢は
 * **カメラ座標の光線を回してから**レンズに通す——魚眼では「画像を主点まわりに
 * 回す」近似が成り立たない（端ほど像の回り方が違う）ため。ピンホールでは
 * 従来の画像面回転と同じ結果になる。 */
function makeProjector(w: number, h: number, camHeightSetting: number, cam: CamId) {
  const vs = live.vs
  const geom = camGeom(cam)
  // 左右とも同じ高さに付いているので、設定パネルで校正した高さを両方に使う
  const height = camHeightSetting
  const lens = lensFor(cam, w, h)

  // IMU が無効・未受信の間は姿勢ぶんの補正をかけない（0 扱い＝固定値のみ）
  const dPitch = vs?.imu_ok ? vs.pitch : 0
  const dRoll = vs?.imu_ok ? vs.roll : 0

  // 前が沈む（dPitch 負）ほどカメラも一緒に下を向くので、符号を反転して足す。
  // roll: 光軸まわりの回転。右が沈む（dRoll 正）とカメラも右に傾き、
  // 写真の一般則どおり像は光軸を中心に反時計回りへ回って見える。
  // **返す関数は base_link 基準の地面点 `(x=前, y=左)` を受け取る**（カメラ取付位置の
  // 差し引きは `groundProjector` がやる）
  return groundProjector(
    lens,
    { x: geom.x, y: geom.y, height, pitch: geom.pitch - dPitch },
    dRoll,
  )
}

/**
 * `line_trace` が認識している白線の近傍・遠方点（`LineScan`、既に base_link
 * 座標に逆投影済み）を、進路ガイドと同じ路面→画像投影でカメラ映像に重ねる。
 * **前カメラでしか意味を持たない**（`line_perception_node.py` は前方カメラ
 * しか見ていない）ので `cam` は取らず、常に前カメラの内部・外部パラメータで
 * 投影する。進路ガイドの琥珀色と混同しないよう別の色（シアン系）にする。
 * 見えていない点は描かない——「見失っている」こと自体は判断欄の `reason` で分かる。
 */
export function drawLineTarget(ctx: CanvasRenderingContext2D, w: number, h: number, camHeightSetting: number) {
  const line = live.lineCam
  if (!line || (!line.near_seen && !line.far_seen)) return
  const project = makeProjector(w, h, camHeightSetting, 'front')

  const nearPt = line.near_seen ? project(line.near_x, line.near_y) : null
  const farPt = line.far_seen ? project(line.far_x, line.far_y) : null
  const pts = [nearPt, farPt].filter((p): p is [number, number] => p != null)
  if (pts.length === 0) return

  ctx.strokeStyle = 'rgba(80,220,255,0.85)'
  ctx.fillStyle = 'rgba(80,220,255,0.85)'
  if (nearPt && farPt) {
    ctx.beginPath()
    ctx.moveTo(nearPt[0], nearPt[1])
    ctx.lineTo(farPt[0], farPt[1])
    ctx.lineWidth = 2
    ctx.stroke()
  }
  for (const [x, y] of pts) {
    ctx.beginPath()
    ctx.arc(x, y, 5, 0, Math.PI * 2)
    ctx.fill()
  }
}

/**
 * `arrow_signal_node.py` の直近判定（`live.signal`）を前方カメラ映像に重畳する。
 * 専用スペースは使わず、ROI帯の枠線＋方向ラベルを映像へ直接描くだけ
 * （会場でのHSVライブチューニング中に見る想定）。`enabled=false`（設定パネルで
 * OFF）の間は何も描かない。確定（`confirmed_value`、`route/select`として実際に
 * 送信中）していれば目立つ色・大きな文字で、confirm前の生判定（`value`）は
 * 控えめに出す——「今まさに見えている」と「確定して経路が切り替わった」を
 * 見た目で区別できるようにする。
 */
export function drawArrowSignal(ctx: CanvasRenderingContext2D, w: number, h: number) {
  const sig = live.signal
  if (!sig || !sig.enabled) return
  const y0 = sig.roi_top * h
  const y1 = sig.roi_bottom * h
  const confirmed = sig.confirmed_value
  const isYolo = sig.method === 'yolo'
  const lit = isYolo ? sig.score > 0 : sig.lit_frac > 0.005

  ctx.save()
  ctx.strokeStyle = confirmed ? 'rgba(255,204,0,0.9)' : 'rgba(255,255,255,0.25)'
  ctx.lineWidth = confirmed ? 3 : 1
  ctx.strokeRect(0, y0, w, y1 - y0)

  // YOLO の検出枠（方向のクラスは水色、方向でないクラス＝非該当は灰色）
  const [bx0, by0, bx1, by1] = sig.box
  if (isYolo && sig.score > 0 && bx1 > bx0 && by1 > by0) {
    ctx.strokeStyle = sig.value ? 'rgba(80,200,255,0.95)' : 'rgba(180,180,180,0.8)'
    ctx.lineWidth = 2
    ctx.strokeRect(bx0 * w, by0 * h, (bx1 - bx0) * w, (by1 - by0) * h)
    ctx.font = '12px sans-serif'
    ctx.fillStyle = ctx.strokeStyle
    ctx.textAlign = 'left'
    ctx.fillText(`${sig.label} ${sig.score.toFixed(2)}`, bx0 * w, Math.max(12, by0 * h - 4))
  }

  const label = confirmed
    ? { left: '◀ 左折', right: '右折 ▶' }[confirmed]
    : (lit ? (sig.value || (isYolo ? '非該当' : '判定中')) : '')
  if (label) {
    ctx.font = confirmed ? 'bold 22px sans-serif' : '13px sans-serif'
    ctx.fillStyle = confirmed ? 'rgba(255,204,0,0.95)' : 'rgba(255,255,255,0.6)'
    ctx.textAlign = 'center'
    ctx.fillText(label, w / 2, Math.max(16, y0 - 8))
  }
  ctx.restore()
}

/** ドラッグ中のROI選択矩形（正規化画像座標）を映像に重ねる。
 * `drag` は始点・終点で、順序（左上/右下）は問わない——描く直前に整える。 */
export function drawRoiDrag(
  ctx: CanvasRenderingContext2D,
  dx: number,
  dy: number,
  dw: number,
  dh: number,
  drag: { x0: number; y0: number; x1: number; y1: number },
) {
  const x0 = Math.min(drag.x0, drag.x1)
  const x1 = Math.max(drag.x0, drag.x1)
  const y0 = Math.min(drag.y0, drag.y1)
  const y1 = Math.max(drag.y0, drag.y1)
  ctx.save()
  ctx.strokeStyle = 'rgba(255,210,80,0.9)'
  ctx.lineWidth = 2
  ctx.setLineDash([6, 4])
  ctx.strokeRect(dx + x0 * dw, dy + y0 * dh, (x1 - x0) * dw, (y1 - y0) * dh)
  ctx.restore()
}

/** 点群・判断がこれより古ければ重ねない [ms]（古い位置に障害物を描かない） */
export const SCAN_FRESH_MS = 400
export const AUTO_FRESH_MS = 600

/** 進路上の障害物の段ごとの色（`bus/pathObstacles.ts`）。赤・琥珀は意味色、白は「あるだけ」 */
const OB_COLOR: Record<ObstacleLevel, string> = {
  danger: '#f2544b',
  caution: '#e8ab21',
  info: 'rgba(255,255,255,0.55)',
}

/**
 * 進路上の障害物（LiDAR）を映像に重ねる。今の舵角のまま進んだら最初に
 * 当たる点群に印を付け、**車体の前端（後退なら後端）が当たる位置に横線**（止まる線）を引く。
 * 色は当たるまでの時間で決まる（近づいていなければ白）。距離は線の上に出す。
 * 計算は `bus/pathObstacles.ts`（点群を取得から今までの車の動きぶん戻してから掃く）
 */
export function drawObstacles(
  ctx: CanvasRenderingContext2D, w: number, h: number, camHeightSetting: number, cam: CamId,
  /** 映像のうち見えている下端の y（これより下は HUD に隠れる） */
  visibleBottom: number,
) {
  const scan = live.scan
  const vs = live.vs
  if (!scan || !vs) return
  // カメラは前向きなので、前進したときの進路上の障害物だけを描く
  const ob = findPathObstacle(scanPointsNow(scan, vs), vs.steer_actual, vs.speed, false)
  if (!ob) return
  const project = makeProjector(w, h, camHeightSetting, cam)
  const col = OB_COLOR[ob.level]
  const lit = ob.level !== 'info'

  ctx.save()
  // 当たる点群（足元＝路面に落とした位置）
  ctx.fillStyle = col
  for (const [x, y] of ob.hits) {
    const p = project(x, y)
    if (p) ctx.fillRect(p[0] - 2.5, p[1] - 2.5, 5, 5)
  }
  // 止まる線。車体の端の位置で、車体の幅いっぱいに
  const { x, y, th } = ob.pose
  const bx = BUMPER_X.front
  const ex = x + bx * Math.cos(th)
  const ey = y + bx * Math.sin(th)
  const nx = -Math.sin(th) * HALF_WIDTH
  const ny = Math.cos(th) * HALF_WIDTH
  const pa = project(ex + nx, ey + ny)
  const pb = project(ex - nx, ey - ny)
  let a: [number, number]
  let b: [number, number]
  // **近いほど路面の手前＝映像の下端（HUD の裏・画面の外）に入る。** 一番大事な近距離が
  // 見えなくなるので、見える下端に寄せて描く（横の位置は投影できた方を使い、無ければ中央）
  const edgeY = visibleBottom - 10
  if (pa && pb && Math.max(pa[1], pb[1]) <= edgeY) {
    a = pa
    b = pb
  } else {
    const xs = [pa, pb].filter((p): p is [number, number] => p != null).map((p) => p[0])
    const x0 = xs.length === 2 ? Math.min(...xs) : w * 0.35
    const x1 = xs.length === 2 ? Math.max(...xs) : w * 0.65
    const span = Math.max(x1 - x0, w * 0.18)
    const mid = (x0 + x1) / 2
    a = [Math.max(4, mid - span / 2), edgeY]
    b = [Math.min(w - 4, mid + span / 2), edgeY]
  }
  ctx.strokeStyle = col
  ctx.lineCap = 'round'
  ctx.lineWidth = ob.level === 'danger' ? 6 : lit ? 4 : 2
  if (lit) {
    ctx.shadowColor = col
    ctx.shadowBlur = 12
  }
  ctx.beginPath()
  ctx.moveTo(a[0], a[1])
  ctx.lineTo(b[0], b[1])
  ctx.stroke()
  ctx.shadowBlur = 0
  // 距離。線の真ん中の上に、読める地の上で
  const mx = (a[0] + b[0]) / 2
  const my = Math.min(a[1], b[1]) - 10
  const text = `${ob.distM.toFixed(2)} m`
  ctx.font = "bold 15px 'DIN Alternate', 'Hiragino Sans', sans-serif"
  ctx.textAlign = 'center'
  ctx.textBaseline = 'bottom'
  const tw = ctx.measureText(text).width
  ctx.fillStyle = 'rgba(0,0,0,0.6)'
  ctx.fillRect(mx - tw / 2 - 6, my - 18, tw + 12, 20)
  ctx.fillStyle = lit ? col : '#fff'
  ctx.fillText(text, mx, my)
  ctx.restore()
}

/** 自動運転の計画の色。計測値の線と同じ青（`--trace`）を指令の進路に、経路は紫 */
const PLAN_CMD = '#4aa8ff'
const PLAN_ROUTE = 'rgba(200,120,255,0.85)'
/** 経路を描く範囲 [m]（base_link の前方） */
const PLAN_ROUTE_MIN_X = 0.15
const PLAN_ROUTE_MAX_X = 6
const PLAN_ROUTE_MAX_Y = 3

/**
 * 自動運転の計画を前方映像に重ねる。**planner が何をしようとしているか**と
 * **実際に見えているもの**のずれを映像の上で読むため。出すもの:
 *
 * - 指令の進路（青の破線）: `target_steer` のまま進んだ弧。実測舵角の進路ガイド（琥珀）と
 *   並べると、舵の遅れや「指令と違う向きに行っている」が分かる。長さは `target_speed` の 1.5 秒ぶん
 * - 地図を持つ planner（`phase` がある＝slam2d 系）のとき: 今走っている経路（紫）、
 *   Pure Pursuit が狙っている点（白い菱形）、検出した動的障害物（赤い輪）。どれも map
 *   フレームなので、自己位置（`pose_*`）で base_link に直してから投影する
 */
export function drawPlan(ctx: CanvasRenderingContext2D, w: number, h: number, camHeightSetting: number) {
  const a = live.auto
  // 判断を出せていない（モデル未選択など）間は描かない。指令 0 の線は情報にならない
  if (!a || !a.ready) return
  const project = makeProjector(w, h, camHeightSetting, 'front')
  ctx.save()

  // 指令の進路
  const len = Math.max(0.8, Math.min(4, Math.abs(a.target_speed) * 1.5))
  if (a.target_speed >= 0) {
    const k = Math.tan(a.target_steer) / VEHICLE_GEOM.wheelbase
    ctx.beginPath()
    let x = 0
    let y = 0
    let th = 0
    let started = false
    for (let s = 0; s < len; s += 0.05) {
      th += 0.05 * k
      if (Math.abs(th) > Math.PI / 2) break
      x += 0.05 * Math.cos(th)
      y += 0.05 * Math.sin(th)
      const p = project(x, y)
      if (!p) {
        started = false
        continue
      }
      if (started) ctx.lineTo(p[0], p[1])
      else ctx.moveTo(p[0], p[1])
      started = true
    }
    ctx.setLineDash([10, 7])
    ctx.strokeStyle = PLAN_CMD
    ctx.lineWidth = 3
    ctx.shadowColor = PLAN_CMD
    ctx.shadowBlur = 6
    ctx.stroke()
    ctx.setLineDash([])
    ctx.shadowBlur = 0
  }

  // 地図を持つ planner のときだけ（自己位置が意味を持つ）
  if (a.phase) {
    const c = Math.cos(a.pose_yaw)
    const sn = Math.sin(a.pose_yaw)
    const toBase = (mx: number, my: number): [number, number] => {
      const dx = mx - a.pose_x
      const dy = my - a.pose_y
      return [dx * c + dy * sn, -dx * sn + dy * c]
    }
    const inView = (x: number, y: number) =>
      x > PLAN_ROUTE_MIN_X && x < PLAN_ROUTE_MAX_X && Math.abs(y) < PLAN_ROUTE_MAX_Y

    const line = live.map?.raceline
    if (line && line.length >= 4) {
      ctx.beginPath()
      let started = false
      let prev: [number, number] | null = null
      for (let i = 0; i + 1 < line.length; i += 2) {
        const [x, y] = toBase(line[i]!, line[i + 1]!)
        const p = inView(x, y) ? project(x, y) : null
        // 範囲外・写らない点・離れた点（経路の継ぎ目）では線を切る
        const jump = prev ? Math.hypot(x - prev[0], y - prev[1]) > 0.5 : false
        prev = [x, y]
        if (!p || jump) {
          started = false
          if (!p) continue
        }
        if (started) ctx.lineTo(p[0], p[1])
        else ctx.moveTo(p[0], p[1])
        started = true
      }
      ctx.strokeStyle = PLAN_ROUTE
      ctx.lineWidth = 3
      ctx.stroke()
    }

    // 狙っている点
    const [tx, ty] = toBase(a.target_x, a.target_y)
    const tp = inView(tx, ty) ? project(tx, ty) : null
    if (tp) {
      ctx.beginPath()
      ctx.moveTo(tp[0], tp[1] - 9)
      ctx.lineTo(tp[0] + 7, tp[1])
      ctx.lineTo(tp[0], tp[1] + 9)
      ctx.lineTo(tp[0] - 7, tp[1])
      ctx.closePath()
      ctx.fillStyle = '#fff'
      ctx.strokeStyle = PLAN_CMD
      ctx.lineWidth = 2
      ctx.fill()
      ctx.stroke()
    }

    // 検出した動的障害物 [x, y, r, ...]
    const obs = a.obstacles ?? []
    ctx.strokeStyle = '#f2544b'
    ctx.lineWidth = 2
    for (let i = 0; i + 2 < obs.length; i += 3) {
      const [ox, oy] = toBase(obs[i]!, obs[i + 1]!)
      const r = obs[i + 2]!
      if (!inView(ox, oy)) continue
      ctx.beginPath()
      let started = false
      for (let j = 0; j <= 16; j++) {
        const t = (j / 16) * Math.PI * 2
        const p = project(ox + r * Math.cos(t), oy + r * Math.sin(t))
        if (!p) {
          started = false
          continue
        }
        if (started) ctx.lineTo(p[0], p[1])
        else ctx.moveTo(p[0], p[1])
        started = true
      }
      ctx.stroke()
    }
  }
  ctx.restore()
}

/** 映像が古いとみなす最短の時間 [ms]。ARM 中の 15〜30fps なら数フレーム分 */
export const CAM_STALE_MIN_MS = 500
/** フレーム間隔（移動平均）の何倍届かなければ古いとみなすか。DISARM 中の低い fps 用 */
export const CAM_STALE_INTERVALS = 3

/**
 * 映像の「遅れ」。止まった（`drawStale`）とまでは言えないが、いつもの間隔より
 * 明らかに来ていない状態。**ARM 中だけ**見る（DISARM 中は 1fps に落としているので常に遅れて見える）。
 * カメラ配信は JPEG だけで撮影時刻が付いていないので、測れるのは「届く間隔」だけ——撮ってから
 * 届くまでの遅れそのものは測れない
 */
const CAM_LAG_MIN_MS = 180
const CAM_LAG_INTERVALS = 2.5

export function isLagging(ageMs: number, intervalMs: number): boolean {
  return !!live.vs?.armed && ageMs > Math.max(CAM_LAG_MIN_MS, intervalMs * CAM_LAG_INTERVALS)
}

/** 遅れ: 映像の縁を琥珀に光らせ、左上に古さを出す（映像そのものは暗くしない） */
export function drawLag(ctx: CanvasRenderingContext2D, dx: number, dy: number, dw: number, dh: number, ageMs: number) {
  ctx.save()
  ctx.strokeStyle = '#e8ab21'
  ctx.lineWidth = 4
  ctx.shadowColor = '#e8ab21'
  ctx.shadowBlur = 10
  ctx.strokeRect(dx + 2, dy + 2, dw - 4, dh - 4)
  ctx.shadowBlur = 0
  drawTag(ctx, dx + 12, dy + 12, `映像 遅れ ${(ageMs / 1000).toFixed(1)}s`, '#e8ab21', '#1a1305')
  ctx.restore()
}

/** 止まった映像を暗くし、中央に古さを警告灯の札で出す。`ageMs=null` は切断中 */
export function drawStale(
  ctx: CanvasRenderingContext2D, dx: number, dy: number, dw: number, dh: number, ageMs: number | null,
) {
  const text = ageMs == null ? '映像 切断' : `映像停止 ${(ageMs / 1000).toFixed(1)}s`
  ctx.save()
  ctx.fillStyle = 'rgba(0, 0, 0, 0.6)'
  ctx.fillRect(dx, dy, dw, dh)
  ctx.font = "bold 16px 'Hiragino Sans', sans-serif"
  const tw = ctx.measureText(text).width
  drawTag(ctx, dx + dw / 2 - tw / 2 - 12, dy + dh / 2 - 15, text, '#e2433b', '#fff', 16)
  ctx.restore()
}

/** 警告灯の札（地を塗った角丸の帯に文字）。左上の位置で受け取る */
function drawTag(ctx: CanvasRenderingContext2D, x: number, y: number, text: string, bg: string, fg: string, size = 13) {
  ctx.font = `bold ${size}px 'Hiragino Sans', sans-serif`
  const tw = ctx.measureText(text).width
  const hgt = size + 12
  ctx.fillStyle = bg
  ctx.beginPath()
  ctx.roundRect(x, y, tw + 24, hgt, 4)
  ctx.fill()
  ctx.fillStyle = fg
  ctx.textAlign = 'left'
  ctx.textBaseline = 'middle'
  ctx.fillText(text, x + 12, y + hgt / 2 + 1)
}

/** DISARM 中は対象を選べないことを映像の左上に出す（`onRoiPointerDown` が無視する理由）。 */
export function drawRoiDisarmedHint(ctx: CanvasRenderingContext2D, dx: number, dy: number) {
  const text = 'DISARM中は対象を選べません（ARMしてから選択）'
  ctx.save()
  ctx.font = '12px ui-monospace, monospace'
  const w = ctx.measureText(text).width
  ctx.fillStyle = 'rgba(0, 0, 0, 0.6)'
  ctx.fillRect(dx + 8, dy + 8, w + 12, 20)
  ctx.fillStyle = '#ffcc66'
  ctx.fillText(text, dx + 14, dy + 22)
  ctx.restore()
}

/** `cam_track_node.py` の追跡結果（`live.track`）のbboxを映像に重ねる。
 * `tracking=False`（未選択）の間は何も描かない。見失っている（`lost`）間は
 * 色を変えて区別する——直前のbboxを保持したまま描き続けるので、
 * 「まだ追ってはいるが自信が無い」ことが見た目で分かる。 */
export function drawTrackBox(ctx: CanvasRenderingContext2D, dx: number, dy: number, dw: number, dh: number) {
  const t = live.track
  if (!t || !t.tracking) return
  const x = dx + (t.bbox_cx - t.bbox_w / 2) * dw
  const y = dy + (t.bbox_cy - t.bbox_h / 2) * dh
  ctx.save()
  ctx.strokeStyle = t.lost ? 'rgba(255,90,90,0.9)' : 'rgba(80,220,255,0.9)'
  ctx.lineWidth = 2
  ctx.strokeRect(x, y, t.bbox_w * dw, t.bbox_h * dh)
  ctx.restore()
}

const VEHICLE_HALF_WIDTH = Math.max(...VEHICLE_GEOM.footprint.map(([, y]) => Math.abs(y)))
