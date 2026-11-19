/**
 * 受信メッセージの型ガード。`as unknown as` で型を付けるだけだったのを、**振り分けに使う札だけは
 * 実行時に確かめてから**型を付ける形にした（技術的負債の棚卸し 2026-11-18 中9）。
 *
 * 全フィールドを実行時に検証はしない（数十のフィールドを 50Hz で舐める価値は無い）。
 * 潰したいのは「Pi 側だけ型が変わった・壊れたフレームを信じて描く」ことで、そのための札
 * （telemetry は `schema`、control は `type` と、status なら必須の数個）だけを見る。
 */
import { MSGS_SCHEMA } from '../generated/msgs'
import type {
  CamE2EModelFile,
  CamModelFile,
  ControlStatus,
  E2EFusionModelFile,
  E2EModelFile,
  LogFile,
  MapFile,
  Snapshot,
} from '../types'

function isRecord(v: unknown): v is Record<string, unknown> {
  return typeof v === 'object' && v !== null && !Array.isArray(v)
}

/** 型定義の札（`schema`）が GUI と同じ telemetry のフレームか。違えば描かずに捨てる */
export function isSnapshot(v: unknown): v is Snapshot {
  return isRecord(v) && v.schema === MSGS_SCHEMA
}

/** 札の値（食い違いの通知用）。数でなければ 0 */
export function schemaOf(v: unknown): number {
  return isRecord(v) && typeof v.schema === 'number' ? v.schema : 0
}

/**
 * `/ws/control` でサーバから降ってくる JSON の**振り分けに要る分だけ**の形。
 * `type` で分岐したあとの中身は宣言を信じる（`any` にしないのは、打ち間違いまで通さないため）。
 */
export type ServerMsg = {
  type?: string
  holder?: string
  reason?: string
  files?: LogFile[]
  cam_model_files?: CamModelFile[]
  e2e_model_files?: E2EModelFile[]
  cam_e2e_model_files?: CamE2EModelFile[]
  e2e_fusion_model_files?: E2EFusionModelFile[]
  stereo_e2e_model_files?: E2EFusionModelFile[]
  map_files?: MapFile[]
  ok?: boolean
  error?: string
  id?: number
}

/** JSON の文字列 → `ServerMsg`。文字列でない・壊れた JSON・オブジェクトでなければ null */
export function parseServerMsg(data: unknown): ServerMsg | null {
  if (typeof data !== 'string') return null
  let v: unknown
  try {
    v = JSON.parse(data)
  } catch {
    return null
  }
  if (!isRecord(v)) return null
  if (v.type !== undefined && typeof v.type !== 'string') return null
  return v as ServerMsg
}

/**
 * `status` か。**画面の判断（操縦権・ARM の封印・E-Stop）に使う札**が揃っているかを見る。
 * 欠けた status を受け入れると、`undefined` が「操縦権なし・封印なし」と読まれてしまう
 */
export function isControlStatus(m: ServerMsg): m is ServerMsg & ControlStatus {
  const r = m as Record<string, unknown>
  return (
    m.type === 'status' &&
    typeof r.has_controller === 'boolean' &&
    typeof r.controller === 'string' &&
    typeof r.arm_inhibited === 'boolean' &&
    typeof r.estop_active === 'boolean'
  )
}
