/**
 * 診断タブのグラフ定義。
 *
 * **1枚のグラフに載せる単位は1つだけ**（`unit` が1つしか書けないのはそのため）。
 * V と A、ms と Hz を同じ軸に載せると、値域の小さい方が潰れて読めなくなる。
 */
import { EV, type SeriesKey } from '../../bus/history'
import { BATT_EMPTY_V, BATT_WARN_V, DRIVE_CUTOFF_A, TEMP_BAD_C, TEMP_WARN_C } from '../../format'
import type { Tone } from './uplotBase'

export type SeriesDef = {
  key: SeriesKey
  label: string
  tone: Tone
  /** 指令・目標など「実測ではない線」は破線にする */
  dash?: boolean
}

export type ChartDef = {
  title: string
  unit: string
  series: SeriesDef[]
  /**
   * 縦軸の最小の幅。**停車中は値が平らになり、uPlot の既定では軸が 0〜100 になる。**
   * 0 m/s の速度計に「100」の目盛りが付くと、一瞬あり得ない値が出ているように見える。
   * その物理量として意味のある最小レンジをここで決めておく。
   */
  minSpan: number
  /** 凡例の値の小数桁 */
  digits?: number
  /** 軸に必ず含める値（しきい値までの距離を読ませたいとき） */
  include?: number[]
  /** 軸の下限。件数・電流・使用率のように負にならない量で、平らなときに軸が負へ広がるのを止める */
  floor?: number
  /** しきい値の横線 */
  lines?: { v: number; tone: Tone; label: string }[]
  /** このグラフに重ねる介入フラグ（`EV` のビット和）。異常系は全グラフ共通で重なる */
  bands?: number
}

const TORQUE_BANDS = EV.TC | EV.ABS | EV.LIFT

export const CH = {
  speed: {
    title: '速度',
    unit: 'm/s',
    series: [
      { key: 'speed', label: '実測', tone: 'accent' },
      { key: 'speedCmd', label: '指令', tone: 'live', dash: true },
    ],
    minSpan: 0.6,
    bands: EV.AUTOSTOP,
  },
  wheels: {
    title: '車輪速',
    unit: 'm/s',
    series: [
      { key: 'wheelFL', label: '前左', tone: 'dim' },
      { key: 'wheelFR', label: '前右', tone: 'fg' },
      { key: 'wheelRL', label: '後左', tone: 'accent' },
      { key: 'wheelRR', label: '後右', tone: 'live' },
    ],
    minSpan: 0.6,
    bands: TORQUE_BANDS,
  },
  steer: {
    title: '舵角',
    unit: '°',
    series: [
      { key: 'steer', label: '実測', tone: 'accent' },
      { key: 'steerCmd', label: '指令', tone: 'live', dash: true },
    ],
    minSpan: 20,
    digits: 1,
  },
  voltage: {
    title: 'バッテリー電圧',
    unit: 'V',
    series: [
      { key: 'battDrive', label: '駆動', tone: 'accent' },
      { key: 'battSignal', label: '信号', tone: 'live' },
    ],
    minSpan: 1,
    include: [BATT_EMPTY_V - 0.2],
    lines: [
      { v: BATT_WARN_V, tone: 'warn', label: '注意' },
      { v: BATT_EMPTY_V, tone: 'bad', label: '放電終止' },
    ],
  },
  battCurrent: {
    title: 'バッテリー電流',
    unit: 'A',
    series: [
      { key: 'currDrive', label: '駆動', tone: 'accent' },
      { key: 'currSignal', label: '信号', tone: 'live' },
    ],
    minSpan: 1,
    floor: 0,
    lines: [{ v: DRIVE_CUTOFF_A, tone: 'bad', label: '駆動 遮断' }],
  },
  motorCurrent: {
    title: 'モータ電流',
    unit: 'A',
    series: [
      { key: 'motorRL', label: '後左', tone: 'accent' },
      { key: 'motorRR', label: '後右', tone: 'live' },
      { key: 'motorST', label: 'ステア', tone: 'ok' },
    ],
    minSpan: 2,
  },
  temp: {
    title: '温度',
    unit: '℃',
    series: [
      { key: 'temp0', label: 'MD後左', tone: 'accent' },
      { key: 'temp1', label: 'MD後右', tone: 'live' },
      { key: 'temp2', label: 'MDステア', tone: 'ok' },
      { key: 'temp3', label: 'MCU', tone: 'dim' },
      { key: 'tempPi', label: 'RasPi', tone: 'fg' },
    ],
    minSpan: 15,
    digits: 0,
    lines: [
      { v: TEMP_WARN_C, tone: 'warn', label: '注意' },
      { v: TEMP_BAD_C, tone: 'bad', label: '危険' },
    ],
  },
  torqueRL: {
    title: '後左トルク',
    unit: 'N·m',
    series: [
      { key: 'trqCmdRL', label: '実際', tone: 'accent' },
      { key: 'trqReqRL', label: '要求', tone: 'live', dash: true },
    ],
    minSpan: 0.05,
    digits: 3,
    include: [0],
    bands: TORQUE_BANDS,
  },
  torqueRR: {
    title: '後右トルク',
    unit: 'N·m',
    series: [
      { key: 'trqCmdRR', label: '実際', tone: 'accent' },
      { key: 'trqReqRR', label: '要求', tone: 'live', dash: true },
    ],
    minSpan: 0.05,
    digits: 3,
    include: [0],
    bands: TORQUE_BANDS,
  },
  slip: {
    title: 'スリップ率',
    unit: '%',
    series: [
      { key: 'slipRL', label: '後左', tone: 'accent' },
      { key: 'slipRR', label: '後右', tone: 'live' },
    ],
    minSpan: 10,
    digits: 1,
    include: [0],
    bands: TORQUE_BANDS,
  },
  torqueLimit: {
    title: 'TC・ABS トルク上限',
    unit: 'N·m',
    series: [
      { key: 'tcLimitRL', label: 'TC 後左', tone: 'accent' },
      { key: 'tcLimitRR', label: 'TC 後右', tone: 'live' },
      { key: 'absLimit', label: 'ABS', tone: 'warn' },
    ],
    minSpan: 0.02,
    digits: 3,
    bands: TORQUE_BANDS,
  },
  yawRate: {
    title: 'ヨーレート',
    unit: 'rad/s',
    series: [
      { key: 'yawRate', label: '実測', tone: 'accent' },
    ],
    minSpan: 0.4,
    bands: EV.TV,
  },
  tvRatio: {
    title: 'TV 配分の比率',
    unit: '%',
    series: [{ key: 'tvRatio', label: '右輪が多い＝正', tone: 'live' }],
    minSpan: 10,
    digits: 1,
    include: [0],
    bands: EV.TV,
  },
  yawMoment: {
    title: 'ヨーモーメント',
    unit: 'N·m',
    series: [
      { key: 'tvApplied', label: '実際の左右差', tone: 'accent' },
      { key: 'tvMoment', label: 'TV の要求', tone: 'live', dash: true },
    ],
    minSpan: 0.05,
    digits: 3,
    bands: EV.TV,
  },
  accel: {
    title: '加速度',
    unit: 'm/s²',
    series: [
      { key: 'accelX', label: '前後', tone: 'accent' },
      { key: 'accelY', label: '左右', tone: 'live' },
    ],
    minSpan: 4,
  },
  delay: {
    title: '往復遅延',
    unit: 'ms',
    series: [
      { key: 'rttMs', label: 'Pi⇄STM32', tone: 'accent' },
      { key: 'wsRttMs', label: 'ブラウザ⇄Pi', tone: 'live' },
    ],
    minSpan: 20,
    digits: 1,
    floor: 0,
    lines: [{ v: 50, tone: 'bad', label: '警告' }],
  },
  rate: {
    title: '受信レート',
    unit: 'Hz',
    series: [
      { key: 'rxHz', label: 'テレメトリ', tone: 'accent' },
      { key: 'scanHz', label: 'LiDAR', tone: 'live' },
    ],
    minSpan: 10,
    digits: 1,
    floor: 0,
  },
  errors: {
    title: 'UART エラーの増分',
    unit: '件',
    series: [
      { key: 'crcDelta', label: 'CRC（Pi 受信）', tone: 'bad' },
      { key: 'stmCrcDelta', label: 'CRC（STM32 受信）', tone: 'warn' },
      { key: 'lossDelta', label: 'ロス', tone: 'accent' },
    ],
    minSpan: 4,
    digits: 0,
    floor: 0,
  },
  loop: {
    title: 'io ループ最大・ハートビート遅れ',
    unit: 'ms',
    series: [
      { key: 'loopMaxMs', label: 'ループ最大', tone: 'accent' },
      { key: 'hbLateMs', label: 'HB 最大遅れ', tone: 'live' },
    ],
    minSpan: 5,
    digits: 2,
    floor: 0,
  },
  wifi: {
    title: 'Wi-Fi 電波強度',
    unit: 'dBm',
    series: [{ key: 'wifiDbm', label: 'RSSI', tone: 'accent' }],
    minSpan: 20,
    digits: 0,
    lines: [
      { v: -60, tone: 'warn', label: '注意' },
      { v: -75, tone: 'bad', label: '危険' },
    ],
  },
  piCpu: {
    title: 'RasPi CPU 使用率',
    unit: '%',
    series: [
      { key: 'piCpu', label: '平均', tone: 'accent' },
      { key: 'piCpuMax', label: '最大コア', tone: 'live' },
    ],
    minSpan: 40,
    digits: 0,
    floor: 0,
  },
  ultrasonic: {
    title: '超音波',
    unit: 'm',
    series: [
      { key: 'usFront', label: '前', tone: 'accent' },
      { key: 'usRear', label: '後', tone: 'live' },
    ],
    minSpan: 0.5,
    floor: 0,
  },
} satisfies Record<string, ChartDef>
