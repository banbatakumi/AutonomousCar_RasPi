/**
 * 層 C（異常時のみ）— 温度・電圧・電流・リンク統計。**自動運転ビュー専用。**
 *
 * **平常時は1行に畳み、しきい値を超えたものだけが自動的に前に出る**
 * （`architecture.md` §10.1）。正常な温度を常時大きく出しても情報量はゼロで、
 * その面積は層 A（カメラと点群）に回すべき。
 *
 * 自動展開の条件はここ1箇所に集める。散らすと「なぜ開いたのか」が追えなくなる。
 * 中身の表は診断タブと共通（`DiagGrid`）で、**畳む/開くの判断だけがここの仕事**。
 *
 * ラジコンビューはこれを使わない。運転中に読む数字ではないため、丸ごと診断タブへ寄せた。
 */
import { useNumbers } from '../bus/live'
import { battLevel, tempLevel } from '../format'
import { useUi } from '../store/ui'
import { DiagGrid, TEMP_LABEL } from './DiagGrid'

export function DiagStrip() {
  const n = useNumbers()
  const { diagOpen, set } = useUi()
  const vs = n.vs
  const link = n.link

  // ── 「今まさに異常」と「これまでに何回あった」を分ける ──
  //
  // **累積カウンタを自動展開の条件にしてはいけない。** CRC エラーが起動直後に
  // 1回出ただけで、以降ずっとパネルが開いたままになり、層 C を畳んだ意味が消える。
  // 自動展開は「現在の状態」だけ、累積は数字として並べるだけにする。
  const now: string[] = [] // 今まさに異常 → 自動展開する
  const past: string[] = [] // 累積 → バッジで出すが開かない

  if (vs) {
    vs.temp.forEach((t, i) => {
      const lv = tempLevel(t)
      if (lv !== 'ok') now.push(`${TEMP_LABEL[i]} ${t == null ? '通信断' : `${t}℃`}`)
    })
    if (battLevel(vs.batt_voltage[0]) !== 'ok') now.push(`駆動 ${vs.batt_voltage[0].toFixed(1)}V`)
    if (battLevel(vs.batt_voltage[1]) !== 'ok') now.push(`信号 ${vs.batt_voltage[1].toFixed(1)}V`)
    for (const f of vs.faults) now.push(f)
    if (!vs.imu_ok) now.push('IMU 異常')
    if (!vs.lidar_ok) now.push('LiDAR 受信断')
  }
  if (link) {
    if (link.health !== 'OK' && link.health !== 'INIT') now.push(`リンク ${link.health}`)
    if (link.hb_alive === false) now.push('GPIO6 ハートビート停止')
    if ((link.rx?.crc_error ?? 0) > 0) past.push(`CRC 累計 ${link.rx.crc_error}`)
    if ((link.rx?.packet_loss ?? 0) > 0) past.push(`loss 累計 ${link.rx.packet_loss}`)
    if (link.lidar_sectors_lost > 0) past.push(`セクタ落ち 累計 ${link.lidar_sectors_lost}`)
    if (link.hb_stalls > 0) past.push(`hb 停止 ${link.hb_stalls}回`)
  }

  // **今まさに異常なら人間の操作を待たずに開く。** 畳んだまま見逃すのが一番困る
  const open = diagOpen || now.length > 0

  return (
    <div className={`diag ${open ? 'open' : ''}`}>
      <button className="diag-toggle" onClick={() => set({ diagOpen: !diagOpen })}>
        {open ? '▼' : '▶'} 温度・電圧・リンク
        {now.length > 0 && <span className="badge-bad">{now.length}</span>}
        {now.length === 0 && past.length === 0 && <span className="dim"> 正常</span>}
        {now.length === 0 && past.length > 0 && <span className="dim"> 現在は正常</span>}
      </button>

      {(now.length > 0 || past.length > 0) && (
        <div className="diag-alerts">
          {now.map((a) => (
            <span className="badge-bad" key={a}>
              {a}
            </span>
          ))}
          {past.map((a) => (
            <span className="badge-warn" key={a}>
              {a}
            </span>
          ))}
        </div>
      )}

      {open && vs && <DiagGrid n={n} />}
    </div>
  )
}
