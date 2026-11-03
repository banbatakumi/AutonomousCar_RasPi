/**
 * クリアランスソナーの音（2026-11-03）。進む向きの半周（LiDAR＋超音波、`bus/parkSensor.ts`）が
 * 近いと断続音、さらに近いと連続音。**ARM を保持している間だけ**鳴る。どのタブを見ていても鳴る
 * （運転中に診断タブを開いていても後ろの壁は近づく）。
 *
 * `AudioContext` はブラウザの自動再生制限で、ユーザー操作の後でないと音が出ない。最初の
 * キー・ポインタ操作で作る（ARM の Enter・クリックがそれになる）。GUI の演出のみで車両には無関係
 */
import { useEffect } from 'react'
import { live } from '../bus/live'
import { beepPeriodMs, relevantDistance, sectorClearance } from '../bus/parkSensor'
import { reverseSelected, useUi } from '../store/ui'

const TONE_HZ = 1900
const BEEP_MS = 70
const GAIN = 0.12
/** 判定の周期 [ms]。超音波は 20Hz なのでこれで足りる */
const TICK_MS = 25
/** 点群がこれより古ければ使わない（超音波だけで判断する）[ms] */
const SCAN_FRESH_MS = 400

export function useParkBeep() {
  useEffect(() => {
    let ac: AudioContext | null = null
    /** 連続音の発振器（鳴っている間だけ） */
    let cont: { osc: OscillatorNode; gain: GainNode } | null = null
    let nextBeepAt = 0

    const ensureAudio = () => {
      if (!ac) ac = new AudioContext()
      if (ac.state === 'suspended') void ac.resume()
    }
    window.addEventListener('keydown', ensureAudio, true)
    window.addEventListener('pointerdown', ensureAudio, true)

    const beep = () => {
      if (!ac || ac.state !== 'running') return
      const t = ac.currentTime
      const osc = ac.createOscillator()
      const g = ac.createGain()
      osc.type = 'sine'
      osc.frequency.value = TONE_HZ
      g.gain.setValueAtTime(0, t)
      g.gain.linearRampToValueAtTime(GAIN, t + 0.005)
      g.gain.setValueAtTime(GAIN, t + BEEP_MS / 1000 - 0.01)
      g.gain.linearRampToValueAtTime(0, t + BEEP_MS / 1000)
      osc.connect(g).connect(ac.destination)
      osc.start(t)
      osc.stop(t + BEEP_MS / 1000 + 0.02)
    }
    const startCont = () => {
      if (cont || !ac || ac.state !== 'running') return
      const osc = ac.createOscillator()
      const gain = ac.createGain()
      osc.type = 'sine'
      osc.frequency.value = TONE_HZ
      gain.gain.setValueAtTime(0, ac.currentTime)
      gain.gain.linearRampToValueAtTime(GAIN, ac.currentTime + 0.01)
      osc.connect(gain).connect(ac.destination)
      osc.start()
      cont = { osc, gain }
    }
    const stopCont = () => {
      if (!cont || !ac) return
      const c = cont
      cont = null
      c.gain.gain.setValueAtTime(c.gain.gain.value, ac.currentTime)
      c.gain.gain.linearRampToValueAtTime(0, ac.currentTime + 0.02)
      c.osc.stop(ac.currentTime + 0.05)
    }

    const timer = window.setInterval(() => {
      const ui = useUi.getState()
      // 背後のタブでは鳴らさない（見えていない画面から音だけ出ると何の音か分からない）
      const active = ui.parkBeep && ui.armRequested && !document.hidden
      const scan = performance.now() - live.lastScanMs < SCAN_FRESH_MS ? live.scan : null
      const period = active
        ? beepPeriodMs(relevantDistance(sectorClearance(scan, live.vs), live.vs, reverseSelected(ui)))
        : null
      if (period === 0) {
        startCont()
        return
      }
      stopCont()
      if (period == null) {
        nextBeepAt = 0
        return
      }
      const now = performance.now()
      if (now >= nextBeepAt) {
        beep()
        nextBeepAt = now + period
      } else {
        // 近づいて周期が縮んだら、次の音を前倒しする
        nextBeepAt = Math.min(nextBeepAt, now + period)
      }
    }, TICK_MS)

    return () => {
      window.clearInterval(timer)
      window.removeEventListener('keydown', ensureAudio, true)
      window.removeEventListener('pointerdown', ensureAudio, true)
      stopCont()
      void ac?.close()
    }
  }, [])
}
