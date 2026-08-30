/**
 * エンジン音シミュレーション — Web Audio API で GUI 側だけで合成する。
 *
 * 車両にスピーカーは無い。**鳴るのはこの画面を開いているブラウザのスピーカーだけ**
 * （STM32・車両側には一切関与しない、純粋な演出）。速度とギアから架空の「回転数」を
 * 作り、オシレータ + 高調波レイヤ + ノイズ質感 + ローパスフィルタでそれらしい音にする。
 *
 * MT モードでは「現在ギアの上限に対する速度の割合」を回転数として使うので、
 * 実車と同じく**シフトダウンで音程が上がり、シフトアップで下がる**——各ギアに
 * それぞれのレッドラインがあるので、低いギアでは低速でもレブリミットに当たる。
 * 呼び出し側は `useEngineSound.ts`（`rpmFrac` の計算・`shiftBlip()` の呼び出し）。
 *
 * ⚠ ブラウザの自動再生制限により `AudioContext` はユーザー操作（ON トグルのクリック）
 * の中でしか作れない。`start()` はその呼び出し元でだけ呼ぶこと。
 *
 * ## 音作り（2026-08-30 刷新）
 *
 * 以前は組み込みの `sawtooth`/`sine` を生のまま2発デチューンして鳴らしていたが、
 * のこぎり波は倍音が 1/n でしか減衰せず高域が耳に刺さる「電子音っぽさ」が抜けなかった。
 * 実車らしい「唸り」に寄せるため、以下を足した:
 *
 * - **カスタム倍音（`Profile.harmonics`）**: `createPeriodicWave` で低次倍音を
 *   強調しつつ高次を素早く落とすスペクトルを作る（`makeHarmonicWave`）。
 * - **サチュレーション（`driveAmount`）**: `WaveShaperNode` の tanh 系ソフトクリップで、
 *   排気音らしい「ガツン」とした押し出し感を足す（メインの唸り層だけに掛け、
 *   サブベース・ノイズ層はクリーンなまま——歪ませると低域がすぐ濁るため）。
 * - **ステレオ幅（`stereoPan`）**: デチューンした2発（osc1/osc2）を左右に振り、
 *   ブラウザのステレオスピーカー/ヘッドホンで広がりを持たせる。
 * - **アイドルの「もたつき」（`idleChugHz`/`idleChugDepth`）**: 実車はアイドル付近で
 *   気筒の点火が粗く感じられ、回転が上がるほど滑らかになる。専用 LFO で振幅を
 *   軽く揺らし、回転が上がるにつれ `update()` が深さを自動的に絞る。
 *
 * ## 音色（`EngineSoundType`）
 *
 * 波形（倍音構成）・周波数レンジ・フィルタ・ゲインの組を `PROFILES` に持ち、
 * `setType()` で**鳴っている最中でも作り直さずに差し替える**（`AudioContext` は
 * 1回作ったら使い回す——毎回作り直すとブラウザによっては自動再生制限に引っかかる）。
 *
 * ## レブリミッター演出（内燃機関だけの現象、2026-08-30 で明確化）
 *
 * 実車がレブリミットに当たると、点火が数〜十数Hzで断続的にカットされて
 * 「バババッ」と音が途切れる——これは**点火という機構がある内燃機関だけの現象**。
 * EV/モータには点火が無く、トルク制限は電流を滑らかに絞るだけで音が途切れる
 * 仕組みが無いため、`Profile.hasIgnitionCutLimiter=false`（EV）では
 * `update()` が `limiting` 判定自体を常に false にし、この演出をまるごと無効化する
 * ——パラメータを弱めているのではなく、その現象がそもそも存在しないことにしてある。
 *
 * 内燃機関側の実装は rAF ではなく**専用の LFO（`limiterLfo`）でオーディオレート
 * 変調**することで、メインスレッドのジッタに関係なく正確なリズムで再現している。
 * `limiterGain`（常に基準値 1）の `.gain` に LFO 出力を加算接続する古典的な
 * トレモロ手法——`frac`（回転数相当）がプロファイルの `limiterFrac` を超えた瞬間
 * だけ `ampModGain`/`pitchModGain`（変調の深さ）を 0 から立ち上げる。
 * LFO 自体は（EV でも）常時回しっぱなしにし、深さ 0 のときは無音（クリック音対策。
 * オシレータの start/stop を頻発させるとブツ切れノイズが出る——`hasIgnitionCutLimiter`
 * は「深さを 0 に固定する」という形で効くので、ノード自体は共通のまま安全に使い回せる）。
 * アイドルのもたつき LFO（`idleChugLfo`）も同じ理由で常時回しっぱなしにしてある。
 */
export type EngineSoundType = 'combustion' | 'ev'

type Profile = {
  /**
   * メイン音（osc1/osc2）の倍音構成 [第1次(基音)〜第n次の振幅、0-1]。
   * 組み込みの `sawtooth` 等ではなくこの配列から `createPeriodicWave` でカスタム
   * スペクトルを作る（`makeHarmonicWave`）——低次倍音を厚くし高次を素早く落とすと、
   * 生ののこぎり波よりずっと「唸り」らしくなる。
   */
  harmonics: number[]
  /** 3つ目のレイヤの波形。内燃機関風では低いサブオクターブの唸り、EV風では高い倍音のシマーに使う */
  layer3Type: OscillatorType
  /** osc2 のデチューン量 [cent]。うねり（ビート）の強さ */
  detuneCents: number
  /** osc2 の周波数比（osc1 に対して）。**あえて整数比を外すと「唸り」らしくなる**
   * （EV のギア鳴きは減速機の歯数比に由来する非整数の唸りが特徴） */
  osc2Ratio: number
  /** layer3 の周波数比 */
  layer3Ratio: number
  oscGain: number
  layer3Gain: number
  /** アイドル〜レッドライン相当の周波数レンジ [Hz] */
  idleHz: number
  redlineHz: number
  /** 停止中も完全な無音にはしないアイドルの下限（0〜1、回転数の割合）。
   * EV は実車同様ほぼ無音に近づけるため小さい値にしてある */
  idleFloor: number
  filterBase: number
  filterSpan: number
  filterQ: number
  gainBase: number
  gainSpan: number
  /** 質感を足すフィルタ済みノイズ層。内燃機関は排気・機械音のザラつき、
   * EV はインバータ/減速機のシャーというノイズ感を狙う */
  noiseFilterType: 'lowpass' | 'bandpass'
  noiseBaseHz: number
  noiseSpanHz: number
  noiseQ: number
  noiseGainMax: number
  /**
   * イグニッションカット式のレブリミッター演出を使うか（2026-08-30）。
   * **内燃機関だけの物理現象**——点火を間引いて音を強制的に途切れさせる仕組み。
   * EV/モータには点火という概念自体が無く、実際のトルク制限は電流を滑らかに
   * 絞るだけで音が途切れる仕組みが無いため、EV では `false` にしてこの演出全体を
   * 無効化する（`limiterAmpDepth` 等を弱めるのではなく、`update()` が
   * `limiting` の判定自体を常に false にする——「パラメータの調整」ではなく
   * 「その現象が存在しない」ことをコードで表す）。
   */
  hasIgnitionCutLimiter: boolean
  /** ここから上をレブリミッター演出とみなす（0〜1、`frac` と同じスケール）。
   * `hasIgnitionCutLimiter=false` のプロファイルでは無視される */
  limiterFrac: number
  /** バタつきの周波数 [Hz]。`hasIgnitionCutLimiter=false` では無視される */
  limiterRateHz: number
  limiterWave: 'square' | 'sine'
  /** 振幅変調の深さ（0〜1）。大きいほど「ブツブツ途切れる」感じが強くなる。
   * `hasIgnitionCutLimiter=false` では無視される */
  limiterAmpDepth: number
  /** ピッチ変調の深さ [cent]。`hasIgnitionCutLimiter=false` では無視される */
  limiterPitchDepthCents: number
  /** メイン音だけに掛けるサチュレーション量 [0-1]。実車の排気音らしい押し出し感。
   * EV はほぼクリーンなままにするため小さい値にする */
  driveAmount: number
  /** アイドル付近の「もたつき」LFO の周波数 [Hz]（気筒の点火間隔に相当） */
  idleChugHz: number
  /** 同、アイドルでの振幅変調の深さ [0-1]。回転が上がるほど `update()` が自動的に絞る。
   * EV はほぼ 0（実車の EV に「もたつき」は無い） */
  idleChugDepth: number
  /** osc1/osc2 を左右に振る幅 [0-1]（`StereoPannerNode` の pan 値の絶対値） */
  stereoPan: number
  /**
   * 速度 0（停止中）のとき完全に無音にするか（2026-08-30）。**EV 用。** 実車の EV/HV は
   * 走り出していないと駆動音がしない——`idleFloor` によるアイドル分のピッチ/質感計算は
   * そのまま行うが、`update()` が出力ゲイン（メイン・ノイズ両層）だけを 0 に落とす
   * （ピッチ計算を止めないのは、鳴り出す瞬間に不自然な飛びが出ないようにするため）。
   * 内燃機関は停止中もアイドリングするので `false`。
   */
  silentAtStandstill: boolean
}

const PROFILES: Record<EngineSoundType, Profile> = {
  /** 内燃機関風。低次倍音を厚くしたカスタム波形2発+デチューンでガラガラした唸りを作り、
   * サチュレーションで排気音らしい押し出し感を足す。アイドルは軽くもたつかせ、
   * レブリミットでは矩形波 LFO による鋭いバタつき（点火カット）を強めに掛ける */
  combustion: {
    harmonics: [1, 0.62, 0.78, 0.34, 0.46, 0.2, 0.26, 0.12],
    layer3Type: 'sine',
    detuneCents: 9,
    osc2Ratio: 1,
    layer3Ratio: 0.5, // 1オクターブ下のサブベース
    oscGain: 0.42,
    layer3Gain: 0.34,
    // 2026-08-30: 「もう少し低い音に」の指示でアイドル/レッドラインとフィルタの
    // 明るさを全体的に1段落とした（構成・キャラクターは変えず、音域だけシフト）
    idleHz: 40,
    redlineHz: 165,
    idleFloor: 0.07,
    filterBase: 260,
    filterSpan: 2100,
    filterQ: 1.05,
    gainBase: 0.12,
    gainSpan: 0.32,
    noiseFilterType: 'lowpass',
    noiseBaseHz: 420,
    noiseSpanHz: 1900,
    noiseQ: 0.6,
    noiseGainMax: 0.14,
    hasIgnitionCutLimiter: true,
    limiterFrac: 0.97,
    limiterRateHz: 14,
    limiterWave: 'square',
    limiterAmpDepth: 0.55,
    limiterPitchDepthCents: 70,
    driveAmount: 0.4,
    idleChugHz: 23,
    idleChugDepth: 0.3,
    stereoPan: 0.38,
    silentAtStandstill: false,
  },
  /**
   * EV/フォーミュラE風（2026-08-30 再刷新）。**内燃機関と「本質的に違う音」を
   * 目指し、単なる周波数・ゲインの数値違いではなく合成の仕方自体を変えてある**:
   *
   * - **倍音をほぼ削いだ純音に近いスペクトル**（`harmonics`）。内燃機関の
   *   「唸り」は倍音の多さ（ガラガラ感）から来るが、EV/モータの音は逆に
   *   倍音が少なくクリーンな単一トーンに近い——ここを内燃機関と共有すると
   *   「同じ合成方法に違う数値を入れただけ」になってしまうため、明確に分けた。
   * - **サチュレーションはほぼ無し**（`driveAmount` を大幅に下げた）。内燃機関の
   *   「ガツン」とした押し出し感（歪み）は排気音特有のもので、EV には無い。
   * - **個性は倍音ではなく「ビート（うなり）」と「共鳴」で作る**。osc2/layer3を
   *   基音のごく近く（ユニゾンに近い比率）にデチューンし、干渉によるゆっくりした
   *   唸り・キーンという響きを作る（減速機のギア鳴み・インバータのキャリア音の
   *   イメージ）。フィルタの Q を高くして「ヒュイィン」という共鳴の効いた
   *   質感を加える——これも倍音ではなく共鳴で個性を出す発想。
   * - **音域は内燃機関よりは高いが、指示により全体的に落とした**（以前は
   *   220〜1450Hz と高すぎた）。
   *
   * **停止中は完全に無音のまま**（`silentAtStandstill`。実車の EV/HV も
   * フォーミュラEも、走り出していないと駆動音がしないため）。もたつきは無し
   * （リプロカティング機構が無いので「気筒の点火」に相当するものが無い）。
   */
  ev: {
    // ほぼ純音（基音+ごく薄い2次・3次倍音だけ）。内燃機関のような倍音の多さは
    // 意図的に持たせない——ここが「音色そのものが違う」の核心
    harmonics: [1, 0.05, 0.03],
    layer3Type: 'sine',
    detuneCents: 15, // ユニゾンに近いデチューンで「唸り」を作る（倍音ではなくビート）
    osc2Ratio: 1.004, // ほぼユニゾン。僅かなズレがゆっくりしたビートになる
    layer3Ratio: 2.008, // ほぼ1オクターブ上。僅かにズラして高音側にも薄いビートを足す
    oscGain: 0.42,
    layer3Gain: 0.16,
    idleHz: 130,
    redlineHz: 720,
    idleFloor: 0.04,
    filterBase: 850,
    filterSpan: 2400,
    filterQ: 3.4, // 高いQでレゾナンスの効いた「ヒュイィン」を作る（倍音の多さではなく共鳴で個性を出す）
    gainBase: 0.03,
    gainSpan: 0.28,
    noiseFilterType: 'bandpass',
    noiseBaseHz: 3000,
    noiseSpanHz: 2200,
    noiseQ: 2.8,
    noiseGainMax: 0.06,
    // EV/モータに点火カットは無い。以下4項目は hasIgnitionCutLimiter=false により
    // 無視される（値そのものに意味は無く、型を満たすためのプレースホルダ）
    hasIgnitionCutLimiter: false,
    limiterFrac: 0.97,
    limiterRateHz: 13,
    limiterWave: 'square',
    limiterAmpDepth: 0.3,
    limiterPitchDepthCents: 45,
    driveAmount: 0.02, // ほぼクリーン。歪みは内燃機関の排気音特有のものなのでEVには持たせない
    idleChugHz: 18,
    idleChugDepth: 0,
    stereoPan: 0.25,
    silentAtStandstill: true,
  },
}

/** `harmonics[n]`（n=0が基音）を振幅とする加算合成スペクトルの `PeriodicWave` を作る。
 * 組み込み波形と違い、低次を厚く高次を素早く落とすなど自由に整形できる */
function makeHarmonicWave(ctx: AudioContext, harmonics: number[]): PeriodicWave {
  const real = new Float32Array(harmonics.length + 1)
  const imag = new Float32Array(harmonics.length + 1)
  for (let n = 0; n < harmonics.length; n++) imag[n + 1] = harmonics[n] ?? 0
  return ctx.createPeriodicWave(real, imag, { disableNormalization: false })
}

/**
 * ソフトクリップ（tanh）のウェーブシェイパーカーブ。`amount=0` で実質恒等関数
 * （分母・分子とも同じ極限に潰れるので `x` に収束する）、大きいほど強く潰れて
 * 倍音が増える＝「歪んだ排気音」らしくなる。ピークが常に ±1 になるよう正規化してある
 * ので、`amount` を変えても音量が急に跳ねない。
 */
function makeDriveCurve(amount: number): Float32Array {
  const n = 1024
  const curve = new Float32Array(n)
  const k = Math.max(0.0005, amount * 18)
  const norm = Math.tanh(k)
  for (let i = 0; i < n; i++) {
    const x = (i / (n - 1)) * 2 - 1
    curve[i] = Math.tanh(k * x) / norm
  }
  return curve
}

export class EngineSound {
  private ctx: AudioContext | null = null
  private osc1: OscillatorNode | null = null
  private osc2: OscillatorNode | null = null
  private layer3: OscillatorNode | null = null
  private osc1Gain: GainNode | null = null
  private osc2Gain: GainNode | null = null
  private pannerL: StereoPannerNode | null = null
  private pannerR: StereoPannerNode | null = null
  private drive: WaveShaperNode | null = null
  private layer3Gain: GainNode | null = null
  private filter: BiquadFilterNode | null = null
  private noise: AudioBufferSourceNode | null = null
  private noiseFilter: BiquadFilterNode | null = null
  private noiseGain: GainNode | null = null
  private limiterLfo: OscillatorNode | null = null
  private ampModGain: GainNode | null = null
  private pitchModGain: GainNode | null = null
  private idleChugLfo: OscillatorNode | null = null
  private idleChugModGain: GainNode | null = null
  private master: GainNode | null = null
  private type: EngineSoundType = 'combustion'

  /** ユーザー操作のハンドラの中で呼ぶこと（自動再生制限を通すため） */
  start(type: EngineSoundType = this.type) {
    if (this.ctx) return
    this.type = type
    const ctx = new AudioContext()

    const osc1 = ctx.createOscillator()
    const osc2 = ctx.createOscillator()
    const layer3 = ctx.createOscillator()

    const osc1Gain = ctx.createGain()
    const osc2Gain = ctx.createGain()
    const pannerL = ctx.createStereoPanner()
    const pannerR = ctx.createStereoPanner()
    const drive = ctx.createWaveShaper()
    const layer3Gain = ctx.createGain()

    // アイドルの「もたつき」用の直列ゲイン。基準値1（無変調時は完全に透過）
    const idleGain = ctx.createGain()
    idleGain.gain.value = 1
    const idleChugLfo = ctx.createOscillator()
    idleChugLfo.type = 'sine'
    idleChugLfo.frequency.value = 20
    const idleChugModGain = ctx.createGain()
    idleChugModGain.gain.value = 0 // 深さ0＝無変調。update() が回転数に応じて調整する

    const filter = ctx.createBiquadFilter()
    filter.type = 'lowpass'
    filter.frequency.value = 300

    // ── ノイズ質感層。1〜2秒ループの白色雑音をフィルタで色付けする ──
    const noiseBuf = ctx.createBuffer(1, ctx.sampleRate * 2, ctx.sampleRate)
    const data = noiseBuf.getChannelData(0)
    for (let i = 0; i < data.length; i++) data[i] = Math.random() * 2 - 1
    const noise = ctx.createBufferSource()
    noise.buffer = noiseBuf
    noise.loop = true
    const noiseFilter = ctx.createBiquadFilter()
    const noiseGain = ctx.createGain()
    noiseGain.gain.value = 0

    // ── レブリミッターの振幅/ピッチ変調 ──
    // limiterGain は常に基準値1（無変調時は完全に透過）。LFO 出力を
    // ampModGain（深さ）経由で `.gain` に加算し、古典的なトレモロを作る
    const limiterGain = ctx.createGain()
    limiterGain.gain.value = 1
    const limiterLfo = ctx.createOscillator()
    limiterLfo.type = 'square'
    limiterLfo.frequency.value = 14
    const ampModGain = ctx.createGain()
    ampModGain.gain.value = 0 // 深さ0＝無変調。update() が limiterFrac 超えで立ち上げる
    const pitchModGain = ctx.createGain()
    pitchModGain.gain.value = 0

    const master = ctx.createGain()
    master.gain.value = 0 // update() が来るまで無音

    // osc1/osc2 は左右に振ってステレオ幅を出し、idleGain（もたつき）→
    // drive（サチュレーション）を通してから filter へ。サブベース（layer3）と
    // ノイズ層はクリーンなまま filter で直接合流させる（歪ませると低域が濁るため）
    osc1.connect(osc1Gain)
    osc2.connect(osc2Gain)
    osc1Gain.connect(pannerL)
    osc2Gain.connect(pannerR)
    pannerL.connect(idleGain)
    pannerR.connect(idleGain)
    idleGain.connect(drive)
    drive.connect(filter)

    layer3.connect(layer3Gain)
    layer3Gain.connect(filter)

    noise.connect(noiseFilter)
    noiseFilter.connect(noiseGain)
    noiseGain.connect(limiterGain) // ノイズ層もレブリミッターで一緒に「途切れる」

    filter.connect(limiterGain)
    limiterGain.connect(master)
    master.connect(ctx.destination)

    limiterLfo.connect(ampModGain)
    ampModGain.connect(limiterGain.gain)
    limiterLfo.connect(pitchModGain)
    pitchModGain.connect(osc1.detune)
    pitchModGain.connect(osc2.detune)

    idleChugLfo.connect(idleChugModGain)
    idleChugModGain.connect(idleGain.gain)

    osc1.start()
    osc2.start()
    layer3.start()
    noise.start()
    limiterLfo.start()
    idleChugLfo.start()

    this.ctx = ctx
    this.osc1 = osc1
    this.osc2 = osc2
    this.layer3 = layer3
    this.osc1Gain = osc1Gain
    this.osc2Gain = osc2Gain
    this.pannerL = pannerL
    this.pannerR = pannerR
    this.drive = drive
    this.layer3Gain = layer3Gain
    this.filter = filter
    this.noise = noise
    this.noiseFilter = noiseFilter
    this.noiseGain = noiseGain
    this.limiterLfo = limiterLfo
    this.ampModGain = ampModGain
    this.pitchModGain = pitchModGain
    this.idleChugLfo = idleChugLfo
    this.idleChugModGain = idleChugModGain
    this.master = master

    this.applyProfile(type)
  }

  stop() {
    if (!this.ctx) return
    this.osc1?.stop()
    this.osc2?.stop()
    this.layer3?.stop()
    this.noise?.stop()
    this.limiterLfo?.stop()
    this.idleChugLfo?.stop()
    void this.ctx.close()
    this.ctx = null
    this.osc1 = this.osc2 = this.layer3 = null
    this.osc1Gain = this.osc2Gain = null
    this.pannerL = this.pannerR = null
    this.drive = null
    this.layer3Gain = null
    this.filter = null
    this.noise = null
    this.noiseFilter = null
    this.noiseGain = null
    this.limiterLfo = null
    this.ampModGain = null
    this.pitchModGain = null
    this.idleChugLfo = null
    this.idleChugModGain = null
    this.master = null
  }

  /** 音色を切り替える。**鳴っている最中でも波形・ミックスだけ差し替え、作り直さない** */
  setType(type: EngineSoundType) {
    this.type = type
    if (this.ctx) this.applyProfile(type)
  }

  private applyProfile(type: EngineSoundType) {
    const ctx = this.ctx
    if (
      !ctx || !this.osc1 || !this.osc2 || !this.layer3 || !this.osc1Gain || !this.osc2Gain ||
      !this.pannerL || !this.pannerR || !this.drive || !this.layer3Gain ||
      !this.filter || !this.noiseFilter || !this.limiterLfo || !this.idleChugLfo
    ) {
      return
    }
    const p = PROFILES[type]
    const wave = makeHarmonicWave(ctx, p.harmonics)
    this.osc1.setPeriodicWave(wave)
    this.osc2.setPeriodicWave(wave)
    this.layer3.type = p.layer3Type
    this.osc2.detune.value = p.detuneCents
    this.osc1Gain.gain.value = p.oscGain
    this.osc2Gain.gain.value = p.oscGain
    this.pannerL.pan.value = -p.stereoPan
    this.pannerR.pan.value = p.stereoPan
    this.drive.curve = makeDriveCurve(p.driveAmount) as Float32Array<ArrayBuffer>
    this.layer3Gain.gain.value = p.layer3Gain
    this.filter.Q.value = p.filterQ
    this.noiseFilter.type = p.noiseFilterType
    this.noiseFilter.Q.value = p.noiseQ
    this.limiterLfo.type = p.limiterWave
    this.limiterLfo.frequency.value = p.limiterRateHz
    this.idleChugLfo.frequency.value = p.idleChugHz
  }

  /**
   * 毎フレーム呼ぶ。`rpmFrac`（0〜1、アイドル〜レッドライン相当）と
   * `armed`（音を出してよいか＝エンジンがかかっているか）を渡す。
   * **急な値変化は `setTargetAtTime` でなめらかにする**（プチノイズ対策）。
   */
  update(rpmFrac: number, armed: boolean) {
    const ctx = this.ctx
    if (
      !ctx || !this.osc1 || !this.osc2 || !this.layer3 || !this.filter || !this.master ||
      !this.noiseFilter || !this.noiseGain || !this.ampModGain || !this.pitchModGain ||
      !this.idleChugModGain
    ) {
      return
    }
    const p = PROFILES[this.type]
    const now = ctx.currentTime
    const smooth = 0.05 // なめらかさの時定数 [s]

    const frac = armed ? p.idleFloor + rpmFrac * (1 - p.idleFloor) : 0
    const freq = p.idleHz + (p.redlineHz - p.idleHz) * frac

    // **EV は停止中（rpmFrac≈0）完全に無音**（`silentAtStandstill`）。ピッチ/フィルタの
    // 計算自体は止めない——出力ゲインだけ0にすることで、鳴り出す瞬間の飛びを防ぐ
    const standstillSilent = p.silentAtStandstill && rpmFrac <= 0.001
    const audible = armed && !standstillSilent

    this.osc1.frequency.setTargetAtTime(freq, now, smooth)
    this.osc2.frequency.setTargetAtTime(freq * p.osc2Ratio, now, smooth)
    this.layer3.frequency.setTargetAtTime(freq * p.layer3Ratio, now, smooth)

    // 回転が上がるほど音を明るく・大きくする（負荷がかかっている感じを出す）
    this.filter.frequency.setTargetAtTime(p.filterBase + frac * p.filterSpan, now, smooth)
    const gain = audible ? p.gainBase + frac * p.gainSpan : 0
    this.master.gain.setTargetAtTime(gain, now, smooth)

    this.noiseFilter.frequency.setTargetAtTime(p.noiseBaseHz + frac * p.noiseSpanHz, now, smooth)
    this.noiseGain.gain.setTargetAtTime(audible ? frac * p.noiseGainMax : 0, now, smooth)

    // ── アイドルの「もたつき」。低回転ほど深く、回転が上がるほど自動的に消える ──
    // （実車は高回転になるほど点火が滑らかにつながって聞こえる）
    const idleAmount = audible ? Math.max(0, 1 - rpmFrac * 2.2) : 0
    this.idleChugModGain.gain.setTargetAtTime(idleAmount * p.idleChugDepth, now, 0.08)

    // ── レブリミッター。frac が上限に張り付いたときだけ LFO 変調の深さを開く。
    // 立ち上がりは速く（当たった瞬間にすぐバタつく）、収まりは少し遅くする ──
    const limiting = p.hasIgnitionCutLimiter && audible && frac >= p.limiterFrac
    const limiterSmooth = limiting ? 0.02 : 0.08
    this.ampModGain.gain.setTargetAtTime(limiting ? p.limiterAmpDepth : 0, now, limiterSmooth)
    this.pitchModGain.gain.setTargetAtTime(limiting ? p.limiterPitchDepthCents : 0, now, limiterSmooth)
  }

  /**
   * MT のシフト操作時に呼ぶ。クラッチを切った一瞬の音の落ち込みを模す（音色共通）。
   * **`rpmFrac` の連続的な計算とは無関係のワンショット演出**——ギアを変えても
   * 定常的な音程は変わらない（`useEngineSound.ts` 参照）。フィルタを一瞬だけ
   * 閉じてすぐ戻すことで「一瞬こもって戻る」という体感を作る。
   */
  shiftBlip() {
    const ctx = this.ctx
    if (!ctx || !this.filter) return
    const now = ctx.currentTime
    const cur = this.filter.frequency.value
    this.filter.frequency.cancelScheduledValues(now)
    this.filter.frequency.setValueAtTime(cur, now)
    this.filter.frequency.linearRampToValueAtTime(Math.max(120, cur * 0.4), now + 0.05)
    this.filter.frequency.linearRampToValueAtTime(cur, now + 0.2)
  }
}
