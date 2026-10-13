/**
 * 描画中の例外を、その部分だけで受け止める。
 *
 * React は描画中に例外が出るとツリーごと外す。境界が無いと**画面全体が白くなり**、
 * E-STOP ボタンも、指令を送り続けている `useDriving`（`App` のフック）も一緒に消える。
 * 手動走行はデッドマンで止まるが、自律走行は Wi-Fi の瞬断を待つ猶予のあいだ走り続ける。
 *
 * 例外の入口になりやすいのは `/ws/control` の `status`（実行時の型検査をしていない）で、
 * Pi 側と GUI の版がずれたときに `undefined.toFixed` の類が出る。
 *
 * **境界はタブの中身だけを囲む。** `StatusBar`（E-STOP・ARM）と `useDriving` は外に置くので、
 * 中身が落ちても止める手段は残る。落ちたときは `onError` で車も止める（呼び出し側が E-STOP を
 * 送り ARM を下ろす）——何が表示できていないか分からないまま走らせない。
 */
import { Component, type ErrorInfo, type ReactNode } from 'react'

type Props = {
  /** 何の表示か（フォールバックの文に出す） */
  name: string
  /** 例外を受けたときに1回呼ぶ。車を止めるのに使う */
  onError?: () => void
  /** 小さい場所（タブ行の部品）用。文を短くする */
  compact?: boolean
  children: ReactNode
}

type State = { error: Error | null }

export class ErrorBoundary extends Component<Props, State> {
  state: State = { error: null }

  static getDerivedStateFromError(error: Error): State {
    return { error }
  }

  componentDidCatch(error: Error, info: ErrorInfo) {
    console.error(`「${this.props.name}」の描画で例外`, error, info.componentStack)
    this.props.onError?.()
  }

  render() {
    const { error } = this.state
    if (!error) return this.props.children
    if (this.props.compact) {
      return (
        <span className="badge-bad" title={error.message}>
          {this.props.name} 表示エラー
        </span>
      )
    }
    return (
      <div style={{ padding: 16 }}>
        <p className="badge-bad">
          「{this.props.name}」の表示でエラーが起きました。車は停止させました（E-STOP・ARM 解除）。
        </p>
        <p className="dim" style={{ whiteSpace: 'pre-wrap' }}>
          {error.message}
          {'\n'}Pi 側と GUI の版が合っていない可能性があります（gui を再ビルドして配り直す）。
        </p>
        <button onClick={() => this.setState({ error: null })}>表示し直す</button>
      </div>
    )
  }
}
