/**
 * 操縦権の共有トークン。
 *
 * サーバ（`telemetry_node --token`）がトークンを設定していると、`take_control`
 * にこれを添えないと操縦権が取れない。**止める操作（E-Stop・解放）には要らない**
 * ——止める指令に条件を足すと、一番止めたい状況で止まらなくなる。
 *
 * ## 受け取り方
 *
 * 一度だけ `http://surge.local:8000/?token=xxxx` で開くと `localStorage` に入り、
 * 以後は素の URL で繋がる。**URL からはすぐ消す**（アドレスバーに残ったまま
 * スクリーンショットや画面共有に載るのを防ぐ）。
 *
 * 想定しているのは攻撃者というより、**隣の班の PC が誤って繋ぐ事故**。
 * だから強度より「一度入れたら忘れてよい」ことを優先している。
 */
const KEY = 'surge.token'

/** URL の `?token=` を localStorage に取り込み、アドレスバーからは消す。 */
export function adoptTokenFromUrl(): void {
  const url = new URL(location.href)
  const t = url.searchParams.get('token')
  if (t === null) return
  if (t) localStorage.setItem(KEY, t)
  else localStorage.removeItem(KEY)   // `?token=` 空 = 消したいという意思
  url.searchParams.delete('token')
  history.replaceState(null, '', url.pathname + url.search + url.hash)
}

/** 保存済みトークン。**無ければ空文字**（サーバ側が未設定なら通る）。 */
export function authToken(): string {
  return localStorage.getItem(KEY) ?? ''
}
