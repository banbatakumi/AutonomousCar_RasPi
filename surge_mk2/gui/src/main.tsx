import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import { App } from './App'
import { adoptTokenFromUrl } from './ws/token'
import './styles.css'

// **React を立てる前に。** `?token=` を localStorage に移してアドレスバーから消す。
// 描画後だと、トークン付き URL が一瞬でも表示されたまま残る
adoptTokenFromUrl()

createRoot(document.getElementById('root')!).render(
  <StrictMode>
    <App />
  </StrictMode>,
)
