// shell/FixedStage.jsx — 固定論理キャンバスを画面いっぱいに拡大する外枠。
//
// 中身（#app 以下）は常に stageMetrics() が返す論理サイズちょうどの箱に
// 描かれ、この箱ごと transform: scale() で拡大される。したがって
// レイアウトは端末の画面サイズに一切依存しない（2026-09-02 の変更意図）。
//
// 拡大率の計算そのものは shell/stageMetrics.js の純関数側にあり、
// ここは「測って CSS 変数に流す」だけに絞ってある。
import { useEffect, useState } from 'react'
import { stageMetrics } from './stageMetrics.js'

function measure() {
  // visualViewport があればそちらが正。iOS Safari ではソフトキーボードや
  // ツールバーの出入りで innerHeight が実際の可視領域とずれる。
  const vv = typeof window !== 'undefined' ? window.visualViewport : null
  const w = vv?.width ?? window.innerWidth
  const h = vv?.height ?? window.innerHeight
  return stageMetrics(w, h)
}

export default function FixedStage({ children }) {
  const [metrics, setMetrics] = useState(measure)

  useEffect(() => {
    const onResize = () => setMetrics(measure())
    onResize()   // マウント直後の実測で初期値を上書きする
    window.addEventListener('resize', onResize)
    window.addEventListener('orientationchange', onResize)
    window.visualViewport?.addEventListener('resize', onResize)
    return () => {
      window.removeEventListener('resize', onResize)
      window.removeEventListener('orientationchange', onResize)
      window.visualViewport?.removeEventListener('resize', onResize)
    }
  }, [])

  return (
    <div
      id="stage"
      data-orientation={metrics.orientation}
      data-testid="stage"
      style={{
        '--stage-w': `${metrics.width}px`,
        '--stage-h': `${metrics.height}px`,
        '--stage-scale': metrics.scale,
      }}
    >
      {children}
    </div>
  )
}
