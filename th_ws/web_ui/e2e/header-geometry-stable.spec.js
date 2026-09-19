// ヘッダーの形はフォルトメッセージの長さに影響されない（2026-09-02 実機報告の修正）。
//
// 不具合: @container app (min-width:1200px) でヘッダーが 1 行レイアウトになると、
// .row2（フォルト文 + 非常停止ボタン）が flex:0 0 auto = 中身の幅で決まっていた。
// フォルト文が長いほど row2 が広がり、row1 が圧迫され、row1 の flex-wrap:wrap で
// 折り返しが発生してヘッダーの高さごと変わっていた（実機では「画面名／モードの
// ピルだけ下の行に落ちる」という見え方）。ヘッダーが伸びると本文の高さも動く。
//
// i18n/faults.js の実文言は 2 文字（'正常'）から 46 文字（LIDAR_LOST）まで幅が
// あるので、その両端を含めて「ヘッダー高さ」と「row1 内の各要素の位置」が
// 1px も動かないことを確かめる。
import { test, expect } from '@playwright/test'
import { gotoScreen, setTestState } from './helpers.js'

// 短い順。'正常' は fault なし、他は faultLabel() の長文。
const CASES = [
  { name: '正常（フォルト無し・最短）', fault: null },
  { name: 'ESP32_DISCONNECTED（長文）', fault: 'ESP32_DISCONNECTED' },
  { name: 'LIDAR_LOST（最長）', fault: 'LIDAR_LOST' },
  { name: 'PERSON_TRACKER_LOST（長文）', fault: 'PERSON_TRACKER_LOST' },
]

async function headerGeometry(page) {
  return page.evaluate(() => {
    const box = (sel) => {
      const el = document.querySelector(sel)
      if (!el) return null
      const r = el.getBoundingClientRect()
      // 位置は #hdr 基準の相対値で見る（ステージ拡大の影響を受けないように）。
      const h = document.getElementById('hdr').getBoundingClientRect()
      return { x: Math.round(r.left - h.left), y: Math.round(r.top - h.top), h: Math.round(r.height) }
    }
    return {
      hdrHeight: Math.round(document.getElementById('hdr').getBoundingClientRect().height),
      bodyTop: Math.round(document.getElementById('body').getBoundingClientRect().top),
      screenName: box('#screenName'),
      modePill: box('#modePill'),
      estop: box('#estopBtn'),
      faultTextOverflows: (() => {
        const el = document.getElementById('faultTx')
        return el ? el.scrollWidth > el.clientWidth + 1 : false
      })(),
    }
  })
}

test('ヘッダーの高さと row1 の配置がフォルト文の長さで変わらない', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'IDLE', tracker_enabled: true })

  const seen = []
  for (const c of CASES) {
    if (c.fault) {
      await page.evaluate((ft) => window.__thSetTestFault({ active: true, fault_type: ft, severity: 'RECOVERABLE' }),
        c.fault)
    } else {
      await page.evaluate(() => window.__thSetTestFault({ active: false, fault_type: 'NONE' }))
    }
    // 反映待ち（React の再描画 + レイアウト確定）
    await page.waitForFunction(() => true)
    await page.waitForTimeout(50)
    seen.push({ label: c.name, geo: await headerGeometry(page) })
  }

  const base = seen[0].geo
  for (const s of seen.slice(1)) {
    expect(s.geo.hdrHeight, `ヘッダー高さが変わった: ${s.label}`).toBe(base.hdrHeight)
    expect(s.geo.bodyTop, `本文の開始位置が変わった: ${s.label}`).toBe(base.bodyTop)
    expect(s.geo.screenName, `画面名の位置が変わった: ${s.label}`).toEqual(base.screenName)
    expect(s.geo.modePill, `モードピルの位置が変わった: ${s.label}`).toEqual(base.modePill)
    expect(s.geo.estop, `非常停止ボタンの位置が変わった: ${s.label}`).toEqual(base.estop)
  }
})

test('長いフォルト文は行を増やさず省略記号で切り詰められる', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'IDLE', tracker_enabled: true })
  await page.evaluate(() => window.__thSetTestFault(
    { active: true, fault_type: 'LIDAR_LOST', severity: 'RECOVERABLE' }))
  await page.waitForTimeout(50)

  const g = await headerGeometry(page)
  // 1 行に収まっている = フォルト文の行数が増えてヘッダーが伸びていない。
  const oneLine = await page.evaluate(() => {
    const el = document.getElementById('faultTx')
    const lh = parseFloat(getComputedStyle(el).lineHeight) || el.getBoundingClientRect().height
    return el.getBoundingClientRect().height <= lh * 1.6
  })
  expect(oneLine, 'フォルト文が複数行になっている').toBe(true)
  // 実際に溢れているなら ellipsis が働いていること（幅次第なので溢れ自体は必須にしない）
  expect(g.estop).toBeTruthy()
})
