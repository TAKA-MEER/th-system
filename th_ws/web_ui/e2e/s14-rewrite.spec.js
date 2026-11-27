// S-14 教示再生の作り直し（Spec-webui.md §3.7・§3.3）。
// - 型「走らせる」（RunLayout）。タブを使わない。data-pos の位置は状態で動かない。
// - 入口→再生中→一時停止→終点で右列の位置（data-pos）が動かないこと。
// - 一時停止中の「経路を選び直す」は選択カードを出し直し、進むと ui.route_select を送る。
// - 終端では経路名と「終点」が出る。地図の更新の切り替えは出さない。
// - 「手動」は W-6 を開く。再生速度は走行中も切り替えられる。
import { test, expect } from '@playwright/test'
import {
  gotoScreenWithRoutePreview, gotoScreenWithRouteCatalog, setTestState,
} from './helpers.js'

const ROUTES = [
  { id: 'route_r1', name: '経路A', length_m: 3.14, point_count: 12 },
  { id: 'route_r2', name: '経路B', length_m: 5.55, point_count: 21 },
]
const PREVIEW = [{ x: 0, y: 0 }, { x: 1, y: 0 }, { x: 2, y: 0 }]

async function openS14(page, stateName, status) {
  await gotoScreenWithRoutePreview(page, 'S14',
    { mode: 'REPLAY', state: stateName },
    { routes: ROUTES, preview: PREVIEW, status })
  await page.locator('#s14').waitFor()
}

async function triggerCalls(page) {
  return page.evaluate(() => window.__thTriggerCalls ?? [])
}

// data-pos の現在の位置（左上隅の相対座標）を返す。同じ画面の中で状態が変わっても
// どの枠も動かない（§3.3。右列・下端のボタンはどの状態でも位置を変えない）。
async function posMap(page) {
  return page.evaluate(() => {
    const out = {}
    for (const el of document.querySelectorAll('#s14 [data-pos]')) {
      const r = el.getBoundingClientRect()
      // 位置＝左上隅。理由の吹き出しで高さが伸びるぶんは位置の移動に含めない。
      out[el.dataset.pos] = [Math.round(r.left), Math.round(r.top)]
    }
    return out
  })
}

test('S-14 はタブを使わず、状態を変えても data-pos の位置が動かない', async ({ page }) => {
  await page.setViewportSize({ width: 1280, height: 720 })

  // 経路選択（入口）。
  await gotoScreenWithRouteCatalog(page, 'S14', { mode: 'REPLAY', state: 'ROUTE_SEL' }, ROUTES)
  await page.locator('#s14').waitFor()
  await expect(page.locator('#s14 [role="tab"]')).toHaveCount(0)
  await expect(page.getByTestId('s14-select-card')).toBeVisible()
  const posSel = await posMap(page)

  // 再生中へ。
  await setTestState(page, { state: 'RUN' })
  await page.evaluate(() => window.__thSetTestRouteStatus({
    cross_track_m: 0.08, cross_track_max_m: 0.21, target_index: 1,
  }))
  await expect(page.getByTestId('s14-xtrack')).toContainText('いま 0.08 m')
  await expect(page.getByTestId('s14-xtrack')).toContainText('最大 0.21 m')
  const posRun = await posMap(page)

  // 一時停止 → 終端。
  await setTestState(page, { state: 'PAUSE' })
  await page.evaluate(() => window.__thSetTestRouteStatus({ arrived: true }))
  await expect(page.getByTestId('s14-ended')).toContainText('終点')
  const posEnd = await posMap(page)

  for (const pos of ['opbar', 'main', 'where', 'toggle', 'ops', 'jog']) {
    expect(posRun[pos], `再生中に動いた枠: ${pos}`).toEqual(posSel[pos])
    expect(posEnd[pos], `終端で動いた枠: ${pos}`).toEqual(posSel[pos])
  }
})

test('S-14 は「経路を選び直す」で一時停止中だけ選択カードを出し直し、進むと ui.route_select を送る', async ({ page }) => {
  await openS14(page, 'PAUSE', { arrived: false, target_index: 1 })
  await expect(page.getByTestId('s14-select-card')).toHaveCount(0)

  await page.getByTestId('s14-reselect').click()
  await expect(page.getByTestId('s14-select-card')).toBeVisible()
  await expect(page.getByTestId('stop-run-toggle-run')).toBeDisabled()

  await page.getByTestId('s14-select-route_r2').click()
  await page.getByTestId('s14-dir-rev').click()
  await page.getByTestId('s14-proceed').click()

  const sel = (await triggerCalls(page)).find((c) => c.trigger === 'ui.route_select')
  expect(sel, '選び直して進んでも ui.route_select を送っていない').toBeTruthy()
  expect(sel.argJson.id).toBe('route_r2')
  expect(sel.argJson.reverse).toBe(true)
})

test('S-14 は再生中は「経路を選び直す」が押せない', async ({ page }) => {
  await openS14(page, 'RUN', { target_index: 1 })
  await expect(page.getByTestId('s14-reselect')).toBeDisabled()
  await page.getByTestId('s14-reselect').click({ force: true })
  await expect(page.getByTestId('s14-reselect-reason')).toContainText('一時停止中に選び直せます')
})

test('S-14 は終端で経路名と「終点」が出て、再開の案内ではない', async ({ page }) => {
  await openS14(page, 'PAUSE', {
    arrived: true, target_index: 1, current: { id: 'route_r1', name: '経路A' },
  })
  await expect(page.getByTestId('s14-ended')).toContainText('経路A')
  await expect(page.getByTestId('s14-ended')).toContainText('終点')
  await expect(page.getByTestId('s14-pause-guide')).toContainText('記録の終端に達しました')
  // 終端ではトグルの「再生する」は出ない（再開の案内は出さない）。
  await expect(page.getByTestId('stop-run-toggle-run')).toBeDisabled()
})

test('S-14 は終端で無い一時停止は「再開する」をゆっくり明滅させて案内する', async ({ page }) => {
  await openS14(page, 'PAUSE', { arrived: false, target_index: 1 })
  const run = page.getByTestId('stop-run-toggle-run')
  await expect(run).toContainText('再開する')
  await expect(run).toBeEnabled()
  await expect(run).toHaveClass(/blink-slow/)
  await expect(page.getByTestId('s14-pause-guide')).toContainText('止まったところから')
})

test('S-14 は「手動」で W-6 を開く', async ({ page }) => {
  await page.setViewportSize({ width: 1280, height: 720 })
  await gotoScreenWithRoutePreview(page, 'S14',
    { mode: 'REPLAY', state: 'RUN', zone: 'IN', jog_active: true },
    { routes: ROUTES, preview: PREVIEW, status: { target_index: 1 } })
  await page.locator('#s14').waitFor()
  await page.getByTestId('s14-manual').click()
  await expect(page.locator('#jogWin.show')).toBeVisible()
})

test('S-14 は再生速度を走行中も切り替えられる', async ({ page }) => {
  await openS14(page, 'RUN', { target_index: 1 })
  const speed = page.getByTestId('s14-replay-speed')
  await expect(speed).toBeVisible()
  const high = speed.locator('button', { hasText: '高速' })
  await expect(high).toBeEnabled()
  await high.click()
  await expect(high).toHaveClass(/on/)
})

test('S-14 は地図の更新の切り替えを出さない（どの状態でも）', async ({ page }) => {
  for (const st of ['ROUTE_SEL', 'LOCALIZE', 'READY', 'RUN', 'PAUSE']) {
    await openS14(page, st, st === 'PAUSE' ? { arrived: false } : {})
    await expect(page.getByTestId('s14-map-update-toggle')).toHaveCount(0)
    await expect(page.locator('#s14')).not.toContainText('地図の更新')
  }
})
