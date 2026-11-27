// A-23 ② / A-22: S-14 の「再生」は位置が決まるまで押せず理由を出す。経路を選んでから
// 地図を読み込み終わるまで「地図を読み込んでいます」を地図の上に出す（Spec-webui.md §3.7）。
//
// 本番の送信を縛る: 押せる状態で押したら ui.run が実際に送られること（CLAUDE.md
// 「エージェントの試験が本番の経路を縛っているか」。READY の送信は既存の
// s14-replay-button-sends-ui-run.spec.js でも見ているが、ここでも押せる合図の判定と
// 同じシードで確かめる）。
import { test, expect } from '@playwright/test'
import {
  gotoScreenWithRoutePreview, gotoScreenWithRouteRobot, setTestState,
} from './helpers.js'

const ROUTES = [{ id: 'r1', name: '経路1', length_m: 5, point_count: 10 }]
const PREVIEW = [{ x: 0, y: 0 }, { x: 1, y: 0 }, { x: 2, y: 0 }]
const MAP = {
  info: {
    width: 2, height: 2, resolution: 0.2,
    origin: { position: { x: 0, y: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
  },
  data: [0, 0, 0, 0],
}

async function openS14(page, stateName, status, extra = {}) {
  await gotoScreenWithRouteRobot(page, 'S14',
    { mode: 'REPLAY', state: stateName },
    { routes: ROUTES, preview: PREVIEW, status, ...extra })
  await page.locator('#s14').waitFor()
}

async function triggerCalls(page) {
  return page.evaluate(() => window.__thTriggerCalls ?? [])
}

const run = (page) => page.getByTestId('stop-run-toggle-run')

for (const [quality, reason] of [
  ['searching', '探しています'],
  ['failed', '見つかりません'],
  ['off_route', '経路から離れています'],
]) {
  test(`LOCALIZE（${quality}）は「再生」が押せず、理由「${reason}」が出る`, async ({ page }) => {
    await openS14(page, 'LOCALIZE', { localize_quality: quality })
    await expect(run(page)).toBeDisabled()
    await expect(page.getByTestId('stop-run-toggle-run-reason')).toContainText(reason)
  })
}

test('LOCALIZE の未受信（古いノード）も「再生」が押せず「探しています」が出る', async ({ page }) => {
  await openS14(page, 'LOCALIZE', {})
  await expect(run(page)).toBeDisabled()
  await expect(page.getByTestId('stop-run-toggle-run-reason')).toContainText('探しています')
})

test('READY は「再生」が押せて、押すと ui.run が実際に送られる', async ({ page }) => {
  await openS14(page, 'READY', { localize_quality: 'high' })
  await expect(run(page)).toBeEnabled()
  await expect(page.getByTestId('stop-run-toggle-run-reason')).toHaveCount(0)

  await run(page).click()
  const sent = (await triggerCalls(page)).filter((c) => c.trigger === 'ui.run')
  expect(sent.length, 'READY の再生が ui.run を送っていない').toBeGreaterThan(0)
})

test('unknown（地図の画像が無い経路）でも READY なら「再生」は押せる', async ({ page }) => {
  await openS14(page, 'READY', { localize_quality: 'unknown' })
  await expect(run(page)).toBeEnabled()
  await expect(page.getByTestId('stop-run-toggle-run-reason')).toHaveCount(0)
})

test('PAUSE の「再生」（一時停止の再開）は押せて ui.run を送る', async ({ page }) => {
  await openS14(page, 'PAUSE', { target_index: 1, arrived: false })
  await expect(run(page)).toBeEnabled()
  await run(page).click()
  expect((await triggerCalls(page)).map((c) => c.trigger)).toContain('ui.run')
})

test('地図の読み込み中は地図の上に出て、読み込み後に消える', async ({ page }) => {
  await openS14(page, 'LOCALIZE', { localize_quality: 'searching' }, { map: MAP })
  const loading = page.getByTestId('s14-map-overlay')
  await expect(loading).toBeVisible()
  await expect(loading).toContainText('地図を読み込んでいます')

  // 読み込みが終わって READY へ進む（実機の evt.localize_done 相当）。
  await page.evaluate(() => window.__thSetTestRouteStatus({ localize_quality: 'high' }))
  await setTestState(page, { state: 'READY' })
  await expect(loading).toHaveCount(0)
})

test('地図のトピックが届いていない間は読み込み中を出し、届いたら消す', async ({ page }) => {
  await openS14(page, 'LOCALIZE', {})
  const loading = page.getByTestId('s14-map-overlay')
  await expect(loading).toBeVisible()

  await page.evaluate((m) => window.__thSetTestRouteMap(m), MAP)
  await expect(loading).toHaveCount(0)
})

test('READY では読み込み中を出さない', async ({ page }) => {
  await openS14(page, 'READY', { localize_quality: 'high' }, { map: MAP })
  await expect(page.getByTestId('s14-map-overlay')).toHaveCount(0)
})
