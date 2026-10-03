// W-01 P3: S-14 の確度表示・「完全グローバルで探す」ボタン
// （DetailedDesign-transit-localize-options.md §4）。
// /route/status の localize_quality（ROS 側 P2 が足す。未マージの間は
// rosbridge モック＝__thTestRouteStatus から流す）を見る。
import { test, expect } from '@playwright/test'
import { gotoScreenWithRoutePreview } from './helpers.js'

const ROUTES = [{ id: 'r1', name: '経路1', length_m: 5, point_count: 10 }]

async function openS14(page, stateName, status) {
  await gotoScreenWithRoutePreview(page, 'S14',
    { mode: 'REPLAY', state: stateName },
    { routes: ROUTES, preview: [], status })
  await page.locator('#s14').waitFor()
}

async function triggerCalls(page) {
  return page.evaluate(() => window.__thTriggerCalls ?? [])
}

test('(a) failed でボタンが出て押すと ui.localize_global が実際に送信される', async ({ page }) => {
  await openS14(page, 'LOCALIZE', { localize_quality: 'failed' })
  await expect(page.getByTestId('s14-localize-quality')).toContainText('見つかりません')
  await expect(page.getByTestId('s14-localize-quality')).toContainText('経路を選び直し')

  const btn = page.getByTestId('s14-localize-global')
  await expect(btn).toBeVisible()
  await expect(btn).toBeEnabled()
  await btn.click()

  const sent = (await triggerCalls(page)).filter((c) => c.trigger === 'ui.localize_global')
  expect(sent.length, '完全グローバルボタンが ui.localize_global を送っていない').toBeGreaterThan(0)
})

test('(b) low_margin で警告が出て READY の再生ボタンが押せる', async ({ page }) => {
  await openS14(page, 'READY', { localize_quality: 'low_margin' })
  const quality = page.getByTestId('s14-localize-quality')
  await expect(quality).toContainText('低い')
  await expect(quality).toContainText('似た場所が複数あり')
  // 警告のみで再生は止めない（設計判断。options §4）。
  await expect(quality).not.toContainText('高い')

  const run = page.locator('.opsgrid .op-run')
  await expect(run).toBeEnabled()
  await run.click()
  expect((await triggerCalls(page)).map((c) => c.trigger), 'READY の再生が ui.run を送っていない')
    .toContain('ui.run')
})

test('(c) READY では「完全グローバルで探す」が出ない', async ({ page }) => {
  await openS14(page, 'READY', { localize_quality: 'low_margin' })
  await expect(page.getByTestId('s14-localize-global')).toHaveCount(0)
})

test('(c) RUN では「完全グローバルで探す」が出ない', async ({ page }) => {
  await openS14(page, 'RUN', {})
  await expect(page.getByTestId('s14-localize-global')).toHaveCount(0)
})

test('(c) PAUSE では「完全グローバルで探す」が出ない', async ({ page }) => {
  await openS14(page, 'PAUSE', {})
  await expect(page.getByTestId('s14-localize-global')).toHaveCount(0)
})

test('(d) searching で 60 秒注記が出ない（ボタンは押せない）', async ({ page }) => {
  await page.clock.install()
  await openS14(page, 'LOCALIZE', { localize_quality: 'searching' })
  await expect(page.getByTestId('s14-localize-quality')).toContainText('探しています')

  // 探索中はボタン自体は出すが押せない。
  const btn = page.getByTestId('s14-localize-global')
  await expect(btn).toBeVisible()
  await expect(btn).toBeDisabled()

  // 60 秒を超えても「長引いている」注記は出さない（探索＋地図の読み直しで超えうる）。
  await page.clock.fastForward(61000)
  await expect(page.getByTestId('s14-localize-stuck')).toHaveCount(0)
})

test('high は「高い」と出る（READY。数値は出さない）', async ({ page }) => {
  await openS14(page, 'READY', { localize_quality: 'high' })
  const quality = page.getByTestId('s14-localize-quality')
  await expect(quality).toContainText('高い')
  await expect(quality).not.toContainText('低い')
})

test('unknown・未受信（旧経路）は確度欄を出さない', async ({ page }) => {
  await openS14(page, 'LOCALIZE', { localize_quality: 'unknown' })
  await expect(page.getByTestId('s14-localize-quality')).toHaveCount(0)
  // ボタンは出す（旧経路でも最初から完全グローバルを選べる。Spec-transit §4.1 手順 4）。
  await expect(page.getByTestId('s14-localize-global')).toBeVisible()
})
