// e2e/s30-s40-positions.spec.js — UIR-P7: 配置の固定（Spec-webui §3.3 (a)〜(c)）。
//
// 状態を変えても data-pos の位置が動かないこと、押せないボタンは位置を
// 保ったまま押せない見た目で出る（隠さない）ことを縛る。S-30 の下端
// 「やめる／確かめる」と S-40 の「やめる／進む」、S-31 の下端が対象。
import { test, expect } from '@playwright/test'
import { stubTrigger } from './helpers.js'

async function posOf(page, pos) {
  return page.evaluate((p) => {
    const el = document.querySelector(`[data-pos="${p}"]`)
    if (!el) return null
    const r = el.getBoundingClientRect()
    return { x: Math.round(r.x), y: Math.round(r.y), w: Math.round(r.width), h: Math.round(r.height) }
  }, pos)
}

test('S-30: 確認の前後で下端の位置（data-pos="foot"）が動かない', async ({ page }) => {
  await stubTrigger(page, { 'ui.check_item': { accepted: true } })
  await page.addInitScript(() => {
    window.__thTestState = { mode: 'OPCHECK', state: 'LIST' }
    window.__thTestScreen = 'S30'
  })
  await page.goto('/')
  await page.locator('#body').waitFor()
  const before = await posOf(page, 'foot')
  expect(before).not.toBeNull()

  // 確かめるを押して RUNNING_CHECK へ（S-30 は確認を始める）。
  await page.getByTestId('s30-item-MOTOR').click()
  await page.getByTestId('s30-confirm').click()
  await page.evaluate(() => window.__thSetTestState({ state: 'RUNNING_CHECK' }))
  await page.evaluate(() => window.__thSetTestOpcheckStatus({ item: 'MOTOR', result: 'UNKNOWN', detail: 'done= hold=NONE', next_screen: '' }))

  const after = await posOf(page, 'foot')
  expect(after).not.toBeNull()
  expect(after.x).toBe(before.x)
  expect(after.y).toBe(before.y)
  expect(after.w).toBe(before.w)
  // 候補の「やめる」は実行中に押せる、「確かめる」は押せない（位置は保つ）。
  await expect(page.getByTestId('s30-abort')).toBeEnabled()
  await expect(page.getByTestId('s30-confirm')).toBeDisabled()
})

test('S-40: 段が進んでも下端の位置（data-pos="foot"）が動かない。走行中は「進む」だけ押せない', async ({ page }) => {
  await stubTrigger(page, { 'ui.calib_next': { accepted: true } })
  await page.addInitScript(() => {
    window.__thTestState = { mode: 'CALIB', state: 'S1' }
    window.__thTestScreen = 'S40'
    window.__thTestCalibStatus = {
      item: 'LINEAR', step: 'S1', result: 'GUIDE', preview_before: '', preview_after: '',
      detail: { commanded: 1.0, current: { wheel_radius_scale: 1.0 }, progress: 0.4, warnings: [], history: {}, last_calibrated: {}, last_verification: {} },
    }
  })
  await page.goto('/')
  await page.locator('#body').waitFor()
  const before = await posOf(page, 'foot')
  expect(before).not.toBeNull()
  await expect(page.getByTestId('s40-next')).toHaveText('走らせる')
  await expect(page.getByTestId('s40-next')).toHaveClass(/btn-adv/)

  // S2（走行中）へ。進むは押せない（理由を出す）。左「やめる」は押せる。
  await page.evaluate(() => window.__thSetTestState({ state: 'S2' }))
  await page.evaluate(() => window.__thSetTestCalibStatus({
    item: 'LINEAR', step: 'S2', result: 'RUNNING', preview_before: '', preview_after: '',
    detail: { commanded: 1.0, progress: 0.4, history: {}, last_calibrated: {} },
  }))
  await expect(page.getByTestId('s40-next')).toBeDisabled()
  await expect(page.getByTestId('s40-abort')).toBeEnabled()
  const after = await posOf(page, 'foot')
  expect(after).toEqual(before)
})

test('S-40: 途中（S2）で別の項目を選ぶと ui.abort を先に送り、受理後に ui.calib_item を送る', async ({ page }) => {
  await stubTrigger(page, { 'ui.abort': { accepted: true }, 'ui.calib_item': { accepted: true } })
  await page.addInitScript(() => {
    window.__thTestState = { mode: 'CALIB', state: 'S2' }
    window.__thTestScreen = 'S40'
    window.__thTestCalibStatus = {
      item: 'LINEAR', step: 'S2', result: 'RUNNING', preview_before: '', preview_after: '',
      detail: { commanded: 1.0, progress: 0.4, history: {}, last_calibrated: {} },
    }
  })
  await page.goto('/')
  await page.locator('#body').waitFor()

  await page.getByTestId('s40-item-ROTATION').click()
  await page.waitForFunction(
    () => (window.__thTriggerCalls ?? []).some((c) => c.trigger === 'ui.calib_item'),
    null,
    { timeout: 5000 },
  )
  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  const abortIdx = calls.findIndex((c) => c.trigger === 'ui.abort')
  const itemIdx = calls.findIndex((c) => c.trigger === 'ui.calib_item' && c.argJson?.item === 'ROTATION')
  expect(abortIdx).toBeGreaterThanOrEqual(0)
  expect(itemIdx).toBeGreaterThan(abortIdx)
})

test('S-40: 途中（S2）で「履歴」を開くと ui.abort を先に送る', async ({ page }) => {
  await stubTrigger(page, { 'ui.abort': { accepted: true } })
  await page.addInitScript(() => {
    window.__thTestState = { mode: 'CALIB', state: 'S2' }
    window.__thTestScreen = 'S40'
    window.__thTestCalibStatus = {
      item: 'LINEAR', step: 'S2', result: 'RUNNING', preview_before: '', preview_after: '',
      detail: { commanded: 1.0, progress: 0.4, history: {}, last_calibrated: {} },
    }
  })
  await page.goto('/')
  await page.locator('#body').waitFor()
  await page.getByTestId('s40-history').click()
  await page.waitForFunction(
    () => (window.__thTriggerCalls ?? []).some((c) => c.trigger === 'ui.abort'),
    null,
    { timeout: 5000 },
  )
  await expect(page.getByTestId('s40-history-view')).toBeVisible()
})

test('S-31: 症状のチェックを押すと ✓ が付き、記録はしない（ui.stop は「閉じる」だけで送る）', async ({ page }) => {
  await stubTrigger(page, { 'ui.stop': { accepted: true } })
  await page.addInitScript(() => {
    window.__thTestState = { mode: 'OPCHECK', state: 'REPAIR' }
    window.__thTestScreen = 'S31'
    window.__thTestOpcheckStatus = { item: 'MOTOR', result: 'NG', detail: 'sign_mismatch_L', next_screen: 'repair' }
  })
  await page.goto('/')
  await page.locator('#body').waitFor()
  const first = page.getByTestId('s31-chk-MOTOR-0')
  await expect(first).toBeVisible()
  await first.click()
  await expect(first).toHaveAttribute('aria-pressed', 'true')
  // チェックの控えは画面の中だけ。機体へは送らない。
  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(calls.some((c) => c.trigger !== 'ui.screen')).toBeFalsy()
  await page.getByTestId('s31-back').click()
  const calls2 = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(calls2.some((c) => c.trigger === 'ui.stop')).toBeTruthy()
})
