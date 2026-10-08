// 1b-7 S-13（SG-C5・SG-B11）: 教示タブの表示が機体の状態から決まること。
//
// - 再読み込みしても記録中の表示が戻る（状態から出す。画面ローカルではない）
// - 「記録中」は赤いインジケータ（Spec-webui.md §3.6）
// - 開始時の向きが出る（RouteStatus.start_yaw。rad→度で表示）
// - PAUSE では記録中が出ず一時停止が出る
// - SAVED で拒否された理由 teach_saved_finalized が出る
import { test, expect } from '@playwright/test'
import { gotoScreen, gotoScreenWithRoutePreview } from './helpers.js'

test('S-13 再読み込みしても記録中の表示が戻る', async ({ page }) => {
  await gotoScreen(page, 'S13', { mode: 'TEACH_MANUAL', state: 'REC' })
  await page.locator('#s13').waitFor()
  await expect(page.locator('[data-testid="s13-recording"]')).toBeVisible()

  await page.reload()
  await page.locator('#s13').waitFor()
  await expect(page.locator('[data-testid="s13-recording"]'),
    '再読み込みで記録中の表示に戻っていない').toBeVisible()
})

test('S-13 「記録中」は赤いインジケータ', async ({ page }) => {
  await gotoScreen(page, 'S13', { mode: 'TEACH_MANUAL', state: 'REC' })
  await page.locator('#s13').waitFor()
  const rec = page.locator('[data-testid="s13-recording"]')
  await expect(rec).toBeVisible()
  const cls = await rec.getAttribute('class')
  expect(cls, '記録中に赤系のクラスが無い').toMatch(/tone-ng/)
})

test('S-13 開始時の向きが出る（rad を度で表示）', async ({ page }) => {
  await gotoScreenWithRoutePreview(
    page, 'S13',
    { mode: 'TEACH_MANUAL', state: 'REC' },
    {
      routes: [], preview: [],
      status: {
        state: 'REC', saved: false, recorded_m: 1.5, points: 10,
        elapsed_sec: 5, start_yaw: Math.PI / 2,
      },
    },
  )
  await page.locator('#s13').waitFor()
  // REC なら教示タブが初期表示なのでクリック不要。
  await expect(page.getByTestId('s13-start-yaw')).toHaveText('90°')
})

test('S-13 points=0 では開始時の向きを出さない（未記録の 0 を出さない）', async ({ page }) => {
  await gotoScreenWithRoutePreview(
    page, 'S13',
    { mode: 'TEACH_MANUAL', state: 'REC' },
    {
      routes: [], preview: [],
      status: { state: 'REC', saved: false, recorded_m: 0, points: 0, elapsed_sec: 0, start_yaw: 0 },
    },
  )
  await page.locator('#s13').waitFor()
  await expect(page.getByTestId('s13-start-yaw')).toHaveCount(0)
})

test('S-13 PAUSE では記録中が出ず一時停止が出る', async ({ page }) => {
  await gotoScreenWithRoutePreview(
    page, 'S13',
    { mode: 'TEACH_MANUAL', state: 'PAUSE' },
    {
      routes: [], preview: [],
      status: {
        state: 'PAUSE', saved: false, recorded_m: 1.5, points: 10,
        elapsed_sec: 5, start_yaw: 0.5,
      },
    },
  )
  await page.locator('#s13').waitFor()
  await expect(page.locator('[data-testid="s13-recording"]')).toHaveCount(0)
  await expect(page.getByTestId('s13-paused')).toBeVisible()
})

test('S-13 SAVED で拒否された理由 teach_saved_finalized が出る', async ({ page }) => {
  await gotoScreenWithRoutePreview(
    page, 'S13',
    {
      mode: 'TEACH_MANUAL', state: 'SAVED',
      last_reject_reason: 'teach_saved_finalized',
    },
    { routes: [], preview: [], status: { state: 'SAVED', saved: true } },
  )
  await page.locator('#s13').waitFor()
  await expect(page.getByTestId('s13-reject-reason')).toContainText('保存済みのため記録を続けられません')
})

test('S-13 拒否が無ければ理由を出さない', async ({ page }) => {
  await gotoScreenWithRoutePreview(
    page, 'S13',
    { mode: 'TEACH_MANUAL', state: 'SAVED' },
    { routes: [], preview: [], status: { state: 'SAVED', saved: true } },
  )
  await page.locator('#s13').waitFor()
  await expect(page.getByTestId('s13-reject-reason')).toHaveCount(0)
})
