// 1b-7 SG-B1/SG-C5: S-13 の「記録中」は FSM の状態から出る。
// 画面ローカルのフラグだった頃は、一度記録を始めると PAUSE・SAVED・ESTOP でも
// 「記録中」が出たままになった。いまは教示系の REC のときだけ出る。
import { test, expect } from '@playwright/test'
import { gotoScreen, setTestState, gotoScreenWithRoutePreview } from './helpers.js'

async function openTeachTab(page) {
  await page.locator('#s13').waitFor()
  await page.getByRole('tab', { name: '教示' }).click()
}

test('S-13 REC のときだけ「記録中」が出る', async ({ page }) => {
  await gotoScreen(page, 'S13', { mode: 'TEACH_MANUAL', state: 'REC' })
  await openTeachTab(page)
  await expect(page.locator('[data-testid="s13-recording"]')).toBeVisible()
})

test('S-13 PAUSE では「記録中」が出ない', async ({ page }) => {
  await gotoScreen(page, 'S13', { mode: 'TEACH_MANUAL', state: 'PAUSE' })
  await openTeachTab(page)
  await expect(page.locator('[data-testid="s13-recording"]')).toHaveCount(0)
})

test('S-13 SAVED では「記録中」が出ず「保存しました」が出る', async ({ page }) => {
  await gotoScreenWithRoutePreview(
    page, 'S13',
    { mode: 'TEACH_MANUAL', state: 'SAVED' },
    { routes: [], preview: [], status: { state: 'SAVED', saved: true } },
  )
  await openTeachTab(page)
  await expect(page.locator('[data-testid="s13-recording"]')).toHaveCount(0)
  await expect(page.locator('[data-testid="s13-saved"]')).toBeVisible()
})

test('S-13 PAUSE を挟んで REC に戻ると再び「記録中」が出る', async ({ page }) => {
  await gotoScreen(page, 'S13', { mode: 'TEACH_MANUAL', state: 'REC' })
  await openTeachTab(page)
  await expect(page.locator('[data-testid="s13-recording"]')).toBeVisible()
  // 一時停止では消える。非常停止・手押しから戻った PAUSE も同じ表示。
  await setTestState(page, { mode: 'TEACH_MANUAL', state: 'PAUSE' })
  await expect(page.locator('[data-testid="s13-recording"]')).toHaveCount(0)
  // REC に入ったら再び出る（続きから記録していることと一致）。
  await setTestState(page, { mode: 'TEACH_MANUAL', state: 'REC' })
  await expect(page.locator('[data-testid="s13-recording"]')).toBeVisible()
})
