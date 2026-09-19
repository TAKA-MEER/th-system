// WS-9K-B（2026-09-03）→ WS-9U（2026-09-04）: LOCALIZE が長引くのは地図の
// 読み直し失敗（保存地図が無い経路、posegraph 欠落、始点から離れすぎ 等）。
// FSM は LOCALIZE のまま READY に進まず、画面には「初期姿勢を推定しています…」が
// 出たまま。操作者が気づけないので、しきい値を超えたら手がかり文言を出す。
//
// WS-9S 以降は slam_toolbox の respawn + deserialize に長距離地図で ~45s
// かかりうるので、しきい値は 60 秒に広げた（早すぎると reload 中に誤報する）。
//
// e2e は実時間を待たない。page.clock（fake timers）で setTimeout を進める。
import { test, expect } from '@playwright/test'

async function openLocalize(page) {
  await page.addInitScript((s) => {
    window.__thTestState = s
    window.__thTestScreen = 'S14'
  }, { mode: 'REPLAY', state: 'LOCALIZE' })
  await page.goto('/')
  await page.locator('#s14').waitFor()
}

test('LOCALIZE が閾値（60 秒）を超えたら手がかり文言が出る', async ({ page }) => {
  await page.clock.install()
  await openLocalize(page)

  // 閾値未満（59 秒）では出ない。
  await page.clock.fastForward(59000)
  await expect(page.getByTestId('s14-localize-stuck')).toHaveCount(0)

  // 閾値超過（62 秒）で出る。
  await page.clock.fastForward(3000)
  await expect(page.getByTestId('s14-localize-stuck')).toBeVisible()
  await expect(page.getByTestId('s14-localize-stuck')).toContainText('教示からやり直して')
})

test('LOCALIZE が閾値未満なら手がかり文言が出ない', async ({ page }) => {
  await page.clock.install()
  await openLocalize(page)

  await page.clock.fastForward(59000)
  await expect(page.getByTestId('s14-localize-stuck')).toHaveCount(0)
})

test('LOCALIZE から抜けたら手がかり文言は消える', async ({ page }) => {
  await page.clock.install()
  await openLocalize(page)

  await page.clock.fastForward(61000)
  await expect(page.getByTestId('s14-localize-stuck')).toBeVisible()

  // READY へ遷移（= 推定成功）すれば消える。
  await page.evaluate(() => window.__thSetTestState({ state: 'READY' }))
  await expect(page.getByTestId('s14-localize-stuck')).toHaveCount(0)
})
