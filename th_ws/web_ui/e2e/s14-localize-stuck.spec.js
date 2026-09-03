// WS-9K-B 検証（2026-09-03 実機）: 別セッションの経路を選ぶと、replay_runner の
// ガードで経路は弾かれるが、FSM は LOCALIZE のまま READY に進まず、画面には
// 「初期姿勢を推定しています…」が出たままになる。操作者は弾かれたことに気づけない。
//
// LOCALIZE が一定時間（20 秒）続いたら、画面に手がかり文言（別セッション経路の
// 可能性）を出す。正常時は LOCALIZE はほぼ一瞬（W-01）なので、ここまで留まるのは
// 弾かれているとみなす。
//
// e2e は実時間 20 秒を待たない。page.clock（fake timers）で setTimeout を進める。
import { test, expect } from '@playwright/test'

async function openLocalize(page) {
  await page.addInitScript((s) => {
    window.__thTestState = s
    window.__thTestScreen = 'S14'
  }, { mode: 'REPLAY', state: 'LOCALIZE' })
  await page.goto('/')
  await page.locator('#s14').waitFor()
}

test('LOCALIZE が閾値（20 秒）を超えたら手がかり文言が出る', async ({ page }) => {
  await page.clock.install()
  await openLocalize(page)

  // 閾値未満（19 秒）では出ない。
  await page.clock.fastForward(19000)
  await expect(page.getByTestId('s14-localize-stuck')).toHaveCount(0)

  // 閾値超過（22 秒）で出る。
  await page.clock.fastForward(3000)
  await expect(page.getByTestId('s14-localize-stuck')).toBeVisible()
  await expect(page.getByTestId('s14-localize-stuck')).toContainText('別の起動セッション')
})

test('LOCALIZE が閾値未満なら手がかり文言が出ない', async ({ page }) => {
  await page.clock.install()
  await openLocalize(page)

  await page.clock.fastForward(19000)
  await expect(page.getByTestId('s14-localize-stuck')).toHaveCount(0)
})

test('LOCALIZE から抜けたら手がかり文言は消える', async ({ page }) => {
  await page.clock.install()
  await openLocalize(page)

  await page.clock.fastForward(21000)
  await expect(page.getByTestId('s14-localize-stuck')).toBeVisible()

  // READY へ遷移（= 推定成功）すれば消える。
  await page.evaluate(() => window.__thSetTestState({ state: 'READY' }))
  await expect(page.getByTestId('s14-localize-stuck')).toHaveCount(0)
})
