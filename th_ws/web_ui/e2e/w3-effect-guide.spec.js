// 1b-6 (SG-B2): /system/effect の guide が W-3 の帯に出る。
// 帯の表示は本番の購読 hook (useSystemEffect -> useGuideBanner -> GuideBanner)
// を通す (部品に直接 props を渡さない)。ソケットは張らず TEST_MODE の種で
// 駆動する (e2e/helpers.js gotoScreenWithEffect)。
// 変異: 購読のトピック名を変える／振り分けで guide を捨てる／自動クローズを
// 外すと赤くなること。
import { test, expect } from '@playwright/test'
import { gotoScreenWithEffect, setTestEffect, setTestState } from './helpers.js'

function guide(key, dest = 'WebUI') {
  return { name: 'guide', dest, args_json: JSON.stringify({ key }) }
}

test('guide(summon_clear_timeout) が W-3 の帯に文言で出る', async ({ page }) => {
  await gotoScreenWithEffect(page, 'S01', { mode: 'SUMMON', state: 'WAIT_CLEAR' }, guide('summon_clear_timeout'))
  await expect(page.getByTestId('w3-guide')).toContainText('退避待ちが時間切れ')
})

test('閉じるボタンで消える', async ({ page }) => {
  await gotoScreenWithEffect(page, 'S01', { mode: 'SUMMON', state: 'WAIT_CLEAR' }, guide('summon_clear_timeout'))
  await expect(page.getByTestId('w3-guide')).toBeVisible()
  await page.getByTestId('w3-guide-close').click()
  await expect(page.getByTestId('w3-guide')).toHaveCount(0)
})

test('モードが変わると消える', async ({ page }) => {
  await gotoScreenWithEffect(page, 'S01', { mode: 'SUMMON', state: 'WAIT_CLEAR' }, guide('summon_clear_timeout'))
  await expect(page.getByTestId('w3-guide')).toBeVisible()
  await setTestState(page, { mode: 'IDLE', state: 'NONE' })
  await expect(page.getByTestId('w3-guide')).toHaveCount(0)
})

test('dest が WebUI でない effect では帯が出ない', async ({ page }) => {
  await gotoScreenWithEffect(
    page, 'S01', { mode: 'IDLE', state: 'NONE' },
    { name: 'restart_control_stack', dest: 'connectivity_checker', args_json: '{}' },
  )
  await expect(page.getByTestId('w3-guide')).toHaveCount(0)
})

test('estop_held_at_boot は物理非常停止が離されると消える', async ({ page }) => {
  await gotoScreenWithEffect(
    page, 'S01', { mode: 'INIT', state: 'CHECK', estop_hw: true }, guide('estop_held_at_boot'))
  await expect(page.getByTestId('w3-guide')).toContainText('非常停止ボタンが押されたまま')
  await setTestState(page, { estop_hw: false })
  await expect(page.getByTestId('w3-guide')).toHaveCount(0)
})

test('後から届いた guide で帯が掛け替わる', async ({ page }) => {
  await gotoScreenWithEffect(page, 'S01', { mode: 'SUMMON', state: 'WAIT_CLEAR' }, guide('summon_clear_timeout'))
  await expect(page.getByTestId('w3-guide')).toContainText('退避待ちが時間切れ')
  await setTestEffect(page, guide('summon_target_lost'))
  await expect(page.getByTestId('w3-guide')).toContainText('呼び寄せ中に対象を見失い')
})
