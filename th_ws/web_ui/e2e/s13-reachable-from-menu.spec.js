// P5 / demo-teach-replay: S-01 の「教示（手動）」(mode TEACH_MANUAL) ボタン、
// th_state が ui.enter_mode を受理したら S-01 に留まらず S-13 へ遷移すること
// （S-11 が MANUAL でそうであるのと同じ。main.jsx の
// MODE_TO_SCREEN['TEACH_MANUAL'] = 'S13' を検証する）。
//
// e2e に th_state は無いので受理は window.__thTestTrigger['ui.enter_mode'] の
// stub で代用する（helpers.js stubTrigger / ros/useTrigger.js のテストフック）。
//
// 「教示（手動）」ボタンは menuItems()(mainMenuItems.js) が mode IDLE から
// 許可し、modeLabel('TEACH_MANUAL')(i18n/modes.js) = '教示（手動）' で表示される。
import { test, expect } from '@playwright/test'
import { gotoScreen, stubTrigger } from './helpers.js'

test('S-01 教示（手動）accepted by th_state -> S-13 renders', async ({ page }) => {
  await stubTrigger(page, { 'ui.enter_mode': { accepted: true, reject_reason_key: '' } })
  await gotoScreen(page, 'S01', { mode: 'IDLE', tracker_enabled: true })

  const teach = page.getByRole('button', { name: '教示（手動）' })
  await expect(teach).toBeEnabled()
  await teach.click()

  await expect(page.locator('#s13')).toBeVisible()
})
