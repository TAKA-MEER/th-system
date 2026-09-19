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
//
// 2026-09-02: 画面は SystemState.mode から導出されるようになったので、
// 受理の stub だけでは動かない。th_state がモードを publish するところまで
// 再現する（詳細は s11-reachable-from-menu.spec.js の冒頭コメント）。
import { test, expect } from '@playwright/test'
import { gotoScreen, stubTrigger, setTestState } from './helpers.js'

test('S-01 教示（手動）accepted by th_state -> S-13 renders', async ({ page }) => {
  await stubTrigger(page, { 'ui.enter_mode': { accepted: true, reject_reason_key: '' } })
  await gotoScreen(page, 'S01', { mode: 'IDLE', tracker_enabled: true })

  const teach = page.getByRole('button', { name: '教示（手動）' })
  await expect(teach).toBeEnabled()
  await teach.click()

  // 受理されただけでは動かない。FSM がモードを publish して初めて動く。
  await expect(page.locator('#s13')).toHaveCount(0)

  await setTestState(page, { mode: 'TEACH_MANUAL' })
  await expect(page.locator('#s13')).toBeVisible()
})

// ── 回帰テスト: 教示中にロボット側が IDLE へ落ちたときに閉じ込められない ──
// 実機で報告された「移動不能」の本体。教示画面のまま FSM だけ IDLE に戻ると、
// 教示系の操作も「終了」も全部通らず詰んでいた。
test('教示中にモードが IDLE へ戻ったら画面も S-01 へ追随する', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'TEACH_MANUAL', tracker_enabled: true })
  await expect(page.locator('#s13')).toBeVisible()

  await setTestState(page, { mode: 'IDLE' })

  await expect(page.locator('#s13')).toHaveCount(0)
  await expect(page.locator('#s01')).toBeVisible()
})
