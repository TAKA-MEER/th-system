// WS-9X: S-01「保守・設定」→「設定」で S-50 が開き、タブが切り替わり、
// 「戻る」で S-01 に戻ること。S-50 は FSM のモードではなく S-01 のサブ画面
// （main.jsx の settingsOpen / screenRouting.resolveScreen）。
//
// config_manager の service 呼び出しは TEST_MODE（window.__thTestState 定義時）
// では useTunableParams.js が即 reject するので、ネット無しで回る。
import { test, expect } from '@playwright/test'
import { gotoScreen, setTestState } from './helpers.js'

test('S-01 →「設定」→ S-50、タブ切替、戻る', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'IDLE', tracker_enabled: true })

  await page.getByTestId('s01-open-settings').click()

  const s50 = page.locator('#s50')
  await expect(s50).toBeVisible()
  await expect(page.locator('#screenName')).toHaveText('設定')

  // 既定は一般タブ。3 セクションの保存ボタンが見える。
  await expect(page.getByTestId('s50-save-slam_toolbox')).toBeVisible()

  // 表示タブへ
  await page.getByTestId('s50-tab-display').click()
  await expect(page.getByTestId('s50-font-large')).toBeVisible()

  // 開発モードタブへ
  await page.getByTestId('s50-tab-dev').click()
  await expect(page.getByTestId('s50-dev-toggle')).toBeVisible()

  // 戻る → S-01
  await page.getByTestId('s50-back').click()
  await expect(page.locator('#s50')).toHaveCount(0)
  await expect(page.locator('#s01')).toBeVisible()
})

test('S-50 表示中にモードが IDLE を離れたら設定は閉じて画面が追随する', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'IDLE', tracker_enabled: true })
  await page.getByTestId('s01-open-settings').click()
  await expect(page.locator('#s50')).toBeVisible()

  await setTestState(page, { mode: 'REPLAY' })

  await expect(page.locator('#s50')).toHaveCount(0)
  await expect(page.locator('#s14')).toBeVisible()
})

test('「設定」は他のメニューが押せない状況でも押せる（disabledAll に縛られない）', async ({ page }) => {
  // mode=INIT ではモード選択ボタンが全部 disabled（menuItems の M-1）だが、
  // 設定は FSM を動かさないので読めるようにしてある。
  await gotoScreen(page, 'S01', { mode: 'INIT' })
  const settings = page.getByTestId('s01-open-settings')
  await expect(settings).toBeEnabled()
  await settings.click()
  await expect(page.locator('#s50')).toBeVisible()
})
