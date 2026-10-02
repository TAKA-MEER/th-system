// WS-9X: S-01「保守・設定」→「設定」で S-50 が開き、タブが切り替わり、
// 「戻る」で S-01 に戻ること。S-50 は FSM のモードではなく S-01 のサブ画面
// （main.jsx の settingsOpen / screenRouting.resolveScreen）。
//
// config_manager の service 呼び出しは TEST_MODE（window.__thTestState 定義時）
// では useTunableParams.js が即 reject するので、ネット無しで回る。
import { test, expect } from '@playwright/test'
import { gotoScreen, setTestState, stubTrigger } from './helpers.js'

async function openS50General(page) {
  await gotoScreen(page, 'S01', { mode: 'IDLE', tracker_enabled: true })
  await page.getByTestId('s01-open-settings').click()
  await expect(page.locator('#s50')).toBeVisible()
}

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

// 2026-10-02 Spec-webui.md §3.15: S-50 では死角マスクを編集しない。
// 読み取り専用の表示と S-40 への導線だけを置く。
test('S-50 一般タブに死角の編集欄は無く、表示と「校正で変更する」がある', async ({ page }) => {
  await openS50General(page)

  // 編集欄も保存ボタンも無い（誤って戻すとここが赤になる）。
  await expect(page.getByTestId('s50-save-lidar_filter')).toHaveCount(0)
  // 表示はある（TEST_MODE では取得できないので「取得できませんでした」が出る）。
  await expect(page.getByTestId('s50-blind-current')).toBeVisible()
  await expect(page.getByTestId('s50-goto-calib')).toBeVisible()
  await expect(page.getByTestId('s50-goto-calib')).toHaveText('校正で変更する')
})

test('「校正で変更する」は ui.enter_mode{CALIB} を送り、拒否されたら理由を出す', async ({ page }) => {
  await openS50General(page)

  // スタブ無し = 拒否扱い（useTrigger の TEST_MODE 既定）。
  await page.getByTestId('s50-goto-calib').click()
  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(calls).toHaveLength(1)
  expect(calls[0]).toEqual({
    trigger: 'ui.enter_mode', argJson: { mode: 'CALIB' }, requester: 'web_ui',
  })
  await expect(page.getByTestId('s50-calib-err')).toBeVisible()
})

test('「校正で変更する」が受理されたら理由は出ない', async ({ page }) => {
  await stubTrigger(page, { 'ui.enter_mode': { accepted: true } })
  await openS50General(page)

  await page.getByTestId('s50-goto-calib').click()
  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(calls).toHaveLength(1)
  expect(calls[0].argJson).toEqual({ mode: 'CALIB' })
  await expect(page.getByTestId('s50-calib-err')).toHaveCount(0)
})
