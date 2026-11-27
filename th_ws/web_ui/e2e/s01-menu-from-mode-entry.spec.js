// DetailedDesign-wp1.md WP-UI-02 §5 / §4.3 (M-1): S-01's mode-selection
// buttons come from mode_entry.json (screens/mainMenuItems.js), and none of
// them are pressable while mode is 'INIT'. A denied-but-visible button
// still opens a reason window when pressed (U-11: "拒否された理由 | UI
// (ウィンドウ。押されて初めて出す)"), it just isn't a real rejection round
// trip in this spec since there's no rosbridge backend -- the *disabled*
// case (mode_entry_denied, computed locally) is what's driven through the
// UI here; the "server said no after all" path is covered by menuItems()
// unit tests + code review, not by clicking through a live service.
//
// Spec-webui §3.2 (2026-11-27 作り直し): ライン誘導・電子リードの入口は消えた。
import { test, expect } from '@playwright/test'
import { gotoScreen, gotoScreenWithRouteCatalog, setTestState } from './helpers.js'

test('M-1: every menu tile is disabled while mode is INIT', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'INIT' })
  const buttons = page.locator('.tile')
  // 一番上の 5 タイル（試験場▸・走行▸・点検と校正▸・自律測定・設定）。
  await expect(buttons).toHaveCount(5)
  for (let i = 0; i < 4; i++) {
    await expect(buttons.nth(i)).toHaveClass(/dis/)
  }
  // 設定タイルだけは INIT でも押せる（FSM を動かさない。S-00 の開発モードと同じ）。
  await expect(page.getByTestId('s01-open-settings')).not.toHaveClass(/dis/)
  await expect(page.getByTestId('s01-open-settings')).not.toBeDisabled()
})

test('mode IDLE: every mode tile is enabled (mode_entry.yaml allows all 9 from IDLE)', async ({ page }) => {
  // 1b-11 SG-B13: 教示再生は経路が 1 本以上が前提。
  await gotoScreenWithRouteCatalog(page, 'S01', { mode: 'IDLE', tracker_enabled: true },
    [{ id: 'r1', name: '経路1', length_m: 5, point_count: 10 }])
  // ライン誘導・電子リードは無い（D7）。
  await expect(page.getByTestId('s01-group-field')).toHaveCount(1)
  await page.getByTestId('s01-group-drive').click()
  const tiles = page.locator('.tile')
  for (let i = 0; i < await tiles.count(); i++) {
    await expect(tiles.nth(i)).not.toHaveClass(/\bdis\b/)
  }
  await expect(page.locator('[data-testid="s01-mode-LINE"]')).toHaveCount(0)
  await expect(page.locator('[data-testid="s01-mode-LEASH"]')).toHaveCount(0)
  await page.getByTestId('s01-back').click()
  await expect(page.locator('[data-testid="s01-mode-FOLLOW"]')).toHaveCount(0)
  await expect(page.getByTestId('s01-group-drive')).toHaveCount(1)
})

// 2026-09-02: mode MANUAL では S-11 が描画されるため、検証は専用画面が無い
// FOLLOW（mode_entry.json では IDLE にしか行けない）で同じ形を突く。
// WP-UI-08: OPCHECK は S-30、WP-MAINT-03: CALIB は S-40 の専用画面。
test('mode FOLLOW: 入れないタイルは dis 表示で、押すと mode_entry_denied', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'FOLLOW', tracker_enabled: true })

  // 点検と校正の階層へ。
  await page.getByTestId('s01-group-maint').click()
  const opcheck = page.getByTestId('s01-mode-OPCHECK')
  await expect(opcheck).toHaveClass(/dis/)

  await opcheck.click()
  await expect(page.getByTestId('s01-reason-window')).toBeVisible()
  await expect(page.getByTestId('s01-reason-window')).toContainText('今のモードからは移動できません')
})

test('tracker_disabled: FOLLOW shows its own reason text, distinct from mode_entry_denied', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'IDLE', tracker_enabled: true })
  await setTestState(page, { tracker_enabled: false })

  await page.getByTestId('s01-group-drive').click()
  const follow = page.getByTestId('s01-mode-FOLLOW')
  await expect(follow).toHaveClass(/dis/)
  await follow.click()
  await expect(page.getByTestId('s01-reason-window')).toContainText('人物追跡がOFFです')

  // Closing returns to a clean state (no lingering modal / backdrop).
  await page.getByTestId('s01-reason-window').getByRole('button', { name: '確認' }).click()
  await expect(page.getByTestId('s01-reason-window')).toHaveCount(0)
})
