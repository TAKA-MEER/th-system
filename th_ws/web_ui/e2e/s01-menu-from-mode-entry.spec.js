// DetailedDesign-wp1.md WP-UI-02 §5 / §4.3 (M-1): S-01's mode-selection
// buttons come from mode_entry.json (screens/mainMenuItems.js), and none of
// them are pressable while mode is 'INIT'. A denied-but-visible button
// still opens a reason window when pressed (U-11: "拒否された理由 | UI
// (ウィンドウ。押されて初めて出す)"), it just isn't a real rejection round
// trip in this spec since there's no rosbridge backend -- the *disabled*
// case (mode_entry_denied, computed locally) is what's driven through the
// UI here; the "server said no after all" path is covered by menuItems()
// unit tests + code review, not by clicking through a live service.
import { test, expect } from '@playwright/test'
import { gotoScreen, setTestState } from './helpers.js'

test('M-1: every menu button is disabled while mode is INIT', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'INIT' })
  const buttons = page.locator('.btnrow .btn')
  const count = await buttons.count()
  expect(count).toBeGreaterThan(0)
  for (let i = 0; i < count; i++) {
    await expect(buttons.nth(i)).toBeDisabled()
  }
})

test('mode IDLE: every menu button is enabled (mode_entry.yaml allows all 10 from IDLE)', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'IDLE', tracker_enabled: true })
  const buttons = page.locator('.btnrow .btn')
  const count = await buttons.count()
  for (let i = 0; i < count; i++) {
    await expect(buttons.nth(i)).toBeEnabled()
    await expect(buttons.nth(i)).not.toHaveClass(/\bdis\b/)
  }
})

test('mode MANUAL: only OPCHECK/CALIB (保守) buttons are enabled, others show mode_entry_denied when pressed', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'MANUAL', tracker_enabled: true })

  const opcheck = page.getByRole('button', { name: '始業点検' })
  const calib = page.getByRole('button', { name: '校正' })
  await expect(opcheck).toBeEnabled()
  await expect(opcheck).not.toHaveClass(/\bdis\b/)
  await expect(calib).toBeEnabled()
  await expect(calib).not.toHaveClass(/\bdis\b/)

  const follow = page.getByRole('button', { name: '追従走行' })
  await expect(follow).toBeEnabled() // clickable so the reason can show, but styled as denied
  await expect(follow).toHaveClass(/\bdis\b/)

  await follow.click()
  await expect(page.locator('.win.confirm.show')).toBeVisible()
  await expect(page.locator('.win.confirm.show')).toContainText('今のモードからは移動できません')
})

test('tracker_disabled: FOLLOW shows its own reason text, distinct from mode_entry_denied', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'IDLE', tracker_enabled: true })
  await setTestState(page, { tracker_enabled: false })

  const follow = page.getByRole('button', { name: '追従走行' })
  await expect(follow).toHaveClass(/\bdis\b/)
  await follow.click()
  await expect(page.locator('.win.confirm.show')).toContainText('人物追跡がOFFです')

  // Closing returns to a clean state (no lingering modal / backdrop).
  await page.getByRole('button', { name: '確認' }).click()
  await expect(page.locator('.win.confirm.show')).toHaveCount(0)
})
