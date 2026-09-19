// e2e/w6-keyboard-jog.spec.js — brief-onsite-ux2 F-5: S-20/S-21 の W-6（浮動の
// 手動操作パネル）でも WASD / 矢印キーが効くこと。
// 手本は e2e/w6-stick-responds.spec.js（W-6 のスティックが実際に配線されて
// いることの保険）と e2e/keyboard-jog.spec.js（常設走行タブの keyboard 保険）。
// ここは両者を合わせ、S-20 の実際の導線（操作カードの「手動」→ jogPanel.open()）
// で W-6 を開き、キーボードが本当に /cmd_vel_manual_raw まで届くことを見る。
import { test, expect } from '@playwright/test'
import { gotoScreenWithOnsite, jogPublishes } from './helpers.js'

const PREP = { mode: 'PREP', state: 'MAPPING' }   // attributes.json: PREP.jog === 'allowed'

async function settle(page, ms) {
  await page.waitForTimeout(ms)
}

test('W-6 を開いて w キーを押すと /cmd_vel_manual_raw に前進が出る', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, { pins: [], targets: [] })
  await page.locator('#s20').waitFor()

  await page.locator('#s20 .op-manual').click()
  await page.locator('#jogWin.show').waitFor()
  await page.locator('#jogWin .stick svg').waitFor()

  await page.keyboard.down('w')
  await settle(page, 250)
  await page.keyboard.up('w')

  const cmds = await jogPublishes(page, '/cmd_vel_manual_raw')
  expect(cmds.length, 'W-6 を開いた状態で w を押しても /cmd_vel_manual_raw が出ない').toBeGreaterThan(1)
  const moving = cmds.filter((c) => c.cmd.vx > 0 && c.cmd.wz === 0)
  expect(moving.length, '前進の指令が含まれていない').toBeGreaterThan(0)
})

test('W-6 が閉じている間は w キーを押しても何も出ない', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, { pins: [], targets: [] })
  await page.locator('#s20').waitFor()
  // W-6 を一度も開かない（#jogWin は常時マウントだが display:none）。
  await expect(page.locator('#jogWin.show')).toHaveCount(0)

  await page.keyboard.down('w')
  await settle(page, 250)
  await page.keyboard.up('w')

  const cmds = await jogPublishes(page, '/cmd_vel_manual_raw')
  expect(cmds.length, 'W-6 が閉じているのに /cmd_vel_manual_raw が出た').toBe(0)
})

test('W-6 を閉じると w キーがまた無反応に戻る', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, { pins: [], targets: [] })
  await page.locator('#s20').waitFor()

  await page.locator('#s20 .op-manual').click()
  await page.locator('#jogWin.show').waitFor()
  await page.locator('#jogWin .jw-hd .btn').click()   // 「閉じる」
  await page.locator('#jogWin').waitFor({ state: 'hidden' })

  await page.keyboard.down('w')
  await settle(page, 250)
  await page.keyboard.up('w')

  const cmds = await jogPublishes(page, '/cmd_vel_manual_raw')
  expect(cmds.length, 'W-6 を閉じたのに /cmd_vel_manual_raw が出た').toBe(0)
})
