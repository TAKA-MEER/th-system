// e2e/w6-keyboard-jog.spec.js — S-20 の常設スティックで WASD / 矢印キーが効くこと。
// 画面の作り直し（Spec-webui §3.10）: S-20 はスティックを常設する（「手動」ボタンで
// 開く W-6 は無い）。キーボードは常設の JogConsole（keyboard=true）に届く。
import { test, expect } from '@playwright/test'
import { gotoScreenWithOnsite, jogPublishes } from './helpers.js'

const PREP = { mode: 'PREP', state: 'MAPPING' }   // attributes.json: PREP.jog === 'allowed'

async function settle(page, ms) {
  await page.waitForTimeout(ms)
}

test('常設スティックがある状態で w キーを押すと /cmd_vel_manual_raw に前進が出る', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, { pins: [], targets: [], mappingActive: true })
  await page.locator('#s20').waitFor()

  // W-6 は無い（常設スティック）。スティックが最初から見えている。
  await expect(page.locator('#jogWin.show')).toHaveCount(0)
  await page.locator('#s20 .stick svg').waitFor()

  await page.keyboard.down('w')
  await settle(page, 250)
  await page.keyboard.up('w')

  const cmds = await jogPublishes(page, '/cmd_vel_manual_raw')
  expect(cmds.length, '常設スティックで w を押しても /cmd_vel_manual_raw が出ない').toBeGreaterThan(1)
  const moving = cmds.filter((c) => c.cmd.vx > 0 && c.cmd.wz === 0)
  expect(moving.length, '前進の指令が含まれていない').toBeGreaterThan(0)
})
