// WS-4 / demo-teach-replay: keyboard manual drive (parts/useKeyboardJog.js
// merged into JogConsole) must actually reach the single publisher
// useJogLease -- a dead keydown handler would leave /cmd_vel_manual_raw silent
// (mutation 1). Runs on DRIVE_S11 (the drive tab with the stick console, via
// main.jsx's __thTestScreen); with no input focused, 'w' must publish a forward
// command and releasing it must send a single zero and then stop. With an
// <input> focused the same key must be ignored (never steal from text fields).
import { test, expect } from '@playwright/test'
import { gotoScreen, jogPublishes } from './helpers.js'

async function settle(page, ms) {
  await page.waitForTimeout(ms)
}

test('holding w publishes a forward command via the keyboard', async ({ page }) => {
  await gotoScreen(page, 'DRIVE_S11', { mode: 'MANUAL' })
  await page.locator('#body .stick svg').waitFor()

  await page.keyboard.down('w')
  await settle(page, 250) // a few 10 Hz ticks

  const cmds = await jogPublishes(page, '/cmd_vel_manual_raw')
  expect(cmds.length, 'keyboard hold should publish /cmd_vel_manual_raw').toBeGreaterThan(1)

  // forward: vx = vn(+1) * speed_preset_mid  > 0, wz = 0
  const moving = cmds.filter((c) => c.cmd.vx > 0 && c.cmd.wz === 0)
  expect(moving.length, 'some ticks should be pure forward').toBeGreaterThan(0)
})

test('releasing the key sends a single zero and stops publishing', async ({ page }) => {
  await gotoScreen(page, 'DRIVE_S11', { mode: 'MANUAL' })
  await page.locator('#body .stick svg').waitFor()

  await page.keyboard.down('w')
  await settle(page, 200)
  await page.keyboard.up('w')
  await settle(page, 250)

  const cmds = await jogPublishes(page, '/cmd_vel_manual_raw')
  const zeros = cmds.filter((c) => c.cmd.vx === 0 && c.cmd.wz === 0)
  expect(zeros.length, 'release must send zero exactly once').toBe(1)
  const last = cmds[cmds.length - 1]
  expect(last.cmd.vx).toBe(0)
  expect(last.cmd.wz).toBe(0)

  await settle(page, 250)
  const settled = await jogPublishes(page, '/cmd_vel_manual_raw')
  expect(settled.length, 'timers stopped after release').toBe(cmds.length)
})

test('does not steal keys while an input is focused', async ({ page }) => {
  await gotoScreen(page, 'DRIVE_S11', { mode: 'MANUAL' })
  await page.locator('#body .stick svg').waitFor()

  // Focus the speed slider (an <input>); typing keys must not drive.
  const slider = page.locator('#body .stickBox .slider input')
  await slider.focus()
  await page.keyboard.down('w')
  await settle(page, 250)
  await page.keyboard.up('w')
  await settle(page, 100)

  const cmds = await jogPublishes(page, '/cmd_vel_manual_raw')
  expect(cmds.length, 'no drive command while typing in an input').toBe(0)
})