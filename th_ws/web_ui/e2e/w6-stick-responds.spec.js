// DetailedDesign-wp3.md WP-UI-03 §9 / FMEA ③ (U3-4): touching the stick
// *inside the W-6 panel* must start sending /cmd_vel_manual_raw. This guards
// against the W-6 stick being wired to nothing (the old follow nodes' dead
// /cmd_vel_retreat regression) -- the panel must actually drive the wire.
import { test, expect } from '@playwright/test'
import { gotoScreen, downOnStick, jogPublishes } from './helpers.js'

test('W-6 stick touch starts publishing /cmd_vel_manual_raw', async ({ page }) => {
  await gotoScreen(page, 'DRIVE_S11', { mode: 'IDLE' })
  await page.click('[data-testid="open-jog"]')
  await page.locator('#jogWin.show').waitFor()

  const stick = page.locator('#jogWin .stick svg')
  await stick.waitFor()
  await downOnStick(page, stick, { offsetX: 0.45, offsetY: 0 })
  await page.waitForTimeout(250)
  await page.mouse.up()

  const cmds = await jogPublishes(page, '/cmd_vel_manual_raw')
  expect(cmds.length).toBeGreaterThan(0)
  // And at least the held window delivered a non-zero command (not just the
  // release zero) -- otherwise the stick would be inert.
  const nonZero = cmds.filter((c) => c.cmd.vx !== 0 || c.cmd.wz !== 0)
  expect(nonZero.length).toBeGreaterThan(0)
})
