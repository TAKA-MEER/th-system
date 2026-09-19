// DetailedDesign-wp3.md WP-UI-03 §9: releasing the stick must send zero
// exactly once, then stop -- never a repeated or a persistently-held last
// command. The press is off-centre so the held window is all non-zero and the
// single zero stands out unambiguously.
import { test, expect } from '@playwright/test'
import { gotoScreen, downOnStick, jogPublishes } from './helpers.js'

test('release sends zero exactly once and stops publishing', async ({ page }) => {
  await gotoScreen(page, 'DRIVE_S11', { mode: 'IDLE' })
  // `#body` に限定するのは、W-6（`#jogWin`）にも同じ大きさのスティックが
  // 常時 DOM 上にあるため（webui.md §6.3「常設するものと同じ大きさ」）。
  // `.stick svg` だけだと strict mode violation で 2 件にマッチする。
  const stick = page.locator('#body .stick svg')
  await stick.waitFor()

  await downOnStick(page, stick, { offsetX: 0.45, offsetY: 0 })
  await page.waitForTimeout(200) // a few non-zero ticks
  await page.mouse.up()
  await page.waitForTimeout(250) // let any spurious tick land

  const cmds = await jogPublishes(page, '/cmd_vel_manual_raw')
  expect(cmds.length).toBeGreaterThan(1)

  const zeros = cmds.filter((c) => c.cmd.vx === 0 && c.cmd.wz === 0)
  expect(zeros.length, 'zero must be sent exactly once on release').toBe(1)

  const last = cmds[cmds.length - 1]
  expect(last.cmd.vx).toBe(0)
  expect(last.cmd.wz).toBe(0)

  // And nothing else arrives afterwards (the 10 Hz timer is stopped).
  await page.waitForTimeout(250)
  const settled = await jogPublishes(page, '/cmd_vel_manual_raw')
  expect(settled.length).toBe(cmds.length)
})
