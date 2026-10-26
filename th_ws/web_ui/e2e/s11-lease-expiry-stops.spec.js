// A released stick keeps /ui/jog_lease alive while the normal acceleration
// ramp brakes the vehicle; lease expiry then transitions RUN -> PAUSE.
import { test, expect } from '@playwright/test'
import { gotoScreen, downOnStick, jogPublishes } from './helpers.js'

test('releasing the stick keeps lease publication alive during braking', async ({ page }) => {
  await gotoScreen(page, 'S11', { mode: 'MANUAL', state: 'PAUSE' })
  const stick = page.locator('#body .stick svg')
  await stick.waitFor()

  await downOnStick(page, stick, { offsetX: 0.45, offsetY: 0 })
  await page.waitForTimeout(500)
  const before = (await jogPublishes(page, '/ui/jog_lease')).length
  expect(before).toBeGreaterThan(0)

  await page.mouse.up()

  // Lease expiry occurs after the drive has had time to ramp down.
  await page.waitForTimeout(250)
  const duringTail = (await jogPublishes(page, '/ui/jog_lease')).length
  expect(duringTail).toBeGreaterThan(before)

  const cmds = await jogPublishes(page, '/cmd_vel_manual_raw')
  const zeros = cmds.filter((c) => c.cmd && c.cmd.vx === 0 && c.cmd.wz === 0)
  expect(zeros.length).toBeGreaterThan(1)
  await page.waitForTimeout(1900)
  const settledLeaseCount = (await jogPublishes(page, '/ui/jog_lease')).length
  await page.waitForTimeout(250)
  expect((await jogPublishes(page, '/ui/jog_lease')).length).toBe(settledLeaseCount)
})
