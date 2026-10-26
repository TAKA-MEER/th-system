// Releasing sends zeros during the braking tail, then stops both timers.
import { test, expect } from '@playwright/test'
import { gotoScreen, downOnStick, jogPublishes } from './helpers.js'

test('release sends zeros through the braking tail and then stops publishing', async ({ page }) => {
  await gotoScreen(page, 'DRIVE_S11', { mode: 'IDLE' })
  // `#body` に限定するのは、W-6（`#jogWin`）にも同じ大きさのスティックが
  // 常時 DOM 上にあるため（webui.md §6.3「常設するものと同じ大きさ」）。
  // `.stick svg` だけだと strict mode violation で 2 件にマッチする。
  const stick = page.locator('#body .stick svg')
  await stick.waitFor()

  await downOnStick(page, stick, { offsetX: 0.45, offsetY: 0 })
  await page.waitForTimeout(200) // a few non-zero ticks
  await page.mouse.up()
  const releasedAt = await page.evaluate(() => performance.now())
  const commandsAtRelease = await jogPublishes(page, '/cmd_vel_manual_raw')
  const leasesAtRelease = await jogPublishes(page, '/ui/jog_lease')

  // S-curve stopping takes about 2.2s from 0.55m/s; the tail remains active
  // beyond that and continues publishing zero plus the lease.
  // Leave room for the 200 ms lease interval phase to produce a tick after
  // the 2.2 s stop estimate while remaining inside the 2.6 s tail.
  await page.waitForTimeout(2450)
  const commandsAt1650 = await jogPublishes(page, '/cmd_vel_manual_raw')
  const leasesAt1650 = await jogPublishes(page, '/ui/jog_lease')
  expect(commandsAt1650.length, 'stick tail must publish past 2.2 seconds')
    .toBeGreaterThan(commandsAtRelease.length)
  expect(leasesAt1650.length, 'stick lease must refresh past 2.2 seconds')
    .toBeGreaterThan(leasesAtRelease.length)
  const commandsAfter2200ms = commandsAt1650.filter((entry) => entry.time >= releasedAt + 2200)
  const leasesAfter2200ms = leasesAt1650.filter((entry) => entry.time >= releasedAt + 2200)
  expect(commandsAfter2200ms.length, 'stick commands must continue after 2.2 seconds').toBeGreaterThan(0)
  expect(leasesAfter2200ms.length, 'stick leases must continue after 2.2 seconds').toBeGreaterThan(0)
  expect(commandsAfter2200ms.every(
    (entry) => Math.abs(entry.cmd.vx) < 1e-9 && Math.abs(entry.cmd.wz) < 1e-9,
  ), 'stick tail must publish zero commands after braking').toBe(true)

  await page.waitForTimeout(550)
  const commandsAt3000 = await jogPublishes(page, '/cmd_vel_manual_raw')
  const leasesAt3000 = await jogPublishes(page, '/ui/jog_lease')
  expect(commandsAt3000.filter((entry) => entry.time >= releasedAt + 3000).length,
    'stick command timer must stop by 3.0 seconds').toBe(0)
  expect(leasesAt3000.filter((entry) => entry.time >= releasedAt + 3000).length,
    'stick lease timer must stop by 3.0 seconds').toBe(0)
})

test('touching again during the tail resumes the same command rate', async ({ page }) => {
  await gotoScreen(page, 'DRIVE_S11', { mode: 'IDLE' })
  const stick = page.locator('#body .stick svg')
  await stick.waitFor()
  await downOnStick(page, stick, { offsetX: 0.45, offsetY: 0 })
  await page.waitForTimeout(250)
  await page.mouse.up()
  await page.waitForTimeout(250)
  const countBeforeRetouch = (await jogPublishes(page, '/cmd_vel_manual_raw')).length

  await downOnStick(page, stick, { offsetX: 0.45, offsetY: 0 })
  await page.waitForTimeout(450)
  const commands = await jogPublishes(page, '/cmd_vel_manual_raw')
  const newCommands = commands.slice(countBeforeRetouch)
  expect(newCommands.some((entry) => entry.cmd.vx !== 0 || entry.cmd.wz !== 0)).toBe(true)
  // 10 Hz plus the immediate re-touch publish; a duplicate interval would
  // approximately double this count.
  expect(newCommands.length).toBeLessThanOrEqual(7)
  await page.mouse.up()
})

test('disabling the drive stops immediately without starting a release tail', async ({ page }) => {
  await gotoScreen(page, 'DRIVE_S11', { mode: 'IDLE' })
  const stick = page.locator('#body .stick svg')
  await stick.waitFor()
  await downOnStick(page, stick, { offsetX: 0.45, offsetY: 0 })
  await page.waitForTimeout(250)
  await page.evaluate(() => window.__thSetTestStale(true))
  await page.waitForTimeout(100)
  const afterDisable = await jogPublishes(page, '/cmd_vel_manual_raw')
  const count = afterDisable.length
  expect(afterDisable.at(-1).cmd.vx).toBe(0)
  expect(afterDisable.at(-1).cmd.wz).toBe(0)
  await page.waitForTimeout(250)
  expect((await jogPublishes(page, '/cmd_vel_manual_raw')).length).toBe(count)
})

test('hiding the page stops immediately without a release tail', async ({ page }) => {
  await gotoScreen(page, 'DRIVE_S11', { mode: 'IDLE' })
  const stick = page.locator('#body .stick svg')
  await stick.waitFor()
  await downOnStick(page, stick, { offsetX: 0.45, offsetY: 0 })
  await page.waitForTimeout(250)
  await page.evaluate(() => {
    Object.defineProperty(document, 'visibilityState', { value: 'hidden', configurable: true })
    document.dispatchEvent(new Event('visibilitychange'))
  })
  await page.waitForTimeout(100)
  const stopped = await jogPublishes(page, '/cmd_vel_manual_raw')
  const count = stopped.length
  expect(stopped.at(-1).cmd.vx).toBe(0)
  expect(stopped.at(-1).cmd.wz).toBe(0)
  await page.waitForTimeout(250)
  expect((await jogPublishes(page, '/cmd_vel_manual_raw')).length).toBe(count)
})
