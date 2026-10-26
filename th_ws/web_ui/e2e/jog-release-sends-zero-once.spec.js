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
  await page.waitForTimeout(250) // let any spurious tick land

  const cmds = await jogPublishes(page, '/cmd_vel_manual_raw')
  expect(cmds.length).toBeGreaterThan(1)

  const zeros = cmds.filter((c) => c.cmd.vx === 0 && c.cmd.wz === 0)
  expect(zeros.length, 'zero must be sent throughout the braking tail').toBeGreaterThan(1)
  expect((await jogPublishes(page, '/ui/jog_lease')).length).toBeGreaterThan(1)

  // And nothing else arrives afterwards (the 10 Hz timer is stopped).
  await page.waitForTimeout(1900)
  const settled = await jogPublishes(page, '/cmd_vel_manual_raw')
  await page.waitForTimeout(250)
  expect((await jogPublishes(page, '/cmd_vel_manual_raw')).length).toBe(settled.length)
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
