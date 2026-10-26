// WS-4 / demo-teach-replay: keyboard manual drive (parts/useKeyboardJog.js
// merged into JogConsole) must actually reach the single publisher
// useJogLease -- a dead keydown handler would leave /cmd_vel_manual_raw silent
// (mutation 1). Runs on DRIVE_S11 (the drive tab with the stick console, via
// main.jsx's __thTestScreen); with no input focused, 'w' must publish a forward
// command and releasing it must send a single zero and then stop. Only text
// entry steals keys: a focused speed slider (input[type=range]) must still
// drive, while a focused input[type=text] must be ignored.
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

test('releasing the key sends zeros and lease refreshes through the braking tail', async ({ page }) => {
  await gotoScreen(page, 'DRIVE_S11', { mode: 'MANUAL' })
  await page.locator('#body .stick svg').waitFor()

  await page.keyboard.down('w')
  await settle(page, 200)
  await page.keyboard.up('w')
  const releasedAt = await page.evaluate(() => performance.now())
  const commandsAtRelease = await jogPublishes(page, '/cmd_vel_manual_raw')
  const leasesAtRelease = await jogPublishes(page, '/ui/jog_lease')

  // Leave room for the 200 ms lease interval phase to produce a tick after
  // the 2.2 s stop estimate while remaining inside the 2.6 s tail.
  await settle(page, 2450)
  const commandsAt1650 = await jogPublishes(page, '/cmd_vel_manual_raw')
  const leasesAt1650 = await jogPublishes(page, '/ui/jog_lease')
  expect(commandsAt1650.length, 'keyboard tail must publish past 2.2 seconds')
    .toBeGreaterThan(commandsAtRelease.length)
  expect(leasesAt1650.length, 'keyboard lease must refresh past 2.2 seconds')
    .toBeGreaterThan(leasesAtRelease.length)
  const commandsAfter2200ms = commandsAt1650.filter((entry) => entry.time >= releasedAt + 2200)
  const leasesAfter2200ms = leasesAt1650.filter((entry) => entry.time >= releasedAt + 2200)
  expect(commandsAfter2200ms.length, 'keyboard commands must continue after 2.2 seconds').toBeGreaterThan(0)
  expect(leasesAfter2200ms.length, 'keyboard leases must continue after 2.2 seconds').toBeGreaterThan(0)
  expect(commandsAfter2200ms.every(
    (entry) => Math.abs(entry.cmd.vx) < 1e-9 && Math.abs(entry.cmd.wz) < 1e-9,
  ), 'keyboard tail must publish zero commands after braking').toBe(true)

  await settle(page, 550)
  const commandsAt3000 = await jogPublishes(page, '/cmd_vel_manual_raw')
  const leasesAt3000 = await jogPublishes(page, '/ui/jog_lease')
  expect(commandsAt3000.filter((entry) => entry.time >= releasedAt + 3000).length,
    'keyboard command timer must stop by 3.0 seconds').toBe(0)
  expect(leasesAt3000.filter((entry) => entry.time >= releasedAt + 3000).length,
    'keyboard lease timer must stop by 3.0 seconds').toBe(0)
})

test('slider focus does not steal keys: w still drives', async ({ page }) => {
  await gotoScreen(page, 'DRIVE_S11', { mode: 'MANUAL' })
  await page.locator('#body .stick svg').waitFor()

  // Focus the speed slider (an <input type="range">); R-52: it keeps focus
  // after a tap, and keyboard jog must keep working.
  const slider = page.locator('#body .stickBox .slider input')
  await slider.focus()
  await page.keyboard.down('w')
  await settle(page, 250)
  await page.keyboard.up('w')
  await settle(page, 100)

  const cmds = await jogPublishes(page, '/cmd_vel_manual_raw')
  expect(cmds.length, 'keyboard hold should publish even with slider focused').toBeGreaterThan(1)
  const moving = cmds.filter((c) => c.cmd.vx > 0 && c.cmd.wz === 0)
  expect(moving.length, 'some ticks should be pure forward').toBeGreaterThan(0)
})

test('slider focus + ArrowUp drives without moving the slider', async ({ page }) => {
  await gotoScreen(page, 'DRIVE_S11', { mode: 'MANUAL' })
  await page.locator('#body .stick svg').waitFor()

  const slider = page.locator('#body .stickBox .slider input')
  await slider.focus()
  const before = await slider.inputValue()
  await page.keyboard.down('ArrowUp')
  await settle(page, 250)
  await page.keyboard.up('ArrowUp')
  await settle(page, 100)

  // window keydown preventDefaults arrows before the typing check, so the
  // focused range must not step.
  expect(await slider.inputValue(), 'arrow key must not move the slider').toBe(before)
  const cmds = await jogPublishes(page, '/cmd_vel_manual_raw')
  const moving = cmds.filter((c) => c.cmd.vx > 0 && c.cmd.wz === 0)
  expect(moving.length, 'ArrowUp should drive forward').toBeGreaterThan(0)
})

test('does not steal keys while a text input is focused', async ({ page }) => {
  await gotoScreen(page, 'DRIVE_S11', { mode: 'MANUAL' })
  await page.locator('#body .stick svg').waitFor()

  // The screen has no text field; add one temporarily and focus it.
  await page.evaluate(() => {
    const el = document.createElement('input')
    el.type = 'text'
    el.id = '__thTextProbe'
    document.getElementById('body').appendChild(el)
    el.focus()
  })
  await page.keyboard.down('w')
  await settle(page, 250)
  await page.keyboard.up('w')
  await settle(page, 100)

  const cmds = await jogPublishes(page, '/cmd_vel_manual_raw')
  expect(cmds.length, 'no drive command while typing in a text input').toBe(0)
})
