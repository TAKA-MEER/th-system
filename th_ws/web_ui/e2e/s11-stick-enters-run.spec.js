// DetailedDesign-wp3.md WP-TRANSIT-01 §7 (T1-1) / DetailedDesign-transit.md
// §2.2: "スティックは走行操作であってジョグ介入ではない". Touching the S-11
// stick must immediately start driving — not open a manual panel (W-6) and not
// fall to a jog-intervention PAUSE. The PAUSE -> RUN edge itself is th_state's
// T-MANUAL-01 ("UI は何もしない"), but the UI's own half is: the stick is
// permanent (no "手動" button to open W-6) and touching it starts the
// /ui/jog_lease + /cmd_vel_manual_raw flow that jog_gate / th_state turn into
// a run entry.
//
// So this spec verifies S-11 has no "手動" button, and that in mode MANUAL /
// state PAUSE the stick publishes the drive flow (lease + cmd) the moment it's
// touched — the UI prerequisite to RUN — while staying on S-11 with no
// confirm/PAUSE button being offered.
import { test, expect } from '@playwright/test'
import { gotoScreen, downOnStick, jogPublishes } from './helpers.js'

test('S-11 has no 手動 button (stick is the only drive entry)', async ({ page }) => {
  await gotoScreen(page, 'S11', { mode: 'MANUAL', state: 'PAUSE' })
  // W-6's opening button is never present on the perpetual-stick screens
  // (DetailedDesign-webui.md §4.2: "スティックそのものが走行操作").
  await expect(page.getByRole('button', { name: '手動' })).toHaveCount(0)
})

test('touching the S-11 stick publishes the drive flow (run entry)', async ({ page }) => {
  await gotoScreen(page, 'S11', { mode: 'MANUAL', state: 'PAUSE' })

  const stick = page.locator('#body .stick svg')
  await stick.waitFor()

  await downOnStick(page, stick, { offsetX: 0.45, offsetY: 0 })
  await page.waitForTimeout(600)
  await page.mouse.up()

  // Holding for ~0.6s publishes the lease (run entry) and raw velocity.
  // The WhatGotBroken "no run entry" case would leave these at zero.
  const leases = await jogPublishes(page, '/ui/jog_lease')
  const cmds = await jogPublishes(page, '/cmd_vel_manual_raw')
  expect(leases.length).toBeGreaterThan(0)
  expect(cmds.length).toBeGreaterThan(0)
})

test('while held, S-11 does not fall into a confirm/pause button UI', async ({ page }) => {
  await gotoScreen(page, 'S11', { mode: 'MANUAL', state: 'PAUSE' })
  const stick = page.locator('#body .stick svg')
  await stick.waitFor()

  await downOnStick(page, stick, { offsetX: 0.45, offsetY: 0 })
  await page.waitForTimeout(300)

  // The operation card still has only 停止; no confirm (check) step appears
  // locally (a jog-intervention/confirm flow would surface a check slot).
  await expect(page.locator('#s11 .card.ops .op-check')).toHaveCount(0)
  await page.mouse.up()
})
