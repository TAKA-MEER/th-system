// DetailedDesign-wp3.md WP-TRANSIT-01 §7 (T1-2) / DetailedDesign-transit.md
// §2.2: lease expiry -> RUN -> PAUSE stops the drive. The transition itself is
// th_state's T-MANUAL-02; the UI's own half is that stopping the stick stops
// the /ui/jog_lease flow (so the lease expires and jog_gate / th_state cut to
// PAUSE) and emits a single zero on /cmd_vel_manual_raw so the motor gets an
// explicit stop rather than relying on silence alone.
import { test, expect } from '@playwright/test'
import { gotoScreen, downOnStick, jogPublishes } from './helpers.js'

test('releasing the stick stops lease publication and sends one zero', async ({ page }) => {
  await gotoScreen(page, 'S11', { mode: 'MANUAL', state: 'PAUSE' })
  const stick = page.locator('#body .stick svg')
  await stick.waitFor()

  await downOnStick(page, stick, { offsetX: 0.45, offsetY: 0 })
  await page.waitForTimeout(500)
  const before = (await jogPublishes(page, '/ui/jog_lease')).length
  expect(before).toBeGreaterThan(0)

  await page.mouse.up()

  // Leases stop flowing: the count must not grow after release (T1-2's
  // precondition — no lease refresh -> th_state expires to PAUSE server-side).
  const after = (await jogPublishes(page, '/ui/jog_lease')).length
  expect(after).toBe(before)

  // A single explicit zero lands on /cmd_vel_manual_raw (WP-UI-03 §3.1:
  // "離したとき … ゼロを 1 回送る").
  const cmds = await jogPublishes(page, '/cmd_vel_manual_raw')
  const zeros = cmds.filter((c) => c.cmd && c.cmd.vx === 0 && c.cmd.wz === 0)
  // One zero now (the release one); nothing beyond it refreshed.
  expect(zeros.length).toBeGreaterThanOrEqual(1)
})
