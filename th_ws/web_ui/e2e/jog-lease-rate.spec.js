// DetailedDesign-wp3.md WP-UI-03 §9-8: while the operator holds the stick,
// the UI must refresh the /ui/jog_lease at >= 5 Hz and drive
// /cmd_vel_manual_raw at 10 Hz, and it must never send a `ui.jog.release`.
//
// There is no rosbridge in this e2e environment, so ros/useJogLease.js
// records each send on window.__thJogPublishes (same TEST_MODE pattern as
// useActiveScreenPublisher.js). `data` on a lease entry is the client id --
// if it ever carried a "release" sentinel that would be a `/system/trigger`
// leaking through the wrong code path.
import { test, expect } from '@playwright/test'
import { gotoScreen, downOnStick, jogPublishes } from './helpers.js'

test('leases at >= 5 Hz and never sends a jog.release', async ({ page }) => {
  await gotoScreen(page, 'DRIVE_S11', { mode: 'IDLE' })
  // `#body` に限定するのは、W-6（`#jogWin`）にも同じ大きさのスティックが
  // 常時 DOM 上にあるため（webui.md §6.3「常設するものと同じ大きさ」）。
  // `.stick svg` だけだと strict mode violation で 2 件にマッチする。
  const stick = page.locator('#body .stick svg')
  await stick.waitFor()

  await downOnStick(page, stick, { offsetX: 0.45, offsetY: 0 })
  await page.waitForTimeout(1200) // hold ~1.2s => ~6 lease ticks at 5 Hz
  await page.mouse.up()

  const leases = await jogPublishes(page, '/ui/jog_lease')
  const cmds = await jogPublishes(page, '/cmd_vel_manual_raw')

  // Immediate first publish on mount-of-hold + one per 200ms. 5 Hz floors.
  expect(leases.length).toBeGreaterThanOrEqual(6)
  // cmd refresh 10 Hz (100ms) => at least 12 over the same window.
  expect(cmds.length).toBeGreaterThanOrEqual(12)

  // Lease payload is this client's id, never a "release" marker.
  for (const l of leases) {
    expect(typeof l.data).toBe('string')
    expect(l.data.length).toBeGreaterThan(0)
    expect(l.data).not.toBe('release')
  }
})
