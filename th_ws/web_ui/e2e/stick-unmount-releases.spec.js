// DetailedDesign-wp3.md WP-UI-03 §9 / U3-1: if the screen unmounts while the
// stick is still being dragged (tab switch, leaving the screen), pointerup
// never arrives -- the last jog command must not keep flowing. On unmount the
// hook must send zero once and stop. The test drives the perpetual stick,
// then uses the harness's "離れる" button to unmount DriveTab mid-hold.
import { test, expect } from '@playwright/test'
import { gotoScreen, downOnStick, jogPublishes } from './helpers.js'

test('unmounting the drive tab mid-drag releases (zero once) and stops', async ({ page }) => {
  await gotoScreen(page, 'DRIVE_S11', { mode: 'IDLE' })
  // `#body` に限定するのは、W-6（`#jogWin`）にも同じ大きさのスティックが
  // 常時 DOM 上にあるため（webui.md §6.3「常設するものと同じ大きさ」）。
  // `.stick svg` だけだと strict mode violation で 2 件にマッチする。
  const stick = page.locator('#body .stick svg')
  await stick.waitFor()

  await downOnStick(page, stick, { offsetX: 0.45, offsetY: 0 })
  await page.waitForTimeout(200)
  const heldCount = (await jogPublishes(page, '/cmd_vel_manual_raw')).length
  expect(heldCount).toBeGreaterThanOrEqual(2) // sure it was actually sending

  // Leave the screen without releasing the mouse -- DriveTab unmounts.
  // `page.click()` は使えない。スティックが setPointerCapture() で
  // ポインタを掴んだままなので、実マウスの押下はボタンに届かない
  // （そしてそれこそがこのテストの前提＝離さないまま画面を離れる）。
  // dispatchEvent なら捕捉を迂回してボタンだけを押せる。
  await page.locator('[data-testid="leave-drive"]').dispatchEvent('click')
  // DriveTab が本当に外れたことを、印の div ではなくスティックが
  // 消えたことで確かめる（中身の無い div は Playwright から見て
  // 大きさ 0 = 不可視なので、既定の waitFor() では永久に待つ）。
  await expect(page.locator('#body .stick svg')).toHaveCount(0)
  await page.waitForTimeout(250)

  const cmds = await jogPublishes(page, '/cmd_vel_manual_raw')
  // Exactly one extra publish (the release zero) after the held window.
  expect(cmds.length - heldCount).toBe(1)
  expect(cmds[cmds.length - 1].cmd.vx).toBe(0)
  expect(cmds[cmds.length - 1].cmd.wz).toBe(0)

  // And nothing arrives afterwards.
  await page.waitForTimeout(250)
  expect((await jogPublishes(page, '/cmd_vel_manual_raw')).length).toBe(cmds.length)
})
