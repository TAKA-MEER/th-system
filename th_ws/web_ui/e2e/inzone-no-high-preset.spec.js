// e2e/inzone-no-high-preset.spec.js — Spec-webui.md「速度プリセット」（2026-10-13）:
// 場内の画面（S-20・S-21）では「高速」を出さない／選べない。場外（S-11 など）は従来どおり
// 低速・中速・高速の3段。ブラウザは割合を返すだけで速度上限は持たない（obstacle_limiter が権威）。
import { expect, test } from '@playwright/test'
import {
  downOnStick, gotoScreen, gotoScreenWithOnsite, jogPublishes,
} from './helpers.js'

const MID = 0.55 // generated/speed_presets.json の speed_preset_mid（unit が registry と照合済み）

async function openW6(page, screen) {
  if (screen === 'S20') {
    await gotoScreenWithOnsite(page, 'S20', { mode: 'PREP', state: 'MAPPING' },
      { pins: [], targets: [], mappingActive: true })
    await page.locator('#s20').waitFor()
    await page.locator('#s20 .op-manual').click()
  } else {
    await gotoScreenWithOnsite(page, 'S21', { mode: 'AT_HOME', state: 'IDLE_H' },
      { pins: [], targets: [] })
    await page.locator('#s21').waitFor()
    await page.locator('#s21 .op-manual').click()
  }
  await page.locator('#jogWin.show').waitFor()
}

for (const screen of ['S20', 'S21']) {
  test(`${screen}: 手動操作パネルに「高速」が無い（低速・中速だけ）`, async ({ page }) => {
    await openW6(page, screen)
    const btns = page.locator('#jogWin .spd button')
    await expect(btns).toHaveCount(2)
    await expect(btns.filter({ hasText: '高速' })).toHaveCount(0)
    await expect(btns.filter({ hasText: '中速' })).toHaveCount(1)
    await expect(btns.filter({ hasText: '低速' })).toHaveCount(1)
  })

  test(`${screen}: スライダも中速（55%）までで、スティックを倒し切っても中速の割合を超えない`, async ({ page }) => {
    await openW6(page, screen)
    await expect(page.locator('#jogWin .slider input')).toHaveAttribute('max', '55')
    const stick = page.locator('#jogWin .stick svg')
    await downOnStick(page, stick, { offsetX: 0, offsetY: -0.9 })
    await page.waitForTimeout(4000) // ランプが目標に届くまで
    const cmds = (await jogPublishes(page, '/cmd_vel_manual_raw')).filter((c) => c.cmd)
    expect(cmds.length).toBeGreaterThan(3)
    const max = Math.max(...cmds.map((c) => c.cmd.vx))
    expect(max, '場内の画面で中速を超える指令が出た').toBeLessThanOrEqual(MID + 1e-9)
    expect(max, '中速まで届いていない（試験が空振り）').toBeGreaterThan(0.4)
    await page.mouse.up()
  })
}

test('場外の画面（S-11）は従来どおり低速・中速・高速の3段', async ({ page }) => {
  await gotoScreen(page, 'S11', { mode: 'MANUAL', state: 'PAUSE' })
  const btns = page.locator('#body .spd button')
  await expect(btns).toHaveCount(3)
  await expect(btns.filter({ hasText: '高速' })).toHaveCount(1)
  await expect(page.locator('#body .slider input')).toHaveAttribute('max', '100')
})

test('場外の画面で高速を選んでスティックを倒すと高速の割合が出る（比較対象）', async ({ page }) => {
  await gotoScreen(page, 'S11', { mode: 'MANUAL', state: 'PAUSE' })
  await page.locator('#body .spd button', { hasText: '高速' }).click()
  const stick = page.locator('#body .stick svg')
  await downOnStick(page, stick, { offsetX: 0, offsetY: -0.9 })
  await page.waitForTimeout(4000)
  const cmds = (await jogPublishes(page, '/cmd_vel_manual_raw')).filter((c) => c.cmd)
  expect(Math.max(...cmds.map((c) => c.cmd.vx))).toBeGreaterThan(MID + 0.05)
  await page.mouse.up()
})
