// DetailedDesign-wp3.md WP-TRANSIT-01 §7 (T1-4) / §10 ①: the S-11 operation
// card has exactly one button — 停止. There is deliberately no 走行 / 確認 /
// 保存 / 手動 slot, because the stick is the only way to enter driving
// (T1-4: "走行に入る契機はスティックだけ").
import { test, expect } from '@playwright/test'
import { gotoScreen } from './helpers.js'

test('S-11 operation card offers 停止 only', async ({ page }) => {
  await gotoScreen(page, 'S11', { mode: 'MANUAL', state: 'PAUSE', tracker_enabled: true })

  const ops = page.locator('#s11 .card.ops')
  await expect(ops).toBeVisible()

  await expect(ops.getByRole('button', { name: '停止' })).toHaveCount(1)
  // No other op-card slots (T1-4 / §10 ①: no run / manual).
  await expect(ops.getByRole('button', { name: '走行' })).toHaveCount(0)
  await expect(ops.getByRole('button', { name: '手動' })).toHaveCount(0)
  await expect(ops.getByRole('button', { name: '確認' })).toHaveCount(0)
  await expect(ops.getByRole('button', { name: '保存' })).toHaveCount(0)
})

test('S-11 ops card has no semantic .opsgrid slot beyond stop', async ({ page }) => {
  await gotoScreen(page, 'S11', { mode: 'MANUAL', state: 'PAUSE' })
  // §10 ① machine check: no run:true / manual:true in the slots.
  const grid = page.locator('#s11 .card.ops .opsgrid')
  await expect(grid.locator('.op-run')).toHaveCount(0)
  await expect(grid.locator('.op-manual')).toHaveCount(0)
  await expect(grid.locator('.op-check')).toHaveCount(0)
  await expect(grid.locator('.op-save')).toHaveCount(0)
})
