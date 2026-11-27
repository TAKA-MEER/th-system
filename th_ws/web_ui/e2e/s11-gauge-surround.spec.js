// S-11 の計器・機体のまわりの図・位置固定（Spec-webui.md §3.4・§3.5）。
import { test, expect } from '@playwright/test'
import { gotoScreen } from './helpers.js'

const SCAN = {
  angle_min: -3.14, angle_increment: 0.0175, range_min: 0.1, range_max: 12,
  ranges: Array.from({ length: 360 }, (_, i) => (i === 180 ? 0.87 : 3)),
}

async function openS11(page, extra = {}) {
  await page.addInitScript((ex) => {
    window.__thTestWheelSpeeds = ex.wheels
    window.__thTestOdomTwist = ex.twist
    window.__thTestScan = ex.scan
    window.__thTestLimiterStatus = ex.limiter
  }, extra)
  await gotoScreen(page, 'S11', { mode: 'MANUAL', state: 'RUN', auto_brake: true })
  await page.locator('#s11').waitFor()
}

test('画面名は「手動走行」', async ({ page }) => {
  await openS11(page, { wheels: { measured: [], command: [] }, twist: null, scan: null, limiter: null })
  await expect(page.locator('#hdr')).toContainText('手動走行')
})

test('計器は大きな速さと回転、出力・エンコーダは「—」', async ({ page }) => {
  await openS11(page, {
    wheels: {
      measured: [{ t: Date.now() / 1000, left: 0.4, right: 0.6 }],
      command: [{ t: Date.now() / 1000, left: 0.5, right: 0.5 }],
    },
    twist: { vx: 0.5, wz: 0.25 },
    scan: null,
    limiter: { action: 'PASS', applied_limit_mps: 0.55, in_linear: 0.5, out_linear: 0.5 },
  })
  await expect(page.getByTestId('gauge-speed')).toHaveText('0.50')
  await expect(page.getByTestId('gauge-rot')).toContainText('0.25')
  await expect(page.getByTestId('gauge-left-output-value')).toContainText('—')
  await expect(page.getByTestId('gauge-left-encoder-value')).toContainText('—')
  await expect(page.getByTestId('gauge-right-output-value')).toContainText('—')
  await expect(page.getByTestId('gauge-right-encoder-value')).toContainText('—')
  await expect(page.getByTestId('gauge-left-actual-value')).toHaveText('0.40')
  await expect(page.getByTestId('gauge-right-actual-value')).toHaveText('0.60')
  await expect(page.getByTestId('gauge-wheel-left-actual')).toHaveCount(1)
  await expect(page.getByTestId('gauge-wheel-right-target')).toHaveCount(1)
  await expect(page.locator('#s11 .gauge-labels')).toContainText('m/s')
  await expect(page.locator('#s11 .gauge-labels')).toContainText('目標')
})

test('まわりの図: 減速の輪・機体・ガイド線・死角の扇形が出る（死角の文は出ない）', async ({ page }) => {
  await openS11(page, {
    wheels: { measured: [{ t: Date.now() / 1000, left: 0.5, right: 0.5 }], command: [] },
    twist: { vx: 0.5, wz: 0 },
    scan: SCAN,
    limiter: { action: 'PASS', applied_limit_mps: 0.55, in_linear: 0.5, out_linear: 0.5 },
  })
  await expect(page.getByTestId('surround-slow-ring')).toBeVisible()
  await expect(page.getByTestId('surround-robot')).toBeVisible()
  await expect(page.getByTestId('surround-guide')).toBeVisible()
  await expect(page.locator('#s11')).not.toContainText('死角があります')
})

test('止まっている間はガイド線が出ない', async ({ page }) => {
  await openS11(page, {
    wheels: { measured: [{ t: Date.now() / 1000, left: 0, right: 0 }], command: [] },
    twist: { vx: 0, wz: 0 },
    scan: null,
    limiter: { action: 'PASS', applied_limit_mps: 0.55, in_linear: 0, out_linear: 0 },
  })
  await expect(page.getByTestId('surround-guide')).toHaveCount(0)
  await expect(page.getByTestId('surround-guide-arc')).toHaveCount(0)
})

test('状態を変えても data-pos の位置が動かない', async ({ page }) => {
  await openS11(page, {
    wheels: { measured: [], command: [] }, twist: null, scan: null, limiter: null,
  })
  const pos = async () => {
    const out = {}
    for (const k of ['opbar', 'main', 'where', 'toggle', 'ops', 'jog']) {
      const b = await page.locator(`#s11 [data-pos="${k}"]`).boundingBox()
      out[k] = b ? { x: Math.round(b.x), y: Math.round(b.y), w: Math.round(b.width), h: Math.round(b.height) } : null
    }
    return out
  }
  const before = await pos()
  await page.evaluate(() => window.__thSetTestState({ mode: 'MANUAL', state: 'PAUSE' }))
  await page.waitForTimeout(300)
  const after = await pos()
  expect(after, '状態変化で位置が動いた').toEqual(before)
})

test('値の行と語の行の高さがそろっている（左の値・語・右の値が横 1 行）', async ({ page }) => {
  await openS11(page, {
    wheels: {
      measured: [{ t: Date.now() / 1000, left: 0.35, right: 0.42 }],
      command: [{ t: Date.now() / 1000, left: 0.36, right: 0.44 }],
    },
    twist: { vx: 0.38, wz: 0.15 },
    scan: null,
    limiter: { action: 'PASS', applied_limit_mps: 0.5, in_linear: 0.4, out_linear: 0.4 },
  })
  const rows = [
    ['gauge-left-actual-value', 'gauge-label-unit', 'gauge-right-actual-value'],
    ['gauge-left-target-value', 'gauge-label-target', 'gauge-right-target-value'],
    ['gauge-left-output-value', 'gauge-label-output', 'gauge-right-output-value'],
    ['gauge-left-encoder-value', 'gauge-label-encoder', 'gauge-right-encoder-value'],
  ]
  for (const [l, label, r] of rows) {
    const lb = await page.getByTestId(l).boundingBox()
    const gb = await page.getByTestId(label).boundingBox()
    const rb = await page.getByTestId(r).boundingBox()
    expect(lb, `${l} が表示されていない`).not.toBeNull()
    expect(gb, `${label} が表示されていない`).not.toBeNull()
    expect(rb, `${r} が表示されていない`).not.toBeNull()
    // 同じ行: 中心 y が近い（左の値・語・右の値が横に読める）
    const cy = (b) => b.y + b.height / 2
    expect(Math.abs(cy(lb) - cy(gb)), `${label} が ${l} と同じ行に無い`).toBeLessThan(6)
    expect(Math.abs(cy(rb) - cy(gb)), `${r} が ${label} と同じ行に無い`).toBeLessThan(6)
    // 左の値 < 語 < 右の値（横の順序）
    expect(lb.x + lb.width, '左の値が語より右').toBeLessThanOrEqual(gb.x + 2)
    expect(gb.x + gb.width, '語が右の値より右').toBeLessThanOrEqual(rb.x + 2)
  }
})
