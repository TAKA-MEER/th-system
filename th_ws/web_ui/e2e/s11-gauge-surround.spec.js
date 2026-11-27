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
