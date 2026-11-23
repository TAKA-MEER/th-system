// 8-g P3-ui: ADJUST（盤前の旋回・前後の並進）の画面。
// DetailedDesign-measure.md §6.4（P3-ui の表）。偽 rosbridge の流儀。
import { test, expect } from '@playwright/test'
import {
  gotoScreenWithOnsite, gotoScreenWithMeasure, setTestState, setTestFault, stubTrigger,
} from './helpers.js'

function pin(id, name, kind, x, y, yawDeg = 90) {
  const rad = (yawDeg * Math.PI) / 180
  return {
    id, name, kind, registered_at: '',
    pose: {
      position: { x, y, z: 0 },
      orientation: { x: 0, y: 0, z: Math.sin(rad / 2), w: Math.cos(rad / 2) },
    },
  }
}
const PINS = [
  pin('p1', '盤 A-1', 'PANEL', 86, 88, 92),
  pin('p3', '待機場所', 'HOME', 70, 190, 274),
]
const TARGETS = {
  candidates: [{ x: 1.2, y: 0.4 }],
  selected_index: 0,
  confidence: 0.87,
  is_lost: false,
  lost_reason: '',
}
const PARAMS = {
  get: { effective: { measure_ui_port: 18091, camera_ui_url: 'http://192.168.5.129/' }, pending: {} },
}

async function triggers(page) {
  return page.evaluate(() => window.__thTriggerCalls ?? [])
}

async function goto21(page, state, extra = {}) {
  const seed = {
    mode: 'IDLE', state: 'NONE', tracker_enabled: true, ...state,
  }
  await gotoScreenWithOnsite(page, 'S21', seed, {
    pins: PINS, targets: TARGETS, pose: { x: 0, y: 0, yaw: 0 },
    paramsStubs: PARAMS, ...extra,
  })
  await page.locator('#s21').waitFor()
}

async function stubWindowOpen(page) {
  await page.addInitScript(() => {
    window.__thOpened = []
    window.open = (url, target, features) => {
      window.__thOpened.push({ url, target, features })
      return null
    }
  })
}

// ── a. AT_PANEL/ADJUST の S-21 ──
test('a: AT_PANEL/ADJUST では「停止を解く」が押せず理由が出る。行き先・呼び寄せ・手動も押せない。「測定機能の画面へ」は押せる', async ({ page }) => {
  await stubWindowOpen(page)
  await goto21(page, { mode: 'AT_PANEL', state: 'ADJUST', working: true })

  await expect(page.locator('[data-testid="s21-state"]')).toHaveText('測定中（停止命令）（測定機能の指示で動作中）')

  // 行き先（dest タブが既定）
  await expect(page.locator('[data-testid="s21-dest-home"]')).toBeDisabled()
  await expect(page.locator('[data-testid="s21-dest-next-panel"]')).toBeDisabled()
  await expect(page.locator('[data-testid="s21-dest-summon-here"]')).toBeDisabled()

  // 手動（操作カード）
  await expect(page.locator('[data-testid="reason-manual"]')).toBeVisible()

  // 呼び寄せ
  await page.locator('[data-testid="s21-subtab-summon"]').click()
  await expect(page.locator('[data-testid="s21-summon-start"]')).toBeDisabled()

  // 「停止を解く」は押せない（理由 adjust_in_progress）。押しても W-4 は出ない
  await page.locator('[data-testid="s21-subtab-atp"]').click()
  const unlock = page.locator('[data-testid="s21-working-unlock"]')
  await expect(unlock).toBeVisible()
  await expect(unlock).toBeDisabled()
  await expect(page.locator('[data-testid="reason-s21-working-unlock"]')).toContainText('旋回中・並進中は腕が起きているため解けません')

  // 「測定機能の画面へ」は押せる（測定機能の画面へ戻る）
  const open = page.locator('[data-testid="s21-open-measure"]')
  await expect(open).toBeEnabled()
  const [req] = await Promise.all([
    page.waitForEvent('request', (r) => r.url().includes(':18091')),
    open.click(),
  ])
  expect(req.url()).toBe('http://localhost:18091/')

  expect((await triggers(page)).filter((c) => c.trigger === 'ui.working')).toHaveLength(0)
})

// ── b. AT_PANEL/ADJUST のフォルト → W-1 ──
test('b: AT_PANEL/ADJUST でフォルトが出たら W-1 が出て、解消後に「確認」で ui.resume_ack を送る', async ({ page }) => {
  await goto21(page, { mode: 'AT_PANEL', state: 'ADJUST', working: true })
  await setTestFault(page, { active: true, fault_type: 'LIDAR_LOST', severity: 'RECOVERABLE' })

  const win = page.locator('.win.fault.show')
  await expect(win).toBeVisible()

  await setTestFault(page, { active: false, severity: '', fault_type: 'LIDAR_LOST' })
  const ack = win.getByRole('button', { name: '確認', exact: true })
  await expect(ack).toBeVisible()
  await ack.click()

  await expect(page.locator('[data-testid="w4-arm-ask"]')).toHaveCount(0)
  const sent = (await triggers(page)).filter((c) => c.trigger === 'ui.resume_ack')
  expect(sent).toHaveLength(1)
})

// ── c. AUTO_MEASURE/ADJUST の S-60 ──
test('c: AUTO_MEASURE/ADJUST で S-60 に「測定機能の指示で動作中」が出る', async ({ page }) => {
  await gotoScreenWithMeasure(page, 'S01',
    { mode: 'AUTO_MEASURE', state: 'ADJUST', tracker_enabled: true },
    { link_ok: true, mode: 'AUTO_MEASURE', state: 'ADJUST' })

  await expect(page.locator('[data-testid="s60-state"]')).toHaveText('測定機能の指示で動作中')
  await expect(page.locator('[data-testid="s60-open-measure"]')).toBeVisible()
  await expect(page.locator('[data-testid="s60-finish"]')).toBeVisible()
  await expect(page.locator('[data-testid="s60-manual"]')).toBeVisible()
})

// ── d. ESTOP（prev_state ADJUST・working true）の W-1 は W-4 を挟む ──
test('d: ESTOP prev_state ADJUST・working true の「メインメニューへ」は W-4 を挟む。やめるで送らず、抜けるは ui.resume_no 1 回', async ({ page }) => {
  await goto21(page, {
    mode: 'ESTOP', prev_mode: 'AT_PANEL', prev_state: 'ADJUST', working: true,
    estop_ui: false, estop_hw: false, estop_from_ui: true,
  })

  const menu = page.getByRole('button', { name: 'メインメニューへ' })
  await expect(menu).toBeVisible()
  await menu.click()
  await expect(page.locator('[data-testid="w4-arm-ask"]')).toBeVisible()

  await page.locator('[data-testid="w4-arm-cancel"]').click()
  expect((await triggers(page)).filter((c) => c.trigger === 'ui.resume_no')).toHaveLength(0)

  await menu.click()
  await expect(page.locator('[data-testid="w4-arm-ask"]')).toBeVisible()
  await page.locator('[data-testid="w4-arm-ok"]').click()
  expect((await triggers(page)).filter((c) => c.trigger === 'ui.resume_no')).toHaveLength(1)
})
