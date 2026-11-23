// 8-g P2-ui: S-21 の「測定機能へ」「停止を解く」と WORKING・W-1・腕の確認。
// DetailedDesign-measure.md §6.3（P2-ui の表）。偽 rosbridge の流儀。
import { test, expect } from '@playwright/test'
import {
  gotoScreenWithOnsite, setTestState, setTestFault, stubTrigger,
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

// ── a. 「測定機能へ」→ ui.working 1 回、WORKING になってから移る ──
test('a: AT_PANEL/IDLE_P で「測定機能へ」は ui.working{on:true} をちょうど1回送り、WORKINGになってから measure_ui_port へ移る', async ({ page }) => {
  await stubTrigger(page, { 'ui.working': { accepted: true } })
  await goto21(page, { mode: 'AT_PANEL', state: 'IDLE_P', working: false })

  await page.locator('[data-testid="s21-subtab-atp"]').click()
  const btn = page.locator('[data-testid="s21-measure-go"]')
  await expect(btn).toBeVisible()
  await expect(btn).toBeEnabled()

  const requests = []
  page.on('request', (r) => requests.push(r.url()))

  await btn.click()
  await expect.poll(async () => (await triggers(page)).filter((c) => c.trigger === 'ui.working')).toHaveLength(1)
  const w = (await triggers(page)).filter((c) => c.trigger === 'ui.working')
  expect(w[0].argJson).toEqual({ on: true })

  // 状態が変わらなければ移らない
  await page.waitForTimeout(400)
  expect(requests.filter((u) => u.includes(':18091'))).toHaveLength(0)

  const [req] = await Promise.all([
    page.waitForEvent('request', (r) => r.url().includes(':18091')),
    setTestState(page, { mode: 'AT_PANEL', state: 'WORKING', working: true }),
  ])
  expect(req.url()).toBe('http://localhost:18091/')
})

test('a: 拒否されたら理由を出して移らない', async ({ page }) => {
  await stubTrigger(page, { 'ui.working': { accepted: false, reject_reason_key: 'not_allowed' } })
  await goto21(page, { mode: 'AT_PANEL', state: 'IDLE_P', working: false })

  await page.locator('[data-testid="s21-subtab-atp"]').click()
  const requests = []
  page.on('request', (r) => requests.push(r.url()))

  await page.locator('[data-testid="s21-measure-go"]').click()
  await expect(page.locator('[data-testid="s21-measure-reason"]')).toContainText('今の状態ではこの操作はできません')
  await page.waitForTimeout(300)
  expect(requests.filter((u) => u.includes(':18091'))).toHaveLength(0)
  expect((await triggers(page)).filter((c) => c.trigger === 'ui.working')).toHaveLength(1)
})

// ── b. WORKING の S-21: 行き先・呼び寄せ・手動が押せず理由が出る。カメラは押せる ──
test('b: AT_PANEL/WORKING では行き先・呼び寄せ・手動が押せず理由が出る。「カメラ」は押せる', async ({ page }) => {
  await stubWindowOpen(page)
  await goto21(page, { mode: 'AT_PANEL', state: 'WORKING', working: true })

  await expect(page.locator('[data-testid="s21-state"]')).toHaveText('測定中（停止命令）')

  // 行き先（dest タブが既定）
  await expect(page.locator('[data-testid="s21-dest-home"]')).toBeDisabled()
  await expect(page.locator('[data-testid="reason-s21-dest-home"]')).toContainText('作業中は行き先を選べません')
  await expect(page.locator('[data-testid="s21-dest-next-panel"]')).toBeDisabled()
  await expect(page.locator('[data-testid="s21-dest-summon-here"]')).toBeDisabled()

  // 手動（操作カード）
  await expect(page.locator('[data-testid="reason-manual"]')).toContainText('作業中は行き先を選べません')

  // 呼び寄せ
  await page.locator('[data-testid="s21-subtab-summon"]').click()
  await expect(page.locator('[data-testid="s21-summon-start"]')).toBeDisabled()

  // カメラは押せる
  await page.locator('[data-testid="s21-subtab-atp"]').click()
  const cam = page.locator('[data-testid="s21-camera-open"]')
  await expect(cam).toBeEnabled()
  await cam.click()
  const opened = await page.evaluate(() => window.__thOpened ?? [])
  expect(opened).toHaveLength(1)
  expect(opened[0].url).toBe('http://192.168.5.129/')
  expect((await triggers(page)).filter((c) => c.trigger === 'ui.working')).toHaveLength(0)
})

// ── c. 「停止を解く」→ W-4 → やめるは送らない／解くは ui.working{on:false} 1 回 ──
test('c: 「停止を解く」は W-4 を挟む。やめるで送らず、解くで ui.working{on:false} を1回送る', async ({ page }) => {
  await stubTrigger(page, { 'ui.working': { accepted: true } })
  await goto21(page, { mode: 'AT_PANEL', state: 'WORKING', working: true })

  await page.locator('[data-testid="s21-subtab-atp"]').click()
  const unlock = page.locator('[data-testid="s21-working-unlock"]')
  await expect(unlock).toBeVisible()

  await unlock.click()
  await expect(page.locator('.win.confirm.show')).toContainText('測定機構の腕が格納位置にあることを確かめましたか')
  await page.locator('[data-testid="s21-arm-cancel"]').click()
  expect((await triggers(page)).filter((c) => c.trigger === 'ui.working')).toHaveLength(0)

  await unlock.click()
  await expect(page.locator('.win.confirm.show')).toContainText('腕が格納位置にあること')
  await page.locator('[data-testid="s21-arm-ok"]').click()

  const sent = (await triggers(page)).filter((c) => c.trigger === 'ui.working')
  expect(sent).toHaveLength(1)
  expect(sent[0].argJson).toEqual({ on: false })
})

// ── d. AT_PANEL/WORKING のフォルト → W-1 → 解消後「確認」で ui.resume_ack ──
test('d: AT_PANEL/WORKING でフォルトが出たら W-1 が出て、解消後に「確認」で ui.resume_ack を送る', async ({ page }) => {
  await goto21(page, { mode: 'AT_PANEL', state: 'WORKING', working: true })
  await setTestFault(page, { active: true, fault_type: 'LIDAR_LOST', severity: 'RECOVERABLE' })

  const win = page.locator('.win.fault.show')
  await expect(win).toBeVisible()

  await setTestFault(page, { active: false, severity: '', fault_type: 'LIDAR_LOST' })
  const ack = win.getByRole('button', { name: '確認', exact: true })
  await expect(ack).toBeVisible()
  await ack.click()

  // WORKING に留まる「確認」は腕の確認を挟まない
  await expect(page.locator('[data-testid="w4-arm-ask"]')).toHaveCount(0)
  const sent = (await triggers(page)).filter((c) => c.trigger === 'ui.resume_ack')
  expect(sent).toHaveLength(1)
})

// ── e. ESTOP（prev AT_PANEL・working true）の W-1 は W-4 を挟む。working false ならすぐ送る ──
test('e: ESTOP prev_mode AT_PANEL・working true の「メインメニューへ」は W-4 を挟む。やめるで送らず、抜けるは ui.resume_no 1 回', async ({ page }) => {
  await goto21(page, {
    mode: 'ESTOP', prev_mode: 'AT_PANEL', working: true, estop_ui: false, estop_hw: false, estop_from_ui: true,
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

test('e: working false なら W-4 を出さずすぐ送る', async ({ page }) => {
  await goto21(page, {
    mode: 'ESTOP', prev_mode: 'AT_PANEL', working: false, estop_ui: false, estop_hw: false, estop_from_ui: true,
  })
  await page.getByRole('button', { name: 'メインメニューへ' }).click()
  await expect(page.locator('[data-testid="w4-arm-ask"]')).toHaveCount(0)
  expect((await triggers(page)).filter((c) => c.trigger === 'ui.resume_no')).toHaveLength(1)
})

// ── f. CARRY（working true）の W-2 も W-4 を挟む ──
test('f: CARRY・working true の W-2「メインメニューへ」は W-4 を挟む。やめるで送らず、抜けるは ui.finish 1 回', async ({ page }) => {
  await stubTrigger(page, { 'ui.finish': { accepted: true } })
  await goto21(page, {
    mode: 'CARRY', prev_mode: 'AT_PANEL', working: true, estop_hw: false, estop_ui: false,
  })

  const dismiss = page.locator('.win.carry').locator('[data-testid="w2-c"]')
  await expect(dismiss).toBeVisible()
  await dismiss.click()
  await expect(page.locator('[data-testid="w4-arm-ask"]')).toBeVisible()
  await page.locator('[data-testid="w4-arm-cancel"]').click()
  expect((await triggers(page)).filter((c) => c.trigger === 'ui.finish')).toHaveLength(0)

  await dismiss.click()
  await expect(page.locator('[data-testid="w4-arm-ask"]')).toBeVisible()
  await page.locator('[data-testid="w4-arm-ok"]').click()
  expect((await triggers(page)).filter((c) => c.trigger === 'ui.finish')).toHaveLength(1)
})

// ── g. AT_PANEL/WORKING の「終了」は W-4 を挟む ──
test('g: AT_PANEL/WORKING の「終了」は W-4 を挟む。やめるで送らず、進むは ui.finish 1 回', async ({ page }) => {
  await stubTrigger(page, { 'ui.finish': { accepted: true } })
  await goto21(page, { mode: 'AT_PANEL', state: 'WORKING', working: true })

  const finish = page.locator('[data-testid="s21-finish"]')
  await finish.click()
  await expect(page.locator('.win.confirm.show')).toContainText('測定機構の腕が格納位置にあることを確かめましたか')

  await page.locator('[data-testid="s21-arm-cancel"]').click()
  expect((await triggers(page)).filter((c) => c.trigger === 'ui.finish')).toHaveLength(0)

  await finish.click()
  await expect(page.locator('.win.confirm.show')).toContainText('腕が格納位置にあること')
  await page.locator('[data-testid="s21-arm-ok"]').click()
  expect((await triggers(page)).filter((c) => c.trigger === 'ui.finish')).toHaveLength(1)
})
