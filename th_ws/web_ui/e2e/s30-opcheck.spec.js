// e2e/s30-opcheck.spec.js — S-30 始業点検 (WP-UI-08). Binds the production
// send paths, not just source text:
//   - selecting an item sends ui.check_item (/system/trigger)
//   - ESTOP's はい/いいえ send /opcheck/answer
//   - MOTOR's hold buttons repeat /opcheck/motor_hold while held and send
//     NONE exactly once on every kind of release (pointerup, pointercancel,
//     tab hidden, screen unmount, leaving RUNNING_CHECK) -- and never while
//     the gate is closed at all.
//
// Uses a local goto helper instead of e2e/helpers.js's gotoScreen because
// this screen needs two extra test-mode seeds gotoScreen doesn't carry
// (window.__thTestOpcheckStatus, window.__thTestWheelSpeeds) plus a fixed
// window.__thTestOpcheckAnswer stub (a function, so it can't travel through
// addInitScript's serialized argument -- it's defined inline in the page
// script instead).
import { test, expect } from '@playwright/test'
import { stubTrigger } from './helpers.js'

async function gotoS30(page, { state, opcheckStatus, wheelSpeeds, imuCalib, scan, tunables } = {}) {
  await page.addInitScript(({ s, os, ws, ic, sc, tu }) => {
    if (ic != null) window.__thTestImuCalib = ic
    if (sc) window.__thTestScan = sc
    if (tu) window.__thTunableStubs = tu
    window.__thTestState = s
    window.__thTestScreen = 'S30'
    window.__thTestOpcheckAnswer = () => ({ accepted: true })
    if (os) window.__thTestOpcheckStatus = os
    if (ws) window.__thTestWheelSpeeds = ws
  }, { s: { mode: 'OPCHECK', state: 'LIST', ...state }, os: opcheckStatus, ws: wheelSpeeds, ic: imuCalib, sc: scan, tu: tunables })
  await page.goto('/')
}

// Dispatches a synthetic PointerEvent with an explicit, caller-chosen
// pointerId so pointerdown/pointerup/pointercancel always agree on identity
// (a real Playwright page.mouse.* pointerId isn't guaranteed to match what
// a follow-up dispatchEvent() would pick) -- same technique as
// e2e/s20-prep.spec.js's 2-finger pinch test.
async function firePointer(page, testId, type, id = 7) {
  await page.evaluate(({ testId, type, id }) => {
    const el = document.querySelector(`[data-testid="${testId}"]`)
    const rect = el.getBoundingClientRect()
    el.dispatchEvent(new PointerEvent(type, {
      pointerId: id, clientX: rect.x + rect.width / 2, clientY: rect.y + rect.height / 2,
      bubbles: true, cancelable: true,
    }))
  }, { testId, type, id })
}

async function motorHoldPublishes(page) {
  return page.evaluate(() => window.__thMotorHoldPublishes ?? [])
}

test('項目を選ぶと ui.check_item が実際に送られる', async ({ page }) => {
  await stubTrigger(page, { 'ui.check_item': { accepted: true } })
  await gotoS30(page, { state: { state: 'LIST' } })

  await page.getByTestId('s30-item-MOTOR').click()

  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(calls.some((c) => c.trigger === 'ui.check_item' && c.argJson?.item === 'MOTOR')).toBeTruthy()
})

test('ESTOP: 押下→はい→解除→はい の流れで /opcheck/answer が2回送られる（estop_hw 中も操作できる）', async ({ page }) => {
  await gotoS30(page, {
    state: { state: 'RUNNING_CHECK', estop_hw: false },
    opcheckStatus: { item: 'ESTOP', result: 'UNKNOWN', detail: '', next_screen: '' },
  })

  await expect(page.getByTestId('s30-estop-step')).toContainText('押してください')
  await expect(page.getByTestId('s30-estop-yes')).toHaveCount(0)

  // 物理ボタン押下（estop_hw:true）。W-2/手押しウィンドウは mode==='CARRY' でしか
  // 開かないので、OPCHECK のままここへは被らない -- 「はい」が押せることで確かめる。
  await page.evaluate(() => window.__thSetTestState({ estop_hw: true }))
  await expect(page.getByTestId('s30-estop-step')).toContainText('一致していますか')
  await page.getByTestId('s30-estop-yes').click()

  let calls = await page.evaluate(() => window.__thOpcheckAnswerCalls ?? [])
  expect(calls).toContainEqual({ item: 'ESTOP', ok: true })

  await expect(page.getByTestId('s30-estop-step')).toContainText('解除してください')
  await page.evaluate(() => window.__thSetTestState({ estop_hw: false }))
  await expect(page.getByTestId('s30-estop-step')).toContainText('一致していますか')
  await page.getByTestId('s30-estop-yes').click()

  calls = await page.evaluate(() => window.__thOpcheckAnswerCalls ?? [])
  expect(calls.filter((c) => c.item === 'ESTOP' && c.ok === true).length).toBe(2)
})

test('ESTOP: いいえ で ok:false が送られる', async ({ page }) => {
  await gotoS30(page, {
    state: { state: 'RUNNING_CHECK', estop_hw: true },
    opcheckStatus: { item: 'ESTOP', result: 'UNKNOWN', detail: '', next_screen: '' },
  })
  await page.getByTestId('s30-estop-no').click()
  const calls = await page.evaluate(() => window.__thOpcheckAnswerCalls ?? [])
  expect(calls).toContainEqual({ item: 'ESTOP', ok: false })
})

test('LIST 状態では MOTOR のホールドボタンは存在しない（ゲート: 選択済み項目が無い）', async ({ page }) => {
  await gotoS30(page, { state: { state: 'LIST' } })
  await expect(page.getByTestId('s30-motor-forward')).toHaveCount(0)
})

// brief-a13（SG-A13）: 4 方向の済み／未は /opcheck/status の detail（done=...）から出す。
// 本番は runner が UNKNOWN の detail 先頭に進捗を載せる。
test('MOTOR: 4方向の済み/未が表示される（done= の進捗）', async ({ page }) => {
  await gotoS30(page, {
    state: { state: 'RUNNING_CHECK' },
    opcheckStatus: { item: 'MOTOR', result: 'UNKNOWN', detail: 'done=FORWARD hold=NONE', next_screen: '' },
  })
  await expect(page.getByTestId('s30-motor-dirs')).toBeVisible()
  await expect(page.getByTestId('s30-motor-dir-FORWARD')).toContainText('済')
  await expect(page.getByTestId('s30-motor-dir-BACK')).toContainText('未')
  await expect(page.getByTestId('s30-motor-dir-LEFT')).toContainText('未')
  await expect(page.getByTestId('s30-motor-dir-RIGHT')).toContainText('未')
})

test('MOTOR: 進捗が届くと済みが増える', async ({ page }) => {
  await gotoS30(page, {
    state: { state: 'RUNNING_CHECK' },
    opcheckStatus: { item: 'MOTOR', result: 'UNKNOWN', detail: 'done= hold=NONE', next_screen: '' },
  })
  await expect(page.getByTestId('s30-motor-dir-FORWARD')).toContainText('未')
  await page.evaluate(() => window.__thSetTestOpcheckStatus(
    { item: 'MOTOR', result: 'UNKNOWN', detail: 'done=FORWARD,BACK hold=NONE', next_screen: '' }))
  await expect(page.getByTestId('s30-motor-dir-FORWARD')).toContainText('済')
  await expect(page.getByTestId('s30-motor-dir-BACK')).toContainText('済')
  await expect(page.getByTestId('s30-motor-dir-LEFT')).toContainText('未')
})

test('MOTOR: 押している間 motor_hold が繰り返し送られ、離すと NONE で止まる（以後増えない）', async ({ page }) => {
  await gotoS30(page, {
    state: { state: 'RUNNING_CHECK' },
    opcheckStatus: { item: 'MOTOR', result: 'UNKNOWN', detail: '', next_screen: '' },
  })
  const btn = page.getByTestId('s30-motor-forward')
  await expect(btn).toBeVisible()

  await firePointer(page, 's30-motor-forward', 'pointerdown')
  await page.waitForTimeout(550) // >= 5 repeats at 100ms

  let pubs = await motorHoldPublishes(page)
  expect(pubs.filter((p) => p.data === 'FORWARD').length).toBeGreaterThanOrEqual(4)
  expect(pubs.every((p) => p.data === 'FORWARD')).toBeTruthy() // 離すまで NONE は出ていない

  await firePointer(page, 's30-motor-forward', 'pointerup')
  await page.waitForTimeout(30)
  pubs = await motorHoldPublishes(page)
  expect(pubs[pubs.length - 1].data).toBe('NONE')
  expect(pubs.filter((p) => p.data === 'NONE').length).toBe(1) // ちょうど1回

  const countAfterRelease = pubs.length
  await page.waitForTimeout(400)
  pubs = await motorHoldPublishes(page)
  expect(pubs.length).toBe(countAfterRelease) // 離した後は増えない
})

test('MOTOR: pointercancel でも NONE で止まる', async ({ page }) => {
  await gotoS30(page, {
    state: { state: 'RUNNING_CHECK' },
    opcheckStatus: { item: 'MOTOR', result: 'UNKNOWN', detail: '', next_screen: '' },
  })
  await firePointer(page, 's30-motor-back', 'pointerdown')
  await page.waitForTimeout(150)
  await firePointer(page, 's30-motor-back', 'pointercancel')
  await page.waitForTimeout(30)
  const pubs = await motorHoldPublishes(page)
  expect(pubs[pubs.length - 1].data).toBe('NONE')
  const countAfter = pubs.length
  await page.waitForTimeout(300)
  expect((await motorHoldPublishes(page)).length).toBe(countAfter)
})

test('MOTOR: タブが隠れる（visibilitychange）と NONE で止まる', async ({ page }) => {
  await gotoS30(page, {
    state: { state: 'RUNNING_CHECK' },
    opcheckStatus: { item: 'MOTOR', result: 'UNKNOWN', detail: '', next_screen: '' },
  })
  await firePointer(page, 's30-motor-left', 'pointerdown')
  await page.waitForTimeout(150)
  await page.evaluate(() => {
    Object.defineProperty(document, 'visibilityState', { value: 'hidden', configurable: true })
    document.dispatchEvent(new Event('visibilitychange'))
  })
  await page.waitForTimeout(30)
  const pubs = await motorHoldPublishes(page)
  expect(pubs[pubs.length - 1].data).toBe('NONE')
})

test('MOTOR: 状態が LIST に離脱すると NONE で止まる（画面はまだ開いたまま）', async ({ page }) => {
  await gotoS30(page, {
    state: { state: 'RUNNING_CHECK' },
    opcheckStatus: { item: 'MOTOR', result: 'UNKNOWN', detail: '', next_screen: '' },
  })
  await firePointer(page, 's30-motor-right', 'pointerdown')
  await page.waitForTimeout(150)
  await page.evaluate(() => window.__thSetTestState({ state: 'LIST' }))
  await page.waitForTimeout(30)
  const pubs = await motorHoldPublishes(page)
  expect(pubs[pubs.length - 1].data).toBe('NONE')
  expect(pubs.filter((p) => p.data === 'NONE').length).toBe(1)
})

test('MOTOR: 画面が unmount される（モードが変わる）と NONE で止まる', async ({ page }) => {
  await gotoS30(page, {
    state: { state: 'RUNNING_CHECK' },
    opcheckStatus: { item: 'MOTOR', result: 'UNKNOWN', detail: '', next_screen: '' },
  })
  await firePointer(page, 's30-motor-forward', 'pointerdown')
  await page.waitForTimeout(150)
  await page.evaluate(() => window.__thSetTestState({ mode: 'IDLE', state: 'NONE' }))
  await page.waitForTimeout(30)
  const pubs = await motorHoldPublishes(page)
  expect(pubs[pubs.length - 1].data).toBe('NONE')
})

test('MOTOR: 非常停止(estop_hw)が押されると保持中でも NONE で止まる', async ({ page }) => {
  await gotoS30(page, {
    state: { state: 'RUNNING_CHECK', estop_hw: false },
    opcheckStatus: { item: 'MOTOR', result: 'UNKNOWN', detail: '', next_screen: '' },
  })
  await firePointer(page, 's30-motor-back', 'pointerdown')
  await page.waitForTimeout(150)
  await page.evaluate(() => window.__thSetTestState({ estop_hw: true }))
  await page.waitForTimeout(30)
  const pubs = await motorHoldPublishes(page)
  expect(pubs[pubs.length - 1].data).toBe('NONE')
})

// SM-3.1.2-116/-117（2026-10-04）: 実行中の項目から抜けられる・別の項目へ移れる。
test('実行中に「中断」を押すと ui.abort が実際に送られる', async ({ page }) => {
  await stubTrigger(page, { 'ui.abort': { accepted: true } })
  await gotoS30(page, {
    state: { state: 'RUNNING_CHECK' },
    opcheckStatus: { item: 'LIDAR', result: 'UNKNOWN', detail: '', next_screen: '' },
  })
  await page.getByTestId('s30-abort').click()
  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(calls.some((c) => c.trigger === 'ui.abort')).toBeTruthy()
})

test('実行中に一覧の別の項目を押すと、その項目で ui.check_item が送られ詳細が切り替わる', async ({ page }) => {
  await stubTrigger(page, { 'ui.check_item': { accepted: true } })
  await gotoS30(page, {
    state: { state: 'RUNNING_CHECK' },
    opcheckStatus: { item: 'LIDAR', result: 'UNKNOWN', detail: '', next_screen: '' },
  })
  await expect(page.getByTestId('s30-item-MOTOR')).toBeEnabled()
  await page.getByTestId('s30-item-MOTOR').click()
  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(calls.some((c) => c.trigger === 'ui.check_item' && c.argJson?.item === 'MOTOR')).toBeTruthy()
  // status.item がまだ LIDAR でも、押した MOTOR の詳細（ホールドボタン）が出る
  await expect(page.getByTestId('s30-motor-forward')).toBeVisible()
})

test('実行中の項目そのものを押しても何も送らない', async ({ page }) => {
  await stubTrigger(page, { 'ui.check_item': { accepted: true } })
  await gotoS30(page, {
    state: { state: 'RUNNING_CHECK' },
    opcheckStatus: { item: 'LIDAR', result: 'UNKNOWN', detail: '', next_screen: '' },
  })
  await page.getByTestId('s30-item-LIDAR').click()
  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(calls.some((c) => c.trigger === 'ui.check_item')).toBeFalsy()
})

test('一覧（LIST）では「中断」を出さない', async ({ page }) => {
  await gotoS30(page, { state: { state: 'LIST' } })
  await expect(page.getByTestId('s30-abort')).toHaveCount(0)
})

// ---- SG-C13: IMU の校正状態・LiDAR のライブスキャンと死角マスク（Spec-checks.md §2.4 #3・#4）----
const RUNNING = (item) => ({ item, result: 'UNKNOWN', detail: '', next_screen: '' })

test('IMU: 校正状態 sys/gyro/accel/mag が届いた値で出て、値が変わると追従する', async ({ page }) => {
  // 0b11_10_01_00 = sys3 / gyro2 / accel1 / mag0
  await gotoS30(page, { state: { state: 'RUNNING_CHECK' }, opcheckStatus: RUNNING('IMU'), imuCalib: 0b11100100 })
  await page.getByTestId('s30-item-IMU').click()
  await expect(page.getByTestId('s30-imu-sys')).toContainText('3/3')
  await expect(page.getByTestId('s30-imu-gyro')).toContainText('2/3')
  await expect(page.getByTestId('s30-imu-accel')).toContainText('1/3')
  await expect(page.getByTestId('s30-imu-mag')).toContainText('0/3')
  await page.evaluate(() => window.__thSetTestImuCalib(0xff))
  await expect(page.getByTestId('s30-imu-mag')).toContainText('3/3')
  await expect(page.getByTestId('s30-imu-gyro')).toContainText('3/3')
})

test('IMU: 校正状態がまだ届かないときはデータ待ち', async ({ page }) => {
  await gotoS30(page, { state: { state: 'RUNNING_CHECK' }, opcheckStatus: RUNNING('IMU') })
  await page.getByTestId('s30-item-IMU').click()
  await expect(page.getByTestId('s30-imu-calib-waiting')).toBeVisible()
})

// 前方 (0°) と左 (+90°) に 1 点ずつ。左 (80〜100°) が死角マスク。
const SCAN = { angle_min: 0, angle_increment: Math.PI / 2, range_min: 0.1, range_max: 10,
  ranges: [1.0, 1.0, null, null] }

test('LIDAR: 生スキャンが出て、機体の死角マスク（registry の値）が扇で重なる。マスク内の点は赤', async ({ page }) => {
  await gotoS30(page, {
    state: { state: 'RUNNING_CHECK' }, opcheckStatus: RUNNING('LIDAR'), scan: SCAN,
    tunables: { lidar_filter: { values: { blind_angle_ranges: [80, 100, 170, 190] } } },
  })
  await page.getByTestId('s30-item-LIDAR').click()
  await expect(page.getByTestId('s30-lidar-view')).toBeVisible()
  await expect(page.getByTestId('s30-lidar-mask-wedge')).toHaveCount(2)
  // 90° の点だけがマスクの内側（赤）、0° の点は外側
  await expect(page.locator('[data-testid="s30-lidar-view"] circle[fill="crimson"]')).toHaveCount(1)
})

test('LIDAR: マスクの値が変われば扇も変わる（画面に直書きしていない）', async ({ page }) => {
  await gotoS30(page, {
    state: { state: 'RUNNING_CHECK' }, opcheckStatus: RUNNING('LIDAR'), scan: SCAN,
    tunables: { lidar_filter: { values: { blind_angle_ranges: [40, 50] } } },
  })
  await page.getByTestId('s30-item-LIDAR').click()
  await expect(page.getByTestId('s30-lidar-mask-wedge')).toHaveCount(1)
  await expect(page.locator('[data-testid="s30-lidar-view"] circle[fill="crimson"]')).toHaveCount(0)
})

test('LIDAR: マスクを取得できないときは扇を出さず、取得できない旨を出す', async ({ page }) => {
  await gotoS30(page, { state: { state: 'RUNNING_CHECK' }, opcheckStatus: RUNNING('LIDAR'), scan: SCAN })
  await page.getByTestId('s30-item-LIDAR').click()
  await expect(page.getByTestId('s30-lidar-mask-unknown')).toBeVisible()
  await expect(page.getByTestId('s30-lidar-mask-wedge')).toHaveCount(0)
})
