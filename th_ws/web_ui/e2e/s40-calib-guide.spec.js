// e2e/s40-calib-guide.spec.js — S-30「校正へ」→ S-40 誘導（計画書 §6 #11）。
// 本番の送信経路・本番の順番を縛る（1b-8・SG-B5 で作り直し。以前は
// RUNNING_CHECK＋判定済みという本番では起きない安定状態を手で作っていた。
// 本番は判定と同時に FSM が LIST へ戻る（T-OPC-02/04）ので、ボタンは LIST の
// 行に出ていなければならない）:
//   1. S-30 を RUNNING_CHECK＋判定前（UNKNOWN）で開く
//   2. 判定が届く（/opcheck/status の到着を模擬）
//   3. FSM が LIST へ戻る（T-OPC-02/04 を模擬）
//   4. LIST の行の「校正へ」が ui.enter_mode{mode:CALIB} を実際に送る
//   5. CALIB/LIST に入ると受け渡し（screens/calibGuide.js）を 1 回だけ読んで
//     該当項目（LIDAR→BLIND / IMU→IMU）を強調し、案内を出す。自動では始まらない
//     （開始は人が押す。Spec.md SD-8）
//   - enter_mode 拒否では受け渡しが残らない／別経路（S-01 等）では何も出ない
//
// 画面遷移はモード導出（screens/screenRouting.js）で起きる: __thTestScreen は
// 初期表示だけを決め、その後の mode 変更（__thSetTestState）は Screens() が
// 追う。S-30 → S-40 は同一ページ内の遷移なので、モジュール内の受け渡しが残る。
import { test, expect } from '@playwright/test'
import { stubTrigger } from './helpers.js'

// S-30 を判定前の RUNNING_CHECK で開く（本番の項目実行中の状態）。
async function gotoS30Running(page, item) {
  await page.addInitScript(({ it }) => {
    window.__thTestState = { mode: 'OPCHECK', state: 'RUNNING_CHECK' }
    window.__thTestScreen = 'S30'
    window.__thTestOpcheckAnswer = () => ({ accepted: true })
    window.__thTestOpcheckStatus = { item: it, result: 'UNKNOWN', detail: '', next_screen: '' }
    window.__thTestCalibStatus = {
      item: '', step: '', result: 'IDLE', preview_before: '', preview_after: '', detail: {},
    }
  }, { it: item })
  await page.goto('/')
}

async function gotoS40Direct(page) {
  await page.addInitScript(() => {
    window.__thTestState = { mode: 'CALIB', state: 'LIST' }
    window.__thTestScreen = 'S40'
    window.__thTestCalibStatus = {
      item: '', step: '', result: 'IDLE', preview_before: '', preview_after: '', detail: {},
    }
  })
  await page.goto('/')
}

const setState = (page, patch) => page.evaluate((p) => window.__thSetTestState(p), patch)
const setOpcheckStatus = (page, os) => page.evaluate((o) => window.__thSetTestOpcheckStatus(o), os)
const triggerCalls = (page) => page.evaluate(() => window.__thTriggerCalls ?? [])

// 本番の順番: 判定の到着 → FSM が LIST へ戻る → LIST の行のボタン。
async function verdictThenBackToList(page, opcheckStatus) {
  await setOpcheckStatus(page, opcheckStatus)
  await setState(page, { state: 'LIST' })
}

async function clickGotoCalibInList(page, item) {
  const btn = page.getByTestId(`s30-goto-calib-${item}`)
  await expect(btn).toBeVisible()
  await btn.click()
  await page.waitForFunction(
    () => (window.__thTriggerCalls ?? []).some((c) => c.trigger === 'ui.enter_mode'),
  )
}

// S-30 側の送信（ui.enter_mode{mode:CALIB}）を確かめてから、FSM が遷った
// ものとして CALIB/LIST へ進める（本番は T-OPC-07 で遷る）。
async function enterCalibList(page) {
  const calls = await triggerCalls(page)
  expect(calls.some((c) => c.trigger === 'ui.enter_mode' && c.argJson?.mode === 'CALIB')).toBeTruthy()
  await setState(page, { mode: 'CALIB', state: 'LIST' })
  await expect(page.getByTestId('s40-tab-items')).toBeVisible()
}

test('LIDAR が NG → LIST の行の「校正へ」→ S-40 で BLIND が強調され案内が出る（自動では始まらない）', async ({ page }) => {
  await stubTrigger(page, { 'ui.enter_mode': { accepted: true } })
  await gotoS30Running(page, 'LIDAR')
  await verdictThenBackToList(page, { item: 'LIDAR', result: 'NG', detail: '', next_screen: 'lidar_calib' })

  await clickGotoCalibInList(page, 'LIDAR')
  await enterCalibList(page)

  await expect(page.getByTestId('s40-calib-guide')).toContainText('始業点検')
  await expect(page.getByTestId('s40-calib-guide')).toContainText('LiDAR')
  await expect(page.getByTestId('s40-calib-guide')).toContainText('NGでした')
  await expect(page.getByTestId('s40-calib-guide')).toContainText('開始')
  await expect(page.getByTestId('s40-item-BLIND')).toHaveClass(/guided/)
  await expect(page.getByTestId('s40-item-IMU')).not.toHaveClass(/guided/)
  await expect(page.getByTestId('s40-item-LINEAR')).not.toHaveClass(/guided/)
  // 自動で始まらない: LIST のままで「次へ」は無い
  await expect(page.getByTestId('s40-next')).toHaveCount(0)
  await expect(page.getByTestId('s40-start-BLIND')).toBeEnabled()
})

test('IMU が要確認（WARN）→ LIST の行の「校正へ」→ S-40 で IMU が強調される', async ({ page }) => {
  await stubTrigger(page, { 'ui.enter_mode': { accepted: true } })
  await gotoS30Running(page, 'IMU')
  await verdictThenBackToList(page, { item: 'IMU', result: 'WARN', detail: '', next_screen: 'imu_calib' })

  await clickGotoCalibInList(page, 'IMU')
  await enterCalibList(page)

  await expect(page.getByTestId('s40-calib-guide')).toContainText('要確認でした')
  await expect(page.getByTestId('s40-item-IMU')).toHaveClass(/guided/)
  await expect(page.getByTestId('s40-item-BLIND')).not.toHaveClass(/guided/)
})

test('IMU が NG → LIST の行の「校正へ」が出る（repair 行きではなく校正へ）', async ({ page }) => {
  await stubTrigger(page, { 'ui.enter_mode': { accepted: true } })
  await gotoS30Running(page, 'IMU')
  await verdictThenBackToList(page, { item: 'IMU', result: 'NG', detail: 'bias_too_large', next_screen: 'imu_calib' })

  await clickGotoCalibInList(page, 'IMU')
  await enterCalibList(page)

  await expect(page.getByTestId('s40-calib-guide')).toContainText('始業点検')
  await expect(page.getByTestId('s40-item-IMU')).toHaveClass(/guided/)
})

test('判定前（UNKNOWN）の行には「校正へ」は出ない', async ({ page }) => {
  await stubTrigger(page, { 'ui.enter_mode': { accepted: true } })
  await gotoS30Running(page, 'LIDAR')
  await setState(page, { state: 'LIST' })
  await expect(page.getByTestId('s30-goto-calib-LIDAR')).toHaveCount(0)
})

test('enter_mode が拒否されたら誘導は残らない（別経路で入っても出ない）', async ({ page }) => {
  await stubTrigger(page, { 'ui.enter_mode': { accepted: false, reject_reason_key: 'not_allowed' } })
  await gotoS30Running(page, 'LIDAR')
  await verdictThenBackToList(page, { item: 'LIDAR', result: 'NG', detail: '', next_screen: 'lidar_calib' })

  await clickGotoCalibInList(page, 'LIDAR')
  // 送ったが拒否された
  const calls = await triggerCalls(page)
  expect(calls.some((c) => c.trigger === 'ui.enter_mode' && c.argJson?.mode === 'CALIB')).toBeTruthy()
  // S-01 など別経路で CALIB に入ったものとして遷る → 案内は出ない
  await setState(page, { mode: 'CALIB', state: 'LIST' })
  await expect(page.getByTestId('s40-tab-items')).toBeVisible()
  await expect(page.getByTestId('s40-calib-guide')).toHaveCount(0)
  await expect(page.locator('.s40-row.guided')).toHaveCount(0)
})

test('S-30 以外から S-40 に入ると何も出ない', async ({ page }) => {
  await gotoS40Direct(page)
  await expect(page.getByTestId('s40-tab-items')).toBeVisible()
  await expect(page.getByTestId('s40-calib-guide')).toHaveCount(0)
  await expect(page.locator('.s40-row.guided')).toHaveCount(0)
})

test('誘導の案内はウィザードに入ると畳まれ、LIST に戻っても出ない（1 回だけ）', async ({ page }) => {
  await stubTrigger(page, {
    'ui.enter_mode': { accepted: true },
    'ui.calib_item': { accepted: true },
  })
  await gotoS30Running(page, 'LIDAR')
  await verdictThenBackToList(page, { item: 'LIDAR', result: 'NG', detail: '', next_screen: 'lidar_calib' })

  await clickGotoCalibInList(page, 'LIDAR')
  await enterCalibList(page)
  await expect(page.getByTestId('s40-calib-guide')).toBeVisible()

  // 誘導どおり BLIND を人が開始する
  await page.getByTestId('s40-start-BLIND').click()
  await setState(page, { state: 'S1' })
  await page.evaluate(() => window.__thSetTestCalibStatus({
    item: 'BLIND', step: 'S1', result: 'GUIDE', preview_before: '', preview_after: '', detail: {},
  }))
  await expect(page.getByTestId('s40-calib-guide')).toHaveCount(0)

  // 中断して LIST に戻ってももう出ない（受け渡しは消費済み）
  await setState(page, { state: 'LIST' })
  await page.evaluate(() => window.__thSetTestCalibStatus({
    item: '', step: '', result: 'IDLE', preview_before: '', preview_after: '', detail: {},
  }))
  await expect(page.getByTestId('s40-tab-items')).toBeVisible()
  await expect(page.getByTestId('s40-calib-guide')).toHaveCount(0)
  await expect(page.locator('.s40-row.guided')).toHaveCount(0)
})
