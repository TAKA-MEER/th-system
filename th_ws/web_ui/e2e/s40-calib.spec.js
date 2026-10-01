// e2e/s40-calib.spec.js — S-40 校正 (WP-MAINT-03). 本番の送信経路を縛る:
//   - 「開始」が ui.calib_item、「次へ」が ui.calib_next、「中断」が ui.abort を
//     /system/trigger へ実際に送る（item 付き）
//   - 実測値の「確認」「検証結果を送る」が /calib/submit を、「もう一度走る」が /calib/start を、
//     「この値に戻す」が /calib/rollback を実際に送る（window.__thCalibCalls に残る）
//   - S3 の「次へ」は /calib/status が PREVIEW_OK のときだけ押せる（sane でなければ進めない）
// 2026-09-20 に「本番の送信を殺しても全部緑」の e2e があった。ここは送ったリクエストの中身
// （item・measured・generation）まで確かめる。
import { test, expect } from '@playwright/test'
import { stubTrigger } from './helpers.js'

async function gotoS40(page, { state, calibStatus, calibStubs } = {}) {
  await page.addInitScript(({ s, cs, stubs }) => {
    window.__thTestState = s
    window.__thTestScreen = 'S40'
    if (cs) window.__thTestCalibStatus = cs
    if (stubs) window.__thTestCalib = stubs
  }, { s: { mode: 'CALIB', state: 'LIST', ...state }, cs: calibStatus, stubs: calibStubs })
  await page.goto('/')
}

const setState = (page, patch) => page.evaluate((p) => window.__thSetTestState(p), patch)
const setStatus = (page, v) => page.evaluate((x) => window.__thSetTestCalibStatus(x), v)
const triggerCalls = (page) => page.evaluate(() => window.__thTriggerCalls ?? [])
const calibCalls = (page) => page.evaluate(() => window.__thCalibCalls ?? [])

const DETAIL = {
  commanded: 1.0, current: { wheel_radius_scale: 1.0 }, progress: 0.4, warnings: [],
  last_calibrated: { LINEAR: '2026-10-01T10:00:00', ROTATION: '', IMU: '' },
  history: {
    LINEAR: [{ generation: 1, values: { wheel_radius_scale: 0.98 }, calibrated_at: '2026-09-30T09:00:00' }],
    ROTATION: [], IMU: [],
  },
}
const status = (over = {}) => ({ item: 'LINEAR', step: 'S3', result: 'WAIT_MEASURED',
  preview_before: '', preview_after: '', detail: DETAIL, ...over })

test('一覧: 直進・旋回・IMU に「開始」があり、LiDAR 死角は開始できない。旋回に「先に直進校正を」', async ({ page }) => {
  await gotoS40(page, { calibStatus: status({ result: 'IDLE', item: '' }) })
  for (const it of ['LINEAR', 'ROTATION', 'IMU']) {
    await expect(page.getByTestId(`s40-start-${it}`)).toBeEnabled()
  }
  await expect(page.getByTestId('s40-start-BLIND')).toBeDisabled()
  await expect(page.getByTestId('s40-rotation-first')).toContainText('先に直進校正')
  await expect(page.getByTestId('s40-item-LINEAR')).toContainText('2026-10-01T10:00:00')
  await expect(page.getByTestId('s40-item-ROTATION')).toContainText('未校正')
  // LIST では中断できない（走っていない）
  await expect(page.getByTestId('s40-abort')).toBeDisabled()
})

test('「開始」が ui.calib_item を item 付きで送る', async ({ page }) => {
  await stubTrigger(page, { 'ui.calib_item': { accepted: true } })
  await gotoS40(page)
  await page.getByTestId('s40-start-ROTATION').click()
  const calls = await triggerCalls(page)
  expect(calls.some((c) => c.trigger === 'ui.calib_item' && c.argJson?.item === 'ROTATION')).toBeTruthy()
})

test('拒否されたら理由を出す', async ({ page }) => {
  await stubTrigger(page, { 'ui.calib_item': { accepted: false, reject_reason_key: 'not_allowed' } })
  await gotoS40(page)
  await page.getByTestId('s40-start-LINEAR').click()
  await expect(page.getByTestId('s40-err')).toContainText('今の状態ではこの操作はできません')
})

test('Step 1: 案内と「走り出す」が ui.calib_next を item 付きで送る。Step 表示は 1/4', async ({ page }) => {
  await stubTrigger(page, { 'ui.calib_next': { accepted: true } })
  await gotoS40(page, { state: { state: 'S1' }, calibStatus: status({ result: 'GUIDE', step: 'S1' }) })
  await expect(page.getByTestId('s40-step-title')).toHaveText('Step 1/4')
  await expect(page.getByTestId('s40-guide')).toContainText('1 m')
  await page.getByTestId('s40-next').click()
  const calls = await triggerCalls(page)
  expect(calls.some((c) => c.trigger === 'ui.calib_next' && c.argJson?.item === 'LINEAR')).toBeTruthy()
})

test('旋回の案内に、直進校正が無い・古い警告が出る', async ({ page }) => {
  await gotoS40(page, {
    state: { state: 'S1' },
    calibStatus: status({ item: 'ROTATION', result: 'GUIDE', step: 'S1',
      detail: { ...DETAIL, warnings: ['linear_not_calibrated'] } }),
  })
  await expect(page.getByTestId('s40-warning')).toContainText('先に直進校正を実施してください')
})

test('Step 2: 走行中の進捗バーと、「中断」が ui.abort を送る', async ({ page }) => {
  await stubTrigger(page, { 'ui.abort': { accepted: true } })
  await gotoS40(page, { state: { state: 'S2' }, calibStatus: status({ result: 'RUNNING', step: 'S2' }) })
  await expect(page.getByTestId('s40-step-title')).toHaveText('Step 2/4')
  await expect(page.getByTestId('s40-run-bar')).toHaveAttribute('aria-valuenow', '40')
  await page.getByTestId('s40-abort').click()
  const calls = await triggerCalls(page)
  expect(calls.some((c) => c.trigger === 'ui.abort' && c.argJson?.item === 'LINEAR')).toBeTruthy()
})

test('Step 3: 実測値の「確認」が /calib/submit を item・measured 付きで送る', async ({ page }) => {
  await gotoS40(page, {
    state: { state: 'S3' }, calibStatus: status(),
    calibStubs: { submit: { success: true, preview_before: '{}', preview_after: '{}' } },
  })
  await expect(page.getByTestId('s40-step-title')).toHaveText('Step 3/4')
  await expect(page.getByTestId('s40-submit')).toBeDisabled()          // 未入力
  await page.getByTestId('s40-measured').fill('0.97')
  await page.getByTestId('s40-submit').click()
  const calls = await calibCalls(page)
  expect(calls).toContainEqual({ service: 'submit', item: 'LINEAR', measured: 0.97, arg_json: '{}' })
})

test('Step 3: 不正な入力（0・負）では送れない（数値欄は文字を受け付けない）', async ({ page }) => {
  await gotoS40(page, { state: { state: 'S3' }, calibStatus: status() })
  for (const bad of ['0', '-1']) {
    await page.getByTestId('s40-measured').fill(bad)
    await expect(page.getByTestId('s40-submit')).toBeDisabled()
  }
  expect(await calibCalls(page)).toEqual([])
})

test('Step 3: PREVIEW_OK になるまで「次へ」は押せず、押すと ui.calib_next が送られる。プレビューが出る', async ({ page }) => {
  await stubTrigger(page, { 'ui.calib_next': { accepted: true } })
  await gotoS40(page, { state: { state: 'S3' }, calibStatus: status() })
  await expect(page.getByTestId('s40-next')).toBeDisabled()
  await expect(page.getByTestId('s40-preview')).toHaveCount(0)

  await setStatus(page, status({
    result: 'PREVIEW_INSANE', preview_before: '{"wheel_radius_scale": 1.0}',
    preview_after: '{"wheel_radius_scale": 0.85}' }))
  await expect(page.getByTestId('s40-insane')).toContainText('補正が大きすぎます')
  await expect(page.getByTestId('s40-next')).toBeDisabled()

  await setStatus(page, status({
    result: 'PREVIEW_OK', preview_before: '{"wheel_radius_scale": 1.0}',
    preview_after: '{"wheel_radius_scale": 0.97}' }))
  await expect(page.getByTestId('s40-preview-before')).toHaveText('wheel_radius_scale=1')
  await expect(page.getByTestId('s40-preview-after')).toHaveText('wheel_radius_scale=0.97')
  await expect(page.getByTestId('s40-next')).toBeEnabled()
  await page.getByTestId('s40-next').click()
  const calls = await triggerCalls(page)
  expect(calls.some((c) => c.trigger === 'ui.calib_next' && c.argJson?.item === 'LINEAR')).toBeTruthy()
})

test('別項目の PREVIEW_OK では「次へ」は押せない', async ({ page }) => {
  await gotoS40(page, { state: { state: 'S3' }, calibStatus: status({ item: 'ROTATION', result: 'PREVIEW_OK' }) })
  // 実行中の項目は status.item（ROTATION）。LINEAR の入力欄ではなく ROTATION の画面になる
  await expect(page.getByTestId('s40-next')).toBeEnabled()
  await setStatus(page, status({ item: 'ROTATION', result: 'WAIT_MEASURED' }))
  await expect(page.getByTestId('s40-next')).toBeDisabled()
})

test('Step 4: 検証走行中は進捗、終わったら「検証結果を送る」が /calib/submit を送る', async ({ page }) => {
  await gotoS40(page, {
    state: { state: 'S4' }, calibStatus: status({ result: 'VERIFY_RUNNING', step: 'S4' }),
    calibStubs: { submit: { success: true, preview_before: '{}', preview_after: '{}' } },
  })
  await expect(page.getByTestId('s40-step-title')).toHaveText('Step 4/4')
  await expect(page.getByTestId('s40-run-bar')).toBeVisible()
  await expect(page.getByTestId('s40-verify-submit')).toHaveCount(0)

  await setStatus(page, status({ result: 'WAIT_VERIFY', step: 'S4' }))
  await page.getByTestId('s40-measured').fill('1.005')
  await page.getByTestId('s40-verify-submit').click()
  expect(await calibCalls(page)).toContainEqual(
    { service: 'submit', item: 'LINEAR', measured: 1.005, arg_json: '{}' })
})

test('検証 NG で S2 に戻ったら理由と「もう一度走る」。押すと /calib/start が送られる', async ({ page }) => {
  await gotoS40(page, {
    state: { state: 'S2' },
    calibStatus: status({ result: 'RETRY_WAIT', step: 'S2',
      detail: { ...DETAIL, phase: 'RETRY_WAIT', reason: 'error_exceeds_tolerance:0.2>0.05' } }),
    calibStubs: { start: { started: false, message: '開始できません' } },
  })
  await expect(page.getByTestId('s40-retry-title')).toContainText('適用前に戻しました')
  await expect(page.getByTestId('s40-retry-title')).toContainText('許容範囲')
  await page.getByTestId('s40-retry').click()
  expect(await calibCalls(page)).toContainEqual({ service: 'start', item: 'LINEAR' })
  await expect(page.getByTestId('s40-retry-err')).toContainText('開始できません')
})

test('IMU: 進捗バーが 4 要素を出し、完了で「確定する」が ui.calib_next を送る（走らせない）', async ({ page }) => {
  await stubTrigger(page, { 'ui.calib_next': { accepted: true } })
  const imuStatus = (result, imu) => status({
    item: 'IMU', result, step: 'S2', detail: { ...DETAIL, imu } })
  await gotoS40(page, {
    state: { state: 'S2' },
    calibStatus: imuStatus('RUNNING', { sys: 3, gyro: 3, accel: 0, mag: 0 }),
  })
  await expect(page.getByTestId('s40-imu-bar')).toHaveAttribute('aria-valuenow', '50')
  await expect(page.getByTestId('s40-imu-gyro')).toContainText('3/3')
  await expect(page.getByTestId('s40-imu-mag')).toContainText('0/3')
  await expect(page.getByTestId('s40-step-title')).toHaveCount(0)   // IMU は Step 表示ではなく進捗バー

  await setState(page, { state: 'S3' })
  await setStatus(page, imuStatus('PREVIEW_OK', { sys: 3, gyro: 3, accel: 3, mag: 3 }))
  await expect(page.getByTestId('s40-imu-done')).toBeVisible()
  await page.getByTestId('s40-next').click()
  const calls = await triggerCalls(page)
  expect(calls.some((c) => c.trigger === 'ui.calib_next' && c.argJson?.item === 'IMU')).toBeTruthy()
})

test('履歴: 「この値に戻す」が /calib/rollback を item・generation 付きで送る（LIST のときだけ押せる）', async ({ page }) => {
  await gotoS40(page, {
    calibStatus: status({ result: 'IDLE', item: '' }),
    calibStubs: { rollback: { success: true } },
  })
  await page.getByTestId('s40-tab-history').click()
  await expect(page.getByTestId('s40-history-LINEAR')).toContainText('1 世代前: wheel_radius_scale=0.98')
  await expect(page.getByTestId('s40-history-ROTATION')).toContainText('履歴はありません')
  await page.getByTestId('s40-rollback-LINEAR-1').click()
  expect(await calibCalls(page)).toContainEqual({ service: 'rollback', item: 'LINEAR', generation: 1 })
  await expect(page.getByTestId('s40-rollback-msg')).toContainText('戻しました')

  await setState(page, { state: 'S2' })
  await expect(page.getByTestId('s40-rollback-LINEAR-1')).toBeDisabled()
})

test('ロールバック失敗の応答では失敗を表示する', async ({ page }) => {
  await gotoS40(page, { calibStatus: status({ result: 'IDLE', item: '' }), calibStubs: { rollback: { success: false } } })
  await page.getByTestId('s40-tab-history').click()
  await page.getByTestId('s40-rollback-LINEAR-1').click()
  await expect(page.getByTestId('s40-rollback-msg')).toContainText('戻せませんでした')
})

test('LIST の「始業点検へ戻る」が ui.enter_mode{mode:OPCHECK} を送る。確定の通知が出る', async ({ page }) => {
  await stubTrigger(page, { 'ui.enter_mode': { accepted: true } })
  await gotoS40(page, { calibStatus: status({ result: 'IDLE', item: '' }) })
  await setStatus(page, status({ result: 'COMMITTED', item: '' }))
  await expect(page.getByTestId('s40-committed')).toBeVisible()
  await page.getByTestId('s40-to-opcheck').click()
  const calls = await triggerCalls(page)
  expect(calls.some((c) => c.trigger === 'ui.enter_mode' && c.argJson?.mode === 'OPCHECK')).toBeTruthy()
})
