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

async function gotoS40(page, { state, calibStatus, calibStubs, scan } = {}) {
  await page.addInitScript(({ s, cs, stubs, sc }) => {
    window.__thTestState = s
    window.__thTestScreen = 'S40'
    if (cs) window.__thTestCalibStatus = cs
    if (stubs) window.__thTestCalib = stubs
    if (sc) window.__thTestScan = sc
  }, { s: { mode: 'CALIB', state: 'LIST', ...state }, cs: calibStatus, stubs: calibStubs, sc: scan })
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

test('一覧: 直進・旋回・IMU・LiDAR 死角のすべてに「開始」がある。旋回に「先に直進校正を」', async ({ page }) => {
  await gotoS40(page, { calibStatus: status({ result: 'IDLE', item: '' }) })
  for (const it of ['LINEAR', 'ROTATION', 'IMU', 'BLIND']) {
    await expect(page.getByTestId(`s40-start-${it}`)).toBeEnabled()
  }
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
  // 二段階アーム（Spec-webui.md §6）: 1 回目は「本当に戻す？」に変わるだけで送らない
  await page.getByTestId('s40-rollback-LINEAR-1').click()
  await expect(page.getByTestId('s40-rollback-LINEAR-1')).toHaveText('本当に戻す？')
  expect((await calibCalls(page)).some((c) => c.service === 'rollback')).toBe(false)
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

test('PREVIEW_INSANE が続く間は理由が出続け、測り直して状態が変わったら消える', async ({ page }) => {
  await gotoS40(page, { state: { state: 'S3' }, calibStatus: status({ result: 'PREVIEW_INSANE' }) })
  await expect(page.getByTestId('s40-insane')).toBeVisible()
  // calib_runner は 0.5s ごとに同じ PREVIEW_INSANE を再配信する。再配信でも消えない。
  for (let i = 0; i < 4; i += 1) {
    await page.waitForTimeout(300)
    await setStatus(page, status({ result: 'PREVIEW_INSANE' }))
    await expect(page.getByTestId('s40-insane')).toBeVisible()
  }
  await expect(page.getByTestId('s40-next')).toBeDisabled()
  await setStatus(page, status({ result: 'WAIT_MEASURED' }))
  await expect(page.getByTestId('s40-insane')).toHaveCount(0)
})


// ════════════════════════════════════════════════════════════
// BLIND（LiDAR 死角）— ライブスキャン上をなぞって選ぶ
// ════════════════════════════════════════════════════════════
const SCAN = {
  angle_min: -Math.PI, angle_increment: Math.PI / 180, range_min: 0.05, range_max: 12,
  ranges: Array.from({ length: 360 }, (_, i) => (i >= 170 && i <= 190 ? 0.3 : 3.0)),
}
const BLIND_DETAIL = {
  current: { blind_angle_ranges: [45.9, 61.5, -59.5, -38.9] },
  blind: { limits: { max_sector_deg: 30, max_total_deg: 90, max_sectors: 8 } },
  last_calibrated: {}, history: {}, warnings: [],
}
const blindStatus = (over = {}) => ({ item: 'BLIND', step: 'S2', result: 'RUNNING',
  preview_before: '', preview_after: '', detail: BLIND_DETAIL, ...over })

// 前方 0°・左 +90° の角度 → 画面座標（キャンバスの bounding box から）。
async function pointAt(page, deg, radius = 120) {
  const box = await page.getByTestId('s40-blind-canvas').boundingBox()
  const k = box.width / 400
  const a = (deg * Math.PI) / 180
  return { x: box.x + box.width / 2 - radius * k * Math.sin(a), y: box.y + box.height / 2 - radius * k * Math.cos(a) }
}
async function dragRange(page, from, to) {
  const a = await pointAt(page, from)
  const b = await pointAt(page, to)
  await page.mouse.move(a.x, a.y)
  await page.mouse.down()
  const steps = 6
  for (let i = 1; i <= steps; i += 1) {
    const d = from + ((to - from) * i) / steps
    const p = await pointAt(page, d)
    await page.mouse.move(p.x, p.y)
  }
  await page.mouse.move(b.x, b.y)
  await page.mouse.up()
}
const lastSubmit = async (page) => {
  const calls = (await calibCalls(page)).filter((c) => c.service === 'submit')
  return calls[calls.length - 1]
}

test('BLIND: 「開始」が ui.calib_item を item=BLIND で送る。案内は「ロボットは動きません」', async ({ page }) => {
  await stubTrigger(page, { 'ui.calib_item': { accepted: true } })
  await gotoS40(page, { calibStatus: status({ result: 'IDLE', item: '' }) })
  await page.getByTestId('s40-start-BLIND').click()
  const calls = await triggerCalls(page)
  expect(calls.some((c) => c.trigger === 'ui.calib_item' && c.argJson?.item === 'BLIND')).toBeTruthy()
  await setState(page, { state: 'S1' })
  await setStatus(page, blindStatus({ result: 'GUIDE', step: 'S1' }))
  await expect(page.getByTestId('s40-guide')).toContainText('ロボットは動きません')
})

test('BLIND: スキャン上をなぞると選択に足され、「確認」が選んだ範囲を /calib/submit で送る', async ({ page }) => {
  await gotoS40(page, {
    state: { state: 'S2' }, calibStatus: blindStatus(), scan: SCAN,
    calibStubs: { submit: { success: true, preview_before: '{}', preview_after: '{}' } },
  })
  await expect(page.getByTestId('s40-blind-canvas')).toBeVisible()
  // 登録済み 2 区間が選択の初期値
  await expect(page.getByTestId('s40-blind-range-0')).toContainText('45.9〜61.5°')
  await expect(page.getByTestId('s40-blind-range-1')).toContainText('-59.5〜-38.9°')
  await dragRange(page, 150, 170)          // 左後ろ寄り 20° をなぞる
  await expect(page.getByTestId('s40-blind-range-2')).toBeVisible()
  await page.getByTestId('s40-blind-submit').click()
  const call = await lastSubmit(page)
  expect(call.item).toBe('BLIND')
  const body = JSON.parse(call.arg_json)
  expect(body.ranges).toHaveLength(3)
  const added = body.ranges[2]
  expect(Math.abs(added[0] - 150)).toBeLessThan(4)
  expect(Math.abs(added[1] - 170)).toBeLessThan(4)
  expect(body.ranges[0]).toEqual([45.9, 61.5])
})

test('BLIND: 逆向き（時計回り）になぞっても同じ区間になる。±180° をまたぐなぞりも送れる', async ({ page }) => {
  await gotoS40(page, {
    state: { state: 'S2' }, calibStatus: blindStatus({ detail: { ...BLIND_DETAIL, current: { blind_angle_ranges: [] } } }),
    scan: SCAN, calibStubs: { submit: { success: true } },
  })
  await dragRange(page, 170, 150)
  await dragRange(page, 175, -175)         // 後ろ（180°）をまたぐ 10°
  await page.getByTestId('s40-blind-submit').click()
  const body = JSON.parse((await lastSubmit(page)).arg_json)
  expect(body.ranges).toHaveLength(2)
  expect(Math.abs(body.ranges[0][0] - 150)).toBeLessThan(4)
  expect(Math.abs(body.ranges[0][1] - 170)).toBeLessThan(4)
  expect(body.ranges[1][0]).toBeGreaterThan(165)
  expect(body.ranges[1][1]).toBeLessThan(-165)
})

test('BLIND: 幅の上限を超える選択は「確認」を押せず、何も送られない', async ({ page }) => {
  await gotoS40(page, {
    state: { state: 'S2' }, calibStatus: blindStatus({ detail: { ...BLIND_DETAIL, current: { blind_angle_ranges: [] } } }),
    scan: SCAN, calibStubs: { submit: { success: true } },
  })
  await dragRange(page, 10, 60)            // 50° > 1 区間 30°
  await expect(page.getByTestId('s40-blind-problem')).toContainText('広すぎる区間')
  await expect(page.getByTestId('s40-blind-submit')).toBeDisabled()
  expect((await calibCalls(page)).filter((c) => c.service === 'submit')).toEqual([])
  // 削除すれば押せる（空 = マスクを全部外す選択）
  await page.getByTestId('s40-blind-delete-0').click()
  await expect(page.getByTestId('s40-blind-problem')).toHaveCount(0)
  await expect(page.getByTestId('s40-blind-submit')).toBeEnabled()
})

test('BLIND: 登録済みの区間を削除して送れる。「登録済みに戻す」で初期値へ戻る', async ({ page }) => {
  await gotoS40(page, {
    state: { state: 'S2' }, calibStatus: blindStatus(), scan: SCAN, calibStubs: { submit: { success: true } },
  })
  await page.getByTestId('s40-blind-delete-0').click()
  await expect(page.getByTestId('s40-blind-range-0')).toContainText('-59.5〜-38.9°')
  await page.getByTestId('s40-blind-submit').click()
  expect(JSON.parse((await lastSubmit(page)).arg_json).ranges).toEqual([[-59.5, -38.9]])
  await page.getByTestId('s40-blind-reset').click()
  await expect(page.getByTestId('s40-blind-range-1')).toBeVisible()
})

test('BLIND: スキャンが届いていないときは確認できない', async ({ page }) => {
  await gotoS40(page, { state: { state: 'S2' }, calibStatus: blindStatus() })
  await expect(page.getByTestId('s40-blind-no-scan')).toBeVisible()
  await expect(page.getByTestId('s40-blind-submit')).toBeDisabled()
})

test('BLIND: 機体が上限超過と返したら理由が出る（PREVIEW_INSANE）。S3 でプレビューが出て、次へが ui.calib_next', async ({ page }) => {
  await stubTrigger(page, { 'ui.calib_next': { accepted: true } })
  await gotoS40(page, {
    state: { state: 'S2' }, scan: SCAN,
    calibStatus: blindStatus({ result: 'PREVIEW_INSANE',
      preview_after: '{"blind_angle_ranges":[],"masked_points":0,"total_deg":0,"reason":"total_too_wide:95.0>90.0"}' }),
  })
  await expect(page.getByTestId('s40-insane')).toContainText('総幅が広すぎます')
  await setState(page, { state: 'S3' })
  await setStatus(page, blindStatus({ step: 'S3', result: 'PREVIEW_OK',
    preview_after: '{"blind_angle_ranges":[-12,12],"masked_points":21,"total_deg":24,"reason":""}' }))
  await expect(page.getByTestId('s40-preview-after')).toContainText('消える点: 21 点')
  await expect(page.getByTestId('s40-preview-after')).toContainText('総幅 24°')
  await page.getByTestId('s40-next').click()
  const calls = await triggerCalls(page)
  expect(calls.some((c) => c.trigger === 'ui.calib_next' && c.argJson?.item === 'BLIND')).toBeTruthy()
})

test('BLIND: S3 は PREVIEW_OK になるまで「次へ」が出ない。S4 は検証中の表示のみ（実測値の入力は無い）', async ({ page }) => {
  await gotoS40(page, { state: { state: 'S3' }, scan: SCAN, calibStatus: blindStatus({ step: 'S3', result: 'WAIT_MEASURED' }) })
  await expect(page.getByTestId('s40-next')).toHaveCount(0)
  await setState(page, { state: 'S4' })
  await setStatus(page, blindStatus({ step: 'S4', result: 'VERIFY_RUNNING' }))
  await expect(page.getByTestId('s40-verifying')).toContainText('検証中')
  await expect(page.getByTestId('s40-verify-submit')).toHaveCount(0)
  await expect(page.getByTestId('s40-blind-canvas')).toHaveCount(0)
})

test('BLIND: 検証 NG で S2 に戻ったら理由が出て、選び直せる', async ({ page }) => {
  await gotoS40(page, {
    state: { state: 'S2' }, scan: SCAN,
    calibStatus: blindStatus({ result: 'RETRY_WAIT',
      detail: { ...BLIND_DETAIL, phase: 'RETRY_WAIT', reason: 'offset_exceeds_tolerance:20.0>5.0' } }),
  })
  await expect(page.getByTestId('s40-retry-title')).toContainText('適用前に戻しました')
  await expect(page.getByTestId('s40-retry-title')).toContainText('写り込みが残っています')
  await expect(page.getByTestId('s40-blind-canvas')).toBeVisible()
})

test('BLIND: 履歴に死角の世代が出て、「この値に戻す」が /calib/rollback を item=BLIND で送る', async ({ page }) => {
  await gotoS40(page, {
    calibStatus: status({ result: 'IDLE', item: '', detail: { ...DETAIL, history: {
      ...DETAIL.history, BLIND: [{ generation: 1, values: { blind_angle_ranges: [40, 60] }, calibrated_at: '2026-10-01T10:00:00' }] } } }),
    calibStubs: { rollback: { success: true } },
  })
  await page.getByTestId('s40-tab-history').click()
  await expect(page.getByTestId('s40-history-BLIND')).toContainText('1 世代前: 40〜60°')
  await page.getByTestId('s40-rollback-BLIND-1').click()
  await page.getByTestId('s40-rollback-BLIND-1').click()
  expect(await calibCalls(page)).toContainEqual({ service: 'rollback', item: 'BLIND', generation: 1 })
})
