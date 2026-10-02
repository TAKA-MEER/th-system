// e2e/s40-calib-guide.spec.js — S-30「校正へ」→ S-40 誘導（計画書 §6 #11）。
// 本番の送信経路を縛る:
//   - S-30 の「校正へ」が ui.enter_mode{mode:CALIB} を /system/trigger へ実際に送る
//   - CALIB/LIST に入ると受け渡し（screens/calibGuide.js）を 1 回だけ読んで
//     該当項目（LIDAR→BLIND / IMU→IMU）を強調し、案内を出す。自動では始まらない
//     （開始は人が押す。Spec.md SD-8）
//   - enter_mode 拒否では受け渡しが残らない／別経路（S-01 等）では何も出ない
//
// 画面遷移はモード導出（screens/screenRouting.js）で起きる: __thTestScreen は
// 初期表示だけを決め、その後の mode 変更（__thSetTestState）は Screens() が
// 追う。S-30 → S-40 は同一ページ内の遷移なので、モジュール内の受け渡しが残る。
import { test, expect } from '@playwright/test'
import { stubTrigger } from './helpers.js'

// S-30 を RUNNING_CHECK＋判定済みで開く（s30-opcheck.spec.js の gotoS30 と同型に
// calib 側の種も足したもの）。S-40 に遷る段階では __thSetTestState で mode だけ
// 変えるので、__thTestScreen='S30' のまま S-40 がマウントされる。
async function gotoS30WithVerdict(page, opcheckStatus) {
  await page.addInitScript(({ os, cs }) => {
    window.__thTestState = { mode: 'OPCHECK', state: 'RUNNING_CHECK' }
    window.__thTestScreen = 'S30'
    window.__thTestOpcheckAnswer = () => ({ accepted: true })
    window.__thTestOpcheckStatus = os
    window.__thTestCalibStatus = cs
  }, {
    os: opcheckStatus,
    cs: { item: '', step: '', result: 'IDLE', preview_before: '', preview_after: '', detail: {} },
  })
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
const triggerCalls = (page) => page.evaluate(() => window.__thTriggerCalls ?? [])

async function clickGotoCalib(page) {
  await expect(page.getByTestId('s30-goto-calib')).toBeVisible()
  await page.getByTestId('s30-goto-calib').click()
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

test('LIDAR が NG →「校正へ」→ S-40 で BLIND が強調され案内が出る（自動では始まらない）', async ({ page }) => {
  await stubTrigger(page, { 'ui.enter_mode': { accepted: true } })
  await gotoS30WithVerdict(page, { item: 'LIDAR', result: 'NG', detail: '', next_screen: 'lidar_calib' })

  await clickGotoCalib(page)
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

test('IMU が要確認（WARN）→「校正へ」→ S-40 で IMU が強調される', async ({ page }) => {
  await stubTrigger(page, { 'ui.enter_mode': { accepted: true } })
  await gotoS30WithVerdict(page, { item: 'IMU', result: 'WARN', detail: '', next_screen: 'imu_calib' })

  await clickGotoCalib(page)
  await enterCalibList(page)

  await expect(page.getByTestId('s40-calib-guide')).toContainText('要確認でした')
  await expect(page.getByTestId('s40-item-IMU')).toHaveClass(/guided/)
  await expect(page.getByTestId('s40-item-BLIND')).not.toHaveClass(/guided/)
})

test('enter_mode が拒否されたら誘導は残らない（別経路で入っても出ない）', async ({ page }) => {
  await stubTrigger(page, { 'ui.enter_mode': { accepted: false, reject_reason_key: 'not_allowed' } })
  await gotoS30WithVerdict(page, { item: 'LIDAR', result: 'NG', detail: '', next_screen: 'lidar_calib' })

  await clickGotoCalib(page)
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
  await gotoS30WithVerdict(page, { item: 'LIDAR', result: 'NG', detail: '', next_screen: 'lidar_calib' })

  await clickGotoCalib(page)
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
