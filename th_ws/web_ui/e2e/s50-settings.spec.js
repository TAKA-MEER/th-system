// S-50 設定（Spec-webui §3.15。2026-11-27 作り直し: タブ無し・左に区分 8 つ）。
// S-01「設定」→ S-50 が開き、区分が切り替わり、「戻る」で S-01 に戻ること。
// S-50 は FSM のモードではなく S-01 のサブ画面
// （main.jsx の settingsOpen / screenRouting.resolveScreen）。
//
// config_manager の service 呼び出しは TEST_MODE（window.__thTestState 定義時）
// では useTunableParams.js が即 reject するので、ネット無しで回る。
import { test, expect } from '@playwright/test'
import { gotoScreen, setTestState, stubTrigger, gotoScreenWithTunables, gotoScreenWithParams, paramsSetCalls } from './helpers.js'

async function openS50(page) {
  await gotoScreen(page, 'S01', { mode: 'IDLE', tracker_enabled: true })
  await page.getByTestId('s01-open-settings').click()
  await expect(page.locator('#s50')).toBeVisible()
}

test('S-01 →「設定」→ S-50、区分切替、「いつ効くか」、戻る', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'IDLE', tracker_enabled: true })
  await page.getByTestId('s01-open-settings').click()

  const s50 = page.locator('#s50')
  await expect(s50).toBeVisible()
  await expect(page.locator('#screenName')).toHaveText('設定')

  // 区分は 8 つ（速さとブレーキ・再生の位置合わせ・知らせる・画面・つなぎ先・
  // 異常の履歴・LiDAR 死角・開発モード）。
  await expect(page.getByTestId('s50-sec-run')).toBeVisible()
  await expect(page.getByTestId('s50-sec-loc')).toBeVisible()
  await expect(page.getByTestId('s50-sec-notify')).toBeVisible()
  await expect(page.getByTestId('s50-sec-screen')).toBeVisible()
  await expect(page.getByTestId('s50-sec-link')).toBeVisible()
  await expect(page.getByTestId('s50-sec-log')).toBeVisible()
  await expect(page.getByTestId('s50-sec-blind')).toBeVisible()
  await expect(page.getByTestId('s50-sec-dev')).toBeVisible()

  // 既定は速さとブレーキ。保存は1つだけ。
  await expect(page.getByTestId('s50-save-run')).toBeVisible()
  await expect(page.getByTestId('s50-when-run')).toContainText('次の起動から')
  await page.getByTestId('s50-sec-loc').click()
  await expect(page.getByTestId('s50-save-slam_toolbox')).toBeVisible()
  await expect(page.getByTestId('s50-when-loc')).toContainText('読み直')

  // 画面の区分へ
  await page.getByTestId('s50-sec-screen').click()
  await expect(page.getByTestId('s50-font-large')).toBeVisible()

  // 開発モード区分へ
  await page.getByTestId('s50-sec-dev').click()
  await expect(page.getByTestId('s50-dev-toggle')).toBeVisible()

  // 戻る → S-01
  await page.getByTestId('s50-back').click()
  await expect(page.locator('#s50')).toHaveCount(0)
  await expect(page.locator('#s01')).toBeVisible()
})

test('S-50 表示中にモードが IDLE を離れたら設定は閉じて画面が追随する', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'IDLE', tracker_enabled: true })
  await page.getByTestId('s01-open-settings').click()
  await expect(page.locator('#s50')).toBeVisible()

  await setTestState(page, { mode: 'REPLAY' })

  await expect(page.locator('#s50')).toHaveCount(0)
  await expect(page.locator('#s14')).toBeVisible()
})

// 1b-5: MANUAL（手動走行）では設定を変更できない。S-50 は MANUAL では
// 開けない（S-11 に切り替わり設定は畳まれる）ため、入力の無効表示ではなく
// 画面遷移で縛る（test/unit/stop-only-guard.test.js とサーバ側拒否が担う）。
test('MANUAL では S-50 が閉じて設定を変更できない（S-11 に追随）', async ({ page }) => {
  await openS50(page)
  await expect(page.getByTestId('s50-sec-run')).toBeVisible()

  await setTestState(page, { mode: 'MANUAL', state: 'NONE' })

  await expect(page.locator('#s50')).toHaveCount(0)
  await expect(page.locator('#s11')).toBeVisible()
})

test('INIT のときは「開発モードの設定」から S-50 を開ける', async ({ page }) => {
  // mode=INIT では S-01 のモードボタンが全部押せない（M-1）。S-00 の「開発モードの
  // 設定」（§5）から S-50 の開発モード区分を開ける（2026-11-27 作り直し後の導線）。
  await gotoScreen(page, 'S00', { mode: 'INIT' })
  await expect(page.locator('#s00')).toBeVisible()
  await expect(page.getByTestId('s00-advance')).toHaveClass(/dis/)
  await expect(page.getByTestId('s00-open-dev')).toBeEnabled()
  await page.getByTestId('s00-open-dev').click()
  await expect(page.locator('#s50')).toBeVisible()
  await expect(page.getByTestId('s50-dev-toggle')).toBeVisible()
})

test('S-50 LiDAR 死角の区分に編集欄は無く、表示と「校正で変更する」がある', async ({ page }) => {
  await openS50(page)
  await page.getByTestId('s50-sec-blind').click()

  await expect(page.getByTestId('s50-blind-current')).toBeVisible()
  await expect(page.getByTestId('s50-goto-calib')).toBeVisible()
  await expect(page.getByTestId('s50-goto-calib')).toHaveText('校正で変更する')
})

test('「校正で変更する」は ui.enter_mode{CALIB} を送り、拒否されたら理由を出す', async ({ page }) => {
  await openS50(page)
  await page.getByTestId('s50-sec-blind').click()

  await page.getByTestId('s50-goto-calib').click()
  const calls = (await page.evaluate(() => window.__thTriggerCalls ?? []))
    .filter((c) => c.trigger !== 'ui.screen')
  expect(calls).toHaveLength(1)
  expect(calls[0]).toEqual({
    trigger: 'ui.enter_mode', argJson: { mode: 'CALIB' }, requester: 'web_ui',
  })
  await expect(page.getByTestId('s50-calib-err')).toBeVisible()
})

test('「校正で変更する」が受理されたら理由は出ない', async ({ page }) => {
  await stubTrigger(page, { 'ui.enter_mode': { accepted: true } })
  await openS50(page)
  await page.getByTestId('s50-sec-blind').click()

  await page.getByTestId('s50-goto-calib').click()
  const calls = (await page.evaluate(() => window.__thTriggerCalls ?? []))
    .filter((c) => c.trigger !== 'ui.screen')
  expect(calls).toHaveLength(1)
  expect(calls[0].argJson).toEqual({ mode: 'CALIB' })
  await expect(page.getByTestId('s50-calib-err')).toHaveCount(0)
})

// 速度プリセット区画は /params/get で効き値を読み、保存は /params/set
//（出どころ付き）で行う。走行中の値には触れない（再起動後に反映）。
test('速度プリセット区画は効き値と次回反映の差を出し、保存は /params/set を呼ぶ', async ({ page }) => {
  await gotoScreenWithParams(page, 'S01', { mode: 'IDLE', tracker_enabled: true }, {
    get: {
      effective: { speed_preset_low: 0.27, speed_preset_mid: 0.55, speed_preset_high: 1.0 },
      pending: { speed_preset_mid: 0.6 },
      origins: {},
      restart_required: true,
    },
    set: { success: true, message: 'OK' },
  })
  await page.getByTestId('s01-open-settings').click()
  await expect(page.locator('#s50')).toBeVisible()

  await expect(page.getByTestId('s50-save-run')).toBeVisible()
  await expect(page.getByTestId('s50-preset-pending')).toContainText('次回から')
  await expect(page.getByTestId('s50-preset-pending')).toContainText('speed_preset_mid')

  // 理由なしでは送らない。
  await page.getByTestId('s50-save-run').click()
  await expect(page.getByTestId('s50-status-preset')).toContainText('理由')
  expect(await paramsSetCalls(page)).toHaveLength(0)

  await page.getByTestId('s50-preset-reason').fill('現場で低速を詰める')
  await page.getByLabel(/中速/).fill('0.65')
  await page.getByLabel(/中速/).press('Tab')
  await page.getByTestId('s50-save-run').click()
  const calls = await paramsSetCalls(page)
  expect(calls.length).toBeGreaterThan(0)
  expect(calls[0].setBy).toBe('web_ui')
  expect(calls[0].reason).toBe('現場で低速を詰める')
  expect(calls[0].values).toEqual({ speed_preset_mid: 0.65 })
})

test('速度プリセットの順序違反は保存不可で理由を表示する', async ({ page }) => {
  await gotoScreenWithParams(page, 'S01', { mode: 'IDLE', tracker_enabled: true }, {
    get: {
      effective: { speed_preset_low: 0.27, speed_preset_mid: 0.55, speed_preset_high: 1.0 },
      pending: {},
      origins: {},
      restart_required: false,
    },
    set: { success: true, message: 'OK' },
  })
  await page.getByTestId('s01-open-settings').click()
  await expect(page.locator('#s50')).toBeVisible()

  await expect(page.getByTestId('s50-save-run')).toBeEnabled()
  await expect(page.getByTestId('s50-preset-order')).toHaveCount(0)

  await page.getByLabel(/中速/).fill('0.1')
  await page.getByLabel(/中速/).press('Tab')
  await expect(page.getByTestId('s50-preset-order')).toContainText('中速以下')
  await expect(page.getByTestId('s50-save-run')).toBeDisabled()
  expect(await paramsSetCalls(page)).toHaveLength(0)
})

// 自動ブレーキの既定は /params/set で保存し、次の起動から効く。
test('自動ブレーキ既定は /params/set で保存する（速さとブレーキの区分）', async ({ page }) => {
  await gotoScreenWithParams(page, 'S01', { mode: 'IDLE', tracker_enabled: true }, {
    get: {
      effective: {
        speed_preset_low: 0.27, speed_preset_mid: 0.55, speed_preset_high: 1.0,
        auto_brake_default_outdoor: 'off', auto_brake_default_indoor: 'on',
      },
      pending: {},
      origins: {},
      restart_required: false,
    },
    set: { success: true, message: 'OK' },
  })
  await page.getByTestId('s01-open-settings').click()
  await expect(page.locator('#s50')).toBeVisible()

  await expect(page.getByTestId('s50-brake-outdoor')).toHaveValue('off')
  await expect(page.getByTestId('s50-brake-indoor')).toHaveValue('on')
  await expect(page.getByTestId('s50-sec-run')).toContainText('速さとブレーキ')
})

test('カメラの開く先区画は http(s) だけ /params/set で保存できる', async ({ page }) => {
  await gotoScreenWithParams(page, 'S01', { mode: 'IDLE', tracker_enabled: true }, {
    get: {
      effective: { camera_ui_url: 'http://192.168.5.129/' },
      pending: {},
      origins: {},
      restart_required: false,
    },
    set: { success: true, message: 'OK' },
  })
  await page.getByTestId('s01-open-settings').click()
  await expect(page.locator('#s50')).toBeVisible()
  await page.getByTestId('s50-sec-link').click()

  await expect(page.getByTestId('s50-camera-url')).toHaveValue('http://192.168.5.129/')

  await page.getByTestId('s50-camera-url').fill('ftp://192.168.5.129/')
  await expect(page.getByTestId('s50-save-camera')).toBeDisabled()
  expect(await paramsSetCalls(page)).toHaveLength(0)

  await page.getByTestId('s50-camera-url').fill('http://192.168.5.10/cam')
  await page.getByTestId('s50-camera-reason').fill('DHCP が変わった')
  await expect(page.getByTestId('s50-save-camera')).toBeEnabled()
  await page.getByTestId('s50-save-camera').click()
  const calls = await paramsSetCalls(page)
  expect(calls).toHaveLength(1)
  expect(calls[0].values).toEqual({ camera_ui_url: 'http://192.168.5.10/cam' })
})

test('画面の区分で音声・縦横・文字・ヘルプを変えると記憶される', async ({ page }) => {
  await openS50(page)
  await page.getByTestId('s50-sec-notify').click()

  await expect(page.getByTestId('s50-voice-on')).toHaveClass(/on/)
  await page.getByTestId('s50-voice-off').click()
  expect(await page.evaluate(() => window.localStorage.getItem('th.voiceMaster'))).toBe('0')
  await page.getByTestId('s50-voice-on').click()
  expect(await page.evaluate(() => window.localStorage.getItem('th.voiceMaster'))).toBe('1')

  await page.getByTestId('s50-sec-screen').click()
  await page.getByTestId('s50-orientation-landscape').click()
  await expect(page.locator('[data-testid="stage"]')).toHaveAttribute('data-orientation', 'landscape')
  await page.getByTestId('s50-orientation-portrait').click()
  await expect(page.locator('[data-testid="stage"]')).toHaveAttribute('data-orientation', 'portrait')
  await page.getByTestId('s50-orientation-auto').click()

  await page.getByTestId('s50-help-off').click()
  expect(await page.evaluate(() => window.localStorage.getItem('th.helpShown'))).toBe('0')
  await page.getByTestId('s50-help-on').click()
  expect(await page.evaluate(() => window.localStorage.getItem('th.helpShown'))).toBe('1')
})
