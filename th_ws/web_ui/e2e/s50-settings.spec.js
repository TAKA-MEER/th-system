// WS-9X: S-01「保守・設定」→「設定」で S-50 が開き、タブが切り替わり、
// 「戻る」で S-01 に戻ること。S-50 は FSM のモードではなく S-01 のサブ画面
// （main.jsx の settingsOpen / screenRouting.resolveScreen）。
//
// config_manager の service 呼び出しは TEST_MODE（window.__thTestState 定義時）
// では useTunableParams.js が即 reject するので、ネット無しで回る。
import { test, expect } from '@playwright/test'
import { readFileSync } from 'node:fs'
import { gotoScreen, setTestState, stubTrigger, gotoScreenWithTunables, tunableApplyCalls, gotoScreenWithParams, paramsSetCalls } from './helpers.js'

// registry 連動の上限（scripts/gen_param_limits.py の生成物）。天井が変われば
// 作り直されるので、ここでは値を直書きせず生成物を読む。
const PARAM_LIMITS = JSON.parse(
  readFileSync(new URL('../src/generated/param_limits.json', import.meta.url), 'utf8'))

async function openS50General(page) {
  await gotoScreen(page, 'S01', { mode: 'IDLE', tracker_enabled: true })
  await page.getByTestId('s01-open-settings').click()
  await expect(page.locator('#s50')).toBeVisible()
}

test('S-01 →「設定」→ S-50、タブ切替、戻る', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'IDLE', tracker_enabled: true })

  await page.getByTestId('s01-open-settings').click()

  const s50 = page.locator('#s50')
  await expect(s50).toBeVisible()
  await expect(page.locator('#screenName')).toHaveText('設定')

  // 既定は一般タブ。3 セクションの保存ボタンが見える。
  await expect(page.getByTestId('s50-save-slam_toolbox')).toBeVisible()

  // 表示タブへ
  await page.getByTestId('s50-tab-display').click()
  await expect(page.getByTestId('s50-font-large')).toBeVisible()

  // 開発モードタブへ
  await page.getByTestId('s50-tab-dev').click()
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
// 画面遷移で縛る。入力欄の無効化そのものは test/unit/stop-only-guard.test.js
//（mapOrParamOpAllowed(MANUAL)=false）とサーバ側の拒否が担う。
test('MANUAL では S-50 が閉じて設定を変更できない（S-11 に追随）', async ({ page }) => {
  await openS50General(page)
  // IDLE では入力できる（対照。空振り防止）。
  await expect(page.getByLabel(/最高速度/)).toBeEnabled()
  await expect(page.getByTestId('s50-guard')).toHaveCount(0)

  await setTestState(page, { mode: 'MANUAL', state: 'NONE' })

  // S-50 は畳まれ、手動走行（S-11）になる。設定は開けない＝変更できない。
  await expect(page.locator('#s50')).toHaveCount(0)
  await expect(page.locator('#s11')).toBeVisible()
})

test('「設定」は他のメニューが押せない状況でも押せる（disabledAll に縛られない）', async ({ page }) => {
  // mode=INIT ではモード選択ボタンが全部 disabled（menuItems の M-1）だが、
  // 設定は FSM を動かさないので読めるようにしてある。
  await gotoScreen(page, 'S01', { mode: 'INIT' })
  const settings = page.getByTestId('s01-open-settings')
  await expect(settings).toBeEnabled()
  await settings.click()
  await expect(page.locator('#s50')).toBeVisible()
})

// 2026-10-02 Spec-webui.md §3.15: S-50 では死角マスクを編集しない。
// 読み取り専用の表示と S-40 への導線だけを置く。
test('S-50 一般タブに死角の編集欄は無く、表示と「校正で変更する」がある', async ({ page }) => {
  await openS50General(page)

  // 編集欄も保存ボタンも無い（誤って戻すとここが赤になる）。
  await expect(page.getByTestId('s50-save-lidar_filter')).toHaveCount(0)
  // 表示はある（TEST_MODE では取得できないので「取得できませんでした」が出る）。
  await expect(page.getByTestId('s50-blind-current')).toBeVisible()
  await expect(page.getByTestId('s50-goto-calib')).toBeVisible()
  await expect(page.getByTestId('s50-goto-calib')).toHaveText('校正で変更する')
})

test('「校正で変更する」は ui.enter_mode{CALIB} を送り、拒否されたら理由を出す', async ({ page }) => {
  await openS50General(page)

  // スタブ無し = 拒否扱い（useTrigger の TEST_MODE 既定）。
  await page.getByTestId('s50-goto-calib').click()
  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(calls).toHaveLength(1)
  expect(calls[0]).toEqual({
    trigger: 'ui.enter_mode', argJson: { mode: 'CALIB' }, requester: 'web_ui',
  })
  await expect(page.getByTestId('s50-calib-err')).toBeVisible()
})

test('「校正で変更する」が受理されたら理由は出ない', async ({ page }) => {
  await stubTrigger(page, { 'ui.enter_mode': { accepted: true } })
  await openS50General(page)

  await page.getByTestId('s50-goto-calib').click()
  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(calls).toHaveLength(1)
  expect(calls[0].argJson).toEqual({ mode: 'CALIB' })
  await expect(page.getByTestId('s50-calib-err')).toHaveCount(0)
})

// SG-B9: 本番の bringup（use_stub なし）では follow_planner_mapless が起動して
// いないので、その読み込み失敗を全体の失敗として扱ってはならない。起動して
// いるぶん（lidar_filter / slam_toolbox）は出す。
test('片方の取得に失敗しても一般タブ全体は落ちない（節ごとの注記になる）', async ({ page }) => {
  await gotoScreenWithTunables(page, 'S01', { mode: 'IDLE', tracker_enabled: true }, {
    // follow_planner_mapless のエントリ無し = reject（未起動ノードと同じ）。
    lidar_filter: { values: { blind_angle_ranges: [] } },
    slam_toolbox: {
      values: {
        minimum_travel_distance: 0.1,
        minimum_travel_heading: 0.1,
        correlation_search_space_dimension: 0.5,
        correlation_search_space_resolution: 0.01,
        link_match_minimum_response_fine: 0.1,
      },
    },
  })
  await page.getByTestId('s01-open-settings').click()
  await expect(page.locator('#s50')).toBeVisible()

  // 全体の失敗にしない。
  await expect(page.getByTestId('s50-load-error')).toHaveCount(0)
  // 失敗した節だけ注記が出る。
  await expect(page.getByTestId('s50-status-follow_planner_mapless'))
    .toHaveText('現在値を取得できませんでした')
  // 取れた節は使えるまま（保存ボタンと取得値が見える）。
  await expect(page.getByTestId('s50-save-slam_toolbox')).toBeVisible()
  await expect(page.getByTestId('s50-save-follow_planner_mapless')).toBeVisible()
})

// SG-B9: 旧ノードの v_max（UI 上限 1.5）は registry の v_max を超えて保存
// できない。1.5 を入れても天井（param_limits.json）で丸められて送られる。
test('v_max に上限超えを入れても registry の天井で丸められる', async ({ page }) => {
  await gotoScreenWithTunables(page, 'S01', { mode: 'IDLE', tracker_enabled: true }, {
    follow_planner_mapless: { values: { v_max: 0.3 } },
    lidar_filter: { values: { blind_angle_ranges: [] } },
    slam_toolbox: { values: {} },
  })
  await page.getByTestId('s01-open-settings').click()
  await expect(page.locator('#s50')).toBeVisible()

  await page.getByLabel(/最高速度/).fill('1.5')
  await page.getByLabel(/最高速度/).press('Tab')
  const calls = await tunableApplyCalls(page)
  const vMaxCalls = calls.filter((c) => c.paramName === 'v_max')
  expect(vMaxCalls).toHaveLength(1)
  expect(vMaxCalls[0].nodeName).toBe('follow_planner_mapless')
  // 上限自体が昔の UI 上限（1.5）より小さいことが、この試験が空振りでないことの保証。
  expect(PARAM_LIMITS.v_max).toBeLessThan(1.5)
  expect(vMaxCalls[0].value).toBe(PARAM_LIMITS.v_max)
})

// SG-B8: 速度プリセット区画は /params/get で効き値を読み、保存は /params/set
//（出どころ付き）で行う。走行中の値には触れない（再起動後に反映）。
// 保存済みでまだ効いていない値は「次回から」で示す。
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

  // 「再起動後に反映」の注記と、効き値と保存済み値の差がある。
  await expect(page.getByTestId('s50-save-preset')).toBeVisible()
  await expect(page.getByTestId('s50-preset-pending')).toContainText('次回から')
  await expect(page.getByTestId('s50-preset-pending')).toContainText('speed_preset_mid')

  // 理由なしでは送らない（PT-4: 出どころの無い数値を作らない）。
  await page.getByTestId('s50-save-preset').click()
  await expect(page.getByTestId('s50-status-preset')).toContainText('理由')
  expect(await paramsSetCalls(page)).toHaveLength(0)

  // 理由を入れて値を変えて保存すると /params/set が呼ばれる。
  await page.getByTestId('s50-preset-reason').fill('現場で低速を詰める')
  await page.getByLabel(/中速/).fill('0.65')
  await page.getByLabel(/中速/).press('Tab')
  await page.getByTestId('s50-save-preset').click()
  const calls = await paramsSetCalls(page)
  expect(calls).toHaveLength(1)
  expect(calls[0].setBy).toBe('web_ui')
  expect(calls[0].reason).toBe('現場で低速を詰める')
  expect(calls[0].values).toEqual({ speed_preset_mid: 0.65 })
})
