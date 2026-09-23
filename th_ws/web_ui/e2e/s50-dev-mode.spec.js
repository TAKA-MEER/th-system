// WP-DEV-01B: S-50 の開発モードタブが ROS 側と繋がっていること。
//
// TEST_MODE（window.__thTestState 定義時）では rosbridge を開かず、
// useDevMode.js が送信を window.__thDevParamCalls に記録し、
// window.__thSetTestDevMode で /system/dev_mode 受信を再現する。
import { test, expect } from '@playwright/test'
import {
  gotoScreen, gotoScreenWithDevMode, setTestDevMode, devParamCalls, setTestState,
} from './helpers.js'

const ITEMS = ['link', 'lidar_fault', 'scan_stop', 'battery', 'opcheck', 'auto_brake']
const ALL_TRUE = Object.fromEntries(ITEMS.map((k) => [k, true]))
const ALL_FALSE = Object.fromEntries(ITEMS.map((k) => [k, false]))

const DEV_ON = {
  dev_mode: true, ignore: ALL_TRUE, effective: ALL_TRUE,
  estop_hw_known: false, estop_hw_pressed: false,
}
const DEV_OFF = {
  dev_mode: false, ignore: ALL_FALSE, effective: ALL_FALSE,
  estop_hw_known: false, estop_hw_pressed: false,
}

async function openDevTab(page) {
  await page.getByTestId('s01-open-settings').click()
  await page.locator('#s50').waitFor()
  await page.getByTestId('s50-tab-dev').click()
  await expect(page.getByTestId('s50-dev-toggle')).toBeVisible()
}

test('トグルを押すと dev_mode パラメータ送信が記録される（完了条件1）', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'IDLE' })
  await openDevTab(page)

  // 種なし＝未受信なので localStorage（初期 false）表示。「有効にする」が出る。
  await expect(page.getByTestId('s50-dev-toggle')).toHaveText('開発モードを有効にする')

  await page.getByTestId('s50-dev-toggle').click()

  const calls = await devParamCalls(page)
  const hit = calls.find((c) => c.name === 'dev_mode')
  expect(hit, 'トグルが dev_mode パラメータ送信をしていない').toBeTruthy()
  expect(hit.node, '送信先が connectivity_checker ではない').toBe('connectivity_checker')
  expect(hit.value, 'ON 操作なのに true が送られていない').toBe(true)

  // 見た目は即時反映（正本の到着を待たない）。
  await expect(page.getByTestId('s50-dev-toggle')).toHaveText('開発モードを無効にする')
})

test('項目トグルを押すと dev_ignore_* が記録される（完了条件3）', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'IDLE' })
  await openDevTab(page)

  await page.getByTestId('s50-dev-ignore-link').click()

  const calls = await devParamCalls(page)
  const hit = calls.find((c) => c.name === 'dev_ignore_link')
  expect(hit, '項目トグルが dev_ignore_link を送っていない').toBeTruthy()
  expect(hit.node).toBe('connectivity_checker')
  // 既定 false（開発モードに入っただけでは何も外さない）からの反転なので true。
  expect(hit.value).toBe(true)
})

test('項目の既定は全部 OFF（開発モードに入っただけでは通常運用と同じ）', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'IDLE' })
  await openDevTab(page)
  for (const item of ITEMS) {
    await expect(page.getByTestId(`s50-dev-ignore-${item}`)).toHaveAttribute('aria-pressed', 'false')
  }
})

test('LiDAR 無し走行の 2 項目は dev_ignore_lidar_fault / dev_ignore_scan_stop を送る', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'IDLE' })
  await openDevTab(page)
  await page.getByTestId('s50-dev-ignore-lidar_fault').click()
  await page.getByTestId('s50-dev-ignore-scan_stop').click()
  const calls = await devParamCalls(page)
  for (const name of ['dev_ignore_lidar_fault', 'dev_ignore_scan_stop']) {
    const hit = calls.find((c) => c.name === name)
    expect(hit, `${name} が送られていない`).toBeTruthy()
    expect(hit.node).toBe('connectivity_checker')
    expect(hit.value).toBe(true)
  }
})

test('S-00 から開発モードの設定に入り、IDLE に達したら戻って進める', async ({ page }) => {
  await gotoScreen(page, 'S00', { mode: 'INIT' })
  await page.locator('#s00').waitFor()
  // INIT のあいだは「進む」は無く、開発モードの導線がある。
  await expect(page.getByTestId('s00-advance')).toHaveCount(0)
  await page.getByTestId('s00-open-dev').click()

  // 開発モードタブで始まる（INIT では一般タブの読み込みが通らない）。
  await page.locator('#s50').waitFor()
  await expect(page.getByTestId('s50-dev-toggle')).toBeVisible()
  await page.getByTestId('s50-dev-toggle').click()
  await page.getByTestId('s50-dev-ignore-link').click()
  const calls = await devParamCalls(page)
  expect(calls.find((c) => c.name === 'dev_mode' && c.value === true)).toBeTruthy()
  expect(calls.find((c) => c.name === 'dev_ignore_link' && c.value === true)).toBeTruthy()

  // 機体側が evt.link_ok を出して IDLE に進んだ。
  await setTestState(page, { mode: 'IDLE' })
  await page.getByTestId('s50-back').click()

  await page.locator('#s00').waitFor()
  await page.getByTestId('s00-advance').click()
  await page.getByTestId('s01-open-settings').waitFor()
})

test('/system/dev_mode が来るとヘッダと S-50 が追従する（完了条件2）', async ({ page }) => {
  await gotoScreenWithDevMode(page, 'S01', { mode: 'IDLE' }, DEV_OFF)
  await openDevTab(page)

  await expect(page.locator('#devPill')).toHaveCount(0)
  await expect(page.getByTestId('s50-dev-effective')).toHaveText('なし（通常運用）')

  await setTestDevMode(page, DEV_ON)

  // ヘッダの pill が出る（localStorage ではなく topic で）。
  await expect(page.locator('#devPill')).toBeVisible()
  // トグル表示も topic に追従する。
  await expect(page.getByTestId('s50-dev-toggle')).toHaveText('開発モードを無効にする')
  // いま何を無視しているかが出る。
  await expect(page.getByTestId('s50-dev-effective')).toContainText('機器未接続')
  // E-Stop 未受信の注記が出る。
  await expect(page.getByTestId('s50-dev-estop-unknown')).toBeVisible()

  await setTestDevMode(page, DEV_OFF)
  await expect(page.locator('#devPill')).toHaveCount(0)
  await expect(page.getByTestId('s50-dev-toggle')).toHaveText('開発モードを有効にする')
})

test('効かない旨と「選んだ項目だけが外れる」注記が出ている（Spec-safety.md §10）', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'IDLE' })
  await openDevTab(page)

  await expect(page.getByTestId('s50-dev-no-gate')).toBeVisible()
  await expect(page.getByTestId('s50-dev-scope')).toContainText('選んだ項目だけ')
  // 2026-09-23 改定で「無視できない一覧」は廃止した。
  await expect(page.getByTestId('s50-dev-unignorable')).toHaveCount(0)
})
