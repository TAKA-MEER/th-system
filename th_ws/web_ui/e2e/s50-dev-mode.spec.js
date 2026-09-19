// WP-DEV-01B: S-50 の開発モードタブが ROS 側と繋がっていること。
//
// TEST_MODE（window.__thTestState 定義時）では rosbridge を開かず、
// useDevMode.js が送信を window.__thDevParamCalls に記録し、
// window.__thSetTestDevMode で /system/dev_mode 受信を再現する。
import { test, expect } from '@playwright/test'
import {
  gotoScreen, gotoScreenWithDevMode, setTestDevMode, devParamCalls,
} from './helpers.js'

const ALL_TRUE = { link: true, battery: true, opcheck: true, auto_brake: true }
const ALL_FALSE = { link: false, battery: false, opcheck: false, auto_brake: false }

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
  // 既定 true からの反転なので false が送られる。
  expect(hit.value).toBe(false)
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

test('効かない旨と無視できない一覧が出ている（完了条件4・5）', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'IDLE' })
  await openDevTab(page)

  await expect(page.getByTestId('s50-dev-no-gate')).toBeVisible()
  const unignorable = page.getByTestId('s50-dev-unignorable')
  for (const name of ['物理非常停止ボタン', 'ESP32 のウォッチドッグ', 'UI 非常停止ボタン', '自律系の障害物停止']) {
    await expect(unignorable).toContainText(name)
  }
})
