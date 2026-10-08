// 1b-7 S-13（SG-C5）: S-13 の経路選択が本番の経路で ui.route_select を送ること。
//
// Spec-transit.md §3.2 手順 2〜4: 教示タブで「既存経路の再設定」か「新規経路の
// 設定」を選ぶと、選択直後から記録が始まる。「記録開始」ボタンは無い。
// 過去に「ボタンが在るのに押しても無反応」（onClick 空）を出したため、
// trigger が本当に送られることを検証する（S-11 の ui.stop と同じ狙い）。
// 送信は ros/useTrigger.js の本番の経路（TEST_MODE の記録先
// window.__thTriggerCalls）。
//
// WS-9K E-1: 入力欄の値を id に使い、空欄なら route_ で始まる自動名になること、
// / \ は入力欄（onChange）で _ に置換され、画面に表示されている文字列がそのまま
// 登録名になることを固定する。
import { test, expect } from '@playwright/test'
import { gotoScreen, gotoScreenWithRouteCatalog } from './helpers.js'

async function openTeach(page) {
  await page.locator('#s13').waitFor()
  // ROUTE_SEL では初期表示は走行タブ → 教示タブへ切替。
  await page.getByRole('tab', { name: '教示' }).click()
}

async function routeSelectCalls(page) {
  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  return calls.filter((c) => c.trigger === 'ui.route_select')
}

test('S-13 新規経路 sends ui.route_select with new:true and the entered name as id', async ({ page }) => {
  await gotoScreen(page, 'S13', { mode: 'TEACH_MANUAL', state: 'ROUTE_SEL' })
  await openTeach(page)

  await page.locator('[data-testid="s13-route-name"]').fill('デモ経路')
  await page.getByRole('button', { name: 'この名前で記録を始める' }).click()

  const sels = await routeSelectCalls(page)
  expect(sels.length, '新規選択が ui.route_select を送っていない').toBe(1)
  // useTrigger の TEST_MODE は argJson を JSON 文字列ではなく生オブジェクトで記録する。
  expect(sels[0].argJson.new, '新規教示 (new:true) が送られていない').toBe(true)
  expect(sels[0].argJson.id, '入力した名前が id になっていない').toBe('デモ経路')
})

test('S-13 新規経路 空欄なら route_ で始まる自動名になる', async ({ page }) => {
  await gotoScreen(page, 'S13', { mode: 'TEACH_MANUAL', state: 'ROUTE_SEL' })
  await openTeach(page)

  await page.locator('[data-testid="s13-route-name"]').fill('   ')
  await page.getByRole('button', { name: 'この名前で記録を始める' }).click()

  const sels = await routeSelectCalls(page)
  expect(sels.length, '新規選択が ui.route_select を送っていない').toBe(1)
  expect(sels[0].argJson.new).toBe(true)
  expect(sels[0].argJson.id, '空欄なら route_ で始まる自動名になる').toMatch(/^route_/)
})

test('S-13 新規経路 / \\ は入力欄で _ に置換され、その値が id に載る（画面表示と保存名を揃える）', async ({ page }) => {
  await gotoScreen(page, 'S13', { mode: 'TEACH_MANUAL', state: 'ROUTE_SEL' })
  await openTeach(page)

  const field = page.locator('[data-testid="s13-route-name"]')
  // onChange で / \ が _ に置換されるため、入力欄の表示そのものが正規化される。
  await field.fill('校舎1周/本番\\午後')
  await expect(field).toHaveValue('校舎1周_本番_午後')

  await page.getByRole('button', { name: 'この名前で記録を始める' }).click()
  const sels = await routeSelectCalls(page)
  expect(sels.length, '新規選択が ui.route_select を送っていない').toBe(1)
  expect(sels[0].argJson.id, '正規化後の値が id になっていない').toBe('校舎1周_本番_午後')
})

test('S-13 既存経路の再設定 sends ui.route_select with the id (no new flag)', async ({ page }) => {
  await gotoScreenWithRouteCatalog(page, 'S13',
    { mode: 'TEACH_MANUAL', state: 'ROUTE_SEL' },
    [{ id: 'route_a', name: '経路A', generation: 1, length_m: 12.5, point_count: 100 }])
  await openTeach(page)

  // 同名の上書きなので二段階アーム式（Spec-webui.md §6）。1 段目では送らない。
  await page.getByRole('button', { name: '録り直す' }).click()
  expect(await routeSelectCalls(page), 'アームの1段目で送ってはいけない').toEqual([])

  await page.getByRole('button', { name: '本当に上書きする' }).click()
  const sels = await routeSelectCalls(page)
  expect(sels.length, '2段目で ui.route_select を送っていない').toBe(1)
  expect(sels[0].argJson.id, '選んだ既存経路の id が送られていない').toBe('route_a')
  expect(sels[0].argJson.new, '既存の再設定に new が付いてはいけない').toBeFalsy()
})

test('S-13 経路が0本なら一覧の代わりに「教示済みの経路がありません」と出る', async ({ page }) => {
  await gotoScreenWithRouteCatalog(page, 'S13',
    { mode: 'TEACH_MANUAL', state: 'ROUTE_SEL' }, [])
  await openTeach(page)

  await expect(page.getByTestId('s13-empty')).toBeVisible()
  await expect(page.getByTestId('s13-route-row')).toHaveCount(0)
})

test('S-13 新規名が既存 id と衝突するときはアームを挟む（黙って上書きしない）', async ({ page }) => {
  await gotoScreenWithRouteCatalog(page, 'S13',
    { mode: 'TEACH_MANUAL', state: 'ROUTE_SEL' },
    [{ id: 'route_a', name: '経路A', generation: 1, length_m: 12.5, point_count: 100 }])
  await openTeach(page)

  await page.locator('[data-testid="s13-route-name"]').fill('route_a')
  const newBox = page.locator('[data-testid="s13-new-start"]')
  await newBox.getByRole('button', { name: '録り直す' }).click()
  expect(await routeSelectCalls(page), 'アームの1段目で送ってはいけない').toEqual([])

  await newBox.getByRole('button', { name: '本当に上書きする' }).click()
  const sels = await routeSelectCalls(page)
  expect(sels.length, '2段目で ui.route_select を送っていない').toBe(1)
  expect(sels[0].argJson.new).toBe(true)
  expect(sels[0].argJson.id).toBe('route_a')
})
