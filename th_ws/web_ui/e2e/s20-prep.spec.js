// WP-UI-06: S-20 試験準備の操作が本当に th_state へ届くこと（送ったものを数えられる
// __thTriggerCalls ／ __thOnsiteServiceCalls で検証）。過去に「ボタンが在るのに
// onClick 空」を出したため、全ての操作ボタンで trigger/service の記録を確認する。
// th_state は無いので PREP/MAPPING を addInitScript で直接注入し、REGISTER への
// 遷移は __thSetTestState で再現する（Server 側 FSM を起動しない）。
import { test, expect } from '@playwright/test'
import {
  gotoScreen, gotoScreenWithOnsite, onsiteServiceCalls,
  setTestState, setTestPersonTargets, setTestOnsitePins,
} from './helpers.js'

// Pin.msg の最小形（pose.position が地図座標、kind が HOME/PANEL）。
function pin(id, name, kind, x, y, yawDeg = 90) {
  const rad = (yawDeg * Math.PI) / 180
  return {
    id, name, kind, registered_at: '',
    pose: {
      position: { x, y, z: 0 },
      orientation: { x: 0, y: 0, z: Math.sin(rad / 2), w: Math.cos(rad / 2) },
    },
  }
}

const PINS = [
  pin('p1', '盤 A-1', 'PANEL', 86, 88, 92),
  pin('p2', '盤 A-2', 'PANEL', 176, 88, 91),
  pin('p3', '待機場所', 'HOME', 70, 190, 274),
]

const TARGETS = {
  candidates: [{ x: 1.2, y: 0.4 }, { x: -0.8, y: 0.6 }],
  selected_index: 0,
  confidence: 0.87,
  is_lost: false,
  lost_reason: '',
}

const PREP = { mode: 'PREP', state: 'MAPPING' }

// 最小 OccupancyGrid: 4×4, res 0.5, origin (-1,-1) → world x/y ∈ [-1,1]。
// 0 自由 / 100 占有 / -1 未知（RoutePreview e2e と同じ形）。
const ROUTE_MAP = {
  info: {
    resolution: 0.5,
    width: 4,
    height: 4,
    origin: { position: { x: -1, y: -1 }, orientation: { w: 1, z: 0 } },
  },
  data: [0, 100, -1, 0, 100, 0, 0, -1, -1, 0, 100, 0, 0, -1, 100, 0],
}

async function triggers(page) {
  return page.evaluate(() => window.__thTriggerCalls ?? [])
}

test('S-20 表示とタイトル', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, { pins: PINS, targets: TARGETS, pose: { x: 0, y: 0, yaw: 0 } })
  await page.locator('#s20').waitFor()
  await expect(page.locator('#s20 [data-testid="s20-finish"]')).toBeVisible()
  // 地図タブが初期表示: ピン 3 個とロボットマーカー。
  await expect(page.locator('[data-testid="s20-map-pin-p1"]')).toBeVisible()
  await expect(page.locator('[data-testid="s20-map-pin-p3"]')).toBeVisible()
  await expect(page.locator('[data-testid="s20-map-robot"]')).toBeVisible()
})

test('終了は ui.finish が 1 回だけ送られる', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, { pins: PINS, targets: TARGETS })
  await page.locator('#s20').waitFor()
  await page.locator('[data-testid="s20-finish"]').click()
  const finish = (await triggers(page)).filter((c) => c.trigger === 'ui.finish')
  expect(finish, '終了が ui.finish を送っていない').toHaveLength(1)
})

test('待機場所/配電盤の登録は ui.register の kind を送る', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, { pins: PINS, targets: TARGETS })
  await page.locator('#s20').waitFor()

  await page.locator('[data-testid="s20-reg-home"]').click()
  let reg = (await triggers(page)).filter((c) => c.trigger === 'ui.register')
  expect(reg, '待機場所の登録が ui.register を送っていない').toHaveLength(1)
  expect(reg[0].argJson.kind).toBe('HOME')

  await page.locator('[data-testid="s20-reg-panel"]').click()
  reg = (await triggers(page)).filter((c) => c.trigger === 'ui.register')
  expect(reg, '配電盤の登録が ui.register を送っていない').toHaveLength(2)
  expect(reg[1].argJson.kind).toBe('PANEL')
})

test('2 点指示ウィザードは REGISTER のときだけ出て、index 1→2 で /onsite/two_point を呼ぶ', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, {
    pins: PINS, targets: TARGETS,
    twoPoint: { accepted: true, reject_reason_key: null, yaw: 0 }, // Step1->2 のスタブ
  })
  await page.locator('#s20').waitFor()
  await expect(page.locator('[data-testid="s20-wizard"]')).not.toBeVisible()

  // 配電盤を選んでから REGISTER へ（purpose が PANEL になることを見る）。
  await page.locator('[data-testid="s20-reg-panel"]').click()
  await setTestState(page, { state: 'REGISTER' })
  await expect(page.locator('[data-testid="s20-wizard"]')).toBeVisible()

  await page.locator('[data-testid="s20-wiz-press"]').click()
  let calls = await onsiteServiceCalls(page)
  expect(calls).toHaveLength(1)
  expect(calls[0].service).toBe('/onsite/two_point')
  expect(calls[0].request).toMatchObject({ purpose: 'PANEL', index: 1 })

  // Step 2 / 2 に進む（スタブが accepted なので）。purpose は保持される。
  await expect(page.locator('[data-testid="s20-wiz-press"]')).toBeVisible()
  await page.evaluate(() => { window.__thTestTwoPoint = { accepted: true, reject_reason_key: null, yaw: Math.PI / 2 } })
  await page.locator('[data-testid="s20-wiz-press"]').click()
  calls = await onsiteServiceCalls(page)
  expect(calls).toHaveLength(2)
  expect(calls[1].request).toMatchObject({ purpose: 'PANEL', index: 2 })
  // Step 2 受理で向き表示（yaw=90°）。
  await expect(page.locator('[data-testid="s20-wiz-done"]')).toHaveText('向き 90°')
})

test('2 点指示が拒否されたら理由を表示して Step を進めない', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, {
    pins: PINS, targets: TARGETS,
    twoPoint: { accepted: false, reject_reason_key: 'tracker_lost' },
  })
  await page.locator('#s20').waitFor()
  await page.locator('[data-testid="s20-reg-home"]').click()
  await setTestState(page, { state: 'REGISTER' })
  await page.locator('[data-testid="s20-wiz-press"]').click()
  await expect(page.locator('[data-testid="s20-wiz-err"]')).toHaveText('対象を見失っています')
  await expect(page.locator('[data-testid="s20-wiz-press"]')).toBeVisible()
})

test('停止/保存の操作カードと対象選択（radar）', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, { pins: PINS, targets: TARGETS, pose: { x: 0, y: 0, yaw: 0 } })
  await page.locator('#s20').waitFor()

  // 操作カード: 停止(ui.stop) / 保存(ui.save)。
  await page.locator('#s20 .op-stop').click()
  await page.locator('#s20 .op-save').click()
  const t = await triggers(page)
  expect(t.some((c) => c.trigger === 'ui.stop'), '停止が ui.stop を送っていない').toBe(true)
  expect(t.some((c) => c.trigger === 'ui.save'), '保存が ui.save を送っていない').toBe(true)

  // 対象選択タブ: レーダーに候補 2 件。
  await page.getByRole('tab', { name: '対象選択' }).click()
  await expect(page.locator('[data-testid="radar-cand-0"]')).toBeVisible()
  await page.locator('[data-testid="radar-cand-1"]').click()
  const sel = (await triggers(page)).find((c) => c.trigger === 'ui.select_target')
  expect(sel, 'レーダーのタップが ui.select_target を送っていない').toBeTruthy()
  expect(sel.argJson).toMatchObject({ index: 1 })
})

test('レーダーの空表示と見失い表示', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, { pins: PINS, targets: TARGETS })
  await page.locator('#s20').waitFor()
  await page.getByRole('tab', { name: '対象選択' }).click()

  await setTestPersonTargets(page, { candidates: [], selected_index: -1, confidence: 0, is_lost: false, lost_reason: '' })
  await expect(page.locator('[data-testid="radar-empty"]')).toBeVisible()

  await setTestPersonTargets(page, { ...TARGETS, is_lost: true, lost_reason: 'tracker_lost' })
  await expect(page.locator('[data-testid="radar-lost"]')).toBeVisible()
})

test('待機場所のピンが無ければ 1 ボタンで戻すは非活性、あれば ui.return_home', async ({ page }) => {
  // HOME ピン無し → 非活性。
  await gotoScreenWithOnsite(page, 'S20', PREP, {
    pins: [pin('p1', '盤 A-1', 'PANEL', 86, 88, 92)], targets: TARGETS,
  })
  await page.locator('#s20').waitFor()
  await page.getByRole('tab', { name: 'ピン' }).click()
  await expect(page.locator('[data-testid="s20-return-home"]')).toBeDisabled()

  // HOME ピン有り → クリックで ui.return_home。
  await setTestOnsitePins(page, PINS)
  await page.locator('[data-testid="s20-return-home"]').click()
  const back = (await triggers(page)).find((c) => c.trigger === 'ui.return_home')
  expect(back, '待機場所に戻すが ui.return_home を送っていない').toBeTruthy()
})

test('ピン一覧の編集（改名）と削除は /onsite/edit_pin を呼ぶ', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, { pins: PINS, targets: TARGETS })
  await page.locator('#s20').waitFor()
  await page.getByRole('tab', { name: 'ピン' }).click()

  // 一覧にピン名と向きが出ること。
  await expect(page.locator('[data-testid="s20-pin-name-p1"]')).toHaveText('盤 A-1')
  await expect(page.locator('[data-testid="s20-pin-name-p3"]')).toHaveText('待機場所')

  // 改名。
  await page.locator('[data-testid="s20-pin-edit-p1"]').click()
  await expect(page.locator('[data-testid="s20-pin-editor"]')).toBeVisible()
  await page.locator('[data-testid="s20-pin-edit-name"]').fill('盤 A-1改')
  await page.locator('[data-testid="s20-pin-rename"]').click()
  const calls = await onsiteServiceCalls(page)
  const rename = calls.find((c) => c.service === '/onsite/edit_pin')
  expect(rename, '改名が /onsite/edit_pin を呼んでいない').toBeTruthy()
  expect(rename.request).toMatchObject({ id: 'p1', new_name: '盤 A-1改', is_delete: false })

  // 削除。
  await page.locator('[data-testid="s20-pin-edit-p1"]').click()
  await page.locator('[data-testid="s20-pin-delete"]').click()
  const del = (await onsiteServiceCalls(page)).find(
    (c) => c.service === '/onsite/edit_pin' && c.request.is_delete === true)
  expect(del, '削除が is_delete:true の /onsite/edit_pin を呼んでいない').toBeTruthy()
  expect(del.request.id).toBe('p1')
})

// 空の pins で tab ピンに「ピンがありません」相当（testid s20-pins-empty）が出る。
test('ピンが 0 件なら空表示', async ({ page }) => {
  await gotoScreen(page, 'S20', PREP)
  await page.locator('#s20').waitFor()
  await page.getByRole('tab', { name: 'ピン' }).click()
  await expect(page.locator('[data-testid="s20-pins-empty"]')).toBeVisible()
})

// brief-onsite-fix C.5: 実地図（/route/map_view）が届いていれば占有格子ラスタが
// 地図タブに描かれ、ピン（map フレーム）/ ロボット（map フレーム）と重なる。
test('地図シードありで s20-map-raster が描かれピンが地図座標に載る', async ({ page }) => {
  await gotoScreenWithOnsite(
    page, 'S20', PREP,
    { pins: PINS, targets: TARGETS, pose: { x: 0, y: 0, yaw: 0 }, routeMap: ROUTE_MAP },
  )
  await page.locator('#s20').waitFor()
  // ラスタ描画は canvas→dataURL なので /route/map_view 受信後に非同期で出る。
  await expect(page.locator('[data-testid="s20-map-raster"]')).toBeVisible()
  await expect(page.locator('[data-testid="s20-map-pin-p1"]')).toBeVisible()
  await expect(page.locator('[data-testid="s20-map-robot"]')).toBeVisible()
})

test('地図シードなしならラスタは描かれない（モックアップ部屋も描かない。F-4）', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, { pins: PINS, targets: TARGETS, pose: { x: 0, y: 0, yaw: 0 } })
  await page.locator('#s20').waitFor()
  await expect(page.locator('[data-testid="s20-map-raster"]')).not.toBeVisible()
  await expect(page.locator('[data-testid="s20-map-robot"]')).toBeVisible()
})

// brief-onsite-fix F-2: レーダーに機体マーク（前方上向き三角＋中心丸）が出る。
test('レーダーに機体マーク（radar-heading）が描かれる', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, { pins: PINS, targets: TARGETS, pose: { x: 0, y: 0, yaw: 0 } })
  await page.locator('#s20').waitFor()
  await page.getByRole('tab', { name: '対象選択' }).click()
  await expect(page.locator('[data-testid="radar-heading"]')).toBeVisible()
})