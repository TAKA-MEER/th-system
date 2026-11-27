// WP-UI-07: S-21 試験の操作が本当に th_state / onsite service へ届くこと
// （__thTriggerCalls ／ __thOnsiteServiceCalls で検証）。th_state は無いので
// IDLE / PANEL_NAV / SUMMON / AT_PANEL / HOME_NAV を addInitScript で直接注入し、
// 遷移は __thSetTestState で再現する（Server 側 FSM を起動しない）。
// S-21 は「モードに関係なく直接開く」画面なので、テストは ?__thTestScreen=S21
// で開く（brief-UI-S21 §2.1・main.jsx の TEST_SCREEN 上書き）。IDLE のままでも
// 表示されることを最初のテストで確かめる。
//
// 作り直し（Spec-webui §3.11。型「走らせる」）: タブ・手順バー・待機場所の宣言は
// 無く、右列の固定 9 個と地図の吹き出しに置き換えた。このファイルの試験は
// 新しい置き場所を縛る（testid は新しい置き場所のもの）。
import { test, expect } from '@playwright/test'
import {
  gotoScreenWithOnsite, onsiteServiceCalls, setTestState,
  setTestPersonTargets, setTestWaitClear, unlockOnsiteVenueMap,
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

const IDLE = { mode: 'IDLE', state: 'NONE' }

// 最小 OccupancyGrid（S-20 と同じ。地図シードを渡すと地図にラスタが描かれる）。
const ROUTE_MAP = {
  info: {
    resolution: 0.5,
    width: 4,
    height: 4,
    origin: { position: { x: -1, y: -1 }, orientation: { x: 0, y: 0, w: 1, z: 0 } },
  },
  data: [0, 100, -1, 0, 100, 0, 0, -1, -1, 0, 100, 0, 0, -1, 100, 0],
}

const COSTMAP = {
  info: {
    resolution: 0.5,
    width: 4,
    height: 4,
    origin: { position: { x: -1, y: -1 }, orientation: { x: 0, y: 0, w: 1, z: 0 } },
  },
  data: [0, 50, -1, 0, 100, 0, 0, -1, 0, 0, 80, 0, 0, -1, 100, 40],
}

async function triggers(page) {
  return page.evaluate(() => window.__thTriggerCalls ?? [])
}

async function goto21(page, state = IDLE, extra = {}) {
  // brief-tracker-default-off §3.4: 人検出が走っている前提で seed する
  // （既定 OFF の挙動は tracker-toggle スペックで検証する）。
  const seed = {
    ...IDLE, ...state,
    tracker_enabled: state?.tracker_enabled ?? true,
  }
  await gotoScreenWithOnsite(page, 'S21', seed, { pins: PINS, targets: TARGETS, pose: { x: 0, y: 0, yaw: 0 }, ...extra })
  await page.locator('#s21').waitFor()
}

// ReasonButton は押せない間も disabled 属性を付けず aria-disabled で示す
// （押すと理由の吹き出しが出る。§3.3.3）。押せる・押せないの判定はこれで見る。
async function expectUnpressable(locator) {
  await expect(locator).toHaveAttribute('aria-disabled', 'true')
}

test('S-21 表示（IDLE のまま直接開く）: 終了・右列の固定ボタン・地図の門', async ({ page }) => {
  await goto21(page, IDLE, { openVenueMap: { success: true, message: '' } })
  await expect(page.locator('[data-testid="s21-finish"]')).toBeVisible()
  // 右列 ①「どこへ」
  await expect(page.locator('[data-testid="s21-where"]')).toContainText('行き先を選んでください')
  // 「会場地図を開く」を押すまで地図は出ない（主表示の中央にボタンだけ）。
  await expect(page.locator('[data-testid="s21-map"]')).toHaveCount(0)
  await expect(page.locator('[data-testid="s21-map-gate-msg"]')).toContainText('待機場所')
  await unlockOnsiteVenueMap(page)
  await expect(page.locator('[data-testid="s21-map-pin-p1"]')).toBeVisible()
  await expect(page.locator('[data-testid="s21-map-pin-p3"]')).toBeVisible()
  await expect(page.locator('[data-testid="s21-map-robot"]')).toBeVisible()
  // 保存は出ない（地図更新は使えない間は出さない。§3.3 (c)）。
  await expect(page.getByRole('button', { name: '保存', exact: true })).toHaveCount(0)
})

// WS-9Y: 「会場地図を開く」の前提（機体を待機場所に置いてから押す）を画面に出す。
test('WS-9Y: 会場地図を開く前に「機体を待機場所に置いてから」の案内が出る', async ({ page }) => {
  await goto21(page, IDLE, { openVenueMap: { success: true, message: '' } })
  await expect(page.locator('[data-testid="s21-map-gate-msg"]')).toContainText('待機場所')
  await unlockOnsiteVenueMap(page)
  await expect(page.locator('[data-testid="s21-map-gate-msg"]')).toHaveCount(0)
})

// MAP-1: 追従対象者は is_lost:false のときだけ地図に出る。
test('MAP-1: 追従対象者は is_lost:false のときだけ地図に出る', async ({ page }) => {
  await goto21(page, IDLE, {
    openVenueMap: { success: true, message: '' },
    personStatus: { position: { x: 1, y: 0, z: 0 }, confidence: 0.9, is_lost: false, lost_reason: '' },
  })
  await unlockOnsiteVenueMap(page)
  await expect(page.locator('[data-testid="s21-map-person"]')).toBeVisible()

  await page.evaluate(() => {
    window.__thSetTestPersonStatus({
      position: { x: 1, y: 0, z: 0 }, confidence: 0, is_lost: true, lost_reason: 'DETECTION_LOST',
    })
  })
  await expect(page.locator('[data-testid="s21-map-person"]')).toBeHidden()
})

const MINIMAL_SCAN = {
  angle_min: -Math.PI,
  angle_max: Math.PI,
  angle_increment: Math.PI / 2,
  range_min: 0,
  range_max: 10,
  ranges: [1, 1.5, 2, 0.5],
}

test('MAP-SCAN: 点群は pose があると s21-map-scan に円として出る', async ({ page }) => {
  await goto21(page, IDLE, {
    openVenueMap: { success: true, message: '' },
    scan: MINIMAL_SCAN,
  })
  await unlockOnsiteVenueMap(page)
  const group = page.locator('[data-testid="s21-map-scan"]')
  await expect(group).toBeVisible()
  await expect(group.locator('circle')).toHaveCount(MINIMAL_SCAN.ranges.length)
})

test('MAP-SCAN: pose を渡さないとき点群は出ない', async ({ page }) => {
  await goto21(page, IDLE, {
    openVenueMap: { success: true, message: '' },
    scan: MINIMAL_SCAN,
    pose: undefined,
  })
  await unlockOnsiteVenueMap(page)
  await expect(page.locator('[data-testid="s21-map-scan"]')).toHaveCount(0)
})

test('終了は ui.finish が 1 回だけ送られる', async ({ page }) => {
  await goto21(page)
  await page.locator('[data-testid="s21-finish"]').click()
  const finish = (await triggers(page)).filter((c) => c.trigger === 'ui.finish')
  expect(finish, '終了が ui.finish を送っていない').toHaveLength(1)
})

// 停止│走行（StopRunToggle）。押せる側だけ塗る（止める黒・走る緑）。
test('停止│走行: PANEL_NAV/PAUSE は再開するが押せて ui.run、AT_PANEL では押せない', async ({ page }) => {
  await goto21(page, { mode: 'PANEL_NAV', state: 'PAUSE' })
  const run = page.locator('[data-testid="stop-run-toggle-run"]')
  await expect(run).toBeEnabled()
  await expect(run).toHaveText('再開する')
  await run.click()
  expect(
    (await triggers(page)).some((c) => c.trigger === 'ui.run'),
    '再開が ui.run を送っていない',
  ).toBe(true)

  await goto21(page, { mode: 'AT_PANEL', state: 'IDLE_P' })
  await expect(page.locator('[data-testid="stop-run-toggle-run"]')).toBeDisabled()
  await expect(page.locator('[data-testid="stop-run-toggle-stop"]')).toBeDisabled()
})

test('停止│走行: 走行中は停止するが押せて ui.stop', async ({ page }) => {
  await goto21(page, { mode: 'PANEL_NAV', state: 'NAV' })
  const stop = page.locator('[data-testid="stop-run-toggle-stop"]')
  await expect(stop).toBeEnabled()
  await stop.click()
  expect((await triggers(page)).some((c) => c.trigger === 'ui.stop'), '停止が ui.stop を送っていない').toBe(true)
})

// W-5: BLOCKED は窓（再検索／中断）で出す。帯ではない。
test('W-5: PANEL_NAV/BLOCKED で窓が出て「再検索」→ui.reroute、「中断」→ui.abort', async ({ page }) => {
  await goto21(page, { mode: 'PANEL_NAV', state: 'BLOCKED' })
  await expect(page.locator('[data-testid="s21-blocked"]')).toBeVisible()
  await expect(page.locator('[data-testid="s21-blocked"]')).toContainText('経路が塞がれています')
  await page.locator('[data-testid="s21-blocked-reroute"]').click()
  expect((await triggers(page)).some((c) => c.trigger === 'ui.reroute'),
    '再検索が ui.reroute を送っていない').toBe(true)
  await page.locator('[data-testid="s21-blocked-abort"]').click()
  expect((await triggers(page)).some((c) => c.trigger === 'ui.abort'),
    '中断が ui.abort を送っていない').toBe(true)

  await goto21(page, { mode: 'PANEL_NAV', state: 'NAV' })
  await expect(page.locator('[data-testid="s21-blocked"]')).toBeHidden()
})

// D12: 中断は走行中（NAV・ALIGN）も出す。
for (const [mode, state] of [['PANEL_NAV', 'PAUSE'], ['HOME_NAV', 'PAUSE'], ['SUMMON', 'PAUSE'], ['PANEL_NAV', 'NAV'], ['PANEL_NAV', 'ALIGN']]) {
  test(`中断は ${mode}/${state} で見えて押すと ui.abort`, async ({ page }) => {
    await goto21(page, { mode, state })
    const ab = page.locator('[data-testid="s21-abort"]')
    await expect(ab).toBeVisible()
    await ab.click()
    expect((await triggers(page)).some((c) => c.trigger === 'ui.abort'),
      '中断が ui.abort を送っていない').toBe(true)
  })
}

test('中断は行き先を選べない状態（AT_PANEL）では押せない理由が出る', async ({ page }) => {
  await goto21(page, { mode: 'AT_PANEL', state: 'IDLE_P' })
  const ab = page.locator('[data-testid="s21-abort"]')
  await expectUnpressable(ab)
  await ab.click({ force: true })
  await expect(page.locator('[data-testid="s21-abort-reason"]')).toContainText('押せません')
  expect((await triggers(page)).filter((c) => c.trigger === 'ui.abort'), '押せないのに ui.abort が送られている').toHaveLength(0)
})

// 地図の吹き出し: ピンを押すと「行き先にする」、押すとそのまま出発する。
// 地図のピンの吹き出し（クリックできる座標のピンを使う。既定 PINS は far なので差し替える。）
const NEAR_PINS = [
  pin('p1', '盤 A-1', 'PANEL', 1, 1),
  pin('p2', '盤 A-2', 'PANEL', 2, 2),
  pin('p3', '待機場所', 'HOME', -1, -1),
]

test('地図の吹き出し: ピン→「行き先にする」→ select_pin + ui.goto{PANEL}', async ({ page }) => {
  await goto21(page, IDLE, { openVenueMap: { success: true, message: '' }, selectPin: { success: true, message: '' }, pins: NEAR_PINS })
  await unlockOnsiteVenueMap(page)
  await page.locator('[data-testid="s21-map-pin-p1"]').click()
  await expect(page.locator('[data-testid="s21-pin-bubble"]')).toBeVisible()
  await page.locator('[data-testid="s21-pin-go"]').click()
  const calls = (await onsiteServiceCalls(page)).filter((c) => c.service === '/onsite/select_pin')
  expect(calls).toHaveLength(1)
  expect(calls[0].request).toMatchObject({ panel_id: 'p1' })
  const gotos = (await triggers(page)).filter((c) => c.trigger === 'ui.goto')
  expect(gotos).toHaveLength(1)
  expect(gotos[0].argJson).toMatchObject({ kind: 'PANEL' })
  // 出発したら吹き出しは消える。
  await expect(page.locator('[data-testid="s21-pin-bubble"]')).toHaveCount(0)
})

test('地図の吹き出し: 待機場所ピンはそのまま ui.goto{HOME}', async ({ page }) => {
  await goto21(page, IDLE, { openVenueMap: { success: true, message: '' }, pins: NEAR_PINS })
  await unlockOnsiteVenueMap(page)
  await page.locator('[data-testid="s21-map-pin-p3"]').click()
  await page.locator('[data-testid="s21-pin-go"]').click()
  const gotos = (await triggers(page)).filter((c) => c.trigger === 'ui.goto')
  expect(gotos).toHaveLength(1)
  expect(gotos[0].argJson).toMatchObject({ kind: 'HOME' })
  expect((await onsiteServiceCalls(page)).filter((c) => c.service === '/onsite/select_pin'),
    '待機場所へは select_pin を送っていない').toHaveLength(0)
})

test('地図の吹き出し: ピン以外を押すと消える', async ({ page }) => {
  await goto21(page, IDLE, { openVenueMap: { success: true, message: '' }, pins: NEAR_PINS })
  await unlockOnsiteVenueMap(page)
  await page.locator('[data-testid="s21-map-pin-p1"]').click()
  await expect(page.locator('[data-testid="s21-pin-bubble"]')).toBeVisible()
  await page.locator('[data-testid="s21-map"]').click({ position: { x: 5, y: 5 } })
  await expect(page.locator('[data-testid="s21-pin-bubble"]')).toHaveCount(0)
})

test('地図の吹き出し: select_pin が拒否されたらメッセージを出し、ui.goto を送らない', async ({ page }) => {
  await goto21(page, IDLE, { openVenueMap: { success: true, message: '' }, selectPin: { success: false, message: 'そのピンには行けません' }, pins: NEAR_PINS })
  await unlockOnsiteVenueMap(page)
  await page.locator('[data-testid="s21-map-pin-p1"]').click()
  await page.locator('[data-testid="s21-pin-go"]').click()
  await expect(page.locator('[data-testid="s21-go-err"]')).toContainText('そのピンには行けません')
  expect((await triggers(page)).filter((c) => c.trigger === 'ui.goto'), 'reject なのに ui.goto を送っている').toHaveLength(0)
})

// SM-3.1.2-118: 移動中・一時停止中は行き先を押せない（押すと理由が出る）。
for (const [mode, state, text] of [
  ['PANEL_NAV', 'PAUSE', '一時停止中は選べません'],
  ['HOME_NAV', 'PAUSE', '一時停止中は選べません'],
  ['PANEL_NAV', 'NAV', '移動中は行き先を選べません'],
]) {
  test(`行き先は押せない: ${mode}/${state}（次の配電盤・待機場所・呼び寄せ）`, async ({ page }) => {
    await goto21(page, { mode, state }, { openVenueMap: { success: true, message: '' }, selectPin: { success: true, message: '' } })
    for (const tid of ['s21-dest-next-panel', 's21-dest-home', 's21-dest-summon-here']) {
      const btn = page.locator(`[data-testid="${tid}"]`)
      await expectUnpressable(btn)
      await btn.click({ force: true })
      await expect(page.locator(`[data-testid="reason-${tid}"]`)).toContainText(text)
    }
    expect((await onsiteServiceCalls(page)).filter((c) => c.service === '/onsite/select_pin'),
      '押せないはずなのに select_pin が呼ばれている').toHaveLength(0)
    expect((await triggers(page)).filter((c) => c.trigger === 'ui.goto'),
      '押せないはずなのに ui.goto が送られている').toHaveLength(0)
  })
}

// SM-3.1.2-119: 会場地図を持たない間は行き先と「走行」を押せない。
const unloaded = (page) => page.evaluate(() => window.__thSetTestMapSession(
  { slot: 'VENUE', session_id: 'venue', mode: 'UNLOADED', dirty: false, instance_id: 'x' }))
const loaded = (page) => page.evaluate(() => window.__thSetTestMapSession(
  { slot: 'VENUE', session_id: 'venue', mode: 'LOCALIZING', dirty: false, instance_id: 'x' }))
for (const [mode, state] of [['IDLE', 'NONE'], ['AT_HOME', 'IDLE_H'], ['AT_PANEL', 'IDLE_P']]) {
  test(`会場地図なし: ${mode}/${state} は行き先を押せず、押すと理由が出る`, async ({ page }) => {
    await goto21(page, { mode, state }, { selectPin: { success: true, message: '' } })
    await unloaded(page)
    for (const tid of ['s21-dest-next-panel', 's21-dest-home', 's21-dest-summon-here']) {
      const btn = page.locator(`[data-testid="${tid}"]`)
      await expectUnpressable(btn)
      await btn.click({ force: true })
      await expect(page.locator(`[data-testid="reason-${tid}"]`)).toContainText('会場地図がありません')
    }
    expect((await onsiteServiceCalls(page)).filter((c) => c.service === '/onsite/select_pin')).toHaveLength(0)
    await loaded(page)
    await expect(page.locator('[data-testid="s21-dest-home"]')).not.toHaveAttribute('aria-disabled', 'true')
  })
}

test('会場地図なし: PANEL_NAV/PAUSE の再開するは押せない（読み込み済みなら押せて ui.run）', async ({ page }) => {
  await goto21(page, { mode: 'PANEL_NAV', state: 'PAUSE' })
  const run = page.locator('[data-testid="stop-run-toggle-run"]')
  await expect(run).toBeEnabled()
  await unloaded(page)
  await expect(run).toBeDisabled()
  await expect(page.locator('[data-testid="stop-run-toggle-run-reason"]')).toContainText('会場地図がありません')
  await loaded(page)
  await expect(run).toBeEnabled()
  await run.click()
  expect((await triggers(page)).filter((c) => c.trigger === 'ui.run')).toHaveLength(1)
})

test('次の配電盤: 順番が次に来る盤へそのまま出発する（行き先の名前入り）', async ({ page }) => {
  // 機体が p1（盤 A-1）の上に居る（到着）ので次は p2（盤 A-2）。
  await goto21(page, IDLE, { pose: { x: 86, y: 88, yaw: 0 }, selectPin: { success: true, message: '' } })
  await expect(page.locator('[data-testid="s21-next-panel-name"]')).toContainText('盤 A-2')
  await page.locator('[data-testid="s21-dest-next-panel"]').click()
  const calls = await onsiteServiceCalls(page)
  expect(calls[0].service).toBe('/onsite/select_pin')
  expect(calls[0].request).toMatchObject({ panel_id: 'p2' })
  const gotos = (await triggers(page)).filter((c) => c.trigger === 'ui.goto')
  expect(gotos).toHaveLength(1)
  expect(gotos[0].argJson).toMatchObject({ kind: 'PANEL' })
})

test('次の行き先: 待機場所→ui.goto{HOME}、呼び寄せ→ui.goto{SUMMON}', async ({ page }) => {
  await goto21(page)
  await page.locator('[data-testid="s21-dest-home"]').click()
  let gotos = (await triggers(page)).filter((c) => c.trigger === 'ui.goto')
  expect(gotos).toHaveLength(1)
  expect(gotos[0].argJson).toMatchObject({ kind: 'HOME' })

  await page.locator('[data-testid="s21-dest-summon-here"]').click()
  gotos = (await triggers(page)).filter((c) => c.trigger === 'ui.goto')
  expect(gotos).toHaveLength(2)
  expect(gotos[1].argJson).toMatchObject({ kind: 'SUMMON' })
})

test('呼び寄せ: SUMMON/POINT は主表示に帯とレーダーが出て 2 点指示が purpose=SUMMON で index 1→2', async ({ page }) => {
  await goto21(page, IDLE, { twoPoint: { accepted: true, reject_reason_key: null, yaw: 0 }, openVenueMap: { success: true, message: '' } })
  await unlockOnsiteVenueMap(page)
  await page.locator('[data-testid="s21-dest-summon-here"]').click()
  await setTestState(page, { mode: 'SUMMON', state: 'POINT' })
  await expect(page.locator('[data-testid="s21-summon-band"]')).toBeVisible()
  await expect(page.locator('[data-testid="radar"]')).toBeVisible()
  await expect(page.locator('[data-testid="s21-wizard"]')).toBeVisible()

  await page.locator('[data-testid="s21-wiz-press"]').click()
  let calls = await onsiteServiceCalls(page)
  const summon = calls.filter((c) => c.service === '/onsite/two_point')
  expect(summon).toHaveLength(1)
  expect(summon[0].request).toMatchObject({ purpose: 'SUMMON', index: 1 })

  await page.evaluate(() => { window.__thTestTwoPoint = { accepted: true, reject_reason_key: null, yaw: Math.PI / 2 } })
  await page.locator('[data-testid="s21-wiz-press"]').click()
  calls = await onsiteServiceCalls(page)
  expect(calls.filter((c) => c.service === '/onsite/two_point')).toHaveLength(2)
  expect(calls.filter((c) => c.service === '/onsite/two_point')[1].request).toMatchObject({ purpose: 'SUMMON', index: 2 })
  await expect(page.locator('[data-testid="s21-wiz-done"]')).toHaveText('向き 90°')
})

test('呼び寄せ: 対象を選ばなければ記録は押せない（対象が見えないまま実行させない）', async ({ page }) => {
  await goto21(page, { mode: 'SUMMON', state: 'POINT' }, { targets: { ...TARGETS, selected_index: -1 }, twoPoint: { accepted: true, reject_reason_key: null, yaw: 0 } })
  const btn = page.locator('[data-testid="s21-wiz-press"]')
  await expectUnpressable(btn)
  await btn.click({ force: true })
  await expect(page.locator('[data-testid="s21-wiz-reason"]')).toContainText('対象を選んでから')
  expect((await onsiteServiceCalls(page)).filter((c) => c.service === '/onsite/two_point'),
    '対象無しで two_point が呼ばれている').toHaveLength(0)
  await setTestPersonTargets(page, TARGETS)
  await expect(btn).toBeEnabled()
})

test('退避待ち: WAIT_CLEAR で進捗ボックスが出て、中止は ui.abort', async ({ page }) => {
  await goto21(page, { mode: 'SUMMON', state: 'WAIT_CLEAR' }, { waitClear: { distance_m: 0.4, remaining_sec: 7, satisfied: false, verdict: 'WAITING' } })
  await expect(page.locator('[data-testid="s21-clear-box"]')).toBeVisible()
  await expect(page.locator('[data-testid="s21-clear-dist"]')).toHaveText('0.4 m')

  await setTestWaitClear(page, { distance_m: 0.2, remaining_sec: 3, satisfied: false, verdict: 'WAITING' })
  await expect(page.locator('[data-testid="s21-clear-dist"]')).toHaveText('0.2 m')

  await setTestWaitClear(page, { distance_m: 0.1, remaining_sec: 0, satisfied: true, verdict: 'OK' })
  await expect(page.locator('[data-testid="s21-clear-bar"].ok')).toBeAttached()

  await page.locator('[data-testid="s21-wait-cancel"]').click()
  const abort = (await triggers(page)).filter((c) => c.trigger === 'ui.abort')
  expect(abort, '中止が ui.abort を送っていない').toHaveLength(1)
})

const CAMERA_STUBS = {
  get: { effective: { camera_ui_url: 'http://192.168.5.129/' }, pending: {} },
  set: { success: true, message: 'OK' },
}

async function stubWindowOpen(page) {
  await page.addInitScript(() => {
    window.__thOpened = []
    window.open = (url, target, features) => {
      window.__thOpened.push({ url, target, features })
      return null
    }
  })
}

async function opened(page) {
  return page.evaluate(() => window.__thOpened ?? [])
}

test('「カメラ」は /params/get の開く先を別タブで開き、機体へ何も送らない', async ({ page }) => {
  await stubWindowOpen(page)
  await goto21(page, { mode: 'AT_PANEL', state: 'IDLE_P' }, { paramsStubs: CAMERA_STUBS })
  const btn = page.locator('[data-testid="s21-camera-open"]')
  await expect(btn).toBeEnabled()
  await btn.click()
  const urls = await opened(page)
  expect(urls).toHaveLength(1)
  expect(urls[0]).toEqual({ url: 'http://192.168.5.129/', target: '_blank', features: 'noopener' })
  const w = (await triggers(page)).filter((c) => c.trigger === 'ui.working')
  expect(w, '「カメラ」が ui.working を送っている').toHaveLength(0)
})

test('「カメラ」は AT_PANEL 以外では押せない', async ({ page }) => {
  await stubWindowOpen(page)
  await goto21(page, IDLE, { paramsStubs: CAMERA_STUBS })
  await page.locator('[data-testid="s21-camera-open"]').click({ force: true })
  await expect(page.locator('[data-testid="s21-camera-reason"]')).toContainText('押せません')
  await expect(await opened(page)).toHaveLength(0)
})

test('「カメラ」の開く先が http(s) 以外なら押せなく理由を出す', async ({ page }) => {
  await stubWindowOpen(page)
  await goto21(page, { mode: 'AT_PANEL', state: 'IDLE_P' }, {
    paramsStubs: { get: { effective: { camera_ui_url: 'ftp://192.168.5.129/' }, pending: {} } },
  })
  await page.locator('[data-testid="s21-camera-open"]').click({ force: true })
  await expect(page.locator('[data-testid="s21-camera-reason"]')).toContainText('http://')
  expect((await triggers(page)).filter((c) => c.trigger === 'ui.working')).toHaveLength(0)
})

test('「測定機能へ」は盤の前で停止命令（ui.working）を送る', async ({ page }) => {
  await goto21(page, { mode: 'AT_PANEL', state: 'IDLE_P' })
  await page.locator('[data-testid="s21-measure"]').click()
  const w = (await triggers(page)).filter((c) => c.trigger === 'ui.working')
  expect(w, '測定機能へが ui.working を送っていない').toHaveLength(1)
})

test('「停止を解く」は停止命令の間だけ出て、W-4 の確認のあと ui.working で解く', async ({ page }) => {
  await goto21(page, { mode: 'AT_PANEL', state: 'WORKING' })
  await expect(page.locator('[data-testid="s21-measure"]')).toHaveCount(0)
  await page.locator('[data-testid="s21-unstop"]').click()
  await expect(page.locator('[data-testid="s21-unstop-confirm"]')).toBeVisible()
  await page.locator('[data-testid="s21-unstop-confirm"]').click()
  const w = (await triggers(page)).filter((c) => c.trigger === 'ui.working')
  expect(w, '停止を解くが ui.working を送っていない').toHaveLength(1)
})

test('対象選択レーダー: SUMMON/POINT は主表示に出て ui.select_target を送る', async ({ page }) => {
  await goto21(page, IDLE, { openVenueMap: { success: true, message: '' } })
  await unlockOnsiteVenueMap(page)
  await setTestState(page, { mode: 'SUMMON', state: 'POINT' })
  await expect(page.locator('[data-testid="radar-cand-0"]')).toBeVisible()
  await page.locator('[data-testid="radar-cand-1"]').click()
  const sel = (await triggers(page)).find((c) => c.trigger === 'ui.select_target')
  expect(sel, 'SUMMON のレーダーが ui.select_target を送っていない').toBeTruthy()
  expect(sel.argJson).toMatchObject({ index: 1 })
})

test('レーダーの空表示と見失い表示', async ({ page }) => {
  await goto21(page, IDLE, { openVenueMap: { success: true, message: '' } })
  await unlockOnsiteVenueMap(page)
  await setTestState(page, { mode: 'SUMMON', state: 'POINT' })
  await setTestPersonTargets(page, { candidates: [], selected_index: -1, confidence: 0, is_lost: false, lost_reason: '' })
  await expect(page.locator('[data-testid="radar-empty"]')).toBeVisible()

  await setTestPersonTargets(page, { ...TARGETS, is_lost: true, lost_reason: 'tracker_lost' })
  await expect(page.locator('[data-testid="radar-lost-reselect"]')).toBeVisible()
  await expect(page.locator('[data-testid="radar-cand-0"]')).toBeVisible()
  await expect(page.locator('[data-testid="radar-lost"]')).toHaveCount(0)

  await setTestPersonTargets(page, { candidates: [], selected_index: -1, confidence: 0, is_lost: true, lost_reason: 'tracker_lost' })
  await expect(page.locator('[data-testid="radar-lost"]')).toBeVisible()
})

test('見失った状態でも候補をタップして選び直せる（SUMMON/POINT）', async ({ page }) => {
  await goto21(page, IDLE, { openVenueMap: { success: true, message: '' }, targets: { ...TARGETS, is_lost: true } })
  await unlockOnsiteVenueMap(page)
  await setTestState(page, { mode: 'SUMMON', state: 'POINT' })
  await expect(page.locator('[data-testid="radar-lost-reselect"]')).toBeVisible()
  await expect(page.locator('[data-testid="radar-cand-1"]')).toBeVisible()
  await page.locator('[data-testid="radar-cand-1"]').click()
  const sel = (await triggers(page)).find((c) => c.trigger === 'ui.select_target')
  expect(sel, '見失い中のタップが ui.select_target を送っていない').toBeTruthy()
  expect(sel.argJson).toMatchObject({ index: 1 })
})

test('地図シードありで s21-map-raster が描かれピンが地図座標に載る', async ({ page }) => {
  await goto21(page, IDLE, { routeMap: ROUTE_MAP, openVenueMap: { success: true, message: '' } })
  await unlockOnsiteVenueMap(page)
  await expect(page.locator('[data-testid="s21-map-raster"]')).toBeVisible()
  await expect(page.locator('[data-testid="s21-map-pin-p1"]')).toBeVisible()
  await expect(page.locator('[data-testid="s21-map-robot"]')).toBeVisible()
})

test('「保存した会場地図を開く」を押すまで s21-map は存在せず、押すと現れる', async ({ page }) => {
  await goto21(page, IDLE, { openVenueMap: { success: true, message: '' } })
  await expect(page.locator('[data-testid="s21-map"]')).toHaveCount(0)
  await expect(page.locator('[data-testid="s21-map-gate-msg"]')).toBeVisible()
  await unlockOnsiteVenueMap(page)
  await expect(page.locator('[data-testid="s21-map"]')).toBeVisible()
})

test('保存した会場地図を開くが /map_session/open を呼ぶ', async ({ page }) => {
  await goto21(page)
  const btn = page.locator('[data-testid="s21-map-gate-open"]')
  await expect(btn).toBeVisible()
  await btn.click()
  const open = (await onsiteServiceCalls(page)).find(
    (c) => c.service === '/map_session/open')
  expect(open, '「保存した会場地図を開く」が /map_session/open を呼んでいない').toBeTruthy()
  expect(open.request).toMatchObject({ slot: 'VENUE', mode: 'reload', session_id: 'venue' })
})

test('MAP-COSTMAP: costmap と Nav2 経路が地図に重なる', async ({ page }) => {
  await goto21(page, IDLE, {
    routeMap: ROUTE_MAP,
    openVenueMap: { success: true, message: '' },
    costmap: COSTMAP,
    plannedPath: [{ x: -0.5, y: -0.5 }, { x: 0.5, y: 0.5 }, { x: 1, y: 1 }],
  })
  await unlockOnsiteVenueMap(page)
  await expect(page.locator('[data-testid="s21-map-raster"]')).toBeVisible()
  await expect(page.locator('[data-testid="s21-map-costmap"]')).toBeVisible()
  await expect(page.locator('[data-testid="s21-map-path"]')).toBeVisible()
})

test('MAP-COSTMAP: costmap/経路を seed していなければ何も描かない', async ({ page }) => {
  await goto21(page, IDLE, { routeMap: ROUTE_MAP, openVenueMap: { success: true, message: '' } })
  await unlockOnsiteVenueMap(page)
  await expect(page.locator('[data-testid="s21-map-raster"]')).toBeVisible()
  await expect(page.locator('[data-testid="s21-map-costmap"]')).toHaveCount(0)
  await expect(page.locator('[data-testid="s21-map-path"]')).toHaveCount(0)
})

test('右列 ①: 経路があれば「残り」の距離が出る', async ({ page }) => {
  await goto21(page, { mode: 'PANEL_NAV', state: 'NAV' }, {
    plannedPath: [{ x: 0, y: 0 }, { x: 3, y: 0 }, { x: 3, y: 4 }],
  })
  await expect(page.locator('[data-testid="s21-where-remain"]')).toHaveText('残り 7.0 m')
})

// SG-C8: 退避待ちのバーは、ノードが出す remaining_sec の最初の値を全体とする。
for (const total of [30, 60]) {
  test(`退避待ち: バーの全体はノードの残り時間の初回値（${total}s）から取る`, async ({ page }) => {
    await goto21(page, { mode: 'SUMMON', state: 'WAIT_CLEAR' }, { waitClear: { distance_m: 0.4, remaining_sec: total, satisfied: false, verdict: 'WAITING' } })
    await expect(page.locator('[data-testid="s21-clear-box"]')).toBeVisible()
    await setTestWaitClear(page, { distance_m: 0.4, remaining_sec: total / 2, satisfied: false, verdict: 'WAITING' })
    const w = async () => page.locator('[data-testid="s21-clear-bar"] i').evaluate((el) => el.style.width)
    await expect.poll(w).toBe('50%')
  })
}

test('退避待ち: 見失い中は距離を「—」にする', async ({ page }) => {
  await goto21(page, { mode: 'SUMMON', state: 'WAIT_CLEAR' }, { waitClear: { distance_m: 0.4, remaining_sec: 20, satisfied: false, verdict: 'NOT_CLEAR' } })
  await expect(page.locator('[data-testid="s21-clear-dist"]')).toHaveText('0.4 m')
  await setTestWaitClear(page, { distance_m: -1, remaining_sec: 19, satisfied: false, verdict: 'NOT_CLEAR' })
  await expect(page.locator('[data-testid="s21-clear-dist"]')).toHaveText('—')
})

test('MAP-COSTMAP: 非表示設定では seed しても costmap を描かない', async ({ page }) => {
  await page.addInitScript(() => {
    window.localStorage.setItem('th.costmapVisible', '0')
  })
  await goto21(page, IDLE, {
    routeMap: ROUTE_MAP,
    openVenueMap: { success: true, message: '' },
    costmap: COSTMAP,
    plannedPath: [{ x: -0.5, y: -0.5 }, { x: 0.5, y: 0.5 }, { x: 1, y: 1 }],
  })
  await unlockOnsiteVenueMap(page)
  await expect(page.locator('[data-testid="s21-map-raster"]')).toBeVisible()
  await expect(page.locator('[data-testid="s21-map-costmap"]')).toHaveCount(0)
  await expect(page.locator('[data-testid="s21-map-path"]')).toBeVisible()
})

// data-pos: 状態を変えても右列のボタンの位置は動かない（§3.3）。
test('data-pos: 状態を変えても右列のボタンの位置が動かない', async ({ page }) => {
  await goto21(page, IDLE, { openVenueMap: { success: true, message: '' } })
  await unlockOnsiteVenueMap(page)
  const ids = ['s21-finish', 'stop-run-toggle-stop', 'stop-run-toggle-run', 's21-manual', 's21-abort', 's21-dest-home', 's21-dest-next-panel', 's21-dest-summon-here', 's21-measure', 's21-camera-open']
  // ヘッダの札の高さは状態で変わる（本文ごと下がりうる）ので、位置は
  // 終了ボタン基準の相対位置で比べる（右列内部での並びが動かないこと）。
  const rel = async () => {
    const out = {}
    const base = await page.locator('[data-testid="s21-finish"]').boundingBox()
    for (const id of ids) {
      const b = await page.locator(`[data-testid="${id}"]`).boundingBox()
      out[id] = { x: b.x - base.x, y: b.y - base.y, width: b.width, height: b.height }
    }
    return out
  }
  const before = await rel()
  const samePlace = (a, b) => Math.abs(a.x - b.x) <= 2 && Math.abs(a.y - b.y) <= 2 && Math.abs(a.width - b.width) <= 2 && Math.abs(a.height - b.height) <= 2
  await setTestState(page, { mode: 'PANEL_NAV', state: 'NAV' })
  const afterNav = await rel()
  for (const id of ids) {
    expect(samePlace(afterNav[id], before[id]), `${id} の位置が動いている`).toBe(true)
  }
  await setTestState(page, { mode: 'PANEL_NAV', state: 'PAUSE' })
  const afterPause = await rel()
  for (const id of ids) {
    expect(samePlace(afterPause[id], before[id]), `${id} の位置が動いている`).toBe(true)
  }
})

test('ヘッダの札: IDLE_H は記号でなく「待機中」を出し、表に無い状態は札を出さない（Spec-webui §8）', async ({ page }) => {
  await goto21(page, { mode: 'AT_HOME', state: 'IDLE_H' })
  const pill = page.locator('#modePill')
  await expect(pill).toHaveText('待機中')
  await expect(pill).toHaveAttribute('data-tone', 'neutral')

  await setTestState(page, { mode: 'AT_HOME', state: 'NO_SUCH_STATE' })
  await expect(page.locator('#modePill')).toHaveCount(0)
})

test('右列 ①: 行き先を特定できないまま NAV でも「呼び寄せの位置」でなく向かっている盤の名を出す', async ({ page }) => {
  // ui.goto を送っていない（行き先の特定が無い）。経路の終端（86,88）に最も近い盤は p1（盤 A-1）。
  await goto21(page, { mode: 'PANEL_NAV', state: 'NAV' }, {
    plannedPath: [{ x: 0, y: 0 }, { x: 86, y: 88 }],
  })
  await expect(page.locator('[data-testid="s21-where"]')).toContainText('盤 A-1 へ')
  await expect(page.locator('[data-testid="s21-where"]')).not.toContainText('呼び寄せの位置')
})

test('ボタンに矢印・四角の印は付かない（アイコンは例外だけ。§3.3.7）', async ({ page }) => {
  await goto21(page, IDLE)
  const ops = page.locator('[data-testid="s21-dest-home"], [data-testid="s21-dest-next-panel"], [data-testid="s21-dest-summon-here"], [data-testid="s21-measure"], [data-testid="s21-camera-open"], [data-testid="s21-abort"]')
  await expect(ops.locator('svg')).toHaveCount(0)
  await expect(page.locator('[data-testid="s21-dest-summon-here"]')).toContainText('呼び寄せ')
})
