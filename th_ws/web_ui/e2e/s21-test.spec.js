// WP-UI-07: S-21 試験の操作が本当に th_state / onsite service へ届くこと
// （__thTriggerCalls ／ __thOnsiteServiceCalls で検証）。th_state は無いので
// IDLE / PANEL_NAV / SUMMON / AT_PANEL / HOME_NAV を addInitScript で直接注入し、
// 遷移は __thSetTestState で再現する（Server 側 FSM を起動しない）。
// S-21 は「モードに関係なく直接開く」画面なので、テストは ?__thTestScreen=S21
// で開く（brief-UI-S21 §2.1・main.jsx の TEST_SCREEN 上書き）。IDLE のままでも
// 表示されることを最初のテストで確かめる。
import { test, expect } from '@playwright/test'
import {
  gotoScreenWithOnsite, onsiteServiceCalls, setTestState,
  setTestPersonTargets, setTestWaitClear, setTestHomeDeclared, unlockOnsiteVenueMap,
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

// S-21 の最低限の地図入力（home_declarer は未宣言 → ピン 2 本）。
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

// 最小 OccupancyGrid（S-20 と同じ。地図シードを渡すと地図タブにラスタが描かれる）。
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

async function goto21(page, state = IDLE, extra = {}) {
  await gotoScreenWithOnsite(page, 'S21', state, { pins: PINS, targets: TARGETS, pose: { x: 0, y: 0, yaw: 0 }, ...extra })
  await page.locator('#s21').waitFor()
}

test('S-21 表示とタイトル（IDLE のまま直接開く）', async ({ page }) => {
  await goto21(page, IDLE, { openVenueMap: { success: true, message: '' } })
  await expect(page.locator('main')).toContainText('試験')
  await expect(page.locator('[data-testid="s21-finish"]')).toBeVisible()
  // IDLE では「行き先を選んでください」を状態欄に出す（UX-5: タブ列右のピルは
  // 状態欄と同じ文字列を二重に出していたので削った）。
  await expect(page.locator('[data-testid="s21-state"]')).toHaveText('行き先を選んでください')
  // brief-onsite-ux2 F-6: 地図タブは「保存した会場地図を開く」を押すまで
  // OnsiteMap をマウントしない。押した後にピン 3 個とロボットマーカーが出る。
  await unlockOnsiteVenueMap(page)
  await expect(page.locator('[data-testid="s21-map-pin-p1"]')).toBeVisible()
  await expect(page.locator('[data-testid="s21-map-pin-p3"]')).toBeVisible()
  await expect(page.locator('[data-testid="s21-map-robot"]')).toBeVisible()
  // 地図更新は OFF 固定。UX-5 で行（ラベル＋非活性ボタン）から操作カード脇の
  // 小さな印（pill）に変えた。保存は出ない（map_update=false）。
  await expect(page.locator('[data-testid="s21-map-update-pill"]')).toHaveText('地図の更新 OFF')
  await expect(page.locator('#s21 .op-save')).toBeHidden()
})

test('終了は ui.finish が 1 回だけ送られる', async ({ page }) => {
  await goto21(page)
  await page.locator('[data-testid="s21-finish"]').click()
  const finish = (await triggers(page)).filter((c) => c.trigger === 'ui.finish')
  expect(finish, '終了が ui.finish を送っていない').toHaveLength(1)
})

test('操作カードの停止は ui.stop、map_update のときだけ保存が表示され ui.save', async ({ page }) => {
  await goto21(page, { mode: 'PANEL_NAV', state: 'NAV' })
  await page.locator('#s21 .op-stop').click()
  const t = await triggers(page)
  expect(t.some((c) => c.trigger === 'ui.stop'), '停止が ui.stop を送っていない').toBe(true)
  await expect(page.locator('#s21 .op-save')).toBeHidden()

  // 地図更新が ON になった体制（今の画面は OFF 固定だが、FSM が map_update を
  // 立てた場合の表示切り替え）: 保存が出て、押すと ui.save。
  await setTestState(page, { map_update: true })
  await expect(page.locator('#s21 .op-save')).toBeVisible()
  await page.locator('#s21 .op-save').click()
  expect((await triggers(page)).some((c) => c.trigger === 'ui.save'), '保存が ui.save を送っていない').toBe(true)
})

test('待機場所の宣言: 失敗で force 再宣言を促し、成功で宣言済みになる', async ({ page }) => {
  await goto21(page, IDLE, { declareHome: { success: false, message: '待機場所は別の場所にあります' } })
  await expect(page.locator('[data-testid="s21-home-pill"]')).toHaveText('未宣言')

  // 失敗 → メッセージと「強制的に宣言」が出る。
  await page.locator('[data-testid="s21-home-declare"]').click()
  await expect(page.locator('[data-testid="s21-home-err"]')).toContainText('待機場所は別の場所にあります')
  let calls = await onsiteServiceCalls(page)
  expect(calls[0].service).toBe('/onsite/declare_home')
  expect(calls[0].request).toMatchObject({ force: false })

  // force 再宣言（スタブを成功に差し替え）→ /onsite/declare_home の force:true。
  await page.evaluate(() => { window.__thTestDeclareHome = { success: true, offset_m: 0, offset_deg: 0, message: '' } })
  await page.locator('[data-testid="s21-home-force"]').click()
  calls = await onsiteServiceCalls(page)
  expect(calls).toHaveLength(2)
  expect(calls[1].service).toBe('/onsite/declare_home')
  expect(calls[1].request).toMatchObject({ force: true })
  expect((await triggers(page)).length, '宣言時に ui.goto などの誤 trigger を送っていない').toBe(0)

  // home_declared を受信した前提（宣言済み）に変えるとピルが切り替わる。
  await setTestHomeDeclared(page, true)
  await expect(page.locator('[data-testid="s21-home-pill"]')).toHaveText('宣言済み')
  await expect(page.locator('[data-testid="s21-home-declare"]')).toBeDisabled()
})

test('ピンの「移動」は select_pin を保留し、成功後に ui.goto{PANEL}', async ({ page }) => {
  await goto21(page, IDLE, { selectPin: { success: true, message: '' } })
  await page.locator('[data-testid="s21-pin-move-p1"]').click()
  const calls = await onsiteServiceCalls(page)
  expect(calls[0].service).toBe('/onsite/select_pin')
  expect(calls[0].request).toMatchObject({ panel_id: '盤 A-1' })
  const gotos = (await triggers(page)).filter((c) => c.trigger === 'ui.goto')
  expect(gotos).toHaveLength(1)
  expect(gotos[0].argJson).toMatchObject({ kind: 'PANEL' })
})

test('ピンの「移動」が reject されたらメッセージを出し、ui.goto を送らない', async ({ page }) => {
  await goto21(page, IDLE, { selectPin: { success: false, message: 'そのピンには行けません' } })
  await page.locator('[data-testid="s21-pin-move-p1"]').click()
  await expect(page.locator('[data-testid="s21-go-err"]')).toContainText('そのピンには行けません')
  const calls = await onsiteServiceCalls(page)
  expect(calls[0].request).toMatchObject({ panel_id: '盤 A-1' })
  expect((await triggers(page)).filter((c) => c.trigger === 'ui.goto'), 'reject なのに ui.goto を送っている').toHaveLength(0)
})

test('次の行き先: 待機場所→ui.goto{HOME}、その場で呼ぶ→ui.goto{SUMMON}、次の配電盤→ピン一覧を促す', async ({ page }) => {
  await goto21(page)

  await page.locator('[data-testid="s21-dest-home"]').click()
  let gotos = (await triggers(page)).filter((c) => c.trigger === 'ui.goto')
  expect(gotos).toHaveLength(1)
  expect(gotos[0].argJson).toMatchObject({ kind: 'HOME' })

  await page.locator('[data-testid="s21-dest-next-panel"]').click()
  await expect(page.locator('[data-testid="s21-panel-pick-hint"]')).toBeVisible()

  await page.locator('[data-testid="s21-dest-summon-here"]').click()
  gotos = (await triggers(page)).filter((c) => c.trigger === 'ui.goto')
  expect(gotos).toHaveLength(2)
  expect(gotos[1].argJson).toMatchObject({ kind: 'SUMMON' })
  // 呼び寄せサブタブに切り替わり、ウィザードはまだ出ない（SUMMON/POINT ではない）。
  await expect(page.locator('[data-testid="s21-subtab-summon"]')).toHaveAttribute('aria-selected', 'true')
  await expect(page.locator('[data-testid="s21-wizard"]')).toBeHidden()
})

test('呼び寄せ: SUMMON/POINT の 2 点指示ウィザードは purpose=SUMMON で index 1→2', async ({ page }) => {
  await goto21(page, IDLE, { twoPoint: { accepted: true, reject_reason_key: null, yaw: 0 } })
  await page.locator('[data-testid="s21-subtab-summon"]').click()
  await page.locator('[data-testid="s21-summon-start"]').click()
  await setTestState(page, { mode: 'SUMMON', state: 'POINT' })
  await expect(page.locator('[data-testid="s21-wizard"]')).toBeVisible()
  await expect(page.locator('[data-testid="s21-state"]')).toHaveText('位置指示中')

  await page.locator('[data-testid="s21-wiz-press"]').click()
  let calls = await onsiteServiceCalls(page)
  const summon = calls.filter((c) => c.service === '/onsite/two_point')
  expect(summon).toHaveLength(1)
  expect(summon[0].request).toMatchObject({ purpose: 'SUMMON', index: 1 })

  // Step 2/2 へ。受理で向き表示（yaw=90°）。
  await page.evaluate(() => { window.__thTestTwoPoint = { accepted: true, reject_reason_key: null, yaw: Math.PI / 2 } })
  await page.locator('[data-testid="s21-wiz-press"]').click()
  calls = await onsiteServiceCalls(page)
  expect(calls.filter((c) => c.service === '/onsite/two_point')).toHaveLength(2)
  expect(calls.filter((c) => c.service === '/onsite/two_point')[1].request).toMatchObject({ purpose: 'SUMMON', index: 2 })
  await expect(page.locator('[data-testid="s21-wiz-done"]')).toHaveText('向き 90°')
})

test('退避待ち: WAIT_CLEAR で進捗ボックスが出て、中止は ui.abort', async ({ page }) => {
  await goto21(page, IDLE, { waitClear: { distance_m: 0.4, remaining_sec: 7, satisfied: false, verdict: 'WAITING' } })
  await page.locator('[data-testid="s21-subtab-summon"]').click()
  await page.locator('[data-testid="s21-summon-start"]').click()
  await setTestState(page, { mode: 'SUMMON', state: 'WAIT_CLEAR' })

  await expect(page.locator('[data-testid="s21-clear-box"]')).toBeVisible()
  await expect(page.locator('[data-testid="s21-clear-dist"]')).toHaveText('0.4 m')

  // 進捗を更新（残り 3 秒 → バーが伸びる）。
  await setTestWaitClear(page, { distance_m: 0.2, remaining_sec: 3, satisfied: false, verdict: 'WAITING' })
  await expect(page.locator('[data-testid="s21-clear-dist"]')).toHaveText('0.2 m')

  // OK になったらバーが満杯（satisfied 表示）。
  await setTestWaitClear(page, { distance_m: 0.1, remaining_sec: 0, satisfied: true, verdict: 'OK' })
  await expect(page.locator('[data-testid="s21-clear-bar"].ok')).toBeAttached()

  await page.locator('[data-testid="s21-wait-cancel"]').click()
  const abort = (await triggers(page)).filter((c) => c.trigger === 'ui.abort')
  expect(abort, '中止が ui.abort を送っていない').toHaveLength(1)
})

test('配電盤前: 作業中トグルは T-ATP-01/02 の ui.working を送る', async ({ page }) => {
  await goto21(page, { mode: 'AT_PANEL', state: 'IDLE_P' })
  await page.locator('[data-testid="s21-subtab-atp"]').click()

  // IDLE_P → OFF。押すと ui.working（IDLE_P → WORKING）。
  await expect(page.locator('[data-testid="s21-work-toggle"]')).toHaveText('OFF')
  await page.locator('[data-testid="s21-work-toggle"]').click()
  let w = (await triggers(page)).filter((c) => c.trigger === 'ui.working')
  expect(w).toHaveLength(1)

  // WORKING → ON。
  await setTestState(page, { state: 'WORKING' })
  await expect(page.locator('[data-testid="s21-work-toggle"]')).toHaveText('ON')
  await page.locator('[data-testid="s21-work-toggle"]').click()
  w = (await triggers(page)).filter((c) => c.trigger === 'ui.working')
  expect(w, '作業中の切り替えが ui.working を送っていない').toHaveLength(2)
})

test('対象選択レーダー: SUMMON のときだけ ui.select_target を送る', async ({ page }) => {
  await goto21(page, { mode: 'SUMMON', state: 'POINT' })
  await page.getByRole('tab', { name: '対象選択' }).click()
  await expect(page.locator('[data-testid="radar-cand-0"]')).toBeVisible()

  await page.locator('[data-testid="radar-cand-1"]').click()
  const sel = (await triggers(page)).find((c) => c.trigger === 'ui.select_target')
  expect(sel, 'SUMMON のレーダーが ui.select_target を送っていない').toBeTruthy()
  expect(sel.argJson).toMatchObject({ index: 1 })
})

test('対象選択レーダー: PANEL_NAV では選べない（set_target を送らない）', async ({ page }) => {
  await goto21(page, { mode: 'PANEL_NAV', state: 'NAV' })
  await page.getByRole('tab', { name: '対象選択' }).click()
  await expect(page.locator('[data-testid="radar-cand-0"]')).toBeVisible()
  await page.locator('[data-testid="radar-cand-1"]').click()
  expect(
    (await triggers(page)).filter((c) => c.trigger === 'ui.select_target'),
    'PANEL_NAV では ui.select_target を出すべきではない',
  ).toHaveLength(0)
})

test('レーダーの空表示と見失い表示', async ({ page }) => {
  await goto21(page, { mode: 'SUMMON', state: 'POINT' })
  await page.getByRole('tab', { name: '対象選択' }).click()

  await setTestPersonTargets(page, { candidates: [], selected_index: -1, confidence: 0, is_lost: false, lost_reason: '' })
  await expect(page.locator('[data-testid="radar-empty"]')).toBeVisible()

  await setTestPersonTargets(page, { ...TARGETS, is_lost: true, lost_reason: 'tracker_lost' })
  await expect(page.locator('[data-testid="radar-lost"]')).toBeVisible()
})

test('SUMMON 以外のモードでも見た目はそのまま（PANEL_NAV で状態欄は stateLabel）', async ({ page }) => {
  await goto21(page, { mode: 'PANEL_NAV', state: 'NAV' })
  await expect(page.locator('[data-testid="s21-state"]')).toHaveText('移動中')
})

// brief-onsite-fix C.5: 実地図シードで s21 地図タブにラスタが描かれ、ピン/ロボットが載る。
test('地図シードありで s21-map-raster が描かれピンが地図座標に載る', async ({ page }) => {
  await goto21(page, IDLE, { routeMap: ROUTE_MAP, openVenueMap: { success: true, message: '' } })
  await unlockOnsiteVenueMap(page)
  await expect(page.locator('[data-testid="s21-map-raster"]')).toBeVisible()
  await expect(page.locator('[data-testid="s21-map-pin-p1"]')).toBeVisible()
  await expect(page.locator('[data-testid="s21-map-robot"]')).toBeVisible()
})

// brief-onsite-ux2 F-6: 押す前は地図タブに OnsiteMap がマウントされない
// （案内文と「保存した会場地図を開く」ボタンだけ）。押すと現れる。
test('F-6: 「保存した会場地図を開く」を押すまで s21-map は存在せず、押すと現れる', async ({ page }) => {
  await goto21(page, IDLE, { openVenueMap: { success: true, message: '' } })
  await expect(page.locator('[data-testid="s21-map"]')).toHaveCount(0)
  await expect(page.locator('[data-testid="s21-map-gate-msg"]')).toBeVisible()
  await unlockOnsiteVenueMap(page)
  await expect(page.locator('[data-testid="s21-map"]')).toBeVisible()
})

// brief-onsite-fix E: 「保存した会場地図を開く」を押すと /map_session/open
// （slot:VENUE / mode:reload / session_id:venue）を呼ぶ。
test('保存した会場地図を開くが /map_session/open を呼ぶ', async ({ page }) => {
  await goto21(page)
  const btn = page.locator('[data-testid="s21-open-venue-map"]')
  await expect(btn).toBeVisible()
  await btn.click()
  const open = (await onsiteServiceCalls(page)).find(
    (c) => c.service === '/map_session/open')
  expect(open, '「保存した会場地図を開く」が /map_session/open を呼んでいない').toBeTruthy()
  expect(open.request).toMatchObject({ slot: 'VENUE', mode: 'reload', session_id: 'venue' })
})

