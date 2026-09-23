// WP-UI-06: S-20 試験準備の操作が本当に th_state へ届くこと（送ったものを数えられる
// __thTriggerCalls ／ __thOnsiteServiceCalls で検証）。過去に「ボタンが在るのに
// onClick 空」を出したため、全ての操作ボタンで trigger/service の記録を確認する。
// th_state は無いので PREP/MAPPING を addInitScript で直接注入し、REGISTER への
// 遷移は __thSetTestState で再現する（Server 側 FSM を起動しない）。
import { test, expect } from '@playwright/test'
import {
  gotoScreen, gotoScreenWithOnsite, onsiteServiceCalls,
  setTestState, setTestPersonTargets, setTestOnsitePins, unlockOnsiteMap, stdTriggerCalls,
  stubServices,
} from './helpers.js'
import { onsiteMapTransform } from '../src/mapGeometry.js'

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

const PREP = { mode: 'PREP', state: 'MAPPING', tracker_enabled: true }

// 最小 OccupancyGrid: 4×4, res 0.5, origin (-1,-1) → world x/y ∈ [-1,1]。
// 0 自由 / 100 占有 / -1 未知（RoutePreview e2e と同じ形）。
const ROUTE_MAP = {
  info: {
    resolution: 0.5,
    width: 4,
    height: 4,
    // quatToYaw() は x/y も参照する（無回転の単位クォータニオン）。x/y を省くと
    // NaN になり、pin/toPx が NaN 座標になる（従来のテストは raster の可視性しか
    // 見ていなかったため気づかれなかった。MAPTAP の非等倍回帰テストで判明）。
    origin: { position: { x: -1, y: -1 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
  },
  data: [0, 100, -1, 0, 100, 0, 0, -1, -1, 0, 100, 0, 0, -1, 100, 0],
}

async function triggers(page) {
  return page.evaluate(() => window.__thTriggerCalls ?? [])
}

test('S-20 表示とタイトル', async ({ page }) => {
  await gotoScreenWithOnsite(
    page, 'S20', PREP,
    { pins: PINS, targets: TARGETS, pose: { x: 0, y: 0, yaw: 0 }, mappingActive: true },
  )
  await page.locator('#s20').waitFor()
  await expect(page.locator('#s20 [data-testid="s20-finish"]')).toBeVisible()
  // brief-onsite-ux2 F-6: 地図タブは「地図作成開始」を押すまで OnsiteMap を
  // マウントしない。押した後にピン 3 個とロボットマーカーが出る。
  await unlockOnsiteMap(page)
  await expect(page.locator('[data-testid="s20-map-pin-p1"]')).toBeVisible()
  await expect(page.locator('[data-testid="s20-map-pin-p3"]')).toBeVisible()
  await expect(page.locator('[data-testid="s20-map-robot"]')).toBeVisible()
})

// MAP-1: 追従対象者（/person/status、base_link 相対）を地図上に表示する。
// is_lost:false のときだけ baseToWorld() で map 座標に変換して出す。
test('MAP-1: 追従対象者は is_lost:false のときだけ地図に出る', async ({ page }) => {
  await gotoScreenWithOnsite(
    page, 'S20', PREP,
    {
      pins: PINS, targets: TARGETS, pose: { x: 0, y: 0, yaw: 0 }, mappingActive: true,
      personStatus: { position: { x: 1, y: 0, z: 0 }, confidence: 0.9, is_lost: false, lost_reason: '' },
    },
  )
  await page.locator('#s20').waitFor()
  await unlockOnsiteMap(page)
  await expect(page.locator('[data-testid="s20-map-person"]')).toBeVisible()

  // is_lost:true に切り替えるとマーカーは消える。
  await page.evaluate(() => {
    window.__thSetTestPersonStatus({
      position: { x: 1, y: 0, z: 0 }, confidence: 0, is_lost: true, lost_reason: 'DETECTION_LOST',
    })
  })
  await expect(page.locator('[data-testid="s20-map-person"]')).toBeHidden()
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

// WS-9AB: 壁近接警告。②押下で pin_registrar が /onsite/pin_warning を立て、
// two_point の応答は accepted:true + reject_reason_key:'pin_close_to_wall'。
// ウィザードは「done」にならず、3 択ダイアログが出て /onsite/resolve_pin を呼ぶ。
test('WS-9AB: 壁に近すぎる警告で 3 択が出て /onsite/resolve_pin を呼ぶ', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, {
    pins: PINS, targets: TARGETS,
    twoPoint: { accepted: true, reject_reason_key: 'pin_close_to_wall', yaw: 0 },
    resolvePin: { success: true, message: 'ok' },
  })
  await page.locator('#s20').waitFor()
  // pinWarning は未着火（active:false）の状態で登録を開始できることを確認する
  // （WS-9AC: 未解決の警告があるときだけ入口ボタンを無効化するので、開始時点では
  // 押せて良い）。
  await page.locator('[data-testid="s20-reg-panel"]').click()
  await setTestState(page, { state: 'REGISTER' })
  // ② 押下（Step2）相当で /onsite/pin_warning が立つのを模す。
  await page.evaluate(() => {
    window.__thSetTestPinWarning({
      active: true, kind: 'PANEL', nearest_m: 0.22, min_m: 0.35,
      retreat_x: 3.0, retreat_y: -3.3, retreat_yaw: -1.8,
    })
  })
  // 警告ダイアログが出てウィザードは隠れる
  await expect(page.locator('[data-testid="s20-pinwarn"]')).toBeVisible()
  await expect(page.locator('[data-testid="s20-wizard"]')).not.toBeVisible()
  await expect(page.locator('[data-testid="s20-wiz-done"]')).not.toBeVisible()

  await page.locator('[data-testid="s20-pinwarn-retreat"]').click()
  const calls = await onsiteServiceCalls(page)
  const r = calls.find((c) => c.service === '/onsite/resolve_pin')
  expect(r, 'resolve_pin が呼ばれていない').toBeTruthy()
  expect(r.request).toMatchObject({ action: 'retreat' })
})

// brief-onsite-register-fix REG-1: two_point_too_close の実機連発の核心。
// 拒否理由が i18n に無いと生キーが丸見えになり「なぜ失敗するか」が伝わらない。
// 日本語（生キーでない）が出ることと、理由があれば Step が進まないことを固定する。
test('REG-1: two_point_too_close 拒否で日本語の理由が出る（生キーでない）', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, {
    pins: PINS, targets: TARGETS,
    twoPoint: { accepted: false, reject_reason_key: 'two_point_too_close' },
  })
  await page.locator('#s20').waitFor()
  await page.locator('[data-testid="s20-reg-home"]').click()
  await setTestState(page, { state: 'REGISTER' })
  await page.locator('[data-testid="s20-wiz-press"]').click()
  const err = page.locator('[data-testid="s20-wiz-err"]')
  await expect(err).toBeVisible()
  await expect(err).not.toHaveText('two_point_too_close')
  await expect(err).toContainText('一歩進んで')
  // Step 2 へ進まず、押し直しを促すボタンが残る。
  await expect(page.locator('[data-testid="s20-wiz-press"]')).toBeVisible()
})

// brief-onsite-register-fix REG-1: 同じ文言だと「2 回目も同じ場所で押せばよい」と
// 誤解され、一歩進むことを知らされないまま拒否され続ける。Step1/Step2 で
// 表示文言が変わることを固定する。
test('REG-1: 2 点指示の Step1 と Step2 で表示文言が変わる', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, {
    pins: PINS, targets: TARGETS,
    twoPoint: { accepted: true, reject_reason_key: null, yaw: 0 },
  })
  await page.locator('#s20').waitFor()
  await page.locator('[data-testid="s20-reg-home"]').click()
  await setTestState(page, { state: 'REGISTER' })
  await expect(page.locator('[data-testid="s20-wizard"]')).toBeVisible()
  const step1 = await page.locator('[data-testid="s20-wizard"] .mut').textContent()
  await expect(page.locator('[data-testid="s20-wiz-press"]')).toBeVisible()
  await page.locator('[data-testid="s20-wiz-press"]').click()
  const step2 = await page.locator('[data-testid="s20-wizard"] .mut').textContent()
  expect(step1, 'Step1 の文言を確認できなかった').toBeTruthy()
  expect(step2, 'Step2 の文言を確認できなかった').toBeTruthy()
  expect(step1, 'Step1/Step2 の文言が同じ（誤解の元）').not.toBe(step2)
  await expect(page.locator('[data-testid="s20-wiz-press"]')).toBeVisible()
})

// brief-onsite-register-fix REG-2: 機体姿勢での登録。「いまの姿勢で待機場所を
// 登録」を押すと /onsite/register_pin に {kind:'HOME', method:'ROBOT_POSE'} が
// 飛び、成功スタブで成功メッセージ、失敗スタブで生キーでない日本語が出る。
test('REG-2: 機体姿勢で登録（HOME）は registerPin{method:ROBOT_POSE} を呼ぶ', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, { pins: PINS, targets: TARGETS })
  await page.locator('#s20').waitFor()

  // 成功スタブ（call 時に読むので click 前に設定）。
  await page.evaluate(() => {
    window.__thTestRegisterPinHere = { success: true, pin: {}, message: '機体の現在姿勢で登録しました' }
  })
  await page.locator('[data-testid="s20-reg-home-here"]').click()
  const calls = await onsiteServiceCalls(page)
  const reg = calls.find((c) => c.service === '/onsite/register_pin')
  expect(reg, '「いまの姿勢で待機場所を登録」が /onsite/register_pin を呼んでいない').toBeTruthy()
  expect(reg.request).toMatchObject({ kind: 'HOME', method: 'ROBOT_POSE' })
  await expect(page.locator('[data-testid="s20-reg-here-msg"]')).toHaveText('機体の現在姿勢で登録しました')

  // 失敗スタブ: message が空でも日本語（nil ではなく OK 相当の空を避ける）。
  await page.evaluate(() => {
    window.__thTestRegisterPinHere = { success: true, pin: {}, message: '' }
  })
  await page.locator('[data-testid="s20-reg-home-here"]').click()
  await expect(page.locator('[data-testid="s20-reg-here-msg"]')).toContainText('待機場所')
})

test('REG-2: 機体姿勢で登録（PANEL）と失敗時の日本語表示', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, { pins: PINS, targets: TARGETS })
  await page.locator('#s20').waitFor()

  await page.evaluate(() => {
    window.__thTestRegisterPinHere = { success: true, pin: {}, message: '' }
  })
  await page.locator('[data-testid="s20-reg-panel-here"]').click()
  const calls = await onsiteServiceCalls(page)
  const reg = calls.find((c) => c.service === '/onsite/register_pin')
  expect(reg).toBeTruthy()
  expect(reg.request).toMatchObject({ kind: 'PANEL', method: 'ROBOT_POSE' })
  await expect(page.locator('[data-testid="s20-reg-here-msg"]')).toContainText('配電盤')

  // 失敗（TF 無し等）: message をそのまま出す。生キーでないことを確認。
  await page.evaluate(() => {
    window.__thTestRegisterPinHere = { success: false, pin: {}, message: '自己位置（地図座標）が取得できません' }
  })
  await page.locator('[data-testid="s20-reg-panel-here"]').click()
  const msg = page.locator('[data-testid="s20-reg-here-msg"]')
  await expect(msg).toBeVisible()
  await expect(msg).toHaveClass(/(^|\s)err(\s|$)/)
  await expect(msg).toHaveText('自己位置（地図座標）が取得できません')
})

// WS-9AC(2026-09-11): ROBOT_POSE は FSM を経由しないので state は MAPPING の
// まま（REGISTER にならない）。以前は 3 択ダイアログが isRegister（state===
// REGISTER）でも囲っていたため、この経路では絶対に表示できなかった。
test('WS-9AC: いまの姿勢で登録が壁に近いと state=MAPPING のままでも 3 択が出る', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, { pins: PINS, targets: TARGETS })
  await page.locator('#s20').waitFor()

  await page.evaluate(() => {
    window.__thTestRegisterPinHere = {
      success: false,
      message: '壁から 0.28m しかありません。このまま登録 / 離して登録 / やめる を選んでください',
    }
  })
  await page.locator('[data-testid="s20-reg-panel-here"]').click()
  // MAPPING のまま。/onsite/pin_warning の topic 更新を模す。
  await page.evaluate(() => {
    window.__thSetTestPinWarning({
      active: true, kind: 'PANEL', nearest_m: 0.28, min_m: 0.45,
      retreat_x: 3.0, retreat_y: -3.3, retreat_yaw: -1.8,
    })
  })
  await expect(page.locator('[data-testid="s20-pinwarn"]')).toBeVisible()
  // 未解決の警告が出ている間は登録系ボタンを無効化する。
  await expect(page.locator('[data-testid="s20-reg-panel-here"]')).toBeDisabled()
  await expect(page.locator('[data-testid="s20-reg-home"]')).toBeDisabled()
})

// 2026-09-09 レビューで確認: ROBOT_POSE は _place_pin_effect() を直接呼ぶため、
// 2 点指示ウィザードの受付状態（_accepting/_p1/_yaw）を横から上書きして壊してしまう。
// ウィザードが開いている（isRegister）間はボタンを無効化して、この衝突を防ぐ。
test('REG-2: 2点指示ウィザードが開いている間は「いまの姿勢で登録」を無効化', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, {
    pins: PINS, targets: TARGETS,
    twoPoint: { accepted: true, reject_reason_key: null, yaw: 0 },
  })
  await page.locator('#s20').waitFor()
  await expect(page.locator('[data-testid="s20-reg-home-here"]')).toBeEnabled()

  await page.locator('[data-testid="s20-reg-home"]').click()
  await setTestState(page, { state: 'REGISTER' })
  await expect(page.locator('[data-testid="s20-wizard"]')).toBeVisible()

  await expect(page.locator('[data-testid="s20-reg-home-here"]')).toBeDisabled()
  await expect(page.locator('[data-testid="s20-reg-panel-here"]')).toBeDisabled()
})

test('停止/保存の操作カードと対象選択（radar）', async ({ page }) => {
  await gotoScreenWithOnsite(
    page, 'S20', PREP,
    { pins: PINS, targets: TARGETS, pose: { x: 0, y: 0, yaw: 0 }, mappingActive: true },
  )
  await page.locator('#s20').waitFor()
  await unlockOnsiteMap(page)

  // 「停止」「走行」は PREP では inert（2026-09-10 WS-9AA。将来の追従走行に予約）。
  // ボタンは両方出て ui.stop / ui.run を送るが、FSM 側は状態を変えない自己ループ。
  await expect(page.locator('#s20 .op-run')).toBeVisible()
  await page.locator('#s20 .op-run').click()
  expect(
    (await triggers(page)).some((c) => c.trigger === 'ui.run'),
    '走行ボタンが ui.run を送っていない',
  ).toBe(true)

  // 操作カード: 停止(ui.stop)。保存は、この PINS/TARGETS シードだと全段完了で
  // 「次にやること」ボタンも保存になるため、重複を避けて操作カード側には出ない
  // （brief-onsite-ux-fix UX-6-c）。保存自体はそちらのボタンで送る。
  await page.locator('#s20 .op-stop').click()
  await expect(page.locator('#s20 .op-save')).toBeHidden()
  await page.locator('[data-testid="s20-next-action"]').click()
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

  // 2026-09-08 実機修正: 候補が居る間は isLost でも候補を隠さずタップ可能にする
  // （人物追跡ライブラリは見失うと自動で再取得しないため、選び直す手段を必ず残す）。
  // 候補 0 件 + isLost のときだけ「見失っています」の全面表示。
  await setTestPersonTargets(page, { ...TARGETS, is_lost: true, lost_reason: 'tracker_lost' })
  await expect(page.locator('[data-testid="radar-lost-reselect"]')).toBeVisible()
  await expect(page.locator('[data-testid="radar-cand-0"]')).toBeVisible()
  await expect(page.locator('[data-testid="radar-lost"]')).toHaveCount(0)

  await setTestPersonTargets(page, { candidates: [], selected_index: -1, confidence: 0, is_lost: true, lost_reason: 'tracker_lost' })
  await expect(page.locator('[data-testid="radar-lost"]')).toBeVisible()
})

// 2026-09-08 実機で確認: 対象を見失った後に再選択できないと復帰不能になる
// （multiple_sensor_person_tracking は明示選択でしか再取得しない設計）。
test('見失った状態でも候補をタップして選び直せる', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, { pins: PINS, targets: { ...TARGETS, is_lost: true } })
  await page.locator('#s20').waitFor()
  await page.getByRole('tab', { name: '対象選択' }).click()

  await expect(page.locator('[data-testid="radar-lost-reselect"]')).toBeVisible()
  await expect(page.locator('[data-testid="radar-cand-1"]')).toBeVisible()

  await page.locator('[data-testid="radar-cand-1"]').click()
  const sel = (await triggers(page)).find((c) => c.trigger === 'ui.select_target')
  expect(sel, '見失い中のタップが ui.select_target を送っていない').toBeTruthy()
  expect(sel.argJson).toMatchObject({ index: 1 })
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
    {
      pins: PINS, targets: TARGETS, pose: { x: 0, y: 0, yaw: 0 }, routeMap: ROUTE_MAP,
      mappingActive: true,
    },
  )
  await page.locator('#s20').waitFor()
  await unlockOnsiteMap(page)
  // ラスタ描画は canvas→dataURL なので /route/map_view 受信後に非同期で出る。
  await expect(page.locator('[data-testid="s20-map-raster"]')).toBeVisible()
  await expect(page.locator('[data-testid="s20-map-pin-p1"]')).toBeVisible()
  await expect(page.locator('[data-testid="s20-map-robot"]')).toBeVisible()
})

test('地図シードなしならラスタは描かれない（モックアップ部屋も描かない。F-4）', async ({ page }) => {
  await gotoScreenWithOnsite(
    page, 'S20', PREP,
    { pins: PINS, targets: TARGETS, pose: { x: 0, y: 0, yaw: 0 }, mappingActive: true },
  )
  await page.locator('#s20').waitFor()
  await unlockOnsiteMap(page)
  await expect(page.locator('[data-testid="s20-map-raster"]')).not.toBeVisible()
  await expect(page.locator('[data-testid="s20-map-robot"]')).toBeVisible()
})

// brief-onsite-ux2 F-6: 押す前は地図タブに OnsiteMap がマウントされない
// （地図作成開始ボタンと案内文だけ）。押すと現れる。
test('F-6: 「地図作成開始」を押すまで s20-map は存在せず、押すと現れる', async ({ page }) => {
  await gotoScreenWithOnsite(
    page, 'S20', PREP,
    { pins: PINS, targets: TARGETS, pose: { x: 0, y: 0, yaw: 0 }, mappingActive: true },
  )
  await page.locator('#s20').waitFor()
  await expect(page.locator('[data-testid="s20-map"]')).toHaveCount(0)
  await expect(page.locator('[data-testid="s20-map-gate-msg"]')).toBeVisible()
  const startBtn = page.locator('[data-testid="s20-map-gate-start"]')
  await expect(startBtn).toBeEnabled()
  await startBtn.click()
  await expect(page.locator('[data-testid="s20-map"]')).toBeVisible()
})

// brief-onsite-ux2 F-6: mappingActive が不明（null）の間はボタンを非活性にし、
// race で稼働中の地図作成を誤って止めないようにする。
test('F-6: mappingActive 不明の間は「地図作成開始」が非活性', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, { pins: PINS, targets: TARGETS })
  await page.locator('#s20').waitFor()
  await expect(page.locator('[data-testid="s20-map-gate-start"]')).toBeDisabled()
})

// brief-onsite-ux2 F-6 の核心: mappingActive===true（起動時から動いている通常
// ケース）で「地図作成開始」を押しても /slam_control/toggle_mapping を呼ばない
// （呼ぶと動いている地図作成を誤って止めてしまう）。表示だけ解禁する。
test('F-6: mappingActive===true なら toggle_mapping を呼ばずに表示だけ解禁', async ({ page }) => {
  await gotoScreenWithOnsite(
    page, 'S20', PREP,
    { pins: PINS, targets: TARGETS, mappingActive: true },
  )
  await page.locator('#s20').waitFor()
  await page.locator('[data-testid="s20-map-gate-start"]').click()
  await expect(page.locator('[data-testid="s20-map"]')).toBeVisible()
  const calls = await stdTriggerCalls(page)
  expect(calls.some((c) => c.service === '/slam_control/toggle_mapping'),
    'mappingActive===true なのに toggle_mapping を呼んだ（動いている地図作成を止めてしまう）').toBe(false)
})

// mappingActive===false なら toggle_mapping を呼んでから表示を解禁する。
test('F-6: mappingActive===false なら toggle_mapping を呼んでから表示を解禁', async ({ page }) => {
  await gotoScreenWithOnsite(
    page, 'S20', PREP,
    { pins: PINS, targets: TARGETS, mappingActive: false },
  )
  await page.locator('#s20').waitFor()
  await page.locator('[data-testid="s20-map-gate-start"]').click()
  await expect(page.locator('[data-testid="s20-map"]')).toBeVisible()
  const calls = await stdTriggerCalls(page)
  expect(calls.some((c) => c.service === '/slam_control/toggle_mapping')).toBe(true)
})

// WS-9Y: 前回セッションのピンが残っているときだけ「新しい試験日として開始」を出す。
test('WS-9Y: ピンが残っていれば地図ゲートにリセット案内が出る', async ({ page }) => {
  await gotoScreenWithOnsite(
    page, 'S20', PREP,
    { pins: PINS, targets: TARGETS, mappingActive: true },
  )
  await page.locator('#s20').waitFor()
  await expect(page.locator('[data-testid="s20-reset-venue"]')).toBeVisible()
})

test('WS-9Y: ピンが 0 件ならリセット案内は出ない', async ({ page }) => {
  await gotoScreenWithOnsite(
    page, 'S20', PREP,
    { pins: [], targets: TARGETS, mappingActive: true },
  )
  await page.locator('#s20').waitFor()
  await expect(page.locator('[data-testid="s20-reset-venue"]')).toHaveCount(0)
})

// WS-9Y の核心: ArmedButton は 2 回押さないと発火しない（誤操作防止）。1 回目では
// discard_map / reset_pins のどちらも呼ばれず、2 回目で discard_map → reset_pins
// の順に呼ばれる。
test('WS-9Y: リセットは2回押しで discard_map → reset_pins の順に呼ぶ', async ({ page }) => {
  await stubServices(page, {
    '/slam_control/discard_map': { success: true, message: '' },
    '/onsite/reset_pins': { success: true, message: '' },
  })
  await gotoScreenWithOnsite(
    page, 'S20', PREP,
    { pins: PINS, targets: TARGETS, mappingActive: true },
  )
  await page.locator('#s20').waitFor()
  const btn = page.locator('[data-testid="s20-reset-venue"] button')
  await btn.click()
  expect(await stdTriggerCalls(page)).toEqual([])
  await btn.click()
  const calls = await stdTriggerCalls(page)
  expect(calls.map((c) => c.service)).toEqual([
    '/slam_control/discard_map', '/onsite/reset_pins',
  ])
})

// 失敗時はエラー文言を出し、reset_pins は呼ばない（discard_map の失敗で止まる）。
test('WS-9Y: discard_map が失敗したら reset_pins を呼ばずエラーを出す', async ({ page }) => {
  await stubServices(page, {
    '/slam_control/discard_map': { success: false, message: 'slam_toolbox に接続できません' },
  })
  await gotoScreenWithOnsite(
    page, 'S20', PREP,
    { pins: PINS, targets: TARGETS, mappingActive: true },
  )
  await page.locator('#s20').waitFor()
  const btn = page.locator('[data-testid="s20-reset-venue"] button')
  await btn.click()
  await btn.click()
  await expect(page.locator('[data-testid="s20-reset-err"]')).toBeVisible()
  const calls = await stdTriggerCalls(page)
  expect(calls.map((c) => c.service)).toEqual(['/slam_control/discard_map'])
})

// brief-onsite-fix F-2: レーダーに機体マーク（前方上向き三角＋中心丸）が出る。
test('レーダーに機体マーク（radar-heading）が描かれる', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, { pins: PINS, targets: TARGETS, pose: { x: 0, y: 0, yaw: 0 } })
  await page.locator('#s20').waitFor()
  await page.getByRole('tab', { name: '対象選択' }).click()
  await expect(page.locator('[data-testid="radar-heading"]')).toBeVisible()
})

// brief-MAPTAP-FRONTEND §2.5: 地図タップ登録（方式C）。「地図タップで配電盤を
// 登録」→ 地図タブへ切り替わり、ドラッグで確定カードが出て、確定で
// /onsite/register_pin{method:MAP_TAP} が飛ぶ。
test('MAPTAP: 地図タップ登録はドラッグ→確定で register_pin{method:MAP_TAP} を呼ぶ', async ({ page }) => {
  await gotoScreenWithOnsite(
    page, 'S20', PREP,
    {
      pins: PINS, targets: TARGETS, pose: { x: 0, y: 0, yaw: 0 }, routeMap: ROUTE_MAP,
      mappingActive: true,
    },
  )
  await page.locator('#s20').waitFor()
  await unlockOnsiteMap(page)
  // ズーム/パン配線のツールバー（全体表示＋縮尺）が出ていること。
  await expect(page.locator('[data-testid="s20-map-reset-view"]')).toBeVisible()
  await expect(page.locator('[data-testid="s20-map-scale"]')).toBeVisible()

  // 地図タブから離れてから入口ボタンを押す（タブが切り替わることを見るため）。
  await page.getByRole('tab', { name: '対象選択' }).click()
  await page.evaluate(() => {
    window.__thTestRegisterPinMapTap = { success: true, pin: {}, message: '地図タップで登録しました' }
  })
  await page.locator('[data-testid="s20-reg-panel-maptap"]').click()
  await expect(page.locator('[data-testid="s20-tab-map"]')).toHaveAttribute('aria-selected', 'true')

  // SVG 内ドラッグ（press→move→release）で確定カードが出る。
  const map = page.locator('[data-testid="s20-map"]')
  const box = await map.boundingBox()
  await page.mouse.move(box.x + box.width * 0.3, box.y + box.height * 0.4)
  await page.mouse.down()
  await page.mouse.move(box.x + box.width * 0.6, box.y + box.height * 0.4, { steps: 5 })
  await page.mouse.up()
  await expect(page.locator('[data-testid="s20-maptap-confirm-card"]')).toBeVisible()
  await expect(page.locator('[data-testid="s20-maptap-preview"]')).toContainText('向き')
  await expect(page.locator('[data-testid="s20-map-preview"]')).toBeVisible()

  await page.locator('[data-testid="s20-maptap-confirm"]').click()
  const calls = await onsiteServiceCalls(page)
  const reg = calls[calls.length - 1]
  expect(reg.service).toBe('/onsite/register_pin')
  expect(reg.request.method).toBe('MAP_TAP')
  expect(reg.request.kind).toBe('PANEL')
  for (const k of ['tap1_x', 'tap1_y', 'tap2_x', 'tap2_y']) {
    expect(typeof reg.request[k], `${k} が数値で送られていない`).toBe('number')
  }
  await expect(page.locator('[data-testid="s20-reg-here-msg"]')).toHaveText('地図タップで登録しました')
  // 確定後はカードが消える。
  await expect(page.locator('[data-testid="s20-maptap-confirm-card"]')).toHaveCount(0)
})

// キャンセル経路: カードが消え、新規のサービス呼び出しが増えない。
test('MAPTAP: キャンセルでカードが消えサービスを呼ばない', async ({ page }) => {
  await gotoScreenWithOnsite(
    page, 'S20', PREP,
    {
      pins: PINS, targets: TARGETS, pose: { x: 0, y: 0, yaw: 0 }, routeMap: ROUTE_MAP,
      mappingActive: true,
    },
  )
  await page.locator('#s20').waitFor()
  await unlockOnsiteMap(page)
  await page.evaluate(() => {
    window.__thTestRegisterPinMapTap = { success: true, pin: {}, message: '地図タップで登録しました' }
  })
  await page.locator('[data-testid="s20-reg-home-maptap"]').click()

  const map = page.locator('[data-testid="s20-map"]')
  const box = await map.boundingBox()
  await page.mouse.move(box.x + box.width * 0.3, box.y + box.height * 0.6)
  await page.mouse.down()
  await page.mouse.move(box.x + box.width * 0.5, box.y + box.height * 0.3, { steps: 5 })
  await page.mouse.up()
  await expect(page.locator('[data-testid="s20-maptap-confirm-card"]')).toBeVisible()

  const before = (await onsiteServiceCalls(page)).length
  await page.locator('[data-testid="s20-maptap-cancel"]').click()
  await expect(page.locator('[data-testid="s20-maptap-confirm-card"]')).toHaveCount(0)
  expect(await onsiteServiceCalls(page)).toHaveLength(before)
})

// 実機確認（2026-09-15）: 「地図タップの位置が実際にタップした場所とずれる」の回帰
// テスト。shell/FixedStage.jsx は #app 全体を transform: scale(var(--stage-scale))
// で画面に合わせて拡大する（論理サイズは stageMetrics.js の 1280x720）。既定の
// Playwright ビューポートはちょうど 1280x720 なので --stage-scale=1 になり、
// この問題は再現しない（元のバグが e2e をすり抜けた理由）。ここでは意図的に
// 1024x768（--stage-scale = min(1024/1280, 768/720) = 0.8）にして、非等倍でも
// タップした画面座標が正しい map 座標に変換されることを確認する。
test('MAPTAP: 非等倍の --stage-scale でもタップ位置が正しい map 座標に変換される（実機のタップずれの回帰）', async ({ page }) => {
  await page.setViewportSize({ width: 1024, height: 768 })
  await gotoScreenWithOnsite(
    page, 'S20', PREP,
    {
      pins: PINS, targets: TARGETS, pose: { x: 0, y: 0, yaw: 0 }, routeMap: ROUTE_MAP,
      mappingActive: true,
    },
  )
  await page.locator('#s20').waitFor()
  await unlockOnsiteMap(page)
  await page.evaluate(() => {
    window.__thTestRegisterPinMapTap = { success: true, pin: {}, message: 'ok' }
  })
  await page.locator('[data-testid="s20-reg-panel-maptap"]').click()

  // OnsiteMap.jsx が実際に使う変換（zoom=1・pan=0 の初期状態）で、選んだ world 座標
  // が SVG の viewBox 上のどこに来るかを求める（本番コードと同じ mapGeometry.js）。
  const worldTap1 = { x: 0.2, y: -0.4 }
  const worldTap2 = { x: 0.7, y: -0.4 }
  const t = onsiteMapTransform(ROUTE_MAP, 340, 250, 24, 22)
  const [vx1, vy1] = t.toPx(worldTap1.x, worldTap1.y)
  const [vx2, vy2] = t.toPx(worldTap2.x, worldTap2.y)

  // viewBox 座標 → 実画面座標。SVG 自身のレターボックス（既定の xMidYMid meet）を
  // 実際の boundingBox（--stage-scale 込みの最終描画矩形）から計算する
  // （OnsiteMap.jsx の clientToViewBox の逆。実際に描画された矩形を使うことで
  // --stage-scale の実際の適用まで検証する）。
  const box = await page.locator('[data-testid="s20-map"]').boundingBox()
  const scale = Math.min(box.width / 340, box.height / 250)
  const drawW = 340 * scale, drawH = 250 * scale
  const offX = box.x + (box.width - drawW) / 2
  const offY = box.y + (box.height - drawH) / 2
  const [sx1, sy1] = [offX + vx1 * scale, offY + vy1 * scale]
  const [sx2, sy2] = [offX + vx2 * scale, offY + vy2 * scale]

  await page.mouse.move(sx1, sy1)
  await page.mouse.down()
  await page.mouse.move(sx2, sy2, { steps: 5 })
  await page.mouse.up()

  await expect(page.locator('[data-testid="s20-maptap-confirm-card"]')).toBeVisible()
  await page.locator('[data-testid="s20-maptap-confirm"]').click()
  const calls = await onsiteServiceCalls(page)
  const reg = calls[calls.length - 1]
  expect(reg.request.method).toBe('MAP_TAP')
  expect(reg.request.tap1_x).toBeCloseTo(worldTap1.x, 1)
  expect(reg.request.tap1_y).toBeCloseTo(worldTap1.y, 1)
  expect(reg.request.tap2_x).toBeCloseTo(worldTap2.x, 1)
  expect(reg.request.tap2_y).toBeCloseTo(worldTap2.y, 1)
})

// 実機確認（2026-09-15）: 「タブレット操作で地図の拡大ができない」の回帰テスト。
// 従来は onWheel（マウスホイール）しか無く、タッチのピンチ操作を一切見ていなかった。
// Playwright にはピンチ操作の高水準 API が無いので、2本指タッチを合成 PointerEvent
// （pointerType: 'touch'）で dispatch して縮尺表示の変化を見る。
test('MAPTAP: 2本指ピンチでズームできる（タブレットでの拡大操作の回帰）', async ({ page }) => {
  await gotoScreenWithOnsite(
    page, 'S20', PREP,
    {
      pins: PINS, targets: TARGETS, pose: { x: 0, y: 0, yaw: 0 }, routeMap: ROUTE_MAP,
      mappingActive: true,
    },
  )
  await page.locator('#s20').waitFor()
  await unlockOnsiteMap(page)

  const scaleBefore = await page.locator('[data-testid="s20-map-scale"]').innerText()

  await page.evaluate(() => {
    const svg = document.querySelector('[data-testid="s20-map"]')
    const rect = svg.getBoundingClientRect()
    const cy = rect.top + rect.height / 2
    const mk = (id, x, y, type) => new PointerEvent(type, {
      pointerId: id, pointerType: 'touch', clientX: x, clientY: y,
      bubbles: true, cancelable: true,
    })
    // 2本指を中心から近い間隔で置き、外側へ広げる（ピンチアウト＝拡大）。
    const cx = rect.left + rect.width / 2
    svg.dispatchEvent(mk(1, cx - 10, cy, 'pointerdown'))
    svg.dispatchEvent(mk(2, cx + 10, cy, 'pointerdown'))
    svg.dispatchEvent(mk(1, cx - 60, cy, 'pointermove'))
    svg.dispatchEvent(mk(2, cx + 60, cy, 'pointermove'))
    svg.dispatchEvent(mk(1, cx - 60, cy, 'pointerup'))
    svg.dispatchEvent(mk(2, cx + 60, cy, 'pointerup'))
  })

  const scaleAfter = await page.locator('[data-testid="s20-map-scale"]').innerText()
  // ズームすると同じ 60px のスケールバーが表す実寸は小さくなる（拡大＝1pxあたりの
  // 実寸が減る）。数値化して比較する（末尾 "m"/"cm" を含む文字列）。
  const parseM = (s) => (s.includes('cm') ? parseFloat(s) / 100 : parseFloat(s))
  expect(parseM(scaleAfter)).toBeLessThan(parseM(scaleBefore))
})