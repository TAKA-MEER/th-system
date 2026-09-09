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
// orientation は ROS と同じ 4 成分（x/y/z/w）を必ず書く — x/y を省くと
// mapGeometry.quatToYaw() が undefined*undefined = NaN になり toPx() が NaN を返す。
const ROUTE_MAP = {
  info: {
    resolution: 0.5,
    width: 4,
    height: 4,
    origin: { position: { x: -1, y: -1 }, orientation: { x: 0, y: 0, w: 1, z: 0 } },
  },
  data: [0, 100, -1, 0, 100, 0, 0, -1, -1, 0, 100, 0, 0, -1, 100, 0],
}

// brief-MAP-COSTMAP: Nav2 グローバル costmap（/global_costmap/costmap）の最小形。
// 静的地図（ROUTE_MAP）とは別の origin/resolution/サイズでも構わない（OnsiteMap は
// costmap 自身の info から実世界の四隅を toPx() で変換して配置する）。
// orientation は ROUTE_MAP と同じく 4 成分を書く（上記コメント参照）。
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

// WS-9Y: 「会場地図を開く」の前提（機体を待機場所に置いてから押す）を画面に出す。
// これが要る理由: slam_control は HOME ピンの姿勢を初期姿勢にフォールバックする
// ため、機体が実際に待機場所にいないと自己位置が合わない。
test('WS-9Y: 会場地図を開く前に「機体を待機場所に置いてから」の案内が出る', async ({ page }) => {
  await goto21(page, IDLE, { openVenueMap: { success: true, message: '' } })
  await expect(page.locator('[data-testid="s21-open-venue-hint"]')).toBeVisible()
  // 開いたあとは地図タブのゲート自体が消える（案内も含めて）。
  await unlockOnsiteVenueMap(page)
  await expect(page.locator('[data-testid="s21-open-venue-hint"]')).toHaveCount(0)
})

// MAP-1: 追従対象者（/person/status、base_link 相対）を地図上に表示する。
// is_lost:false のときだけ baseToWorld() で map 座標に変換して出す。
test('MAP-1: 追従対象者は is_lost:false のときだけ地図に出る', async ({ page }) => {
  await goto21(page, IDLE, {
    openVenueMap: { success: true, message: '' },
    personStatus: { position: { x: 1, y: 0, z: 0 }, confidence: 0.9, is_lost: false, lost_reason: '' },
  })
  await unlockOnsiteVenueMap(page)
  await expect(page.locator('[data-testid="s21-map-person"]')).toBeVisible()

  // is_lost:true に切り替えるとマーカーは消える。
  await page.evaluate(() => {
    window.__thSetTestPersonStatus({
      position: { x: 1, y: 0, z: 0 }, confidence: 0, is_lost: true, lost_reason: 'DETECTION_LOST',
    })
  })
  await expect(page.locator('[data-testid="s21-map-person"]')).toBeHidden()
})

// MAP-SCAN: /scan_filtered（sensor_msgs/LaserScan）を地図タブに点群として重ねる。
// routePose（pose オプション）を seed したときだけ baseToWorld() できるので、
// 点群が出るのは pose を渡した場合だけ。
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
  // pose を外すと routePose も未受信になり、baseToWorld() できないので null。
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

// 2026-09-09 実機で確認: PANEL_NAV / SUMMON / HOME_NAV はジョグ後に PAUSE へ
// 落ち、脱出路は ui.run（T-PNAV-*/T-SUM-*/T-HNAV-*）だけなのに操作カードの
// run が false で潰されていて詰んでいた。AT_PANEL / AT_HOME はリース満了で
// 自動的に IDLE_P/IDLE_H へ戻る（override_common）ので run は要らない
// （run_state が null なので operationCardLayout が自然に隠す）。
test('走行ボタン: PANEL_NAV には出る、AT_PANEL には出ない', async ({ page }) => {
  await goto21(page, { mode: 'PANEL_NAV', state: 'PAUSE' })
  await expect(page.locator('#s21 .op-run')).toBeVisible()
  await page.locator('#s21 .op-run').click()
  expect(
    (await triggers(page)).some((c) => c.trigger === 'ui.run'),
    '走行ボタンが ui.run を送っていない',
  ).toBe(true)

  await goto21(page, { mode: 'AT_PANEL', state: 'IDLE_P' })
  await expect(page.locator('#s21 .op-run')).toBeHidden()
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

// 2026-09-09 実機で確認: venue_navigator._on_select_pin は pin.id で照合する
// （名前では照合しない）。以前は `pin.name ?? pin.id` を panel_id に送っていて、
// このテストの PINS フィクスチャが偶然すべて非空の name を持っていたため
// （'盤 A-1' 等）バグに気づけなかった。実機の大半のピンは改名しておらず
// name が空文字 '' で、`??` は null/undefined しかフォールバックしないので
// 空文字はすり抜け、panel_id:'' が送られて常に「ピンが見つかりません」に
// なっていた（選択の黄枠自体は画面ローカルの状態なので正常に見えた）。
// 期待値を id（'p1'）に直し、name が空でも壊れないことも別テストで確認する。
test('ピンの「移動」は select_pin を保留し、成功後に ui.goto{PANEL}（panel_id は id）', async ({ page }) => {
  await goto21(page, IDLE, { selectPin: { success: true, message: '' } })
  await page.locator('[data-testid="s21-pin-move-p1"]').click()
  const calls = await onsiteServiceCalls(page)
  expect(calls[0].service).toBe('/onsite/select_pin')
  expect(calls[0].request).toMatchObject({ panel_id: 'p1' })
  const gotos = (await triggers(page)).filter((c) => c.trigger === 'ui.goto')
  expect(gotos).toHaveLength(1)
  expect(gotos[0].argJson).toMatchObject({ kind: 'PANEL' })
})

test('ピンの「移動」が reject されたらメッセージを出し、ui.goto を送らない', async ({ page }) => {
  await goto21(page, IDLE, { selectPin: { success: false, message: 'そのピンには行けません' } })
  await page.locator('[data-testid="s21-pin-move-p1"]').click()
  await expect(page.locator('[data-testid="s21-go-err"]')).toContainText('そのピンには行けません')
  const calls = await onsiteServiceCalls(page)
  expect(calls[0].request).toMatchObject({ panel_id: 'p1' })
  expect((await triggers(page)).filter((c) => c.trigger === 'ui.goto'), 'reject なのに ui.goto を送っている').toHaveLength(0)
})

// 名前が未設定（空文字 ''）のピンでも panel_id が空にならないこと（実機で
// 踏んだ不具合の直接の再現）。
test('ピンの「移動」: 名前が空文字のピンでも panel_id は id が送られる', async ({ page }) => {
  await goto21(page, IDLE, {
    pins: [
      { id: 'panel_1', name: '', kind: 'PANEL', registered_at: '',
        pose: { position: { x: 0.9, y: -0.06, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } },
    ],
    selectPin: { success: true, message: '' },
  })
  await page.locator('[data-testid="s21-pin-move-panel_1"]').click()
  const calls = await onsiteServiceCalls(page)
  expect(calls[0].request).toMatchObject({ panel_id: 'panel_1' })
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
test('見失った状態でも候補をタップして選び直せる（SUMMON/POINT）', async ({ page }) => {
  await goto21(page, { mode: 'SUMMON', state: 'POINT' }, { targets: { ...TARGETS, is_lost: true } })
  await page.getByRole('tab', { name: '対象選択' }).click()

  await expect(page.locator('[data-testid="radar-lost-reselect"]')).toBeVisible()
  await expect(page.locator('[data-testid="radar-cand-1"]')).toBeVisible()

  await page.locator('[data-testid="radar-cand-1"]').click()
  const sel = (await triggers(page)).find((c) => c.trigger === 'ui.select_target')
  expect(sel, '見失い中のタップが ui.select_target を送っていない').toBeTruthy()
  expect(sel.argJson).toMatchObject({ index: 1 })
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

// brief-MAP-COSTMAP: costmap（/global_costmap/costmap）と直近の Nav2 経路（/plan）が
// 地図タブに重なる。シードは gotoScreenWithOnsite の costmap / plannedPath オプション
// （__thTestCostmap / __thTestPlannedPath）経由。testid は s21-map-costmap /
// s21-map-path。
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

