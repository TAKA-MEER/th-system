// brief-tracker-default-off §3.4: 「人検出を開始/停止」が対象選択タブから
// /system/set_flag tracker_enabled を本当に送ること、および既定 OFF のときの
// 2 点指示の登録ボタン・呼び寄せの非活性を e2e で縛る。
//
// th_state は無いので tracker_enabled は addInitScript で注入する（このスペック
// は「既定 OFF」を検証するので、明示しない限り false —— helpers の goto21 と
// 違い、ここは自分で gotoScreenWithOnsite を呼ぶ）。送信は __thSetFlagCalls
// （ros/useSetFlag.js のテストフック）で数える。変異③（「人検出を開始」が
// onClick 空で何も送らない）は、この配列が空のままになるため赤。
import { test, expect } from '@playwright/test'
import { gotoScreenWithOnsite, setFlagCalls, setTestState, stubSetFlag } from './helpers.js'

const PREP_OFF = { mode: 'PREP', state: 'MAPPING' }

const SUMMON_ON = { mode: 'SUMMON', state: 'POINT', tracker_enabled: true }

const PINS = [
  {
    id: 'p1', name: '盤 A-1', kind: 'PANEL', registered_at: '',
    pose: {
      position: { x: 1, y: 0, z: 0 },
      orientation: { x: 0, y: 0, z: 0, w: 1 },
    },
  },
]

const TARGETS = {
  candidates: [{ x: 1.2, y: 0.4 }, { x: -0.8, y: 0.6 }],
  selected_index: 0,
  confidence: 0.87,
  is_lost: false,
  lost_reason: '',
}

async function openS20(page, state = PREP_OFF) {
  await gotoScreenWithOnsite(page, 'S20', state, {
    pins: PINS, targets: TARGETS, pose: { x: 0, y: 0, yaw: 0 }, mappingActive: true,
  })
  await page.locator('#s20').waitFor()
}

async function openS21(page, state = SUMMON_ON) {
  await gotoScreenWithOnsite(page, 'S21', state, {
    pins: PINS, targets: TARGETS, pose: { x: 0, y: 0, yaw: 0 },
  })
  await page.locator('#s21').waitFor()
}

function startCall(calls) {
  return calls.find((c) => c.flag === 'tracker_enabled' && c.value === true)
}

// ── S-20（試験準備） ─────────────────────────────────────────────────────────

test('S-20: 人検出が止まっている間は対象選択タブの中心に「人検出を開始」でレーダーは出ない', async ({ page }) => {
  await openS20(page, PREP_OFF)
  await page.getByRole('tab', { name: '対象選択' }).click()

  await expect(page.locator('[data-testid="s20-tracker-off"]')).toHaveText('人検出が止まっています')
  await expect(page.locator('[data-testid="s20-tracker-start"]')).toBeVisible()
  // 候補は定義上無いのでレーダーは出さない。
  await expect(page.locator('[data-testid="radar"]')).toHaveCount(0)

  // 「人検出を開始」が本当に /system/set_flag tracker_enabled=true を送る
  // （変異③: onClick 空にするとこの検証が赤になる）。
  await stubSetFlag(page, {
    tracker_enabled: (value) => ({ accepted: true, reject_reason_key: null }),
  })
  await page.locator('[data-testid="s20-tracker-start"]').click()
  const calls = await setFlagCalls(page)
  const start = startCall(calls)
  expect(start, '「人検出を開始」が tracker_enabled=true を送っていない').toBeTruthy()
  expect(start.requester).toBe('s20-target')
})

test('S-20: 人検出が止まっている間は 2 点指示の登録ボタンが非活性で注記が出る', async ({ page }) => {
  await openS20(page, PREP_OFF)
  await page.getByRole('tab', { name: '登録' }).click()

  await expect(page.locator('[data-testid="s20-reg-tracker-off"]')).toHaveText('人検出が止まっています')
  await expect(page.locator('[data-testid="s20-reg-home"]')).toBeDisabled()
  await expect(page.locator('[data-testid="s20-reg-panel"]')).toBeDisabled()
  // 機体姿勢での直接登録（ROBOT_POSE）は人検出を要らないので非活性にしない。
  await expect(page.locator('[data-testid="s20-reg-home-here"]')).toBeEnabled()

  // 人検出を開始すると（状態の正本は server なので setTestState で再現）有効に戻る。
  await setTestState(page, { tracker_enabled: true })
  await expect(page.locator('[data-testid="s20-reg-tracker-off"]')).toHaveCount(0)
  await expect(page.locator('[data-testid="s20-reg-home"]')).toBeEnabled()
})

test('S-20: 人検出が動いている間は「人検出を停止」が出て、送信は value:false', async ({ page }) => {
  await openS20(page, { ...PREP_OFF, tracker_enabled: true })
  await page.getByRole('tab', { name: '対象選択' }).click()

  await expect(page.locator('[data-testid="s20-tracker-stop"]')).toBeVisible()
  // 候補が出ていれば「動作中」、レーダーも表示する。
  await expect(page.locator('[data-testid="s20-tracker-status"]')).toHaveText('人検出 動作中')
  await expect(page.locator('[data-testid="radar-cand-0"]')).toBeVisible()

  await page.locator('[data-testid="s20-tracker-stop"]').click()
  const calls = await setFlagCalls(page)
  const stop = calls.find((c) => c.flag === 'tracker_enabled' && c.value === false)
  expect(stop, '「人検出を停止」が tracker_enabled=false を送っていない').toBeTruthy()
})

test('S-20: 起動中（フラグ true だが候補なし）は「起動中…」と表示する', async ({ page }) => {
  await openS20(page, { ...PREP_OFF, tracker_enabled: true })
  await page.evaluate(() => {
    window.__thSetTestPersonTargets({
      candidates: [], selected_index: -1, confidence: 0, is_lost: false, lost_reason: '',
    })
  })
  await page.getByRole('tab', { name: '対象選択' }).click()

  await expect(page.locator('[data-testid="s20-tracker-status"]')).toHaveText('起動中…')
  await expect(page.locator('[data-testid="radar-empty"]')).toBeVisible()
})

test('S-20: REGISTER では「人検出を停止」が押せず理由が出る（MAPPING では押せる）', async ({ page }) => {
  await openS20(page, { ...PREP_OFF, tracker_enabled: true })
  await page.getByRole('tab', { name: '対象選択' }).click()

  // MAPPING → 停止は押せる。
  await expect(page.locator('[data-testid="s20-tracker-stop"]')).toBeEnabled()

  // REGISTER（2 点指示の登録中）→ 人検出が要るので停止を事前無効化し理由を出す。
  await setTestState(page, { state: 'REGISTER' })
  await expect(page.locator('[data-testid="s20-tracker-stop"]')).toBeDisabled()
  await expect(page.locator('[data-testid="s20-tracker-stop-denied"]')).toContainText('人検出')
})

// ── S-21（試験当日） ─────────────────────────────────────────────────────────

test('S-21: 人検出が止まっている間は対象選択タブの中心に「人検出を開始」', async ({ page }) => {
  await openS21(page, { mode: 'SUMMON', state: 'POINT' })
  await page.getByRole('tab', { name: '対象選択' }).click()

  await expect(page.locator('[data-testid="s21-tracker-start"]')).toBeVisible()
  await expect(page.locator('[data-testid="radar"]')).toHaveCount(0)

  await page.locator('[data-testid="s21-tracker-start"]').click()
  const start = startCall(await setFlagCalls(page))
  expect(start, '「人検出を開始」が tracker_enabled=true を送っていない').toBeTruthy()
})

test('S-21: SUMMON では「人検出を停止」が押せず理由が出る（人検出が要る）', async ({ page }) => {
  await openS21(page, SUMMON_ON)
  await page.getByRole('tab', { name: '対象選択' }).click()

  await expect(page.locator('[data-testid="s21-tracker-stop"]')).toBeDisabled()
  await expect(page.locator('[data-testid="s21-tracker-stop-denied"]')).toContainText('人検出')
  // SUMMON/POINT かつ人検出 ON → レーダーは従来どおり出る。
  await expect(page.locator('[data-testid="radar-cand-0"]')).toBeVisible()
})

test('S-21: 人検出が止まっている間は呼び寄せ（2 点指示）が非活性で注記が出る', async ({ page }) => {
  await openS21(page, { mode: 'SUMMON', state: 'POINT' })
  await page.getByRole('tab', { name: '呼び寄せ' }).click()

  await expect(page.locator('[data-testid="s21-summon-tracker-off"]')).toHaveText('人検出が止まっています')
  await expect(page.locator('[data-testid="s21-summon-start"]')).toBeDisabled()

  await setTestState(page, { tracker_enabled: true })
  await expect(page.locator('[data-testid="s21-summon-tracker-off"]')).toHaveCount(0)
  await expect(page.locator('[data-testid="s21-summon-start"]')).toBeEnabled()
})