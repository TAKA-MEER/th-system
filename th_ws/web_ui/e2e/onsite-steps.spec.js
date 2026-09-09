// brief-onsite-ux UX-1: 手順バー（ステッパー）と「次にやること」ボタン。
// 受け入れ条件: S-20 / S-21 に手順バーが出て現在の段がわかる（aria-current="step"）
// こと、次操作ボタンで必要なタブ/サブタブが自動で切り替わること、そして
// 「押すと対応する trigger / service を本当に送る」こと（ボタン保険）。
import { test, expect } from '@playwright/test'
import {
  gotoScreenWithOnsite, onsiteServiceCalls, setTestHomeDeclared, setTestState, unlockOnsiteMap,
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
const HOME_ONLY = [pin('h1', '待機場所', 'HOME', 70, 190, 274)]
const PANEL_NO_HOME = [pin('p1', '盤 A-1', 'PANEL', 86, 88, 92)]

const TARGETS = {
  candidates: [{ x: 1.2, y: 0.4 }, { x: -0.8, y: 0.6 }],
  selected_index: 0,
  confidence: 0.87,
  is_lost: false,
  lost_reason: '',
}
const NO_TARGET = { ...TARGETS, selected_index: -1, confidence: 0 }

const PREP = { mode: 'PREP', state: 'MAPPING' }
const IDLE = { mode: 'IDLE', state: 'NONE' }

async function triggers(page) {
  return page.evaluate(() => window.__thTriggerCalls ?? [])
}

// ── S-20 ─────────────────────────────────────────────────

test('S-20: 手順バーが出て現在段に aria-current が付く（空→段2）', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, { pins: [], targets: NO_TARGET, mappingActive: true })
  await page.locator('#s20').waitFor()
  await unlockOnsiteMap(page)

  await expect(page.locator('[data-testid="s20-stepper"]')).toBeVisible()
  await expect(page.locator('[data-testid="step-1"]')).toHaveClass(/done/)
  await expect(page.locator('[data-testid="step-2"]')).toHaveAttribute('aria-current', 'step')
  await expect(page.locator('[data-testid="step-2"]')).not.toHaveClass(/done/)
  // 段1の丸に ✓（done）が出る（SVG が在る）。
  await expect(page.locator('[data-testid="step-1"] .stepbar-check')).toBeVisible()
})

test('S-20: 「対象を選ぶ」で対象選択タブに切り替わる', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, { pins: [], targets: NO_TARGET, mappingActive: true })
  await page.locator('#s20').waitFor()
  await unlockOnsiteMap(page)

  const btn = page.locator('[data-testid="s20-next-action"]')
  await expect(btn).toHaveText('対象を選ぶ')
  await btn.click()
  await expect(page.locator('[data-testid="s20-tab-target"]')).toHaveAttribute('aria-selected', 'true')
  await expect(page.locator('[data-testid="radar-cand-0"]')).toBeVisible()
})

test('S-20: 「待機場所を登録」で登録サブタブへ切り替わり ui.register{HOME}', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, { pins: [], targets: TARGETS, mappingActive: true })
  await page.locator('#s20').waitFor()
  await unlockOnsiteMap(page)

  const btn = page.locator('[data-testid="s20-next-action"]')
  await expect(btn).toHaveText('待機場所を登録')
  await btn.click()
  await expect(page.locator('[data-testid="s20-subtab-register"]')).toHaveAttribute('aria-selected', 'true')
  const reg = (await triggers(page)).filter((c) => c.trigger === 'ui.register')
  expect(reg, '待機場所の登録が ui.register を送っていない').toHaveLength(1)
  expect(reg[0].argJson).toMatchObject({ kind: 'HOME' })
})

test('S-20: 「配電盤を登録」で ui.register{PANEL}', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, { pins: HOME_ONLY, targets: TARGETS, mappingActive: true })
  await page.locator('#s20').waitFor()
  await unlockOnsiteMap(page)

  const btn = page.locator('[data-testid="s20-next-action"]')
  await expect(btn).toHaveText('配電盤を登録')
  await btn.click()
  const reg = (await triggers(page)).filter((c) => c.trigger === 'ui.register')
  expect(reg, '配電盤の登録が ui.register を送っていない').toHaveLength(1)
  expect(reg[0].argJson).toMatchObject({ kind: 'PANEL' })
})

test('S-20: 全部完了（保存まで済み）でも現在は段5、保存ボタンが動く', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, { pins: PINS, targets: TARGETS, mappingActive: true })
  await page.locator('#s20').waitFor()
  await unlockOnsiteMap(page)

  await expect(page.locator('[data-testid="step-5"]')).toHaveAttribute('aria-current', 'step')
  const btn = page.locator('[data-testid="s20-next-action"]')
  await expect(btn).toHaveText('保存')
  await btn.click()
  expect((await triggers(page)).some((c) => c.trigger === 'ui.save'), '保存が ui.save を送っていない').toBe(true)
})

test('S-20: 2 点指示ウィザード（REGISTER）中は次操作ボタンを隠す', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, { pins: HOME_ONLY, targets: TARGETS, mappingActive: true })
  await page.locator('#s20').waitFor()
  await unlockOnsiteMap(page)

  await expect(page.locator('[data-testid="s20-next-action"]')).toBeVisible()
  await setTestState(page, { state: 'REGISTER' })
  await expect(page.locator('[data-testid="s20-wizard"]')).toBeVisible()
  await expect(page.locator('[data-testid="s20-next-action"]')).toBeHidden()
})

// ── S-21 ─────────────────────────────────────────────────

async function goto21(page, state = IDLE, extra = {}) {
  await gotoScreenWithOnsite(page, 'S21', state, {
    pins: PINS, targets: TARGETS, pose: { x: 0, y: 0, yaw: 0 }, ...extra,
  })
  await page.locator('#s21').waitFor()
}

test('S-21: 手順バーが出て現在段は段1（会場地図を開く）', async ({ page }) => {
  await goto21(page)
  await expect(page.locator('[data-testid="s21-stepper"]')).toBeVisible()
  await expect(page.locator('[data-testid="step-1"]')).toHaveAttribute('aria-current', 'step')
  await expect(page.locator('[data-testid="s21-next-action"]')).toHaveText('会場地図を開く')
})

test('S-21: 「会場地図を開く」で /map_session/open を呼び段が進む', async ({ page }) => {
  await goto21(page, IDLE, { openVenueMap: { success: true, message: '' } })

  const btn = page.locator('[data-testid="s21-next-action"]')
  await expect(btn).toHaveText('会場地図を開く')
  await btn.click()
  const open = (await onsiteServiceCalls(page)).find((c) => c.service === '/map_session/open')
  expect(open, '「会場地図を開く」が /map_session/open を呼んでいない').toBeTruthy()
  expect(open.request).toMatchObject({ slot: 'VENUE', mode: 'reload', session_id: 'venue' })

  // 成功で段1完了 → 現在は段2（待機場所を宣言）。
  await expect(page.locator('[data-testid="step-1"]')).toHaveClass(/done/)
  await expect(page.locator('[data-testid="step-2"]')).toHaveAttribute('aria-current', 'step')
  await expect(btn).toHaveText('待機場所を宣言')
})

test('S-21: 「待機場所を宣言」で declare_home(force:false) を呼び段3へ', async ({ page }) => {
  await goto21(page, IDLE, {
    openVenueMap: { success: true, message: '' },
    declareHome: { success: true, offset_m: 0, offset_deg: 0, message: '' },
  })

  const btn = page.locator('[data-testid="s21-next-action"]')
  await expect(btn).toHaveText('会場地図を開く')
  await btn.click()
  await expect(btn).toHaveText('待機場所を宣言')
  await btn.click()
  const dh = (await onsiteServiceCalls(page)).find((c) => c.service === '/onsite/declare_home')
  expect(dh, '宣言が /onsite/declare_home を呼んでいない').toBeTruthy()
  expect(dh.request).toMatchObject({ force: false })

  // home_declared の受信で段2完了 → 段3（行き先を選ぶ）。
  await setTestHomeDeclared(page, true)
  await expect(page.locator('[data-testid="step-2"]')).toHaveClass(/done/)
  await expect(page.locator('[data-testid="step-3"]')).toHaveAttribute('aria-current', 'step')
  await expect(btn).toHaveText('行き先を選ぶ')
})

test('S-21: 「行き先を選ぶ」で行き先サブタブ＋ピン選択を促す', async ({ page }) => {
  await goto21(page, IDLE, {
    openVenueMap: { success: true, message: '' },
    homeDeclared: true,
  })

  const btn = page.locator('[data-testid="s21-next-action"]')
  await expect(btn).toHaveText('会場地図を開く')
  await btn.click() // 段1（会場地図を開く）
  // 段2（宣言）は homeDeclared 済みなのでスキップ → 段3（行き先を選ぶ）。
  await expect(btn).toHaveText('行き先を選ぶ')
  await btn.click()
  await expect(page.locator('[data-testid="s21-subtab-dest"]')).toHaveAttribute('aria-selected', 'true')
  await expect(page.locator('[data-testid="s21-panel-pick-hint"]')).toBeVisible()
})

// 2026-09-09 実機フィードバック「移動を押すと停止ボタンが2つ出て両方反応しない」:
// 段4（move、移動中）の「次にやること」は以前 kind:'stop' で ui.stop を送るだけの
// ボタンだったが、操作カードに常時ある「停止」（.op-stop）と全く同じ操作の重複
// だった。段4では「次にやること」自体を出さず、操作カードの停止だけにする
// （UX-6-c が「保存」で既にやっている「同じ操作を二重に出さない」原則を停止にも
// 適用）。
test('S-21: 移動中は「次にやること」を出さない（操作カードの停止のみ）、作業中は「作業中」で ui.working', async ({ page }) => {
  // PANEL_NAV: 段1〜3が完了（open・home・nav）だが段4（AT_PANEL）未完了 → current=段4。
  await goto21(page, { mode: 'PANEL_NAV', state: 'NAV' }, {
    openVenueMap: { success: true, message: '' },
    homeDeclared: true,
  })
  const btn = page.locator('[data-testid="s21-next-action"]')
  await expect(btn).toHaveText('会場地図を開く')
  await btn.click() // 段1（会場地図を開く）
  await expect(page.locator('[data-testid="step-4"]')).toHaveAttribute('aria-current', 'step')
  await expect(btn).toBeHidden()
  // 停止は操作カード側（常時表示）だけで送る。
  await page.locator('#s21 .op-stop').click()
  expect((await triggers(page)).some((c) => c.trigger === 'ui.stop'), '停止が ui.stop を送っていない').toBe(true)

  // AT_PANEL に着くと段4完了 → current=段5「作業中」。
  await setTestState(page, { mode: 'AT_PANEL', state: 'IDLE_P' })
  await expect(page.locator('[data-testid="step-5"]')).toHaveAttribute('aria-current', 'step')
  await expect(btn).toHaveText('作業中')
  await btn.click()
  const w = (await triggers(page)).filter((c) => c.trigger === 'ui.working')
  expect(w, '作業中トグルが ui.working を送っていない').toHaveLength(1)
})

test('S-21: 2 点指示ウィザード（SUMMON/POINT）中は次操作ボタンを隠す', async ({ page }) => {
  await goto21(page, { mode: 'SUMMON', state: 'POINT' }, {
    openVenueMap: { success: true, message: '' },
  })
  await page.locator('[data-testid="s21-subtab-summon"]').click()
  await expect(page.locator('[data-testid="s21-wizard"]')).toBeVisible()
  await expect(page.locator('[data-testid="s21-next-action"]')).toBeHidden()
})