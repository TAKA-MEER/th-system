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

// brief-tracker-default-off §3.4: このファイルの既存テストは「人検出が走って
// いる」前提（レーダーへの切替・2 点指示の登録）なので tracker_enabled:true を
// seed する（既定 OFF の挙動は e2e/tracker-toggle.spec.js が false で検証する）。
const PREP = { mode: 'PREP', state: 'MAPPING', tracker_enabled: true }
const IDLE = { mode: 'IDLE', state: 'NONE' }

async function triggers(page) {
  return page.evaluate(() => window.__thTriggerCalls ?? [])
}

// ── S-20 ─────────────────────────────────────────────────

// 画面の作り直し（Spec-webui.md §3.10）: 手順バー・「次にやること」ボタン・
// タブは無い（D87）。S-20 の旧テスト（s20-stepper / s20-next-action / タブ切替）は
// この段で削除し、S-20 の登録は e2e/s20-prep.spec.js が縛る。
// ここに残す S-20 の検証: 登録ボタン（方式「人の位置で」）が ui.register を送ること。
test('S-20: 方式「人の位置で」の登録ボタンが ui.register を送る', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, { pins: [], targets: TARGETS, mappingActive: true })
  await page.locator('#s20').waitFor()
  await page.locator('[data-testid="s20-method-person"]').click()
  await page.locator('[data-testid="s20-reg-home"]').click()
  const reg = (await triggers(page)).filter((c) => c.trigger === 'ui.register')
  expect(reg, '待機場所の登録が ui.register を送っていない').toHaveLength(1)
  expect(reg[0].argJson).toMatchObject({ kind: 'HOME' })

  await page.locator('[data-testid="s20-reg-panel"]').click()
  const reg2 = (await triggers(page)).filter((c) => c.trigger === 'ui.register')
  expect(reg2, '配電盤の登録が ui.register を送っていない').toHaveLength(2)
  expect(reg2[1].argJson).toMatchObject({ kind: 'PANEL' })
})

test('S-20: 保存ボタンが ui.save を送る', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, { pins: PINS, targets: TARGETS, mappingActive: true })
  await page.locator('#s20').waitFor()
  await page.locator('[data-testid="s20-save"]').click()
  expect((await triggers(page)).some((c) => c.trigger === 'ui.save'), '保存が ui.save を送っていない').toBe(true)
})

test('S-20: 2 点指示ウィザード（REGISTER）中は登録ボタンが手順の帯に置き換わる', async ({ page }) => {
  await gotoScreenWithOnsite(page, 'S20', PREP, { pins: HOME_ONLY, targets: TARGETS, mappingActive: true })
  await page.locator('#s20').waitFor()
  await expect(page.locator('[data-testid="s20-reg-home"]')).toBeVisible()
  await setTestState(page, { state: 'REGISTER' })
  await expect(page.locator('[data-testid="s20-wizard"]')).toBeVisible()
  await expect(page.locator('[data-testid="s20-reg-home"]')).toHaveCount(0)
})

// ── S-21 ─────────────────────────────────────────────────
// 作り直し（Spec-webui §3.11）: 手順バー・「次にやること」・待機場所の宣言は
// 廃止し、右列の固定ボタンと主表示の帯に置き換えた。ここでは S-21 の
// 「会場地図を開く」のゲートと呼び寄せ帯だけを縛る（操作の送付は
// s21-test.spec.js が縛る）。

async function goto21(page, state = IDLE, extra = {}) {
  await gotoScreenWithOnsite(page, 'S21', state, {
    pins: PINS, targets: TARGETS, pose: { x: 0, y: 0, yaw: 0 }, ...extra,
  })
  await page.locator('#s21').waitFor()
}

test('S-21: 「会場地図を開く」で /map_session/open を呼んで地図が出る', async ({ page }) => {
  await goto21(page, IDLE, { openVenueMap: { success: true, message: '' } })
  await expect(page.locator('[data-testid="s21-map"]')).toHaveCount(0)
  await page.locator('[data-testid="s21-map-gate-open"]').click()
  const open = (await onsiteServiceCalls(page)).find((c) => c.service === '/map_session/open')
  expect(open, '「会場地図を開く」が /map_session/open を呼んでいない').toBeTruthy()
  expect(open.request).toMatchObject({ slot: 'VENUE', mode: 'reload', session_id: 'venue' })
  await expect(page.locator('[data-testid="s21-map"]')).toBeVisible()
})

// 1b-5 (SG-A6): 移動中は「会場地図を開く」が押せない（画面の事前無効化。
// サーバ側も拒否する）。走行中に押して SLAM が再起動した件の再発防止。
test('S-21: 移動中（PANEL_NAV/NAV）は「会場地図を開く」が押せない', async ({ page }) => {
  await goto21(page, { mode: 'PANEL_NAV', state: 'NAV' }, {
    openVenueMap: { success: true, message: '' },
  })
  await expect(page.locator('[data-testid="s21-map-gate-open"]')).toHaveAttribute('aria-disabled', 'true')
  const open = (await onsiteServiceCalls(page)).filter((c) => c.service === '/map_session/open')
  expect(open, '移動中に /map_session/open が呼ばれている').toHaveLength(0)
})

test('S-21: 2 点指示ウィザード（SUMMON/POINT）は主表示の帯にそのまま出る（タブを押さない）', async ({ page }) => {
  await goto21(page, { mode: 'SUMMON', state: 'POINT', tracker_enabled: true }, {
    openVenueMap: { success: true, message: '' },
  })
  await expect(page.locator('[data-testid="s21-wizard"]')).toBeVisible()
  await expect(page.locator('[data-testid="s21-summon-band"]')).toBeVisible()
})
