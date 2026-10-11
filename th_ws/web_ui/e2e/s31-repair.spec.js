// e2e/s31-repair.spec.js — S-31 故障診断 (WP-MAINT-03). NG の内容に応じたチェック手順が出る
// （手順の正本は th_maintenance の repair_hints.yaml → generated/repair_hints.json）。
// 「一覧へ戻る」は ui.stop を実際に送る。
import { test, expect } from '@playwright/test'
import { stubTrigger } from './helpers.js'

async function gotoS31(page, opcheckStatus) {
  await page.addInitScript(({ os }) => {
    window.__thTestState = { mode: 'OPCHECK', state: 'REPAIR' }
    window.__thTestScreen = 'S31'
    if (os) window.__thTestOpcheckStatus = os
  }, { os: opcheckStatus })
  await page.goto('/')
}

test('ESTOP no_press: 症状と、バイパス確認を含むチェック手順が出る', async ({ page }) => {
  await gotoS31(page, { item: 'ESTOP', result: 'NG', detail: 'no_press', next_screen: 'repair' })
  await expect(page.getByTestId('s31-symptom-ESTOP')).toContainText('押下を検出できませんでした')
  const hints = page.getByTestId('s31-hints-ESTOP')
  await expect(hints).toContainText('チェック手順')
  await expect(hints).toContainText('ESTOP_BENCH_TEST_BYPASS')
  await expect(hints.locator('li')).not.toHaveCount(0)
})

test('MOTOR: 理由ごとに違う手順が出る（符号逆と片輪の追従なし）', async ({ page }) => {
  await gotoS31(page, { item: 'MOTOR', result: 'NG', detail: 'sign_mismatch_L', next_screen: 'repair' })
  await expect(page.getByTestId('s31-hints-MOTOR')).toContainText('エンコーダの極性')
  await expect(page.getByTestId('s31-hints-MOTOR')).toContainText('左輪')
})

test('MOTOR no_follow_R は右輪の手順', async ({ page }) => {
  await gotoS31(page, { item: 'MOTOR', result: 'NG', detail: 'no_follow_R', next_screen: 'repair' })
  await expect(page.getByTestId('s31-hints-MOTOR')).toContainText('右輪')
  await expect(page.getByTestId('s31-hints-MOTOR')).not.toContainText('左輪')
})

// brief-a13（SG-A13）: 確定 NG の理由は `DIR:reason` 形。症状は方向込みで出て、
// 手順は素の理由で引く（strip しないと手順が出ない）。
test('MOTOR の DIR:付き理由は方向込みの症状と素の理由の手順が出る', async ({ page }) => {
  await gotoS31(page, { item: 'MOTOR', result: 'NG', detail: 'LEFT:sign_mismatch_L', next_screen: 'repair' })
  await expect(page.getByTestId('s31-symptom-MOTOR')).toContainText('左旋回')
  await expect(page.getByTestId('s31-hints-MOTOR')).toContainText('エンコーダの極性')
})

test('症状が無ければ手順は出さない', async ({ page }) => {
  await gotoS31(page, null)
  await expect(page.getByTestId('s31-hints-ESTOP')).toHaveCount(0)
  await expect(page.getByTestId('s31-hints-MOTOR')).toHaveCount(0)
})

test('「一覧へ戻る」が ui.stop を送る', async ({ page }) => {
  await stubTrigger(page, { 'ui.stop': { accepted: true } })
  await gotoS31(page, { item: 'MOTOR', result: 'NG', detail: 'no_samples', next_screen: 'repair' })
  await page.getByTestId('s31-back').click()
  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(calls.some((c) => c.trigger === 'ui.stop')).toBeTruthy()
})

// 実機 2026-10-10: 番号付きの手順が枠(card)の左端より外へはみ出していた。
// 番号(list marker)は li の左に付くので、li の左端が card の内側の余白より右にあれば
// 番号も枠内に収まる（marker 幅は li の padding-left 分の内側に入るよう確保している）。
test('チェック手順の番号は枠の内側に収まる', async ({ page }) => {
  await gotoS31(page, { item: 'MOTOR', result: 'NG', detail: 'sign_mismatch_L', next_screen: 'repair' })
  const card = page.getByTestId('s31-hints-MOTOR')
  await expect(card.locator('li').first()).toBeVisible()
  const m = await page.evaluate(() => {
    const card = document.querySelector('[data-testid="s31-hints-MOTOR"]')
    const ol = card.querySelector('ol')
    const li = ol.querySelector('li')
    const cs = getComputedStyle(ol)
    const c = card.getBoundingClientRect()
    const l = li.getBoundingClientRect()
    return {
      cardLeft: c.left, liLeft: l.left, liRight: l.right, cardRight: c.right,
      pad: parseFloat(cs.paddingLeft), pos: cs.listStylePosition, type: cs.listStyleType,
      fs: parseFloat(getComputedStyle(li).fontSize),
    }
  })
  // 番号は li の左端の外側（outside）に描かれる。幅は桁数×約 0.6em + 余白。
  const markerLeft = m.liLeft - m.fs * 1.5
  expect(m.pos).toBe('outside')
  expect(m.type).toBe('decimal')
  expect(markerLeft).toBeGreaterThanOrEqual(m.cardLeft)
  expect(m.liRight).toBeLessThanOrEqual(m.cardRight)
})
