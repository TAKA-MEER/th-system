// WS-9P (2026-09-04): S-14 の一時停止の文言は「なぜ止まったか」で変える。
//
// 実機フィードバック: ESP32 が一瞬途切れて止まっただけなのに、画面が
// 「一時停止（記録の終端に達しました。終了を押してください）」と出していた。
// 操作者は終了するしかないと判断してしまう。実際には「再生」を押せば
// 止まったところから続きを走れる（T-REPLAY-07 / resume_path。index は保持）。
//
// 状態名だけでは区別できない。終端到達（T-REPLAY-09）もフォルト（C-03）も
// ジョグ介入（C-01）も停止ボタン（T-REPLAY-06）も、同じ PAUSE に落ちる。
// replay_runner が RouteStatus.arrived を載せてくるので、それで出し分ける。
import { test, expect } from '@playwright/test'
import { gotoScreenWithRoutePreview } from './helpers.js'

const ROUTES = [{ id: 'r1', name: 'r1', length_m: 5, point_count: 6 }]
const PREVIEW = [{ x: 0, y: 0 }, { x: 1, y: 0 }, { x: 2, y: 0 }]

async function openPaused(page, arrived) {
  await gotoScreenWithRoutePreview(page, 'S14', { mode: 'REPLAY', state: 'PAUSE' }, {
    routes: ROUTES,
    preview: PREVIEW,
    status: { state: 'PAUSE', target_index: 1, points: 6, arrived },
  })
}

test('終端に達していない一時停止は「再生で続きから」を案内する', async ({ page }) => {
  await openPaused(page, false)
  const body = page.locator('#s14')
  await expect(body).toContainText('再生')
  await expect(body).toContainText('止まったところから')
  await expect(body).not.toContainText('記録の終端に達しました')
})

test('終端に達した一時停止は従来どおり終了を促す', async ({ page }) => {
  await openPaused(page, true)
  const body = page.locator('#s14')
  await expect(body).toContainText('記録の終端に達しました')
})

test('arrived が未設定（古いノード）でも終了を促す文言にはしない', async ({ page }) => {
  // RouteStatus.arrived を載せない replay_runner と繋いだ場合、undefined になる。
  // 「終わった」と誤って案内するより「続きから走れる」に倒す方が安全側
  // （終端なら再生を押しても即 PAUSE に戻るだけで、機体は動かない）。
  await openPaused(page, undefined)
  await expect(page.locator('#s14')).not.toContainText('記録の終端に達しました')
})
