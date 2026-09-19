// DetailedDesign-wp3.md WP-TRANSIT-01 §7: S-01's 手動走行 (mode MANUAL) button,
// once th_state accepts ui.enter_mode, must land on S-11 instead of staying on
// S-01 (this packet's whole point: "今回の目的そのもの").
//
// There is no th_state in the e2e environment, so the acceptance is stubbed
// via window.__thTestTrigger['ui.enter_mode'] = { accepted: true } — the same
// TEST_MODE pattern useStdTrigger.js uses for services (helpers.js stubTrigger,
// ros/useTrigger.js's test hook).
//
// 2026-09-02: 画面はもう「受理された瞬間」には動かない。main.jsx の
// resolveScreen() が SystemState.mode から画面を導出するようになったため、
// 実機と同じく **th_state が mode を変えて publish して初めて** 画面が動く。
// なので受理の stub に加えて setTestState で mode の反映まで再現する
// （これがロボット側が実際にやること）。楽観的に画面だけ進めていた頃の
// 契約は、モードと画面が乖離して操作不能になる不具合そのものだった。
import { test, expect } from '@playwright/test'
import { gotoScreen, stubTrigger, setTestState } from './helpers.js'

test('S-01 手動走行 accepted by th_state -> S-11 renders', async ({ page }) => {
  await stubTrigger(page, { 'ui.enter_mode': { accepted: true, reject_reason_key: '' } })
  await gotoScreen(page, 'S01', { mode: 'IDLE', tracker_enabled: true })

  const manual = page.getByRole('button', { name: '手動走行' })
  await expect(manual).toBeEnabled()
  await manual.click()

  // 受理されただけでは動かない（FSM がまだ IDLE を publish している段階）。
  await expect(page.locator('#s11')).toHaveCount(0)

  // th_state がモードを切り替えて publish した = 実機で起きること。
  await setTestState(page, { mode: 'MANUAL' })
  await expect(page.locator('#s11')).toBeVisible()
})

test('S-01 手動走行 rejected by th_state stays on S-01 with a reason window', async ({ page }) => {
  // Do not stub: useTrigger defaults non-stubbed triggers to a denial, so
  // this exercises the reject branch without a reason key (U-11: the UI
  // shows why even for a denied-but-visible button).
  await gotoScreen(page, 'S01', { mode: 'IDLE', tracker_enabled: true })

  const manual = page.getByRole('button', { name: '手動走行' })
  await manual.click()

  // Rejected: no S-11, reason window shown on S-01.
  await expect(page.locator('#s11')).toHaveCount(0)
  await expect(page.locator('.win.confirm.show')).toBeVisible()
})

// ── 回帰テスト: 2026-09-02 に直した「モードと画面の乖離で操作不能」 ──
// 以前の main.jsx は画面をローカル state だけで持っていたので、ロボット側が
// 独自にモードを戻しても画面は取り残された。画面は S-11 のまま FSM は IDLE、
// という状態では手動操作は全部拒否され、「終了」も既に IDLE なので効かず、
// どこにも行けなくなっていた。
test('ロボット側がモードを IDLE へ戻したら画面も S-01 へ追随する', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'MANUAL', tracker_enabled: true })
  await expect(page.locator('#s11')).toBeVisible()

  // 重大フォルトによる IDLE への強制遷移 / 他端末の操作 / ESTOP 復帰など、
  // UI を経由しないモード変更。
  await setTestState(page, { mode: 'IDLE' })

  await expect(page.locator('#s11')).toHaveCount(0)
  await expect(page.locator('#s01')).toBeVisible()
})
