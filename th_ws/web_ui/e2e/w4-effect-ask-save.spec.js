// 1b-7 (SG-B3): /system/effect の ask_save／ask_save_if_unsaved が W-4 の
// 保存確認に出る。「はい」→ ui.save（保存）、「いいえ」→ ui.discard（破棄）。
// 本番の購読 hook（useSystemEffect -> useSaveConfirm -> Windows）を通す
// （部品に直接 props を渡さない）。ソケットは張らず TEST_MODE の種で駆動する。
// 変異: ask の振り分けを捨てる／未保存判定を外す／ボタンの送信を空にすると
// 赤くなること。
import { test, expect } from '@playwright/test'

function askMsg(name, routeId = null) {
  const args = routeId ? { route_id: routeId } : {}
  return { name, dest: 'WebUI', args_json: JSON.stringify(args) }
}

async function gotoWithAsk(page, state, effect, status) {
  await page.addInitScript(({ s, e, st }) => {
    window.__thTestState = s
    window.__thTestScreen = 'S01'
    if (e) window.__thTestSystemEffect = e
    if (st) window.__thTestRouteStatus = st
  }, { s: state, e: effect, st: status })
  await page.goto('/')
}

async function triggerCalls(page) {
  return page.evaluate(() => window.__thTriggerCalls ?? [])
}

const UNSAVED = { points: 12, saved: false, state: 'REC', recorded_m: 5.0, elapsed_sec: 30 }

test('「終了」の ask_save_if_unsaved＋未保存で W-4 が出て経路名が見える', async ({ page }) => {
  await gotoWithAsk(
    page,
    { mode: 'IDLE', state: 'NONE' },
    askMsg('ask_save_if_unsaved', 'demo-route'),
    UNSAVED,
  )
  await expect(page.getByTestId('w4-save-ask')).toBeVisible()
  await expect(page.getByTestId('w4-save-ask')).toContainText('demo-route')
})

test('W-4「保存する」で ui.save が送られ窓が閉じる', async ({ page }) => {
  await gotoWithAsk(
    page,
    { mode: 'IDLE', state: 'NONE' },
    askMsg('ask_save_if_unsaved', 'demo-route'),
    UNSAVED,
  )
  await page.getByTestId('w4-save-yes').click()
  const calls = await triggerCalls(page)
  expect(calls.some((c) => c.trigger === 'ui.save'), 'ui.save が送られていない').toBe(true)
  await expect(page.getByTestId('w4-save-ask')).toHaveCount(0)
})

test('W-4「破棄する」は二段階で、二度押しで ui.discard が送られ窓が閉じる', async ({ page }) => {
  await gotoWithAsk(
    page,
    { mode: 'IDLE', state: 'NONE' },
    askMsg('ask_save', 'demo-route'),
    UNSAVED,
  )
  await expect(page.getByTestId('w4-save-ask')).toBeVisible()
  const noButton = page.getByRole('button', { name: '破棄する' })
  await noButton.click()
  // 1 回目はアームだけ（送らない）。
  let calls = await triggerCalls(page)
  expect(calls.some((c) => c.trigger === 'ui.discard'), '一度押しで送ってはいけない').toBe(false)
  await expect(page.getByRole('button', { name: '本当に破棄する' })).toBeVisible()
  await page.getByRole('button', { name: '本当に破棄する' }).click()
  calls = await triggerCalls(page)
  expect(calls.some((c) => c.trigger === 'ui.discard'), 'ui.discard が送られていない').toBe(true)
  await expect(page.getByTestId('w4-save-ask')).toHaveCount(0)
})

test('未保存が無ければ W-4 は出ない（保存済み・空・未到来・再生の status）', async ({ page }) => {
  // 保存済み。
  await gotoWithAsk(
    page, { mode: 'IDLE', state: 'NONE' },
    askMsg('ask_save_if_unsaved', 'demo-route'),
    { points: 12, saved: true },
  )
  await expect(page.getByTestId('w4-save-ask')).toHaveCount(0)

  // 再生側の status（current.id あり）は記録の未保存とみなさない。
  await gotoWithAsk(
    page, { mode: 'IDLE', state: 'NONE' },
    askMsg('ask_save_if_unsaved', 'demo-route'),
    { points: 30, saved: false, current: { id: 'r1' } },
  )
  await expect(page.getByTestId('w4-save-ask')).toHaveCount(0)
})

test('未保存が無い ask_save_if_unsaved（他モードの終了）では W-4 は出ない', async ({ page }) => {
  await gotoWithAsk(
    page, { mode: 'IDLE', state: 'NONE' },
    askMsg('ask_save_if_unsaved'),
    null,
  )
  await expect(page.getByTestId('w4-save-ask')).toHaveCount(0)
})
