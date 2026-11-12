// A-16（SM-3.1.1-10）: 試験準備で作った試験場内地図が未保存のまま「終了」したら W-4 で
// 保存可否を問う。「保存」で本当に保存の要求（/map_session/open slot=VENUE mode=save）が
// 送られ、失敗したら閉じずに理由を出す。保存済み（unsaved に venue_map が無い）なら出ない。
//
// 本番の経路（useSystemEffect -> useSaveConfirm -> Windows）を通す。機体の判定は
// /system/state.unsaved（運用の終了の一覧と同じ出どころ）なので、それを種にする。
// 変異: venueMapUnsaved を常に偽にする／保存の送信を外す／IDLE を見る前に閉じる → 赤。
import { test, expect } from '@playwright/test'

const askMsg = (name, routeId = null) => ({
  name, dest: 'WebUI', args_json: JSON.stringify(routeId ? { route_id: routeId } : {}),
})

async function goto(page, { state, effect, status = null, stubs = {} }) {
  await page.addInitScript(({ s, e, st, sb }) => {
    window.__thTestState = s
    window.__thTestScreen = 'S01'
    window.__thTestSystemEffect = e
    if (st) window.__thTestRouteStatus = st
    for (const [k, v] of Object.entries(sb)) window[k] = v
  }, { s: state, e: effect, st: status, sb: stubs })
  await page.goto('/')
}

const PREP_DIRTY = { mode: 'PREP', state: 'MAPPING', unsaved: ['venue_map'] }
const onsiteCalls = (page) => page.evaluate(() => window.__thOnsiteServiceCalls ?? [])
const triggers = async (page) => (await page.evaluate(() => window.__thTriggerCalls ?? [])).map((c) => c.trigger)
const stdCalls = async (page) => (await page.evaluate(() => window.__thStdTriggerCalls ?? [])).map((c) => c.service)

test('試験場内地図が未保存なら、効果が状態より先に届いても W-4 が出る（終了の遷移の順序）', async ({ page }) => {
  // 実機は ask の effect が先、/system/state（IDLE）が後。効果の時点のモードはまだ PREP。
  await goto(page, { state: PREP_DIRTY, effect: askMsg('ask_save_if_unsaved') })
  await page.evaluate(() => window.__thSetTestState({ mode: 'IDLE', state: 'NONE' }))
  const w4 = page.getByTestId('w4-save-ask')
  await expect(w4).toBeVisible()
  await expect(w4).toContainText('試験場内の地図')
  await expect(w4).toContainText('作り直す')
})

test('保存済み（unsaved に venue_map が無い）なら出ない', async ({ page }) => {
  await goto(page, {
    state: { mode: 'IDLE', state: 'NONE', unsaved: [] }, effect: askMsg('ask_save_if_unsaved'),
  })
  await expect(page.getByTestId('w4-save-ask')).toHaveCount(0)
})

test('「保存する」で試験場内地図の保存が送られ（ui.save ではない）、窓が閉じる', async ({ page }) => {
  await goto(page, {
    state: { ...PREP_DIRTY, mode: 'IDLE', state: 'NONE' }, effect: askMsg('ask_save_if_unsaved'),
    stubs: { __thTestSaveVenueMap: { success: true, message: '' } },
  })
  await page.getByTestId('w4-save-yes').click()
  await expect(page.getByTestId('w4-save-ask')).toHaveCount(0)
  const calls = await onsiteCalls(page)
  const save = calls.find((c) => c.request?.slot === 'VENUE' && c.request?.mode === 'save')
  expect(save, 'VENUE の保存要求が送られていない').toBeTruthy()
  expect(await triggers(page), '教示の記録は無いので ui.save を送ってはいけない').not.toContain('ui.save')
})

test('保存に失敗したら窓は閉じず、理由を出す（保存したように見せない）', async ({ page }) => {
  await goto(page, {
    state: { ...PREP_DIRTY, mode: 'IDLE', state: 'NONE' }, effect: askMsg('ask_save_if_unsaved'),
    stubs: { __thTestSaveVenueMap: { success: false, message: 'ジョグ中のため保存できません' } },
  })
  await page.getByTestId('w4-save-yes').click()
  await expect(page.getByTestId('w4-save-error')).toContainText('ジョグ中のため保存できません')
  await expect(page.getByTestId('w4-save-ask')).toBeVisible()
})

test('「破棄する」は二段階で、地図の破棄が送られる（保存は送らない）', async ({ page }) => {
  await goto(page, {
    state: { ...PREP_DIRTY, mode: 'IDLE', state: 'NONE' }, effect: askMsg('ask_save_if_unsaved'),
    stubs: { __thTestServices: { '/slam_control/discard_map': { success: true, message: '' } } },
  })
  await page.getByRole('button', { name: '破棄する' }).click()
  expect(await stdCalls(page), '一度押しで捨ててはいけない').toEqual([])
  await page.getByRole('button', { name: '本当に破棄する' }).click()
  await expect(page.getByTestId('w4-save-ask')).toHaveCount(0)
  expect(await stdCalls(page)).toContain('/slam_control/discard_map')
  expect((await onsiteCalls(page)).some((c) => c.request?.mode === 'save')).toBe(false)
})

test('教示の記録と地図の両方が未保存なら、一緒に保存する（地図の保存と ui.save）', async ({ page }) => {
  await goto(page, {
    state: { ...PREP_DIRTY, mode: 'IDLE', state: 'NONE' },
    effect: askMsg('ask_save_if_unsaved', 'demo-route'),
    status: { points: 12, saved: false },
    stubs: { __thTestSaveVenueMap: { success: true, message: '' } },
  })
  await expect(page.getByTestId('w4-save-ask')).toContainText('demo-route')
  await page.getByTestId('w4-save-yes').click()
  await expect(page.getByTestId('w4-save-ask')).toHaveCount(0)
  expect((await onsiteCalls(page)).some((c) => c.request?.mode === 'save')).toBe(true)
  expect(await triggers(page)).toContain('ui.save')
})

test('記録の途切れ（ask_save）は地図の未保存では開かない（地図の話ではない）', async ({ page }) => {
  await goto(page, {
    state: { mode: 'IDLE', state: 'NONE', unsaved: ['venue_map'] }, effect: askMsg('ask_save'),
  })
  await expect(page.getByTestId('w4-save-ask')).toHaveCount(0)
})
