// e2e/fault-history.spec.js — フォルト履歴の画面表示（Spec-webui.md §1・§3.15・§4.1、
// Spec-safety.md §3.5.5。2026-10-12）。
//
// 実機で、回復フォルトが 0.1 秒で解消して「何が起きたか」が読めなかった。
// 実機と同じ順序を再現する: フォルトが立つ → W-1 が出る → 0.1 秒で解消 →
// /system/fault_history の最新の 1 通（ongoing=false の記録入り）が届く →
// W-1 が再開確認に変わる。履歴の到着は window.__thSetTestFaultHistory
// （/system/fault_history の 1 通が届いた、の再現。TRANSIENT_LOCAL の最新 1 通）。
import { test, expect } from '@playwright/test'
import { gotoWithState, gotoScreen, setTestFault } from './helpers.js'

test.use({ timezoneId: 'Asia/Tokyo' })

// 12:34:56 JST
const T0 = Date.UTC(2026, 9, 12, 3, 34, 56) / 1000

const R = (o = {}) => ({
  fault_type: 'LIDAR_LOST', started_at: T0, duration_s: 0.1, ongoing: false,
  mode: 'PANEL_NAV', state: 'RUN', severity: 'RECOVERABLE', outcome: 'PAUSE', ...o,
})
const history = (...records) => ({ records })   // 新しい順
const setHistory = (page, msg) => page.evaluate((m) => window.__thSetTestFaultHistory(m), msg)

const PAUSED = { mode: 'PANEL_NAV', state: 'PAUSE', jog_active: false }
const lidar = (active) => ({ active, fault_type: 'LIDAR_LOST', severity: 'RECOVERABLE' })

test('(a) 0.1 秒で解消しても、再開確認に「解消した異常」が出て、再開を押すまで残る', async ({ page }) => {
  await gotoWithState(page, PAUSED)
  await setTestFault(page, lidar(true))
  await expect(page.locator('.win.fault.show')).toBeVisible()
  await setTestFault(page, lidar(false))
  await setHistory(page, history(R()))
  const win = page.locator('.win.fault.show')
  await expect(win.getByRole('button', { name: 'はい' })).toBeVisible()   // 再開確認に変わった
  await expect(win.getByTestId('w1-resolved-fault')).toHaveText('解消した異常：LiDAR 途絶（0.1 秒・12:34:56）')
  // 残る: 時間が経っても、他の履歴の更新があっても、再開を押すまで消えない。
  await page.waitForTimeout(700)
  await expect(win.getByTestId('w1-resolved-fault')).toBeVisible()
  await win.getByRole('button', { name: 'はい' }).click()
  const calls = await page.evaluate(() => window.__thTriggerCalls ?? [])
  expect(calls.map((c) => c.trigger)).toContain('ui.resume_yes')
})

test('(b) 同じ種別が短時間に続いたら「ほか N 回」', async ({ page }) => {
  await gotoWithState(page, PAUSED)
  await setTestFault(page, lidar(true))
  await setTestFault(page, lidar(false))
  await setHistory(page, history(
    R({ started_at: T0 }), R({ started_at: T0 - 5 }), R({ started_at: T0 - 20 }),
    R({ started_at: T0 - 200 }),                         // 60 秒より前: 数えない
    R({ started_at: T0 - 3, fault_type: 'MUX_DEAD' })))  // 別種別: 数えない
  await expect(page.getByTestId('w1-resolved-more')).toHaveText('ほか 2 回')
})

test('(g) 逆順: 履歴が解消より先に届く／解消より後に届く、どちらでも揃う', async ({ page }) => {
  // 先に届く（解消前）: 再開確認になるまでは出さない。
  await gotoWithState(page, PAUSED)
  await setTestFault(page, lidar(true))
  await setHistory(page, history(R({ ongoing: true, duration_s: -1 })))
  await expect(page.getByTestId('w1-resolved-fault')).toHaveCount(0)
  await setHistory(page, history(R()))                    // 解消の記録が先に届く
  await expect(page.getByTestId('w1-resolved-fault')).toHaveCount(0)  // fault はまだ active
  await setTestFault(page, lidar(false))
  await expect(page.getByTestId('w1-resolved-fault')).toHaveText('解消した異常：LiDAR 途絶（0.1 秒・12:34:56）')

  // 後から届く: 再開確認が先に出て、履歴が届いた時点で行が現れる。
  const p2 = await page.context().newPage()
  await gotoWithState(p2, PAUSED)
  await setTestFault(p2, lidar(true))
  await setTestFault(p2, lidar(false))
  await expect(p2.getByRole('button', { name: 'はい' })).toBeVisible()
  await expect(p2.getByTestId('w1-resolved-fault')).toHaveCount(0)
  await setHistory(p2, history(R()))
  await expect(p2.getByTestId('w1-resolved-fault')).toBeVisible()
})

test('(g2) 前のフォルトの記録は、別種別のフォルトの再開確認に出ない', async ({ page }) => {
  await gotoWithState(page, PAUSED)
  await setHistory(page, history(R({ fault_type: 'ESP32_DISCONNECTED', started_at: T0 - 600 })))
  await setTestFault(page, lidar(true))
  await setTestFault(page, lidar(false))
  await expect(page.getByRole('button', { name: 'はい' })).toBeVisible()
  await expect(page.getByTestId('w1-resolved-fault')).toHaveCount(0)
})

test('(c)(d) ヘッダに「直近の異常」が出て、押すと一覧の窓が開く', async ({ page }) => {
  await gotoWithState(page, { mode: 'IDLE', state: 'IDLE' })
  await expect(page.getByTestId('recent-fault')).toHaveCount(0)
  await setHistory(page, history(
    R({ fault_type: 'MUX_DEAD', started_at: T0 + 100, ongoing: true, duration_s: -1,
        severity: 'CRITICAL', outcome: 'ESTOP', mode: 'FOLLOW', state: 'RUN' }),
    R({ started_at: T0 }),
    R({ fault_type: 'ESP32_DISCONNECTED', started_at: T0 - 3000, duration_s: 185,
        mode: 'IDLE', state: 'IDLE', outcome: 'NONE' })))
  const chip = page.getByTestId('recent-fault')
  await expect(chip).toHaveText('直近の異常：速度指令経路 途絶 12:36:36')
  await chip.click()
  const win = page.getByTestId('fh-window')
  await expect(win).toBeVisible()
  const rows = win.getByTestId('fh-row')
  await expect(rows).toHaveCount(3)
  // 新しい順。継続中: 「継続中」＋経過、重大、非常停止。
  await expect(rows.nth(0).getByTestId('fh-type')).toHaveText('速度指令経路 途絶')
  await expect(rows.nth(0)).toHaveAttribute('data-ongoing', 'true')
  await expect(rows.nth(0).getByTestId('fh-dur')).toContainText('継続中')
  await expect(rows.nth(0).getByTestId('fh-sev')).toHaveText('重大')
  await expect(rows.nth(0).getByTestId('fh-outcome')).toHaveText('非常停止')
  // 解消済み: 時刻・種別・継続時間・モードと状態・回復・一時停止。
  await expect(rows.nth(1).getByTestId('fh-time')).toHaveText('12:34:56')
  await expect(rows.nth(1).getByTestId('fh-type')).toHaveText('LiDAR 途絶')
  await expect(rows.nth(1).getByTestId('fh-dur')).toHaveText('0.1 秒')
  await expect(rows.nth(1)).toHaveAttribute('data-ongoing', 'false')
  await expect(rows.nth(1).getByTestId('fh-sev')).toHaveText('回復')
  await expect(rows.nth(1).getByTestId('fh-outcome')).toHaveText('一時停止')
  await expect(rows.nth(2).getByTestId('fh-dur')).toHaveText('3 分 5 秒')
  await expect(rows.nth(2).getByTestId('fh-outcome')).toHaveText('停止なし')
  // ヘッダは窓より上: 非常停止が押せる。
  await expect(page.locator('#estopBtn')).toBeVisible()
  await page.getByTestId('fh-close').click()
  await expect(win).toHaveCount(0)
})

test('(c2) 継続中のフォルトがある間は、ヘッダは現在のフォルトが優先（直近の異常は出さない）', async ({ page }) => {
  await gotoWithState(page, { mode: 'IDLE', state: 'IDLE' })
  await setHistory(page, history(R()))
  await expect(page.getByTestId('recent-fault')).toBeVisible()
  await setTestFault(page, { active: true, fault_type: 'UI_DISCONNECTED', severity: 'RECOVERABLE' })
  await expect(page.getByTestId('recent-fault')).toHaveCount(0)
  await expect(page.locator('#faultTx')).toContainText('タブレット')
})

test('(e) S-50 の「フォルト履歴」タブに同じ一覧が出る', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'IDLE' })
  await setHistory(page, history(R(), R({ fault_type: 'ESP32_DISCONNECTED', started_at: T0 - 90, duration_s: 12 })))
  await page.getByTestId('s01-open-settings').click()
  await page.locator('#s50').waitFor()
  await page.getByTestId('s50-tab-faults').click()
  const rows = page.getByTestId('s50-faults').getByTestId('fh-row')
  await expect(rows).toHaveCount(2)
  await expect(rows.nth(1).getByTestId('fh-dur')).toHaveText('12 秒')
  // 後から開いたタブにも、最新の 1 通が届いている（TRANSIENT_LOCAL の再現）。
  await expect(rows.nth(0).getByTestId('fh-type')).toHaveText('LiDAR 途絶')
})

test('(f) 空のときは「フォルトの記録はありません」', async ({ page }) => {
  await gotoScreen(page, 'S01', { mode: 'IDLE' })
  await page.getByTestId('s01-open-settings').click()
  await page.locator('#s50').waitFor()
  await page.getByTestId('s50-tab-faults').click()
  await expect(page.getByTestId('fh-empty')).toHaveText('フォルトの記録はありません')
  // ヘッダの窓も同じ空表示（記録が 0 件ならチップ自体が出ない）。
  await expect(page.getByTestId('recent-fault')).toHaveCount(0)
})

test('ヘッダは 2 行を超えず、非常停止が押せる（長い種別名・狭い幅でも）', async ({ page }) => {
  for (const size of [{ width: 420, height: 840 }, { width: 768, height: 1024 }, { width: 1280, height: 800 }]) {
    await page.setViewportSize(size)
    await gotoWithState(page, { mode: 'FOLLOW', state: 'RUN', zone: 'IN' })
    const before = (await page.locator('#hdr').boundingBox()).height
    await setHistory(page, history(R({ fault_type: 'MUX_DEAD' })))
    await expect(page.getByTestId('recent-fault')).toBeVisible()
    const after = (await page.locator('#hdr').boundingBox()).height
    expect(after).toBe(before)
    await expect(page.locator('#estopBtn')).toBeVisible()
    const e = await page.locator('#estopBtn').boundingBox()
    expect(e.x + e.width).toBeLessThanOrEqual(size.width + 1)
  }
})
