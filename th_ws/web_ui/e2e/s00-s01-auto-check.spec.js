// 起動時の自動点検の警告（Spec-ops.md §2.6・Spec-webui.md §3.1/S-00・§3.2/S-01）。
// S-00 の「総合」の下に自動判定を出し、S-01 の「ネットワーク接続確認」の
// 要約にも NG / WARN のときに同じ警告を出す。警告だけで、運用に入る・
// 進むボタンの活性は変えない（C-r4）。開発モードで opcheck を外すと警告を
// 出さず、消していることが分かる表示にする。
import { test, expect } from '@playwright/test'
import { gotoScreenWithAutoCheck, setTestAutoCheck } from './helpers.js'

const NG_AUTO = {
  overall: 'NG',
  suppressed: false,
  items: {
    ESTOP: { result: 'OK', reason: '' },
    IMU: { result: 'OK', reason: '' },
    LIDAR: { result: 'NG', reason: 'no_data' },
  },
}

const OK_AUTO = {
  overall: 'OK',
  suppressed: false,
  items: {
    ESTOP: { result: 'OK', reason: '' },
    IMU: { result: 'OK', reason: '' },
    LIDAR: { result: 'OK', reason: '' },
  },
}

const SUPPRESSED_AUTO = { ...NG_AUTO, suppressed: true }

test('S-00: NG のとき自動判定の警告が出て、項目別の理由も見える', async ({ page }) => {
  await gotoScreenWithAutoCheck(page, 'S00', { mode: 'IDLE' }, NG_AUTO)
  await expect(page.locator('[data-testid="s00-auto-check"]')).toBeVisible()
  await expect(page.locator('[data-testid="s00-auto-overall"]')).toContainText('要確認')
  // 1b-6 SG-C4: 項目別の結果は日本語で出す（OK/NG/WARN の英字を出さない）。
  await expect(page.locator('[data-testid="s00-auto-LIDAR"]')).toContainText('異常')
  await expect(page.locator('[data-testid="s00-auto-LIDAR"]')).not.toContainText('NG')
  await expect(page.locator('[data-testid="s00-auto-LIDAR"]')).toContainText('no_data')
  await expect(page.locator('[data-testid="s00-auto-ESTOP"]')).toContainText('正常')
})

test('S-00: NG でもメインメニューへ進める（警告のみでゲートしない）', async ({ page }) => {
  await gotoScreenWithAutoCheck(page, 'S00', { mode: 'IDLE' }, NG_AUTO)
  const advance = page.locator('[data-testid="s00-advance"]')
  await expect(advance).toBeVisible()
  await advance.click()
  await expect(page.locator('#screenName')).toHaveText('メインメニュー')
})

test('S-00: suppressed のときは「消している」表示で、項目表は出さない', async ({ page }) => {
  await gotoScreenWithAutoCheck(page, 'S00', { mode: 'IDLE' }, SUPPRESSED_AUTO)
  await expect(page.locator('[data-testid="s00-auto-overall"]')).toContainText('消しています')
  await expect(page.locator('[data-testid="s00-auto-LIDAR"]')).toHaveCount(0)
})

test('S-00: 未受信のときは「判定中」で OK と取り違えない', async ({ page }) => {
  await gotoScreenWithAutoCheck(page, 'S00', { mode: 'IDLE' }, null)
  await expect(page.locator('[data-testid="s00-auto-overall"]')).toContainText('まだ届いていません')
})

test('S-01: NG のとき要約に警告が出るが、走行ボタンは活性のまま', async ({ page }) => {
  await gotoScreenWithAutoCheck(page, 'S01', { mode: 'IDLE', tracker_enabled: true }, NG_AUTO)
  await expect(page.locator('[data-testid="s01-net-summary"]')).toBeVisible()
  await expect(page.locator('[data-testid="s01-net-warn"]')).toBeVisible()
  // 警告だけ。メニューボタンは非活性にしない（変異チェック⑤の標的）。
  const buttons = page.locator('.btnrow .btn')
  expect(await buttons.count()).toBeGreaterThan(0)
  for (let i = 0; i < await buttons.count(); i++) {
    await expect(buttons.nth(i)).toBeEnabled()
  }
})

test('S-01: suppressed のときは警告なしで「消している」表示', async ({ page }) => {
  await gotoScreenWithAutoCheck(page, 'S01', { mode: 'IDLE', tracker_enabled: true }, SUPPRESSED_AUTO)
  await expect(page.locator('[data-testid="s01-net-warn"]')).toHaveCount(0)
  await expect(page.locator('[data-testid="s01-net-summary"]')).toContainText('消しています')
})

test('S-01: OK のときは警告が出ない', async ({ page }) => {
  await gotoScreenWithAutoCheck(page, 'S01', { mode: 'IDLE', tracker_enabled: true }, OK_AUTO)
  await expect(page.locator('[data-testid="s01-net-warn"]')).toHaveCount(0)
  await expect(page.locator('[data-testid="s01-net-summary"]')).toContainText('正常')
})

test('S-01: 「詳細を見る」で S-00 相当へ行ける', async ({ page }) => {
  await gotoScreenWithAutoCheck(page, 'S01', { mode: 'IDLE', tracker_enabled: true }, NG_AUTO)
  await page.locator('[data-testid="s01-net-open"]').click()
  await expect(page.locator('#screenName')).toHaveText('接続確認')
  await expect(page.locator('[data-testid="s00-auto-check"]')).toBeVisible()
})

test('S-01: 後から NG になっても警告が出る（live 更新）', async ({ page }) => {
  await gotoScreenWithAutoCheck(page, 'S01', { mode: 'IDLE', tracker_enabled: true }, OK_AUTO)
  await expect(page.locator('[data-testid="s01-net-warn"]')).toHaveCount(0)
  await setTestAutoCheck(page, NG_AUTO)
  await expect(page.locator('[data-testid="s01-net-warn"]')).toBeVisible()
})
