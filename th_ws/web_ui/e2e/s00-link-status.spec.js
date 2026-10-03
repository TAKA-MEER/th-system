// S-00 の機器別の行（Spec-webui.md §3.1。2026-10-04）。以前は 4 行とも全体の合否
// （mode が INIT か）を出すだけで、どの機器が繋がっていないか分からなかった。
// /system/link_status の項目別の状態が、各行に別々に出ることを縛る。
import { test, expect } from '@playwright/test'

async function gotoS00(page, state, link) {
  await page.addInitScript(({ s, l }) => {
    window.__thTestState = s
    window.__thTestScreen = 'S00'
    if (l) window.__thTestLinkStatus = l
  }, { s: state, l: link })
  await page.goto('/')
}

const PARTIAL = {
  items: {
    esp32_feedback: { ok: true, age_ms: 80, ignored: false, excluded: false },
    esp32_loopback: { ok: true, age_ms: 90, ignored: false, excluded: false },
    lidar: { ok: false, age_ms: null, points: 0, expected_points: 720, ignored: false, excluded: false },
    nodes: { ok: false, missing: ['obstacle_limiter'], ignored: false, excluded: false },
  },
  timeout_ms: 3000,
  estop_hw: { seen: true, pressed: false },
}

test('INIT 中でも、繋がっている機器と繋がっていない機器が行ごとに分かれて出る', async ({ page }) => {
  await gotoS00(page, { mode: 'INIT' }, PARTIAL)
  await expect(page.getByTestId('s00-link-esp32_feedback')).toContainText('疎通')
  await expect(page.getByTestId('s00-link-esp32_feedback')).toHaveClass(/tone-ok/)
  await expect(page.getByTestId('s00-link-lidar')).toContainText('未接続')
  await expect(page.getByTestId('s00-link-lidar')).toHaveClass(/tone-ng/)
  await expect(page.getByTestId('s00-link-detail-lidar')).toContainText('一度も受信していない')
  await expect(page.getByTestId('s00-link-detail-nodes')).toContainText('obstacle_limiter')
  await expect(page.locator('[data-testid="s00-advance"]')).toHaveCount(0)
})

test('届くたびに行が更新される', async ({ page }) => {
  await gotoS00(page, { mode: 'INIT' }, PARTIAL)
  await page.evaluate((v) => window.__thSetTestLinkStatus(v), {
    ...PARTIAL,
    items: { ...PARTIAL.items, lidar: { ok: true, age_ms: 100, points: 720, expected_points: 720 } },
  })
  await expect(page.getByTestId('s00-link-lidar')).toContainText('疎通')
})

test('項目別の状態が届いていない間は、IDLE でも緑にしない（確認中）', async ({ page }) => {
  await gotoS00(page, { mode: 'IDLE' }, null)
  await expect(page.getByTestId('s00-link-esp32_feedback')).toContainText('確認中')
  await expect(page.getByTestId('s00-link-esp32_feedback')).toHaveClass(/tone-warn/)
})

test('開発モードで外していても本当の状態を出し、無視中と添える', async ({ page }) => {
  await gotoS00(page, { mode: 'IDLE' }, {
    items: {
      esp32_feedback: { ok: false, age_ms: null, ignored: true },
      esp32_loopback: { ok: false, age_ms: null, ignored: true },
      lidar: { ok: false, age_ms: null, ignored: true },
      nodes: { ok: true, missing: [], ignored: true },
    },
  })
  await expect(page.getByTestId('s00-link-esp32_feedback')).toContainText('未接続')
  await expect(page.getByTestId('s00-link-detail-esp32_feedback')).toContainText('開発モードで無視中')
})

test('S-01 の要約に、繋がっていない機器の名前が出る', async ({ page }) => {
  await page.addInitScript(({ l }) => {
    window.__thTestState = { mode: 'IDLE' }
    window.__thTestScreen = 'S01'
    window.__thTestLinkStatus = l
  }, { l: PARTIAL })
  await page.goto('/')
  await expect(page.getByTestId('s01-net-link-down')).toContainText('RaspberryPi4（LiDAR）')
  await expect(page.getByTestId('s01-net-link-down')).toContainText('PC（ノード起動）')
  await expect(page.getByTestId('s01-net-link-down')).not.toContainText('ホイールフィードバック')
})
