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

// 1b-6 SG-C4: 項目別の状態が届いているのに揃わないときは、総合欄に
// 「必須機器が繋がっていません」（従来は「確認中」のままだった）。
test('総合欄: 項目別の状態が届いて揃わなければ「必須機器が繋がっていません」', async ({ page }) => {
  await gotoS00(page, { mode: 'INIT' }, PARTIAL)
  await expect(page.getByTestId('s00-overall')).toContainText('必須機器が繋がっていません')
})

test('総合欄: 項目別の状態が届いていなければ「確認中」のまま', async ({ page }) => {
  await gotoS00(page, { mode: 'INIT' }, null)
  await expect(page.getByTestId('s00-overall')).toContainText('確認中')
})

// 1b-6 SG-B7: 再起動中（restart.attempt >= 2）は「制御系を再起動しています（n 回目）」。
// Spec-ops.md §2.4・Spec-webui.md §3.1 の文言どおり。
test('再起動中は「制御系を再起動しています（n 回目）」が出る', async ({ page }) => {
  await gotoS00(page, { mode: 'INIT' }, { ...PARTIAL, restart: { attempt: 2, calls: 1 } })
  await expect(page.getByTestId('s00-restart')).toContainText('制御系を再起動しています（2 回目）')
})

test('初回起動（attempt 1）では再起動表示は出ない', async ({ page }) => {
  await gotoS00(page, { mode: 'INIT' }, { ...PARTIAL, restart: { attempt: 1, calls: 0 } })
  await expect(page.getByTestId('s00-restart')).toHaveCount(0)
})

test('再起動表示は、運用に入れる（IDLE）ようになったら消える', async ({ page }) => {
  await gotoS00(page, { mode: 'IDLE' }, { ...PARTIAL, restart: { attempt: 3, calls: 0 } })
  await expect(page.getByTestId('s00-restart')).toHaveCount(0)
})

test('全項目が疎通しているのに INIT のとき（link_ok の直前）は「確認中」のまま', async ({ page }) => {
  const ALL_OK = JSON.parse(JSON.stringify(PARTIAL))
  ALL_OK.items.lidar.ok = true
  ALL_OK.items.nodes.ok = true
  await gotoS00(page, { mode: 'INIT' }, ALL_OK)
  await expect(page.getByTestId('s00-overall')).toContainText('確認中')
  await expect(page.getByTestId('s00-overall')).not.toContainText('繋がっていません')
})

test('タブレット行と AP 行に状態が出る（接続中は ✓）', async ({ page }) => {
  await gotoS00(page, { mode: 'INIT' }, PARTIAL)
  await expect(page.getByTestId('s00-tablet')).toContainText('接続している')
  await expect(page.getByTestId('s00-ap')).toContainText('通信できている')
})
