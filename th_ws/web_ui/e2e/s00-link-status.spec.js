// S-00 の機器のつながりの図（Spec-webui.md §3.1。2026-11-27 作り直し）。
// 表をやめ図にした。/system/link_status の項目別の状態が、図の各機器に
// 別々に・実データ（受信の間隔）で出ることを縛る。
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

test('INIT 中でも、つながっている機器とつながらない機器が図に分かれて出る', async ({ page }) => {
  await gotoS00(page, { mode: 'INIT' }, PARTIAL)
  await expect(page.getByTestId('s00-node-esp')).toHaveAttribute('data-m', 'ok')
  await expect(page.getByTestId('s00-node-lidar')).toHaveAttribute('data-m', 'ng')
  await expect(page.getByTestId('s00-node-pc')).toHaveAttribute('data-m', 'ng')
  // LiDAR・PC の実データ（受信の間隔・不足ノード）が図に出る。
  await expect(page.getByTestId('s00-node-lidar')).toContainText('一度も受信していない')
  await expect(page.getByTestId('s00-node-pc')).toContainText('obstacle_limiter')
  // つながっていない機器への線は灰の点線。
  await expect(page.getByTestId('s00-edge-lidar')).toHaveAttribute('data-dead', 'true')
  await expect(page.getByTestId('s00-edge-esp')).toHaveAttribute('data-dead', 'false')
  // 図の下にだめな機器の対処の 1 行が出る。
  await expect(page.getByTestId('s00-fix')).toContainText('LiDAR の電源とケーブルを確かめる')
  await expect(page.locator('[data-testid="s00-advance"]')).toHaveClass(/dis/)
})

test('届くたびに図の機器が更新される', async ({ page }) => {
  await gotoS00(page, { mode: 'INIT' }, PARTIAL)
  await page.evaluate((v) => window.__thSetTestLinkStatus(v), {
    ...PARTIAL,
    items: { ...PARTIAL.items, lidar: { ok: true, age_ms: 100, points: 720, expected_points: 720 } },
  })
  await expect(page.getByTestId('s00-node-lidar')).toHaveAttribute('data-m', 'ok')
})

test('項目別の状態が届いていない間は、IDLE でも緑にしない（—）', async ({ page }) => {
  await gotoS00(page, { mode: 'IDLE' }, null)
  await expect(page.getByTestId('s00-node-esp')).toHaveAttribute('data-m', 'none')
  await expect(page.getByTestId('s00-node-lidar')).toHaveAttribute('data-m', 'none')
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
  await expect(page.getByTestId('s00-node-esp')).toHaveAttribute('data-m', 'ng')
  await expect(page.getByTestId('s00-node-esp')).toContainText('開発モードで無視中')
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

// 1b-6 SG-C4: 項目別の状態が届いているのに揃わないときは、押せない理由に
// 「必須機器が繋がっていません」が出る。
test('揃わないときは押せない理由に「必須機器が繋がっていません」が出る', async ({ page }) => {
  await gotoS00(page, { mode: 'INIT' }, PARTIAL)
  await page.locator('[data-testid="s00-advance"]').click({ force: true })
  await expect(page.getByTestId('s00-advance-reason')).toContainText('必須機器が繋がっていません')
})

test('揃わないときは押せない理由に「確認中」が出る', async ({ page }) => {
  await gotoS00(page, { mode: 'INIT' }, null)
  await page.locator('[data-testid="s00-advance"]').click({ force: true })
  await expect(page.getByTestId('s00-advance-reason')).toContainText('確認中')
})

// 1b-6 SG-B7: 再起動中（restart.attempt >= 2）は「制御系を再起動しています（n 回目）」。
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

test('全項目が疎通しているのに INIT のとき（link_ok の直前）は進めない', async ({ page }) => {
  const ALL_OK = JSON.parse(JSON.stringify(PARTIAL))
  ALL_OK.items.lidar.ok = true
  ALL_OK.items.nodes.ok = true
  await gotoS00(page, { mode: 'INIT' }, ALL_OK)
  await expect(page.getByTestId('s00-node-esp')).toHaveAttribute('data-m', 'ok')
  await expect(page.locator('[data-testid="s00-advance"]')).toHaveClass(/dis/)
})

test('機器の名前は §1.2 の 15 px 以上で読める（描画後の大きさ）', async ({ page }) => {
  await page.setViewportSize({ width: 1280, height: 720 })
  await gotoS00(page, { mode: 'INIT' }, PARTIAL)
  // SVG は viewBox を縮小するので、描画後の高さ（CSS px）で縛る。
  const heights = await page.evaluate(() => {
    const svg = document.querySelector('svg')
    return [...svg.querySelectorAll('text')]
      .filter((t) => t.textContent.trim())
      .map((t) => Math.round(t.getBoundingClientRect().height))
  })
  expect(heights.length).toBeGreaterThan(0)
  for (const h of heights) {
    expect(h, `図の文字が小さい: ${h}px`).toBeGreaterThanOrEqual(15)
  }
})

test('カメラは任意で、だめでも図の線は灰にしない', async ({ page }) => {
  await gotoS00(page, { mode: 'INIT' }, PARTIAL)
  await expect(page.getByTestId('s00-edge-cam')).toHaveAttribute('data-dead', 'false')
  // カメラは機体から値が届かないので「—」を出す（値は作らない）。
  await expect(page.getByTestId('s00-node-cam')).toHaveAttribute('data-m', 'none')
})
