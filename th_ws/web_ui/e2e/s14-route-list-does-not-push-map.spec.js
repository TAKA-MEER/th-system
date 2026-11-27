// P5 / demo-teach-replay: S-14 の経路一覧が長くても地図（route-preview）を
// 下へ押し出さないこと。一覧は .lst-scroll で枠内スクロールになり、上限に
// 達したあとは経路が何本あっても地図の位置は動かない。
import { test, expect } from '@playwright/test'
import { gotoScreenWithRouteCatalog } from './helpers.js'

const makeRoutes = (n) =>
  Array.from({ length: n }, (_, i) => ({
    id: `route_r${i}`,
    name: `経路 ${i}`,
    length_m: 3 + i * 0.1,
    point_count: 10 + i,
    start_yaw: 0,
    recorded_at: 0,
  }))

async function mapTop(page) {
  return page.locator('[data-testid="s14-map-wrap"]').evaluate(
    (el) => el.getBoundingClientRect().top,
  )
}

test('S-14 入口のカードは枠内スクロールで、地図を押し出さない', async ({ page }) => {
  await gotoScreenWithRouteCatalog(page, 'S14', { mode: 'REPLAY', state: 'ROUTE_SEL' }, makeRoutes(30))
  await page.locator('#s14').waitFor()

  // 一覧は中央のカードの中にあり、カードが枠内スクロールで頭打ちになる。
  const box = page.getByTestId('s14-select-card')
  await expect(box).toBeVisible()
  const { clientH, scrollH } = await box.evaluate((el) => ({
    clientH: el.clientHeight,
    scrollH: el.scrollHeight,
  }))
  expect(scrollH, '経路一覧が枠内スクロールになっていない').toBeGreaterThan(clientH)
})

test('S-14 adding more routes past the cap does not move the map down', async ({ page }) => {
  await gotoScreenWithRouteCatalog(page, 'S14', { mode: 'REPLAY', state: 'ROUTE_SEL' }, makeRoutes(10))
  await page.locator('#s14').waitFor()
  const top10 = await mapTop(page)

  await gotoScreenWithRouteCatalog(page, 'S14', { mode: 'REPLAY', state: 'ROUTE_SEL' }, makeRoutes(40))
  await page.locator('#s14').waitFor()
  const top40 = await mapTop(page)

  // 10 本でも 40 本でも一覧は既に上限に達しているので、地図の縦位置は動かない。
  expect(Math.abs(top40 - top10), '経路を増やすと地図が下へ押し下げられている').toBeLessThan(8)
})
