// WS-9K E-2: S-13 の「保存しました」は route_recorder が実際にファイルを書いた
// （RouteStatus.saved === true）ときだけ表示する。FSM の state が SAVED でも
// 保存に失敗して saved が false のままなら「保存されていません」を出す。
//
// RouteStatus.saved は impl 側で msg に追加した新しいフィールド。実 msg の
// ビルドに依存しないよう、__thTestRouteStatus のシードで saved を明示的に
// 与えてテストする。
import { test, expect } from '@playwright/test'
import { gotoScreenWithRoutePreview } from './helpers.js'

async function openS13Teach(page, status) {
  await gotoScreenWithRoutePreview(
    page, 'S13',
    { mode: 'TEACH_MANUAL', state: 'SAVED' },
    { routes: [], preview: [], status },
  )
  await page.locator('#s13').waitFor()
  await page.getByRole('tab', { name: '教示' }).click()
}

test('S-13 saved=true なら「保存しました」を出す', async ({ page }) => {
  await openS13Teach(page, { state: 'SAVED', saved: true })
  await expect(page.getByTestId('s13-saved')).toBeVisible()
  await expect(page.getByTestId('s13-save-failed')).toHaveCount(0)
})

test('S-13 FSM が SAVED でも saved=false なら「保存されていません」を出し、「保存しました」は出さない', async ({ page }) => {
  // FSM は SAVED になったのに保存に失敗（saved が false のまま）した状態。
  await openS13Teach(page, { state: 'SAVED', saved: false })
  await expect(page.getByTestId('s13-save-failed')).toBeVisible()
  await expect(page.getByTestId('s13-saved')).toHaveCount(0)
})

test('S-13 saved が未定義（impl 未反映）のときは「保存しました」を出さない', async ({ page }) => {
  // impl の RouteStatus.saved がまだ入っていない時点では saved は undefined。
  await openS13Teach(page, { state: 'SAVED' })
  await expect(page.getByTestId('s13-save-failed')).toBeVisible()
  await expect(page.getByTestId('s13-saved')).toHaveCount(0)
})
