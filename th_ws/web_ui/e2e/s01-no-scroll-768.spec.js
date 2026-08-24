// DetailedDesign-wp1.md WP-UI-02 §7 (M-3, DetailedDesign-webui.md §2.4 /
// §9-2 taken early for S-01 only, per this packet's own scope note --
// "画面が揃うまで実行できない。WP-UI-08の完了条件に置く" for the other 14
// screens): at short edge >= 768px, S-01 must not require scrolling. The
// mockup measurement that produced M-3 (DetailedDesign-webui.md §8.3) found
// that expanding the unsaved list *into the card* pushes the screen 41px
// past the viewport at this size -- this spec seeds a non-empty unsaved
// list specifically to catch a regression back to that layout.
import { test, expect } from '@playwright/test'
import { gotoScreen } from './helpers.js'

const SIZES = [
  { width: 768, height: 1024 }, // portrait, short edge 768
  { width: 1024, height: 768 }, // landscape, short edge 768
]

for (const size of SIZES) {
  test(`S-01 does not scroll at ${size.width}x${size.height}`, async ({ page }) => {
    await page.setViewportSize(size)
    await gotoScreen(page, 'S01', {
      mode: 'IDLE',
      tracker_enabled: true,
      unsaved: ['venue_map', 'route', 'calib', 'map_patch'],
    })
    // M-3: the unsaved count must show as a badge, not an expanded list
    // (that expansion is what broke the 768px budget in the mockup).
    await expect(page.locator('.card', { hasText: '運用の終了' })).toContainText('4 件')

    const scrollable = await page.evaluate(
      () => document.body.scrollHeight > window.innerHeight,
    )
    expect(scrollable, 'S-01 must fit without scrolling at short edge 768px (M-3)').toBe(false)
  })
}
