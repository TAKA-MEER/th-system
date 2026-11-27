// e2e/w0a-toggle-frame.spec.js — 停止│走行トグル・自律枠の縛り（§3.3.2/§3.3.5）。
import { test, expect } from '@playwright/test'
import { gotoWithState, setTestState } from './helpers.js'

test('自律走行中だけ緑の枠が出る（§3.3.5）', async ({ page }) => {
  await gotoWithState(page, { mode: 'FOLLOW', state: 'RUN' })
  await expect(page.getByTestId('autonomous-frame')).toBeVisible()
  await setTestState(page, { state: 'PAUSE' })
  await expect(page.getByTestId('autonomous-frame')).toHaveCount(0)
  await setTestState(page, { mode: 'MANUAL', state: 'RUN' })
  await expect(page.getByTestId('autonomous-frame')).toHaveCount(0)
})
