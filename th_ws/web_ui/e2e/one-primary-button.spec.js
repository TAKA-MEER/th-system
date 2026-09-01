// DetailedDesign-webui.md §9-6 (U-17): the operation card never shows two
// "blue" (current-state) buttons at once. Driven through every mode's
// resume_state / run_state / and one "in-between" state from
// generated/attributes.json, via src/TestHarness.jsx (all 5 slots shown at
// once so this doesn't depend on which screen exists yet).
import { test, expect } from '@playwright/test'
import { readFileSync } from 'node:fs'
import { fileURLToPath } from 'node:url'
import { gotoWithState, setTestState } from './helpers.js'

const attributes = JSON.parse(
  readFileSync(fileURLToPath(new URL('../src/generated/attributes.json', import.meta.url)), 'utf8'),
)

function statesToTry(attrs) {
  const states = new Set(['NONE'])
  if (attrs.run_state) states.add(attrs.run_state)
  if (attrs.resume_state) states.add(attrs.resume_state)
  states.add(attrs.initial_state)
  return [...states]
}

test('operation card shows at most one primary (blue) button, across every mode/state', async ({ page }) => {
  await gotoWithState(page, { mode: 'IDLE', state: 'NONE' })
  await expect(page.locator('[data-testid="opcard-harness"] .opsgrid')).toBeVisible()

  for (const [mode, attrs] of Object.entries(attributes)) {
    for (const stateName of statesToTry(attrs)) {
      await setTestState(page, { mode, state: stateName })
      const onCount = await page.locator('[data-testid="opcard-harness"] .opsgrid .btn.on').count()
      expect(onCount, `mode=${mode} state=${stateName} had ${onCount} primary buttons`).toBeLessThanOrEqual(1)
    }
  }
})
